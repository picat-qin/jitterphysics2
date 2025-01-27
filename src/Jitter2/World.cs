/*
 * Copyright (c) Thorben Linneweber and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.DataStructures;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.UnmanagedMemory;

namespace Jitter2;

/// <summary>
/// ���� <br></br><br></br>
/// ��ʾһ��ģ�⻷�����û������к͹�������ģ������״̬��<br></br><br></br>
/// Represents a simulation environment that holds and manages the state of all simulation objects.
/// </summary>
public sealed partial class World : IDisposable
{
    /// <summary>
    /// �߳�ģ������
    /// </summary>
    public enum ThreadModelType
    {
        /// <summary>
        /// ���ڵ�
        /// </summary>
        Regular,
        /// <summary>
        /// ������
        /// </summary>
        Persistent
    }

    /// <summary>
    /// �ṩ�Է��й��ڴ��ж���ķ��ʡ��˲������ܲ���ȫ��<br></br><br></br>
    /// Provides access to objects in unmanaged memory. This operation is potentially unsafe.
    /// </summary>
    public readonly struct SpanData
    {
        private readonly World world;

        public SpanData(World world)
        {
            this.world = world;
        }

        /// <summary>
        /// �������ֽ�Ϊ��λ����ķ��й��ڴ����������<br></br><br></br>
        /// Returns the total amount of unmanaged memory allocated in bytes.
        /// </summary>
        public long TotalBytesAllocated =>
            world.memRigidBodies.TotalBytesAllocated +
            world.memContacts.TotalBytesAllocated +
            world.memConstraints.TotalBytesAllocated +
            world.memSmallConstraints.TotalBytesAllocated;

        /// <summary>
        /// ��ĸ��弯�ڴ��
        /// </summary>
        public Span<RigidBodyData> ActiveRigidBodies => world.memRigidBodies.Active;
        /// <summary>
        /// �ǻ�ĸ��弯�ڴ��
        /// </summary>
        public Span<RigidBodyData> InactiveRigidBodies => world.memRigidBodies.Inactive;
        /// <summary>
        /// ���и��弯�ڴ��
        /// </summary>
        public Span<RigidBodyData> RigidBodies => world.memRigidBodies.Elements;

        /// <summary>
        /// ��ĽӴ����ڴ��
        /// </summary>
        public Span<ContactData> ActiveContacts => world.memContacts.Active;
        /// <summary>
        /// �ǻ�ĽӴ����ڴ��
        /// </summary>
        public Span<ContactData> InactiveContacts => world.memContacts.Inactive;
        /// <summary>
        /// ���нӴ����ڴ��
        /// </summary>
        public Span<ContactData> Contacts => world.memContacts.Elements;

        /// <summary>
        /// ���Լ�����ڴ��
        /// </summary>
        public Span<ConstraintData> ActiveConstraints => world.memConstraints.Active;
        /// <summary>
        /// �ǻ��Լ�����ڴ��
        /// </summary>
        public Span<ConstraintData> InactiveConstraints => world.memConstraints.Inactive;
        /// <summary>
        /// ����Լ�����ڴ��
        /// </summary>
        public Span<ConstraintData> Constraints => world.memConstraints.Elements;

        /// <summary>
        /// ���С��Լ�����ڴ��
        /// </summary>
        public Span<SmallConstraintData> ActiveSmallConstraints => world.memSmallConstraints.Active;
        /// <summary>
        /// �ǻ��С��Լ�����ڴ��
        /// </summary>
        public Span<SmallConstraintData> InactiveSmallConstraints => world.memSmallConstraints.Inactive;
        /// <summary>
        /// ����С��Լ�����ڴ��
        /// </summary>
        public Span<SmallConstraintData> SmallConstraints => world.memSmallConstraints.Elements;
    }

    private readonly PartitionedBuffer<ContactData> memContacts;
    private readonly PartitionedBuffer<RigidBodyData> memRigidBodies;
    private readonly PartitionedBuffer<ConstraintData> memConstraints;
    private readonly PartitionedBuffer<SmallConstraintData> memSmallConstraints;

    /// <summary>
    /// ���粽
    /// </summary>
    /// <param name="dt"></param>
    public delegate void WorldStep(Real dt);

    // Post- and Pre-step
    /// <summary>
    /// Ԥ�Ȳ�
    /// </summary>
    public event WorldStep? PreStep;
    /// <summary>
    /// ����
    /// </summary>
    public event WorldStep? PostStep;

    /// <summary>
    /// �������פ���ڷ��й��ڴ��еĶ���<br></br><br></br>
    /// �˲������ܲ���ȫ��������ʹ����Ӧ�ı���������������ա�<br></br><br></br>
    /// Grants access to objects residing in unmanaged memory. This operation can be potentially unsafe. Utilize
    /// the corresponding native properties where possible to mitigate risk.
    /// </summary>
    public SpanData RawData => new(this);

    private readonly ConcurrentDictionary<ArbiterKey, Arbiter> arbiters = new();

    private readonly PartitionedSet<Island> islands = new();
    private readonly PartitionedSet<RigidBody> bodies = new();

    private static ulong _idCounter;

    /// <summary>
    /// ����һ��Ψһ��ID��<br></br><br></br>
    /// Generates a unique ID.
    /// </summary>
    public static ulong RequestId()
    {
        return Interlocked.Increment(ref _idCounter);
    }

    /// <summary>
    /// ����һϵ��Ψһ��ID��<br></br><br></br>
    /// Generates a range of unique IDs.
    /// </summary>
    /// <param name="count"
    /// >Ҫ���ɵ�ID������<br></br><br></br>
    /// The number of IDs to generate.</param>
    /// <returns>
    /// �������ɷ�Χ����С���������ID��Ԫ�顣�Ͻ���������ġ�<br></br><br></br>
    /// A tuple containing the minimum and maximum request IDs in the generated range. The upper
    /// bound is exclusive.</returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// ������С��1ʱ�׳���<br></br><br></br>
    /// Thrown when count is less than 1.</exception>
    public static (ulong min, ulong max) RequestId(int count)
    {
        if (count < 1) throw new ArgumentOutOfRangeException(nameof(count), "Count must be greater zero.");
        ulong count64 = (ulong)count;
        ulong max = Interlocked.Add(ref _idCounter, count64) + 1;
        return (max - count64, max);
    }

    /// <summary>
    /// �������������õ��߳�ģ�͡�<br></br><br></br>
    /// <see cref="ThreadModelType.Persistent"/> ģ��ʹ�����̳߳�����Ծ��<br></br>
    /// ��ʹ�� <see cref="World.Step(Real, bool)"/> ��������ʱҲ����ˣ�<br></br>
    /// ����ܻ����ĸ����CPU���ڣ�������Ӱ����������������Ⱦ�������ܡ�<br></br>
    /// ���ǣ���ȷ���߳�����һ�ε��� <see cref="World.Step(Real, bool)"/> ʱ���֡���ů����<br></br><br></br>
    /// �෴��<see cref="ThreadModelType.Regular"/> ģ���������߳��ó����е���������<br></br><br></br>
    /// Defines the two available thread models. The <see cref="ThreadModelType.Persistent"/> model keeps the worker
    /// threads active continuously, even when the <see cref="World.Step(Real, bool)"/> is not in operation, which might
    /// consume more CPU cycles and possibly affect the performance of other operations such as rendering. However, it ensures that the threads
    /// remain 'warm' for the next invocation of <see cref="World.Step(Real, bool)"/>. Conversely, the <see cref="ThreadModelType.Regular"/> model allows
    /// the worker threads to yield and undertake other tasks.
    /// </summary>
    public ThreadModelType ThreadModel { get; set; } = ThreadModelType.Regular;

    /// <summary>
    /// ��������ϵ�������ײ���졣<br></br><br></br>
    /// All collision islands in this world.
    /// </summary>
    public ReadOnlyPartitionedSet<Island> Islands => new(islands);

    /// <summary>
    /// ��������ϵ����и��塣<br></br><br></br>
    /// All rigid bodies in this world.
    /// </summary>
    public ReadOnlyPartitionedSet<RigidBody> RigidBodies => new ReadOnlyPartitionedSet<RigidBody>(bodies);

    /// <summary>
    /// ��̬�� <br></br><br></br>
    /// ���� <see cref="DynamicTree"/> ʵ����ʵ��ֻ���� Jitter �޸ġ�<br></br><br></br>
    /// Access to the <see cref="DynamicTree"/> instance. The instance
    /// should only be modified by Jitter.
    /// </summary>
    public DynamicTree DynamicTree { get; }

    /// <summary>
    /// �̶����� <br></br><br></br>
    /// һ���̶����壬�̶������硣������������Լ����<br></br><br></br>
    /// A fixed body, pinned to the world. Can be used to create constraints with.
    /// </summary>
    public RigidBody NullBody { get; }

    /// <summary>
    /// ����ͣ�û��� <br></br><br></br>
    /// ָ���Ƿ�����Jitter��ͣ�û��ơ��������Ϊfalse���򲻻ἤ��ǻ����<br></br><br></br>
    /// Specifies whether the deactivation mechanism of Jitter is enabled.
    /// Does not activate inactive objects if set to false.
    /// </summary>
    public bool AllowDeactivation { get; set; } = true;

    /// <summary>
    /// ÿ���Ӳ���ĵ�����������������ɳڣ�������� <see cref="SubstepCount"/>����<br></br><br></br>
    /// Number of iterations (solver and relaxation) per substep (see <see cref="SubstepCount"/>).
    /// </summary>
    /// <remarks>
    /// Ĭ��ֵ�����������6���ɳڶȣ�4��<br></br><br></br>
    /// Default value: (solver: 6, relaxation: 4)</remarks>
    /// <value></value>
    public (int solver, int relaxation) SolverIterations
    {
        get => (solverIterations, velocityRelaxations);
        set
        {
            if (value.solver < 1)
            {
                throw new ArgumentException("Solver iterations can not be smaller than one.",
                    nameof(SolverIterations));
            }

            if (value.relaxation < 0)
            {
                throw new ArgumentException("Relaxation iterations can not be smaller than zero.",
                    nameof(SolverIterations));
            }

            solverIterations = value.solver;
            velocityRelaxations = value.relaxation;
        }
    }

    /// <summary>
    /// �Ӳ����� <br></br><br></br>
    /// ÿ�ε��� <see cref="World.Step(Real, bool)"/> ʱ���Ӳ���������<br></br>
    /// ������Ϊ 1 ʱ���Ӳ��������á�<br></br><br></br>
    /// The number of substeps for each call to <see cref="World.Step(Real, bool)"/>.
    /// Substepping is deactivated when set to one.
    /// </summary>
    public int SubstepCount
    {
        get => substeps;
        set
        {
            if (value < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "The number of substeps has to be larger than zero.");
            }

            substeps = value;
        }
    }

    private JVector gravity = new(0, -(Real)9.81, 0);

    /// <summary>
    /// Ĭ��������Ҳ����� <see cref="RigidBody.AffectedByGravity"/>��<br></br><br></br>
    /// Default gravity, see also <see cref="RigidBody.AffectedByGravity"/>.
    /// </summary>
    public JVector Gravity
    {
        get => gravity;
        set => gravity = value;
    }

    // Make this global since it is used by nearly every method called
    // in World.Step.
    private volatile int solverIterations = 6;
    private volatile int velocityRelaxations = 4;
    private volatile int substeps = 1;

    private Real substep_dt = (Real)(1.0 / 100.0);
    private Real step_dt = (Real)(1.0 / 100.0);

    /// <summary>
    /// ����һ������Ĭ��������<see cref="World"/>���ʵ����<br></br><br></br>
    /// ��ʹ��<see cref="Capacity.Default"/>�ж���������������Ӵ���Լ����С��Լ����Ĭ��ֵ��ʼ�����硣<br></br><br></br>
    /// Creates an instance of the <see cref="World"/> class with the default capacity.
    /// This initializes the world using default values for the number of bodies, contacts,
    /// constraints, and small constraints as defined in <see cref="Capacity.Default"/>.
    /// </summary>
    /// <seealso cref="World(Capacity)"/>
    public World() : this(Capacity.Default) { }

    /// <summary>
    /// ����һ�� World ���ʵ����<br></br><br></br>
    /// ���� Jitter ʹ�ò�ͬ���ڴ�ģ�ͣ���˱�����ǰָ�������������<br></br><br></br>
    /// Creates an instance of the World class. As Jitter utilizes a distinct memory model, it is necessary to specify
    /// the capacity of the world in advance.
    /// </summary>
    public World(Capacity capacity)
    {
        memRigidBodies = new PartitionedBuffer<RigidBodyData>(capacity.BodyCount);
        memContacts = new PartitionedBuffer<ContactData>(capacity.ContactCount);
        memConstraints = new PartitionedBuffer<ConstraintData>(capacity.ConstraintCount);
        memSmallConstraints = new PartitionedBuffer<SmallConstraintData>(capacity.SmallConstraintCount);

        NullBody = CreateRigidBody();
        NullBody.IsStatic = true;

        DynamicTree = new DynamicTree(DefaultDynamicTreeFilter);
    }

    /// <summary>
    /// <see cref="DynamicTree"/> ��Ĭ�Ϲ��˺�����<br></br><br></br>
    /// ������������� <see cref="RigidBodyShape"/> ���������ڲ�ͬ��ʵ�����򷵻�true��<br></br><br></br>
    /// Default filter function for the <see cref="DynamicTree"/>. 
    /// Returns true if both proxies are of type <see cref="RigidBodyShape"/>
    /// and belong to different instances.
    /// </summary>
    public static bool DefaultDynamicTreeFilter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        if (proxyA is RigidBodyShape rbsA && proxyB is RigidBodyShape rbsB)
        {
            return rbsA.RigidBody != rbsB.RigidBody;
        }

        return true;
    }

    /// <summary>
    /// ��ģ���������Ƴ�����ʵ�塣<br></br><br></br>
    /// Removes all entities from the simulation world.
    /// </summary>
    public void Clear()
    {
        // create a copy, since we are going to modify the list
        Stack<RigidBody> bodyStack = new(bodies);
        while (bodyStack.Count > 0) Remove(bodyStack.Pop());

        // Left-over shapes not associated with a rigid body.
        Stack<IDynamicTreeProxy> proxies = new(DynamicTree.Proxies);
        while (proxies.Count > 0) DynamicTree.RemoveProxy(proxies.Pop());
    }

    /// <summary>
    /// ���������Ƴ�ָ�������塣<br></br><br></br>
    /// �˲������Զ������κ���ص�Լ����<br></br><br></br>
    /// Removes the specified body from the world. This operation also automatically discards any associated contacts
    /// and constraints.
    /// </summary>
    public void Remove(RigidBody body)
    {
        // No need to copy the hashset content first. Removing while iterating does not invalidate
        // the enumerator any longer, see https://github.com/dotnet/runtime/pull/37180
        // This comes in very handy for us.

        foreach (var constraint in body.constraints)
        {
            Remove(constraint);
        }

        foreach (var shape in body.Shapes)
        {
            DynamicTree.RemoveProxy(shape);
            shape.RigidBody = null!;
        }

        foreach (var contact in body.contacts)
        {
            Remove(contact);
        }

        if (body == NullBody) return;

        memRigidBodies.Free(body.handle);

        // we must be our own island..
        Debug.Assert(body.island is { bodies.Count: 1 });

        body.handle = JHandle<RigidBodyData>.Zero;

        IslandHelper.BodyRemoved(islands, body);

        bodies.Remove(body);
    }

    /// <summary>
    /// ���������Ƴ�һ���ض���Լ����<br></br><br></br>
    /// ������ʱ����Լ�����뿼��ʹ��<see cref="Constraint.IsEnabled"/>���ԡ�<br></br><br></br>
    /// Removes a specific constraint from the world. For temporary deactivation of constraints, consider using the
    /// <see cref="Constraint.IsEnabled"/> property.
    /// </summary>
    /// <param name="constraint">Ҫɾ����Լ����<br></br><br></br> The constraint to be removed.</param>
    public void Remove(Constraint constraint)
    {
        ActivateBodyNextStep(constraint.Body1);
        ActivateBodyNextStep(constraint.Body2);

        IslandHelper.ConstraintRemoved(islands, constraint);

        if (constraint.IsSmallConstraint)
        {
            memSmallConstraints.Free(constraint.SmallHandle);
        }
        else
        {
            memConstraints.Free(constraint.Handle);
        }

        constraint.Handle = JHandle<ConstraintData>.Zero;
    }

    /// <summary>
    /// ���ض����ٲ��ߴ��������Ƴ���<br></br><br></br>
    /// Removes a particular arbiter from the world.
    /// </summary>
    public void Remove(Arbiter arbiter)
    {
        ActivateBodyNextStep(arbiter.Body1);
        ActivateBodyNextStep(arbiter.Body2);

        IslandHelper.ArbiterRemoved(islands, arbiter);
        arbiters.TryRemove(arbiter.Handle.Data.Key, out _);

        brokenArbiters.Remove(arbiter.Handle);
        memContacts.Free(arbiter.Handle);

        Arbiter.Pool.Push(arbiter);

        arbiter.Handle = JHandle<ContactData>.Zero;
    }

    internal void UpdateShape(RigidBodyShape shape)
    {
        shape.UpdateWorldBoundingBox();
        DynamicTree.Update(shape);
    }

    internal void ActivateBodyNextStep(RigidBody body)
    {
        body.sleepTime = 0;
        AddToActiveList(body.island);
    }

    internal void DeactivateBodyNextStep(RigidBody body)
    {
        body.sleepTime = Real.PositiveInfinity;
    }

    /// <summary>
    /// ����ָ�����͵�Լ����<br></br><br></br>
    /// �����󣬱���ʹ�� Constraint.Initialize ������ʼ��Լ����<br></br><br></br>
    /// Constructs a constraint of the specified type. After creation, it is mandatory to initialize the constraint using the Constraint.Initialize method.
    /// </summary>
    /// <typeparam name="T">Ҫ������Լ���ľ������͡�<br></br><br></br> The specific type of constraint to create.</typeparam>
    /// <param name="body1">Լ�����漰�ĵ� 1 �����塣<br></br><br></br> The first rigid body involved in the constraint.</param>
    /// <param name="body2">Լ�����漰�ĵ� 2 �����塣<br></br><br></br> The second rigid body involved in the constraint.</param>
    /// <returns>
    /// ָ��Լ�����͵���ʵ����<br></br><br></br>
    /// A new instance of the specified constraint type.
    /// </returns>
    public T CreateConstraint<T>(RigidBody body1, RigidBody body2) where T : Constraint, new()
    {
        T constraint = new();

        if (constraint.IsSmallConstraint)
        {
            constraint.Create(memSmallConstraints.Allocate(true, true), body1, body2);
        }
        else
        {
            constraint.Create(memConstraints.Allocate(true, true), body1, body2);
        }

        IslandHelper.ConstraintCreated(islands, constraint);

        AddToActiveList(body1.island);
        AddToActiveList(body2.island);

        return constraint;
    }

    private void AddToActiveList(Island island)
    {
        island.MarkedAsActive = true;
        island.NeedsUpdate = true;
        islands.MoveToActive(island);
    }

    /// <summary>
    /// ��ģ�������д��������һ���µĸ��塣<br></br><br></br>
    /// Creates and adds a new rigid body to the simulation world.
    /// </summary>
    /// <returns>
    /// <see cref="RigidBody"/>.����ʵ�� <br></br><br></br>
    /// A newly created instance of <see cref="RigidBody"/>.
    /// </returns>
    public RigidBody CreateRigidBody()
    {
        RigidBody body = new(memRigidBodies.Allocate(true, true), this);
        body.Data.IsActive = true;

        bodies.Add(body, true);

        IslandHelper.BodyAdded(islands, body);

        AddToActiveList(body.island);

        return body;
    }

    public void Dispose()
    {
        memContacts.Dispose();
        memRigidBodies.Dispose();
        memConstraints.Dispose();
        memSmallConstraints.Dispose();
    }
}