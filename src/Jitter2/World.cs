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
/// 世界 <br></br><br></br>
/// 表示一个模拟环境，该环境持有和管理所有模拟对象的状态。<br></br><br></br>
/// Represents a simulation environment that holds and manages the state of all simulation objects.
/// </summary>
public sealed partial class World : IDisposable
{
    /// <summary>
    /// 线程模型类型
    /// </summary>
    public enum ThreadModelType
    {
        /// <summary>
        /// 周期的
        /// </summary>
        Regular,
        /// <summary>
        /// 持续的
        /// </summary>
        Persistent
    }

    /// <summary>
    /// 提供对非托管内存中对象的访问。此操作可能不安全。<br></br><br></br>
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
        /// 返回以字节为单位分配的非托管内存的总数量。<br></br><br></br>
        /// Returns the total amount of unmanaged memory allocated in bytes.
        /// </summary>
        public long TotalBytesAllocated =>
            world.memRigidBodies.TotalBytesAllocated +
            world.memContacts.TotalBytesAllocated +
            world.memConstraints.TotalBytesAllocated +
            world.memSmallConstraints.TotalBytesAllocated;

        /// <summary>
        /// 活动的刚体集内存段
        /// </summary>
        public Span<RigidBodyData> ActiveRigidBodies => world.memRigidBodies.Active;
        /// <summary>
        /// 非活动的刚体集内存段
        /// </summary>
        public Span<RigidBodyData> InactiveRigidBodies => world.memRigidBodies.Inactive;
        /// <summary>
        /// 所有刚体集内存段
        /// </summary>
        public Span<RigidBodyData> RigidBodies => world.memRigidBodies.Elements;

        /// <summary>
        /// 活动的接触集内存段
        /// </summary>
        public Span<ContactData> ActiveContacts => world.memContacts.Active;
        /// <summary>
        /// 非活动的接触集内存段
        /// </summary>
        public Span<ContactData> InactiveContacts => world.memContacts.Inactive;
        /// <summary>
        /// 所有接触集内存段
        /// </summary>
        public Span<ContactData> Contacts => world.memContacts.Elements;

        /// <summary>
        /// 活动的约束集内存段
        /// </summary>
        public Span<ConstraintData> ActiveConstraints => world.memConstraints.Active;
        /// <summary>
        /// 非活动的约束集内存段
        /// </summary>
        public Span<ConstraintData> InactiveConstraints => world.memConstraints.Inactive;
        /// <summary>
        /// 所有约束集内存段
        /// </summary>
        public Span<ConstraintData> Constraints => world.memConstraints.Elements;

        /// <summary>
        /// 活动的小型约束集内存段
        /// </summary>
        public Span<SmallConstraintData> ActiveSmallConstraints => world.memSmallConstraints.Active;
        /// <summary>
        /// 非活动的小型约束集内存段
        /// </summary>
        public Span<SmallConstraintData> InactiveSmallConstraints => world.memSmallConstraints.Inactive;
        /// <summary>
        /// 所有小型约束集内存段
        /// </summary>
        public Span<SmallConstraintData> SmallConstraints => world.memSmallConstraints.Elements;
    }

    private readonly PartitionedBuffer<ContactData> memContacts;
    private readonly PartitionedBuffer<RigidBodyData> memRigidBodies;
    private readonly PartitionedBuffer<ConstraintData> memConstraints;
    private readonly PartitionedBuffer<SmallConstraintData> memSmallConstraints;

    /// <summary>
    /// 世界步
    /// </summary>
    /// <param name="dt"></param>
    public delegate void WorldStep(Real dt);

    // Post- and Pre-step
    /// <summary>
    /// 预先步
    /// </summary>
    public event WorldStep? PreStep;
    /// <summary>
    /// 后处理步
    /// </summary>
    public event WorldStep? PostStep;

    /// <summary>
    /// 允许访问驻留在非托管内存中的对象。<br></br><br></br>
    /// 此操作可能不安全。尽可能使用相应的本地属性来减轻风险。<br></br><br></br>
    /// Grants access to objects residing in unmanaged memory. This operation can be potentially unsafe. Utilize
    /// the corresponding native properties where possible to mitigate risk.
    /// </summary>
    public SpanData RawData => new(this);

    private readonly ConcurrentDictionary<ArbiterKey, Arbiter> arbiters = new();

    private readonly PartitionedSet<Island> islands = new();
    private readonly PartitionedSet<RigidBody> bodies = new();

    private static ulong _idCounter;

    /// <summary>
    /// 生成一个唯一的ID。<br></br><br></br>
    /// Generates a unique ID.
    /// </summary>
    public static ulong RequestId()
    {
        return Interlocked.Increment(ref _idCounter);
    }

    /// <summary>
    /// 生成一系列唯一的ID。<br></br><br></br>
    /// Generates a range of unique IDs.
    /// </summary>
    /// <param name="count"
    /// >要生成的ID数量。<br></br><br></br>
    /// The number of IDs to generate.</param>
    /// <returns>
    /// 包含生成范围内最小和最大请求ID的元组。上界绑定是排他的。<br></br><br></br>
    /// A tuple containing the minimum and maximum request IDs in the generated range. The upper
    /// bound is exclusive.</returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当计数小于1时抛出。<br></br><br></br>
    /// Thrown when count is less than 1.</exception>
    public static (ulong min, ulong max) RequestId(int count)
    {
        if (count < 1) throw new ArgumentOutOfRangeException(nameof(count), "Count must be greater zero.");
        ulong count64 = (ulong)count;
        ulong max = Interlocked.Add(ref _idCounter, count64) + 1;
        return (max - count64, max);
    }

    /// <summary>
    /// 定义了两个可用的线程模型。<br></br><br></br>
    /// <see cref="ThreadModelType.Persistent"/> 模型使工作线程持续活跃，<br></br>
    /// 即使在 <see cref="World.Step(Real, bool)"/> 不在运行时也是如此，<br></br>
    /// 这可能会消耗更多的CPU周期，并可能影响其他操作（如渲染）的性能。<br></br>
    /// 但是，它确保线程在下一次调用 <see cref="World.Step(Real, bool)"/> 时保持“温暖”。<br></br><br></br>
    /// 相反，<see cref="ThreadModelType.Regular"/> 模型允许工作线程让出并承担其他任务。<br></br><br></br>
    /// Defines the two available thread models. The <see cref="ThreadModelType.Persistent"/> model keeps the worker
    /// threads active continuously, even when the <see cref="World.Step(Real, bool)"/> is not in operation, which might
    /// consume more CPU cycles and possibly affect the performance of other operations such as rendering. However, it ensures that the threads
    /// remain 'warm' for the next invocation of <see cref="World.Step(Real, bool)"/>. Conversely, the <see cref="ThreadModelType.Regular"/> model allows
    /// the worker threads to yield and undertake other tasks.
    /// </summary>
    public ThreadModelType ThreadModel { get; set; } = ThreadModelType.Regular;

    /// <summary>
    /// 这个世界上的所有碰撞岛屿。<br></br><br></br>
    /// All collision islands in this world.
    /// </summary>
    public ReadOnlyPartitionedSet<Island> Islands => new(islands);

    /// <summary>
    /// 这个世界上的所有刚体。<br></br><br></br>
    /// All rigid bodies in this world.
    /// </summary>
    public ReadOnlyPartitionedSet<RigidBody> RigidBodies => new ReadOnlyPartitionedSet<RigidBody>(bodies);

    /// <summary>
    /// 动态树 <br></br><br></br>
    /// 访问 <see cref="DynamicTree"/> 实例。实例只能由 Jitter 修改。<br></br><br></br>
    /// Access to the <see cref="DynamicTree"/> instance. The instance
    /// should only be modified by Jitter.
    /// </summary>
    public DynamicTree DynamicTree { get; }

    /// <summary>
    /// 固定物体 <br></br><br></br>
    /// 一个固定物体，固定在世界。可以用来创建约束。<br></br><br></br>
    /// A fixed body, pinned to the world. Can be used to create constraints with.
    /// </summary>
    public RigidBody NullBody { get; }

    /// <summary>
    /// 启用停用机制 <br></br><br></br>
    /// 指定是否启用Jitter的停用机制。如果设置为false，则不会激活非活动对象。<br></br><br></br>
    /// Specifies whether the deactivation mechanism of Jitter is enabled.
    /// Does not activate inactive objects if set to false.
    /// </summary>
    public bool AllowDeactivation { get; set; } = true;

    /// <summary>
    /// 每个子步骤的迭代次数（求解器和松弛）（请参阅 <see cref="SubstepCount"/>）。<br></br><br></br>
    /// Number of iterations (solver and relaxation) per substep (see <see cref="SubstepCount"/>).
    /// </summary>
    /// <remarks>
    /// 默认值：（求解器：6，松弛度：4）<br></br><br></br>
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
    /// 子步数量 <br></br><br></br>
    /// 每次调用 <see cref="World.Step(Real, bool)"/> 时，子步的数量。<br></br>
    /// 当设置为 1 时，子步将被禁用。<br></br><br></br>
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
    /// 默认重力，也请参阅 <see cref="RigidBody.AffectedByGravity"/>。<br></br><br></br>
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
    /// 创建一个具有默认容量的<see cref="World"/>类的实例。<br></br><br></br>
    /// 这使用<see cref="Capacity.Default"/>中定义的身体数量、接触、约束和小型约束的默认值初始化世界。<br></br><br></br>
    /// Creates an instance of the <see cref="World"/> class with the default capacity.
    /// This initializes the world using default values for the number of bodies, contacts,
    /// constraints, and small constraints as defined in <see cref="Capacity.Default"/>.
    /// </summary>
    /// <seealso cref="World(Capacity)"/>
    public World() : this(Capacity.Default) { }

    /// <summary>
    /// 创建一个 World 类的实例。<br></br><br></br>
    /// 由于 Jitter 使用不同的内存模型，因此必须提前指定世界的容量。<br></br><br></br>
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
    /// <see cref="DynamicTree"/> 的默认过滤函数。<br></br><br></br>
    /// 如果两个代理都是 <see cref="RigidBodyShape"/> 类型且属于不同的实例，则返回true。<br></br><br></br>
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
    /// 从模拟世界中移除所有实体。<br></br><br></br>
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
    /// 从世界中移除指定的身体。<br></br><br></br>
    /// 此操作还自动丢弃任何相关的约束。<br></br><br></br>
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
    /// 从世界中移除一个特定的约束。<br></br><br></br>
    /// 对于临时禁用约束，请考虑使用<see cref="Constraint.IsEnabled"/>属性。<br></br><br></br>
    /// Removes a specific constraint from the world. For temporary deactivation of constraints, consider using the
    /// <see cref="Constraint.IsEnabled"/> property.
    /// </summary>
    /// <param name="constraint">要删除的约束。<br></br><br></br> The constraint to be removed.</param>
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
    /// 将特定的仲裁者从世界中移除。<br></br><br></br>
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
    /// 构造指定类型的约束。<br></br><br></br>
    /// 创建后，必须使用 Constraint.Initialize 方法初始化约束。<br></br><br></br>
    /// Constructs a constraint of the specified type. After creation, it is mandatory to initialize the constraint using the Constraint.Initialize method.
    /// </summary>
    /// <typeparam name="T">要创建的约束的具体类型。<br></br><br></br> The specific type of constraint to create.</typeparam>
    /// <param name="body1">约束中涉及的第 1 刚性体。<br></br><br></br> The first rigid body involved in the constraint.</param>
    /// <param name="body2">约束中涉及的第 2 刚性体。<br></br><br></br> The second rigid body involved in the constraint.</param>
    /// <returns>
    /// 指定约束类型的新实例。<br></br><br></br>
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
    /// 在模拟世界中创建并添加一个新的刚体。<br></br><br></br>
    /// Creates and adds a new rigid body to the simulation world.
    /// </summary>
    /// <returns>
    /// <see cref="RigidBody"/>.的新实例 <br></br><br></br>
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