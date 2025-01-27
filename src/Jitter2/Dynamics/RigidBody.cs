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
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.DataStructures;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.UnmanagedMemory;

namespace Jitter2.Dynamics;

/// <summary>
/// 刚体数据
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct RigidBodyData
{
    public int _index;
    public int _lockFlag;

    /// <summary>
    /// 位置
    /// </summary>
    public JVector Position;
    /// <summary>
    /// 速度
    /// </summary>
    public JVector Velocity;
    /// <summary>
    /// 角速度
    /// </summary>
    public JVector AngularVelocity;

    /// <summary>
    /// Delta速度
    /// </summary>
    public JVector DeltaVelocity;
    /// <summary>
    /// Delta角速度
    /// </summary>
    public JVector DeltaAngularVelocity;

    /// <summary>
    /// 方向
    /// </summary>
    public JQuaternion Orientation;
    /// <summary>
    /// 反惯性世界
    /// </summary>
    public JMatrix InverseInertiaWorld;

    /// <summary>
    /// 反质量
    /// </summary>
    public Real InverseMass;
    /// <summary>
    /// 是否激活
    /// </summary>
    public bool IsActive;
    /// <summary>
    /// 是否静止
    /// </summary>
    public bool IsStatic;

    /// <summary>
    /// 静止还是激活
    /// </summary>
    public readonly bool IsStaticOrInactive => !IsActive || IsStatic;
}

/// <summary>
/// 刚体 <br></br><br></br>
/// 表示 Jitter 物理世界中的主要实体。<br></br><br></br>
/// Represents the primary entity in the Jitter physics world.
/// </summary>
public sealed class RigidBody : IPartitionedSetIndex, IDebugDrawable
{
    internal JHandle<RigidBodyData> handle;

    /// <summary>
    /// 注册物体ID
    /// </summary>
    public readonly ulong RigidBodyId;

    private Real restitution = (Real)0.0;
    private Real friction = (Real)0.2;

    /// <summary>
    /// 物体的数据 <br></br><br></br>
    /// 由于性能考虑，用于模拟此物体的数据（例如，速度或位置）
    /// 保存在连续的非托管内存块中。这指的是原始内存位置。
    /// 并且很少（如果有的话）应该在引擎之外使用。相反，使用提供的属性<br></br><br></br>
    /// 由 <see cref="RigidBody"/> 类本身实现。
    /// Due to performance considerations, the data used to simulate this body (e.g., velocity or position)
    /// is stored within a contiguous block of unmanaged memory. This refers to the raw memory location
    /// and should seldom, if ever, be utilized outside of the engine. Instead, use the properties provided
    /// by the <see cref="RigidBody"/> class itself.
    /// </summary>
    public ref RigidBodyData Data => ref handle.Data;

    /// <summary>
    /// 获取刚体数据的句柄，请参阅 <see cref="Data"/>。<br></br><br></br>
    /// Gets the handle to the rigid body data, see <see cref="Data"/>.
    /// </summary>
    public JHandle<RigidBodyData> Handle => handle;

    internal readonly List<RigidBodyShape> shapes = new(1);

    // There is only one way to create a body: world.CreateRigidBody. There, we add an island
    // to the new body. This should never be null.
    internal Island island = null!;

    /// <summary>
    /// 获取与这个刚体关联的碰撞岛屿。<br></br><br></br>
    /// Gets the collision island associated with this rigid body.
    /// </summary>
    public Island Island => island;

    internal readonly List<RigidBody> connections = new(0);
    internal readonly HashSet<Arbiter> contacts = new(0);
    internal readonly HashSet<Constraint> constraints = new(0);

    /// <summary>
    /// 当创建新仲裁者时触发的事件，表示两个实体已开始碰撞。<br></br><br></br>
    /// Event triggered when a new arbiter is created, indicating that two bodies have begun colliding.
    /// </summary>
    /// <remarks>
    /// 此事件提供包含碰撞细节的 <see cref="Arbiter"/> 对象。<br></br>
    /// 使用此事件处理应在两个物体之间的碰撞开始时发生的逻辑。<br></br><br></br>
    /// This event provides an <see cref="Arbiter"/> object which contains details about the collision.
    /// Use this event to handle logic that should occur at the start of a collision between two bodies.
    /// </remarks>
    public event Action<Arbiter>? BeginCollide;

    /// <summary>
    /// 当仲裁者被摧毁时触发的事件，表示两个物体已停止碰撞。<br></br>
    /// 在此调用后，对仲裁器的引用将无效。    <br></br><br></br>
    /// Event triggered when an arbiter is destroyed, indicating that two bodies have stopped colliding.
    /// The reference to this arbiter becomes invalid after this call.
    /// </summary>
    /// <remarks>
    /// 此事件提供 <see cref="Arbiter"/> 对象，其中包含有关已结束的碰撞的详细信息。<br></br>
    /// 使用此事件处理当两个物体之间的碰撞结束时应该发生的逻辑。<br></br><br></br>
    /// This event provides an <see cref="Arbiter"/> object which contains details about the collision that has ended.
    /// Use this event to handle logic that should occur when the collision between two bodies ends.
    /// </remarks>
    public event Action<Arbiter>? EndCollide;

    internal void RaiseBeginCollide(Arbiter arbiter)
    {
        BeginCollide?.Invoke(arbiter);
    }

    internal void RaiseEndCollide(Arbiter arbiter)
    {
        EndCollide?.Invoke(arbiter);
    }

    /// <summary>
    /// 所有连接的物体 <br></br><br></br>
    /// Contains all bodies this body is in contact with.
    /// </summary>
    public ReadOnlyList<RigidBody> Connections => new(connections);

    /// <summary>
    /// 包含此主体参与的所有接触联系。<br></br><br></br>
    /// Contains all contacts in which this body is involved.
    /// </summary>
    public ReadOnlyHashSet<Arbiter> Contacts => new(contacts);

    /// <summary>
    /// 包含与这个主体相关的所有约束条件。<br></br><br></br>
    /// Contains all constraints connected to this body.
    /// </summary>
    public ReadOnlyHashSet<Constraint> Constraints => new(constraints);

    /// <summary>
    /// 获取添加到此刚体的形状列表。
    /// Gets the list of shapes added to this rigid body.
    /// </summary>
    public ReadOnlyList<RigidBodyShape> Shapes => new(shapes);

    internal int islandMarker;

    internal Real sleepTime = (Real)0.0;

    internal Real inactiveThresholdLinearSq = (Real)0.1;
    internal Real inactiveThresholdAngularSq = (Real)0.1;
    internal Real deactivationTimeThreshold = (Real)1.0;

    internal Real linearDampingMultiplier = (Real)0.998;
    internal Real angularDampingMultiplier = (Real)0.995;

    internal JMatrix inverseInertia = JMatrix.Identity;
    internal Real inverseMass = (Real)1.0;

    /// <remarks>
    /// 摩擦系数, 决定了滑动运动的阻力。<br></br>
    /// 值通常从0（无摩擦）开始向上。
    /// 较高的值表示强烈的摩擦或粘附效应。默认值为0.2。<br></br><br></br>
    /// The friction coefficient determines the resistance to sliding motion.
    /// Values typically range from 0 (no friction) upwards.
    /// Higher values represent strong friction or adhesion effects.
    /// Default is 0.2.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 如果值为负数，则抛出此异常。<br></br><br></br>
    /// Thrown if the value is negative.
    /// </exception>
    public Real Friction
    {
        get => friction;
        set
        {
            if (value < (Real)0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "Friction must be non-negative.");
            }

            friction = value;
        }
    }

    /// <summary>
    /// 反弹系数<br></br>
    /// 获取或设置此对象的恢复（反弹性）。<br></br><br></br>
    /// Gets or sets the restitution (bounciness) of this object.
    /// </summary>
    /// <remarks>
    /// 恢复值决定了碰撞后保留的能量量，<br></br>
    /// 0表示非弹性碰撞（无反弹），1表示完全弹性碰撞（完全反弹）。<br></br>
    /// 介于0和1之间的值会产生部分弹性的碰撞效果。默认值为0.0。<br></br><br></br>
    /// The restitution value determines how much energy is retained after a collision,
    /// with 0 representing an inelastic collision (no bounce) and 1 representing a perfectly elastic collision (full bounce).
    /// Values between 0 and 1 create a partially elastic collision effect.
    /// Default is 0.0.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 如果值不在0和1之间，则抛出此异常 <br></br><br></br>
    /// Thrown if the value is not between 0 and 1.
    /// </exception>
    public Real Restitution
    {
        get => restitution;
        set
        {
            if (value < (Real)0.0 || value > (Real)1.0)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "Restitution must be between 0 and 1.");
            }

            restitution = value;
        }
    }

    private readonly int hashCode;

    /// <summary>
    /// 获取物体所属世界。<br></br><br></br>
    /// Gets or sets the world assigned to this body.
    /// </summary>
    public World World { get; }

    internal RigidBody(JHandle<RigidBodyData> handle, World world)
    {
        this.handle = handle;
        World = world;

        Data.Orientation = JQuaternion.Identity;
        SetDefaultMassInertia();

        RigidBodyId = World.RequestId();
        uint h = (uint)RigidBodyId;

        // The rigid body is used in hash-based data structures, provide a
        // good hash - Thomas Wang, Jan 1997
        h = h ^ 61 ^ (h >> 16);
        h += h << 3;
        h ^= h >> 4;
        h *= 0x27d4eb2d;
        h ^= h >> 15;

        hashCode = unchecked((int)h);

        Data._lockFlag = 0;
    }

    /// <summary>
    /// 停用时间。<br></br>
    /// 如果刚体的角速度和线速度都很大，如果<see cref = "DeactivationThreshold" /> 
    /// 在指定时间内保持在 <see cref="DeactivationThreshold"/> 以下，则主体将被停用。<br></br><br></br>
    /// Gets or sets the deactivation time. If the magnitudes of both the angular and linear velocity of the rigid body
    /// remain below the <see cref="DeactivationThreshold"/> for the specified time, the body is deactivated.
    /// </summary>
    public TimeSpan DeactivationTime
    {
        get => TimeSpan.FromSeconds(deactivationTimeThreshold);
        set => deactivationTimeThreshold = (Real)value.TotalSeconds;
    }

    /// <summary>
    /// 停用阈值。<br></br><br></br>
    /// 如果刚体的角速度和线速度都达到该阈值，则刚体停用。
    /// 在<see cref = "DeactivationTime" /> 的持续时间内，如果<see cref = "DeactivationThreshold" /> 始终低于指定值，则物体将被停用。<br></br>
    /// 阈值值分别以 (弧度/秒) 和 (长度单位/秒) 给出。<br></br><br></br>
    /// Gets or sets the deactivation threshold. If the magnitudes of both the angular and linear velocity of the rigid body
    /// remain below the specified values for the duration of <see cref="DeactivationTime"/>, the body is deactivated.
    /// The threshold values are given in rad/s and length units/s, respectively.
    /// </summary>

    public (Real angular, Real linear) DeactivationThreshold
    {
        get => (MathR.Sqrt(inactiveThresholdAngularSq), MathR.Sqrt(inactiveThresholdLinearSq));
        set
        {
            if (value.linear < 0 || value.angular < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "Both linear and angular thresholds must be non-negative.");
            }

            inactiveThresholdLinearSq = value.linear * value.linear;
            inactiveThresholdAngularSq = value.angular * value.angular;
        }
    }

    /// <summary>
    /// 线性运动和角运动的阻尼系数。<br></br><br></br>
    /// 阻尼系数 0 表示物体没有阻尼，而 1 则带来身体立即停止。<br></br>
    /// 调用 <see cref="World.Step(Real, bool)"/> 抖动将相应的速度每步减 1 减去阻尼系数。
    /// 请注意，值不按时间缩放; 较小的时间步长 <see cref="World.Step(Real, bool)"/> 会导致阻尼增加。<br></br><br></br>
    /// Gets or sets the damping factors for linear and angular motion.
    /// A damping factor of 0 means the body is not damped, while 1 brings
    /// the body to a halt immediately. Damping is applied when calling
    /// <see cref="World.Step(Real, bool)"/>. Jitter multiplies the respective
    /// velocity each step by 1 minus the damping factor. Note that the values
    /// are not scaled by time; a smaller time-step in
    /// <see cref="World.Step(Real, bool)"/> results in increased damping.
    /// </summary>
    /// <remarks>
    /// 阻尼系数在 [0, 1] 之间 <br></br><br></br>
    /// The damping factors should be within the range [0, 1].
    /// </remarks>
    public (Real linear, Real angular) Damping
    {
        get => ((Real)1.0 - linearDampingMultiplier, (Real)1.0 - angularDampingMultiplier);
        set
        {
            if (value.linear < (Real)0.0 || value.linear > (Real)1.0 || value.angular < (Real)0.0 || value.angular > (Real)1.0)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "Damping multiplier has to be within [0, 1].");
            }

            linearDampingMultiplier = (Real)1.0 - value.linear;
            angularDampingMultiplier = (Real)1.0 - value.angular;
        }
    }

    public override int GetHashCode()
    {
        return hashCode;
    }

    private void SetDefaultMassInertia()
    {
        inverseInertia = JMatrix.Identity;
        Data.InverseMass = (Real)1.0;
        UpdateWorldInertia();
    }

    /// <summary>
    /// 逆惯性
    /// </summary>
    public JMatrix InverseInertia => inverseInertia;

    /// <summary>
    /// 位置
    /// </summary>
    public JVector Position
    {
        get => handle.Data.Position;
        set
        {
            handle.Data.Position = value;
            Move();
        }
    }

    /// <summary>
    /// 方向
    /// </summary>
    public JQuaternion Orientation
    {
        get => Data.Orientation;
        set
        {
            Data.Orientation = value;
            Move();
        }
    }

    private void Move()
    {
        UpdateWorldInertia();
        foreach (RigidBodyShape shape in shapes)
        {
            World.UpdateShape(shape);
        }

        World.ActivateBodyNextStep(this);
    }

    /// <summary>
    /// 速度
    /// </summary>
    public JVector Velocity
    {
        get => handle.Data.Velocity;
        set => handle.Data.Velocity = value;
    }

    /// <summary>
    /// 角速度
    /// </summary>
    public JVector AngularVelocity
    {
        get => handle.Data.AngularVelocity;
        set => handle.Data.AngularVelocity = value;
    }

    /// <summary>
    /// 是否受到重力影响
    /// </summary>
    public bool AffectedByGravity { get; set; } = true;

    /// <summary>
    /// 指向自定义用户数据的托管指针。引擎不利用此功能。<br></br><br></br>
    /// A managed pointer to custom user data. This is not utilized by the engine.
    /// </summary>
    public object? Tag { get; set; }

    /// <summary>
    /// 启用预测性接触
    /// </summary>
    public bool EnableSpeculativeContacts { get; set; } = false;

    private void UpdateWorldInertia()
    {
        if (Data.IsStatic)
        {
            Data.InverseInertiaWorld = JMatrix.Zero;
            Data.InverseMass = (Real)0.0;
        }
        else
        {
            var bodyOrientation = JMatrix.CreateFromQuaternion(Data.Orientation);
            JMatrix.Multiply(bodyOrientation, inverseInertia, out Data.InverseInertiaWorld);
            JMatrix.MultiplyTransposed(Data.InverseInertiaWorld, bodyOrientation, out Data.InverseInertiaWorld);
            Data.InverseMass = inverseMass;
        }
    }

    /// <summary>
    /// 是否静止
    /// </summary>
    public bool IsStatic
    {
        set
        {
            if ((!Data.IsStatic && value) || (Data.IsStatic && !value))
            {
                Data.Velocity = JVector.Zero;
                Data.AngularVelocity = JVector.Zero;
            }

            Data.IsStatic = value;
            UpdateWorldInertia();

            if (value) World.DeactivateBodyNextStep(this);
            else World.ActivateBodyNextStep(this);
        }
        get => Data.IsStatic;
    }

    /// <summary>
    /// 指示刚体是否处于活动状态或被视为处于睡眠状态。<br></br>
    /// 使用 <see cref="SetActivationState"/> 来更改激活状态。<br></br><br></br>
    /// Indicates whether the rigid body is active or considered to be in a sleeping state.
    /// Use <see cref="SetActivationState"/> to alter the activation state.
    /// </summary>
    public bool IsActive => Data.IsActive;

    /// <summary>
    /// 命令Jitter在下一个时间步开始时激活或停用身体。当前状态不会立即改变。<br></br><br></br>
    /// Instructs Jitter to activate or deactivate the body at the commencement of
    /// the next time step. The current state does not change immediately.
    /// </summary>
    public void SetActivationState(bool active)
    {
        if (active) World.ActivateBodyNextStep(this);
        else World.DeactivateBodyNextStep(this);
    }

    private void AttachToShape(RigidBodyShape shape)
    {
        if (shape.RigidBody != null)
        {
            throw new ArgumentException("Shape has already been added to a body.", nameof(shape));
        }

        shape.RigidBody = this;
        shape.UpdateWorldBoundingBox();
        World.DynamicTree.AddProxy(shape, IsActive);
    }

    /// <summary>
    /// 批量添加形状到刚体中。<br></br>
    /// 如果需要，质量属性只重新计算一次。<br></br><br></br>
    /// Adds several shapes to the rigid body at once. Mass properties are
    /// recalculated only once, if requested.
    /// </summary>
    /// <param name="shapes">要添加的形状。<br></br><br></br> The Shapes to add.</param>
    /// <param name="setMassInertia">
    /// 是否设置质量<br></br><br></br>
    /// 如果为 true，则使用形状的质量属性来确定物体的质量属性，假设形状的单位密度。<br></br>
    /// 如果为 false，则惯性质量和质量保持不变。<br></br><br></br>
    /// If true, uses the mass properties of the Shapes to determine the
    /// body's mass properties, assuming unit density for the Shapes. If false, the inertia and mass remain unchanged.
    /// </param>
    public void AddShape(IEnumerable<RigidBodyShape> shapes, bool setMassInertia = true)
    {
        foreach (RigidBodyShape shape in shapes)
        {
            AttachToShape(shape);
            this.shapes.Add(shape);
        }

        if (setMassInertia) SetMassInertia();
    }

    /// <summary>
    /// 添加形状到主体 <br></br><br></br>
    /// Adds a shape to the body.
    /// </summary>
    /// <param name="shape">要添加的形状 <br></br><br></br> The shape to be added.</param>
    /// <param name="setMassInertia">
    /// 是否设置质量<br></br><br></br>
    /// 如果为 true，则使用形状的质量属性来确定物体的质量属性，假设形状的单位密度。<br></br>
    /// 如果为 false，则惯性质量和质量保持不变。<br></br><br></br>
    /// If true, uses the mass properties of the Shapes to determine the
    /// body's mass properties, assuming unit density for the Shapes. If false, the inertia and mass remain unchanged.
    /// </param>
    public void AddShape(RigidBodyShape shape, bool setMassInertia = true)
    {
        if (shape.IsRegistered)
        {
            throw new ArgumentException("Shape can not be added. Is the shape already registered?");
        }

        AttachToShape(shape);
        shapes.Add(shape);
        if (setMassInertia) SetMassInertia();
    }

    /// <summary> 
    /// 力 <br></br><br></br>
    /// 表示在下一次调用 <see cref="World.Step(Real, bool)"/> 时施加到身体上的力。
    /// 调用后，此值会自动重置为0。<br></br><br></br>
    /// Represents the force to be applied to the body during the next call to <see cref="World.Step(Real, bool)"/>.
    /// This value is automatically reset to zero after the call.
    /// </summary>
    public JVector Force { get; set; }

    /// <summary>
    /// 扭矩 <br></br><br></br>
    /// 表示在下一次调用 <see cref="World.Step(Real, bool)"/> 时施加到身体上的扭矩。
    /// 调用后，此值会自动重置为0。<br></br><br></br>
    /// Represents the torque to be applied to the body during the next call to <see cref="World.Step(Real, bool)"/>.
    /// This value is automatically reset to zero after the call.
    /// </summary>
    public JVector Torque { get; set; }

    /// <summary>
    /// 施加力 <br></br><br></br>
    /// 将力施加到刚体，从而改变其速度。<br></br>
    /// 此力仅对单个帧有效，并在下一次调用 <see cref="World.Step(Real, bool)"/> 时重置为 0。<br></br><br></br>
    /// Applies a force to the rigid body, thereby altering its velocity. This force is effective for a single frame only and is reset to zero during the next call to <see cref="World.Step(Real, bool)"/>.
    /// </summary>
    /// <param name="force">追加的力 <br></br><br></br> The force to be applied.</param>
    public void AddForce(in JVector force)
    {
        Force += force;
    }

    /// <summary>
    /// 施加力 <br></br><br></br>
    /// 向刚体施加力，改变其速度。<br></br>
    /// 此力仅应用于单个帧，并在后续调用<see cref="World.Step(Real, bool)"/>时重置为 0。<br></br><br></br>
    /// Applies a force to the rigid body, altering its velocity. This force is applied for a single frame only and is
    /// reset to zero with the subsequent call to <see cref="World.Step(Real, bool)"/>.
    /// </summary>
    /// <param name="force">追加的力 <br></br><br></br> The force to be applied.</param>
    /// <param name="position">力将被施加的位置。<br></br><br></br> The position where the force will be applied.</param>
    [ReferenceFrame(ReferenceFrame.World)]
    public void AddForce(in JVector force, in JVector position)
    {
        ref RigidBodyData data = ref Data;

        if (data.IsStatic) return;

        JVector.Subtract(position, data.Position, out JVector torque);
        JVector.Cross(torque, force, out torque);

        Force += force;
        Torque += torque;
    }

    /// <summary>
    /// 从刚体中移除指定的形状。<br></br><br></br>
    /// Removes a specified shape from the rigid body.
    /// </summary>
    /// <remarks>
    /// 这个操作的时间复杂度为O（n），其中n是附着在身体上的形状的数量。<br></br><br></br>
    /// This operation has a time complexity of O(n), where n is the number of shapes attached to the body.
    /// </remarks>
    /// <param name="shape">要移除的形状 <br></br><br></br> The shape to remove from the rigid body.</param>
    /// <param name="setMassInertia">
    /// 是否重算质量 <br></br><br></br>
    /// 指定在移除形状后是否调整刚体的质量惯性属性。默认值为 true。<br></br><br></br>
    /// Specifies whether to adjust the mass inertia properties of the rigid body after removing the shape. The default value is true.
    /// </param>
    public void RemoveShape(RigidBodyShape shape, bool setMassInertia = true)
    {
        if (!shapes.Remove(shape))
        {
            throw new ArgumentException(
                "Shape is not part of this body.");
        }

        foreach (var arbiter in contacts)
        {
            if (arbiter.Handle.Data.Key.Key1 == shape.ShapeId || arbiter.Handle.Data.Key.Key2 == shape.ShapeId)
            {
                // Removes the current element we are iterating over from Contacts, i.e. the HashSet
                // we are iterating over is altered. This is allowed.
                World.Remove(arbiter);
            }
        }

        World.DynamicTree.RemoveProxy(shape);
        shape.RigidBody = null!;

        if (setMassInertia) SetMassInertia();
    }

    /// <summary>
    /// 批量移除主体中的形状 <br></br><br></br>
    /// Removes several shapes from the body.
    /// </summary>
    /// <remarks>
    /// 这个操作的时间复杂度为O（n），其中n是附着在身体上的形状的数量。<br></br><br></br>
    /// This operation has a time complexity of O(n), where n is the number of shapes attached to the body.
    /// </remarks>
    /// <param name="shapes">要移除的形状集 <br></br> The shapes to remove from the rigid body.</param>
    /// <param name="setMassInertia">
    /// 是否重算质量 <br></br><br></br>
    /// 指定在移除形状后是否调整刚体的质量惯性属性。默认值为 true。<br></br><br></br>
    /// Specifies whether to adjust the mass inertia properties of the rigid body after removing the shape. The default value is true.
    /// </param>
    public void RemoveShape(IEnumerable<RigidBodyShape> shapes, bool setMassInertia = true)
    {
        HashSet<ulong> sids = new HashSet<ulong>();

        foreach (var shape in shapes)
        {
            if (shape.RigidBody != this)
            {
                throw new ArgumentException($"Shape {shape} is not attached to this body.", nameof(shapes));
            }

            sids.Add(shape.ShapeId);
        }

        foreach (var arbiter in contacts)
        {
            if (sids.Contains(arbiter.Handle.Data.Key.Key1) || sids.Contains(arbiter.Handle.Data.Key.Key2))
            {
                // Removes the current element we are iterating over from Contacts, i.e. the HashSet
                // we are iterating over is altered. This is allowed.
                World.Remove(arbiter);
            }
        }

        for (int i = this.shapes.Count; i-- > 0;)
        {
            var shape = this.shapes[i];

            if (sids.Contains(shape.ShapeId))
            {
                World.DynamicTree.RemoveProxy(shape);
                shape.RigidBody = null!;
                this.shapes.RemoveAt(i);
            }
        }

        if (setMassInertia) SetMassInertia();

        sids.Clear();
    }

    /// <summary>
    /// 移除所有刚体关联的形状 <br></br><br></br>
    /// Removes all shapes associated with the rigid body.
    /// </summary>
    /// <remarks>
    /// 这个操作的时间复杂度为O（n），其中n是附着在身体上的形状的数量。<br></br><br></br>
    /// This operation has a time complexity of O(n), where n is the number of shapes attached to the body.
    /// </remarks>
    /// <param name="setMassInertia">
    /// 是否重算质量 <br></br><br></br>
    /// 如果设置为false，则刚体的质量属性保持不变。<br></br><br></br>
    /// If set to false, the mass properties of the rigid body remain unchanged.
    /// </param>
    [Obsolete($"{nameof(ClearShapes)} is deprecated, please use {nameof(RemoveShape)} instead.")]
    public void ClearShapes(bool setMassInertia = true)
    {
        RemoveShape(shapes, setMassInertia);
    }

    /// <summary>
    /// 设置质量惯性 <br></br><br></br>
    /// 利用形状的质量属性来确定刚体的质量属性。<br></br><br></br>
    /// Utilizes the mass properties of the shape to determine the mass properties of the rigid body.
    /// </summary>
    public void SetMassInertia()
    {
        if (shapes.Count == 0)
        {
            inverseInertia = JMatrix.Identity;
            Data.InverseMass = (Real)1.0;
            return;
        }

        JMatrix inertia = JMatrix.Zero;
        Real mass = (Real)0.0;

        for (int i = 0; i < shapes.Count; i++)
        {
            shapes[i].CalculateMassInertia(out var shapeInertia, out _, out var shapeMass);

            inertia += shapeInertia;
            mass += shapeMass;
        }

        if (!JMatrix.Inverse(inertia, out inverseInertia))
        {
            throw new InvalidOperationException("Inertia matrix is not invertible. This might happen if a shape has " +
                                                "invalid mass properties. If you encounter this while calling " +
                                                "RigidBody.AddShape, call AddShape with setMassInertia set to false.");
        }

        inverseMass = (Real)1.0 / mass;

        UpdateWorldInertia();
    }

    /// <summary>
    /// 设置质量惯性 <br></br><br></br>
    /// 设置新的质量值，并根据旧质量与新质量的比例对惯性进行缩放。<br></br><br></br>
    /// Sets a new mass value and scales the inertia according to the ratio of the old mass to the new mass.
    /// </summary>
    public void SetMassInertia(Real mass)
    {
        if (mass <= (Real)0.0)
        {
            // we do not protect against NaN here, since it is the users responsibility
            // to not feed NaNs to the engine.
            throw new ArgumentException("Mass can not be zero or negative.", nameof(mass));
        }

        SetMassInertia();
        inverseInertia = JMatrix.Multiply(inverseInertia, (Real)1.0 / (Data.InverseMass * mass));
        inverseMass = (Real)1.0 / mass;
        UpdateWorldInertia();
    }

    /// <summary>
    /// 设置质量惯性
    /// 通过直接指定惯性和质量来设置此物体的新的质量属性。<br></br><br></br>
    /// Sets the new mass properties of this body by specifying both inertia and mass directly.
    /// </summary>
    /// <param name="setAsInverse">Set the inverse values.</param>
    /// <exception cref="ArgumentException"></exception>
    public void SetMassInertia(in JMatrix inertia, Real mass, bool setAsInverse = false)
    {
        if (setAsInverse)
        {
            if (Real.IsInfinity(mass) || mass < (Real)0.0)
            {
                throw new ArgumentException("Inverse mass must be finite and not negative.", nameof(mass));
            }

            inverseInertia = inertia;
            inverseMass = mass;
        }
        else
        {
            if (mass <= (Real)0.0)
            {
                throw new ArgumentException("Mass can not be zero or negative.", nameof(mass));
            }

            if (!JMatrix.Inverse(inertia, out inverseInertia))
            {
                throw new ArgumentException("Inertia matrix is not invertible.", nameof(inertia));
            }

            inverseMass = (Real)1.0 / mass;
        }

        UpdateWorldInertia();
    }

    private static List<JTriangle>? debugTriangles;

    /// <summary>
    /// 生成身体形状的大致三角形近似值。<br></br>
    /// 由于生成速度慢，这应该仅用于调试目的 <br></br><br></br>
    /// Generates a rough triangle approximation of the shapes of the body.
    /// Since the generation is slow this should only be used for debugging
    /// purposes.
    /// </summary>
    public void DebugDraw(IDebugDrawer drawer)
    {
        debugTriangles ??= new List<JTriangle>();

        foreach (var shape in shapes)
        {
            ShapeHelper.MakeHull(shape, debugTriangles);

            foreach (var tri in debugTriangles)
            {
                drawer.DrawTriangle(
                    JVector.Transform(tri.V0, Data.Orientation) + Data.Position,
                    JVector.Transform(tri.V1, Data.Orientation) + Data.Position,
                    JVector.Transform(tri.V2, Data.Orientation) + Data.Position);
            }
        }

        debugTriangles.Clear();
    }

    /// <summary>
    /// 刚体的质量 <br></br><br></br>
    /// 要修改质量，使用
    /// <see cref="RigidBody.SetMassInertia(Real)"/> or
    /// <see cref="RigidBody.SetMassInertia(in JMatrix, Real, bool)"/>.<br></br><br></br>
    /// Gets the mass of the rigid body. To modify the mass, use
    /// <see cref="RigidBody.SetMassInertia(Real)"/> or
    /// <see cref="RigidBody.SetMassInertia(in JMatrix, Real, bool)"/>.
    /// </summary>
    public Real Mass => (Real)1.0 / inverseMass;

    int IPartitionedSetIndex.SetIndex { get; set; } = -1;
}