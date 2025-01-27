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
using Jitter2.DataStructures;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 碰撞系统的主要实体。<br></br><br></br>
/// 实现 <see cref="ISupportMappable"/> 以进行窄相碰撞检测，<br></br>
/// 并实现 <see cref="IDynamicTreeProxy"/> 以进行宽相碰撞检测。<br></br>
/// 形状本身没有位置或方向。形状可以与<see cref="RigidBody"/> 的实例相关联。<br></br><br></br>
/// he main entity of the collision system. Implements <see cref="ISupportMappable"/> for
/// narrow-phase and <see cref="IDynamicTreeProxy"/> for broad-phase collision detection.
/// The shape itself does not have a position or orientation. Shapes can be associated with
/// instances of <see cref="RigidBody"/>.
/// </summary>
public abstract class Shape : IDynamicTreeProxy, IUpdatableBoundingBox, ISupportMappable, IRayCastable
{
    int IPartitionedSetIndex.SetIndex { get; set; } = -1;

    /// <summary>
    /// 表示形状 ID 的 64 位整数。<br></br>
    /// 需要按照明确定义的顺序排列形状的算法会使用它。<br></br><br></br>
    /// A 64-bit integer representing the shape ID. This is used by algorithms that require
    /// arranging shapes in a well-defined order.
    /// </summary>
    public readonly ulong ShapeId = World.RequestId();

    /// <summary>
    /// 世界空间中形状的边界框。<br></br><br></br>
    /// 当 <see cref="RigidBody"/> 的相应实例的位置或方向发生变化时，它会自动更新。<br></br><br></br>
    /// The bounding box of the shape in world space. It is automatically updated when the position or
    /// orientation of the corresponding instance of <see cref="RigidBody"/> changes.
    /// </summary>
    public JBBox WorldBoundingBox { get; protected set; }

    int IDynamicTreeProxy.NodePtr { get; set; }

    protected void SweptExpandBoundingBox(Real dt)
    {
        JVector sweptDirection = dt * Velocity;

        JBBox box = WorldBoundingBox;

        Real sxa = MathR.Abs(sweptDirection.X);
        Real sya = MathR.Abs(sweptDirection.Y);
        Real sza = MathR.Abs(sweptDirection.Z);

        Real max = MathR.Max(MathR.Max(sxa, sya), sza);

        if (sweptDirection.X < (Real)0.0) box.Min.X -= max;
        else box.Max.X += max;

        if (sweptDirection.Y < (Real)0.0) box.Min.Y -= max;
        else box.Max.Y += max;

        if (sweptDirection.Z < (Real)0.0) box.Min.Z -= max;
        else box.Max.Z += max;

        WorldBoundingBox = box;
    }

    /// <summary>
    /// 已注册
    /// </summary>
    public bool IsRegistered => (this as IPartitionedSetIndex).SetIndex != -1;

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract JVector Velocity { get; }

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract void UpdateWorldBoundingBox(Real dt = (Real)0.0);

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract bool RayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda);

    /// <inheritdoc/>
    [ReferenceFrame(ReferenceFrame.Local)]
    public abstract void SupportMap(in JVector direction, out JVector result);

    /// <inheritdoc/>
    [ReferenceFrame(ReferenceFrame.Local)]
    public abstract void GetCenter(out JVector point);
}