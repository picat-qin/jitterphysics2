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

using Jitter2.DataStructures;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 应添加到 <see cref="DynamicTree"/> 的实体的接口。<br></br><br></br>
/// Interface for entities which should be added to the <see cref="DynamicTree"/>.
/// </summary>
public interface IDynamicTreeProxy : IPartitionedSetIndex
{
    /// <summary>
    /// 指针值, 仅应由树内部修改的 <br></br><br></br>
    /// A pointer value which should only by internally modified by the tree.
    /// </summary>
    int NodePtr { get; set; }

    /// <summary>
    /// 实体的速度 <br></br><br></br>
    /// The velocity of the entity.
    /// </summary>
    JVector Velocity { get; }

    /// <summary>
    /// 世界边界框
    /// The world bounding box of the entity.
    /// </summary>
    JBBox WorldBoundingBox { get; }
}

/// <summary>
/// 表示可以更新边界框的对象。<br></br><br></br>
/// Represents an object for which the bounding box can be updated.
/// </summary>
public interface IUpdatableBoundingBox
{
    /// <summary>
    /// 更新世界边框
    /// Updates the bounding box.
    /// </summary>
    public void UpdateWorldBoundingBox(Real dt);
}

/// <summary>
/// 表示可以被射线相交的物体
/// Represents an object that can be intersected by a ray.
/// </summary>
public interface IRayCastable
{
    /// <summary>
    /// 对物体进行射线投射，检查从指定点发出并沿指定方向传播的射线是否与物体相交。<br></br><br></br>
    /// Performs a ray cast against the object, checking if a ray originating from a specified point
    /// and traveling in a specified direction intersects with the object.
    /// </summary>
    /// <param name="origin">射线的原点 <br></br><br></br> The starting point of the ray.</param>
    /// <param name="direction">
    /// 射线的方向, 该向量不必标准化 <br></br><br></br>
    /// The direction of the ray. This vector does not need to be normalized.
    /// </param>
    /// <param name="normal">
    /// 当此方法返回时，如果发生相交，则包含相交点处的表面法线。<br></br><br></br>
    /// When this method returns, contains the surface normal at the point of intersection, if an intersection occurs.
    /// </param>
    /// <param name="lambda">
    /// 此方法返回时，包含表示沿射线方向向量的距离的标量值 <br></br>
    /// 从 <paramref name="origin"/> 到交点。命中点可以按如下方式计算：
    /// <c>origin + lambda * direction</c>.<br></br><br></br>
    /// When this method returns, contains the scalar value representing the distance along the ray's direction vector
    /// from the <paramref name="origin"/> to the intersection point. The hit point can be calculated as:
    /// <c>origin + lambda * direction</c>.
    /// </param>
    /// <returns>
    /// 如果射线与物体相交，则为 <c>true</c>；否则为 <c>false</c>。<br></br><br></br>
    /// <c>true</c> if the ray intersects with the object; otherwise, <c>false</c>.
    /// </returns>
    public bool RayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda);
}