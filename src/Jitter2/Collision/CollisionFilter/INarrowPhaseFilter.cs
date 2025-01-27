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

using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 窄相滤波器<br></br>
/// 接口用于促进通用过滤器的实现。此过滤器可以排除某些形状对，也可以在 Jitter 执行形状之间的窄相碰撞检测后修改碰撞信息。<br></br><br></br>
/// nterface to facilitate the implementation of a generic filter. This filter can either exclude certain pairs of shapes or modify collision
/// information subsequent to Jitter's execution of narrow phase collision detection between the shapes.
/// </summary>
public interface INarrowPhaseFilter
{
    /// <summary>
    /// 在 Jitter 中碰撞检测的窄阶段之后调用。这允许修改碰撞信息。<br></br>
    /// 有关参数的详细信息，请参阅相应的 <see cref="NarrowPhase"/> 方法。<br></br><br></br>
    /// Invoked following the narrow phase of collision detection in Jitter. This allows for the modification of collision information.
    /// Refer to the corresponding <see cref="NarrowPhase"/> methods for details on the parameters.
    /// </summary>
    /// <returns>
    /// 如果应过滤掉碰撞则为 false，否则为 true。<br></br>
    /// False if the collision should be filtered out, true otherwise.
    /// </returns>
    bool Filter(RigidBodyShape shapeA, RigidBodyShape shapeB,
        ref JVector pointA, ref JVector pointB,
        ref JVector normal, ref Real penetration);
}