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

using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 定义通用凸形的接口，该接口以其支持函数为特征。<br></br><br></br>
/// Defines an interface for a generic convex shape, which is characterized by its support function.
/// </summary>
public interface ISupportMappable
{
    /// <summary>
    /// 标识形状上沿指定方向最远的点。<br></br><br></br>
    /// Identifies the point on the shape that is furthest in the specified direction.
    /// </summary>
    /// <param name="direction">
    /// 寻找最远点的方向。无需归一化。<br></br><br></br>
    /// The direction in which to search for the furthest point. It does not need to be normalized.
    /// </param>
    void SupportMap(in JVector direction, out JVector result);

    /// <summary>
    /// 返回形状深处的一个点。<br></br>
    /// 这用于与支持映射函数的隐式定义一起工作的算法中。重心是一个不错的选择。<br></br><br></br>
    /// Returns a point deep within the shape. This is used in algorithms which work with the implicit
    /// definition of the support map function. The center of mass is a good choice.
    /// </summary>
    void GetCenter(out JVector point);
}