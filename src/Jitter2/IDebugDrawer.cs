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

namespace Jitter2;

/// <summary>
/// 拥有调试模式下能进行渲染的能力
/// </summary>
public interface IDebugDrawable
{
    public void DebugDraw(IDebugDrawer drawer);
}

/// <summary>
/// 调试模式下渲染
/// </summary>
public interface IDebugDrawer
{
    /// <summary>
    /// 渲染线段 
    /// </summary>
    /// <param name="pA"></param>
    /// <param name="pB"></param>
    public void DrawSegment(in JVector pA, in JVector pB);
    /// <summary>
    /// 渲染三角形
    /// </summary>
    /// <param name="pA"></param>
    /// <param name="pB"></param>
    /// <param name="pC"></param>
    public void DrawTriangle(in JVector pA, in JVector pB, in JVector pC);
    /// <summary>
    /// 渲染点
    /// </summary>
    /// <param name="p"></param>
    public void DrawPoint(in JVector p);
}