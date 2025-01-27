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

namespace Jitter2;

/// <summary>
/// 表示参考框架的枚举。<br></br><br></br>
/// Enum representing reference frames.
/// </summary>
public enum ReferenceFrame
{
    /// <summary>
    /// 本地
    /// </summary>
    Local,
    /// <summary>
    /// 全局
    /// </summary>
    World
}

/// <summary>
/// 参考特性 <br></br><br></br>
/// 指示使用的参考系类型是本地还是全局
/// Attribute to specify the reference frame of a member.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class ReferenceFrameAttribute : Attribute
{
    /// <summary>
    /// Gets or sets the reference frame.
    /// </summary>
    public ReferenceFrame Frame { get; set; }

    /// <summary>
    /// Initializes a new instance of the <see cref="ReferenceFrameAttribute"/> class.
    /// </summary>
    /// <param name="frame">The reference frame.</param>
    public ReferenceFrameAttribute(ReferenceFrame frame)
    {
        Frame = frame;
    }
}