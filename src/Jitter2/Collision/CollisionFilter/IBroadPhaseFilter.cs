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

// ��ײ
namespace Jitter2.Collision;

/// <summary>
/// ����ʵ��ͨ�ù������Ľӿڣ����ų��� Jitter ����ײϵͳ�ܵ��в�Ӧ���ǵ��ض���״�ԡ�<br></br>
/// �����������<see cref="World.BroadPhaseFilter"/><br></br><br></br>
/// Interface for implementing a generic filter to exclude specific pairs of shapes <br></br>
/// that should not be considered in the collision system pipeline of Jitter.<br></br>
/// Refer to <see cref="World.BroadPhaseFilter"/> for more details.
/// </summary>
public interface IBroadPhaseFilter
{
    /// <summary>
    /// ���˵���Ӧ�����Ӵ�����״�ԡ�<br></br>
    /// Filters out pairs of shapes that should not generate contacts.
    /// </summary>
    /// <returns>
    /// ���Ӧ���˵���ײ����Ϊ false ������Ϊ true ��<br></br>
    /// False if the collision should be filtered out; true otherwise.
    /// </returns>
    bool Filter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB);
}