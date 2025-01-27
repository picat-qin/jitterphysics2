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
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Jitter2.UnmanagedMemory;

namespace Jitter2.DataStructures;

/// <summary>
/// 分区集合索引接口
/// </summary>
public interface IPartitionedSetIndex
{
    /// <summary>
    /// 设置的索引
    /// </summary>
    int SetIndex { get; set; }
}

/// <summary>
/// 只读分区集合
/// </summary>
/// <typeparam name="T"></typeparam>
public readonly struct ReadOnlyPartitionedSet<T> : IEnumerable<T> where T : class, IPartitionedSetIndex
{
    private readonly PartitionedSet<T> partitionedSet;

    internal ReadOnlyPartitionedSet(PartitionedSet<T> partitionedSet)
    {
        this.partitionedSet = partitionedSet;
    }

    /// <summary>
    /// 激活状态
    /// </summary>
    public int Active => partitionedSet.ActiveCount;

    /// <summary>
    /// 集合数量
    /// </summary>
    public int Count => partitionedSet.Count;

    public T this[int i] => partitionedSet[i];

    public bool IsActive(T element)
    {
        return partitionedSet.IsActive(element);
    }

    public PartitionedSet<T>.Enumerator GetEnumerator()
    {
        return new PartitionedSet<T>.Enumerator(partitionedSet);
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}

/// <summary>
/// 表示可分为活动子集和非活动子集的对象集合。<br></br><br></br>
/// Represents a collection of objects that can be partitioned into active and inactive subsets.
/// </summary>
/// <typeparam name="T">
/// 集合中元素的类型，必须实现<see cref="IPartitionedSetIndex"/>。<br></br><br></br>
/// The type of elements in the set, which must implement <see cref="IPartitionedSetIndex"/>.
/// </typeparam>
/// <remarks>
/// 方法 <see cref="Add(T, bool)"/>、<see cref="Remove(T)"/>、<see cref="Is Active(T)"/>、
/// <see cref="MoveToActive(T)"/> 和 <see cref="MoveToInactive(T)"/> 全部以 O(1) 时间复杂度运行。<br></br><br></br>
/// The methods <see cref="Add(T, bool)"/>, <see cref="Remove(T)"/>, <see cref="IsActive(T)"/>,
/// <see cref="MoveToActive(T)"/>, and <see cref="MoveToInactive(T)"/> all operate in O(1) time complexity.
/// </remarks>
public class PartitionedSet<T> : IEnumerable<T> where T : class, IPartitionedSetIndex
{
    /// <summary>
    /// 枚举器
    /// </summary>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly PartitionedSet<T> partitionedSet;
        private int index = -1;

        public Enumerator(PartitionedSet<T> partitionedSet)
        {
            this.partitionedSet = partitionedSet;
        }

        public readonly T Current => (index >= 0 ? partitionedSet[index] : null)!;

        readonly object IEnumerator.Current => Current;

        public readonly void Dispose()
        {
        }

        public bool MoveNext()
        {
            if (index < partitionedSet.Count - 1)
            {
                index++;
                return true;
            }

            return false;
        }

        public void Reset()
        {
            index = -1;
        }
    }

    private T[] elements;

    /// <summary>
    /// 激活状态的数量
    /// </summary>
    public int ActiveCount { get; private set; }

    public PartitionedSet(int initialSize = 1024)
    {
        elements = new T[initialSize];
    }

    public T this[int i] => elements[i];

    public void Clear()
    {
        for (int i = 0; i < Count; i++)
        {
            elements[i].SetIndex = -1;
            elements[i] = null!;
        }

        Count = 0;
        ActiveCount = 0;
    }

    public int Count { get; private set; }

    public Span<T> AsSpan() => this.elements.AsSpan(0, Count);

    public void Add(T element, bool active = false)
    {
        Debug.Assert(element.SetIndex == -1);

        if (Count == elements.Length)
        {
            Array.Resize(ref elements, elements.Length * 2);
        }

        element.SetIndex = Count;
        elements[Count++] = element;

        if (active) MoveToActive(element);
    }

    /// <summary>
    /// 交换
    /// </summary>
    /// <param name="index0"></param>
    /// <param name="index1"></param>
    private void Swap(int index0, int index1)
    {
        (elements[index0], elements[index1]) =
            (elements[index1], elements[index0]);

        elements[index0].SetIndex = index0;
        elements[index1].SetIndex = index1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsActive(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        return (element.SetIndex < ActiveCount);
    }

    public bool MoveToActive(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        if (element.SetIndex < ActiveCount) return false;
        Swap(ActiveCount, element.SetIndex);
        ActiveCount += 1;
        return true;
    }

    /// <summary>
    /// 移动到非活动状态
    /// </summary>
    /// <param name="element"></param>
    /// <returns></returns>
    public bool MoveToInactive(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        if (element.SetIndex >= ActiveCount) return false;
        ActiveCount -= 1;
        Swap(ActiveCount, element.SetIndex);
        return true;
    }

    public void Remove(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        MoveToInactive(element);

        int li = element.SetIndex;

        Count -= 1;

        elements[li] = elements[Count];
        elements[li].SetIndex = li;
        elements[Count] = null!;

        element.SetIndex = -1;
    }

    public Enumerator GetEnumerator()
    {
        return new Enumerator(this);
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}