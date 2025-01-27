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
using System.Runtime.InteropServices;
using System.Threading;

namespace Jitter2.Collision;

/// <summary>
/// 一个存储 (int, int) 值对的哈希集实现。<br></br>
/// 该实现基于开放寻址。<br></br><br></br>
/// A hash set implementation which stores pairs of (int, int) values.
/// The implementation is based on open addressing.
/// </summary>
internal unsafe class PairHashSet : IEnumerable<PairHashSet.Pair>
{
    /// <summary>
    /// 值对
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public readonly struct Pair
    {
        [FieldOffset(0)] public readonly long ID;

        [FieldOffset(0)] public readonly int ID1;

        [FieldOffset(4)] public readonly int ID2;

        /// <summary>
        /// 零
        /// </summary>
        public static Pair Zero = new Pair();

        public Pair(int id1, int id2)
        {
            if (id1 < id2)
            {
                (ID1, ID2) = (id1, id2);
            }
            else
            {
                (ID1, ID2) = (id2, id1);
            }
        }

        public int GetHash()
        {
            return (ID1 + 2281 * ID2) & 0x7FFFFFFF;
        }
    }

    /// <summary>
    /// 枚举器
    /// </summary>
    public struct Enumerator : IEnumerator<Pair>
    {
        private readonly PairHashSet hashSet;
        private int index = -1;

        public Enumerator(PairHashSet hashSet)
        {
            this.hashSet = hashSet;
        }

        /// <summary>
        /// 当前
        /// </summary>
        public readonly Pair Current => hashSet.Slots[index];

        readonly object IEnumerator.Current => Current;

        public readonly void Dispose()
        {
        }

        public bool MoveNext()
        {
            var slots = hashSet.Slots;

            while (index < slots.Length - 1)
            {
                if (slots[++index].ID != 0) return true;
            }

            return false;
        }

        public void Reset()
        {
            index = -1;
        }
    }

    /// <summary>
    /// 插槽
    /// </summary>
    public Pair[] Slots = Array.Empty<Pair>();
    private int count;

    // 16384*8/1024 KB = 128 KB
    /// <summary>
    /// 最小尺寸
    /// </summary>
    public const int MinimumSize = 16384;
    /// <summary>
    /// 修剪因子
    /// </summary>
    public const int TrimFactor = 8;

    /// <summary>
    /// 数量
    /// </summary>
    public int Count => count;

    private static int PickSize(int size = -1)
    {
        int p2 = MinimumSize;
        while (p2 < size)
        {
            p2 *= 2;
        }

        return p2;
    }

    /// <summary>
    /// 清楚插槽
    /// </summary>
    public void Clear()
    {
        Array.Clear(Slots, 0, Slots.Length);
    }

    public PairHashSet()
    {
        Resize(PickSize());
    }

    /// <summary>
    /// 调整插槽大小
    /// </summary>
    /// <param name="size"></param>
    private void Resize(int size)
    {
        if (Slots.Length == size) return;

        Trace.WriteLine($"PairHashSet: Resizing {Slots.Length} -> {size}");

        var newSlots = new Pair[size];

        for (int i = 0; i < Slots.Length; i++)
        {
            Pair pair = Slots[i];
            if (pair.ID != 0)
            {
                int hash = pair.GetHash();
                int hashIndex = FindSlot(newSlots, hash, pair.ID);
                newSlots[hashIndex] = pair;
            }
        }

        Slots = newSlots;
    }

    private int FindSlot(Pair[] slots, int hash, long id)
    {
        int modder = slots.Length - 1;

        hash &= modder;

        while (true)
        {
            if (slots[hash].ID == 0 || slots[hash].ID == id) return hash;
            hash = (hash + 1) & modder;
        }
    }

    /// <summary>
    /// 是否包含某值对
    /// </summary>
    /// <param name="pair"></param>
    /// <returns></returns>
    public bool Contains(Pair pair)
    {
        int hash = pair.GetHash();
        int hashIndex = FindSlot(Slots, hash, pair.ID);
        return Slots[hashIndex].ID != 0;
    }

    /// <summary>
    /// 添加值对
    /// </summary>
    /// <param name="pair"></param>
    /// <returns></returns>
    public bool Add(Pair pair)
    {
        int hash = pair.GetHash();
        int hashIndex = FindSlot(Slots, hash, pair.ID);

        if (Slots[hashIndex].ID == 0)
        {
            Slots[hashIndex] = pair;
            Interlocked.Increment(ref count);

            if (Slots.Length < 2 * count)
            {
                Resize(PickSize(Slots.Length * 2));
            }

            return true;
        }

        return false;
    }

    private Jitter2.Parallelization.ReaderWriterLock rwLock;

    /// <summary>
    /// 并发添加值对
    /// </summary>
    /// <param name="pair"></param>
    /// <returns></returns>
    public bool ConcurrentAdd(Pair pair)
    {
        int hash = pair.GetHash();

        // Fast path: This is a *huge* optimization for the case of frequent additions
        // of already existing entries. Entirely bypassing any locks or synchronization.
        int fpHashIndex = FindSlot(Slots, hash, pair.ID);
        if (Slots[fpHashIndex].ID != 0) return false;

        rwLock.EnterReadLock();

        fixed (Pair* slotsPtr = Slots)
        {
            while (true)
            {
                var hashIndex = FindSlot(Slots, hash, pair.ID);
                var slotPtr = &slotsPtr[hashIndex];

                if (slotPtr->ID == pair.ID)
                {
                    rwLock.ExitReadLock();
                    return false;
                }

                if (Interlocked.CompareExchange(ref *(long*)slotPtr, pair.ID, 0) != 0)
                {
                    continue;
                }

                Interlocked.Increment(ref count);
                rwLock.ExitReadLock();

                if (Slots.Length < 2 * count)
                {
                    rwLock.EnterWriteLock();

                    // check if another thread already performed a resize.
                    if (Slots.Length < 2 * count)
                    {
                        Resize(PickSize(Slots.Length * 2));
                    }

                    rwLock.ExitWriteLock();
                }

                return true;
            } // while
        } // fixed
    }

    /// <summary>
    /// 移除指定插槽
    /// </summary>
    /// <param name="slot"></param>
    /// <returns></returns>
    public bool Remove(int slot)
    {
        int modder = Slots.Length - 1;

        if (Slots[slot].ID == 0)
        {
            return false;
        }

        int hash_j = slot;

        while (true)
        {
            hash_j = (hash_j + 1) & modder;

            if (Slots[hash_j].ID == 0)
            {
                break;
            }

            int hash_k = Slots[hash_j].GetHash() & modder;

            // https://en.wikipedia.org/wiki/Open_addressing
            if ((hash_j > slot && (hash_k <= slot || hash_k > hash_j)) ||
                (hash_j < slot && hash_k <= slot && hash_k > hash_j))
            {
                Slots[slot] = Slots[hash_j];
                slot = hash_j;
            }
        }

        Slots[slot] = Pair.Zero;
        Interlocked.Decrement(ref count);

        if (Slots.Length > MinimumSize && count * TrimFactor < Slots.Length)
        {
            Resize(PickSize(count * 2));
        }

        return true;
    }

    /// <summary>
    /// 移除指定值对
    /// </summary>
    /// <param name="pair"></param>
    /// <returns></returns>
    public bool Remove(Pair pair)
    {
        int hash = pair.GetHash();
        int hashIndex = FindSlot(Slots, hash, pair.ID);
        return Remove(hashIndex);
    }

    public IEnumerator<Pair> GetEnumerator()
    {
        return new Enumerator(this);
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}