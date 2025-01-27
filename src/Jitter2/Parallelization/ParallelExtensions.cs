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
using Jitter2.UnmanagedMemory;

namespace Jitter2.Parallelization;

/// <summary>
/// 并行类拓展方法
/// Provides a ParallelForBatch extension for <see cref="PartitionedBuffer{T}"/> and <see
/// cref="PartitionedBuffer{T}"/>.
/// </summary>
public static class ParallelExtensions
{
    /// <summary>
    /// 批量遍历数组中的元素。<br></br><br></br>
    /// Loop in batches over the elements of an array.
    /// </summary>
    /// <param name="taskThreshold">
    /// 如果元素数量小于此值，则只生成一个批次。<br></br><br></br>
    /// If the number of elements is less than this value, onl one batch is generated.
    /// </param>
    /// <param name="execute">
    /// >如果应调用<see cref="ThreadPool.Execute"/>，则为true。<br></br><br></br>
    /// True if <see cref="ThreadPool.Execute"/> should be called.
    /// </param>
    /// <returns>
    /// 生成的批次（/任务）的数量。<br></br><br></br>
    /// The number of batches(/tasks) generated.
    /// </returns>
    public static int ParallelForBatch<T>(this Array array, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : unmanaged
    {
        int numTasks = array.Length / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, array.Length, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 批量遍历 <see cref="PartitionedBuffer{T}"/> 的活动元素。<br></br><br></br>
    /// Loop in batches over the active elements of the <see cref="PartitionedBuffer{T}"/>.
    /// </summary>
    /// <param name="taskThreshold">
    /// 如果元素数量小于此值，则只 生成一个批次。<br></br><br></br>
    /// If the number of elements is less than this value, only
    /// one batch is generated.</param>
    /// <param name="execute">
    /// 如果应调用<see cref="ThreadPool.Execute"/>，则为true。<br></br><br></br>
    /// True if <see cref="ThreadPool.Execute"/> should be called.</param>
    /// <returns>The number of batches(/tasks) generated.</returns>
    public static int ParallelForBatch<T>(this PartitionedBuffer<T> list, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : unmanaged
    {
        int numTasks = list.Active.Length / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, list.Active.Length, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 批量遍历 <see cref="ReadOnlyPartitionedSet{T}"/> 的活动元素。<br></br><br></br>
    /// Loop in batches over the active elements of the <see cref="ReadOnlyPartitionedSet{T}"/>.
    /// </summary>
    /// <param name="taskThreshold">
    /// 如果元素数量小于此值，则只生成一个批次。<br></br><br></br>
    /// If the number of elements is less than this value, only
    /// one batch is generated.</param>
    /// <param name="execute">
    /// 如果应调用<see cref="ThreadPool.Execute"/>，则为true。<br></br><br></br>
    /// True if <see cref="ThreadPool.Execute"/> should be called.</param>
    /// <returns>生成的批次（/任务）的数量。<br></br><br></br> The number of batches(/tasks) generated.</returns>
    public static int ParallelForBatch<T>(this ReadOnlyPartitionedSet<T> list, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : class, IPartitionedSetIndex
    {
        int numTasks = list.Active / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, list.Active, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 批量遍历 <see cref="PartitionedSet{T}"/> 的活动元素。<br></br><br></br>
    /// Loop in batches over the active elements of the <see cref="PartitionedSet{T}"/>.
    /// </summary>
    /// <param name="taskThreshold">
    /// 如果元素数量小于此值，则只生成一个批次。<br></br><br></br>
    /// If the number of elements is less than this value, only
    /// one batch is generated.</param>
    /// <param name="execute">
    /// 如果应调用<see cref="ThreadPool.Execute"/>，则为true。<br></br><br></br>
    /// True if <see cref="ThreadPool.Execute"/> should be called.</param>
    /// <returns>生成的批次（/任务）的数量。<br></br><br></br> The number of batches(/tasks) generated.</returns>
    internal static int ParallelForBatch<T>(this PartitionedSet<T> partitionedSet, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : class, IPartitionedSetIndex
    {
        int numTasks = partitionedSet.ActiveCount / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, partitionedSet.ActiveCount, numTasks, action, execute);

        return numTasks;
    }

    internal static int ParallelForBatch<T>(this SlimBag<T> list, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : class, IPartitionedSetIndex
    {
        int numTasks = list.Count / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, list.Count, numTasks, action, execute);

        return numTasks;
    }
}