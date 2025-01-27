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
using System.Diagnostics;

namespace Jitter2.Parallelization;

/// <summary>
/// 并行类 <br></br><br></br>
/// 包含用于Jitter Physics引擎内并行化的方法和结构。<br></br><br></br>
/// Contains methods and structures used for parallelization within the Jitter Physics engine.
/// </summary>
public static class Parallel
{
    /// <summary>
    /// 批 <br></br><br></br>
    /// 表示由起始索引、结束索引和批次索引定义的批次。<br></br>
    /// 这个结构体用于 <see cref="ForBatch"/>，以便在 for 循环中实现多线程的批量处理。<br></br><br></br>
    /// Represents a batch defined by a start index, an end index, and a batch index.
    /// This struct is utilized in <see cref="ForBatch"/> to facilitate multi-threaded batch processing within a for-loop.
    /// </summary>
    public readonly struct Batch
    {
        public Batch(int start, int end, ushort index = 0)
        {
            Start = start;
            End = end;
        }

        public override string ToString()
        {
            return $"Batch(Start: {Start}, End: {End})";
        }

        public readonly int Start;
        public readonly int End;
    }

    /// <summary>
    /// 获取约束 <br></br><br></br>
    /// 考虑到总元素数、分割次数和一个特定的部分索引，<br></br>
    /// 该方法计算该部分的开始和结束索引。<br></br><br></br>
    /// Given the total number of elements, the number of divisions, and a specific part index,
    /// this method calculates the start and end indices for that part.
    /// </summary>
    /// <param name="numElements">要分割的总元素数。 <br></br><br></br> The total number of elements to be divided.</param>
    /// <param name="numDivisions">将元素分割成多少个部分。<br></br><br></br> The number of divisions to split the elements into.</param>
    /// <param name="part">特定部分的索引（0起）。<br></br><br></br> The index of the specific part (0-based).</param>
    /// <param name="start">计算出的指定部分的起始索引 <br></br><br></br> The calculated start index for the specified part (output parameter).</param>
    /// <param name="end">计算指定部分的结束索引 <br></br><br></br> The calculated end index for the specified part (output parameter).</param>
    /// <example>
    /// For numElements = 14, numDivisions = 4, the parts are divided as follows:
    /// - Part 0: start = 0, end = 4
    /// - Part 1: start = 4, end = 8
    /// - Part 2: start = 8, end = 11
    /// - Part 3: start = 11, end = 14
    /// </example>
    public static void GetBounds(int numElements, int numDivisions, int part, out int start, out int end)
    {
        Debug.Assert(part < numDivisions);

        int div = Math.DivRem(numElements, numDivisions, out int mod);

        start = div * part + Math.Min(part, mod);
        end = start + div + (part < mod ? 1 : 0);
    }

    private static readonly ThreadPool threadPool = ThreadPool.Instance;

    /// <summary>
    ///  通过使用 <see cref="ThreadPool"/> 将工作分成批次，并行执行任务。<br></br><br></br>
    /// Executes tasks in parallel by dividing the work into batches using the <see cref="ThreadPool"/>.
    /// </summary>
    /// <param name="lower">要处理的范围的包含下限。<br></br><br></br> The inclusive lower bound of the range to be processed.</param>
    /// <param name="upper">要处理的范围的独占上限。<br></br><br></br> The exclusive upper bound of the range to be processed.</param>
    /// <param name="numTasks">将工作分成多少批的数量。<br></br><br></br> The number of batches to divide the work into.</param>
    /// <param name="action">每个批次执行的回调函数。<br></br><br></br> The callback function to execute for each batch.</param>
    /// <param name="execute">指示是否在将任务添加到线程池后立即执行它们。<br></br><br></br> Indicates whether to execute the tasks immediately after adding them to the thread pool.</param>
    /// <remarks>
    /// 此方法将范围 [lower, upper） 拆分为 <paramref name="numTasks"/> 个批次，并行处理每个批次。<br></br>
    /// 对于每个批次，都会调用 <paramref name="action"/> 回调，它由 <see cref="Batch"/> 结构表示。<br></br>
    /// 如果 <paramref name="execute"/> 为 true，则该方法将调用 <see cref="ThreadPool.Execute"/> 来开始执行任务。<br></br><br></br>
    /// This method splits the range [lower, upper) into <paramref name="numTasks"/> batches and processes each batch in parallel.
    /// The <paramref name="action"/> callback is invoked for each batch, which is represented by a <see cref="Batch"/> struct.
    /// If <paramref name="execute"/> is true, the method will call <see cref="ThreadPool.Execute"/> to start executing the tasks.
    /// </remarks>
    public static void ForBatch(int lower, int upper, int numTasks, Action<Batch> action, bool execute = true)
    {
        Debug.Assert(numTasks <= ushort.MaxValue);
        for (int i = 0; i < numTasks; i++)
        {
            GetBounds(upper - lower, numTasks, i, out int start, out int end);
            threadPool.AddTask(action, new Batch(start + lower, end + lower, (ushort)i));
        }

        if (execute) threadPool.Execute();
    }
}