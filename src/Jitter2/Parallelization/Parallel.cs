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
/// ������ <br></br><br></br>
/// ��������Jitter Physics�����ڲ��л��ķ����ͽṹ��<br></br><br></br>
/// Contains methods and structures used for parallelization within the Jitter Physics engine.
/// </summary>
public static class Parallel
{
    /// <summary>
    /// �� <br></br><br></br>
    /// ��ʾ����ʼ��������������������������������Ρ�<br></br>
    /// ����ṹ������ <see cref="ForBatch"/>���Ա��� for ѭ����ʵ�ֶ��̵߳���������<br></br><br></br>
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
    /// ��ȡԼ�� <br></br><br></br>
    /// ���ǵ���Ԫ�������ָ������һ���ض��Ĳ���������<br></br>
    /// �÷�������ò��ֵĿ�ʼ�ͽ���������<br></br><br></br>
    /// Given the total number of elements, the number of divisions, and a specific part index,
    /// this method calculates the start and end indices for that part.
    /// </summary>
    /// <param name="numElements">Ҫ�ָ����Ԫ������ <br></br><br></br> The total number of elements to be divided.</param>
    /// <param name="numDivisions">��Ԫ�طָ�ɶ��ٸ����֡�<br></br><br></br> The number of divisions to split the elements into.</param>
    /// <param name="part">�ض����ֵ�������0�𣩡�<br></br><br></br> The index of the specific part (0-based).</param>
    /// <param name="start">�������ָ�����ֵ���ʼ���� <br></br><br></br> The calculated start index for the specified part (output parameter).</param>
    /// <param name="end">����ָ�����ֵĽ������� <br></br><br></br> The calculated end index for the specified part (output parameter).</param>
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
    ///  ͨ��ʹ�� <see cref="ThreadPool"/> �������ֳ����Σ�����ִ������<br></br><br></br>
    /// Executes tasks in parallel by dividing the work into batches using the <see cref="ThreadPool"/>.
    /// </summary>
    /// <param name="lower">Ҫ����ķ�Χ�İ������ޡ�<br></br><br></br> The inclusive lower bound of the range to be processed.</param>
    /// <param name="upper">Ҫ����ķ�Χ�Ķ�ռ���ޡ�<br></br><br></br> The exclusive upper bound of the range to be processed.</param>
    /// <param name="numTasks">�������ֳɶ�������������<br></br><br></br> The number of batches to divide the work into.</param>
    /// <param name="action">ÿ������ִ�еĻص�������<br></br><br></br> The callback function to execute for each batch.</param>
    /// <param name="execute">ָʾ�Ƿ��ڽ�������ӵ��̳߳غ�����ִ�����ǡ�<br></br><br></br> Indicates whether to execute the tasks immediately after adding them to the thread pool.</param>
    /// <remarks>
    /// �˷�������Χ [lower, upper�� ���Ϊ <paramref name="numTasks"/> �����Σ����д���ÿ�����Ρ�<br></br>
    /// ����ÿ�����Σ�������� <paramref name="action"/> �ص������� <see cref="Batch"/> �ṹ��ʾ��<br></br>
    /// ��� <paramref name="execute"/> Ϊ true����÷��������� <see cref="ThreadPool.Execute"/> ����ʼִ������<br></br><br></br>
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