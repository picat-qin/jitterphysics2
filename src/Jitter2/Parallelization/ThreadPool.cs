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
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Jitter2.DataStructures;

namespace Jitter2.Parallelization;

/// <summary> 线程池 <br></br><br></br>
/// 管理工作线程，可以运行任意委托<see cref="Action"/> <br></br><br></br>
/// Manages worker threads, which can run arbitrary delegates <see cref="Action"/>
/// multiThreaded.
/// </summary>
public sealed class ThreadPool
{
    private interface ITask
    {
        public void Perform();
    }

    private sealed class Task<T> : ITask
    {
        public Action<T> action = null!;
        public T parameter = default!;

        public void Perform()
        {
            counter = total;
            action(parameter);
        }

        private static readonly List<Task<T>> pool = new(32);

        private static volatile int counter;
        private static volatile int total;

        public static Task<T> GetFree()
        {
            if (counter == 0)
            {
                counter++;
                total++;
                pool.Add(new Task<T>());
            }

            return pool[^counter--];
        }
    }

    /// <summary>
    /// 每个处理器线程数
    /// </summary>
    public const float ThreadsPerProcessor = 0.9f;

    // ManualResetEventSlim performs much better than the regular ManualResetEvent.
    // mainResetEvent.Wait() is a 'fallthrough' for the persistent threading model in Jitter.
    // Here the performance improvement of ManualResetEvent is mostly visible.
    private readonly ManualResetEventSlim mainResetEvent;
    private Thread[] threads = Array.Empty<Thread>();

    private readonly SlimBag<ITask> taskList = new();
    private readonly ConcurrentQueue<ITask> taskQueue = new();

    private static volatile bool running = true;

    private volatile int tasksLeft;
    internal int threadCount;

    private static ThreadPool? instance;

    /// <summary>
    /// 线程数量
    /// Get the number of threads used by the ThreadManager to execute
    /// tasks.
    /// </summary>
    public int ThreadCount => threadCount;

    private ThreadPool()
    {
        threadCount = 0;
        mainResetEvent = new ManualResetEventSlim(true);

        int initialThreadCount = ThreadCountSuggestion;

#if !NET9_0_OR_GREATER
        // .NET versions below 9.0 have a known issue that can cause hangups or freezing
        // when debugging on non-Windows systems. See: https://github.com/dotnet/runtime/pull/95555
        // To avoid this issue, multi-threading is disabled when a debugger is attached on non-Windows systems.
        if (!OperatingSystem.IsWindows() && Debugger.IsAttached)
        {
            System.Diagnostics.Trace.WriteLine(
                "Multi-threading disabled to prevent potential hangups: Debugger attached, " +
                ".NET version < 9.0, non-Windows system detected.");
            initialThreadCount = 1; // Forces single-threading to avoid hangups
        }
#endif

        ChangeThreadCount(initialThreadCount);
    }

    /// <summary>
    /// 线程建议数量
    /// </summary>
    public static int ThreadCountSuggestion => Math.Max((int)(Environment.ProcessorCount * ThreadsPerProcessor), 1);

    /// <summary>
    /// 改变工作线程的数量 <br></br><br></br>
    /// Changes the number of worker threads.
    /// </summary>
    public void ChangeThreadCount(int numThreads)
    {
        if (numThreads == threadCount) return;

        running = false;
        mainResetEvent.Set();

        for (int i = 0; i < threadCount - 1; i++)
        {
            threads[i].Join();
        }

        running = true;
        threadCount = numThreads;

        threads = new Thread[threadCount - 1];

        var initWaitHandle = new AutoResetEvent(false);

        for (int i = 0; i < threadCount - 1; i++)
        {
            threads[i] = new Thread(() =>
            {
                initWaitHandle.Set();
                ThreadProc();
            });

            threads[i].IsBackground = true;
            threads[i].Start();
            initWaitHandle.WaitOne();
        }

        SignalReset();
    }

    /// <summary>
    /// 添加一个任务到队列中, 调用 <see cref="Execute"/> 附加并执行任务 <br></br><br></br>
    /// Add a task to the task queue. Call <see cref="Execute"/> to
    /// execute added tasks.
    /// </summary>
    public void AddTask<T>(Action<T> action, T parameter)
    {
        var instance = Task<T>.GetFree();
        instance.action = action;
        instance.parameter = parameter;
        taskList.Add(instance);
    }

    /// <summary>
    ///  指示 <see cref="ThreadPool"/> 实例是否已初始化 <br></br><br></br>
    /// Indicates whether the <see cref="ThreadPool"/> instance is initialized.
    /// </summary>
    /// <value><c>true</c> 如果已初始化；否则，<c>false</c>。<br></br><br></br> 
    /// <c>true</c> if initialized; otherwise, <c>false</c>.</value>
    public static bool InstanceInitialized => instance != null;

    /// <summary>
    /// 实现单例模式，提供一个ThreadPool的单一实例。<br></br><br></br>
    /// Implements the singleton pattern to provide a single instance of the ThreadPool.
    /// </summary>
    public static ThreadPool Instance
    {
        get
        {
            instance ??= new ThreadPool();
            return instance;
        }
    }

    /// <summary>
    /// 启动任务的执行，或者允许工作线程在连续循环中等待新任务。<br></br><br></br>
    /// Initiates the execution of tasks or allows worker threads to wait for new tasks in a continuous loop.
    /// </summary>
    public void SignalWait()
    {
        mainResetEvent.Set();
    }

    /// <summary>
    /// 指示所有工作线程在完成所有当前任务后暂停。调用<see cref="SignalWait"/>以恢复处理新任务。<br></br><br></br>
    /// Instructs all worker threads to pause after completing all current tasks. Call <see cref="SignalWait"/> to resume processing new tasks.
    /// </summary>
    public void SignalReset()
    {
        mainResetEvent.Reset();
    }

    private void ThreadProc()
    {
        while (running)
        {
            if (taskQueue.TryDequeue(out ITask? result))
            {
                result.Perform();
                Interlocked.Decrement(ref tasksLeft);
            }
            else
            {
                mainResetEvent.Wait();
            }
        }
    }

    /// <summary>
    /// 启动线程池中添加的所有任务的执行。此方法仅在所有任务完成后才返回。<br></br><br></br>
    /// Initiates the execution of all tasks added to the ThreadPool. This method returns only after all tasks have been completed.
    /// </summary>
    public void Execute()
    {
        SignalWait();

        int totalTasks = taskList.Count;
        tasksLeft = totalTasks;

        for (int i = 0; i < totalTasks; i++)
        {
            taskQueue.Enqueue(taskList[i]);
        }

        taskList.Clear();

        while (taskQueue.TryDequeue(out ITask? result))
        {
            result.Perform();
            Interlocked.Decrement(ref tasksLeft);
        }

        while (tasksLeft > 0)
        {
            Thread.SpinWait(1);
        }
    }
}