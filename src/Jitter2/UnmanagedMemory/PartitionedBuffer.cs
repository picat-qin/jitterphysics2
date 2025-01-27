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
using System.Runtime.CompilerServices;
using Jitter2.Parallelization;

namespace Jitter2.UnmanagedMemory;

/// <summary>
/// ���йܶ���ľ����<br></br><br></br>
/// Handle for an unmanaged object.
/// </summary>
public unsafe struct JHandle<T> where T : unmanaged
{
    /// <summary>
    /// �վ��
    /// </summary>
    public static readonly JHandle<T> Zero = new(null);

    internal T** Pointer;

    /// <summary>
    /// ����
    /// </summary>
    public ref T Data => ref Unsafe.AsRef<T>(*Pointer);

    internal JHandle(T** ptr)
    {
        Pointer = ptr;
    }

    /// <summary>
    /// �Ƿ��ǿվ��
    /// </summary>
    public bool IsZero => Pointer == null;

    /// <summary>
    /// ��Ϊ���
    /// </summary>
    /// <typeparam name="K"></typeparam>
    /// <param name="handle"></param>
    /// <returns></returns>
    public static JHandle<K> AsHandle<K>(JHandle<T> handle) where K : unmanaged
    {
        return new JHandle<K>((K**)handle.Pointer);
    }
}

/// <summary>
/// ���������� <br></br><br></br>
/// ������йܽṹ���ڴ棬�����ǰ�˳��洢���������ڴ���С�ÿ���ṹ�����ǻ��ǻ��<br></br><br></br>
/// Manages memory for unmanaged structs, storing them sequentially in contiguous memory blocks. Each struct can either be active or inactive.
/// </summary>
public sealed unsafe class PartitionedBuffer<T> : IDisposable where T : unmanaged
{
    // this is a mixture of a data structure and an allocator.

    // layout:
    // 0 [ .... ] active [ .... ] count [ .... ] size
    private T* memory;
    private T** handles;

    private int activeCount;

    private int size;

    private bool disposed;

    private readonly int maximumSize;

    /// <summary>
    /// ����
    /// </summary>
    public int Count { get; private set; }

    /// <summary>
    /// ��ʼ������µ�ʵ����<br></br><br></br>
    /// Initializes a new instance of the class.
    /// </summary>
    /// <param name="maximumSize">
    /// �˽ṹ�п����ɵ����Ԫ������. <br></br>
    /// ��<see cref="Allocate"/>����ȷ����<br></br>
    /// Ԥ������ڴ����Ϊ���Size��IntPtr.Size�����ֽ�Ϊ��λ���ĳ˻���<br></br><br></br>
    /// The maximum number of elements that can be accommodated within this structure, 
    /// as determined by the <see cref="Allocate"/> method. The preallocated memory 
    /// is calculated as the product of maximumSize and IntPtr.Size (in bytes).</param>
    /// <param name="initialSize">
    /// �����ڴ��ĳ�ʼ��С����Ԫ��������ʾ��Ĭ��ֵΪ1024��<br></br><br></br>
    /// The initial size of the contiguous memory block, denoted in the number of elements. The default value is 1024.</param>
    public PartitionedBuffer(int maximumSize, int initialSize = 1024)
    {
        if (maximumSize < initialSize) initialSize = maximumSize;

        size = initialSize;
        this.maximumSize = maximumSize;

        memory = (T*)MemoryHelper.AllocateHeap(size * sizeof(T));
        handles = (T**)MemoryHelper.AllocateHeap(maximumSize * sizeof(IntPtr));

        for (int i = 0; i < size; i++)
        {
            Unsafe.AsRef<int>(&memory[i]) = i;
        }
    }

    /// <summary>
    /// �������ֽ�Ϊ��λ����ķ��й��ڴ����������<br></br><br></br>
    /// Returns the total amount of unmanaged memory allocated in bytes.
    /// </summary>
    public long TotalBytesAllocated => size * sizeof(T) + maximumSize * sizeof(IntPtr);

    /// <summary>
    /// �����ݽṹ��ɾ�������Ľṹ��<br></br><br></br>
    /// Removes the associated native structure from the data structure.
    /// </summary>
    public void Free(JHandle<T> handle)
    {
        Debug.Assert(!disposed);

        MoveToInactive(handle);

        Count -= 1;
        (**handle.Pointer, memory[Count]) = (memory[Count], **handle.Pointer);

        handles[Unsafe.Read<int>(*handle.Pointer)] = *handle.Pointer;
        handles[Unsafe.Read<int>(&memory[Count])] = &memory[Count];

        handle.Pointer = (T**)0;
    }

    /// <summary>
    /// ���б��Ϊ���Ԫ���ڴ�Ρ�<br></br><br></br>
    /// A span for all elements marked as active.
    /// </summary>
    public Span<T> Active => new(memory, activeCount);

    /// <summary>
    /// ���б��Ϊ�����Ԫ�ص��ڴ�Ρ�<br></br><br></br>
    /// A span for all elements marked as inactive.
    /// </summary>
    public Span<T> Inactive => new(&memory[activeCount], Count - activeCount);

    /// <summary>
    /// ����Ԫ�ص��ڴ��. <br></br><br></br>
    /// A span for all elements.
    /// </summary>
    public Span<T> Elements => new(memory, Count);

    /// <summary>
    /// ���ض���ľ�����ö������λ�ڴ� <see cref="PartitionedBuffer{T}"/> ʵ���С��˲����� O��1����<br></br><br></br>
    /// Returns the handle of the object. The object has to be in this instance of
    /// <see cref="PartitionedBuffer{T}"/>. This operation is O(1).
    /// </summary>
    public JHandle<T> GetHandle(ref T t)
    {
        return new JHandle<T>(&handles[Unsafe.Read<int>(Unsafe.AsPointer(ref t))]);
    }

    /// <summary>
    /// ���Ԫ���Ƿ���Ϊ�Ԫ�ش洢��<br></br><br></br>
    /// �������λ�ڴ� <see cref="PartitionedBuffer{T}"/> ʵ���С��˲����� O��1����<br></br><br></br>
    /// Checks if the element is stored as an active element. The object has to be in this instance
    /// of <see cref="PartitionedBuffer{T}"/>. This operation is O(1).
    /// </summary>
    public bool IsActive(JHandle<T> handle)
    {
        Debug.Assert(*handle.Pointer - memory < Count);
        return (nint)(*handle.Pointer) - (nint)memory < activeCount * sizeof(T);
    }

    /// <summary>
    /// ������ӷǻ״̬�ƶ����״̬��<br></br><br></br>
    /// Moves an object from inactive to active.
    /// </summary>
    public void MoveToActive(JHandle<T> handle)
    {
        Debug.Assert(*handle.Pointer - memory < Count);

        if ((nint)(*handle.Pointer) - (nint)memory < activeCount * sizeof(T)) return;
        (**handle.Pointer, memory[activeCount]) = (memory[activeCount], **handle.Pointer);
        handles[Unsafe.Read<int>(*handle.Pointer)] = *handle.Pointer;
        handles[Unsafe.Read<int>(&memory[activeCount])] = &memory[activeCount];
        activeCount += 1;
    }

    /// <summary>
    /// ������ӻ״̬�ƶ����ǻ״̬��<br></br><br></br>
    /// Moves an object from active to inactive.
    /// </summary>
    public void MoveToInactive(JHandle<T> handle)
    {
        if ((nint)(*handle.Pointer) - (nint)memory >= activeCount * sizeof(T)) return;

        activeCount -= 1;
        (**handle.Pointer, memory[activeCount]) = (memory[activeCount], **handle.Pointer);
        handles[Unsafe.Read<int>(*handle.Pointer)] = *handle.Pointer;
        handles[Unsafe.Read<int>(&memory[activeCount])] = &memory[activeCount];
    }

    /// <summary>
    /// ��д����<br></br><br></br> 
    /// ������������С���� <see cref="PartitionedBuffer{T}.Allocate(bool, bool)"/> ������ʱ����д��������<br></br>
    /// ������Сȷʵ���ƶ����нṹ�����ǵ��ڴ��ַ���ڴ˹�����ʹ�þ����<see cref = "JHandle{T}" />���ǲ���ȫ�Ĳ�����<br></br>
    /// �����<see cref = "Allocate" /> �����˲������ã���ʹ�ö����������ʱ������ݡ�<br></br><br></br>
    /// Reader-writer lock. Locked by a writer when a resize (triggered by <see
    /// cref="PartitionedBuffer{T}.Allocate(bool, bool)"/>) occurs. Resizing does move all structs and
    /// their memory addresses. It is not safe to use handles (<see cref="JHandle{T}"/>) during this
    /// operation. Use a reader lock to access native data if concurrent calls to <see cref="Allocate"/>
    /// are made.
    /// </summary>
    public ReaderWriterLock ResizeLock;

    /// <summary>
    /// Ϊ unmanaged ��������ڴ档<br></br><br></br>
    /// Allocates an unmanaged object.
    /// </summary>
    /// <param name="active">���״̬ <br></br><br></br> The state of the object.</param>
    /// <param name="clear">����д�������ڴ��С�<br></br><br></br> Write zeros into the object's memory.</param>
    /// <returns>A native handle.</returns>
    /// <exception cref="MaximumSizeException">
    /// �����ݽṹ������С���Ʊ�����ʱ�׳���<br></br><br></br>
    /// Raised when the maximum size limit
    /// of the datastructure is exceeded.</exception>
    public JHandle<T> Allocate(bool active = false, bool clear = false)
    {
        Debug.Assert(!disposed);

        JHandle<T> handle;

        if (Count == size)
        {
            ResizeLock.EnterWriteLock();

            int osize = size;

            if (osize == maximumSize)
            {
                throw new MaximumSizeException($"{nameof(PartitionedBuffer<T>)} reached " +
                                               $"its maximum size limit ({nameof(maximumSize)}={maximumSize}).");
            }

            size = Math.Min(2 * osize, maximumSize);

            Trace.WriteLine($"{nameof(PartitionedBuffer<T>)}: " +
                            $"Resizing to {size}x{typeof(T)} ({size}x{sizeof(T)} Bytes).");

            var oldmemory = memory;
            memory = (T*)MemoryHelper.AllocateHeap(size * sizeof(T));

            for (int i = 0; i < osize; i++)
            {
                memory[i] = oldmemory[i];
                handles[Unsafe.Read<int>(&memory[i])] = &memory[i];
            }

            for (int i = osize; i < size; i++)
            {
                Unsafe.AsRef<int>(&memory[i]) = i;
            }

            MemoryHelper.Free(oldmemory);
            ResizeLock.ExitWriteLock();
        }

        int hdl = Unsafe.Read<int>(&memory[Count]);
        handles[hdl] = &memory[Count];

        handle = new JHandle<T>(&handles[hdl]);

        if (clear)
        {
            MemoryHelper.MemSet((byte*)handles[hdl] + sizeof(IntPtr),
                sizeof(T) - sizeof(IntPtr));
        }

        Count += 1;

        if (active) MoveToActive(handle);

        return handle;
    }

    private void FreeResources()
    {
        if (!disposed)
        {
            MemoryHelper.Free(handles);
            handles = (T**)0;

            MemoryHelper.Free(memory);
            memory = (T*)0;

            disposed = true;
        }
    }

    ~PartitionedBuffer()
    {
        FreeResources();
    }

    /// <summary>
    /// ��������ʽ�ͷ����з��й��ڴ档<br></br><br></br>
    /// �⽫ʹ�Ը� <see cref="PartitionedBuffer{T}"/> ���ʵ�����κν�һ��ʹ����Ч��<br></br><br></br>
    /// Call to explicitly free all unmanaged memory. Invalidates any further use of this instance
    /// of <see cref="PartitionedBuffer{T}"/>.
    /// </summary>
    public void Dispose()
    {
        FreeResources();
        GC.SuppressFinalize(this);
    }
}