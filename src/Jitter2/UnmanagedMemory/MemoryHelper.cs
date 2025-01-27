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

using System.Runtime.InteropServices;

namespace Jitter2.UnmanagedMemory;

/// <summary>
/// 内存帮助类
/// </summary>
public static unsafe class MemoryHelper
{
    /// <summary>
    /// 一个内存块，其大小相当于 6 个 <see cref="Real"/> 类型的实例。<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// 该结构体使用连续布局和固定大小，以确保一致的内存对齐和布局。<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 6 * sizeof(Real))]
    public struct MemBlock6Real { }

    /// <summary>
    /// 一个内存块，其大小相当于 9 个 <see cref="Real"/> 类型的实例。<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// 该结构体使用连续布局和固定大小，以确保一致的内存对齐和布局。<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 9 * sizeof(Real))]
    public struct MemBlock9Real { }

    /// <summary>
    /// 一个内存块，其大小相当于 12 个 <see cref="Real"/> 类型的实例。<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// 该结构体使用连续布局和固定大小，以确保一致的内存对齐和布局。<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 12 * sizeof(Real))]
    public struct MemBlock12Real { }

    /// <summary>
    /// 一个内存块，其大小相当于 16 个 <see cref="Real"/> 类型的实例。<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// 该结构体使用连续布局和固定大小，以确保一致的内存对齐和布局。<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 16 * sizeof(Real))]
    public struct MemBlock16Real { }

    /// <summary>
    ///  为指定类型的数组分配一块非托管内存。<br></br><br></br>
    /// Allocates a block of unmanaged memory for an array of the specified type.
    /// </summary>
    /// <typeparam name="T">
    /// 要为其分配内存的元素的非托管类型。<br></br><br></br>
    /// The unmanaged type of the elements to allocate memory for.</typeparam>
    /// <param name="num">
    /// 要分配内存的元素数量。<br></br><br></br>
    /// The number of elements to allocate memory for.</param>
    /// <returns>
    /// 指向分配的内存块的指针。<br></br><br></br>
    /// A pointer to the allocated memory block.
    /// </returns>
    public static T* AllocateHeap<T>(int num) where T : unmanaged
    {
        return (T*)AllocateHeap(num * sizeof(T));
    }

    /// <summary>
    /// 释放先前为指定类型的数组分配的未管理内存块。<br></br><br></br>
    /// Frees a block of unmanaged memory previously allocated for an array of the specified type.
    /// </summary>
    /// <typeparam name="T">
    /// 内存块中元素的 unmanaged 类型。<br></br><br></br>
    /// The unmanaged type of the elements in the memory block.</typeparam>
    /// <param name="ptr">
    /// 指向要释放的内存块的指针。<br></br><br></br>
    /// A pointer to the memory block to free.</param>
    public static void Free<T>(T* ptr) where T : unmanaged
    {
        Free((void*)ptr);
    }

    /// <summary>
    /// 分配指定长度的字节数块非托管内存。<br></br><br></br>
    /// Allocates a block of unmanaged memory of the specified length in bytes.
    /// </summary>
    /// <param name="len">要分配的内存块的长度，以字节为单位。<br></br><br></br>
    /// The length of the memory block to allocate, in bytes.</param>
    /// <returns>
    /// 指向分配的内存块的指针。<br></br><br></br>
    /// A pointer to the allocated memory block.</returns>
    public static void* AllocateHeap(int len) => NativeMemory.Alloc((nuint)len);

    /// <summary>
    /// 释放之前分配的一组非托管内存。<br></br><br></br>
    /// Frees a block of unmanaged memory previously allocated.
    /// </summary>
    /// <param name="ptr">
    /// 指向要释放的内存块的指针。<br></br><br></br>
    /// A pointer to the memory block to free.</param>
    public static void Free(void* ptr) => NativeMemory.Free(ptr);

    /// <summary>
    /// 清除非托管内存。<br></br><br></br>
    /// Zeros out unmanaged memory.
    /// </summary>
    /// <param name="buffer">
    /// 一个指向要清零的内存块的指针。<br></br><br></br>
    /// A pointer to the memory block to zero out.</param>
    /// <param name="len">
    /// 要清零的内存块的长度，以字节为单位。<br></br>
    /// The length of the memory block to zero out, in bytes.</param>
    public static void MemSet(void* buffer, int len)
    {
        for (int i = 0; i < len; i++)
        {
            *((byte*)buffer + i) = 0;
        }
    }
}