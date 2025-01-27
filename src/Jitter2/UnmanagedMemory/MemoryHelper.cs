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
/// �ڴ������
/// </summary>
public static unsafe class MemoryHelper
{
    /// <summary>
    /// һ���ڴ�飬���С�൱�� 6 �� <see cref="Real"/> ���͵�ʵ����<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// �ýṹ��ʹ���������ֺ͹̶���С����ȷ��һ�µ��ڴ����Ͳ��֡�<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 6 * sizeof(Real))]
    public struct MemBlock6Real { }

    /// <summary>
    /// һ���ڴ�飬���С�൱�� 9 �� <see cref="Real"/> ���͵�ʵ����<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// �ýṹ��ʹ���������ֺ͹̶���С����ȷ��һ�µ��ڴ����Ͳ��֡�<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 9 * sizeof(Real))]
    public struct MemBlock9Real { }

    /// <summary>
    /// һ���ڴ�飬���С�൱�� 12 �� <see cref="Real"/> ���͵�ʵ����<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// �ýṹ��ʹ���������ֺ͹̶���С����ȷ��һ�µ��ڴ����Ͳ��֡�<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 12 * sizeof(Real))]
    public struct MemBlock12Real { }

    /// <summary>
    /// һ���ڴ�飬���С�൱�� 16 �� <see cref="Real"/> ���͵�ʵ����<br></br><br></br>
    /// A memory block with a size equivalent to six instances of the <see cref="Real"/> type.
    /// </summary>
    /// <remarks>
    /// �ýṹ��ʹ���������ֺ͹̶���С����ȷ��һ�µ��ڴ����Ͳ��֡�<br></br><br></br>
    /// The struct uses sequential layout and a fixed size to ensure consistent memory alignment and layout.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 16 * sizeof(Real))]
    public struct MemBlock16Real { }

    /// <summary>
    ///  Ϊָ�����͵��������һ����й��ڴ档<br></br><br></br>
    /// Allocates a block of unmanaged memory for an array of the specified type.
    /// </summary>
    /// <typeparam name="T">
    /// ҪΪ������ڴ��Ԫ�صķ��й����͡�<br></br><br></br>
    /// The unmanaged type of the elements to allocate memory for.</typeparam>
    /// <param name="num">
    /// Ҫ�����ڴ��Ԫ��������<br></br><br></br>
    /// The number of elements to allocate memory for.</param>
    /// <returns>
    /// ָ�������ڴ���ָ�롣<br></br><br></br>
    /// A pointer to the allocated memory block.
    /// </returns>
    public static T* AllocateHeap<T>(int num) where T : unmanaged
    {
        return (T*)AllocateHeap(num * sizeof(T));
    }

    /// <summary>
    /// �ͷ���ǰΪָ�����͵���������δ�����ڴ�顣<br></br><br></br>
    /// Frees a block of unmanaged memory previously allocated for an array of the specified type.
    /// </summary>
    /// <typeparam name="T">
    /// �ڴ����Ԫ�ص� unmanaged ���͡�<br></br><br></br>
    /// The unmanaged type of the elements in the memory block.</typeparam>
    /// <param name="ptr">
    /// ָ��Ҫ�ͷŵ��ڴ���ָ�롣<br></br><br></br>
    /// A pointer to the memory block to free.</param>
    public static void Free<T>(T* ptr) where T : unmanaged
    {
        Free((void*)ptr);
    }

    /// <summary>
    /// ����ָ�����ȵ��ֽ�������й��ڴ档<br></br><br></br>
    /// Allocates a block of unmanaged memory of the specified length in bytes.
    /// </summary>
    /// <param name="len">Ҫ������ڴ��ĳ��ȣ����ֽ�Ϊ��λ��<br></br><br></br>
    /// The length of the memory block to allocate, in bytes.</param>
    /// <returns>
    /// ָ�������ڴ���ָ�롣<br></br><br></br>
    /// A pointer to the allocated memory block.</returns>
    public static void* AllocateHeap(int len) => NativeMemory.Alloc((nuint)len);

    /// <summary>
    /// �ͷ�֮ǰ�����һ����й��ڴ档<br></br><br></br>
    /// Frees a block of unmanaged memory previously allocated.
    /// </summary>
    /// <param name="ptr">
    /// ָ��Ҫ�ͷŵ��ڴ���ָ�롣<br></br><br></br>
    /// A pointer to the memory block to free.</param>
    public static void Free(void* ptr) => NativeMemory.Free(ptr);

    /// <summary>
    /// ������й��ڴ档<br></br><br></br>
    /// Zeros out unmanaged memory.
    /// </summary>
    /// <param name="buffer">
    /// һ��ָ��Ҫ������ڴ���ָ�롣<br></br><br></br>
    /// A pointer to the memory block to zero out.</param>
    /// <param name="len">
    /// Ҫ������ڴ��ĳ��ȣ����ֽ�Ϊ��λ��<br></br>
    /// The length of the memory block to zero out, in bytes.</param>
    public static void MemSet(void* buffer, int len)
    {
        for (int i = 0; i < len; i++)
        {
            *((byte*)buffer + i) = 0;
        }
    }
}