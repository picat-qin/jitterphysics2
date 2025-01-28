using System;
using System.Collections.Generic;
using Jitter2;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace JitterDemo;

/// <summary>
/// 体素网格
/// </summary>
public class VoxelGrid
{
    /// <summary>
    /// 大小
    /// </summary>
    public const int Size = 100;
    private readonly World world;
    /// <summary>
    /// 体素集
    /// </summary>
    public HashSet<int> Voxels = new();

    public VoxelGrid(World world)
    {
        this.world = world;
    }

    /// <summary>
    /// 获取临近体素
    /// </summary>
    /// <param name="index">临近体素的索引</param>
    /// <returns></returns>
    public uint GetNeighbours(int index)
    {
        uint result = 0;

        if (Voxels.Contains(index + 1)) result |= 1;
        if (Voxels.Contains(index - 1)) result |= 2;
        if (Voxels.Contains(index + Size)) result |= 4;
        if (Voxels.Contains(index - Size)) result |= 8;
        if (Voxels.Contains(index + Size * Size)) result |= 16;
        if (Voxels.Contains(index - Size * Size)) result |= 32;

        return result;
    }

    /// <summary>
    /// 从索引获取位置
    /// </summary>
    /// <param name="index"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentOutOfRangeException"></exception>
    public JVector PositionFromIndex(int index)
    {
        if (index < 0 || index >= Size * Size * Size)
        {
            throw new ArgumentOutOfRangeException();
        }

        int z = index / (Size * Size);
        int y = (index - z * (Size * Size)) / Size;
        int x = index - z * (Size * Size) - y * Size;
        return new JVector(x, y, z);
    }

    /// <summary>
    /// 主体
    /// </summary>
    public RigidBody? Body { get; }

    /// <summary>
    /// 添加体素
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentOutOfRangeException"></exception>
    public bool AddVoxel(int x, int y, int z)
    {
        if (x < 0 || x >= Size || y < 0 || y >= Size || z < 0 || z >= Size)
        {
            throw new ArgumentOutOfRangeException();
        }

        return Voxels.Add(x + y * Size + z * Size * Size);
    }
}