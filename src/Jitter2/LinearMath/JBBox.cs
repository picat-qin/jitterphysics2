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
using System.Runtime.InteropServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 轴对齐包围盒 <br></br><br></br>
/// 表示一个轴对齐的包围盒（AABB），一个矩形包围盒，其边缘与坐标轴平行。<br></br><br></br>
/// Represents an axis-aligned bounding box (AABB), a rectangular bounding box whose edges are parallel to the coordinate axes.
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 6*sizeof(Real))]
public struct JBBox : IEquatable<JBBox>
{
    /// <summary>
    /// 处理精度
    /// </summary>
    public const Real Epsilon = (Real)1e-12;

    /// <summary>
    /// 关系描述
    /// </summary>
    public enum ContainmentType
    {
        /// <summary>
        /// 不相交
        /// </summary>
        Disjoint,
        /// <summary>
        /// 包含
        /// </summary>
        Contains,
        /// <summary>
        /// 相交
        /// </summary>
        Intersects
    }

    /// <summary>
    /// 最小点
    /// </summary>
    [FieldOffset(0*sizeof(Real))]
    public JVector Min;

    /// <summary>
    /// 最大点
    /// </summary>
    [FieldOffset(3*sizeof(Real))]
    public JVector Max;

    /// <summary>
    /// 大盒子
    /// </summary>
    public static readonly JBBox LargeBox;

    /// <summary>
    /// 小盒子
    /// </summary>
    public static readonly JBBox SmallBox;

    static JBBox()
    {
        LargeBox.Min = new JVector(Real.MinValue);
        LargeBox.Max = new JVector(Real.MaxValue);
        SmallBox.Min = new JVector(Real.MaxValue);
        SmallBox.Max = new JVector(Real.MinValue);
    }

    /// <summary>
    /// Returns a string representation of the <see cref="JBBox"/>.
    /// </summary>
    public override string ToString()
    {
        return $"Min={{{Min}}}, Max={{{Max}}}";
    }

    public JBBox(JVector min, JVector max)
    {
        Min = min;
        Max = max;
    }

    internal void InverseTransform(ref JVector position, ref JMatrix orientation)
    {
        JVector.Subtract(Max, position, out Max);
        JVector.Subtract(Min, position, out Min);

        JVector.Add(Max, Min, out JVector center);
        center.X *= (Real)0.5;
        center.Y *= (Real)0.5;
        center.Z *= (Real)0.5;

        JVector.Subtract(Max, Min, out JVector halfExtents);
        halfExtents.X *= (Real)0.5;
        halfExtents.Y *= (Real)0.5;
        halfExtents.Z *= (Real)0.5;

        JVector.TransposedTransform(center, orientation, out center);

        JMatrix.Absolute(orientation, out JMatrix abs);
        JVector.TransposedTransform(halfExtents, abs, out halfExtents);

        JVector.Add(center, halfExtents, out Max);
        JVector.Subtract(center, halfExtents, out Min);
    }

    /// <summary>
    /// 转换, 变换
    /// </summary>
    /// <param name="orientation"></param>
    public void Transform(ref JMatrix orientation)
    {
        JVector halfExtents = (Real)0.5 * (Max - Min);
        JVector center = (Real)0.5 * (Max + Min);

        JVector.Transform(center, orientation, out center);

        JMatrix.Absolute(orientation, out var abs);
        JVector.Transform(halfExtents, abs, out halfExtents);

        Max = center + halfExtents;
        Min = center - halfExtents;
    }

    private bool Intersect1D(Real start, Real dir, Real min, Real max,
        ref Real enter, ref Real exit)
    {
        if (dir * dir < Epsilon * Epsilon) return start >= min && start <= max;

        Real t0 = (min - start) / dir;
        Real t1 = (max - start) / dir;

        if (t0 > t1)
        {
            (t0, t1) = (t1, t0);
        }

        if (t0 > exit || t1 < enter) return false;

        if (t0 > enter) enter = t0;
        if (t1 < exit) exit = t1;
        return true;
    }

    /// <summary>
    /// 线段相交
    /// </summary>
    /// <param name="origin">原点</param>
    /// <param name="direction">方向</param>
    /// <returns></returns>
    public bool SegmentIntersect(in JVector origin, in JVector direction)
    {
        Real enter = (Real)0.0, exit = (Real)1.0;

        if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            return false;

        return true;
    }

    /// <summary>
    /// 射线相交
    /// </summary>
    /// <param name="origin">原点</param>
    /// <param name="direction">方向</param>
    /// <returns></returns>
    public bool RayIntersect(in JVector origin, in JVector direction)
    {
        Real enter = (Real)0.0, exit = Real.MaxValue;

        if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            return false;

        return true;
    }

    /// <summary>
    /// 射线相交
    /// </summary>
    /// <param name="origin">原点</param>
    /// <param name="direction">方向</param>
    /// <param name="enter">进入的长度?</param>
    /// <returns></returns>
    public bool RayIntersect(in JVector origin, in JVector direction, out Real enter)
    {
        enter = (Real)0.0;
        Real exit = Real.MaxValue;

        if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            return false;

        return true;
    }

    /// <summary>
    /// 获取和一个点的关系
    /// </summary>
    /// <param name="point"></param>
    /// <returns></returns>
    public ContainmentType Contains(in JVector point)
    {
        return Min.X <= point.X && point.X <= Max.X &&
               Min.Y <= point.Y && point.Y <= Max.Y &&
               Min.Z <= point.Z && point.Z <= Max.Z
            ? ContainmentType.Contains
            : ContainmentType.Disjoint;
    }

    /// <summary>
    /// 获取边角
    /// </summary>
    /// <param name="corners">边角向量数组引用, 共8个边角</param>
    public void GetCorners(JVector[] corners)
    {
        corners[0].Set(Min.X, Max.Y, Max.Z);
        corners[1].Set(Max.X, Max.Y, Max.Z);
        corners[2].Set(Max.X, Min.Y, Max.Z);
        corners[3].Set(Min.X, Min.Y, Max.Z);
        corners[4].Set(Min.X, Max.Y, Min.Z);
        corners[5].Set(Max.X, Max.Y, Min.Z);
        corners[6].Set(Max.X, Min.Y, Min.Z);
        corners[7].Set(Min.X, Min.Y, Min.Z);
    }

    /// <summary>
    /// 添加点
    /// </summary>
    /// <param name="point"></param>
    public void AddPoint(in JVector point)
    {
        JVector.Max(Max, point, out Max);
        JVector.Min(Min, point, out Min);
    }

    /// <summary>
    /// 通过点集创建 AABB 包盒
    /// </summary>
    /// <param name="points"></param>
    /// <returns></returns>
    public static JBBox CreateFromPoints(JVector[] points)
    {
        JVector vector3 = new JVector(Real.MaxValue);
        JVector vector2 = new JVector(Real.MinValue);

        for (int i = 0; i < points.Length; i++)
        {
            JVector.Min(vector3, points[i], out vector3);
            JVector.Max(vector2, points[i], out vector2);
        }

        return new JBBox(vector3, vector2);
    }

    /// <summary>
    /// 获取与一个 AABB 包盒的关系
    /// </summary>
    /// <param name="box"></param>
    /// <returns></returns>
    public readonly ContainmentType Contains(in JBBox box)
    {
        ContainmentType result = ContainmentType.Disjoint;
        if (Max.X >= box.Min.X && Min.X <= box.Max.X && Max.Y >= box.Min.Y && Min.Y <= box.Max.Y &&
            Max.Z >= box.Min.Z && Min.Z <= box.Max.Z)
        {
            result = Min.X <= box.Min.X && box.Max.X <= Max.X && Min.Y <= box.Min.Y && box.Max.Y <= Max.Y &&
                     Min.Z <= box.Min.Z && box.Max.Z <= Max.Z
                ? ContainmentType.Contains
                : ContainmentType.Intersects;
        }

        return result;
    }

    /// <summary>
    /// 是否与一个 AABB 包盒相交
    /// </summary>
    /// <param name="box"></param>
    /// <returns></returns>
    public readonly bool NotDisjoint(in JBBox box)
    {
        return Max.X >= box.Min.X && Min.X <= box.Max.X && Max.Y >= box.Min.Y && Min.Y <= box.Max.Y &&
               Max.Z >= box.Min.Z && Min.Z <= box.Max.Z;
    }

    /// <summary>
    /// 是否与一个 AABB 包盒相交不相交
    /// </summary>
    /// <param name="box"></param>
    /// <returns></returns>
    public readonly bool Disjoint(in JBBox box)
    {
        return !(Max.X >= box.Min.X && Min.X <= box.Max.X && Max.Y >= box.Min.Y && Min.Y <= box.Max.Y &&
                 Max.Z >= box.Min.Z && Min.Z <= box.Max.Z);
    }

    /// <summary>
    /// 是否包括一个 AABB 包盒
    /// </summary>
    /// <param name="box"></param>
    /// <returns></returns>
    public readonly bool Encompasses(in JBBox box)
    {
        return Min.X <= box.Min.X && Max.X >= box.Max.X &&
               Min.Y <= box.Min.Y && Max.Y >= box.Max.Y &&
               Min.Z <= box.Min.Z && Max.Z >= box.Max.Z;
    }

    /// <summary>
    /// 合并两个 AABB 包盒
    /// </summary>
    /// <param name="original">源 AABB 包盒</param>
    /// <param name="additional">添加的 AABB 包盒</param>
    /// <returns></returns>
    public static JBBox CreateMerged(in JBBox original, in JBBox additional)
    {
        CreateMerged(original, additional, out JBBox result);
        return result;
    }

    /// <summary>
    /// 合并两个 AABB 包盒
    /// </summary>
    /// <param name="original">源 AABB 包盒</param>
    /// <param name="additional">添加的 AABB 包盒</param>
    /// <param name="result">结果 AABB 包盒</param>
    public static void CreateMerged(in JBBox original, in JBBox additional, out JBBox result)
    {
        JVector.Min(original.Min, additional.Min, out result.Min);
        JVector.Max(original.Max, additional.Max, out result.Max);
    }

    /// <summary>
    /// 中心点
    /// </summary>
    public readonly JVector Center => (Min + Max) * ((Real)(1.0 / 2.0));

    /// <summary>
    /// 获取体积
    /// </summary>
    /// <returns></returns>
    public Real GetVolume()
    {
        JVector len = Max - Min;
        return len.X * len.Y * len.Z;
    }

    /// <summary>
    /// 获取表面积
    /// </summary>
    /// <returns></returns>
    public Real GetSurfaceArea()
    {
        JVector len = Max - Min;
        return (Real)2.0 * (len.X * len.Y + len.Y * len.Z + len.Z * len.X);
    }

    public bool Equals(JBBox other)
    {
        return Min.Equals(other.Min) && Max.Equals(other.Max);
    }

    public override bool Equals(object? obj)
    {
        return obj is JBBox other && Equals(other);
    }

    public override int GetHashCode()
    {
        return Min.GetHashCode() ^ Max.GetHashCode();
    }
}