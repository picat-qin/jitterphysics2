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
using System.Runtime.InteropServices;
using Jitter2.LinearMath;
using Jitter2.UnmanagedMemory;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 圆锥限制约束，限制一个物体相对于另一个物体的倾斜。
/// Implements the ConeLimit constraint, which restricts the tilt of one body relative to
/// another body.
/// </summary>
public unsafe class ConeLimit : Constraint
{
    /// <summary>
    /// 圆锥约束数据
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ConeLimitData
    {
        internal int _internal;
        /// <summary>
        /// 迭代指针
        /// </summary>
        public delegate*<ref ConstraintData, void> Iterate;
        /// <summary>
        /// 预迭代指针
        /// </summary>
        public delegate*<ref ConstraintData, Real, void> PrepareForIteration;

        /// <summary>
        /// 物体1
        /// </summary>
        public JHandle<RigidBodyData> Body1;
        /// <summary>
        /// 物体2
        /// </summary>
        public JHandle<RigidBodyData> Body2;

        /// <summary>
        /// 本地轴
        /// </summary>
        public JVector LocalAxis1, LocalAxis2;

        /// <summary>
        /// 偏差因子
        /// </summary>
        public Real BiasFactor;
        /// <summary>
        /// 柔软因子
        /// </summary>
        public Real Softness;

        /// <summary>
        /// 有效质量
        /// </summary>
        public Real EffectiveMass;
        /// <summary>
        /// 累积冲量
        /// </summary>
        public Real AccumulatedImpulse;
        /// <summary>
        /// 偏移/偏见
        /// </summary>
        public Real Bias;

        /// <summary>
        /// 下限
        /// </summary>
        public Real LimitLow;
        /// <summary>
        /// 上限
        /// </summary>
        public Real LimitHigh;

        /// <summary>
        /// 钳子?
        /// </summary>
        public short Clamp;

        public MemoryHelper.MemBlock6Real J0;
    }

    private JHandle<ConeLimitData> handle;

    protected override void Create()
    {
        Trace.Assert(sizeof(ConeLimitData) <= sizeof(ConstraintData));
        iterate = &Iterate;
        prepareForIteration = &PrepareForIteration;
        handle = JHandle<ConstraintData>.AsHandle<ConeLimitData>(Handle);
    }

    /// <summary>
    /// 初始化 <br></br><br></br>
    /// Initializes the constraint.
    /// </summary>
    /// <param name="axis">The axis in world space.</param>
    public void Initialize(JVector axis, AngularLimit limit)
    {
        ref ConeLimitData data = ref handle.Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        axis.Normalize();

        JVector.ConjugatedTransform(axis, body1.Orientation, out data.LocalAxis1);
        JVector.ConjugatedTransform(axis, body2.Orientation, out data.LocalAxis2);

        data.Softness = (Real)0.001;
        data.BiasFactor = (Real)0.2;

        Real lower = (Real)limit.From;
        Real upper = (Real)limit.To;

        data.LimitLow = MathR.Cos(lower);
        data.LimitHigh = MathR.Cos(upper);
    }

    /// <summary>
    /// 角度
    /// </summary>
    public JAngle Angle
    {
        get
        {
            ref ConeLimitData data = ref handle.Data;

            ref RigidBodyData body1 = ref data.Body1.Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector a1);
            JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector a2);

            return (JAngle)MathR.Acos(JVector.Dot(a1, a2));
        }
    }

    /// <summary>
    /// 预迭代
    /// </summary>
    /// <param name="constraint"></param>
    /// <param name="idt"></param>
    public static void PrepareForIteration(ref ConstraintData constraint, Real idt)
    {
        ref ConeLimitData data = ref Unsafe.AsRef<ConeLimitData>(Unsafe.AsPointer(ref constraint));

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector a1);
        JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector a2);

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 2);

        jacobian[0] = JVector.Cross(a2, a1);
        jacobian[1] = JVector.Cross(a1, a2);

        data.Clamp = 0;

        Real error = JVector.Dot(a1, a2);

        if (error < data.LimitHigh)
        {
            data.Clamp = 1;
            error -= data.LimitHigh;
        }
        else if (error > data.LimitLow)
        {
            data.Clamp = 2;
            error -= data.LimitLow;
        }
        else
        {
            data.AccumulatedImpulse = (Real)0.0;
            return;
        }

        data.EffectiveMass = JVector.Transform(jacobian[0], body1.InverseInertiaWorld) * jacobian[0] +
                             JVector.Transform(jacobian[1], body2.InverseInertiaWorld) * jacobian[1];

        data.EffectiveMass += data.Softness * idt;

        data.EffectiveMass = (Real)1.0 / data.EffectiveMass;

        data.Bias = -error * data.BiasFactor * idt;

        body1.AngularVelocity +=
            JVector.Transform(data.AccumulatedImpulse * jacobian[0], body1.InverseInertiaWorld);

        body2.AngularVelocity +=
            JVector.Transform(data.AccumulatedImpulse * jacobian[1], body2.InverseInertiaWorld);
    }

    /// <summary>
    /// 柔软因子
    /// </summary>
    public Real Softness
    {
        get => handle.Data.Softness;
        set => handle.Data.Softness = value;
    }

    /// <summary>
    /// 偏移/偏见值
    /// </summary>
    public Real Bias
    {
        get => handle.Data.BiasFactor;
        set => handle.Data.BiasFactor = value;
    }

    /// <summary>
    /// 冲击
    /// </summary>
    public Real Impulse => handle.Data.AccumulatedImpulse;

    /// <summary>
    /// 迭代
    /// </summary>
    /// <param name="constraint"></param>
    /// <param name="idt"></param>
    public static void Iterate(ref ConstraintData constraint, Real idt)
    {
        ref ConeLimitData data = ref Unsafe.AsRef<ConeLimitData>(Unsafe.AsPointer(ref constraint));
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        if (data.Clamp == 0) return;

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 2);

        Real jv =
            body1.AngularVelocity * jacobian[0] +
            body2.AngularVelocity * jacobian[1];

        Real softnessScalar = data.AccumulatedImpulse * data.Softness * idt;

        Real lambda = -data.EffectiveMass * (jv + data.Bias + softnessScalar);

        Real oldacc = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse = MathR.Min(data.AccumulatedImpulse, (Real)0.0);
        }
        else
        {
            data.AccumulatedImpulse = MathR.Max(data.AccumulatedImpulse, (Real)0.0);
        }

        lambda = data.AccumulatedImpulse - oldacc;

        body1.AngularVelocity += JVector.Transform(lambda * jacobian[0], body1.InverseInertiaWorld);
        body2.AngularVelocity += JVector.Transform(lambda * jacobian[1], body2.InverseInertiaWorld);
    }
}