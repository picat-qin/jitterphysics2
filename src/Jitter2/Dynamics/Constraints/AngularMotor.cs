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
/// 角约束, 表示驱动两个轴之间的相对角运动的运动约束，这两个轴固定在各自身体的参考系内。<br></br><br></br>
/// Represents a motor constraint that drives relative angular movement between two axes, which are fixed within the reference frames of their respective bodies.
/// </summary>
public unsafe class AngularMotor : Constraint
{
    /// <summary>
    /// 角驱动数据
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct AngularMotorData
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

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        /// <summary>
        /// 本地轴1
        /// </summary>
        public JVector LocalAxis1;
        /// <summary>
        /// 本地轴2
        /// </summary>
        public JVector LocalAxis2;

        /// <summary>
        /// 速度
        /// </summary>
        public Real Velocity;
        /// <summary>
        /// 最大力
        /// </summary>
        public Real MaxForce;

        /// <summary>
        /// 最大距离?
        /// </summary>
        public Real MaxLambda;

        /// <summary>
        /// 有效质量
        /// </summary>
        public Real EffectiveMass;

        /// <summary>
        /// 累积冲量
        /// </summary>
        public Real AccumulatedImpulse;
    }

    private JHandle<AngularMotorData> handle;

    protected override void Create()
    {
        Trace.Assert(sizeof(AngularMotorData) <= sizeof(ConstraintData));
        iterate = &Iterate;
        prepareForIteration = &PrepareForIteration;
        handle = JHandle<ConstraintData>.AsHandle<AngularMotorData>(Handle);
    }

    /// <summary>
    /// 初始化约束。<br></br><br></br>
    /// Initializes the constraint.
    /// </summary>
    /// <param name="axis1">T
    /// 第一个物体上的轴，在世界空间中定义。<br></br><br></br>
    /// he axis on the first body, defined in world space.
    /// </param>
    /// <param name="axis2">
    /// 第二个物体上的轴，在世界空间中定义。<br></br><br></br>
    /// The axis on the second body, defined in world space.
    /// </param>
    public void Initialize(JVector axis1, JVector axis2)
    {
        ref AngularMotorData data = ref handle.Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        axis1.Normalize();
        axis2.Normalize();

        JVector.ConjugatedTransform(axis1, body1.Orientation, out data.LocalAxis1);
        JVector.ConjugatedTransform(axis2, body2.Orientation, out data.LocalAxis2);

        data.MaxForce = 0;
        data.Velocity = 0;
    }

    /// <summary>
    /// 初始化
    /// </summary>
    /// <param name="axis"></param>
    public void Initialize(JVector axis)
    {
        Initialize(axis, axis);
    }

    /// <summary>
    /// 目标速度
    /// </summary>
    public Real TargetVelocity
    {
        get => handle.Data.Velocity;
        set => handle.Data.Velocity = value;
    }

    /// <summary>
    /// 本地轴1
    /// </summary>
    public JVector LocalAxis1 => handle.Data.LocalAxis1;

    /// <summary>
    /// 本地轴2
    /// </summary>
    public JVector LocalAxis2 => handle.Data.LocalAxis2;

    /// <summary>
    /// 最大力
    /// </summary>
    public Real MaximumForce
    {
        get => handle.Data.MaxForce;
        set
        {
            if (value < (Real)0.0)
            {
                throw new ArgumentException("Maximum force must not be negative.");
            }

            handle.Data.MaxForce = value;
        }
    }

    /// <summary>
    /// 预迭代
    /// </summary>
    /// <param name="constraint"></param>
    /// <param name="idt"></param>
    public static void PrepareForIteration(ref ConstraintData constraint, Real idt)
    {
        ref AngularMotorData data = ref Unsafe.AsRef<AngularMotorData>(Unsafe.AsPointer(ref constraint));

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector j1);
        JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector j2);

        data.EffectiveMass = JVector.Transform(j1, body1.InverseInertiaWorld) * j1 +
                             JVector.Transform(j2, body2.InverseInertiaWorld) * j2;
        data.EffectiveMass = (Real)1.0 / data.EffectiveMass;

        data.MaxLambda = (Real)1.0 / idt * data.MaxForce;

        body1.AngularVelocity -= JVector.Transform(j1 * data.AccumulatedImpulse, body1.InverseInertiaWorld);
        body2.AngularVelocity += JVector.Transform(j2 * data.AccumulatedImpulse, body2.InverseInertiaWorld);
    }

    /// <summary>
    /// 迭代
    /// </summary>
    /// <param name="constraint"></param>
    /// <param name="idt"></param>
    public static void Iterate(ref ConstraintData constraint, Real idt)
    {
        ref AngularMotorData data = ref Unsafe.AsRef<AngularMotorData>(Unsafe.AsPointer(ref constraint));
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector j1);
        JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector j2);

        Real jv = -j1 * body1.AngularVelocity + j2 * body2.AngularVelocity;

        Real lambda = -(jv - data.Velocity) * data.EffectiveMass;

        Real olda = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        data.AccumulatedImpulse = Math.Clamp(data.AccumulatedImpulse, -data.MaxLambda, data.MaxLambda);

        lambda = data.AccumulatedImpulse - olda;

        body1.AngularVelocity -= JVector.Transform(j1 * lambda, body1.InverseInertiaWorld);
        body2.AngularVelocity += JVector.Transform(j2 * lambda, body2.InverseInertiaWorld);
    }
}