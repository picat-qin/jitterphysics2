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

using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.LinearMath;
using Jitter2.UnmanagedMemory;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 球窝约束。<br></br>
/// 此约束将一个物体的参考系中的固定点锚定到另一个物体的参考系中的固定点，从而消除了三个平移自由度。<br></br><br></br>
/// Implements the BallSocket constraint. This constraint anchors a fixed point in the reference frame of
/// one body to a fixed point in the reference frame of another body, eliminating three translational
/// degrees of freedom.
/// </summary>
public unsafe class BallSocket : Constraint
{
    /// <summary>
    /// 球约束数据
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BallSocketData
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
        /// 本地锚点1
        /// </summary>
        public JVector LocalAnchor1;
        /// <summary>
        /// 本地锚点2
        /// </summary>
        public JVector LocalAnchor2;

        public JVector U;
        public JVector R1;
        public JVector R2;

        /// <summary>
        /// 偏差因子
        /// </summary>
        public Real BiasFactor;
        /// <summary>
        /// 柔软值
        /// </summary>
        public Real Softness;

        /// <summary>
        /// 有效质量
        /// </summary>
        public JMatrix EffectiveMass;
        /// <summary>
        /// 累积冲量
        /// </summary>
        public JVector AccumulatedImpulse;
        /// <summary>
        /// 偏移/偏见?
        /// </summary>
        public JVector Bias;
    }

    private JHandle<BallSocketData> handle;

    protected override void Create()
    {
        Trace.Assert(sizeof(BallSocketData) <= sizeof(ConstraintData));

        iterate = &Iterate;
        prepareForIteration = &PrepareForIteration;
        handle = JHandle<ConstraintData>.AsHandle<BallSocketData>(Handle);
    }

    /// <summary>
    /// 初始化约束 <br></br><br></br>
    /// Initializes the constraint.
    /// </summary>
    /// <param name="anchor">
    /// 世界空间中两个物体的锚点。<br></br><br></br>
    /// Anchor point for both bodies in world space.
    /// </param>
    public void Initialize(JVector anchor)
    {
        ref BallSocketData data = ref handle.Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Subtract(anchor, body1.Position, out data.LocalAnchor1);
        JVector.Subtract(anchor, body2.Position, out data.LocalAnchor2);

        JVector.ConjugatedTransform(data.LocalAnchor1, body1.Orientation, out data.LocalAnchor1);
        JVector.ConjugatedTransform(data.LocalAnchor2, body2.Orientation, out data.LocalAnchor2);

        data.BiasFactor = (Real)0.2;
        data.Softness = (Real)0.0;
    }

    /// <summary>
    /// 预迭代
    /// </summary>
    /// <param name="constraint"></param>
    /// <param name="idt"></param>
    public static void PrepareForIteration(ref ConstraintData constraint, Real idt)
    {
        ref BallSocketData data = ref Unsafe.AsRef<BallSocketData>(Unsafe.AsPointer(ref constraint));
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAnchor1, body1.Orientation, out data.R1);
        JVector.Transform(data.LocalAnchor2, body2.Orientation, out data.R2);

        JVector.Add(body1.Position, data.R1, out JVector p1);
        JVector.Add(body2.Position, data.R2, out JVector p2);

        JMatrix cr1 = JMatrix.CreateCrossProduct(data.R1);
        JMatrix cr2 = JMatrix.CreateCrossProduct(data.R2);

        data.EffectiveMass = body1.InverseMass * JMatrix.Identity +
                             JMatrix.Multiply(cr1, JMatrix.MultiplyTransposed(body1.InverseInertiaWorld, cr1)) +
                             body2.InverseMass * JMatrix.Identity +
                             JMatrix.Multiply(cr2, JMatrix.MultiplyTransposed(body2.InverseInertiaWorld, cr2));

        Real softness = data.Softness * idt;

        data.EffectiveMass.M11 += softness;
        data.EffectiveMass.M22 += softness;
        data.EffectiveMass.M33 += softness;

        JMatrix.Inverse(data.EffectiveMass, out data.EffectiveMass);

        data.Bias = (p2 - p1) * data.BiasFactor * idt;

        JVector acc = data.AccumulatedImpulse;

        body1.Velocity -= body1.InverseMass * acc;
        body1.AngularVelocity -= JVector.Transform(JVector.Transform(acc, cr1), body1.InverseInertiaWorld);

        body2.Velocity += body2.InverseMass * acc;
        body2.AngularVelocity += JVector.Transform(JVector.Transform(acc, cr2), body2.InverseInertiaWorld);
    }

    /// <summary>
    /// 柔软值
    /// </summary>
    public Real Softness
    {
        get => handle.Data.Softness;
        set => handle.Data.Softness = value;
    }

    /// <summary>
    /// 偏移值/偏见值
    /// </summary>
    public Real Bias
    {
        get => handle.Data.BiasFactor;
        set => handle.Data.BiasFactor = value;
    }

    /// <summary>
    /// 冲击
    /// </summary>
    public JVector Impulse => handle.Data.AccumulatedImpulse;

    /// <summary>
    /// 迭代
    /// </summary>
    /// <param name="constraint"></param>
    /// <param name="idt"></param>
    public static void Iterate(ref ConstraintData constraint, Real idt)
    {
        ref BallSocketData data = ref Unsafe.AsRef<BallSocketData>(Unsafe.AsPointer(ref constraint));
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JMatrix cr1 = JMatrix.CreateCrossProduct(data.R1);
        JMatrix cr2 = JMatrix.CreateCrossProduct(data.R2);

        JVector softnessVector = data.AccumulatedImpulse * data.Softness * idt;

        JVector jv = -body1.Velocity + JVector.Transform(body1.AngularVelocity, cr1) + body2.Velocity -
                     JVector.Transform(body2.AngularVelocity, cr2);

        JVector lambda = -(Real)1.0 * JVector.Transform(jv + data.Bias + softnessVector, data.EffectiveMass);

        data.AccumulatedImpulse += lambda;

        body1.Velocity -= body1.InverseMass * lambda;
        body1.AngularVelocity -= JVector.Transform(JVector.Transform(lambda, cr1), body1.InverseInertiaWorld);

        body2.Velocity += body2.InverseMass * lambda;
        body2.AngularVelocity += JVector.Transform(JVector.Transform(lambda, cr2), body2.InverseInertiaWorld);
    }
}