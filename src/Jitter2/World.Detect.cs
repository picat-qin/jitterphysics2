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
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2;

//检测
public sealed partial class World
{
    /// <summary>
    /// 凸包交集
    /// </summary>
    private struct ConvexHullIntersection
    {
        private JVector[] manifoldData;

        private int leftCount;
        private int rightCount;
        private int manifoldCount;

        /// <summary>
        /// 流形 A
        /// </summary>
        public Span<JVector> ManifoldA => manifoldData.AsSpan(0, manifoldCount);
        /// <summary>
        /// 流形 B
        /// </summary>
        public Span<JVector> ManifoldB => manifoldData.AsSpan(6, manifoldCount);

        /// <summary>
        /// 数量 ( 流形数量 )
        /// </summary>
        public int Count => manifoldCount;

        private void PushLeft(Span<JVector> left, in JVector v)
        {
            const Real epsilon = (Real)0.001;

            if (leftCount > 0)
            {
                if ((left[0] - v).LengthSquared() < epsilon) return;
            }

            if (leftCount > 1)
            {
                if ((left[leftCount - 1] - v).LengthSquared() < epsilon) return;
            }

            left[leftCount++] = v;
        }

        private void PushRight(Span<JVector> right, in JVector v)
        {
            const Real epsilon = (Real)0.001;

            if (rightCount > 0)
            {
                if ((right[0] - v).LengthSquared() < epsilon) return;
            }

            if (rightCount > 1)
            {
                if ((right[rightCount - 1] - v).LengthSquared() < epsilon) return;
            }

            right[rightCount++] = v;
        }

        public void Reset()
        {
            leftCount = 0;
            rightCount = 0;
            manifoldCount = 0;
        }

        /// <summary>
        /// 构建流形
        /// </summary>
        /// <param name="shapeA"></param>
        /// <param name="shapeB"></param>
        /// <param name="pA"></param>
        /// <param name="pB"></param>
        /// <param name="normal"></param>
        /// <param name="penetration"></param>
        [System.Runtime.CompilerServices.SkipLocalsInit]
        public void BuildManifold(RigidBodyShape shapeA, RigidBodyShape shapeB,
            in JVector pA, in JVector pB, in JVector normal, Real penetration)
        {
            manifoldData ??= new JVector[12];
            Reset();

            static void Support(RigidBodyShape shape, in JVector direction, out JVector v)
            {
                JVector.ConjugatedTransform(direction, shape.RigidBody.Data.Orientation, out JVector tmp);
                shape.SupportMap(tmp, out v);
                JVector.Transform(v, shape.RigidBody.Data.Orientation, out v);
                JVector.Add(v, shape.RigidBody.Data.Position, out v);
            }

            JVector crossVector1 = MathHelper.CreateOrthonormal(normal);
            JVector crossVector2 = normal % crossVector1;

            Span<JVector> left = stackalloc JVector[6];
            Span<JVector> right = stackalloc JVector[6];

            const Real sqrt3Over2 = (Real)0.8660254;

            Span<Real> hexagonVertices = stackalloc Real[]
                {(Real)1,(Real)0, (Real)0.5, sqrt3Over2, -(Real)0.5, sqrt3Over2, -1f,(Real)0, -(Real)0.5, -sqrt3Over2, (Real)0.5, -sqrt3Over2 };

            for (int e = 0; e < 6; e++)
            {
                JVector ptNormal = normal + hexagonVertices[2 * e + 0] * (Real)0.01 * crossVector1 +
                                   hexagonVertices[2 * e + 1] * (Real)0.01 * crossVector2;

                Support(shapeA, ptNormal, out JVector np1);
                PushLeft(left, np1);

                ptNormal.Negate();
                Support(shapeB, ptNormal, out JVector np2);
                PushRight(right, np2);
            }

            Span<JVector> mA = manifoldData.AsSpan(0);
            Span<JVector> mB = manifoldData.AsSpan(6);

            // ---

            if (leftCount > 2)
            {
                for (int e = 0; e < rightCount; e++)
                {
                    JVector p = right[e];
                    JVector a = left[leftCount - 1];
                    JVector b = left[0];

                    JVector cr = (b - a) % (p - a);

                    bool sameSign = true;

                    for (int i = 0; i < leftCount - 1; i++)
                    {
                        a = left[i];
                        b = left[i + 1];

                        JVector cr2 = (b - a) % (p - a);

                        sameSign = JVector.Dot(cr, cr2) > (Real)1e-3;
                        if (!sameSign) break;
                    }

                    if (sameSign)
                    {
                        Real diff = JVector.Dot(p - pA, normal);
                        mB[manifoldCount] = p;
                        mA[manifoldCount++] = p - diff * normal;

                        if (manifoldCount == 6) return;
                    }
                }
            }

            // ---
            if (rightCount > 2)
            {
                for (int e = 0; e < leftCount; e++)
                {
                    JVector p = left[e];
                    JVector a = right[rightCount - 1];
                    JVector b = right[0];

                    JVector cr = (b - a) % (p - a);

                    bool sameSign = true;

                    for (int i = 0; i < rightCount - 1; i++)
                    {
                        a = right[i];
                        b = right[i + 1];

                        JVector cr2 = (b - a) % (p - a);

                        sameSign = JVector.Dot(cr, cr2) > (Real)1e-3;
                        if (!sameSign) break;
                    }

                    if (sameSign)
                    {
                        Real diff = JVector.Dot(p - pB, normal);
                        mA[manifoldCount] = p;
                        mB[manifoldCount++] = p - diff * normal;

                        if (manifoldCount == 6) return;
                    }
                }
            }
        } // BuildManifold
    }

    /// <summary>
    /// 无效的碰撞类型异常
    /// </summary>
    public class InvalidCollisionTypeException : Exception
    {
        /// <summary>
        /// 无效的碰撞类型异常
        /// </summary>
        /// <param name="proxyA"></param>
        /// <param name="proxyB"></param>
        public InvalidCollisionTypeException(Type proxyA, Type proxyB)
            : base($"Don't know how to handle collision between {proxyA} and {proxyB}." +
                   $" Register a BroadPhaseFilter to handle and/or filter out these collision types.")
        {
        }
    }

    /// <summary>
    /// 指定用于碰撞检测的 <see cref="INarrowPhaseFilter"/> 的实现。<br></br><br></br>
    /// 默认实例的类型为<see cref = "TriangleEdgeCollisionFilter" />。<br></br><br></br>
    /// Specifies an implementation of the <see cref="INarrowPhaseFilter"/> to be used in collision detection.
    /// The default instance is of type <see cref="TriangleEdgeCollisionFilter"/>.
    /// </summary>
    public INarrowPhaseFilter? NarrowPhaseFilter { get; set; } = new TriangleEdgeCollisionFilter();

    /// <summary>
    /// 指定用于碰撞检测的<see cref="IBroadPhaseFilter"/>的实现。<br></br><br></br>
    /// 默认值为 null。<br></br><br></br>
    /// Specifies an implementation of the <see cref="IBroadPhaseFilter"/> to be used in collision detection.
    /// The default value is null.
    /// </summary>
    public IBroadPhaseFilter? BroadPhaseFilter { get; set; }

    /// <summary>
    /// 允许为接触的平面表面生成额外的接触。<br></br><br></br>
    /// 传统上，碰撞系统报告两个物体之间的最深碰撞点。
    /// 然后，使用接触缓存生成多个时间步的全接触流形，这可能是不稳定的。<br></br>
    /// 该方法尝试在单个时间步内构建更完整或完整的接触流形。<br></br><br></br>
    /// Enables the generation of additional contacts for flat surfaces that are in contact.
    /// Traditionally, the collision system reports the deepest collision point between two objects.
    /// A full contact manifold is then generated over several time steps using contact caching, which
    /// can be unstable. This method attempts to build a fuller or complete contact manifold within a single time step.
    /// </summary>
    public bool EnableAuxiliaryContactPoints { set; get; } = true;

    /// <summary>
    /// 投机性松弛系数<br></br><br></br>
    /// 推测性接触会减缓物体的速度，使其在单个帧内不会穿透或穿过障碍物。
    /// <see cref="SpeculativeRelaxationFactor"/>用于缩放减速，<br></br>
    /// 范围从0（物体在本帧内立即停止）到1（物体和障碍物在下一个速度积分后刚刚接触）。<br></br>
    /// 低于1的值是优选的，因为剩余的速度可能足以在下一个帧内触发另一个推测性接触。<br></br><br></br>
    /// A speculative contact slows a body down such that it does not penetrate or tunnel through
    /// an obstacle within one frame. The <see cref="SpeculativeRelaxationFactor"/> scales the
    /// slowdown, ranging from 0 (where the body stops immediately during this frame) to 1 (where the body and the
    /// obstacle just touch after the next velocity integration). A value below 1 is preferred, as the leftover velocity
    /// might be sufficient to trigger another speculative contact in the next frame.
    /// </summary>
    public Real SpeculativeRelaxationFactor { get; set; } = (Real)0.9;

    /// <summary>
    /// 投机性速度阈值 <br></br><br></br>
    /// 当朝向障碍物的速度超过阈值时，就会生成推测性接触。<br></br>
    /// 为了防止直径为D的物体通过薄壁，应将此阈值设置为大约D/时间步，<br></br>
    /// 例如，对于单位立方体和时间步长为0.01s。<br></br><br></br>
    /// Speculative contacts are generated when the velocity towards an obstacle exceeds
    /// the threshold value. To prevent bodies with a diameter of D from tunneling through thin walls, this
    /// threshold should be set to approximately D / timestep, e.g., 100 for a unit cube and a
    /// timestep of 0.01s.
    /// </summary>
    public Real SpeculativeVelocityThreshold { get; set; } =(Real)10;

    /// <summary>
    /// 注册接触
    /// </summary>
    /// <param name="arbiter"></param>
    /// <param name="point1"></param>
    /// <param name="point2"></param>
    /// <param name="normal"></param>
    /// <param name="penetration"></param>
    /// <param name="speculative"></param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RegisterContact(Arbiter arbiter, in JVector point1, in JVector point2,
        in JVector normal, Real penetration, bool speculative = false)
    {
        lock (arbiter)
        {
            memContacts.ResizeLock.EnterReadLock();
            arbiter.Handle.Data.IsSpeculative = speculative;
            arbiter.Handle.Data.AddContact(point1, point2, normal, penetration);
            memContacts.ResizeLock.ExitReadLock();
        }
    }

    /// <summary>
    /// 注册接触
    /// </summary>
    /// <param name="id0"></param>
    /// <param name="id1"></param>
    /// <param name="body1"></param>
    /// <param name="body2"></param>
    /// <param name="point1"></param>
    /// <param name="point2"></param>
    /// <param name="normal"></param>
    /// <param name="penetration"></param>
    /// <param name="speculative"></param>
    public void RegisterContact(ulong id0, ulong id1, RigidBody body1, RigidBody body2,
        in JVector point1, in JVector point2, in JVector normal, Real penetration, bool speculative = false)
    {
        GetArbiter(id0, id1, body1, body2, out Arbiter arbiter);
        RegisterContact(arbiter, point1, point2, normal, penetration, speculative);
    }

    [ThreadStatic] private static ConvexHullIntersection cvh;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Detect(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        if (BroadPhaseFilter != null)
        {
            if (!BroadPhaseFilter.Filter(proxyA, proxyB))
            {
                return;
            }
        }

        if (proxyA is not RigidBodyShape sA || proxyB is not RigidBodyShape sB)
        {
            throw new InvalidCollisionTypeException(proxyA.GetType(), proxyB.GetType());
        }

        if (sB.ShapeId < sA.ShapeId)
        {
            (sA, sB) = (sB, sA);
        }

        bool colliding;
        Unsafe.SkipInit(out JVector normal);
        Unsafe.SkipInit(out JVector pA);
        Unsafe.SkipInit(out JVector pB);
        Real penetration;

        Debug.Assert(sA.RigidBody != sB.RigidBody);
        Debug.Assert(sA.RigidBody.World == this);
        Debug.Assert(sB.RigidBody.World == this);

        Debug.Assert(sA.RigidBody != null);
        Debug.Assert(sB.RigidBody != null);

        if (!sA.RigidBody.Data.IsActive && !sB.RigidBody.Data.IsActive) return;
        if (sA.RigidBody.Data.IsStatic && sB.RigidBody.Data.IsStatic) return;

        ref RigidBodyData b1 = ref sA.RigidBody.Data;
        ref RigidBodyData b2 = ref sB.RigidBody.Data;

        bool speculative = sA.RigidBody.EnableSpeculativeContacts || sB.RigidBody.EnableSpeculativeContacts;

        /*
        if (UseFullEPASolver)
        {
            bool success = NarrowPhase.Collision(sA, sB, b1.Orientation, b2.Orientation, b1.Position, b2.Position,
                out pA, out pB, out normal, out penetration);

            if (!success) return;

            colliding = penetration >= (Real)0.0;
        }
        else
        */
        {
            colliding = NarrowPhase.MPREPA(sA, sB, b1.Orientation, b2.Orientation, b1.Position, b2.Position,
                out pA, out pB, out normal, out penetration);
        }

        if (!colliding)
        {
            if (!speculative) return;

            JVector dv = sB.RigidBody.Velocity - sA.RigidBody.Velocity;

            if (dv.LengthSquared() < SpeculativeVelocityThreshold * SpeculativeVelocityThreshold) return;

            bool success = NarrowPhase.Sweep(sA, sB, b1.Orientation, b2.Orientation,
                b1.Position, b2.Position,b1.Velocity, b2.Velocity,
                out pA, out pB, out normal, out Real toi);

            if (!success || toi > step_dt || toi == (Real)0.0) return;

            penetration = normal * (pA - pB) * SpeculativeRelaxationFactor;

            if (NarrowPhaseFilter != null)
            {
                if (!NarrowPhaseFilter.Filter(sA, sB, ref pA, ref pB, ref normal, ref penetration))
                {
                    return;
                }
            }

            GetArbiter(sA.ShapeId, sB.ShapeId, sA.RigidBody, sB.RigidBody, out Arbiter arbiter2);

            lock (arbiter2)
            {
                // (see. 1)
                arbiter2.Handle.Data.IsSpeculative = true;
                memContacts.ResizeLock.EnterReadLock();
                arbiter2.Handle.Data.AddContact(pA, pB, normal, penetration);
                memContacts.ResizeLock.ExitReadLock();
            }

            return;
        }

        cvh.Reset();

        // Auxiliary Flat Surface Contact Points
        //
        if (EnableAuxiliaryContactPoints)
        {
            // We cannot run the NarrowPhaseFilter in advance since it
            // may modify normal and penetration values. We need the 'correct'
            // values from the narrow phase algorithm to build a meaningful
            // contact manifold.
            cvh.BuildManifold(sA, sB, pA, pB, normal, penetration);
        }

        if (NarrowPhaseFilter != null)
        {
            if (!NarrowPhaseFilter.Filter(sA, sB, ref pA, ref pB, ref normal, ref penetration))
            {
                return;
            }
        }

        GetArbiter(sA.ShapeId, sB.ShapeId, sA.RigidBody, sB.RigidBody, out Arbiter arbiter);

        lock (arbiter)
        {
            // Do no add contacts while contacts might be resized
            memContacts.ResizeLock.EnterReadLock();

            arbiter.Handle.Data.IsSpeculative = false;

            for (int e = 0; e < cvh.Count; e++)
            {
                JVector mfA = cvh.ManifoldA[e];
                JVector mfB = cvh.ManifoldB[e];

                Real nd = JVector.Dot(mfA - mfB, normal);
                if (nd < (Real)0.0) continue;

                arbiter.Handle.Data.AddContact(mfA, mfB, normal, nd);
            }

            arbiter.Handle.Data.AddContact(pA, pB, normal, penetration);

            memContacts.ResizeLock.ExitReadLock();
        }
    }

    /// <summary>
    /// 获取仲裁器
    /// </summary>
    /// <param name="id0"></param>
    /// <param name="id1"></param>
    /// <param name="b0"></param>
    /// <param name="b1"></param>
    /// <param name="arbiter"></param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void GetArbiter(ulong id0, ulong id1, RigidBody b0, RigidBody b1, out Arbiter arbiter)
    {
        ArbiterKey arbiterKey = new(id0, id1);
        bool exists = arbiters.TryGetValue(arbiterKey, out Arbiter? arb);

        if (!exists)
        {
            lock (memContacts)
            {
                if (!Arbiter.Pool.TryPop(out arb))
                {
                    arb = new Arbiter();
                }

                var h = memContacts.Allocate(true);
                arb.Handle = h;
                h.Data.Init(b0, b1);
                h.Data.Key = arbiterKey;
                arb.Body1 = b0;
                arb.Body2 = b1;

                Debug.Assert(arb != null && memContacts.IsActive(arb.Handle));

                bool success = arbiters.TryAdd(arbiterKey, arb);

                if (success)
                {
                    deferredArbiters.Add(arb);
                }
                else
                {
                    memContacts.Free(h);
                    Arbiter.Pool.Push(arb);
                }
            }
        }

        arbiter = arb!;
    }
}