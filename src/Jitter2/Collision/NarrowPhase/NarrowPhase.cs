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
using Jitter2.LinearMath;
using Vertex = Jitter2.Collision.MinkowskiDifference.Vertex;

namespace Jitter2.Collision;

/// <summary>
/// �Ը��������������ײ���<br></br>
/// Ϊһ��͹�����ṩ��Ч��׼ȷ����ײ����㷨<br></br>
/// ��֧�ź�����ʽ���壬�μ�<see cref="ISupportMappable"/>��<br></br><br></br>
/// Provides efficient and accurate collision detection algorithms for general convex objects
/// implicitly defined by a support function, see <see cref="ISupportMappable"/>.
/// </summary>
public static class NarrowPhase
{
    private const Real NumericEpsilon = (Real)1e-16;

    /// <summary>
    /// �����
    /// </summary>
    private struct Solver
    {
        private ConvexPolytope convexPolytope;

        public bool PointTest(in ISupportMappable supportA, in JVector origin)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 34;

            JVector x = origin;

            supportA.GetCenter(out var center);
            JVector v = x - center;

            Unsafe.SkipInit(out SimplexSolver simplexSolver);
            simplexSolver.Reset();

            int maxIter = MaxIter;

            Real distSq = v.LengthSquared();

            while (distSq > CollideEpsilon * CollideEpsilon && maxIter-- != 0)
            {
                supportA.SupportMap(v, out JVector p);
                JVector.Subtract(x, p, out JVector w);

                Real vw = JVector.Dot(v, w);

                if (vw >= (Real)0.0)
                {
                    return false;
                }

                if (!simplexSolver.AddVertex(w, out v))
                {
                    goto converged;
                }

                distSq = v.LengthSquared();
            }

        converged:

            return true;
        }

        public bool RayCast(in ISupportMappable supportA, in JVector origin, in JVector direction, out Real lambda, out JVector normal)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 34;

            normal = JVector.Zero;
            lambda = (Real)0.0;

            JVector r = direction;
            JVector x = origin;

            supportA.GetCenter(out var center);
            JVector v = x - center;

            Unsafe.SkipInit(out SimplexSolver simplexSolver);
            simplexSolver.Reset();

            int maxIter = MaxIter;

            Real distSq = v.LengthSquared();

            while (distSq > CollideEpsilon * CollideEpsilon && maxIter-- != 0)
            {
                supportA.SupportMap(v, out JVector p);

                JVector.Subtract(x, p, out JVector w);

                Real VdotW = JVector.Dot(v, w);

                if (VdotW > (Real)0.0)
                {
                    Real VdotR = JVector.Dot(v, r);

                    if (VdotR >= -NumericEpsilon)
                    {
                        lambda = Real.PositiveInfinity;
                        return false;
                    }

                    lambda -= VdotW / VdotR;

                    JVector.Multiply(r, lambda, out x);
                    JVector.Add(origin, x, out x);
                    JVector.Subtract(x, p, out w);
                    normal = v;
                }

                if (!simplexSolver.AddVertex(w, out v))
                {
                    goto converged;
                }

                distSq = v.LengthSquared();
            }

        converged:

            Real nlen2 = normal.LengthSquared();

            if (nlen2 > NumericEpsilon)
            {
                normal *= (Real)1.0 / MathR.Sqrt(nlen2);
            }

            return true;
        }

        public bool Sweep(ref MinkowskiDifference mkd, in JVector sweep,
            out JVector p1, out JVector p2, out JVector normal, out Real lambda)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 34;

            Unsafe.SkipInit(out SimplexSolverAB simplexSolver);
            simplexSolver.Reset();

            mkd.GetCenter(out var center);

            JVector posB = mkd.PositionB;

            lambda = (Real)0.0;

            p1 = p2 = JVector.Zero;

            JVector r = sweep;
            JVector v = -center.V;

            normal = JVector.Zero;

            int iter = MaxIter;

            Real distSq = Real.MaxValue;

            while ((distSq > CollideEpsilon * CollideEpsilon) && (iter-- != 0))
            {
                mkd.Support(v, out Vertex vertex);
                var w = vertex.V;

                Real VdotW = -JVector.Dot(v, w);

                if (VdotW > (Real)0.0)
                {
                    Real VdotR = JVector.Dot(v, r);

                    if (VdotR >= -(Real)1e-12)
                    {
                        lambda = Real.PositiveInfinity;
                        return false;
                    }

                    lambda -= VdotW / VdotR;

                    mkd.PositionB = posB + lambda * r;
                    normal = v;
                }

                if (!simplexSolver.AddVertex(vertex, out v))
                {
                    goto converged;
                }

                v.Negate();

                distSq = v.LengthSquared();
            }

        converged:

            simplexSolver.GetClosest(out p1, out p2);

            Real nlen2 = normal.LengthSquared();

            if (nlen2 > NumericEpsilon)
            {
                normal *= (Real)1.0 / MathR.Sqrt(nlen2);
            }

            return true;
        }

        private bool SolveMPREPA(in MinkowskiDifference mkd, ref JVector point1, ref JVector point2, ref JVector normal, ref Real penetration)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 85;

            convexPolytope.InitTetrahedron();

            int iter = 0;

            Unsafe.SkipInit(out ConvexPolytope.Triangle ctri);

            while (++iter < MaxIter)
            {
                ctri = convexPolytope.GetClosestTriangle();

                JVector searchDir = ctri.ClosestToOrigin;
                Real searchDirSq = ctri.ClosestToOriginSq;

                if (ctri.ClosestToOriginSq < NumericEpsilon)
                {
                    searchDir = ctri.Normal;
                    searchDirSq = ctri.NormalSq;
                }

                mkd.Support(searchDir, out Vertex vertex);

                // compare with the corresponding code in SolveGJKEPA.
                Real deltaDist = JVector.Dot(ctri.ClosestToOrigin - vertex.V, searchDir);

                if (deltaDist * deltaDist <= CollideEpsilon * CollideEpsilon * searchDirSq)
                {
                    goto converged;
                }

                if (!convexPolytope.AddVertex(vertex))
                {
                    goto converged;
                }
            }

            Trace.WriteLine($"EPA: Could not converge within {MaxIter} iterations.");

            return false;

        converged:

            convexPolytope.CalculatePoints(ctri, out point1, out point2);

            normal = ctri.Normal * ((Real)1.0 / MathR.Sqrt(ctri.NormalSq));
            penetration = MathR.Sqrt(ctri.ClosestToOriginSq);

            return true;
        }

        public bool SolveMPR(in MinkowskiDifference mkd,
            out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
        {
            /*
            XenoCollide is available under the zlib license:

            XenoCollide Collision Detection and Physics Library
            Copyright (c) 2007-2014 Gary Snethen http://xenocollide.com

            This software is provided 'as-is', without any express or implied warranty.
            In no event will the authors be held liable for any damages arising
            from the use of this software.
            Permission is granted to anyone to use this software for any purpose,
            including commercial applications, and to alter it and redistribute it freely,
            subject to the following restrictions:

            1. The origin of this software must not be misrepresented; you must
            not claim that you wrote the original software. If you use this
            software in a product, an acknowledgment in the product documentation
            would be appreciated but is not required.
            2. Altered source versions must be plainly marked as such, and must
            not be misrepresented as being the original software.
            3. This notice may not be removed or altered from any source distribution.
            */
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 34;

            // If MPR reports a penetration deeper than this value we do not trust
            // MPR to have found the global minimum and perform an EPA run.
            const Real EPAPenetrationThreshold = (Real)0.02;

            Unsafe.SkipInit(out Vertex v0);
            Unsafe.SkipInit(out Vertex v1);
            Unsafe.SkipInit(out Vertex v2);
            Unsafe.SkipInit(out Vertex v3);
            Unsafe.SkipInit(out Vertex v4);

            Unsafe.SkipInit(out JVector temp1);
            Unsafe.SkipInit(out JVector temp2);
            Unsafe.SkipInit(out JVector temp3);

            penetration = (Real)0.0;

            mkd.GetCenter(out v0);

            if (Math.Abs(v0.V.X) < NumericEpsilon &&
                Math.Abs(v0.V.Y) < NumericEpsilon &&
                Math.Abs(v0.V.Z) < NumericEpsilon)
            {
                // any direction is fine
                v0.V.X = (Real)1e-05;
            }

            JVector.Negate(v0.V, out normal);

            mkd.Support(normal, out v1);

            pointA = v1.A;
            pointB = v1.B;

            if (JVector.Dot(v1.V, normal) <= (Real)0.0) return false;
            JVector.Cross(v1.V, v0.V, out normal);

            if (normal.LengthSquared() < NumericEpsilon)
            {
                JVector.Subtract(v1.V, v0.V, out normal);

                normal.Normalize();

                JVector.Subtract(v1.A, v1.B, out temp1);
                penetration = JVector.Dot(temp1, normal);

                return true;
            }

            mkd.Support(normal, out v2);

            if (JVector.Dot(v2.V, normal) <= (Real)0.0) return false;

            // Determine whether origin is on + or - side of plane (v1.V,v0.V,v2.V)
            JVector.Subtract(v1.V, v0.V, out temp1);
            JVector.Subtract(v2.V, v0.V, out temp2);
            JVector.Cross(temp1, temp2, out normal);

            Real dist = JVector.Dot(normal, v0.V);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            if (dist > (Real)0.0)
            {
                JVector.Swap(ref v1.V, ref v2.V);
                JVector.Swap(ref v1.A, ref v2.A);
                JVector.Swap(ref v1.B, ref v2.B);
                JVector.Negate(normal, out normal);
            }

            int phase2 = 0;
            int phase1 = 0;
            bool hit = false;

            // Phase One: Identify a portal
            while (true)
            {
                if (phase1 > MaxIter) return false;

                phase1++;

                mkd.Support(normal, out v3);

                if (JVector.Dot(v3.V, normal) <= (Real)0.0)
                {
                    return false;
                }

                // If origin is outside (v1.V,v0.V,v3.V), then eliminate v2.V and loop
                JVector.Cross(v1.V, v3.V, out temp1);
                if (JVector.Dot(temp1, v0.V) < (Real)0.0)
                {
                    v2 = v3;
                    JVector.Subtract(v1.V, v0.V, out temp1);
                    JVector.Subtract(v3.V, v0.V, out temp2);
                    JVector.Cross(temp1, temp2, out normal);
                    continue;
                }

                // If origin is outside (v3.V,v0.V,v2.V), then eliminate v1.V and loop
                JVector.Cross(v3.V, v2.V, out temp1);
                if (JVector.Dot(temp1, v0.V) < (Real)0.0)
                {
                    v1 = v3;
                    JVector.Subtract(v3.V, v0.V, out temp1);
                    JVector.Subtract(v2.V, v0.V, out temp2);
                    JVector.Cross(temp1, temp2, out normal);
                    continue;
                }

                break;
            }

            // Phase Two: Refine the portal
            // We are now inside of a wedge...
            while (true)
            {
                phase2++;

                // Compute normal of the wedge face
                JVector.Subtract(v2.V, v1.V, out temp1);
                JVector.Subtract(v3.V, v1.V, out temp2);
                JVector.Cross(temp1, temp2, out normal);

                // normal.Normalize();
                Real normalSq = normal.LengthSquared();

                // Can this happen???  Can it be handled more cleanly?
                if (normalSq < NumericEpsilon)
                {
                    // was: return true;
                    // better not return a collision
                    Trace.WriteLine("MPR: This should not happen.");
                    return false;
                }

                if (!hit)
                {
                    // Compute distance from origin to wedge face
                    Real d = JVector.Dot(normal, v1.V);
                    // If the origin is inside the wedge, we have a hit
                    hit = d >= 0;
                }

                mkd.Support(normal, out v4);

                JVector.Subtract(v4.V, v3.V, out temp3);
                Real delta = JVector.Dot(temp3, normal);
                penetration = JVector.Dot(v4.V, normal);

                // If the boundary is thin enough or the origin is outside the support plane for the newly discovered
                // vertex, then we can terminate
                if (delta * delta <= CollideEpsilon * CollideEpsilon * normalSq || penetration <= (Real)0.0 ||
                    phase2 > MaxIter)
                {
                    if (hit)
                    {
                        Real invnormal = (Real)1.0 / MathR.Sqrt(normalSq);

                        penetration *= invnormal;

                        if (penetration > EPAPenetrationThreshold)
                        {
                            convexPolytope.InitHeap();
                            convexPolytope.GetVertex(0) = v0;
                            convexPolytope.GetVertex(1) = v1;
                            convexPolytope.GetVertex(2) = v2;
                            convexPolytope.GetVertex(3) = v3;

                            // If epa fails it does not set any result data. We continue with the mpr data.
                            if (SolveMPREPA(mkd, ref pointA, ref pointB, ref normal, ref penetration)) return true;
                        }

                        normal *= invnormal;

                        // Compute the barycentric coordinates of the origin
                        JVector.Cross(v1.V, temp1, out temp3);
                        Real gamma = JVector.Dot(temp3, normal) * invnormal;
                        JVector.Cross(temp2, v1.V, out temp3);
                        Real beta = JVector.Dot(temp3, normal) * invnormal;
                        Real alpha = (Real)1.0 - gamma - beta;

                        pointA = alpha * v1.A + beta * v2.A + gamma * v3.A;
                        pointB = alpha * v1.B + beta * v2.B + gamma * v3.B;
                    }

                    return hit;
                }

                // Compute the tetrahedron dividing face (v4.V,v0.V,v3.V)
                JVector.Cross(v4.V, v0.V, out temp1);
                Real dot = JVector.Dot(temp1, v1.V);

                if (dot >= (Real)0.0)
                {
                    dot = JVector.Dot(temp1, v2.V);

                    if (dot >= (Real)0.0)
                    {
                        v1 = v4; // Inside d1 & inside d2 -> eliminate v1.V
                    }
                    else
                    {
                        v3 = v4; // Inside d1 & outside d2 -> eliminate v3.V
                    }
                }
                else
                {
                    dot = JVector.Dot(temp1, v3.V);

                    if (dot >= (Real)0.0)
                    {
                        v2 = v4; // Outside d1 & inside d3 -> eliminate v2.V
                    }
                    else
                    {
                        v1 = v4; // Outside d1 & outside d3 -> eliminate v1.V
                    }
                }
            }
        }

        public bool Distance(in MinkowskiDifference mkd,
            out JVector point1, out JVector point2, out Real distance)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 34;

            Unsafe.SkipInit(out SimplexSolverAB simplexSolver);
            simplexSolver.Reset();

            int maxIter = MaxIter;

            mkd.GetCenter(out var center);
            JVector v = center.V;
            Real distSq = v.LengthSquared();

            while (maxIter-- != 0)
            {
                mkd.Support(-v, out var w);

                distSq = v.LengthSquared();

                Real deltaDist = JVector.Dot(v - w.V, v);
                if (deltaDist * deltaDist < CollideEpsilon * CollideEpsilon * distSq)
                {
                    break;
                }

                if (distSq < CollideEpsilon * CollideEpsilon ||
                    !simplexSolver.AddVertex(w, out v))
                {
                    distance = (Real)0.0;
                    point1 = point2 = JVector.Zero;
                    return false;
                }
            }

            distance = MathR.Sqrt(distSq);
            simplexSolver.GetClosest(out point1, out point2);

            return true;
        }

        public bool Overlap(in MinkowskiDifference mkd)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 34;

            Unsafe.SkipInit(out SimplexSolverAB simplexSolver);
            simplexSolver.Reset();

            int maxIter = MaxIter;

            mkd.GetCenter(out var center);
            JVector v = center.V;
            Real distSq = v.LengthSquared();

            while (distSq > CollideEpsilon * CollideEpsilon && maxIter-- != 0)
            {
                mkd.Support(-v, out var w);
                Real vw = JVector.Dot(v, w.V);
                if (vw >= (Real)0.0)
                    return false;
                if (!simplexSolver.AddVertex(w, out v)) return true;
                distSq = v.LengthSquared();
            }

            return true;
        }

        public bool Collision(in MinkowskiDifference mkd,
            out JVector point1, out JVector point2, out JVector normal, out Real penetration)
        {
            const Real CollideEpsilon = (Real)1e-4;
            const int MaxIter = 85;

            mkd.GetCenter(out Vertex centerVertex);
            JVector center = centerVertex.V;

            convexPolytope.InitHeap();
            convexPolytope.InitTetrahedron(center);

            int iter = 0;

            Unsafe.SkipInit(out ConvexPolytope.Triangle ctri);

            while (++iter < MaxIter)
            {
                ctri = convexPolytope.GetClosestTriangle();

                JVector searchDir = ctri.ClosestToOrigin;
                Real searchDirSq = ctri.ClosestToOriginSq;

                if (!convexPolytope.OriginEnclosed) searchDir.Negate();

                if (ctri.ClosestToOriginSq < NumericEpsilon)
                {
                    searchDir = ctri.Normal;
                    searchDirSq = ctri.NormalSq;
                }

                mkd.Support(searchDir, out Vertex vertex);

                // Can we further "extend" the convex hull by adding the new vertex?
                //
                // v = Vertices[vPointer] (support point)
                // c = Triangles[Head].ClosestToOrigin
                // s = searchDir
                //
                // abs(dot(c - v, s)) / len(s) < e <=> [dot(c - v, s)]^2 = e*e*s^2
                Real deltaDist = JVector.Dot(ctri.ClosestToOrigin - vertex.V, searchDir);

                if (deltaDist * deltaDist <= CollideEpsilon * CollideEpsilon * searchDirSq)
                {
                    goto converged;
                }

                if (!convexPolytope.AddVertex(vertex))
                {
                    goto converged;
                }
            }

            point1 = point2 = normal = JVector.Zero;
            penetration = (Real)0.0;

            Trace.WriteLine($"EPA: Could not converge within {MaxIter} iterations.");

            return false;

        converged:

            convexPolytope.CalculatePoints(ctri, out point1, out point2);

            penetration = MathR.Sqrt(ctri.ClosestToOriginSq);
            if (!convexPolytope.OriginEnclosed) penetration *= -(Real)1.0;

            if (MathR.Abs(penetration) > NumericEpsilon) normal = ctri.ClosestToOrigin * ((Real)1.0 / penetration);
            else normal = ctri.Normal * ((Real)1.0 / MathR.Sqrt(ctri.NormalSq));

            return true;
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    [ThreadStatic] private static Solver solver;

    /// <summary>
    /// ���ĳ�����Ƿ�λ����״�ڡ�<br></br><br></br>
    /// Check if a point is inside a shape.
    /// </summary>
    /// <param name="support">
    /// ֧��map��Ϊ��״��<br></br><br></br> 
    /// Support map representing the shape.
    /// </param>
    /// <param name="point">
    /// Ҫ���ĵ㡣<br></br><br></br> 
    /// Point to check.
    /// </param>
    /// <returns>
    /// ����õ��������״�ڣ��򷵻� true�����򷵻� false��<br></br><br></br>
    /// Returns true if the point is contained within the shape, false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointTest(in ISupportMappable support, in JVector point)
    {
        return solver.PointTest(support, point);
    }

    /// <summary>
    /// ���ĳ�����Ƿ�λ����״�ڡ�<br></br><br></br>
    /// Check if a point is inside a shape.
    /// </summary>
    /// <param name="support">֧��map��Ϊ��״��<br></br> Support map representing the shape.</param>
    /// <param name="orientation">��״�ķ��� <br></br> Orientation of the shape.</param>
    /// <param name="position">��״��λ�� <br> </br>Position of the shape.</param>
    /// <param name="point">Ҫ���ĵ� <br></br> Point to check.</param>
    /// <returns>
    /// ����õ��������״�ڣ��򷵻� true�����򷵻� false��<br></br><br></br>
    /// Returns true if the point is contained within the shape, false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointTest(in ISupportMappable support, in JMatrix orientation,
        in JVector position, in JVector point)
    {
        JVector transformedOrigin = JVector.TransposedTransform(point - position, orientation);
        return solver.PointTest(support, transformedOrigin);
    }

    /// <summary>
    /// ��ĳ����״��������Ͷ�䡣<br></br><br></br>
    /// Performs a ray cast against a shape.
    /// </summary>
    /// <param name="support">֧��map��Ϊ��״��<br></br> Support map representing the shape.</param>
    /// <param name="orientation">����ռ�����״�ķ��� <br></br> The orientation of the shape in world space.</param>
    /// <param name="position">����ռ�����״��λ�� <br></br><br> </br>The position of the shape in world space.</param>
    /// <param name="origin">���ߵ�ԭ�� <br></br><br></br> The origin of the ray.</param>
    /// <param name="direction">
    /// ���ߵķ���, ͨ���Ǳ�Ҫ <br></br><br></br> 
    /// The direction of the ray; normalization is not necessary.
    /// </param>
    /// <param name="lambda">
    /// ָ�����ߵ����е㣬����Ϊ��ԭ�� + ���� * ���򡱡�<br></br><br></br>
    /// Specifies the hit point of the ray, calculated as 'origin + lambda * direction'.</param>
    /// <param name="normal">
    /// ��ֱ�ڱ���ķ���������ָ���ⲿ���������δ���У���˲���Ϊ�㡣<br></br><br></br>
    /// The normalized normal vector perpendicular to the surface, pointing outwards. If the ray does not
    /// hit, this parameter will be zero.
    /// </param>
    /// <returns>Returns true if the ray intersects with the shape; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RayCast(in ISupportMappable support, in JQuaternion orientation,
        in JVector position, in JVector origin, in JVector direction, out Real lambda, out JVector normal)
    {
        // rotate the ray into the reference frame of bodyA..
        JVector tdirection = JVector.TransposedTransform(direction, orientation);
        JVector torigin = JVector.TransposedTransform(origin - position, orientation);

        bool result = solver.RayCast(support, torigin, tdirection, out lambda, out normal);

        // ..rotate back.
        JVector.Transform(normal, orientation, out normal);

        return result;
    }

    /// <summary>
    /// ��ĳ����״��������Ͷ�䡣<br></br><br></br>
    /// Performs a ray cast against a shape.
    /// </summary>
    /// <param name="support">��״��֧�ź��� <br></br><br></br>The support function of the shape.</param>
    /// <param name="origin">���ߵ�ԭ�� <br></br><br></br>The origin of the ray.</param>
    /// <param name="direction">
    /// ���ߵķ���, ͨ���Ǳ�Ҫ <br></br><br></br> 
    /// The direction of the ray; normalization is not necessary.
    /// </param>
    /// <param name="lambda">
    /// ָ�����ߵ����е㣬����Ϊ��ԭ�� + ���� * ���򡱡�<br></br><br></br>
    /// Specifies the hit point of the ray, calculated as 'origin + lambda * direction'.
    /// </param>
    /// <param name="normal">
    /// ��ֱ�ڱ���ķ���������ָ���ⲿ���������δ���У���˲���Ϊ�㡣<br></br><br></br>
    /// The normalized normal vector perpendicular to the surface, pointing outwards. If the ray does not
    /// hit, this parameter will be zero.
    /// </param>
    /// <returns>
    /// �����������״�ཻ���򷵻� true�����򷵻� false��<br></br><br></br>
    /// Returns true if the ray intersects with the shape; otherwise, false.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RayCast(in ISupportMappable support, in JVector origin, in JVector direction, out Real lambda, out JVector normal)
    {
        return solver.RayCast(support, origin, direction, out lambda, out normal);
    }

    /// <summary>
    /// ȷ������͹���Ƿ��ص���Ϊ�ص��ͷ��������ṩ��ϸ��Ϣ��������֧����״ A λ����λ����δ��ת��<br></br>
    /// ���ڲ����÷���������չ�������㷨 (EPA) ���ռ���ײ��Ϣ��<br></br><br></br>
    /// Determines whether two convex shapes overlap, providing detailed information for both overlapping and separated
    /// cases. It assumes that support shape A is at position zero and not rotated.
    /// Internally, the method employs the Expanding Polytope Algorithm (EPA) to gather collision information.
    /// </summary>
    /// <param name="supportA">��״A��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״B��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationB">��״B�ķ��� <br></br><br></br> The orientation of shape B.</param>
    /// <param name="positionB">��״B��λ�� <br></br><br></br> The position of shape B.</param>
    /// <param name="pointA">
    /// �����ص����������״ A �ϵ������λ����״ B �ڣ�<br></br>
    /// ���ڷ������������������״ A �� B ֮�䡣<br></br><br></br>
    /// For the overlapping case: the deepest point on shape A inside shape B; for the separated case: the
    /// closest point on shape A to shape B.
    /// </param>
    /// <param name="pointB">
    /// �����ص����������״ B ����״ A �ڵ�����㣻<br></br>
    /// ���ڷ������������������״ A �� B ֮�䡣<br></br><br></br>
    /// For the overlapping case: the deepest point on shape B inside shape A; for the separated case: the
    /// closest point on shape B to shape A.
    /// </param>
    /// <param name="normal">
    /// �ӵ�Bָ���A�ı�׼��ײ��������<br></br><br></br>
    /// ��ʹ��A�͵�B�غϣ����������Ȼ�ж��塣����ʾ��״Ӧ��������������ƶ���С���루�ɴ�͸��ȶ��壩��
    /// �����ص�������½����Ƿֿ������ڷ��������½����ǽӴ���<br></br><br></br>
    /// The normalized collision normal pointing from pointB to pointA. This normal remains defined even
    /// if pointA and pointB coincide. It denotes the direction in which the shapes should be moved by the minimum distance
    /// (defined by the penetration depth) to either separate them in the overlapping case or bring them into contact in
    /// the separated case.
    /// </param>
    /// <param name="penetration">
    /// ��͸��� <br></br><br></br>
    /// The penetration depth.
    /// </param>
    /// <returns>
    /// ����㷨�ɹ���ɣ��򷵻�true�����򷵻�false�����㷨����ʧ�ܵ�����£���ײ��Ϣ���ָ�Ϊ�����͵�Ĭ��ֵ��<br></br><br></br>
    /// Returns true if the algorithm completes successfully, false otherwise. In case of algorithm convergence
    /// failure, collision information reverts to the type's default values.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Collision(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationB, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;
        mkd.PositionB = positionB;
        mkd.OrientationB = orientationB;

        // ..perform collision detection..
        bool success = solver.Collision(mkd, out pointA, out pointB, out normal, out penetration);

        return success;
    }

    /// <summary>
    /// ȷ������͹��״�Ƿ��ص�<br></br>
    /// ��Ϊ�ص��ͷ��������ṩ��ϸ��Ϣ�����ڲ����÷���ʹ����չ�������㷨��EPA�����ռ���ײ��Ϣ��<br></br><br></br>
    /// Determines whether two convex shapes overlap, providing detailed information for both overlapping and separated
    /// cases. Internally, the method employs the Expanding Polytope Algorithm (EPA) to gather collision information.
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationA">��״ A ������ռ��еķ��� <br></br><br></br> The orientation of shape A in world space.</param>
    /// <param name="orientationB">��״ A ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionA">��״ A ������ռ��е�λ�� <br></br><br></br> The position of shape A in world space.</param>
    /// <param name="positionB">��״ A ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="pointA">
    /// �����ص����������״ A �ϵ������λ����״ B �ڣ�<br></br>
    /// ���ڷ���������������� A �� B ֮�䡣<br></br><br></br>
    /// For the overlapping case: the deepest point on shape A inside shape B; for the separated case: the
    /// closest point on shape A to shape B.
    /// </param>
    /// <param name="pointB">
    /// �����ص����������״ B �ϵ������λ����״ A �ڣ�<br></br>
    /// ���ڷ���������������� A �� B ֮�䡣<br></br><br></br>
    /// For the overlapping case: the deepest point on shape B inside shape A; for the separated case: the
    /// closest point on shape B to shape A.
    /// </param>
    /// <param name="normal">
    /// �ӵ�Bָ���A�ı�׼��ײ��������<br></br><br></br>
    /// ��ʹ��A�͵�B�غϣ����������Ȼ�ж��塣����ʾ��״Ӧ��������������ƶ���С���루�ɴ�͸��ȶ��壩��
    /// �����ص�������½����Ƿֿ������ڷ��������½����ǽӴ���<br></br><br></br>
    /// The normalized collision normal pointing from pointB to pointA. This normal remains defined even
    /// if pointA and pointB coincide. It denotes the direction in which the shapes should be moved by the minimum distance
    /// (defined by the penetration depth) to either separate them in the overlapping case or bring them into contact in
    /// the separated case.
    /// </param>
    /// <param name="penetration">��͸��� <br></br><br></br> The penetration depth.</param>
    /// <returns>
    /// ����㷨�ɹ���ɣ��򷵻� true�����򷵻� false��<br></br>
    /// ����㷨���� ʧ�ܣ���ײ��Ϣ���ָ�Ϊ���͵�Ĭ��ֵ��<br></br><br></br>
    /// Returns true if the algorithm completes successfully, false otherwise. In case of algorithm convergence
    /// failure, collision information reverts to the type's default values.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Collision(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;

        // rotate into the reference frame of bodyA..
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out mkd.OrientationB);
        JVector.Subtract(positionB, positionA, out mkd.PositionB);
        JVector.ConjugatedTransform(mkd.PositionB, orientationA, out mkd.PositionB);

        // ..perform collision detection..
        bool success = solver.Collision(mkd, out pointA, out pointB, out normal, out penetration);

        // ..rotate back. this hopefully saves some matrix vector multiplication
        // when calling the support function multiple times.
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        return success;
    }

    /// <summary>
    /// �ṩ���ص���״�ľ��������㡣<br></br>
    /// ������֧����״ A λ��λ������δ��ת��<br></br><br></br> 
    /// Provides the distance and closest points for non overlapping shapes. It
    /// assumes that support shape A is located at position zero and not rotated.
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationB">��״ B ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionB">��״ B ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="pointA">��״ A �ϵ������, ����ص���Ϊ 0 <br></br><br></br> Closest point on shape A. Zero if shapes overlap.</param>
    /// <param name="pointB">��״ B �ϵ������, ����ص���Ϊ 0 <br></br><br></br> Closest point on shape B. Zero if shapes overlap.</param>
    /// <param name="distance">������״�ľ���, ����ص���Ϊ 0 <br></br><br></br> The distance between the separating shapes. Zero if shapes overlap.</param>
    /// <returns>
    /// �����״���ص����ҿ����ṩ������Ϣ���򷵻� true��<br></br><br></br>
    /// Returns true if the shapes do not overlap and distance information can be provided.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Distance(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationB, in JVector positionB,
        out JVector pointA, out JVector pointB, out Real distance)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;
        mkd.PositionB = positionB;
        mkd.OrientationB = orientationB;

        // ..perform overlap test..
        return solver.Distance(mkd, out pointA, out pointB, out distance);
    }

    /// <summary>
    /// �ṩ���ص���״�ľ��������㡣<br></br><br></br>
    /// Provides the distance and closest points for non overlapping shapes.
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationA">��״ A ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="orientationB">��״ B ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionA">��״ A ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="positionB">��״ B ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="pointA">��״ A �ϵ������, ����ص���Ϊ 0 <br></br><br></br> Closest point on shape A. Zero if shapes overlap.</param>
    /// <param name="pointB">��״ B �ϵ������, ����ص���Ϊ 0 <br></br><br></br> Closest point on shape B. Zero if shapes overlap.</param>
    /// <param name="distance">������״�ľ���, ����ص���Ϊ 0 <br></br><br></br> The distance between the separating shapes. Zero if shapes overlap.</param>
    /// <returns>
    /// �����״���ص����ҿ����ṩ������Ϣ���򷵻� true��<br></br><br></br>
    /// Returns true if the shapes do not overlap and distance information can be provided.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Distance(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        out JVector pointA, out JVector pointB, out Real distance)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;

        // rotate into the reference frame of bodyA..
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out mkd.OrientationB);
        JVector.Subtract(positionB, positionA, out mkd.PositionB);
        JVector.ConjugatedTransform(mkd.PositionB, orientationA, out mkd.PositionB);

        // ..perform overlap test..
        bool result = solver.Distance(mkd, out pointA, out pointB, out distance);
        if (!result) return false;

        // ..rotate back. This approach potentially saves some matrix-vector multiplication when
        // the support function is called multiple times.
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);

        return true;
    }

    /// <summary>
    /// ִ���ص����ԡ�����֧����״ A λ��λ������δ��ת��<br></br><br></br>
    /// Performs an overlap test. It assumes that support shape A is located
    /// at position zero and not rotated.
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationB">��״ B ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionB">��״ B ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <returns>
    /// �����״�ص��򷵻� true�����򷵻� false��<br></br><br></br>
    /// Returns true of the shapes overlap, and false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Overlap(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationB, in JVector positionB)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;
        mkd.PositionB = positionB;
        mkd.OrientationB = orientationB;

        // ..perform overlap test..
        return solver.Overlap(mkd);
    }

    /// <summary>
    /// ִ���ص����ԡ�<br></br><br></br>
    /// Performs an overlap test.
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationA">��״ A ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="orientationB">��״ B ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionA">��״ A ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="positionB">��״ B ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <returns>
    /// �����״�ص��򷵻� true�����򷵻� false��<br></br><br></br>
    /// Returns true of the shapes overlap, and false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Overlap(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;

        // rotate into the reference frame of bodyA..
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out mkd.OrientationB);
        JVector.Subtract(positionB, positionA, out mkd.PositionB);
        JVector.ConjugatedTransform(mkd.PositionB, orientationA, out mkd.PositionB);

        // ..perform overlap test..
        return solver.Overlap(mkd);
    }

    /// <summary>
    /// �������͹��״�Ƿ��ص�����Ϊ�ص���״�ṩ��ϸ����ײ��Ϣ��<br></br>
    /// �ڲ����˷��������ɿɷ�˹���Ż�ϸ����MPR������ȡ��ײ��Ϣ��
    /// ����MPR����ȷ�������ṩ�˴�͸��ȵ��ϸ��Ͻ硣<br></br>
    /// ����Ͻ糬��Ԥ�������ֵ����ʹ����չ�������㷨��EPA����һ��ϸ�������<br></br><br></br>
    /// Detects whether two convex shapes overlap and provides detailed collision information for overlapping shapes.
    /// Internally, this method utilizes the Minkowski Portal Refinement (MPR) to obtain the collision information.
    /// Although MPR is not exact, it delivers a strict upper bound for the penetration depth. If the upper bound surpasses
    /// a predefined threshold, the results are further refined using the Expanding Polytope Algorithm (EPA).
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationA">��״ A ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="orientationB">��״ B ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionA">��״ A ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="positionB">��״ B ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="pointA">��״ A ��λ����״ B �ڲ�������㡣<br></br><br></br> The deepest point on shape A that is inside shape B.</param>
    /// <param name="pointB">��״ B ��λ����״ A �ڲ�������㡣<br></br><br></br> The deepest point on shape B that is inside shape A.</param>
    /// <param name="normal">
    /// �ӵ� B ָ��� A �ı�׼��ײ��������<br></br>
    /// ��ʹ�� A �͵� B �غϣ��˷�������Ȼ���ֶ��壬��ʾ��״��������С���루�ɴ�͸��Ⱦ������ָ��ķ����Ա����ص���<br></br><br></br>
    /// The normalized collision normal pointing from pointB to pointA. This normal remains defined even
    /// if pointA and pointB coincide, representing the direction in which the shapes must be separated by the minimal
    /// distance (determined by the penetration depth) to avoid overlap.
    /// </param>
    /// <param name="penetration">��͸��ȡ�<br></br><br></br> The penetration depth.</param>
    /// <returns>
    /// �����״�ص�����ײ���򷵻� true�����򷵻� false��<br></br><br></br>
    /// Returns true if the shapes overlap (collide), and false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MPREPA(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;

        // rotate into the reference frame of bodyA..
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out mkd.OrientationB);
        JVector.Subtract(positionB, positionA, out mkd.PositionB);
        JVector.ConjugatedTransform(mkd.PositionB, orientationA, out mkd.PositionB);

        // ..perform collision detection..
        bool res = solver.SolveMPR(mkd, out pointA, out pointB, out normal, out penetration);

        // ..rotate back. This approach potentially saves some matrix-vector multiplication when the support function is called multiple times.
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        return res;
    }

    /// <summary>
    /// �������͹���Ƿ��ص������ṩ�ص���״����ϸ��ײ��Ϣ��<br></br><br></br>
    /// ����֧����״ A λ����λ����δ��ת��<br></br>
    /// ���ڲ����÷������� Minkowski Portal Refinement (MPR) ����ȡ��ײ��Ϣ��
    /// ��Ȼ MPR ������ȷ������Ϊ��͸����ṩ���ϸ�����ޡ�������޳���
    /// Ԥ�������ֵ����ʹ����չ�������㷨 (EPA) ��һ��ϸ�������<br></br><br></br>
    /// Detects whether two convex shapes overlap and provides detailed collision information for overlapping shapes.
    /// It assumes that support shape A is at position zero and not rotated.
    /// Internally, this method utilizes the Minkowski Portal Refinement (MPR) to obtain the collision information.
    /// Although MPR is not exact, it delivers a strict upper bound for the penetration depth. If the upper bound surpasses
    /// a predefined threshold, the results are further refined using the Expanding Polytope Algorithm (EPA).
    /// </summary>
    /// <param name="supportA">��״ A ��֧�ź��� <br></br><br></br> The support function of shape A.</param>
    /// <param name="supportB">��״ B ��֧�ź��� <br></br><br></br> The support function of shape B.</param>
    /// <param name="orientationB">��״ B ������ռ��еķ��� <br></br><br></br> The orientation of shape B in world space.</param>
    /// <param name="positionB">��״ B ������ռ��е�λ�� <br></br><br></br> The position of shape B in world space.</param>
    /// <param name="pointA">��״ A ��λ����״ B �ڲ�������㡣<br></br><br></br> The deepest point on shape A that is inside shape B.</param>
    /// <param name="pointB">��״ B ��λ����״ A �ڲ�������㡣<br></br><br></br> The deepest point on shape B that is inside shape A.</param>
    /// <param name="normal">
    /// �ӵ� B ָ��� A �ı�׼����ײ��������<br></br>
    /// ����� A �͵� B �غϣ��˷��߱��� d ����ʾ��״���뱣�־��루�ɴ�͸��Ⱦ������ķ����Ա����ص���<br></br><br></br>
    /// The normalized collision normal pointing from pointB to pointA. This normal remains d
    /// if pointA and pointB coincide, representing the direction in which the shapes must be
    /// distance (determined by the penetration depth) to avoid overlap.
    /// </param>
    /// <param name="penetration">��͸��ȡ�<br></br><br></br> The penetration depth.</param>
    /// <returns>
    /// �����״�ص�����ײ�����򷵻� true�����򷵻� false��<br></br><br></br>
    /// eturns true if the shapes overlap (collide), and false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MPREPA(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationB, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);
        mkd.SupportA = supportA;
        mkd.SupportB = supportB;
        mkd.PositionB = positionB;
        mkd.OrientationB = orientationB;

        // ..perform collision detection..
        bool res = solver.SolveMPR(mkd, out pointA, out pointB, out normal, out penetration);

        return res;
    }

    /// <summary>
    /// ����������״���ٶ� sweepA �� sweepB �·�����ײ��ʱ�������ռ��е���ײ�㡣<br></br><br></br>
    /// Calculates the time of impact and the collision points in world space for two shapes with velocities sweepA and sweepB.
    /// </summary>
    /// <param name="pointA">
    /// t = 0 ʱ����ռ�����״ A �ϵ���ײ�㣬��ײ���ڴ˷��������û�м�⵽��ײ����Ϊ�㡣<br></br><br></br>
    /// Collision point on shapeA in world space at t = 0, where collision will occur.Zero if no hit is detected.
    /// </param>
    /// <param name="pointB">
    /// t = 0 ʱ����ռ�����״ B �ϵ���ײ�㣬��ײ���ڴ˷��������û�м�⵽��ײ����Ϊ�㡣<br></br><br></br>
    /// Collision point on shapeA in world space at t = 0, where collision will occur.Zero if no hit is detected.
    /// </param>
    /// <param name="lambda">
    /// ײ��ʱ�䡣��δ��⵽ײ����Ϊ����ʱ�䡣<br></br><br></br>
    /// Time of impact. Infinity if no hit is detected.
    /// </param>
    /// <returns>True if the shapes hit, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Sweep(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        in JVector sweepA, in JVector sweepB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);

        mkd.SupportA = supportA;
        mkd.SupportB = supportB;

        // rotate into the reference frame of bodyA..
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out mkd.OrientationB);
        JVector.Subtract(positionB, positionA, out mkd.PositionB);
        JVector.ConjugatedTransform(mkd.PositionB, orientationA, out mkd.PositionB);

        // we also transform the relative velocities
        JVector sweep = sweepB - sweepA;
        JVector.ConjugatedTransform(sweep, orientationA, out sweep);

        // ..perform toi calculation
        bool res = solver.Sweep(ref mkd, sweep, out pointA, out pointB, out normal, out lambda);

        if (!res) return false;

        // ..rotate back. This approach potentially saves some matrix-vector multiplication when the support function is
        // called multiple times.
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        // transform back from the relative velocities

        // This is where the collision will occur in world space:
        //      pointA += lambda * sweepA;
        //      pointB += lambda * sweepA; // sweepA is not a typo

        pointB += lambda * (sweepA - sweepB);

        return true;
    }

    /// <summary>
    /// ִ��ɨ����ԣ�����֧����״ A λ����λ�ã�δ��ת����ɨ�� <br></br><br></br>
    /// Perform a sweep test where support shape A is at position zero, not rotated and has no sweep
    /// direction.
    /// </summary>
    /// <param name="pointA">
    /// ����ռ�����״ A �ϵ���ײ�㡣���û�м�⵽��ײ����Ϊ�㡣 <br></br><br></br>   
    /// Collision point on shapeA in world space. Zero if no hit is detected.
    /// </param>
    /// <param name="pointB">
    /// ����ռ�����״ B �ϵ���ײ�㡣���û�м�⵽��ײ����Ϊ�㡣 <br></br><br></br>   
    /// Collision point on shapeA in world space. Zero if no hit is detected.
    /// </param>    
    /// <param name="lambda">
    /// ײ��ʱ�䡣��δ��⵽ײ����Ϊ����ʱ�䡣<br></br><br></br>
    /// Time of impact. Infinity if no hit is detected.
    /// </param>
    /// <returns>
    /// �����״������Ϊ True������Ϊ false��
    /// True if the shapes hit, false otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Sweep(in ISupportMappable supportA, in ISupportMappable supportB,
        in JQuaternion orientationB, in JVector positionB, in JVector sweepB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
    {
        Unsafe.SkipInit(out MinkowskiDifference mkd);

        mkd.SupportA = supportA;
        mkd.SupportB = supportB;
        mkd.PositionB = positionB;
        mkd.OrientationB = orientationB;

        // ..perform toi calculation
        return solver.Sweep(ref mkd, sweepB, out pointA, out pointB, out normal, out lambda);
    }
}