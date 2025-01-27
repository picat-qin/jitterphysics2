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

using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 动态树
/// </summary>
public partial class DynamicTree
{
    /// <summary>
    /// 射线投射的结果。<br></br><br></br>
    /// Preliminary result of the ray cast.
    /// </summary>
    public struct RayCastResult
    {
        /// <summary>
        /// 代理实体
        /// </summary>
        public IDynamicTreeProxy Entity;
        /// <summary>
        /// 触发距离?
        /// </summary>
        public Real Lambda;
        /// <summary>
        /// 法向量
        /// </summary>
        public JVector Normal;
    }

    /// <summary>
    /// 后过滤器委托。<br></br><br></br>
    /// Post-filter delegate.
    /// </summary>
    /// <returns>False if the hit should be filtered out.</returns>
    public delegate bool RayCastFilterPost(RayCastResult result);

    /// <summary>
    /// 预过滤器委托。<br></br><br></br>
    /// Pre-filter delegate.
    /// </summary>
    /// <returns>False if the hit should be filtered out.</returns>
    public delegate bool RayCastFilterPre(IDynamicTreeProxy result);

    private struct Ray
    {
        public readonly JVector Origin;
        public readonly JVector Direction;

        public RayCastFilterPost? FilterPost;
        public RayCastFilterPre? FilterPre;

        public Real Lambda;

        public Ray(in JVector origin, in JVector direction)
        {
            Origin = origin;
            Direction = direction;
            FilterPost = null;
            FilterPre = null;
            Lambda = Real.MaxValue;
        }
    }

    /// <summary>
    /// 射线向发射世界
    /// Ray cast against the world.
    /// </summary>
    /// <param name="origin">射线的原点 <br></br><br></br> Origin of the ray.</param>
    /// <param name="direction">射线的方向, 无需标准化向量 <br></br><br></br> Direction of the ray. Does not have to be normalized.</param>
    /// <param name="pre">
    /// 可选预过滤器，允许跳过检测中的形状。<br></br><br></br>
    /// Optional pre-filter which allows to skip shapes in the detection.
    /// </param>
    /// <param name="post">
    /// 可选的后过滤器允许跳过检测。<br></br><br></br>
    /// Optional post-filter which allows to skip detections.
    /// </param>
    /// <param name="proxy">
    /// 受到击中的形状。<br></br><br></br>
    /// The shape which was hit.
    /// </param>
    /// <param name="normal">
    /// 射线击中表面的法线。如果射线未击中，则为零。<br></br><br></br>
    /// The normal of the surface where the ray hits. Zero if ray does not hit.
    /// </param>
    /// <param name="lambda">
    /// 从原点到射线命中点的距离，以射线方向的单位表示。<br></br><br></br>
    /// Distance from the origin to the ray hit point in units of the ray's direction.
    /// </param>
    /// <returns>
    /// 如果射线击中则为 True，否则为 false。
    /// True if the ray hits, false otherwise.
    /// </returns>
    public bool RayCast(JVector origin, JVector direction, RayCastFilterPre? pre, RayCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector normal, out Real lambda)
    {
        Ray ray = new(origin, direction)
        {
            FilterPre = pre,
            FilterPost = post
        };
        bool hit = QueryRay(ray, out var result);
        proxy = result.Entity;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    /// <inheritdoc cref="RayCast(JVector, JVector, RayCastFilterPre?, RayCastFilterPost?, out IDynamicTreeProxy?, out JVector, out Real)"/>
    /// <param name="maxLambda">考虑相交的射线长度的最大 lambda。<br></br><br></br> Maximum lambda of the ray's length to consider for intersections.</param>
    public bool RayCast(JVector origin, JVector direction, Real maxLambda, RayCastFilterPre? pre, RayCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector normal, out Real lambda)
    {
        Ray ray = new(origin, direction)
        {
            FilterPre = pre,
            FilterPost = post,
            Lambda = maxLambda
        };
        bool hit = QueryRay(ray, out var result);
        proxy = result.Entity;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    private bool QueryRay(in Ray ray, out RayCastResult result)
    {
        result = new RayCastResult();

        if (root == -1)
        {
            return false;
        }

        stack ??= new Stack<int>(256);

        stack.Push(root);

        bool globalHit = false;

        result.Lambda = ray.Lambda;

        while (stack.Count > 0)
        {
            int pop = stack.Pop();

            ref Node node = ref Nodes[pop];

            if (node.IsLeaf)
            {
                if (node.Proxy is not IRayCastable irc) continue;

                if (ray.FilterPre != null && !ray.FilterPre(node.Proxy)) continue;

                Unsafe.SkipInit(out RayCastResult res);
                bool hit = irc.RayCast(ray.Origin, ray.Direction, out res.Normal, out res.Lambda);
                res.Entity = node.Proxy;

                if (hit && res.Lambda < result.Lambda)
                {
                    if (ray.FilterPost != null && !ray.FilterPost(res)) continue;
                    result = res;
                    globalHit = true;
                }

                continue;
            }

            ref Node lNode = ref Nodes[node.Left];
            ref Node rNode = ref Nodes[node.Right];

            bool lRes = lNode.ExpandedBox.RayIntersect(ray.Origin, ray.Direction, out Real lEnter);
            bool rRes = rNode.ExpandedBox.RayIntersect(ray.Origin, ray.Direction, out Real rEnter);

            if (lEnter > result.Lambda) lRes = false;
            if (rEnter > result.Lambda) rRes = false;

            if (lRes && rRes)
            {
                if (lEnter < rEnter)
                {
                    stack.Push(node.Right);
                    stack.Push(node.Left);
                }
                else
                {
                    stack.Push(node.Left);
                    stack.Push(node.Right);
                }
            }
            else
            {
                if (lRes) stack.Push(node.Left);
                if (rRes) stack.Push(node.Right);
            }
        }

        return globalHit;
    }
}