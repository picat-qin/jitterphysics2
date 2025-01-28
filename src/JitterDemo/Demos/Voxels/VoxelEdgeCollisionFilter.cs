using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

namespace JitterDemo;

/// <summary>
/// ���ر�Ե��ײ������
/// </summary>
public class VoxelEdgeCollisionFilter : INarrowPhaseFilter
{
    //TODO: magnet ����������ע��
    /// <summary>
    /// ��ֵ
    /// </summary>
    public float Threshold { get; set; } = 0.1f;

    //TODO: magnet ����������ע��
    /// <summary>
    /// ������
    /// </summary>
    /// <param name="shapeA">���� A </param>
    /// <param name="shapeB">���� B </param>
    /// <param name="pAA">���� A λ��</param>
    /// <param name="pBB">���� B λ��</param>
    /// <param name="normal">������</param>
    /// <param name="penetration">��͸ϵ��</param>
    /// <returns></returns>
    public bool Filter(RigidBodyShape shapeA, RigidBodyShape shapeB, ref JVector pAA, ref JVector pBB, ref JVector normal,
        ref float penetration)
    {
        VoxelShape? vs1 = shapeA as VoxelShape;
        VoxelShape? vs2 = shapeB as VoxelShape;

        bool c1 = vs1 != null;
        bool c2 = vs2 != null;

        // both shapes are voxels or both of them are not -> return
        if (c1 == c2) return true;

        VoxelShape vshape = c1 ? vs1! : vs2!;


        float trsh = Threshold;
        uint nb = vshape.Neighbours;

        JVector cnormal = normal;
        JVector relPos = pAA;

        if (c2)
        {
            relPos = pBB;
            cnormal.Negate();
        }

        relPos -= vshape.Position;

        // Check if collision normal points into a specific direction.
        // If yes, check if there is a neighbouring voxel. If yes,
        // discard the collision.
        //
        // equivalent, but harder to debug:

        /*
        return  !((relPos.X > 0.0f && cnormal.X > trsh && (nb & 1) != 0) ||
                (relPos.X < 0.0f && cnormal.X < -trsh && (nb & 2) != 0) ||
                (relPos.Y > 0.0f && cnormal.Y > trsh && (nb & 4) != 0) ||
                (relPos.Y < 0.0f && cnormal.Y < -trsh && (nb & 8) != 0) ||
                (relPos.Z > 0.0f && cnormal.Z > trsh && (nb & 16) != 0) ||
                (relPos.Z < 0.0f && cnormal.Z < -trsh && (nb & 32) != 0));
        */

        if (relPos.X > 0.0f && cnormal.X > trsh)
        {
            if ((nb & 1) != 0) return false;
        }

        if (relPos.X < 0.0f && cnormal.X < -trsh)
        {
            if ((nb & 2) != 0) return false;
        }

        if (relPos.Y > 0.0f && cnormal.Y > trsh)
        {
            if ((nb & 4) != 0) return false;
        }

        if (relPos.Y < 0.0f && cnormal.Y < -trsh)
        {
            if ((nb & 8) != 0) return false;
        }

        if (relPos.Z > 0.0f && cnormal.Z > trsh)
        {
            if ((nb & 16) != 0) return false;
        }

        if (relPos.Z < 0.0f && cnormal.Z < -trsh)
        {
            if ((nb & 32) != 0) return false;
        }

        return true;
    }
}