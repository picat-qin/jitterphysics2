using System.IO;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// <summary>
/// 三角网
/// </summary>
public class TriangleMesh : CSMInstance
{
    /// <summary>
    /// 网
    /// </summary>
    public readonly Mesh mesh;

    /// <summary>
    /// 颜色
    /// </summary>
    public Vector3 Color { get; set; }

    /// <summary>
    /// 通过 obj 文件创建网
    /// </summary>
    /// <param name="objFile">文件路径</param>
    /// <param name="scale">比例</param>
    public TriangleMesh(string objFile, float scale = 1.0f)
    {
        string filename = Path.Combine("assets", objFile);
        mesh = Mesh.LoadMesh(filename, true);
        mesh.Transform(MatrixHelper.CreateScale(scale));
    }

    public override (Vertex[] vertices, TriangleVertexIndex[] indices) ProvideVertices()
    {
        return (mesh.Vertices, mesh.Indices);
    }

    public override void LightPass(PhongShader shader)
    {
        //Renderer.OpenGL.Native.GL.PolygonMode(Renderer.OpenGL.Native.GLC.FRONT_AND_BACK, Renderer.OpenGL.Native.GLC.LINE);
        base.LightPass(shader);
        //Renderer.OpenGL.Native.GL.PolygonMode(Renderer.OpenGL.Native.GLC.FRONT_AND_BACK, Renderer.OpenGL.Native.GLC.FILL);
    }
}