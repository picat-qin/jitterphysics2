using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// <summary>
/// 多网络
/// </summary>
public class MultiMesh : CSMInstance
{
    /// <summary>
    /// 网
    /// </summary>
    public readonly Mesh mesh;

    /// <summary>
    /// 从文件中初始化一个网
    /// </summary>
    /// <param name="filename">文件路径, 文本格式或者仅包含一个文本文件的 zip 文件</param>
    /// <param name="scale">比例</param>
    public MultiMesh(string filename, float scale = 1.0f)
    {
        mesh = Mesh.LoadMesh(filename, true);
        mesh.Transform(MatrixHelper.CreateScale(scale));
    }

    public override void LightPass(PhongShader shader)
    {
        if (mesh.Groups.Length == 0) return;

        Texture?.Bind(3);

        shader.MaterialProperties.SetDefaultMaterial();

        Vao.Bind();

        int sof = sizeof(float);

        for (int i = 0; i < mesh.Groups.Length; i++)
        {
            shader.MaterialProperties.Color.Set(ColorGenerator.GetColor(i * (i << 6)));
            shader.MaterialProperties.ColorMixing.Set(0, 1, 0);


            GLDevice.DrawElementsInstanced(DrawMode.Triangles,
                3 * (mesh.Groups[i].ToExclusive - mesh.Groups[i].FromInclusive), IndexType.UnsignedInt,
                mesh.Groups[i].FromInclusive * sof * 3, Count);
        }

        shader.MaterialProperties.SetDefaultMaterial();
    }

    public override (Vertex[] vertices, TriangleVertexIndex[] indices) ProvideVertices()
    {
        return (mesh.Vertices, mesh.Indices);
    }
}