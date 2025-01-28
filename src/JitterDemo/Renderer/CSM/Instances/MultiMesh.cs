using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// <summary>
/// ������
/// </summary>
public class MultiMesh : CSMInstance
{
    /// <summary>
    /// ��
    /// </summary>
    public readonly Mesh mesh;

    /// <summary>
    /// ���ļ��г�ʼ��һ����
    /// </summary>
    /// <param name="filename">�ļ�·��, �ı���ʽ���߽�����һ���ı��ļ��� zip �ļ�</param>
    /// <param name="scale">����</param>
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