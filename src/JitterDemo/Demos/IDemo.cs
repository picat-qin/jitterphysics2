namespace JitterDemo;

public interface ICleanDemo
{
    public void CleanUp();
}

/// <summary>
/// Demo�ӿ�
/// </summary>
public interface IDemo
{
    /// <summary>
    /// ����
    /// </summary>
    public void Build();
    /// <summary>
    /// ��Ⱦ
    /// </summary>
    public void Draw();
    /// <summary>
    /// ����
    /// </summary>
    public string Name { get; }
}