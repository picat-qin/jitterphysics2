namespace JitterDemo;

public interface ICleanDemo
{
    public void CleanUp();
}

/// <summary>
/// Demo接口
/// </summary>
public interface IDemo
{
    /// <summary>
    /// 构建
    /// </summary>
    public void Build();
    /// <summary>
    /// 渲染
    /// </summary>
    public void Draw();
    /// <summary>
    /// 名称
    /// </summary>
    public string Name { get; }
}