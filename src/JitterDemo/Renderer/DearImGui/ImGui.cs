using System;
using System.Runtime.InteropServices;
using System.Text;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer.DearImGui;

/// <summary>
/// 用户界面
/// </summary>
public static unsafe class ImGui
{
    // Allocate three temporary buffers on the heap. This is slower than on the stack but saves us
    // from calling stackalloc in every method. - "1K is more memory than anyone will ever need"
    private static readonly byte** strPtr;
    private const int strPtrSize = 1024;

    static ImGui()
    {
        strPtr = (byte**)NativeMemory.Alloc((nuint)(sizeof(IntPtr) * 3));

        for (int i = 0; i < 3; i++)
        {
            strPtr[i] = (byte*)NativeMemory.Alloc(strPtrSize);
        }
    }

    private static void PushStr(in string str, int index)
    {
        fixed (char* c = str)
        {
            int bc0 = Encoding.UTF8.GetByteCount(c, str.Length);
            if (bc0 + 1 > strPtrSize) throw new ArgumentException("UTF8 endcoded string too long.");
            Encoding.UTF8.GetBytes(c, str.Length, strPtr[index], bc0);
            strPtr[index][bc0] = 0;
        }
    }

    private static string PopStr(int index)
    {
        int len = 0;
        byte* ptr = strPtr[index];

        while (*ptr != 0)
        {
            if (++len == strPtrSize)
            {
                return string.Empty;
            }

            ptr += 1;
        }

        return Encoding.UTF8.GetString(strPtr[index], len);
    }

    /// <summary>
    /// 开始工作区
    /// </summary>
    /// <param name="str_id"></param>
    /// <param name="column"></param>
    /// <param name="flags"></param>
    /// <param name="outerSize"></param>
    /// <param name="innerWidth"></param>
    /// <returns></returns>
    public static bool BeginTable(string str_id, int column, ImGuiTableFlags flags, Vector2 outerSize, float innerWidth)
    {
        PushStr(str_id, 0);
        byte result = ImGuiNative.igBeginTable(strPtr[0], column, flags, outerSize, innerWidth);
        return Convert.ToBoolean(result);
    }

    /// <summary>
    /// Slider
    /// </summary>
    /// <param name="label">标签</param>
    /// <param name="value">值</param>
    /// <param name="min">最小值</param>
    /// <param name="max">最大值</param>
    /// <param name="format">格式</param>
    /// <param name="flags">滑块标志</param>
    /// <returns></returns>
    public static bool Slider(string label, ref int value, int min, int max, string format, ImGuiSliderFlags flags)
    {
        PushStr(label, 0);
        PushStr(format, 1);

        fixed (int* ptr = &value)
        {
            byte result = ImGuiNative.igSliderInt(strPtr[0], ptr, min, max, strPtr[1], flags);
            return Convert.ToBoolean(result);
        }
    }
    
    /// <summary>
    /// 开始
    /// </summary>
    /// <param name="name">名称</param>
    /// <param name="open">是否打开</param>
    /// <param name="flags">窗体标志</param>
    public static void Begin(string name, ref bool open, ImGuiWindowFlags flags)
    {
        PushStr(name, 0);
        byte cb = 0b0;
        if (open) cb = 0b1;
        ImGuiNative.igBegin(strPtr[0], &cb, flags);
        open = cb == 0b1;
    }

    /// <summary>
    /// 禁用 ini
    /// </summary>
    public static void DisableIni()
    {
        var io = ImGuiNative.igGetIO();
        io->IniFilename = (byte*)0;
    }

    /// <summary>
    /// 表下一列
    /// </summary>
    public static void TableNextColumn()
    {
        ImGuiNative.igTableNextColumn();
    }

    /// <summary>
    /// 表下一行
    /// </summary>
    public static void TableNextRow()
    {
        ImGuiNative.igTableNextRow(ImGuiTableRowFlags.None, 0);
    }

    /// <summary>
    /// 表设置的列索引
    /// </summary>
    /// <param name="index"></param>
    public static void TableSetColumnIndex(int index)
    {
        ImGuiNative.igTableSetColumnIndex(index);
    }

    /// <summary>
    /// 结束表
    /// </summary>
    public static void EndTable()
    {
        ImGuiNative.igEndTable();
    }

    /// <summary>
    /// 新框架
    /// </summary>
    public static void NewFrame()
    {
        ImGuiNative.igNewFrame();
    }

    /// <summary>
    /// 设置下一次窗口位置
    /// </summary>
    /// <param name="pos">位置</param>
    /// <param name="cond">条件</param>
    /// <param name="pivot">枢轴</param>
    public static void SetNextWindowsPos(in Vector2 pos, ImGuiCond cond, in Vector2 pivot)
    {
        ImGuiNative.igSetNextWindowPos(pos, cond, pivot);
    }

    /// <summary>
    /// 设置下一个窗口的开始透明度
    /// </summary>
    /// <param name="alpha"></param>
    public static void SetNextWindowBgAlpha(float alpha)
    {
        ImGuiNative.igSetNextWindowBgAlpha(alpha);
    }

    /// <summary>
    /// 设置风格
    /// </summary>
    /// <param name="windowBorderSize">窗口边框大小</param>
    /// <param name="frameBordersize">框架边框大小</param>
    /// <param name="indentSpacing">缩进间距</param>
    public static void SetStyle(float windowBorderSize = 1.0f, float frameBordersize = 1.0f,
        float indentSpacing = 1.0f)
    {
        ImGuiStyle* style = ImGuiNative.igGetStyle();
        style->WindowBorderSize = windowBorderSize;
        style->FrameBorderSize = frameBordersize;
        style->IndentSpacing = indentSpacing;
    }

    /// <summary>
    /// 分隔符
    /// </summary>
    public static void Separator()
    {
        ImGuiNative.igSeparator();
    }

    /// <summary>
    /// 结束
    /// </summary>
    public static void End()
    {
        ImGuiNative.igEnd();
    }

    /// <summary>
    /// 结束框架
    /// </summary>
    public static void EndFrame()
    {
        ImGuiNative.igEndFrame();
    }

    /// <summary>
    /// 渲染
    /// </summary>
    public static void Render()
    {
        ImGuiNative.igRender();
    }

    /// <summary>
    /// 共线, 同一行
    /// </summary>
    /// <param name="offsetFromStartx"></param>
    /// <param name="spacing"></param>
    public static void SameLine(float offsetFromStartx, float spacing)
    {
        ImGuiNative.igSameLine(offsetFromStartx, spacing);
    }

    /// <summary>
    /// 勾选框
    /// </summary>
    /// <param name="label">标签</param>
    /// <param name="value">值</param>
    public static void Checkbox(string label, ref bool value)
    {
        PushStr(label, 0);
        byte cb = 0b0;
        if (value) cb = 0b1;
        ImGuiNative.igCheckbox(strPtr[0], &cb);
        value = cb == 0b1;
    }

    /// <summary>
    /// 获取窗口宽度
    /// </summary>
    /// <returns></returns>
    public static float GetWindowWidth()
    {
        return ImGuiNative.igGetWindowWidth();
    }

    /// <summary>
    /// 按钮
    /// </summary>
    /// <param name="label">标签</param>
    /// <param name="size">大小</param>
    /// <returns></returns>
    public static bool Button(string label, Vector2 size)
    {
        PushStr(label, 0);
        var result = ImGuiNative.igButton(strPtr[0], size);
        return Convert.ToBoolean(result);
    }

    /// <summary>
    /// 输入文字
    /// </summary>
    /// <param name="label">标签</param>
    /// <param name="text">文字</param>
    /// <param name="flags"></param>
    public static void InputText(string label, ref string text, ImGuiInputTextFlags flags)
    {
        PushStr(label, 0);
        PushStr(text, 1);
        ImGuiNative.igInputText(strPtr[0], strPtr[1], strPtrSize, flags, null, (void*)0);
        text = PopStr(1);
    }

    /// <summary>
    /// 文字
    /// </summary>
    /// <param name="text">文字</param>
    /// <param name="color">颜色</param>
    public static void Text(string text, in Vector4 color)
    {
        ImGuiStyle* style = ImGuiNative.igGetStyle();
        Vector4* colors = &style->Colors_0;
        var orig = colors[(int)ImGuiCol.Text];
        colors[(int)ImGuiCol.Text] = color;
        Text(text);
        colors[(int)ImGuiCol.Text] = orig;
    }

    /// <summary>
    /// 推送风格值
    /// </summary>
    /// <param name="var">风格值</param>
    /// <param name="vec2">位置向量?</param>
    public static void PushStyleVar(ImGuiStyleVar var, in Vector2 vec2)
    {
        ImGuiNative.igPushStyleVar_Vec2(var, vec2);
    }

    /// <summary>
    /// 设置列
    /// </summary>
    /// <param name="label">标签</param>
    /// <param name="flags">列标志</param>
    /// <param name="initWidthOrHeight">初始高度或宽度</param>
    /// <param name="userId">用户id</param>
    public static void SetupColumn(string label, ImGuiTableColumnFlags flags, float initWidthOrHeight, uint userId)
    {
        PushStr(label, 0);
        ImGuiNative.igTableSetupColumn(strPtr[0], flags, initWidthOrHeight, userId);
    }

    /// <summary>
    /// 开始一个菜单
    /// </summary>
    /// <param name="label"></param>
    /// <param name="enabled"></param>
    /// <returns></returns>
    public static bool BeginMenu(string label, bool enabled)
    {
        PushStr(label, 0);
        return Convert.ToBoolean(ImGuiNative.igBeginMenu(strPtr[0], Convert.ToByte(enabled)));
    }

    /// <summary>
    /// 菜单项
    /// </summary>
    /// <param name="label">标签</param>
    /// <param name="shortcut">快捷方式</param>
    /// <param name="selected">是否被选择</param>
    /// <param name="enabled">启用</param>
    /// <returns></returns>
    public static bool MenuItem(string label, string shortcut, bool selected, bool enabled)
    {
        PushStr(label, 0);
        PushStr(shortcut, 1);
        return Convert.ToBoolean(
            ImGuiNative.igMenuItem_Bool(strPtr[0], strPtr[1],
                Convert.ToByte(selected), Convert.ToByte(enabled)));
    }

    /// <summary>
    /// 结束菜单项
    /// </summary>
    public static void EndMenu()
    {
        ImGuiNative.igEndMenu();
    }

    /// <summary>
    /// 文本
    /// </summary>
    /// <param name="text"></param>
    public static void Text(string text)
    {
        PushStr(text, 0);
        ImGuiNative.igText(strPtr[0]);
    }

    /// <summary>
    /// 树节点
    /// </summary>
    /// <param name="label">标签</param>
    /// <returns></returns>
    public static bool TreeNode(string label)
    {
        PushStr(label, 0);
        return Convert.ToBoolean(ImGuiNative.igTreeNode_Str(strPtr[0]));
    }

    /// <summary>
    /// 设置下一项宽度
    /// </summary>
    /// <param name="width"></param>
    public static void SetNextItemWidth(float width)
    {
        ImGuiNative.igSetNextItemWidth(width);
    }

    /// <summary>
    /// 树弹出
    /// </summary>
    public static void TreePop()
    {
        ImGuiNative.igTreePop();
    }

    /// <summary>
    /// 下一个树节点打开
    /// </summary>
    /// <param name="open"></param>
    public static void NextTreeNodeOpen(bool open)
    {
        ImGuiNative.igSetNextItemOpen(Convert.ToByte(open), ImGuiCond.FirstUseEver);
    }

    /// <summary>
    /// 下一项打开
    /// </summary>
    /// <param name="open"></param>
    public static void NextItemOpen(bool open)
    {
        ImGuiNative.igSetNextItemOpen(Convert.ToByte(open), ImGuiCond.FirstUseEver);
    }

    /// <summary>
    /// 图形直方图
    /// </summary>
    /// <param name="array">数据组</param>
    /// <param name="label">标签</param>
    /// <param name="overlayText">展示文字</param>
    /// <param name="min">最小值</param>
    /// <param name="max">最大值</param>
    /// <param name="sizeX">X 尺寸</param>
    /// <param name="sizeY">Y 尺寸</param>
    public static void PlotHistogram(float[] array, string label, string overlayText, float min, float max, float sizeX,
        float sizeY)
    {
        PushStr(label, 0);
        PushStr(overlayText, 1);

        fixed (float* ptr = array)
        {
            ImGuiNative.igPlotHistogram_FloatPtr(strPtr[0], ptr, array.Length, 0, strPtr[1], min, max,
                new Vector2(sizeX, sizeY), sizeof(float));
        }
    }

    /// <summary>
    /// 展示Demo
    /// </summary>
    public static void ShowDemo()
    {
        byte popen = 1;
        ImGuiNative.igShowDemoWindow(&popen);
    }
}