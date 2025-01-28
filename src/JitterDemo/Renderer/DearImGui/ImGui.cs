using System;
using System.Runtime.InteropServices;
using System.Text;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer.DearImGui;

/// <summary>
/// �û�����
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
    /// ��ʼ������
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
    /// <param name="label">��ǩ</param>
    /// <param name="value">ֵ</param>
    /// <param name="min">��Сֵ</param>
    /// <param name="max">���ֵ</param>
    /// <param name="format">��ʽ</param>
    /// <param name="flags">�����־</param>
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
    /// ��ʼ
    /// </summary>
    /// <param name="name">����</param>
    /// <param name="open">�Ƿ��</param>
    /// <param name="flags">�����־</param>
    public static void Begin(string name, ref bool open, ImGuiWindowFlags flags)
    {
        PushStr(name, 0);
        byte cb = 0b0;
        if (open) cb = 0b1;
        ImGuiNative.igBegin(strPtr[0], &cb, flags);
        open = cb == 0b1;
    }

    /// <summary>
    /// ���� ini
    /// </summary>
    public static void DisableIni()
    {
        var io = ImGuiNative.igGetIO();
        io->IniFilename = (byte*)0;
    }

    /// <summary>
    /// ����һ��
    /// </summary>
    public static void TableNextColumn()
    {
        ImGuiNative.igTableNextColumn();
    }

    /// <summary>
    /// ����һ��
    /// </summary>
    public static void TableNextRow()
    {
        ImGuiNative.igTableNextRow(ImGuiTableRowFlags.None, 0);
    }

    /// <summary>
    /// �����õ�������
    /// </summary>
    /// <param name="index"></param>
    public static void TableSetColumnIndex(int index)
    {
        ImGuiNative.igTableSetColumnIndex(index);
    }

    /// <summary>
    /// ������
    /// </summary>
    public static void EndTable()
    {
        ImGuiNative.igEndTable();
    }

    /// <summary>
    /// �¿��
    /// </summary>
    public static void NewFrame()
    {
        ImGuiNative.igNewFrame();
    }

    /// <summary>
    /// ������һ�δ���λ��
    /// </summary>
    /// <param name="pos">λ��</param>
    /// <param name="cond">����</param>
    /// <param name="pivot">����</param>
    public static void SetNextWindowsPos(in Vector2 pos, ImGuiCond cond, in Vector2 pivot)
    {
        ImGuiNative.igSetNextWindowPos(pos, cond, pivot);
    }

    /// <summary>
    /// ������һ�����ڵĿ�ʼ͸����
    /// </summary>
    /// <param name="alpha"></param>
    public static void SetNextWindowBgAlpha(float alpha)
    {
        ImGuiNative.igSetNextWindowBgAlpha(alpha);
    }

    /// <summary>
    /// ���÷��
    /// </summary>
    /// <param name="windowBorderSize">���ڱ߿��С</param>
    /// <param name="frameBordersize">��ܱ߿��С</param>
    /// <param name="indentSpacing">�������</param>
    public static void SetStyle(float windowBorderSize = 1.0f, float frameBordersize = 1.0f,
        float indentSpacing = 1.0f)
    {
        ImGuiStyle* style = ImGuiNative.igGetStyle();
        style->WindowBorderSize = windowBorderSize;
        style->FrameBorderSize = frameBordersize;
        style->IndentSpacing = indentSpacing;
    }

    /// <summary>
    /// �ָ���
    /// </summary>
    public static void Separator()
    {
        ImGuiNative.igSeparator();
    }

    /// <summary>
    /// ����
    /// </summary>
    public static void End()
    {
        ImGuiNative.igEnd();
    }

    /// <summary>
    /// �������
    /// </summary>
    public static void EndFrame()
    {
        ImGuiNative.igEndFrame();
    }

    /// <summary>
    /// ��Ⱦ
    /// </summary>
    public static void Render()
    {
        ImGuiNative.igRender();
    }

    /// <summary>
    /// ����, ͬһ��
    /// </summary>
    /// <param name="offsetFromStartx"></param>
    /// <param name="spacing"></param>
    public static void SameLine(float offsetFromStartx, float spacing)
    {
        ImGuiNative.igSameLine(offsetFromStartx, spacing);
    }

    /// <summary>
    /// ��ѡ��
    /// </summary>
    /// <param name="label">��ǩ</param>
    /// <param name="value">ֵ</param>
    public static void Checkbox(string label, ref bool value)
    {
        PushStr(label, 0);
        byte cb = 0b0;
        if (value) cb = 0b1;
        ImGuiNative.igCheckbox(strPtr[0], &cb);
        value = cb == 0b1;
    }

    /// <summary>
    /// ��ȡ���ڿ��
    /// </summary>
    /// <returns></returns>
    public static float GetWindowWidth()
    {
        return ImGuiNative.igGetWindowWidth();
    }

    /// <summary>
    /// ��ť
    /// </summary>
    /// <param name="label">��ǩ</param>
    /// <param name="size">��С</param>
    /// <returns></returns>
    public static bool Button(string label, Vector2 size)
    {
        PushStr(label, 0);
        var result = ImGuiNative.igButton(strPtr[0], size);
        return Convert.ToBoolean(result);
    }

    /// <summary>
    /// ��������
    /// </summary>
    /// <param name="label">��ǩ</param>
    /// <param name="text">����</param>
    /// <param name="flags"></param>
    public static void InputText(string label, ref string text, ImGuiInputTextFlags flags)
    {
        PushStr(label, 0);
        PushStr(text, 1);
        ImGuiNative.igInputText(strPtr[0], strPtr[1], strPtrSize, flags, null, (void*)0);
        text = PopStr(1);
    }

    /// <summary>
    /// ����
    /// </summary>
    /// <param name="text">����</param>
    /// <param name="color">��ɫ</param>
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
    /// ���ͷ��ֵ
    /// </summary>
    /// <param name="var">���ֵ</param>
    /// <param name="vec2">λ������?</param>
    public static void PushStyleVar(ImGuiStyleVar var, in Vector2 vec2)
    {
        ImGuiNative.igPushStyleVar_Vec2(var, vec2);
    }

    /// <summary>
    /// ������
    /// </summary>
    /// <param name="label">��ǩ</param>
    /// <param name="flags">�б�־</param>
    /// <param name="initWidthOrHeight">��ʼ�߶Ȼ���</param>
    /// <param name="userId">�û�id</param>
    public static void SetupColumn(string label, ImGuiTableColumnFlags flags, float initWidthOrHeight, uint userId)
    {
        PushStr(label, 0);
        ImGuiNative.igTableSetupColumn(strPtr[0], flags, initWidthOrHeight, userId);
    }

    /// <summary>
    /// ��ʼһ���˵�
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
    /// �˵���
    /// </summary>
    /// <param name="label">��ǩ</param>
    /// <param name="shortcut">��ݷ�ʽ</param>
    /// <param name="selected">�Ƿ�ѡ��</param>
    /// <param name="enabled">����</param>
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
    /// �����˵���
    /// </summary>
    public static void EndMenu()
    {
        ImGuiNative.igEndMenu();
    }

    /// <summary>
    /// �ı�
    /// </summary>
    /// <param name="text"></param>
    public static void Text(string text)
    {
        PushStr(text, 0);
        ImGuiNative.igText(strPtr[0]);
    }

    /// <summary>
    /// ���ڵ�
    /// </summary>
    /// <param name="label">��ǩ</param>
    /// <returns></returns>
    public static bool TreeNode(string label)
    {
        PushStr(label, 0);
        return Convert.ToBoolean(ImGuiNative.igTreeNode_Str(strPtr[0]));
    }

    /// <summary>
    /// ������һ����
    /// </summary>
    /// <param name="width"></param>
    public static void SetNextItemWidth(float width)
    {
        ImGuiNative.igSetNextItemWidth(width);
    }

    /// <summary>
    /// ������
    /// </summary>
    public static void TreePop()
    {
        ImGuiNative.igTreePop();
    }

    /// <summary>
    /// ��һ�����ڵ��
    /// </summary>
    /// <param name="open"></param>
    public static void NextTreeNodeOpen(bool open)
    {
        ImGuiNative.igSetNextItemOpen(Convert.ToByte(open), ImGuiCond.FirstUseEver);
    }

    /// <summary>
    /// ��һ���
    /// </summary>
    /// <param name="open"></param>
    public static void NextItemOpen(bool open)
    {
        ImGuiNative.igSetNextItemOpen(Convert.ToByte(open), ImGuiCond.FirstUseEver);
    }

    /// <summary>
    /// ͼ��ֱ��ͼ
    /// </summary>
    /// <param name="array">������</param>
    /// <param name="label">��ǩ</param>
    /// <param name="overlayText">չʾ����</param>
    /// <param name="min">��Сֵ</param>
    /// <param name="max">���ֵ</param>
    /// <param name="sizeX">X �ߴ�</param>
    /// <param name="sizeY">Y �ߴ�</param>
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
    /// չʾDemo
    /// </summary>
    public static void ShowDemo()
    {
        byte popen = 1;
        ImGuiNative.igShowDemoWindow(&popen);
    }
}