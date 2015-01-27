using UnityEngine;

public class GUITools
{
    public static float MaxFontSize = 10.0f;

    public static void BackgroundBox(string content)
    {
        GUI.Box(new Rect(10, 10, Screen.width - 20, Screen.height - 20), content);
    }

    public static bool CenteredButton(float x, float y, float width, float height, string content)
    {
        return GUI.Button(new Rect((Screen.width / 2f) - width / 2f + x, (Screen.height / 2f) - height / 2f + y, width, height), content);
    }
}

public class GUILayoutTools
{
    public static bool Button(float width, float height, string content)
    {
        return GUILayout.Button(content, GUILayout.Width(width), GUILayout.Height(height));
    }
}

