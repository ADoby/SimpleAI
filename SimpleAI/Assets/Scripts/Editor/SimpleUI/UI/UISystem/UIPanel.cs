using UnityEngine;

[System.Serializable]
public class UIPanel : UIRect
{
    public bool ShowBox = true;
    public string content = "";

    public GUIStyle BoxStyle;

    public override void DrawMeBeforeChildren()
    {
        if (BoxStyle == null)
            BoxStyle = new GUIStyle(GUI.skin.box);

        if (ShowBox)
            GUI.Box(absoluteRect, content, BoxStyle);
    }
}
