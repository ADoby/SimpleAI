using UnityEngine;

[System.Serializable]
public class UIText : UIRect
{
    [Multiline(5)]
    public string Text = "";

    public float fontSize = 1.0f;

    public GUIStyle FontStyle;

    [SerializeField]
    private bool isInitialized = false;

    [SerializeField]
    public Color normalTextColor, hoverTextColor;

    public override void DrawMeBeforeChildren()
    {
        if (!isInitialized)
        {
            if (FontStyle == null)
                FontStyle = new GUIStyle(GUI.skin.label);

            normalTextColor = FontStyle.normal.textColor;
            hoverTextColor = FontStyle.hover.textColor;
            isInitialized = true;
        }

        FontStyle.fontSize = (int)(absoluteRect.height / (GUITools.MaxFontSize - fontSize));
        if (forceHover)
            FontStyle.normal.textColor = hoverTextColor;
        else
            FontStyle.normal.textColor = normalTextColor;

        GUI.Label(absoluteRect, Text, FontStyle);
    }
}

