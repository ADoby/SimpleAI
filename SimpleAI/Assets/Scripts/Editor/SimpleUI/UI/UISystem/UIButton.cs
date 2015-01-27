using UnityEngine;

[System.Serializable]
public class UIButton : UIRect
{

    [Multiline(3)]
    public string Text = "";

    public UIDefaultCallback Callback;

    public float fontSize = 1.0f;

    public GUIStyle ButtonStyle;

    public delegate void ButtonEvent(UIButton sender);

    public ButtonEvent OnButtonClicked;
    public static ButtonEvent OnAnyButtonClicked;

    [SerializeField]
    private bool isInitialized = false;

    [SerializeField]
    private Color normalTextColor, hoverTextColor;

    public override void DrawMeBeforeChildren()
    {
        if (!isInitialized)
        {
            if (ButtonStyle == null)
                ButtonStyle = new GUIStyle(GUI.skin.button);

            normalTextColor = ButtonStyle.normal.textColor;
            hoverTextColor = ButtonStyle.hover.textColor;
            isInitialized = true;
        }

        ButtonStyle.fontSize = (int)(absoluteRect.height / (GUITools.MaxFontSize - fontSize));

        if (forceHover)
            ButtonStyle.normal.textColor = hoverTextColor;
        else
            ButtonStyle.normal.textColor = normalTextColor;
        
        if (GUI.Button(absoluteRect, Text, ButtonStyle))
        {
            ThrowClicked();
        }
    }

    public void ThrowClicked()
    {
        if (Callback != null)
            Callback.CallBack(this);

        if (OnButtonClicked != null)
            OnButtonClicked(this);

        if (OnAnyButtonClicked != null)
            OnAnyButtonClicked(this);
    }
}
