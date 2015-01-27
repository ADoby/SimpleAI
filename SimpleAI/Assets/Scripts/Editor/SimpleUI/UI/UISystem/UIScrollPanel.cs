using UnityEngine;

[System.Serializable]
public class UIScrollPanel : UIRect
{
    public bool ShowBox = true;
    public string content = "";

    public Rect Padding;

    public GUIStyle BoxStyle;

    public Vector2 scrollPosition = Vector2.zero;

    public GUIStyle HorizontalScrollBarStyle, VerticalScrollBarStyle;

    public float RelHeight = 0f;

    public float Height = 0f;

    public float RelWidth = 0f;
    public float Width;

    public bool autoSize = false;

    void Start()
    {
        if (!autoSize)
            return;

        float minX = 0, minY = 0, maxX = 0, maxY = 0;

        foreach (var child in children)
        {
            if (child.absoluteRect.x < minX)
                minX = child.absoluteRect.x;
            if (child.absoluteRect.y < minY)
                minY = child.absoluteRect.y;

            if (child.absoluteRect.x + child.absoluteRect.width > maxX)
                maxX = child.absoluteRect.x + child.absoluteRect.width;
            if (child.absoluteRect.y + child.absoluteRect.height > maxY)
                maxY = child.absoluteRect.y + child.absoluteRect.height;
        }

        Debug.Log(string.Format("MinX: {0} MaxX: {1} Width: {2}", minX, maxX, absoluteRect.width));

        RelWidth = Mathf.Abs(maxX - minX) / absoluteRect.width;
        RelHeight = Mathf.Abs(maxY - minY) / absoluteRect.height;
    }

    public override void DrawMeBeforeChildren()
    {
        if (BoxStyle == null)
            BoxStyle = new GUIStyle(GUI.skin.box);
        if (HorizontalScrollBarStyle == null)
            HorizontalScrollBarStyle = new GUIStyle(GUI.skin.horizontalScrollbar);
        if (VerticalScrollBarStyle == null)
            VerticalScrollBarStyle = new GUIStyle(GUI.skin.verticalScrollbar);

        if (ShowBox)
            GUI.Box(absoluteRect, content, BoxStyle);

        RelHeight = Mathf.Clamp(RelHeight, 1f, 100f);
        Height = absoluteRect.height * RelHeight;
        RelWidth = Mathf.Clamp(RelWidth, 1f, 100f);
        Width = absoluteRect.width * RelWidth;

        scrollPosition = GUI.BeginScrollView(
            new Rect(absoluteRect.x + Padding.x, absoluteRect.y + Padding.y, absoluteRect.width + Padding.width, absoluteRect.height + Padding.height), 
            scrollPosition,
            new Rect(absoluteRect.x, absoluteRect.y, absoluteRect.width, Height), 
            false, true);
    }

    public override void DrawMeAfterChildern()
    {
        GUI.EndScrollView();
    }
}