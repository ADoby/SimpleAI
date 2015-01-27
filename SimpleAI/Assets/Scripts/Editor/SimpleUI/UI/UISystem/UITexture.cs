using UnityEngine;
using System.Collections;

[System.Serializable]
public class UITexture : UIRect
{

    public Texture Texture;

    public ScaleMode ScaleMode;

    public override void DrawMeBeforeChildren()
    {
        GUI.DrawTexture(absoluteRect, Texture, ScaleMode);
    }
}
