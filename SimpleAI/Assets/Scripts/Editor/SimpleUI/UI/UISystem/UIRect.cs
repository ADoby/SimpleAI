using System;
using System.Linq;
using UnityEngine;
using System.Collections.Generic;

#region Enum and Class Definitions

public enum HorizontalAnchorPoint
{
    LEFT,
    CENTER,
    RIGHT
}

public enum VerticalAnchorPoint
{
    TOP,
    CENTER,
    BOTTOM
}

[System.Serializable]
public class UIPosition
{
    public bool normalized = false;
    public Vector2 Value = Vector2.zero;

    public UIPosition(float x, float y, bool normalized)
    {
        this.normalized = normalized;
        Value.x = x;
        Value.y = y;
    }

    public void SetPosition(float x, float y)
    {
        Value.x = x;
        Value.y = y;
    }
}
[System.Serializable]
public class UISize
{
    public bool normalized = false;
    public Vector2 Value = Vector2.zero;

    public UISize(float x, float y, bool normalized)
    {
        this.normalized = normalized;
        Value.x = x;
        Value.y = y;
    }

    public void SetSize(float x, float y)
    {
        Value.x = x;
        Value.y = y;
    }
}

#endregion

public class UIIdentifierTools
{
    public static string GetIdentifier()
    {
        return Guid.NewGuid().ToString();
    }
}

[System.Serializable]
public class UIRect : MonoBehaviour
{

    #region Public Member

    public string identifier = UIIdentifierTools.GetIdentifier();

    [SerializeField]
    private bool visible = true;
    public bool Visible
    {
        get
        {
            return visible;
        }
        set
        {
            if(value != visible)
            {
                visible = value;
                if (visible)
                    OnShowing();
                else
                    OnHiding();
            }
        }
    }

    public bool Enabled = true;
    public bool ShowBackground = false;
    public Vector2 RelativePosition = Vector2.zero;
    public Vector2 AbsolutePosition = Vector2.zero;
    public Vector2 RelativeSize = Vector2.zero;
    public bool RelativeSizeBasedOnScreen = false;
    public bool RelativeWidthBasedOnHeight = false;
    public Vector2 AbsoluteSize = Vector2.zero;

    public bool forceHover = false;

    public void SetForceHover(bool value)
    {
        forceHover = value;
        for (int i = 0; i < children.Count; i++)
        {
            children[i].SetForceHover(value);
        }
    }

    public void SetRelativePosition(float x, float y)
    {
        RelativePosition.x = x;
        RelativePosition.y = y;
    }

    public void SetAbsolutePosition(float x, float y)
    {
        AbsolutePosition.x = x;
        AbsolutePosition.y = y;
    }

    public void SetRelativeSize(float x, float y)
    {
        RelativeSize.x = x;
        RelativeSize.y = y;
    }

    public void SetAbsoluteSize(float x, float y)
    {
        AbsoluteSize.x = x;
        AbsoluteSize.y = y;
    }

    public HorizontalAnchorPoint HorizontalAnchor;
    public VerticalAnchorPoint VerticalAnchor;

    public HorizontalAnchorPoint HorizontalAlignment;
    public VerticalAnchorPoint VerticalAlignment;

    public Rect absoluteRect;

    public UIRect parent;

    [SerializeField]
    public List<UIRect> children = new List<UIRect>();

    #endregion

    #region Children

    public List<UIRect> GetChildren()
    {
        return children;
    }

    public void AddChild(UIRect newChild)
    {
        children.Add(newChild);
        if (newChild.parent != null && newChild.parent != this)
        {
            newChild.parent.RemoveChild(newChild);
        }
        newChild.parent = this;
    }

    public void RemoveChild(UIRect oldChild)
    {
        if (children.Contains(oldChild))
            children.Remove(oldChild);
    }

    public int ChildCount = 0;

    [ContextMenu("UpdateChildren")]
    public void UpdateChildren()
    {
        children.Clear();

        var newChildren = gameObject.GetComponentsInChildren<UIRect>(includeInactive:true).Where(o => o.transform.parent == transform);

        foreach (var child in newChildren)
        {
            AddChild(child);
            child.UpdateChildren();
        }

        ChildCount = children.Count;
    }

    #endregion


    #region Virtual Functions

    public virtual void OnHiding()
    {

    }
    public virtual void OnShowing()
    {

    }

    public virtual void DrawMeBeforeChildren() 
    { 
        //Override me
    }

    public virtual void DrawMeAfterChildern()
    {

    }

    public void Draw()
    {
        if (ShowBackground)
        {
            GUI.Box(absoluteRect, "");
        }
        
        DrawMeBeforeChildren();
    }

    #endregion

    #region Public Functions

    public void SetActive(bool newActive, bool recursive)
    {
        Enabled = newActive;
        if (recursive)
        {
            foreach (var child in children)
            {
                child.SetActive(newActive, recursive);
            }
        }
    }

    public void UpdateUI()
    {
        if (!gameObject.activeSelf)
            return;

        UpdateRect();

        if (!Enabled && GUI.enabled)
        {
            GUI.enabled = false;
        }

        if (Visible)
        {
            Draw();

            foreach (var child in children)
            {
                if (child)
                    child.UpdateUI();
            }

            DrawMeAfterChildern();
        }
        

        if (!Enabled)
        {
            GUI.enabled = true;
        }
    }

    public void UpdateRects()
    {
        UpdateRect();
        for (int i = 0; i < children.Count; i++)
        {
            children[i].UpdateRects();
        }
    }

    #endregion


    #region Protected Functions

    public void UpdateRect()
    {
        absoluteRect = new Rect(AbsolutePosition.x, AbsolutePosition.y, AbsoluteSize.x, AbsoluteSize.y);
        if (parent == null)
        {
            Rect relRect = GetRelativeRect(Screen.width, Screen.height);
            absoluteRect.x += relRect.x;
            absoluteRect.y += relRect.y;
            absoluteRect.width += relRect.width;
            absoluteRect.height += relRect.height;
        }
        else
        {

            Rect relRect;
            if (RelativeSizeBasedOnScreen)
                relRect = GetRelativeRect(Screen.width, Screen.height);
            else
                relRect = GetRelativeRect(parent.absoluteRect.width, parent.absoluteRect.height);

            absoluteRect.x += relRect.x;
            absoluteRect.y += relRect.y;
            absoluteRect.width += relRect.width;
            absoluteRect.height += relRect.height;

            absoluteRect.x += parent.absoluteRect.x;
            absoluteRect.y += parent.absoluteRect.y;

            switch (HorizontalAnchor)
            {
                case HorizontalAnchorPoint.LEFT:
                    break;
                case HorizontalAnchorPoint.CENTER:
                    absoluteRect.x += parent.absoluteRect.width * 0.5f;
                    break;
                case HorizontalAnchorPoint.RIGHT:
                    absoluteRect.x += parent.absoluteRect.width;
                    break;
                default:
                    break;
            }

            switch (VerticalAnchor)
            {
                case VerticalAnchorPoint.TOP:
                    break;
                case VerticalAnchorPoint.CENTER:
                    absoluteRect.y += parent.absoluteRect.height * 0.5f;
                    break;
                case VerticalAnchorPoint.BOTTOM:
                    absoluteRect.y += parent.absoluteRect.height;
                    break;
                default:
                    break;
            }

            switch (HorizontalAlignment)
            {
                case HorizontalAnchorPoint.LEFT:
                    break;
                case HorizontalAnchorPoint.CENTER:
                    absoluteRect.x -= absoluteRect.width * 0.5f;
                    break;
                case HorizontalAnchorPoint.RIGHT:
                    absoluteRect.x -= absoluteRect.width;
                    break;
                default:
                    break;
            }

            switch (VerticalAlignment)
            {
                case VerticalAnchorPoint.TOP:
                    break;
                case VerticalAnchorPoint.CENTER:
                    absoluteRect.y -= absoluteRect.height * 0.5f;
                    break;
                case VerticalAnchorPoint.BOTTOM:
                    absoluteRect.y -= absoluteRect.height;
                    break;
                default:
                    break;
            }
        }
    }

    protected Rect GetRelativeRect(float width, float height)
    {
        Rect relativeRect = new Rect(RelativePosition.x, RelativePosition.y, RelativeSize.x, RelativeSize.y);
        relativeRect.x *= width;
        relativeRect.y *= height;
        relativeRect.height *= height;

        if (RelativeWidthBasedOnHeight)
        {
            relativeRect.width *= relativeRect.height + absoluteRect.height;
        }
        else
        {
            relativeRect.width *= width;
        }
        
        return relativeRect;
    }

    #endregion
}
