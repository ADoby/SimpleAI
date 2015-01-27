using UnityEngine;
using System.Collections;
using UnityEditor;

public class UICreator : Editor{

    [MenuItem("SimpleUI/Create/UIEditorPanel")]
    private static void CreateUIEditorPanel()
    {
        CreateObject<UIEditorPanel>();
    }

    [MenuItem("SimpleUI/Create/UIPanel")]
    private static void CreateUIPanel()
    {
        CreateObject<UIPanel>();
    }

    [MenuItem("SimpleUI/Create/UIScrollPanel")]
    private static void CreateUIScrollPanel()
    {
        CreateObject<UIScrollPanel>();
    }

    [MenuItem("SimpleUI/Create/UIText")]
    private static void CreateUIText()
    {
        CreateObject<UIText>();
    }

    [MenuItem("SimpleUI/Create/UIButton")]
    private static void CreateUIButton()
    {
        CreateObject<UIButton>();
    }

    [MenuItem("SimpleUI/Create/UIFloatSlider")]
    private static void CreateUIFloatSlider()
    {
        CreateObject<UIFloatSlider>();
    }

    [MenuItem("SimpleUI/Create/UITexture")]
    private static void CreateUITexture()
    {
        CreateObject<UITexture>();
    }

    private static void CreateObject<T>() where T : MonoBehaviour
    {
        GameObject go = new GameObject();
        go.name = typeof(T).ToString();
        go.AddComponent<T>();
        if (Selection.activeTransform)
        {
            go.transform.parent = Selection.activeTransform;
        }
    }
}
