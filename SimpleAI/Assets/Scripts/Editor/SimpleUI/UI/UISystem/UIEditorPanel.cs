using UnityEngine;

[ExecuteInEditMode]
public class UIEditorPanel : UIPanel
{

    void Start()
    {
        if(Application.isPlaying)
            UpdateChildren();
    }

    void OnGUI()
    {
        if (!Application.isPlaying)
            UpdateChildren();

        UpdateUI();
    }
}
