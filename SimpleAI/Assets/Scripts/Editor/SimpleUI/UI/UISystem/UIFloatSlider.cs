using UnityEngine;

[System.Serializable]
public class UIFloatSlider : UIRect
{
    [SerializeField]
    private float lastValue = 0;
    public float currentValue = 0;

    public float minValue = 0.0f;
    public float maxValue = 1.0f;

    public UIDefaultCallback Callback;

    public delegate void FloatSliderEvent(UIFloatSlider sender);

    public FloatSliderEvent OnFloatSliderChanged;
    public static FloatSliderEvent OnAnyFloatSliderChanged;
    
    public float Procentage
    {
        get
        {
            return (currentValue - minValue) / (maxValue - minValue);
        }
    }

    public void Start()
    {
        if (Callback != null)
            Callback.CallBack(this);

        if (OnFloatSliderChanged != null)
            OnFloatSliderChanged(this);

        if (OnAnyFloatSliderChanged != null)
            OnAnyFloatSliderChanged(this);
    }

    public override void DrawMeBeforeChildren()
    {
        currentValue = Mathf.Clamp(currentValue, minValue, maxValue);
        currentValue = GUI.HorizontalSlider(absoluteRect, currentValue, minValue, maxValue);

        if (GUI.changed)
        {
            if (lastValue != currentValue)
            {
                if (Callback != null)
                    Callback.CallBack(this);

                if (OnFloatSliderChanged != null)
                    OnFloatSliderChanged(this);

                if (OnAnyFloatSliderChanged != null)
                    OnAnyFloatSliderChanged(this);

                lastValue = currentValue;
            }
        }
    }
}
