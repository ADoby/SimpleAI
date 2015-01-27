using UnityEngine;

[System.Serializable]
public class UIDefaultCallback
{
    public GameObject MessageReciever;
    public string MethodName = "";

    public virtual void CallBack(UIRect sender)
    {
        if(MessageReciever)
            MessageReciever.SendMessage(MethodName, sender, SendMessageOptions.DontRequireReceiver);
    }
}
