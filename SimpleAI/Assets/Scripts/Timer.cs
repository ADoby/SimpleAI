using UnityEngine;

[System.Serializable]
public class Timer {

	public float Value = 0f;
	private float timer = 0f;

	public float CurrentTime
	{
		get
		{
			return timer;
		}
	}

	// Use this for initialization
	void Start () 
	{
		timer = 0f;
	}
	public void Reset()
	{
		timer = 0f;
	}
	public void Finish()
	{
		timer = Value;
	}
	// Update is called once per frame
	public bool Update ()
	{
		timer = Mathf.Min(timer + Time.deltaTime, Value);
		return Finished;
	}
	public float Procentage
	{
		get
		{
			if (Value == 0)
				return 1f;
			return timer / Value;
		}
	}
	public bool Finished
	{
		get
		{
			return Procentage == 1;
		}
	}
}
