using UnityEngine;
using System.Collections;

public class LerpMovement : MonoBehaviour 
{

	public enum LerpType
	{
		SIN,
		COS
	}

	public LerpType type;

	public Vector3 StartPos, EndPos;
	public float LerpSpeed = 2.0f;

	public float LerpValue = 0f;

	public float Step = 0f;

	
	// Update is called once per frame
	void Update () {
		if (type == LerpType.SIN)
			LerpValue = (Mathf.Sin((Step + Time.time) * LerpSpeed) + 1f) / 2f;
		else if(type == LerpType.COS)
			LerpValue = (Mathf.Cos((Step + Time.time) * LerpSpeed) + 1f) / 2f;

		transform.position = Vector3.Lerp(StartPos, EndPos, LerpValue);
	}
}
