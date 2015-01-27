using UnityEngine;
using System.Collections;

public class SimpleEmptyAgent : MonoBehaviour {


	public Transform trans { get; set; }
	public Vector3 CurrentVelocity = Vector3.zero;

	public virtual void Awake()
	{
		trans = transform;
	}

	public virtual void Start()
	{
		Reset();
	}

	// Use this for initialization
	public virtual void Reset()
	{
		
	}
}
