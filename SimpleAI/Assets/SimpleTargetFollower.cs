using UnityEngine;
using System.Collections;

public class SimpleTargetFollower : MonoBehaviour 
{

	public SimpleEmptyAgent targetAgent;
	public SimpleAgent agent;

	void Awake()
	{
		agent = GetComponent<SimpleAgent>();
		Do();	
	}

	void Start()
	{
		Do();	
	}

	// Update is called once per frame
	void Update () {
		Do();	
	}

	void Do()
	{
		if (agent && targetAgent && targetAgent.trans)
		{
			agent.TargetAgent = targetAgent;
			agent.TargetPosition = targetAgent.trans.position;
		}
	}
}
