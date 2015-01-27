using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SimpleAgent : SimpleEmptyAgent
{
	public bool DebugMode = false;

	public enum SteeringType
	{
		seek,
		flee,
		flee_from_target,
		seek_and_avoid,
		arrive,
		pursuit,
		evade,
		wander,
		interpose,
		hide,
		offPursuit,
		flocking
	}

	public LayerMask FriendLayer;
	public LayerMask EnemieLayer;
	public LayerMask WorldLayer;

	public SteeringType steeringType;
	private SteeringType last_SteeringType;
	public float PanicDistance = 5.0f;
	public float AvoidanceDistance = 2.0f;
	public float ArriveDistance = 2f;

	public float MaxMovementSpeed = 10f;
	public float MovementAcceleration = 1.0f;
	public float MovementBreak = 2.0f;
	public float Damping = 2.0f;
	public float StopDistance = 0.5f;

	public float MaxRotationSpeed = 2f;
	public float RotationAcceleration = 1.0f;
	private float RotationSpeed = 0f;
	public float MinDistanceForRotation = 0.1f;
	public float MaxAngleDiff = 90f;

	public SimpleEmptyAgent TargetAgent = null;
	public Vector3 TargetPosition = Vector3.zero;

	protected Vector3 WanderDirection = Vector3.zero;
	public float WanderJitter = 0.1f;
	public float WanderDistance = 10f;
	public float WanderRadius = 3f;

	public bool EnableObstacleAvoidance = false;
	public float AvoidancceMult = 0.2f;

	public float AngleDiff = 0f;




	#region VelocityCalculations
	public Vector3 WantedVelocity
	{
		get
		{
			switch (steeringType)
			{
				case SteeringType.seek:
					return Seek();
					break;
				case SteeringType.flee:
					return Flee();
					break;
				case SteeringType.flee_from_target:
					return FleeFromTarget();
					break;
				case SteeringType.seek_and_avoid:
					return SeekAndAvoid();
					break;
				case SteeringType.arrive:
					return Arrive();
					break;
				case SteeringType.pursuit:
					return Pursuit();
					break;
				case SteeringType.evade:
					return Evade();
					break;
				case SteeringType.wander:
					return Wander();
					break;
				case SteeringType.interpose:
					return Interpose();
					break;
				case SteeringType.hide:
					return Hide();
					break;
				case SteeringType.offPursuit:
					return OffPursuit();
					break;
				case SteeringType.flocking:
					return Flocking();
					break;
				default:
					return Seek();
					break;
			}
			return Seek();
		}
	}
	public Vector3 Seek()
	{
		return (TargetPosition - trans.position).normalized;
	}
	public Vector3 Flee()
	{
		return GetPanicTargetDirection().normalized;
	}
	public Vector3 FleeFromTarget()
	{
		return -Seek();
	}
	public Vector3 SeekAndAvoid()
	{
		return Seek() + GetPanicTargetDirection() + WorldAvoidance();
	}
	public Vector3 Arrive()
	{
		return Seek() * (Mathf.Max(Distance * (1f / ArriveDistance) - StopDistance, 0));
	}
	public Vector3 Pursuit()
	{
		if (TargetAgent)
			return (TargetPosition + TargetAgent.CurrentVelocity - trans.position).normalized;
		return Seek();
	}
	public Vector3 Evade()
	{
		return Seek();
	}
	public Vector3 Wander()
	{
		WanderDirection += Random.onUnitSphere * WanderJitter;
		WanderDirection.Normalize();
		WanderDirection *= WanderRadius;


		WanderDirection = WanderDirection + CurrentVelocity * WanderDistance;

		if (EnableObstacleAvoidance)
		{
			Vector3 lateralForce = Vector3.zero;
			lateralForce = ObstacleAvoidance();
			WanderDirection += lateralForce;
		}

		return WanderDirection;
	}
	public Vector3 Interpose()
	{
		//TODO find position between to agents
		return Seek();
	}

	public float RangeToHide = 100f;
	public float HideDistance = 2.0f;
	public Vector3 Hide()
	{
		SortedList<float, Vector3> savePositions = new SortedList<float, Vector3>();

		List<Collider> colliderToHide = new List<Collider>();
		colliderToHide.AddRange(Physics.OverlapSphere(trans.position, RangeToHide, WorldLayer));
		colliderToHide.Remove(collider);
		if(TargetAgent)
			colliderToHide.Remove(TargetAgent.collider);
		

		if (TargetAgent)
			TargetPosition += TargetAgent.CurrentVelocity;

		Vector3 hideSpot = Vector3.zero;
		Vector3 hideDirectionFromObject = Vector3.zero;
		foreach (var currentCollider in colliderToHide)
		{
			hideDirectionFromObject = (currentCollider.bounds.center - TargetPosition).normalized;


			hideSpot = currentCollider.bounds.center + hideDirectionFromObject * MaxBounds(currentCollider);

			float hitDistance = 0f;
			if(currentCollider.bounds.IntersectRay(new Ray(hideSpot, -hideDirectionFromObject), out hitDistance))
			{
				hideSpot -= hideDirectionFromObject * hitDistance;
				hideSpot += hideDirectionFromObject * HideDistance;
			}

			savePositions.Add(HideSpotHeuristic(hideSpot), hideSpot);
		}

		if (savePositions.Count == 0)
			return Vector3.zero;

		TargetPosition = savePositions[savePositions.Keys[0]];

		return SeekAndAvoid();
	}

	public Vector3 OffPursuit_Offset = Vector3.zero;
	public Vector3 OffPursuit()
	{
		Vector3 wantedPosition = TargetPosition + OffPursuit_Offset;

		if (TargetAgent)
		{
			wantedPosition = TargetPosition + OffPursuit_Offset;
			wantedPosition += TargetAgent.CurrentVelocity;
		}

		TargetPosition = wantedPosition;

		return SeekAndAvoid();
	}

	public float FlockingRadius = 10f;
	public float FlockingTargetDistance = 50f;

	public float BestNeighbourDistance = 5f;

	public Vector3 Flocking()
	{
		List<Collider> colliders = new List<Collider>();
		colliders.AddRange(Physics.OverlapSphere(trans.position, FlockingRadius, FriendLayer));
		colliders.Remove(collider);

		Vector3 wantedDirection = Vector3.zero;
		wantedDirection += Seperation(colliders);
		//wantedDirection += Alignment(colliders);
		wantedDirection += Cohesion(colliders);

		wantedDirection += (TargetPosition - trans.position);

		TargetPosition = trans.position + wantedDirection;
		return SeekAndAvoid();
	}
	public Vector3 Seperation(List<Collider> colliders)
	{
		Vector3 wantedDirection = Vector3.zero;

		Vector3 direction;
		
		foreach (var collider in colliders)
		{
			direction = trans.position - collider.bounds.center;
			if (direction.magnitude < BestNeighbourDistance)
			{
				wantedDirection += direction.normalized * BestNeighbourDistance - direction;
			}
		}
		return wantedDirection;
	}
	public Vector3 Alignment(List<Collider> colliders)
	{
		if (colliders.Count == 0)
			return Vector3.zero;

		Vector3 direction = Vector3.zero;
		int count = 0;
		SimpleEmptyAgent agent = null;
		foreach (var collider in colliders)
		{
			agent = collider.GetComponent<SimpleEmptyAgent>();
			if (agent)
			{
				direction += agent.trans.forward;
				count++;
			}
		}
		direction /= count;
		direction -= trans.forward;
		return direction;
	}

	public Vector3 Cohesion(List<Collider> colliders)
	{
		Vector3 wantedDirection = Vector3.zero;

		Vector3 direction;

		foreach (var collider in colliders)
		{
			direction = trans.position - collider.bounds.center;
			if (direction.magnitude > BestNeighbourDistance)
			{
				wantedDirection += direction.normalized * BestNeighbourDistance - direction;
			}
		}
		return wantedDirection;
	}


	public float HideSpotHeuristic(Vector3 pos)
	{
		return Vector3.Distance(pos, trans.position);
	}

	public Vector3 ObstacleAvoidance()
	{
		float distance = Mathf.Pow(CurrentVelocity.magnitude, 2f);

		const float AvoidanceRadius = 3f;
		int iterations = Mathf.CeilToInt(distance / AvoidanceRadius);

		for (int i = 1; i <= iterations; i++)
		{
			if (Physics.CheckSphere(trans.position + trans.forward * AvoidanceRadius * i, AvoidanceRadius))
			{
				Collider[] colliders = Physics.OverlapSphere(trans.position + trans.forward * AvoidanceRadius * i, AvoidanceRadius);

				Collider closest = null;

				float minDistance = -1f;
				float currDistance = 0f;
				foreach (var coll in colliders)
				{
					if (coll == collider)
						continue;

					currDistance = Vector3.Distance(trans.position, coll.transform.position);
					if (currDistance < minDistance || minDistance == -1f)
					{
						minDistance = currDistance;
						closest = coll;
					}
				}
				if (closest == null)
					continue;

				Vector3 closestPoint = closest.ClosestPointOnBounds(trans.position + trans.forward * minDistance);


				float forceMultiplier = 1f + (minDistance - Vector3.Distance(trans.position, closestPoint)) / distance;

				Vector3 steeringForce = (closestPoint - closest.transform.position) * forceMultiplier;

				Debug.DrawRay(closest.transform.position, steeringForce);

				return steeringForce;
			}
		}

		return Vector3.zero;
	}

	public float MaxBounds(Collider collider)
	{
		float maxBounds = collider.bounds.max.x;
		if (maxBounds < collider.bounds.max.y)
			maxBounds = collider.bounds.max.y;
		if (maxBounds < collider.bounds.max.z)
			maxBounds = collider.bounds.max.z;

		maxBounds = Mathf.Abs(maxBounds);
		maxBounds *= 1.5f;
		return maxBounds;
	}

	public Vector3 ColliderAvoidance(IEnumerable colliders, float distance)
	{
		if (distance == 0)
			return Vector3.zero;

		Vector3 result = Vector3.zero;
		int obstacles = 0;

		foreach (Collider worldCollider in colliders)
		{
			Vector3 testDirection = (trans.position - worldCollider.bounds.center).normalized;

			float hitDistance = 0f;
			if (worldCollider.bounds.IntersectRay(new Ray(trans.position, -testDirection), out hitDistance))
			{
				testDirection = testDirection.normalized * hitDistance;

				if (testDirection.magnitude < distance)
				{
					Vector3 direction = (trans.position - worldCollider.bounds.center).normalized * distance - testDirection;
					result += direction;
					obstacles++;

					if (DebugMode)
						Debug.DrawRay(trans.position, direction, Color.gray);
				}
			}
		}

		if (obstacles == 0)
			return result;

		result += Vector3.Cross(result, trans.up);
		result /= 2f;

		if (DebugMode)
			Debug.DrawRay(trans.position, result, Color.magenta);
		return result;
	}

	public Vector3 WorldAvoidance()
	{
		List<Collider> colliders = new List<Collider>();
		colliders.AddRange(Physics.OverlapSphere(trans.position, AvoidanceDistance, WorldLayer | FriendLayer));
		colliders.Remove(collider);
		//colliderToHide.Remove(TargetAgent.collider);
		return ColliderAvoidance(colliders, AvoidanceDistance);
	}

	public Vector3 GetPanicTargetDirection()
	{
		List<Collider> colliders = new List<Collider>();
		colliders.AddRange(Physics.OverlapSphere(trans.position, AvoidanceDistance, EnemieLayer));
		//colliders.Remove(collider);
		//colliderToHide.Remove(TargetAgent.collider);
		return ColliderAvoidance(colliders, PanicDistance);
	}

	#endregion

	public float Distance
	{
		get
		{
			return Vector3.Distance(trans.position, TargetPosition);
		}
	}

	// Use this for initialization
	public override void Start ()
	{
		base.Start();
		TargetPosition = trans.position;
		if (TargetAgent)
			TargetPosition = TargetAgent.trans.position;
		OffPursuit_Offset = trans.position - TargetPosition;
	}

	public override void Reset()
	{
		base.Reset();
		RotationSpeed = 0f;
		WanderDirection = Random.onUnitSphere;
		CurrentVelocity = Vector3.zero;
	}

	// Update is called once per frame
	void Update ()
	{
		if (steeringType == SteeringType.offPursuit && last_SteeringType != SteeringType.offPursuit)
		{
			OffPursuit_Offset = trans.position - TargetPosition;
		}
		last_SteeringType = steeringType;

		Vector3 TargetVelocity = WantedVelocity * MaxMovementSpeed;


		Vector3 TargetDirection = TargetVelocity.normalized;
		float SteeringDistance = TargetVelocity.magnitude;

		if (steeringType == SteeringType.wander)
			TargetPosition = trans.position + WanderDirection;

		float RotationDirection = RotateToTarget(TargetDirection);

		RotationSpeed = Mathf.Min(Mathf.Max(RotationSpeed + Time.deltaTime * RotationAcceleration * RotationDirection, 0f), MaxRotationSpeed);

		float CurrentDot = Mathf.Abs(Vector3.Dot(trans.forward, TargetDirection) - 1f);
		AngleDiff = CurrentDot * 180f;

		CurrentDot = Mathf.Max(MaxAngleDiff - AngleDiff, 0f) / MaxAngleDiff;


		if (CurrentDot > 0 && Distance > StopDistance)
			CurrentVelocity = Vector3.ClampMagnitude(CurrentVelocity + CurrentDot * transform.forward * MovementAcceleration * Time.deltaTime, MaxMovementSpeed);
		else
			CurrentVelocity -= CurrentVelocity * MovementBreak * Time.deltaTime;

		if(TargetDirection.magnitude > 0f)
			trans.rotation = Quaternion.Lerp(trans.rotation, Quaternion.LookRotation(TargetDirection), Time.deltaTime * RotationSpeed);

		CurrentVelocity -= CurrentVelocity * Damping * Time.deltaTime;


		trans.position += CurrentVelocity * Time.deltaTime;

		#region Debug
		if (DebugMode)
		{
			Debug.DrawRay(trans.position + trans.right, CurrentVelocity, Color.yellow);
			Debug.DrawRay(trans.position + trans.right * 0.9f, TargetDirection * MaxMovementSpeed, Color.green);
			Debug.DrawLine(trans.position, TargetPosition, Color.white);
		}
		#endregion
	}

	float RotateToTarget(Vector3 TargetDirection)
	{
		Vector3 targetVector = Vector3.zero;
		Vector3 currentVector = Vector3.zero;
		Vector3 crossResult = Vector3.zero;
		float cosAngle = 0f, turnAngle = 0f;

		targetVector = TargetDirection;
		currentVector = trans.forward;
		cosAngle = Vector3.Dot(currentVector, targetVector);
		crossResult = Vector3.Cross(currentVector, targetVector);
		crossResult.Normalize();
		turnAngle = Mathf.Acos(cosAngle);
		turnAngle = Mathf.Min(turnAngle * RotationAcceleration, MaxRotationSpeed);
		turnAngle = turnAngle * Mathf.Rad2Deg;

		if (turnAngle > 0)
			return 1f;
		else
			return -1f;
	}
}
