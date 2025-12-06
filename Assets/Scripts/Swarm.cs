using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;

    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    
    public float alignmentWeight = 0.5f;

    public float cohesionWeight = 1f;

    public float goalWeight = 1f;

    public float obstacleWeight = 0.9f;

    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f;

    public float initializationRadius = 1.0f;

    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;

    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;


    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        InitBoids();
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];
        
        for (int i = 0; i < numberOfBoids; i++) //for every boid give it a random position and random forward heading
        {
            //Random position (local space)
            Vector3 ranPos = Random.insideUnitSphere * initializationRadius;
            boids[i].position = transform.position + ranPos; //swarm in global and place boid in local spae relative to swarm
            
            //Random forward
            float randomAngle = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
            Quaternion randomRotation = Quaternion.Euler(0, randomAngle, 0);
            boids[i].forward = randomRotation * Vector3.forward;
            boids[i].velocity = boids[i].forward.normalized;
            
            //Init boid
            boidObjects[i] = Instantiate(boidPrefab, boids[i].position, Quaternion.LookRotation(boids[i].forward));
        }
    }


    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for (int i = 0; i < boids.Length; i++) //reset everything
        {
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;
        }
    }


    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        ResetBoidForces();
        
        //Build neighbor lists
        for (int i = 0; i < boids.Length; i++)
        {
            List<int> neighbors = new List<int>();
            
            //vision check
            for (int j = 0; j < boids.Length; j++)
            {
                if (i == j) continue;
                
                Vector3 toNeighbor = boids[j].position - boids[i].position;
                float sqrDist = toNeighbor.sqrMagnitude; 
                
                //radius and dot product check
                if (sqrDist < sqrNeighbourDistance)
                {
                    //Dot product check for 180-degree FOV (cos(90°) = 0)
                    if (Vector3.Dot(toNeighbor.normalized, boids[i].forward) > 0f)
                    {
                        neighbors.Add(j);
                    }
                }
            }
            
            // Apply boid rules if we have neighbors
            if (neighbors.Count > 0)
            {
                // Separation rule
                Vector3 separationForce = Vector3.zero;
                foreach (int neighborIdx in neighbors)
                {
                    Vector3 distFromNeigh = boids[i].position - boids[neighborIdx].position;
                    separationForce += distFromNeigh.normalized / distFromNeigh.magnitude;
                }
                if (separationForce != Vector3.zero)
                {
                    boids[i].separation = separationWeight * (separationForce.normalized * boidForceScale - boids[i].velocity);
                }
                
                //Alignment rule
                Vector3 avgVelocity = Vector3.zero;
                foreach (int neighborIdx in neighbors)
                {
                    avgVelocity += boids[neighborIdx].velocity;
                }
                avgVelocity /= neighbors.Count;
                if (avgVelocity != Vector3.zero)
                {
                    boids[i].alignment = alignmentWeight * (avgVelocity.normalized * boidForceScale - boids[i].velocity);
                }

                //Cohesion rule
                Vector3 centerOfMass = Vector3.zero;
                foreach (int neighborIdx in neighbors)
                {
                    centerOfMass += boids[neighborIdx].position;
                }
                centerOfMass /= neighbors.Count;
                Vector3 toCenter = centerOfMass - boids[i].position;
                if (toCenter != Vector3.zero)
                {
                    boids[i].cohesion = cohesionWeight * (toCenter.normalized * boidForceScale - boids[i].velocity);
                }
            }//end if neighbor count

            else//no neighbors
            {
                //wander rule
                boids[i].separation = wanderWeight * (boids[i].velocity.normalized * boidForceScale - boids[i].velocity);
                boids[i].alignment = Vector3.zero;
                boids[i].cohesion = Vector3.zero;
            }
            
            // Obstacle avoidance rule (including world boundaries)
            Collider[] nearbyObstacles = Physics.OverlapSphere(boids[i].position, obstacleCheckRadius);
            Vector3 obstacleForce = Vector3.zero;
            
            foreach (Collider obstacle in nearbyObstacles)
            {
                Vector3 closestPoint = obstacle.ClosestPoint(boids[i].position);
                Vector3 awayFromObstacle = boids[i].position - closestPoint;
                if (awayFromObstacle.sqrMagnitude > 0.001f)
                {
                    obstacleForce += awayFromObstacle.normalized / awayFromObstacle.magnitude;
                }
            }
            
            // World boundaries
            if (boids[i].position.x > 8f)
                obstacleForce += Vector3.left;
            if (boids[i].position.x < -8f)
                obstacleForce += Vector3.right;
            if (boids[i].position.z > 8f)
                obstacleForce += Vector3.back;
            if (boids[i].position.z < -8f)
                obstacleForce += Vector3.forward;
            if (boids[i].position.y > 4f)
                obstacleForce += Vector3.down;
            if (boids[i].position.y < 1f)
                obstacleForce += Vector3.up;
            
            if (obstacleForce.sqrMagnitude > 0.001f)
            {
                boids[i].obstacle = obstacleWeight * (obstacleForce.normalized * boidForceScale - boids[i].velocity);
            }
            
            //Special handling for boid zero path following
            if (i == 0 && boidZeroNavigatingTowardGoal && boidZeroPath != null && 
                boidZeroPath.status == NavMeshPathStatus.PathComplete && 
                boidZeroPath.corners.Length > 1 && currentCorner < boidZeroPath.corners.Length) 
            {
                Vector3 toCorner = boidZeroPath.corners[currentCorner] - boids[i].position;
                Vector3 goalForce = goalWeight * (toCorner.normalized * boidForceScale - boids[i].velocity);
                boids[i].currentTotalForce += goalForce;
            }
            
            //accumulate total forces
            boids[i].currentTotalForce += boids[i].separation + boids[i].alignment + boids[i].cohesion + boids[i].obstacle;
            
            // Symplectic Euler integration
            Vector3 acceleration = boids[i].currentTotalForce;
            boids[i].velocity += acceleration * Time.fixedDeltaTime;
            
            //limit speed
            if (boids[i].velocity.sqrMagnitude > maxSpeed * maxSpeed)
            {
                boids[i].velocity = boids[i].velocity.normalized * maxSpeed;
            }
            
            //update position
            boids[i].position += boids[i].velocity * Time.fixedDeltaTime;
            
            //update forward direction (smooth rotation)
            if (boids[i].velocity.sqrMagnitude > 0.001f)
            {
                Vector3 newForward = Vector3.RotateTowards(boids[i].forward, boids[i].velocity, 
                    rotationSpeed * Mathf.Deg2Rad * Time.fixedDeltaTime, 0f);
                boids[i].forward = newForward.normalized;
            }
            
            //update boid object transform
            boidObjects[i].position = boids[i].position;
            boidObjects[i].rotation = Quaternion.LookRotation(boids[i].forward);
        }
        
        //update boid zero path following
        if (boidZeroNavigatingTowardGoal && boidZeroPath != null && 
            boidZeroPath.status == NavMeshPathStatus.PathComplete && 
            boidZeroPath.corners.Length > 1)
        {
            NavMeshHit hit;
            if (NavMesh.SamplePosition(boids[0].position, out hit, 1.0f, NavMesh.AllAreas))
            {
                //check distance to current corner
                float distanceToCorner = Vector3.Distance(hit.position, boidZeroPath.corners[currentCorner]);
                
                if (distanceToCorner < 1.0f)  // Within 1 unit of the corner
                {
                    currentCorner++;
                    
                    //if reached goal clear infos
                    if (currentCorner >= boidZeroPath.corners.Length)
                    {
                        //Debug.Log("Boid zero reached goal");
                        boidZeroNavigatingTowardGoal = false;
                        boidZeroPath.ClearCorners();
                        currentCorner = 0;
                        boidZeroPath = null;
                    }
                }
            }
        }
    }


    private void Update()
    {
        // Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        
    }


    public void SetGoal(Vector3 goal)
    {
        if (boidZeroNavigatingTowardGoal)
        {
            return;
        }
        
        boidZeroGoal = goal;
        
        // Create new path object
        boidZeroPath = new NavMeshPath();
        
        // Sample the start position from boid zero's current position
        NavMeshHit startHit;
        float maxSampleDistance = 10.0f;
        
        bool foundStart = NavMesh.SamplePosition(boids[0].position, out startHit, maxSampleDistance, NavMesh.AllAreas);
        
        NavMeshHit goalHit;
        bool foundGoal = NavMesh.SamplePosition(goal, out goalHit, maxSampleDistance, NavMesh.AllAreas);
        
        // Calculate the path between start and goal
        bool pathCalculated = NavMesh.CalculatePath(startHit.position, goalHit.position, NavMesh.AllAreas, boidZeroPath);
        if (!pathCalculated)
        {
            //Debug.LogWarning("Failed calculate path");
            return;
        }
        
        // Check if we got a complete path
        if (boidZeroPath.status != NavMeshPathStatus.PathComplete)
        {
            //Debug.LogWarning("Failed complete path");
            return;
        }
        
        // Check if we have enough corners (at least 2: start and goal)
        if (boidZeroPath.corners.Length < 2)
        {
            //Debug.LogWarning("Failed path corners");
            return;
        }
        
        boidZeroNavigatingTowardGoal = true;
        currentCorner = 1; // Start at first corner (index 0 is current position, index 1 is first waypoint)
        
    }
}