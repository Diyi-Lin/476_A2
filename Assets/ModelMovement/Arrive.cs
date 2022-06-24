using UnityEngine;

namespace AI
{
    public class Arrive : AIMovement
    {
        public float slowRadius;
        public float stopRadius;

        private void DrawDebug(AIAgent agent)
        {
            if (debug)
            {
                DebugUtil.DrawCircle(agent.TargetPosition, transform.up, Color.yellow, stopRadius);
                DebugUtil.DrawCircle(agent.TargetPosition, transform.up, Color.magenta, slowRadius);
            }
        }
        

        public override SteeringOutput GetKinematic(AIAgent agent)
        {
            //DrawDebug(agent);

            var output = base.GetKinematic(agent);

            // TODO: calculate linear component
			
			
            return output;
        }

        public override SteeringOutput GetSteering(AIAgent agent)
        {
            //DrawDebug(agent);

            var output = base.GetSteering(agent);

            // TODO: calculate linear component
            Vector3 desiredVelocity = agent.TargetPosition - agent.transform.position;
            float distance = Mathf.Sqrt(desiredVelocity.sqrMagnitude);

            if (distance > stopRadius)
            {
                float rampedSpeed = agent.maxSpeed * (distance / slowRadius);

                float clippedSpeed = Mathf.Min(rampedSpeed, agent.maxSpeed);

                desiredVelocity = desiredVelocity.normalized * clippedSpeed;

            }
            else 
            { 
                desiredVelocity = desiredVelocity.normalized * 0;
                Debug.Log("stop");
            }
            output.linear = desiredVelocity;
            return output;
        }
    }
}
