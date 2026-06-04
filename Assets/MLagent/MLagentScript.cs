using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgentsExamples;

public class HumanoidArticulationAgent : Agent
{
    [Header("Root Body")]
    public ArticulationBody pelvis;

    [Header("Goal")]
    public Transform target;

    [Header("ML Agents Helpers")]
    public OrientationCubeController orientationCube;
    public DirectionIndicator directionIndicator;

    [Header("Joint Settings")]
    public float forceLimit = 100f;
    public float friction = 10f;

    [Header("Training")]
    public float targetRadius = 1f;
    public float spawnRadius = 9f;

    [Header("Controllable Joints")]
    [SerializeField]
    private List<ArticulationBody> controllableJoints =
        new List<ArticulationBody>();

    private List<float> startDriveTargets = new List<float>();

    private Vector3 startPosition;
    private Quaternion startRotation;

    public override void Initialize()
    {
        startPosition = pelvis.transform.position;
        startRotation = pelvis.transform.rotation;

        startDriveTargets.Clear();

        foreach (var joint in controllableJoints)
        {
            if (joint == null)
                continue;

            ArticulationDrive drive = joint.xDrive;

            drive.forceLimit = forceLimit;
            joint.jointFriction = friction;
            joint.xDrive = drive;

            // IMPORTANT: store INITIAL motor target (not jointPosition!)
            startDriveTargets.Add(drive.target);
        }

        Debug.Log($"Found {controllableJoints.Count} controllable joints");
    }

    public override void OnEpisodeBegin()
    {
        ResetRobot();

        target.position =
            startPosition +
            new Vector3(
                Random.Range(-spawnRadius, spawnRadius),
                0,
                Random.Range(-spawnRadius, spawnRadius));

        UpdateOrientationObjects();
    }

    void ResetRobot()
    {
        // 1. Disable physics simulation temporarily (IMPORTANT)
        pelvis.enabled = false;

        // 2. Reset root
        pelvis.TeleportRoot(startPosition, startRotation);

        // 3. Clear velocities (root)
        pelvis.linearVelocity = Vector3.zero;
        pelvis.angularVelocity = Vector3.zero;

        // 4. Reset ALL joints properly
        foreach (var joint in controllableJoints)
        {
            if (joint == null)
                continue;

            joint.enabled = false;   // IMPORTANT: reset internal state

            joint.linearVelocity = Vector3.zero;
            joint.angularVelocity = Vector3.zero;

            ArticulationDrive drive = joint.xDrive;
            drive.target = 0f; // or startDriveTargets if you want
            joint.xDrive = drive;

            joint.enabled = true;    // re-enable rebuilds articulation state
        }

        // 5. Re-enable root LAST (important order)
        pelvis.enabled = true;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        UpdateOrientationObjects();

        Vector3 targetDir =
            (target.position - pelvis.transform.position).normalized;

        sensor.AddObservation(
            orientationCube.transform.InverseTransformDirection(targetDir));

        sensor.AddObservation(
            orientationCube.transform.InverseTransformDirection(pelvis.linearVelocity));

        sensor.AddObservation(
            orientationCube.transform.InverseTransformDirection(pelvis.angularVelocity));

        sensor.AddObservation(
            Vector3.Distance(pelvis.transform.position, target.position));

        sensor.AddObservation(pelvis.transform.forward);
        sensor.AddObservation(pelvis.transform.up);

        foreach (var joint in controllableJoints)
        {
            if (joint.jointPosition.dofCount > 0)
                sensor.AddObservation(joint.jointPosition[0]);

            if (joint.jointVelocity.dofCount > 0)
                sensor.AddObservation(joint.jointVelocity[0]);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int actionIndex = 0;

        foreach (var joint in controllableJoints)
        {
            if (actionIndex >= actions.ContinuousActions.Length)
                break;

            float action =
                Mathf.Clamp(actions.ContinuousActions[actionIndex++], -1f, 1f);

            ArticulationDrive drive = joint.xDrive;

            float targetAngle =
                Mathf.Lerp(
                    drive.lowerLimit,
                    drive.upperLimit,
                    (action + 1f) * 0.5f);

            drive.target = targetAngle;

            joint.xDrive = drive;
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        Vector3 toTarget =
            (target.position - pelvis.transform.position).normalized;

        Vector3 velocity = pelvis.linearVelocity;

        float moveReward =
            Vector3.Dot(velocity.normalized, toTarget);

        if (!float.IsNaN(moveReward))
            AddReward(moveReward * 0.005f);

        float uprightReward =
            Vector3.Dot(pelvis.transform.up, Vector3.up);

        AddReward(uprightReward * 0.002f);

        float distance =
            Vector3.Distance(pelvis.transform.position, target.position);

        if (distance < targetRadius)
        {
            AddReward(2f);
            EndEpisode();
        }

        if (uprightReward < 0.3f)
        {
            AddReward(-1f);
            EndEpisode();
        }

        if (pelvis.transform.position.y < 0.2f)
        {
            AddReward(-1f);
            EndEpisode();
        }
    }

    void UpdateOrientationObjects()
    {
        if (orientationCube != null)
            orientationCube.UpdateOrientation(pelvis.transform, target);

        if (directionIndicator != null)
            directionIndicator.MatchOrientation(orientationCube.transform);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;

        for (int i = 0; i < actions.Length; i++)
            actions[i] = 0f;
    }
}