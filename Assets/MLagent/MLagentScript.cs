using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgentsExamples;

public class HumanoidArticulationAgent : Agent
{
    [Header("Robot Root")]
    public GameObject robotRoot;

    [Header("Root Body")]
    public ArticulationBody pelvis;

    [Header("Goal")]
    public Transform target;

    [Header("Helpers")]
    public OrientationCubeController orientationCube;
    public DirectionIndicator directionIndicator;

    [Header("Joint Settings")]
    public float forceLimit = 1000f;
    public float friction = 10f;

    [Header("Training")]
    public float targetRadius = 1f;
    public float spawnRadius = 0f;

    [Header("Action Settings")]
    public float actionRange = 20f;

    [Header("Controllable Joints")]
    [SerializeField]
    private List<ArticulationBody> controllableJoints = new();

    private List<float> standingPoseTargets = new();

    private Vector3 startPosition;
    private Quaternion startRotation;

    public override void Initialize()
    {
        startPosition = pelvis.transform.position;
        startRotation = pelvis.transform.rotation;

        standingPoseTargets.Clear();

        foreach (var joint in controllableJoints)
        {
            if (joint == null || joint.isRoot)
                continue;

            ArticulationDrive drive = joint.xDrive;

            drive.stiffness = 100000f;
            drive.damping = 5000f;
            drive.forceLimit = forceLimit;

            joint.jointFriction = friction;
            joint.xDrive = drive;

            standingPoseTargets.Add(drive.target);
        }
    }

    public override void OnEpisodeBegin()
    {
        target.position = startPosition + new Vector3(
            Random.Range(-spawnRadius, spawnRadius),
            0f,
            Random.Range(-spawnRadius, spawnRadius));



        UpdateOrientationObjects();

        StartCoroutine(ResetCoroutine());
    }

    IEnumerator ResetCoroutine()
    {
        // Hard teleport root (DO NOT disable object)
        pelvis.TeleportRoot(startPosition, startRotation);

        pelvis.linearVelocity = Vector3.zero;
        pelvis.angularVelocity = Vector3.zero;

        // let physics settle
        for (int i = 0; i < 5; i++)
            yield return new WaitForFixedUpdate();

        int safeCount = 0;

        // Reset joint drives safely
        for (int i = 0; i < controllableJoints.Count; i++)
        {
            var joint = controllableJoints[i];

            if (joint == null || joint.isRoot)
                continue;

            joint.linearVelocity = Vector3.zero;
            //joint.jointVelocity = new ArticulationReducedSpace(0f);
            switch (joint.dofCount)
            {
                case 1:
                    joint.jointVelocity =
                        new ArticulationReducedSpace(0f);
                    break;

                case 2:
                    joint.jointVelocity =
                        new ArticulationReducedSpace(0f, 0f);
                    break;

                case 3:
                    joint.jointVelocity =
                        new ArticulationReducedSpace(0f, 0f, 0f);
                    break;
            }

            var drive = joint.xDrive;
            drive.target = standingPoseTargets[safeCount];
            drive.targetVelocity = 0f;
            joint.xDrive = drive;

            safeCount++;
        }

        pelvis.linearVelocity = Vector3.zero;
        pelvis.angularVelocity = Vector3.zero;

        // extra stabilization frames
        for (int i = 0; i < 3; i++)
            yield return new WaitForFixedUpdate();

        Physics.SyncTransforms();
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
            sensor.AddObservation(
                joint.dofCount > 0 ? joint.jointPosition[0] : 0f);

            sensor.AddObservation(
                joint.dofCount > 0 ? joint.jointVelocity[0] : 0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int safeIndex = 0;

        foreach (var joint in controllableJoints)
        {
            if (joint == null || joint.isRoot)
                continue;

            if (safeIndex >= actions.ContinuousActions.Length)
                break;

            float action = Mathf.Clamp(actions.ContinuousActions[safeIndex], -1f, 1f);

            var drive = joint.xDrive;

            drive.target =
                standingPoseTargets[safeIndex] + action * actionRange;

            joint.xDrive = drive;

            safeIndex++;
        }
    }

    //void FixedUpdate()
    //{
    //    UpdateOrientationObjects();

    //    Vector3 toTarget =
    //        (target.position - pelvis.transform.position).normalized;

    //    float moveReward =
    //        Vector3.Dot(pelvis.linearVelocity.normalized, toTarget);

    //    if (!float.IsNaN(moveReward))
    //        AddReward(moveReward * 0.005f);

    //    float uprightReward =
    //        Vector3.Dot(pelvis.transform.up, Vector3.up);

    //    //AddReward(uprightReward * 0.002f);
    //    AddReward(uprightReward * 0.002f);

    //    float facingReward =
    //        Vector3.Dot(pelvis.transform.forward, toTarget);

    //    AddReward(Mathf.Max(0f, facingReward) * 0.002f);

    //    float distance =
    //        Vector3.Distance(pelvis.transform.position, target.position);

    //    if (distance < targetRadius)
    //    {
    //        AddReward(2f);
    //        EndEpisode();
    //    }

    //    if (uprightReward < 0.2f)
    //    {
    //        AddReward(-2f);
    //        EndEpisode();
    //    }

    //    if (pelvis.transform.position.y < 0.2f)
    //    {
    //        AddReward(-1f);
    //        EndEpisode();
    //    }
    //}

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        Vector3 toTarget = (target.position - pelvis.transform.position).normalized;
        float moveReward = Vector3.Dot(pelvis.linearVelocity.normalized, toTarget);

        float uprightReward = Vector3.Dot(pelvis.transform.up, Vector3.up);

        // Upright is a gate, not the main goal
        if (!float.IsNaN(moveReward))
            AddReward(moveReward * 0.02f);          // doubled

        AddReward(uprightReward * 0.003f);          // reduced

        // Reward getting closer to target (dense distance reward)
        float distance = Vector3.Distance(pelvis.transform.position, target.position);
        AddReward(-distance * 0.0001f);             // constant pull toward target

        if (distance < targetRadius)
        {
            AddReward(2f);
            EndEpisode();
        }

        if (uprightReward < 0.2f)
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

    public void HandTouchedGround()
    {
        AddReward(-2f);
        EndEpisode();
    }
}