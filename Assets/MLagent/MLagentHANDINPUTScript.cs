using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgentsExamples;

public class HumanoidArticulationHandAgent : Agent
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
    //public float forceLimit = 200f;
    public float friction = 10f;

    [Header("Training")]
    public float targetRadius = 1f;
    public float spawnRadius = 4f;

    [Header("Action Settings")]
    public float actionRange = 180f;

    [Header("Controllable Joints")]
    [SerializeField]
    private List<ArticulationBody> controllableJoints = new();

    private List<ArticulationReducedSpace> initialJointPositions = new();
    private List<float> standingPoseTargets = new();

    private Vector3 startPosition;
    private Quaternion startRotation;


    public override void Initialize()
    {
        startPosition = pelvis.transform.position;
        startRotation = pelvis.transform.rotation;
        standingPoseTargets.Clear();

        Debug.Log($"=== JOINT INIT: {controllableJoints.Count} joints in lijst ===");

        foreach (var joint in controllableJoints)
        {
            if (joint == null)
            {
                Debug.LogWarning("NULL joint gevonden in lijst!");
                continue;
            }

            Debug.Log($"Joint: {joint.name} | isRoot: {joint.isRoot} | dofCount: {joint.dofCount} | jointType: {joint.jointType}");

            if (joint.isRoot)
                continue;

            if (joint.dofCount >= 1)
            {
                var d = joint.xDrive;
                d.stiffness = 1000f;
                d.damping = 50f;
                //d.forceLimit = forceLimit;
                d.driveType = ArticulationDriveType.Acceleration;
                //d.lowerLimit = -90f * Mathf.Deg2Rad;  // of gewoon in Inspector zetten
                //d.upperLimit = 90f * Mathf.Deg2Rad;
                joint.xDrive = d;
            }
            if (joint.dofCount >= 2)
            {
                var d = joint.yDrive;
                d.stiffness = 100000f;
                d.damping = 5000f;
                //d.forceLimit = forceLimit;
                d.driveType = ArticulationDriveType.Acceleration;
                joint.yDrive = d;
            }
            if (joint.dofCount >= 3)
            {
                var d = joint.zDrive;
                d.stiffness = 100000f;
                d.damping = 5000f;
                //d.forceLimit = forceLimit;
                d.driveType = ArticulationDriveType.Acceleration;
                joint.zDrive = d;
            }

            joint.jointFriction = friction;

            for (int i = 0; i < joint.dofCount; i++)
            {
                float pos = joint.jointPosition[i];
                standingPoseTargets.Add(pos);
                Debug.Log($"  -> DOF[{i}] positie opgeslagen: {pos}");
            }
        }

        //Debug.Log($"=== Totaal standingPoseTargets: {standingPoseTargets.Count} ===");
    }


    public override void OnEpisodeBegin()
    {
        target.position = startPosition + new Vector3(
            Random.Range(-spawnRadius, spawnRadius),
            0f,
            Random.Range(-spawnRadius, spawnRadius));
        //target.position = startPosition + new Vector3(0f, 0f, 3f);

        UpdateOrientationObjects();

        StartCoroutine(ResetCoroutine());

    }



    IEnumerator ResetCoroutine()
    {
        pelvis.TeleportRoot(startPosition, startRotation);
        pelvis.linearVelocity = Vector3.zero;
        pelvis.angularVelocity = Vector3.zero;

        int safeCount = 0;
        foreach (var joint in controllableJoints)
        {
            if (joint == null || joint.isRoot)
                continue;

            joint.linearVelocity = Vector3.zero;

            // Zero velocities
            switch (joint.dofCount)
            {
                case 1: joint.jointVelocity = new ArticulationReducedSpace(0f); break;
                case 2: joint.jointVelocity = new ArticulationReducedSpace(0f, 0f); break;
                case 3: joint.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f); break;
            }

            // Zet posities terug per as
            if (joint.dofCount == 1)
            {
                joint.jointPosition = new ArticulationReducedSpace(standingPoseTargets[safeCount]);
                var d = joint.xDrive; d.target = standingPoseTargets[safeCount]; d.targetVelocity = 0f; joint.xDrive = d;
            }
            else if (joint.dofCount == 2)
            {
                joint.jointPosition = new ArticulationReducedSpace(standingPoseTargets[safeCount], standingPoseTargets[safeCount + 1]);
                var dx = joint.xDrive; dx.target = standingPoseTargets[safeCount]; dx.targetVelocity = 0f; joint.xDrive = dx;
                var dy = joint.yDrive; dy.target = standingPoseTargets[safeCount + 1]; dy.targetVelocity = 0f; joint.yDrive = dy;
            }
            else if (joint.dofCount == 3)
            {
                joint.jointPosition = new ArticulationReducedSpace(standingPoseTargets[safeCount], standingPoseTargets[safeCount + 1], standingPoseTargets[safeCount + 2]);
                var dx = joint.xDrive; dx.target = standingPoseTargets[safeCount]; dx.targetVelocity = 0f; joint.xDrive = dx;
                var dy = joint.yDrive; dy.target = standingPoseTargets[safeCount + 1]; dy.targetVelocity = 0f; joint.yDrive = dy;
                var dz = joint.zDrive; dz.target = standingPoseTargets[safeCount + 2]; dz.targetVelocity = 0f; joint.zDrive = dz;
            }

            safeCount += joint.dofCount;
        }

        Physics.SyncTransforms();
        yield return new WaitForFixedUpdate();
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
            for (int i = 0; i < joint.dofCount; i++)
            {
                sensor.AddObservation(joint.jointPosition[i]);
                sensor.AddObservation(joint.jointVelocity[i]);
            }
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int poseIndex = 0;
        int actionIndex = 0;

        foreach (var joint in controllableJoints)
        {
            if (joint == null || joint.isRoot)
                continue;

            if (joint.dofCount >= 1 && actionIndex < actions.ContinuousActions.Length)
            {
                float a = Mathf.Clamp(actions.ContinuousActions[actionIndex++], -1f, 1f);
                var d = joint.xDrive;
                float newTarget = standingPoseTargets[poseIndex] + a * actionRange;
                d.target = newTarget;
                joint.xDrive = d;

                // DEBUG: log elke joint die een actie krijgt
                //if (joint.name.Contains("knee") || joint.name.Contains("Knee"))
                //Debug.Log($"{joint.name} | action={a:F3} | poseBase={standingPoseTargets[poseIndex]:F3} | newTarget={newTarget:F3} | xDrive.target na set={joint.xDrive.target:F3} | stiffness={joint.xDrive.stiffness} | driveType={joint.xDrive.driveType}");
            }
            if (joint.dofCount >= 2 && actionIndex < actions.ContinuousActions.Length)
            {
                float a = Mathf.Clamp(actions.ContinuousActions[actionIndex++], -1f, 1f);
                var d = joint.yDrive;
                d.target = standingPoseTargets[poseIndex + 1] + a * actionRange;
                joint.yDrive = d;
            }
            if (joint.dofCount >= 3 && actionIndex < actions.ContinuousActions.Length)
            {
                float a = Mathf.Clamp(actions.ContinuousActions[actionIndex++], -1f, 1f);
                var d = joint.zDrive;
                d.target = standingPoseTargets[poseIndex + 2] + a * actionRange;
                joint.zDrive = d;
            }

            // TIJDELIJK: forceer maximale uitslag op knie
            if (joint.name.Contains("knee") || joint.name.Contains("Knee"))
            {
                var d = joint.xDrive;
                //d.target = joint.xDrive.upperLimit; // ga naar max limiet
                d.target = 20f;
                joint.xDrive = d;
                Debug.Log($"Forceer {joint.name} naar upperLimit={joint.xDrive.upperLimit:F4}");
                poseIndex += joint.dofCount;
                Debug.Log($"{joint.name} | mass={joint.mass} | immovable={joint.immovable}");
                continue;

            }
            if (joint.name.Contains("ankle_pitch"))
            {
                var d = joint.xDrive;
                //d.target = joint.xDrive.upperLimit; // ga naar max limiet
                d.target = 0f;
                joint.xDrive = d;
                Debug.Log($"Forceer {joint.name} naar upperLimit={joint.xDrive.upperLimit:F4}");
                poseIndex += joint.dofCount;
                Debug.Log($"{joint.name} | mass={joint.mass} | immovable={joint.immovable}");
                continue;

            }
            if (joint.name.Contains("ankle_roll"))
            {
                var d = joint.xDrive;
                //d.target = joint.xDrive.upperLimit; // ga naar max limiet
                d.target = 0f;
                joint.xDrive = d;
                Debug.Log($"Forceer {joint.name} naar upperLimit={joint.xDrive.upperLimit:F4}");
                poseIndex += joint.dofCount;
                Debug.Log($"{joint.name} | mass={joint.mass} | immovable={joint.immovable}");
                continue;

            }
            if (joint.name.Contains("hip_pitch"))
            {
                var d = joint.xDrive;
                //d.target = joint.xDrive.upperLimit; // ga naar max limiet
                d.target = -20f;
                joint.xDrive = d;
                Debug.Log($"Forceer {joint.name} naar upperLimit={joint.xDrive.upperLimit:F4}");
                poseIndex += joint.dofCount;
                Debug.Log($"{joint.name} | mass={joint.mass} | immovable={joint.immovable}");
                continue;

            }
            if (joint.name.Contains("torso_link"))
            {
                var d = joint.xDrive;
                //d.target = joint.xDrive.upperLimit; // ga naar max limiet
                d.target = -10f;
                joint.xDrive = d;
                Debug.Log($"Forceer {joint.name} naar upperLimit={joint.xDrive.upperLimit:F4}");
                poseIndex += joint.dofCount;
                Debug.Log($"{joint.name} | mass={joint.mass} | immovable={joint.immovable}");
                continue;

            }
            //if (joint.name.Contains("waist") || joint.name.Contains("Waist"))
            //{
            //    var d = joint.xDrive;
            //    d.target = joint.xDrive.upperLimit; // ga naar max limiet
            //    joint.xDrive = d;
            //    Debug.Log($"Forceer {joint.name} naar upperLimit={joint.xDrive.upperLimit:F4}");
            //    poseIndex += joint.dofCount;
            //    Debug.Log($"{joint.name} | mass={joint.mass} | immovable={joint.immovable}");
            //    continue;

            //}

            poseIndex += joint.dofCount;
        }
    }


    void FixedUpdate()
    {
        //// DEBUG knie
        //foreach (var joint in controllableJoints)
        //{
        //    if (joint == null) continue;
        //    if (joint.name.Contains("knee") || joint.name.Contains("Knee"))
        //        Debug.Log($"[FixedUpdate] {joint.name} | pos={joint.jointPosition[0]:F3} | vel={joint.jointVelocity[0]:F3} | driveTarget={joint.xDrive.target:F3} | stiffness={joint.xDrive.stiffness} | driveType={joint.xDrive.driveType}");
        //}
        UpdateOrientationObjects();

        Vector3 toTarget = (target.position - pelvis.transform.position).normalized;
        float moveReward = Vector3.Dot(pelvis.linearVelocity.normalized, toTarget);

        float pelvisuprightReward = Vector3.Dot(pelvis.transform.up, Vector3.up);
        float torsouprightReward = Vector3.Dot(pelvis.transform.up, Vector3.up);

        //Upright is a gate, not the main goal
        if (!float.IsNaN(moveReward))
            AddReward(moveReward * 0.0001f);

        AddReward(pelvisuprightReward * 0.05f);

        float facingReward =
    Vector3.Dot(pelvis.transform.forward, toTarget);

        // Reward only positive alignment
        AddReward(Mathf.Max(0f, facingReward) * 0.005f);

        //Reward getting closer to target(dense distance reward)
        float distance = Vector3.Distance(pelvis.transform.position, target.position);
        AddReward(-distance * 0.001f);             // constant pull toward target

        if (distance < targetRadius)
        {
            AddReward(5f);
            EndEpisode();
        }

        if (torsouprightReward < 0.1f)
        {
            AddReward(-2f);
            EndEpisode();
        }

        if (pelvisuprightReward < 0.3f)
        {
            AddReward(-20f);
            EndEpisode();
        }

        if (facingReward < 0.3f)
        {
            AddReward(-5f);
            EndEpisode();
        }

        if (pelvis.transform.position.y < 0.1f)
        {
            AddReward(-20f);
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

    //public void HandTouchedGround()
    //{
    //    AddReward(-2f);
    //    EndEpisode();
    //}
}