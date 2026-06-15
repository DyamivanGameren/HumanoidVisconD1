using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;

[RequireComponent(typeof(BehaviorParameters))]
public class G1Phase1StandAgent : Agent
{
    [Header("URDF setup")]
    [SerializeField] private bool addUrdfControlHelpers = true;
    [SerializeField] private bool logDetectedJoints = true;

    [Header("Joint control")]
    [SerializeField] private float actionStepDegrees = 0.5f;
    [SerializeField] private float maxDeviationFromStartDegrees = 15f;

    [SerializeField] private float jointStiffness = 100f;
    [SerializeField] private float jointDamping = 0f;
    [SerializeField] private float jointForceLimit = 100f;
    [SerializeField] private float jointFriction = 10f;
    [SerializeField] private float angularDamping = 10f;

    [Header("Standing target")]
    [SerializeField] private float desiredRootHeight = 0.78f;
    [SerializeField] private float minimumRootHeight = 0.38f;

    [Range(-1f, 1f)]
    [SerializeField] private float minimumUprightness = 0.25f;

    [Header("Rewards")]
    [SerializeField] private float aliveRewardPerSecond = 0.8f;
    [SerializeField] private float uprightReward = 0.008f;
    [SerializeField] private float heightReward = 0.008f;
    [SerializeField] private float poseReward = 0.004f;
    [SerializeField] private float stillnessPenalty = 0.00008f;
    [SerializeField] private float actionPenalty = 0.00008f;
    [SerializeField] private float fallPenalty = -1f;

    [Header("Episode reset")]
    [SerializeField] private int rewardGraceDecisionSteps = 5;
    [SerializeField] private bool randomizeStartPose = false;
    [SerializeField] private float randomYawDegrees = 0f;
    [SerializeField] private float randomTiltDegrees = 0f;

    private readonly List<ArticulationBody> controlledJoints = new();
    private readonly List<float> initialTargets = new();
    private readonly List<float> initialRawJointPositions = new();

    private ArticulationBody rootBody;
    private ArticulationBody[] allBodies;
    private Vector3 initialRootPosition;
    private Quaternion initialRootRotation;

    private int decisionStepsSinceReset;
    private bool initialized;

    public override void Initialize()
    {
        rootBody = GetComponent<ArticulationBody>();

        if (rootBody == null)
        {
            Debug.LogError("Put G1Phase1StandAgent on the root ArticulationBody, usually pelvis.", this);
            enabled = false;
            return;
        }

        if (addUrdfControlHelpers)
        {
            AddUrdfHelpersLikeControllerScript();
        }

        allBodies = GetComponentsInChildren<ArticulationBody>(true);
        initialRootPosition = rootBody.transform.position;
        initialRootRotation = rootBody.transform.rotation;

        FindControlledJoints();
        ConfigureArticulations();
        StoreInitialTargets();
        ConfigureBehaviorParameters();

        initialized = true;

        Debug.Log(
            $"G1Phase1StandAgent ready. Bodies: {allBodies.Length}, " +
            $"controlled joints/actions: {controlledJoints.Count}, " +
            $"observations: {GetObservationSize()}",
            this);
    }

    private void AddUrdfHelpersLikeControllerScript()
    {
        if (GetComponent<FKRobot>() == null)
        {
            gameObject.AddComponent<FKRobot>();
        }

        ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>(true);

        foreach (ArticulationBody body in bodies)
        {
            if (body.GetComponent<JointControl>() == null)
            {
                body.gameObject.AddComponent<JointControl>();
            }

            JointControl jointControl = body.GetComponent<JointControl>();
            jointControl.controltype = ControlType.PositionControl;
            jointControl.direction = RotationDirection.None;

            body.jointFriction = jointFriction;
            body.angularDamping = angularDamping;
            body.useGravity = true;

            ArticulationDrive drive = body.xDrive;
            drive.stiffness = jointStiffness;
            drive.damping = jointDamping;
            drive.forceLimit = jointForceLimit;
            body.xDrive = drive;
        }
    }

    private void FindControlledJoints()
    {
        controlledJoints.Clear();

        foreach (ArticulationBody body in allBodies)
        {
            if (body == null || body.isRoot)
            {
                continue;
            }

            if (body.jointType == ArticulationJointType.FixedJoint)
            {
                continue;
            }

            if (body.jointType != ArticulationJointType.RevoluteJoint &&
                body.jointType != ArticulationJointType.PrismaticJoint)
            {
                Debug.LogWarning($"Skipping {body.name}, joint type {body.jointType} may have multiple DOF.", body);
                continue;
            }

            controlledJoints.Add(body);

            if (logDetectedJoints)
            {
                ArticulationDrive drive = body.xDrive;
                Debug.Log($"Controlled joint: {body.name}, limits {drive.lowerLimit:F1} to {drive.upperLimit:F1}", body);
            }
        }
    }

    private void ConfigureArticulations()
    {
        foreach (ArticulationBody body in allBodies)
        {
            body.useGravity = true;
            body.jointFriction = jointFriction;
            body.angularDamping = angularDamping;
        }

        foreach (ArticulationBody joint in controlledJoints)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.stiffness = jointStiffness;
            drive.damping = jointDamping;
            drive.forceLimit = jointForceLimit;
            joint.xDrive = drive;
        }
    }

    private void StoreInitialTargets()
    {
        initialTargets.Clear();
        initialRawJointPositions.Clear();

        foreach (ArticulationBody joint in controlledJoints)
        {
            initialTargets.Add(joint.xDrive.target);
            initialRawJointPositions.Add(GetRawJointPosition(joint));
        }
    }

    private void ConfigureBehaviorParameters()
    {
        BehaviorParameters behaviorParameters = GetComponent<BehaviorParameters>();

        behaviorParameters.BehaviorName = "G1Phase1Stand";
        behaviorParameters.BrainParameters.ActionSpec =
            ActionSpec.MakeContinuous(controlledJoints.Count);

        behaviorParameters.BrainParameters.VectorObservationSize = GetObservationSize();
        behaviorParameters.BrainParameters.NumStackedVectorObservations = 1;
    }

    private int GetObservationSize()
    {
        return 11 + controlledJoints.Count * 3;
    }

    public override void OnEpisodeBegin()
    {
        if (!initialized)
        {
            return;
        }

        decisionStepsSinceReset = 0;

        Quaternion resetRotation = initialRootRotation;

        if (randomizeStartPose)
        {
            resetRotation =
                Quaternion.Euler(
                    Random.Range(-randomTiltDegrees, randomTiltDegrees),
                    Random.Range(-randomYawDegrees, randomYawDegrees),
                    Random.Range(-randomTiltDegrees, randomTiltDegrees)) *
                initialRootRotation;
        }

        rootBody.TeleportRoot(initialRootPosition, resetRotation);
        ResetVelocities();
        ResetJointTargets();
        rootBody.WakeUp();
    }

    private void ResetVelocities()
    {
        foreach (ArticulationBody body in allBodies)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }
    }

    private void ResetJointTargets()
    {
        for (int i = 0; i < controlledJoints.Count; i++)
        {
            ArticulationBody joint = controlledJoints[i];
            ArticulationDrive drive = joint.xDrive;

            drive.target = initialTargets[i];
            drive.targetVelocity = 0f;
            joint.xDrive = drive;

            if (joint.jointPosition.dofCount > 0)
            {
                joint.jointPosition = new ArticulationReducedSpace(initialRawJointPositions[i]);
                joint.jointVelocity = new ArticulationReducedSpace(0f);
            }

            JointControl jointControl = joint.GetComponent<JointControl>();
            if (jointControl != null)
            {
                jointControl.direction = RotationDirection.None;
            }
        }
    }

    private void FixedUpdate()
    {
        if (!initialized || decisionStepsSinceReset <= rewardGraceDecisionSteps)
        {
            return;
        }

        if (IsAlive())
        {
            AddReward(aliveRewardPerSecond * Time.fixedDeltaTime);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform root = rootBody.transform;

        Vector3 localVelocity = root.InverseTransformDirection(rootBody.linearVelocity);
        Vector3 localAngularVelocity = root.InverseTransformDirection(rootBody.angularVelocity);
        Vector3 worldUpInRootSpace = root.InverseTransformDirection(Vector3.up);

        sensor.AddObservation(localVelocity / 5f);
        sensor.AddObservation(localAngularVelocity / 10f);
        sensor.AddObservation(worldUpInRootSpace);
        sensor.AddObservation(root.position.y / 2f);
        sensor.AddObservation(Vector3.Dot(root.up, Vector3.up));

        foreach (ArticulationBody joint in controlledJoints)
        {
            ArticulationDrive drive = joint.xDrive;

            float jointPosition = GetJointPosition(joint);
            float jointVelocity = GetJointVelocity(joint);

            sensor.AddObservation(NormalizeBetweenLimits(jointPosition, drive.lowerLimit, drive.upperLimit));
            sensor.AddObservation(Mathf.Clamp(jointVelocity / 10f, -1f, 1f));
            sensor.AddObservation(NormalizeTargetDifference(drive.target - jointPosition, drive.lowerLimit, drive.upperLimit));
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!initialized)
        {
            return;
        }

        ActionSegment<float> actions = actionBuffers.ContinuousActions;

        if (actions.Length != controlledJoints.Count)
        {
            Debug.LogError($"Action count mismatch. Got {actions.Length}, expected {controlledJoints.Count}.", this);
            EndEpisode();
            return;
        }

        float totalSquaredAction = ApplyActions(actions);
        decisionStepsSinceReset++;

        if (decisionStepsSinceReset <= rewardGraceDecisionSteps)
        {
            return;
        }

        AddStandingRewards(totalSquaredAction);

        if (!IsAlive())
        {
            AddReward(fallPenalty);
            EndEpisode();
        }
    }

    private float ApplyActions(ActionSegment<float> actions)
    {
        float totalSquaredAction = 0f;

        for (int i = 0; i < controlledJoints.Count; i++)
        {
            float action = Mathf.Clamp(actions[i], -1f, 1f);
            ArticulationBody joint = controlledJoints[i];
            ArticulationDrive drive = joint.xDrive;

            drive.target += action * actionStepDegrees;

            float lowerLimit = drive.lowerLimit;
            float upperLimit = drive.upperLimit;

            if (upperLimit > lowerLimit)
            {
                float startLimitedMin = initialTargets[i] - maxDeviationFromStartDegrees;
                float startLimitedMax = initialTargets[i] + maxDeviationFromStartDegrees;

                float finalMin = Mathf.Max(lowerLimit, startLimitedMin);
                float finalMax = Mathf.Min(upperLimit, startLimitedMax);

                drive.target = Mathf.Clamp(drive.target, finalMin, finalMax);
            }
            else
            {
                drive.target = Mathf.Clamp(
                    drive.target,
                    initialTargets[i] - maxDeviationFromStartDegrees,
                    initialTargets[i] + maxDeviationFromStartDegrees);
            }

            joint.xDrive = drive;
            totalSquaredAction += action * action;
        }

        return totalSquaredAction;
    }

    private void AddStandingRewards(float totalSquaredAction)
    {
        Transform root = rootBody.transform;

        float uprightScore = Mathf.Clamp01(Vector3.Dot(root.up, Vector3.up));

        float heightError = Mathf.Abs(root.position.y - desiredRootHeight);
        float heightScore = Mathf.Clamp01(1f - heightError / Mathf.Max(desiredRootHeight, 0.01f));

        float poseScore = GetInitialPoseScore();

        float rootMotion =
            rootBody.linearVelocity.sqrMagnitude +
            rootBody.angularVelocity.sqrMagnitude;

        float averageAction =
            totalSquaredAction / Mathf.Max(controlledJoints.Count, 1);

        AddReward(uprightScore * uprightReward);
        AddReward(heightScore * heightReward);
        AddReward(poseScore * poseReward);
        AddReward(-rootMotion * stillnessPenalty);
        AddReward(-averageAction * actionPenalty);
    }

    private float GetInitialPoseScore()
    {
        if (controlledJoints.Count == 0)
        {
            return 0f;
        }

        float score = 0f;

        for (int i = 0; i < controlledJoints.Count; i++)
        {
            ArticulationBody joint = controlledJoints[i];
            ArticulationDrive drive = joint.xDrive;

            float range = Mathf.Max(Mathf.Abs(drive.upperLimit - drive.lowerLimit), 1f);
            float error = Mathf.Abs(GetJointPosition(joint) - initialTargets[i]);

            score += Mathf.Clamp01(1f - error / range);
        }

        return score / controlledJoints.Count;
    }

    private bool IsAlive()
    {
        Transform root = rootBody.transform;

        if (!IsFinite(root.position) ||
            !IsFinite(rootBody.linearVelocity) ||
            !IsFinite(rootBody.angularVelocity))
        {
            return false;
        }

        if (root.position.y < minimumRootHeight)
        {
            return false;
        }

        return Vector3.Dot(root.up, Vector3.up) >= minimumUprightness;
    }

    private float GetJointPosition(ArticulationBody joint)
    {
        if (joint.jointPosition.dofCount == 0)
        {
            return joint.xDrive.target;
        }

        if (joint.jointType == ArticulationJointType.PrismaticJoint)
        {
            return joint.jointPosition[0];
        }

        return joint.jointPosition[0] * Mathf.Rad2Deg;
    }

    private float GetRawJointPosition(ArticulationBody joint)
    {
        if (joint.jointPosition.dofCount == 0)
        {
            return 0f;
        }

        return joint.jointPosition[0];
    }

    private float GetJointVelocity(ArticulationBody joint)
    {
        if (joint.jointVelocity.dofCount == 0)
        {
            return 0f;
        }

        return joint.jointVelocity[0];
    }

    private float NormalizeBetweenLimits(float value, float lowerLimit, float upperLimit)
    {
        if (upperLimit <= lowerLimit)
        {
            return 0f;
        }

        return Mathf.Clamp(
            Mathf.InverseLerp(lowerLimit, upperLimit, value) * 2f - 1f,
            -1f,
            1f);
    }

    private float NormalizeTargetDifference(float difference, float lowerLimit, float upperLimit)
    {
        float range = Mathf.Abs(upperLimit - lowerLimit);

        if (range < 0.001f)
        {
            return 0f;
        }

        return Mathf.Clamp(difference / range, -1f, 1f);
    }

    private bool IsFinite(Vector3 value)
    {
        return
            float.IsFinite(value.x) &&
            float.IsFinite(value.y) &&
            float.IsFinite(value.z);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;

        for (int i = 0; i < actions.Length; i++)
        {
            actions[i] = 0f;
        }
    }
}