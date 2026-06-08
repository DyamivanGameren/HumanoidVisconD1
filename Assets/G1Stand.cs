using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

[RequireComponent(typeof(BehaviorParameters))]
public class G1StandAgent : Agent
{
    [Header("Automatic detection")]
    [Tooltip("Automatisch alle beweegbare ArticulationBody-joints vinden.")]
    [SerializeField] private bool automaticallyFindAllJoints = true;

    [Tooltip("Toon de gevonden joints in de Console.")]
    [SerializeField] private bool logDetectedJoints = true;

    [Header("Joint control")]
    [Tooltip("Maximale verandering van het joint-target per action step, in graden.")]
    [SerializeField] private float actionSpeed = 1f;

    [Tooltip("Globale stiffness voor alle bestuurbare joints.")]
    [SerializeField] private float jointStiffness = 3000f;

    [Tooltip("Globale damping voor alle bestuurbare joints.")]
    [SerializeField] private float jointDamping = 150f;

    [Tooltip("Globale force limit voor alle bestuurbare joints.")]
    [SerializeField] private float jointForceLimit = 300f;

    [Header("Standing reward")]
    [Tooltip("Gewenste hoogte van het midden van de pelvis.")]
    [SerializeField] private float desiredPelvisHeight = 0.78f;

    [Tooltip("Onder deze hoogte wordt de episode be�indigd.")]
    [SerializeField] private float minimumPelvisHeight = 0.38f;

    [Tooltip("Minimale dot-product met Vector3.up voordat de robot als gevallen geldt.")]
    [Range(-1f, 1f)]
    [SerializeField] private float minimumUprightness = 0.25f;

    [Tooltip("Reward die de robot per gesimuleerde seconde krijgt zolang hij niet gevallen is.")]
    [SerializeField] private float survivalRewardPerSecond = 1f;

    [SerializeField] private float uprightRewardScale = 0.003f;
    [SerializeField] private float heightRewardScale = 0.002f;
    [SerializeField] private float poseRewardScale = 0.001f;
    [SerializeField] private float velocityPenaltyScale = 0.00005f;
    [SerializeField] private float actionPenaltyScale = 0.00002f;

    [Header("Reset")]
    [Tooltip("Wachttijd in physics-frames na een reset voordat rewards worden gegeven.")]
    [SerializeField] private int resetGraceSteps = 5;

    [Tooltip("Randomiseer de rotatie licht bij iedere episode.")]
    [SerializeField] private bool randomizeStartRotation = true;

    [SerializeField] private float randomYawRange = 5f;
    [SerializeField] private float randomTiltRange = 2f;

    private ArticulationBody rootBody;
    private ArticulationBody[] allBodies;

    private readonly List<ArticulationBody> controlledJoints = new();
    private readonly List<float> initialDriveTargets = new();

    private Vector3 initialRootPosition;
    private Quaternion initialRootRotation;

    private int stepsSinceReset;
    private float episodeLifetimeSeconds;
    private bool initialized;

    public int ControlledJointCount => controlledJoints.Count;

    public override void Initialize()
    {
        rootBody = GetComponent<ArticulationBody>();

        if (rootBody == null)
        {
            Debug.LogError(
                "G1StandAgent moet op het object met de root ArticulationBody staan, bijvoorbeeld pelvis.",
                this);

            enabled = false;
            return;
        }

        if (!rootBody.isRoot)
        {
            Debug.LogWarning(
                $"{rootBody.name} is geen root ArticulationBody. " +
                "Plaats G1StandAgent op de pelvis/root van de robot.",
                this);
        }

        allBodies = GetComponentsInChildren<ArticulationBody>(true);

        initialRootPosition = rootBody.transform.position;
        initialRootRotation = rootBody.transform.rotation;

        FindAllControllableJoints();
        ConfigureAllJointDrives();
        StoreInitialJointTargets();
        ConfigureBehaviorParameters();

        initialized = true;

        Debug.Log(
            $"G1StandAgent ge�nitialiseerd. " +
            $"Articulation bodies: {allBodies.Length}, " +
            $"bestuurbare joints/actions: {controlledJoints.Count}, " +
            $"observaties: {GetObservationSize()}",
            this);
    }

    /// <summary>
    /// Vindt automatisch alle niet-root en niet-fixed ArticulationBody-joints.
    /// </summary>
    private void FindAllControllableJoints()
    {
        controlledJoints.Clear();

        foreach (ArticulationBody body in allBodies)
        {
            if (body == null)
            {
                continue;
            }

            // De pelvis/root is vrij bewegend en heeft geen drive naar een parent.
            if (body.isRoot)
            {
                continue;
            }

            // Fixed joints kunnen niet worden bestuurd.
            if (body.jointType == ArticulationJointType.FixedJoint)
            {
                continue;
            }

            /*
             * Dit script gebruikt ��n action per ArticulationBody.
             * Een ge�mporteerde G1-URDF gebruikt normaal ��n revolute DOF
             * per beweegbare joint.
             */
            if (body.jointType != ArticulationJointType.RevoluteJoint &&
                body.jointType != ArticulationJointType.PrismaticJoint)
            {
                Debug.LogWarning(
                    $"Joint '{body.name}' is van type {body.jointType}. " +
                    "Deze wordt overgeslagen omdat hij mogelijk meerdere DOF's heeft.",
                    body);

                continue;
            }

            controlledJoints.Add(body);

            if (logDetectedJoints)
            {
                ArticulationDrive drive = body.xDrive;

                Debug.Log(
                    $"ML-joint gevonden: {body.name} | " +
                    $"Type: {body.jointType} | " +
                    $"Limits: {drive.lowerLimit:F1} tot {drive.upperLimit:F1}",
                    body);
            }
        }
    }

    /// <summary>
    /// Geeft alle automatisch gevonden joints bruikbare drive-instellingen.
    /// </summary>
    private void ConfigureAllJointDrives()
    {
        foreach (ArticulationBody joint in controlledJoints)
        {
            ArticulationDrive drive = joint.xDrive;

            drive.stiffness = jointStiffness;
            drive.damping = jointDamping;
            drive.forceLimit = jointForceLimit;

            joint.xDrive = drive;

            joint.useGravity = true;
            joint.jointFriction = 10f;
            joint.angularDamping = 0.05f;
        }

        // Ook de pelvis en fixed links gebruiken zwaartekracht.
        foreach (ArticulationBody body in allBodies)
        {
            body.useGravity = true;
        }
    }

    private void StoreInitialJointTargets()
    {
        initialDriveTargets.Clear();

        foreach (ArticulationBody joint in controlledJoints)
        {
            initialDriveTargets.Add(joint.xDrive.target);
        }
    }

    /// <summary>
    /// Configureert het aantal actions automatisch.
    /// </summary>
    private void ConfigureBehaviorParameters()
    {
        BehaviorParameters behaviorParameters =
            GetComponent<BehaviorParameters>();

        behaviorParameters.BehaviorName = "G1Stand";

        /*
         * E�n continuous action per gevonden joint.
         * Geen discrete actions.
         */
        behaviorParameters.BrainParameters.ActionSpec =
            ActionSpec.MakeContinuous(controlledJoints.Count);

        /*
         * Bij gebruik van CollectObservations wordt de observation size
         * door ML-Agents gecontroleerd aan de hand van de verzamelde data.
         */
        behaviorParameters.BrainParameters.VectorObservationSize =
            GetObservationSize();

        behaviorParameters.BrainParameters.NumStackedVectorObservations = 1;
    }

    private int GetObservationSize()
    {
        /*
         * Root:
         * local velocity       = 3
         * local angular vel.   = 3
         * local up direction   = 3
         * pelvis height        = 1
         * Totaal root          = 10
         *
         * Iedere joint:
         * joint position       = 1
         * joint velocity       = 1
         * target difference    = 1
         * Totaal per joint     = 3
         */
        return 10 + controlledJoints.Count * 3;
    }

    public override void OnEpisodeBegin()
    {
        if (!initialized)
        {
            return;
        }

        ResetRobot();
    }

    private void ResetRobot()
    {
        stepsSinceReset = 0;
        episodeLifetimeSeconds = 0f;

        Quaternion resetRotation = initialRootRotation;

        if (randomizeStartRotation)
        {
            float randomYaw = Random.Range(
                -randomYawRange,
                randomYawRange);

            float randomPitch = Random.Range(
                -randomTiltRange,
                randomTiltRange);

            float randomRoll = Random.Range(
                -randomTiltRange,
                randomTiltRange);

            Quaternion randomRotation = Quaternion.Euler(
                randomPitch,
                randomYaw,
                randomRoll);

            resetRotation = randomRotation * initialRootRotation;
        }

        /*
         * TeleportRoot verplaatst de root van de hele articulation.
         */
        rootBody.TeleportRoot(
            initialRootPosition,
            resetRotation);

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

            drive.target = initialDriveTargets[i];
            drive.targetVelocity = 0f;

            joint.xDrive = drive;
        }
    }


    /// <summary>
    /// Geeft iedere physics-stap een kleine survival reward.
    /// Daardoor levert langer overeind blijven automatisch meer totale reward op.
    /// Time.fixedDeltaTime zorgt ervoor dat dit echt reward per gesimuleerde seconde is.
    /// </summary>
    private void FixedUpdate()
    {
        if (!initialized || rootBody == null)
        {
            return;
        }

        episodeLifetimeSeconds += Time.fixedDeltaTime;

        // resetGraceSteps wordt in OnActionReceived geteld en is dus in decision-steps.
        if (stepsSinceReset <= resetGraceSteps)
        {
            return;
        }

        float pelvisHeight = rootBody.transform.position.y;
        float uprightness = Vector3.Dot(
            rootBody.transform.up,
            Vector3.up);

        bool isStillAlive =
            pelvisHeight >= minimumPelvisHeight &&
            uprightness >= minimumUprightness &&
            IsFinite(rootBody.transform.position) &&
            IsFinite(rootBody.linearVelocity) &&
            IsFinite(rootBody.angularVelocity);

        if (isStillAlive)
        {
            AddReward(survivalRewardPerSecond * Time.fixedDeltaTime);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVelocity =
            rootBody.transform.InverseTransformDirection(
                rootBody.linearVelocity);

        Vector3 localAngularVelocity =
            rootBody.transform.InverseTransformDirection(
                rootBody.angularVelocity);

        Vector3 worldUpInRobotSpace =
            rootBody.transform.InverseTransformDirection(
                Vector3.up);

        sensor.AddObservation(localVelocity / 5f);
        sensor.AddObservation(localAngularVelocity / 10f);
        sensor.AddObservation(worldUpInRobotSpace);
        sensor.AddObservation(rootBody.transform.position.y / 2f);

        foreach (ArticulationBody joint in controlledJoints)
        {
            ArticulationDrive drive = joint.xDrive;

            float jointPosition = GetJointPositionDegrees(joint);
            float jointVelocity = GetJointVelocity(joint);

            float normalizedPosition = NormalizeJointPosition(
                jointPosition,
                drive.lowerLimit,
                drive.upperLimit);

            float normalizedVelocity =
                Mathf.Clamp(jointVelocity / 10f, -1f, 1f);

            float normalizedTargetDifference =
                NormalizeTargetDifference(
                    drive.target - jointPosition,
                    drive.lowerLimit,
                    drive.upperLimit);

            sensor.AddObservation(normalizedPosition);
            sensor.AddObservation(normalizedVelocity);
            sensor.AddObservation(normalizedTargetDifference);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!initialized)
        {
            return;
        }

        ActionSegment<float> actions =
            actionBuffers.ContinuousActions;

        if (actions.Length != controlledJoints.Count)
        {
            Debug.LogError(
                $"ML-Agents gaf {actions.Length} actions, " +
                $"maar er zijn {controlledJoints.Count} joints gevonden.",
                this);

            EndEpisode();
            return;
        }

        float totalSquaredAction = 0f;

        for (int i = 0; i < controlledJoints.Count; i++)
        {
            float action = Mathf.Clamp(actions[i], -1f, 1f);

            ArticulationBody joint = controlledJoints[i];
            ArticulationDrive drive = joint.xDrive;

            float targetChange = action * actionSpeed;

            drive.target += targetChange;

            /*
             * Alleen clampen wanneer de joint geldige limits heeft.
             */
            if (drive.upperLimit > drive.lowerLimit)
            {
                drive.target = Mathf.Clamp(
                    drive.target,
                    drive.lowerLimit,
                    drive.upperLimit);
            }

            joint.xDrive = drive;

            totalSquaredAction += action * action;
        }

        stepsSinceReset++;

        if (stepsSinceReset <= resetGraceSteps)
        {
            return;
        }

        GiveStandingReward(totalSquaredAction);
        CheckFallConditions();
    }

    private void GiveStandingReward(float totalSquaredAction)
    {
        Vector3 pelvisUp = rootBody.transform.up;

        float uprightness = Mathf.Clamp01(
            Vector3.Dot(pelvisUp, Vector3.up));

        float heightDifference = Mathf.Abs(
            rootBody.transform.position.y - desiredPelvisHeight);

        float heightScore = Mathf.Clamp01(
            1f - heightDifference / Mathf.Max(desiredPelvisHeight, 0.01f));

        float poseScore = CalculateInitialPoseScore();

        float rootMovement =
            rootBody.linearVelocity.sqrMagnitude +
            rootBody.angularVelocity.sqrMagnitude;

        float normalizedActionPenalty =
            totalSquaredAction /
            Mathf.Max(controlledJoints.Count, 1);

        AddReward(uprightness * uprightRewardScale);
        AddReward(heightScore * heightRewardScale);
        AddReward(poseScore * poseRewardScale);

        AddReward(
            -rootMovement * velocityPenaltyScale);

        AddReward(
            -normalizedActionPenalty * actionPenaltyScale);
    }

    /// <summary>
    /// Beloont de agent wanneer hij ongeveer in zijn oorspronkelijke pose blijft.
    /// Hierdoor wordt direct volledig inklappen minder aantrekkelijk.
    /// </summary>
    private float CalculateInitialPoseScore()
    {
        if (controlledJoints.Count == 0)
        {
            return 0f;
        }

        float totalScore = 0f;

        for (int i = 0; i < controlledJoints.Count; i++)
        {
            ArticulationBody joint = controlledJoints[i];
            ArticulationDrive drive = joint.xDrive;

            float currentPosition = GetJointPositionDegrees(joint);
            float initialTarget = initialDriveTargets[i];

            float range = Mathf.Abs(
                drive.upperLimit - drive.lowerLimit);

            range = Mathf.Max(range, 1f);

            float difference = Mathf.Abs(
                currentPosition - initialTarget);

            float jointScore = Mathf.Clamp01(
                1f - difference / range);

            totalScore += jointScore;
        }

        return totalScore / controlledJoints.Count;
    }

    private void CheckFallConditions()
    {
        float pelvisHeight = rootBody.transform.position.y;

        float uprightness = Vector3.Dot(
            rootBody.transform.up,
            Vector3.up);

        bool pelvisTooLow =
            pelvisHeight < minimumPelvisHeight;

        bool robotTiltedTooFar =
            uprightness < minimumUprightness;

        bool invalidPhysics =
            !IsFinite(rootBody.transform.position) ||
            !IsFinite(rootBody.linearVelocity) ||
            !IsFinite(rootBody.angularVelocity);

        if (pelvisTooLow || robotTiltedTooFar || invalidPhysics)
        {
            AddReward(-1f);
            EndEpisode();
        }
    }

    private float GetJointPositionDegrees(ArticulationBody joint)
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

    private float GetJointVelocity(ArticulationBody joint)
    {
        if (joint.jointVelocity.dofCount == 0)
        {
            return 0f;
        }

        return joint.jointVelocity[0];
    }

    private float NormalizeJointPosition(
        float value,
        float minimum,
        float maximum)
    {
        if (maximum <= minimum)
        {
            return 0f;
        }

        return Mathf.Clamp(
            Mathf.InverseLerp(minimum, maximum, value) * 2f - 1f,
            -1f,
            1f);
    }

    private float NormalizeTargetDifference(
        float difference,
        float minimum,
        float maximum)
    {
        float range = Mathf.Abs(maximum - minimum);

        if (range < 0.001f)
        {
            return 0f;
        }

        return Mathf.Clamp(
            difference / range,
            -1f,
            1f);
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
        ActionSegment<float> actions =
            actionsOut.ContinuousActions;

        /*
         * Heuristic test:
         * - alle joints blijven normaal op 0 action;
         * - omhoog/omlaag beweegt de eerste gevonden joint;
         * - links/rechts beweegt de tweede gevonden joint.
         */
        for (int i = 0; i < actions.Length; i++)
        {
            actions[i] = 0f;
        }

        if (actions.Length > 0)
        {
            actions[0] = Input.GetAxisRaw("Vertical");
        }

        if (actions.Length > 1)
        {
            actions[1] = Input.GetAxisRaw("Horizontal");
        }
    }
}