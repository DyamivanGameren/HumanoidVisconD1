using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;

public class MovementViaCode : MonoBehaviour
{
    private ArticulationBody[] articulationChain;
    public GameObject robot;
    //public ArticulationBody shoulderJoint;

    public float amplitude = 45f; // graden
    public float speed = 1f;      // oscillatiesnelheid
    public float stiffness = 100;
    public float damping = 0;
    public float forceLimit = 100;
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    private void Start()
    {

        //this.gameObject.AddComponent<FKRobot>();
        articulationChain = robot.GetComponentsInChildren<ArticulationBody>();
        Debug.Log("Found " + articulationChain.Length + " articulation bodies");

        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }

    }
    void Update()
    {
        float targetAngle = Mathf.Sin(Time.time * speed) * amplitude;

        ArticulationDrive drive = articulationChain[5].xDrive;
        drive.target = targetAngle;
        articulationChain[5].xDrive = drive;
    }
}
