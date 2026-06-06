using UnityEngine;

public class RobotSpawner : MonoBehaviour
{
    public static RobotSpawner Instance;

    [SerializeField] private GameObject robotPrefab;
    [SerializeField] private Transform spawnPoint;

    private GameObject currentRobot;

    private void Awake()
    {
        Instance = this;
    }

    private void Start()
    {
        SpawnRobot();
    }

    public void RespawnRobot()
    {
        if (currentRobot != null)
        {
            Destroy(currentRobot);
        }

        SpawnRobot();
    }

    private void SpawnRobot()
    {
        currentRobot = Instantiate(
            robotPrefab,
            spawnPoint.position,
            spawnPoint.rotation);
    }
}
