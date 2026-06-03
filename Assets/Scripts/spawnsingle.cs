using UnityEngine;

public class Spawner : MonoBehaviour
{
    public GameObject objectToSpawn;

    void Start()
    {
        Instantiate(objectToSpawn, transform.position, Quaternion.identity);
    }
}