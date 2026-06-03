using UnityEngine;

public class ObjectSpawner : MonoBehaviour
{
    public GameObject objectA;
    public GameObject objectB;

    public float spawnInterval = 1f;

    private bool spawnA = true;

    void Start()
    {
        InvokeRepeating(nameof(SpawnObject), 0f, spawnInterval);
    }

    void SpawnObject()
    {
        if (spawnA)
        {
            Instantiate(objectA, transform.position, Quaternion.identity);
        }
        else
        {
            Instantiate(objectB, transform.position, Quaternion.identity);
        }

        spawnA = !spawnA; // Wissel tussen A en B
    }
}