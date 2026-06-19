using System.Collections;
using UnityEngine;

public class SpawnerV2 : MonoBehaviour
{
    public GameObject prefabToSpawn;

    public int spawnAmount = 1;
    public float spawnInterval = 30f;
    public float objectMass = 1f;

    private Coroutine spawnRoutine;

    public void StartSpawning()
    {
        if (spawnRoutine == null)
        {
            spawnRoutine = StartCoroutine(SpawnLoop());
            Debug.Log("Spawner gestart");
        }
    }

    public void StopSpawning()
    {
        if (spawnRoutine != null)
        {
            StopCoroutine(spawnRoutine);
            spawnRoutine = null;
            Debug.Log("Spawner gestopt");
        }
    }

    private IEnumerator SpawnLoop()
    {
        while (true)
        {
            for (int i = 0; i < spawnAmount; i++)
            {
                GameObject obj = Instantiate(
                    prefabToSpawn,
                    transform.position,
                    Quaternion.identity
                );

                Rigidbody rb = obj.GetComponent<Rigidbody>();

                if (rb != null)
                {
                    rb.mass = objectMass;
                }
            }

            yield return new WaitForSeconds(spawnInterval);
        }
    }

    public void SetSpawnAmount(float value)
    {
        spawnAmount = Mathf.RoundToInt(value);
    }

    public void SetSpawnInterval(float value)
    {
        spawnInterval = value;
    }

    public void SetObjectMass(float value)
    {
        objectMass = value;
    }
}