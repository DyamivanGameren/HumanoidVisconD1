using System.Collections;
using UnityEngine;

public class SpawnerV2 : MonoBehaviour
{
    [Header("Prefab")]
    public GameObject prefabToSpawn;

    [Header("Spawn Settings")]
    public int spawnAmount = 1;
    public float spawnInterval = 2f;
    public float objectMass = 1f;

    private IEnumerator Start()
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