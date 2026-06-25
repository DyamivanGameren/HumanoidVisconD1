using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public class SpawnerV2Tests
{
    private GameObject spawnerObject;
    private SpawnerV2 spawner;
    private GameObject prefab;

    [SetUp]
    public void SetUp()
    {
        spawnerObject = new GameObject("Spawner");
        spawner = spawnerObject.AddComponent<SpawnerV2>();

        prefab = new GameObject("SpawnPrefab");
        prefab.AddComponent<Rigidbody>();

        spawner.prefabToSpawn = prefab;
        spawner.spawnAmount = 1;
        spawner.spawnInterval = 0.1f;
        spawner.objectMass = 5f;
    }

    [TearDown]
    public void TearDown()
    {
        UnityEngine.Object.DestroyImmediate(spawnerObject);
        UnityEngine.Object.DestroyImmediate(prefab);

        foreach (GameObject obj in UnityEngine.Object.FindObjectsByType<GameObject>(FindObjectsSortMode.None))
        {
            if (obj != null && obj.name.Contains("SpawnPrefab"))
            {
                UnityEngine.Object.DestroyImmediate(obj);
            }
        }
    }

    [Test]
    public void SetSpawnAmount_RoundsFloatToInt()
    {
        spawner.SetSpawnAmount(2.7f);

        Assert.AreEqual(3, spawner.spawnAmount);
    }

    [Test]
    public void SetSpawnInterval_UpdatesInterval()
    {
        spawner.SetSpawnInterval(15f);

        Assert.AreEqual(15f, spawner.spawnInterval);
    }

    [Test]
    public void SetObjectMass_UpdatesMass()
    {
        spawner.SetObjectMass(10f);

        Assert.AreEqual(10f, spawner.objectMass);
    }

    [UnityTest]
    public IEnumerator StartSpawning_SpawnsObject()
    {
        spawner.StartSpawning();

        yield return null;

        GameObject spawned = GameObject.Find("SpawnPrefab(Clone)");

        Assert.IsNotNull(spawned);
    }

    [UnityTest]
    public IEnumerator StopSpawning_StopsNewSpawns()
    {
        spawner.spawnInterval = 0.05f;

        spawner.StartSpawning();
        yield return null;

        spawner.StopSpawning();

        int countAfterStop = CountSpawnedObjects();

        yield return new WaitForSeconds(0.1f);

        Assert.AreEqual(countAfterStop, CountSpawnedObjects());
    }

    private int CountSpawnedObjects()
    {
        int count = 0;

        foreach (GameObject obj in UnityEngine.Object.FindObjectsByType<GameObject>(FindObjectsSortMode.None))
        {
            if (obj != null && obj.name.Contains("SpawnPrefab(Clone)"))
            {
                count++;
            }
        }

        return count;
    }
}