using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;

public class SpawnerTests
{
    private GameObject spawnerObject;
    private GameObject prefab;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        // Maak een dummy prefab
        prefab = GameObject.CreatePrimitive(PrimitiveType.Cube);
        prefab.name = "TestPrefab";

        // Maak spawner object
        spawnerObject = new GameObject("Spawner");
        spawnerObject.AddComponent<Spawner>().objectToSpawn = prefab;

        yield return null;
    }

    [UnityTest]
    public IEnumerator Spawner_CreatesObject_OnStart()
    {
        // Wacht 1 frame zodat Start() wordt uitgevoerd
        yield return null;

        GameObject spawned = GameObject.Find("TestPrefab");

        Assert.IsNotNull(spawned, "Object is niet gespawned");
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(spawnerObject);
        Object.DestroyImmediate(prefab);
    }
}