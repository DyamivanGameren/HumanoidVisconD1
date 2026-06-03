using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System.Collections;

public class ObjectSpawnerTests
{
    private GameObject spawnerObject;
    private ObjectSpawner spawner;

    private GameObject prefabA;
    private GameObject prefabB;

    [SetUp]
    public void SetUp()
    {
        spawnerObject = new GameObject("Spawner");
        spawner = spawnerObject.AddComponent<ObjectSpawner>();

        prefabA = new GameObject("ObjectA");
        prefabB = new GameObject("ObjectB");

        spawner.objectA = prefabA;
        spawner.objectB = prefabB;
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(spawnerObject);
        Object.DestroyImmediate(prefabA);
        Object.DestroyImmediate(prefabB);
    }

    [UnityTest]
    public IEnumerator SpawnObject_AlternatesBetweenAAndB()
    {
        spawner.SpawnObject(); // A
        yield return null;

        spawner.SpawnObject(); // B
        yield return null;

        GameObject a = GameObject.Find("ObjectA(Clone)");
        GameObject b = GameObject.Find("ObjectB(Clone)");

        Assert.IsNotNull(a);
        Assert.IsNotNull(b);
    }
}