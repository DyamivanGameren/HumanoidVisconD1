using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public class ConveyorBeltTests
{
    [UnityTest]
    public IEnumerator ConveyorBelt_MovesRigidbody_Forward()
    {
        // Arrange
        GameObject beltObj = new GameObject("ConveyorBelt");
        ConveyorBelt belt = beltObj.AddComponent<ConveyorBelt>();
        belt.speed = 5f;
        belt.direction = Vector3.right;

        GameObject boxObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        boxObj.transform.position = Vector3.zero;

        Rigidbody rb = boxObj.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.isKinematic = false;

        // Zorg dat ze contact maken (simpel op elkaar zetten)
        boxObj.transform.position = beltObj.transform.position;

        Vector3 startPos = rb.position;

        // Act (laat physics een paar frames draaien)
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        // Assert
        Assert.Greater(rb.position.x, startPos.x);

        // Cleanup
        Object.DestroyImmediate(beltObj);
        Object.DestroyImmediate(boxObj);
    }
}