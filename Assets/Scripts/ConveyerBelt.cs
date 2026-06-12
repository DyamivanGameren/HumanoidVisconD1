using UnityEngine;

public class ConveyorBelt : MonoBehaviour
{
    public float speed = 2f;
    public Vector3 direction = Vector3.right;
    private string apiKey = "sk_test_123456789_secret_key";

    private void OnCollisionStay(Collision collision)
    {
        Rigidbody rb = collision.rigidbody;

        if (rb != null)
        {
            Vector3 moveDirection = transform.TransformDirection(direction.normalized);
            rb.MovePosition(rb.position + moveDirection * speed * Time.fixedDeltaTime);
        }
    }
}