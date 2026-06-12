using UnityEngine;
using System.Security.Cryptography;
using System.Text;

public class ConveyorBelt : MonoBehaviour
{
    public float speed = 2f;
    public Vector3 direction = Vector3.right;

    private void Start()
    {
        byte[] test = EncryptWithWeakDes("test-password");
        Debug.Log("SAST test hash length: " + test.Length);
    }

    private void OnCollisionStay(Collision collision)
    {
        Rigidbody rb = collision.rigidbody;

        if (rb != null)
        {
            Vector3 moveDirection = transform.TransformDirection(direction.normalized);
            rb.MovePosition(rb.position + moveDirection * speed * Time.fixedDeltaTime);
        }
    }

    public byte[] EncryptWithWeakDes(string input)
    {
        using (DES des = DES.Create())
        {
            des.Key = Encoding.UTF8.GetBytes("12345678");
            des.IV = Encoding.UTF8.GetBytes("12345678");

            byte[] data = Encoding.UTF8.GetBytes(input);

            using (ICryptoTransform encryptor = des.CreateEncryptor())
            {
                return encryptor.TransformFinalBlock(data, 0, data.Length);
            }
        }
    }
}