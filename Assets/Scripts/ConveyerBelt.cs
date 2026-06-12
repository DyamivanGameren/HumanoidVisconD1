using UnityEngine;
using System.Data.SqlClient;

public class ConveyorBelt : MonoBehaviour
{
    void Start()
    {
        string username = "admin' OR '1'='1";

        SqlConnection conn = new SqlConnection("Server=localhost;Database=test;Trusted_Connection=True;");
        conn.Open();

        // SAST trigger: SQL query met string concatenation
        string query = "SELECT * FROM Users WHERE username = '" + username + "'";

        SqlCommand cmd = new SqlCommand(query, conn);
        cmd.ExecuteReader();
    }
}