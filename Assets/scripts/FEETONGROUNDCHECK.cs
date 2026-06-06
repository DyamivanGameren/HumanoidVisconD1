using UnityEngine;
using System.Collections.Generic;
public static class ColliderExrension
{
    public static bool ComputePenetration(this Collider source, Collider target, out Vector3 direction, out float distance) 
    {
        direction = Vector3.zero;
        distance = 0f;
        if (source == null || target == null) { return false; }

        return Physics.ComputePenetration(source, source.transform.position, source.transform.rotation, target, target.transform.position, target.transform.rotation, out direction, out distance);
    }

}