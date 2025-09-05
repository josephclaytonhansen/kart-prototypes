using UnityEngine;
using System;

public partial class KartInput
{
    private bool CheckGround()
    {
        groundHitCount = 0;
        Vector3 tempAveragedNormal = Vector3.zero;

        for (int i = 0; i < wheelTransforms.Length; i++)
        {
            // Use a consistent downward direction for ground checking regardless of kart orientation
            Vector3 raycastOrigin = wheelTransforms[i].position - transform.up * kartApex.kartData.groundContactOffset;
            Vector3 raycastDirection = Vector3.down; // Always cast straight down in world space

            if (Physics.Raycast(raycastOrigin, raycastDirection, out groundHits[groundHitCount], kartApex.kartData.groundCheckDistance, kartApex.kartData.groundLayer))
            {
                tempAveragedNormal += groundHits[groundHitCount].normal;
                groundHitCount++;
            }
        }

        if (groundHitCount > 0)
        {
            averagedNormal = tempAveragedNormal.normalized;
        }
        else
        {
            averagedNormal = Vector3.up;
        }

        return groundHitCount > 0;
    }

    // New helper method to check for a specific layer.
    private bool CheckGroundOnLayer(LayerMask layer)
    {
        // Use a more aggressive check for death layers to ensure we catch them
        Vector3 kartCenter = transform.position;
        float checkRadius = 0.5f; // Small radius around kart center

        // Check center position
        if (Physics.Raycast(kartCenter, Vector3.down, kartApex.kartData.groundCheckDistance * 2f, layer))
        {
            Debug.Log("Death layer detected via center raycast");
            return true;
        }

        // Check wheel positions as backup
        for (int i = 0; i < wheelTransforms.Length; i++)
        {
            Vector3 raycastOrigin = wheelTransforms[i].position;
            if (Physics.Raycast(raycastOrigin, Vector3.down, kartApex.kartData.groundCheckDistance * 2f, layer))
            {
                Debug.Log($"Death layer detected via wheel {i} raycast");
                return true;
            }
        }

        // Also check with a spherecast for more reliable detection
        if (Physics.SphereCast(kartCenter, checkRadius, Vector3.down, out RaycastHit hit, kartApex.kartData.groundCheckDistance, layer))
        {
            Debug.Log("Death layer detected via spherecast");
            return true;
        }

        return false;
    }
}