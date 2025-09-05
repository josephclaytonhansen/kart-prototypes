using UnityEngine;
using System;

public partial class KartInput
{
    private bool CheckGround()
    {
        groundHitCount = 0;
        Vector3 tempAveragedNormal = Vector3.zero;

        // IMPROVED: Mario Kart style ground detection - always cast from kart center regardless of orientation
        Vector3 kartCenter = transform.position;
        
        // Primary ground check from kart center (most reliable)
        if (Physics.Raycast(kartCenter, Vector3.down, out RaycastHit centerHit, kartApex.kartData.groundCheckDistance * 1.5f, kartApex.kartData.groundLayer))
        {
            groundHits[groundHitCount] = centerHit;
            tempAveragedNormal += centerHit.normal;
            groundHitCount++;
        }

        // Secondary checks from wheel positions (for slope detection)
        for (int i = 0; i < wheelTransforms.Length; i++)
        {
            // Always cast straight down in world space, regardless of kart orientation
            Vector3 raycastOrigin = wheelTransforms[i].position;
            Vector3 raycastDirection = Vector3.down;

            if (Physics.Raycast(raycastOrigin, raycastDirection, out RaycastHit wheelHit, kartApex.kartData.groundCheckDistance, kartApex.kartData.groundLayer))
            {
                // Only add if we haven't exceeded our hit array size
                if (groundHitCount < groundHits.Length)
                {
                    groundHits[groundHitCount] = wheelHit;
                    tempAveragedNormal += wheelHit.normal;
                    groundHitCount++;
                }
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