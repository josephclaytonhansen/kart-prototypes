using UnityEngine;
using System;
using System.Collections.Generic;

public partial class KartInput
{
    
    private void GetTerrainType(RaycastHit[] hits)
    {
        bool foundIce = false, foundMud = false, foundSand = false, foundOffRoad = false, foundDeath = false;
        
        // Debug: Log what each wheel is hitting
        for (int i = 0; i < hits.Length; i++)
        {
            RaycastHit hit = hits[i];
            if (hit.collider != null)
            {
                Debug.Log($"Wheel {i}: Hit {hit.collider.name} on layer {hit.collider.gameObject.layer} at distance {hit.distance}");
            }
        }
        
        // Check each wheel's hit (these should be the closest hits per wheel from CheckGround)
        for (int i = 0; i < hits.Length; i++)
        {
            RaycastHit hit = hits[i];
            if (hit.collider == null) continue;
            
            int layer = hit.collider.gameObject.layer;
            
            // Check in priority order - first match wins for this wheel
            if (((1 << layer) & kartApex.iceLayer) != 0)
                foundIce = true;
            else if (((1 << layer) & kartApex.mudLayer) != 0)
                foundMud = true;
            else if (((1 << layer) & kartApex.sandLayer) != 0)
                foundSand = true;
            else if (((1 << layer) & kartApex.offRoadLayer) != 0)
                foundOffRoad = true;
            else if (((1 << layer) & kartApex.deathLayer) != 0)
                foundDeath = true;
        }
        
        // Overall priority: non-death terrain types first, death only if no other terrain found
        if (foundIce)
            terrainType = TerrainType.Ice;
        else if (foundMud)
            terrainType = TerrainType.Mud;
        else if (foundSand)
            terrainType = TerrainType.Sand;
        else if (foundOffRoad)
            terrainType = TerrainType.OffRoad;
        else if (foundDeath)
            terrainType = TerrainType.Death;
        else
            terrainType = TerrainType.Ground;
            
        Debug.Log($"Final terrain type: {terrainType}");
    }

    private float handleTerrainSpeed(float currentSpeed)
    {
        float modifiedSpeed = currentSpeed;
        switch (terrainType)
        {
            case TerrainType.Ground:
                break;
            case TerrainType.Ice:
                modifiedSpeed *= kartApex.kartData.iceFriction;
                break;
            case TerrainType.Mud:
                modifiedSpeed *= kartApex.kartData.mudFriction;
                break;
            case TerrainType.Sand:
                modifiedSpeed *= kartApex.kartData.sandFriction;
                break;
            case TerrainType.OffRoad:
                modifiedSpeed *= kartApex.kartData.offRoadFriction;
                break;
        }
        return modifiedSpeed;
    }

    private float handleTerrainSteer(float steer){
        float modifiedSteer = steer;
        switch (terrainType)
        {
            case TerrainType.Ground:
                break;
            case TerrainType.Ice:
                modifiedSteer *= kartApex.kartData.iceSteer;
                break;
            case TerrainType.Mud:
                modifiedSteer *= kartApex.kartData.mudSteer;
                break;
            case TerrainType.Sand:
                modifiedSteer *= kartApex.kartData.sandSteer;
                break;
            case TerrainType.OffRoad:
                modifiedSteer *= kartApex.kartData.offRoadSteer;
                break;
        }
        return modifiedSteer;
    }

    private float handleTerrainDrift(float drift)
    {
        float modifiedDrift = drift;
        switch (terrainType)
        {
            case TerrainType.Ground:
                break;
            case TerrainType.Ice:
                modifiedDrift *= kartApex.kartData.iceDrift;
                break;
            case TerrainType.Mud:
                modifiedDrift *= kartApex.kartData.mudDrift;
                break;
            case TerrainType.Sand:
                modifiedDrift *= kartApex.kartData.sandDrift;
                break;
            case TerrainType.OffRoad:
                modifiedDrift *= kartApex.kartData.offRoadDrift;
                break;
        }
        return modifiedDrift;
    }
}