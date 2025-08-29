using UnityEngine;
using System;
using System.Collections.Generic;

public partial class KartInput
{
    void Start()
    {
        // Initialize terrain zone counters
        foreach (TerrainType terrainType in System.Enum.GetValues(typeof(TerrainType)))
        {
            activeTerrainZones[terrainType] = 0;
        }
    }
    
    // These methods will be integrated into the existing OnTriggerEnter/Exit in main KartInput.cs
    
    private TerrainType GetTerrainTypeFromTag(string tag)
    {
        switch (tag)
        {
            case "TerrainMud": return TerrainType.Mud;
            case "TerrainIce": return TerrainType.Ice;
            case "TerrainSand": return TerrainType.Sand;
            case "TerrainDeath": return TerrainType.Death;
            case "TerrainOffRoad": return TerrainType.OffRoad;
            default: return TerrainType.Ground;
        }
    }
    
    private void UpdateCurrentTerrainType()
    {
        // Priority order: Death > Ice > Mud > Sand > OffRoad > Ground
        if (activeTerrainZones[TerrainType.Death] > 0){
            terrainType = TerrainType.Death;
            kartApex.frozen = true;
            onGroundedOnDeathLayer.Invoke(lastGroundedPosition);
        }
        else if (activeTerrainZones[TerrainType.Ice] > 0)
            terrainType = TerrainType.Ice;
        else if (activeTerrainZones[TerrainType.Mud] > 0)
            terrainType = TerrainType.Mud;
        else if (activeTerrainZones[TerrainType.Sand] > 0)
            terrainType = TerrainType.Sand;
        else if (activeTerrainZones[TerrainType.OffRoad] > 0)
            terrainType = TerrainType.OffRoad;
        else
            terrainType = TerrainType.Ground;
            
        Debug.Log($"Terrain updated to: {terrainType}");
    }
    
    // Remove the old ground-based terrain detection methods
    // Keep the terrain effect methods but make them public so they can be called from HandleDriving/HandleSteering
    
    public float HandleTerrainSpeed(float currentSpeed)
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

    public float HandleTerrainSteer(float steer)
    {
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

    public float HandleTerrainDrift(float drift)
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