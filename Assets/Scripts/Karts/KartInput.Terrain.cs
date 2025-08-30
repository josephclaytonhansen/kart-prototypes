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
        
        // Initialize previous terrain type
        previousTerrainType = TerrainType.Ground;
    }
    
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
        TerrainType newTerrainType;
        
        // Priority order: Death > Ice > Mud > Sand > OffRoad > Ground
        if (activeTerrainZones[TerrainType.Death] > 0){
            newTerrainType = TerrainType.Death;
            // Death handling is now done in FixedUpdate, not here
            Debug.Log("Death terrain zone detected");
        }
        else if (activeTerrainZones[TerrainType.Ice] > 0)
            newTerrainType = TerrainType.Ice;
        else if (activeTerrainZones[TerrainType.Mud] > 0)
            newTerrainType = TerrainType.Mud;
        else if (activeTerrainZones[TerrainType.Sand] > 0)
            newTerrainType = TerrainType.Sand;
        else if (activeTerrainZones[TerrainType.OffRoad] > 0)
            newTerrainType = TerrainType.OffRoad;
        else
            newTerrainType = TerrainType.Ground;
        
        // Only update if terrain actually changed
        if (newTerrainType != terrainType)
        {
            Debug.Log($"Terrain changing from {terrainType} to {newTerrainType}");
            previousTerrainType = terrainType;
            terrainType = newTerrainType;
            
            // Start terrain change smoothing to prevent jitter
            terrainChangeSmoothing = TERRAIN_CHANGE_SMOOTHING_DURATION;
        }
    }
    
        // New helper methods for smooth terrain transitions
    private float GetTerrainSpeedMultiplier()
    {
        float baseMultiplier = 1f;
        switch (terrainType)
        {
            case TerrainType.Ground:
                baseMultiplier = 1f;
                break;
            case TerrainType.Ice:
                baseMultiplier = kartApex.kartData.iceFriction;
                break;
            case TerrainType.Mud:
                baseMultiplier = kartApex.kartData.mudFriction;
                break;
            case TerrainType.Sand:
                baseMultiplier = kartApex.kartData.sandFriction;
                break;
            case TerrainType.OffRoad:
                baseMultiplier = kartApex.kartData.offRoadFriction;
                break;
        }
        
        // Apply smoothing during terrain changes
        if (terrainChangeSmoothing > 0f)
        {
            float previousMultiplier = GetPreviousTerrainSpeedMultiplier();
            float t = 1f - (terrainChangeSmoothing / TERRAIN_CHANGE_SMOOTHING_DURATION);
            baseMultiplier = Mathf.Lerp(previousMultiplier, baseMultiplier, t);
        }
        
        return baseMultiplier;
    }
    
    private float GetTerrainSteerMultiplier()
    {
        float baseMultiplier = 1f;
        switch (terrainType)
        {
            case TerrainType.Ground:
                baseMultiplier = 1f;
                break;
            case TerrainType.Ice:
                baseMultiplier = kartApex.kartData.iceSteer;
                break;
            case TerrainType.Mud:
                baseMultiplier = kartApex.kartData.mudSteer;
                break;
            case TerrainType.Sand:
                baseMultiplier = kartApex.kartData.sandSteer;
                break;
            case TerrainType.OffRoad:
                baseMultiplier = kartApex.kartData.offRoadSteer;
                break;
        }
        
        // Apply smoothing during terrain changes
        if (terrainChangeSmoothing > 0f)
        {
            float previousMultiplier = GetPreviousTerrainSteerMultiplier();
            float t = 1f - (terrainChangeSmoothing / TERRAIN_CHANGE_SMOOTHING_DURATION);
            baseMultiplier = Mathf.Lerp(previousMultiplier, baseMultiplier, t);
        }
        
        return baseMultiplier;
    }
    
    private float GetTerrainDriftMultiplier()
    {
        float baseMultiplier = 1f;
        switch (terrainType)
        {
            case TerrainType.Ground:
                baseMultiplier = 1f;
                break;
            case TerrainType.Ice:
                baseMultiplier = kartApex.kartData.iceDrift;
                break;
            case TerrainType.Mud:
                baseMultiplier = kartApex.kartData.mudDrift;
                break;
            case TerrainType.Sand:
                baseMultiplier = kartApex.kartData.sandDrift;
                break;
            case TerrainType.OffRoad:
                baseMultiplier = kartApex.kartData.offRoadDrift;
                break;
        }
        
        // Apply smoothing during terrain changes
        if (terrainChangeSmoothing > 0f)
        {
            float previousMultiplier = GetPreviousTerrainDriftMultiplier();
            float t = 1f - (terrainChangeSmoothing / TERRAIN_CHANGE_SMOOTHING_DURATION);
            baseMultiplier = Mathf.Lerp(previousMultiplier, baseMultiplier, t);
        }
        
        return baseMultiplier;
    }
    
    private float GetPreviousTerrainSpeedMultiplier()
    {
        switch (previousTerrainType)
        {
            case TerrainType.Ground:
                return 1f;
            case TerrainType.Ice:
                return kartApex.kartData.iceFriction;
            case TerrainType.Mud:
                return kartApex.kartData.mudFriction;
            case TerrainType.Sand:
                return kartApex.kartData.sandFriction;
            case TerrainType.OffRoad:
                return kartApex.kartData.offRoadFriction;
            default:
                return 1f;
        }
    }
    
    private float GetPreviousTerrainSteerMultiplier()
    {
        switch (previousTerrainType)
        {
            case TerrainType.Ground:
                return 1f;
            case TerrainType.Ice:
                return kartApex.kartData.iceSteer;
            case TerrainType.Mud:
                return kartApex.kartData.mudSteer;
            case TerrainType.Sand:
                return kartApex.kartData.sandSteer;
            case TerrainType.OffRoad:
                return kartApex.kartData.offRoadSteer;
            default:
                return 1f;
        }
    }
    
    private float GetPreviousTerrainDriftMultiplier()
    {
        switch (previousTerrainType)
        {
            case TerrainType.Ground:
                return 1f;
            case TerrainType.Ice:
                return kartApex.kartData.iceDrift;
            case TerrainType.Mud:
                return kartApex.kartData.mudDrift;
            case TerrainType.Sand:
                return kartApex.kartData.sandDrift;
            case TerrainType.OffRoad:
                return kartApex.kartData.offRoadDrift;
            default:
                return 1f;
        }
    }
}