using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;
using System;
using Unity.Cinemachine;

public partial class KartInput
{
    // Input callback methods
    private void OnLookBack(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        Debug.Log("Look Back Activated");
        if (kartApex.lookbackCamera != null && kartApex.kartCamera != null)
        {
            kartApex.lookbackCamera.Priority = 20;
            kartApex.kartCamera.Priority = 10;
        }
    }
    private void OnLookBackCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        Debug.Log("Look Back Deactivated");
        if (kartApex.lookbackCamera != null && kartApex.kartCamera != null)
        {
            kartApex.lookbackCamera.Priority = 10;
            kartApex.kartCamera.Priority = 20;
        }
    }
    private void OnBrake(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        onBrake.Invoke();
    }
    private void OnSteerCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        steerInput = 0f;
    }
    public void OnSteer(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        float rawInput = context.ReadValue<float>();
        steerInput = Mathf.Abs(rawInput) > kartApex.kartGameSettings.inputDeadzone ? rawInput : 0f;
        onSteer.Invoke();
    }
    public void OnDrift(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        if (currentState == KartState.Grounded && isAccelerating && steerInput != 0 && !isDrifting)
        {
            isDrifting = true;
            driftTimer = 0f;
            onDrift.Invoke();
        }
        
    }
    public void OnDriftCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        if (isDrifting)
        {
            isDrifting = false;
            float driftDuration = driftTimer;
            driftTimer = 0f;
            if (driftDuration >= kartApex.kartGameSettings.driftTimeToOrangeBoost && !kartApex.autoDrift)
            {
                Debug.Log("Orange drift boost");
                onDriftOrangeBoost.Invoke();
            }
            else if (driftDuration >= kartApex.kartGameSettings.driftTimeToBlueBoost && !kartApex.autoDrift)
            {
                Debug.Log("Blue drift boost");
                onDriftBlueBoost.Invoke();
            }
        }
    }
    private void OnAcceleratedCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        isAccelerating = false;
        if (isDrifting)
        {
            kartApex.BL_particleSystem.SetActive(false);
            kartApex.BR_particleSystem.SetActive(false);
            isDrifting = false;
            driftTimer = 0f;
        }
        onDecelerate.Invoke();
    }
    public void OnAccelerate(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        isAccelerating = true;
        onAccelerate.Invoke();
    }
    private void OnTrick(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        if (isOnRamp && currentState == KartState.Grounded)
        {
            trickPerformedOnRamp = true;
            isPerformingTrick = true;
            currentState = KartState.PerformingTrick;
            onTrick.Invoke();
            Debug.Log("Trick performed on ramp takeoff!");
        }
    }

    // Trigger methods
    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("JumpRamp"))
        {
            isOnRamp = true;
            return;
        }
        
        // Check for terrain zone triggers
        TerrainType detectedTerrain = GetTerrainTypeFromTag(other.tag);
        if (detectedTerrain != TerrainType.Ground)
        {
            activeTerrainZones[detectedTerrain]++;
            UpdateCurrentTerrainType();
        }
    }

    void OnTriggerStay(Collider other)
    {
        if (kartApex.frozen) return;
        // Predictive Jump Takeoff
        if (isOnRamp && other.CompareTag("JumpRamp"))
        {
            Vector3 rayOrigin = transform.position + transform.forward * 0.5f;
            RaycastHit hit;
            // Check if the raycast from the front of the kart leaves the ramp
            if (!Physics.Raycast(rayOrigin, transform.forward, out hit, kartApex.kartGameSettings.jumpRayLength, kartApex.kartData.groundLayer))
            {
                HandleRampJump();
            }
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("JumpRamp"))
        {
            isOnRamp = false;
            // Reset trick state if leaving ramp without jumping
            if (!isPerformingArcJump && trickPerformedOnRamp)
            {
                trickPerformedOnRamp = false;
                isPerformingTrick = false;
                Debug.Log("Trick cancelled - left ramp without jumping");
            }
            return;
        }
        
        // Check for terrain zone triggers  
        TerrainType detectedTerrain = GetTerrainTypeFromTag(other.tag);
        if (detectedTerrain != TerrainType.Ground)
        {
            activeTerrainZones[detectedTerrain] = Mathf.Max(0, activeTerrainZones[detectedTerrain] - 1);
            UpdateCurrentTerrainType();
        }
    }

    // OnEnable/OnDisable methods
    public void OnEnable()
    {
        foreach (var action in allActions)
        {
            action.Enable();
        }
    }
    public void OnDisable()
    {
        foreach (var action in allActions)
        {
            action.Disable();
        }
    }
}