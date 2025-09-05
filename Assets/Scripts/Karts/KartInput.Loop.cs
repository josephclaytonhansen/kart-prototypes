using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;
using System;
using Unity.Cinemachine;

public partial class KartInput
{
    public void Awake()
    {
        allActions = new List<InputAction>
        {
            steerAction, accelerateAction, brakeAction, trickAction, driftAction, lookBackAction
        };
        steerAction.performed += OnSteer;
        steerAction.canceled += OnSteerCanceled;
        accelerateAction.performed += OnAccelerate;
        accelerateAction.canceled += OnAcceleratedCanceled;
        brakeAction.performed += OnBrake;
        trickAction.performed += OnTrick;
        driftAction.performed += OnDrift;
        driftAction.canceled += OnDriftCanceled;
        lookBackAction.performed += OnLookBack;
        lookBackAction.canceled += OnLookBackCanceled;
        kartApex.kartRigidbody.isKinematic = true;
        kartApex.kartRigidbody.useGravity = false;
        targetPosition = transform.position;
        targetRotation = transform.rotation;
        wheelTransforms = new Transform[]
        {
            kartApex.leftFrontWheel, kartApex.rightFrontWheel, kartApex.leftBackWheel, kartApex.rightBackWheel
        };

        // Initialize terrain zone counters
        foreach (TerrainType terrainType in System.Enum.GetValues(typeof(TerrainType)))
        {
            activeTerrainZones[terrainType] = 0;
        }
        
        // Initialize speed and acceleration values
        currentMaxSpeed = kartApex.kartData.maxSpeed;
        currentAcceleration = kartApex.kartData.acceleration;
    }

    void FixedUpdate()
    {
        if (kartApex.frozen)
        {
            return; // Exit FixedUpdate if the kart is frozen
        }
        
        // Update timers
        // MKW: Immediate state transitions - no cooldown needed for arcade-style responsiveness
        if (terrainChangeSmoothing > 0f)
        {
            terrainChangeSmoothing = Mathf.Max(0f, terrainChangeSmoothing - Time.fixedDeltaTime);
        }
        
        wasGrounded = isGrounded;
        isGrounded = CheckGround();

        // Check if the kart is on a death layer regardless of grounded state
        // This ensures we catch death zones even during airborne moments
        if (CheckGroundOnLayer(kartApex.deathLayer))
        {
            Debug.Log("Death layer detected! Invoking recovery at position: " + lastGroundedPosition);
            kartApex.frozen = true;
            onGroundedOnDeathLayer.Invoke(lastGroundedPosition);
            return;
        }

        if (isGrounded)
        {
            // Don't process ground logic if we're in the middle of an arc jump
            if (isPerformingArcJump)
            {
                HandleGroundedMovement(); // Still call this for arc jump handling
                return;
            }
            
            // If the kart just became grounded and it's NOT a death zone, store a safe respawn position.
            if (!wasGrounded && !CheckGroundOnLayer(kartApex.deathLayer))
            {
                // IMPROVED: Better respawn position calculation with ground validation
                Vector3 potentialRespawnPos = transform.position;
                
                // Ensure the respawn position is actually on solid ground
                if (Physics.Raycast(potentialRespawnPos + Vector3.up * 2f, Vector3.down, out RaycastHit respawnHit, 5f, kartApex.kartData.groundLayer))
                {
                    // Use the actual ground position plus proper offset
                    lastGroundedPosition = respawnHit.point + Vector3.up * kartApex.kartData.groundContactOffset;
                    
                    // Move it back slightly in the opposite of movement direction for safety
                    Vector3 safetyOffset = -transform.forward * Mathf.Max(currentSpeed * 0.1f, 1f);
                    lastGroundedPosition += safetyOffset;
                    
                    Debug.Log($"Updated safe respawn position: {lastGroundedPosition} (ground-validated)");
                }
                else
                {
                    // Fallback: use current position with higher offset
                    lastGroundedPosition = potentialRespawnPos + Vector3.up * 2f;
                    Debug.Log($"Updated respawn position (fallback): {lastGroundedPosition}");
                }
            }
            
            if (currentState != KartState.Grounded && !isPerformingArcJump)
            {
                currentState = KartState.Grounded;
                if (airborneTimer > kartApex.kartGameSettings.hopGracePeriod)
                {
                    onLanding.Invoke();
                    
                    if (trickPerformedOnRamp)
                    {
                        kartShortBoosted = true;
                        boostTimer = 0f;
                        trickPerformedOnRamp = false; // Reset trick state
                        isPerformingTrick = false;
                        Debug.Log("Trick landing boost applied!");
                    }
                }
            }
            HandleGroundedMovement();
        }
        else // Kart is not on the ground
        {
            if (currentState != KartState.Jumping && currentState != KartState.PerformingTrick && !isPerformingArcJump)
            {
                currentState = KartState.Airborne;
            }
            HandleAirborneMovement();
        }
    }

    void Update()
    {
        if (kartApex.frozen)
        {
            return;
        }
        
        if (isTransitioningToGrounded)
        {
            HandleLandingTransition();
        }
        else if (currentState == KartState.Grounded)
        {
            transform.position = Vector3.Lerp(transform.position, targetPosition, 25f * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 25f * Time.deltaTime);
        }
        HandleChassisVisuals();
        HandleWheelVisuals();
    }
}