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
        stateChangeTimer += Time.fixedDeltaTime;
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
            if (!wasGrounded && stateChangeTimer >= STATE_CHANGE_COOLDOWN && !CheckGroundOnLayer(kartApex.deathLayer))
            {
                // Start with the current position.
                lastGroundedPosition = transform.position;

                Vector3 backwardsDirection = -transform.forward;
                // Subtract a small offset to ensure we don't collide with the ground immediately.
                lastGroundedPosition = lastGroundedPosition - (backwardsDirection * Mathf.Max(currentSpeed, 1f));
                // add a small y offset so it's slightly off the ground
                lastGroundedPosition.y += 1f;
                
                Debug.Log("Updated safe respawn position: " + lastGroundedPosition);
                stateChangeTimer = 0f;
            }
            
            if (currentState != KartState.Grounded && stateChangeTimer >= STATE_CHANGE_COOLDOWN && !isPerformingArcJump)
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
                stateChangeTimer = 0f;
            }
            HandleGroundedMovement();
        }
        else // Kart is not on the ground
        {
            if (currentState != KartState.Jumping && currentState != KartState.PerformingTrick && 
                stateChangeTimer >= STATE_CHANGE_COOLDOWN && !isPerformingArcJump)
            {
                currentState = KartState.Airborne;
                stateChangeTimer = 0f;
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