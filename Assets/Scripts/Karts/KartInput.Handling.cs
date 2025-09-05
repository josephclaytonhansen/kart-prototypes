using UnityEngine;
using System;

public partial class KartInput
{
    private void HandleGroundedMovement()
    {
        if (isPerformingArcJump)
        {
            HandleArcJump();
            return;
        }
        
        if (currentState != KartState.Grounded)
        {
            // Only perform these actions on the frame the kart becomes grounded
            kartApex.kartRigidbody.isKinematic = true;
            kartApex.kartRigidbody.useGravity = false;
            kartApex.kartRigidbody.Sleep();
            isJumping = false;
            isPerformingTrick = false;
            airborneTimer = 0f;
        }

        currentSlope = Vector3.Angle(averagedNormal, Vector3.up);
        HandleSteering();
        HandleDriving();
        HandleAlignment();
        
        // IMPROVED: Prevent extreme tilting even when grounded (Mario Kart style stability)
        CheckGroundedStability();
    }

    private void HandleAirborneMovement()
    {
        airborneTimer += Time.fixedDeltaTime;
        
        // MARIO KART WII STYLE: Arc jumps are completely self-contained
        if (isPerformingArcJump)
        {
            HandleArcJump();
            return;
        }
        
        // MARIO KART WII STYLE: Natural airborne (edges/bumps) uses gravity
        if (kartApex.kartRigidbody.isKinematic)
        {
            // Switch to physics for natural falling (driving off edges, bumps)
            Vector3 correctedForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            if (correctedForward.magnitude > 0.1f)
            {
                transform.rotation = Quaternion.LookRotation(correctedForward, Vector3.up);
            }
            
            kartApex.kartRigidbody.isKinematic = false;
            kartApex.kartRigidbody.useGravity = true;
            kartApex.kartRigidbody.linearVelocity = transform.forward * currentSpeed;
            kartApex.kartRigidbody.angularVelocity = Vector3.zero;
            
            Debug.Log("Natural airborne - switched to gravity physics");
        }
        
        // MARIO KART WII STYLE: Gradual upright correction during natural fall
        HandleAirborneOrientation();
        
        if (Mathf.Abs(steerInput) > kartApex.kartGameSettings.inputDeadzone)
        {
            float airborneControlStrength = kartApex.kartGameSettings.airborneControlStrength;
            float airborneSteer = steerInput * airborneControlStrength * kartApex.kartData.turnSpeed * Time.fixedDeltaTime;
            transform.Rotate(0, airborneSteer, 0);
        }
        
        CheckForLandingAssist();
    }

    private void HandleSteering()
    {
        float effectiveSteeringFactor;
        float absoluteSpeed = Math.Abs(currentSpeed);
        if (absoluteSpeed <= kartApex.kartData.maxSpeed / 2f)
        {
            effectiveSteeringFactor = Mathf.Lerp(0, 1, absoluteSpeed / (kartApex.kartData.maxSpeed / 2f));
        }
        else
        {
            effectiveSteeringFactor = Mathf.Lerp(1, kartApex.kartData.steeringDampenAtMaxSpeed, (absoluteSpeed - kartApex.kartData.maxSpeed / 2f) / (kartApex.kartData.maxSpeed - kartApex.kartData.maxSpeed / 2f));
        }
        
        // Calculate base turn amount
        float turnAmount = steerInput * kartApex.kartData.turnSpeed * effectiveSteeringFactor * Time.fixedDeltaTime;
        
        // Apply terrain steering modification with smoothing
        float terrainSteerMultiplier = GetTerrainSteerMultiplier();
        turnAmount = turnAmount * terrainSteerMultiplier;
        
        if (isDrifting || kartApex.autoDrift)
        {
            // Apply terrain drift modification to the drift boost with smoothing
            float terrainDriftMultiplier = GetTerrainDriftMultiplier();
            float driftBoost = kartApex.kartData.driftTurnBoost * terrainDriftMultiplier;
            turnAmount *= driftBoost;
            driftTimer += Time.fixedDeltaTime;

            ParticleSystem lps = kartApex.BL_particleSystem.GetComponent<ParticleSystem>();
            ParticleSystem rps = kartApex.BR_particleSystem.GetComponent<ParticleSystem>();

            if (driftTimer >= kartApex.kartGameSettings.driftTimeToBlueBoost && !kartApex.autoDrift)
            {
                kartApex.BL_particleSystem.SetActive(true);
                kartApex.BR_particleSystem.SetActive(true);
                
                var lpsMain = lps.main;
                var rpsMain = rps.main;
                lpsMain.startColor = kartApex.kartGameSettings.blueBoostColor;
                rpsMain.startColor = kartApex.kartGameSettings.blueBoostColor;
            }

            if (driftTimer >= kartApex.kartGameSettings.driftTimeToOrangeBoost && !kartApex.autoDrift){
                var lpsMain = lps.main;
                var rpsMain = rps.main;
                lpsMain.startColor = kartApex.kartGameSettings.orangeBoostColor;
                rpsMain.startColor = kartApex.kartGameSettings.orangeBoostColor;
            } 
        }
        
        targetRotation *= Quaternion.Euler(0, turnAmount, 0);
    }

    private void HandleDriving()
    {
        if (isAccelerating)
        {
            if (kartLongBoosted)
            {
                currentMaxSpeed = kartApex.kartData.maxBoostedSpeed;
                currentAcceleration = kartApex.kartGameSettings.boostAcceleration;
                boostTimer += Time.fixedDeltaTime;
                if (boostTimer >= kartApex.kartGameSettings.longBoostDuration)
                {
                    kartLongBoosted = false;
                    boostTimer = 0f;
                    currentMaxSpeed = kartApex.kartData.maxSpeed;
                    currentAcceleration = kartApex.kartData.acceleration;
                    kartApex.BL_particleSystem.SetActive(false);
                    kartApex.BR_particleSystem.SetActive(false);
                    Debug.Log("LONG BOOST ENDED!");
                }
            }
            else if (kartShortBoosted)
            {
                currentMaxSpeed = kartApex.kartData.maxBoostedSpeed;
                currentAcceleration = kartApex.kartGameSettings.boostAcceleration;
                boostTimer += Time.fixedDeltaTime;
                if (boostTimer >= kartApex.kartGameSettings.shortBoostDuration)
                {
                    kartShortBoosted = false;
                    boostTimer = 0f;
                    currentMaxSpeed = kartApex.kartData.maxSpeed;
                    currentAcceleration = kartApex.kartData.acceleration;
                    kartApex.BR_particleSystem.SetActive(false);
                    kartApex.BL_particleSystem.SetActive(false);
                    Debug.Log("SHORT BOOST ENDED!");
                }
            }
            else
            {
                currentMaxSpeed = kartApex.kartData.maxSpeed;
                currentAcceleration = kartApex.kartData.acceleration;
            }
            
            currentSpeed = Mathf.Lerp(currentSpeed, currentMaxSpeed, currentAcceleration * Time.fixedDeltaTime);
        }
        else if (brakeAction.IsPressed())
        {
            if (currentSpeed > 0.1f)
            {
                currentSpeed = Mathf.Max(currentSpeed - kartApex.kartData.brakeForce * Time.fixedDeltaTime, 0f);
            }
            else
            {
                currentSpeed = Mathf.Lerp(currentSpeed, -kartApex.kartData.maxReverseSpeed, kartApex.kartData.reverseAcceleration * Time.fixedDeltaTime);
            }
        }
        else
        {
            currentSpeed = Mathf.Lerp(currentSpeed, 0, kartApex.kartData.deceleration * Time.fixedDeltaTime);
        }
        
        // Apply terrain speed modification with smoothing
        float terrainSpeedMultiplier = GetTerrainSpeedMultiplier();
        currentSpeed = currentSpeed * terrainSpeedMultiplier;

        HandleTailwind();

        float slopeFactor = (float)Math.Sin(currentSlope * Mathf.Deg2Rad);
        float weightInfluence = (kartApex.kartData.weight - kartApex.kartGameSettings.minKartWeight) * kartApex.kartGameSettings.slopeInfluence;
        float forwardDotNormal = Vector3.Dot(transform.forward, averagedNormal);
        
        if (Math.Abs(currentSpeed) > 3f && !kartLongBoosted && !kartShortBoosted)
        {
            if (forwardDotNormal < 0)
            {
                currentSpeed -= (slopeFactor * weightInfluence) * Time.fixedDeltaTime;
            }
            else
            {
                currentSpeed += (slopeFactor * weightInfluence) * Time.fixedDeltaTime;
            }
        }
        
        Vector3 movementDirection = transform.forward;
        Vector3 forwardOnGround = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, averagedNormal);
        if (isDrifting || kartApex.autoDrift)
        {
            Vector3 outwardsAgainstTurn = Vector3.ProjectOnPlane(targetRotation * Vector3.right * -steerInput, averagedNormal).normalized;
            movementDirection = (forwardOnGround.normalized + outwardsAgainstTurn * kartApex.kartData.driftSlideAmount).normalized;
        }
        else
        {
            movementDirection = forwardOnGround.normalized;
        }
        
        float checkDistance = (Math.Abs(currentSpeed) * Time.fixedDeltaTime) + kartApex.kartGameSettings.collisionLookahead;
        Vector3 checkDirection = movementDirection;
        RaycastHit hitInfo = new RaycastHit();
        bool hitFound = false;
        Vector3 colliderWorldCenter = kartApex.kartCollider.transform.position;
        Vector3 colliderForward = kartApex.kartCollider.transform.forward;
        Vector3 colliderRight = kartApex.kartCollider.transform.right;
        Vector3 colliderExtents = kartApex.kartCollider.size / 2f;
        Vector3 centerRayOrigin = colliderWorldCenter + colliderForward * colliderExtents.z;
        Vector3 leftRayOrigin = centerRayOrigin - colliderRight * colliderExtents.x;
        Vector3 rightRayOrigin = centerRayOrigin + colliderRight * colliderExtents.x;
        float originOffset = 0.05f;
        centerRayOrigin += checkDirection * originOffset;
        leftRayOrigin += checkDirection * originOffset;
        rightRayOrigin += checkDirection * originOffset;
        
        if (Physics.Raycast(centerRayOrigin, checkDirection, out hitInfo, checkDistance, kartApex.kartData.groundLayer))
        {
            hitFound = true;
            Debug.DrawRay(centerRayOrigin, checkDirection * checkDistance, Color.cyan);
        }
        else if (Physics.Raycast(leftRayOrigin, checkDirection, out hitInfo, checkDistance, kartApex.kartData.groundLayer))
        {
            hitFound = true;
            Debug.DrawRay(leftRayOrigin, checkDirection * checkDistance, Color.cyan);
        }
        else if (Physics.Raycast(rightRayOrigin, checkDirection, out hitInfo, checkDistance, kartApex.kartData.groundLayer))
        {
            hitFound = true;
            Debug.DrawRay(rightRayOrigin, checkDirection * checkDistance, Color.cyan);
        }
        else
        {
            Debug.DrawRay(centerRayOrigin, checkDirection * checkDistance, Color.green);
            Debug.DrawRay(leftRayOrigin, checkDirection * checkDistance, Color.green);
            Debug.DrawRay(rightRayOrigin, checkDirection * checkDistance, Color.green);
        }
        
        if (hitFound)
        {
            onWallCollision.Invoke();
            float wallAngle = Vector3.Angle(hitInfo.normal, Vector3.up);
            if (wallAngle > kartApex.kartGameSettings.wallAngleThreshold)
            {
                currentSpeed = -kartApex.kartGameSettings.headOnBounceSpeed;
                targetPosition = hitInfo.point + hitInfo.normal * kartApex.kartGameSettings.bounceSafeDistance * ((kartApex.kartGameSettings.minKartWeight / kartApex.kartData.weight) * 10f);
                Debug.DrawLine(transform.position, hitInfo.point, Color.red);
            }
            else
            {
                Vector3 wallTangent = Vector3.Cross(hitInfo.normal, Vector3.up).normalized;
                if (Vector3.Dot(wallTangent, movementDirection) < 0)
                {
                    wallTangent *= -1;
                }
                movementDirection = Vector3.Slerp(wallTangent, hitInfo.normal, kartApex.kartGameSettings.glancingBounceFactor).normalized;
                currentSpeed *= (1f - kartApex.kartGameSettings.glancingSpeedLoss);
                targetPosition = hitInfo.point + hitInfo.normal * kartApex.kartGameSettings.bounceSafeDistance;
                Debug.DrawLine(transform.position, hitInfo.point, Color.yellow);
            }
        }
        
        targetPosition += movementDirection * currentSpeed * Time.fixedDeltaTime;
    }

    private void HandleAlignment()
    {
        // IMPROVED: Mario Kart style orientation - always try to stay upright
        Vector3 desiredUp = averagedNormal;
        Vector3 currentUp = transform.up;
        
        // Prevent extreme tilting during drift (arcade-style stability)
        if (isDrifting)
        {
            // Allow some banking during drift but prevent flipping
            float maxDriftTilt = 25f; // degrees
            Vector3 projectedUp = Vector3.ProjectOnPlane(currentUp, transform.forward).normalized;
            float currentTilt = Vector3.Angle(projectedUp, Vector3.up);
            
            if (currentTilt > maxDriftTilt)
            {
                // Correct excessive tilt back toward upright
                Vector3 correctedUp = Vector3.Slerp(currentUp, Vector3.up, Time.fixedDeltaTime * 3f);
                desiredUp = Vector3.Slerp(desiredUp, correctedUp, 0.7f);
            }
        }
        
        // FIXED: Calculate forward direction from targetRotation (which includes steering)
        Vector3 forwardOnGround = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, desiredUp).normalized;
        if (forwardOnGround.magnitude < 0.1f)
        {
            // Fallback if forward is too close to up vector
            forwardOnGround = Vector3.ProjectOnPlane(transform.forward, desiredUp).normalized;
        }
        
        // FIXED: Preserve steering by using targetRotation's forward direction
        targetRotation = Quaternion.LookRotation(forwardOnGround, desiredUp);
        
        if (groundHitCount > 0)
        {
            float averageHitHeight = 0f;
            for (int i = 0; i < groundHitCount; i++)
            {
                averageHitHeight += groundHits[i].point.y;
            }
            averageHitHeight /= groundHitCount;
            Vector3 newTargetPos = targetPosition;
            
            // Improved height calculation that accounts for boosts and slopes
            float heightOffset = kartApex.kartData.groundContactOffset;
            
            // Add extra height offset during boosts to prevent ground clipping
            if (kartLongBoosted || kartShortBoosted)
            {
                heightOffset += 0.1f; // Small extra offset during boosts
            }
            
            // Adjust height based on slope to prevent penetration
            if (currentSlope > 15f) // On steep slopes
            {
                heightOffset += Mathf.Lerp(0f, 0.2f, (currentSlope - 15f) / 45f);
            }
            
            newTargetPos.y = averageHitHeight + heightOffset;
            targetPosition = newTargetPos;
        }
    }
    
    // MARIO KART WII STYLE: Gradual orientation correction during airborne (works with gravity)
    private void HandleAirborneOrientation()
    {
        Vector3 currentUp = transform.up;
        if (Vector3.Dot(currentUp, Vector3.up) < 0.8f) // If significantly tilted
        {
            Vector3 correctedForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            if (correctedForward.magnitude > 0.1f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(correctedForward, Vector3.up);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.fixedDeltaTime * 2f);
            }
        }
    }
    
    // IMPROVED: Prevent kart from flipping over when grounded (arcade stability)
    private void CheckGroundedStability()
    {
        Vector3 currentUp = transform.up;
        float upwardnessDot = Vector3.Dot(currentUp, Vector3.up);
        
        // If kart is getting close to flipping over (upside down)
        if (upwardnessDot < 0.3f) // More than ~72 degrees from upright
        {
            Debug.Log("Kart stability critical - applying emergency upright correction!");
            
            // Force the kart back to an upright orientation
            Vector3 correctedForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            if (correctedForward.magnitude < 0.1f)
            {
                correctedForward = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, Vector3.up).normalized;
            }
            
            // Gradually correct to upright position
            Quaternion emergencyUpright = Quaternion.LookRotation(correctedForward, Vector3.up);
            targetRotation = Quaternion.Slerp(targetRotation, emergencyUpright, Time.fixedDeltaTime * 5f);
            
            // Also boost the position slightly to clear any ground penetration
            targetPosition += Vector3.up * 0.2f;
        }
    }
}