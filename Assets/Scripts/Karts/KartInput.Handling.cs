using UnityEngine;
using System;

public partial class KartInput
{
    private void HandleGroundedMovement()
    {
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
    }

    private void HandleAirborneMovement()
    {
        airborneTimer += Time.fixedDeltaTime;
        if (kartApex.kartRigidbody.isKinematic)
        {
            kartApex.kartRigidbody.isKinematic = false;
            kartApex.kartRigidbody.useGravity = true;
            kartApex.kartRigidbody.linearVelocity = transform.forward * currentSpeed;
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
            Debug.Log(driftTimer);
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
                
                lps.startColor = new Color(0f, 0.7f, 1f);
                rps.startColor = new Color(0f, 0.7f, 1f);
            }

            if (driftTimer >= kartApex.kartGameSettings.driftTimeToOrangeBoost && !kartApex.autoDrift){

                lps.startColor = new Color(1f, 0.7f, 0f);
                rps.startColor = new Color(1f, 0.7f, 0f);
            } 
        }
        
        targetRotation *= Quaternion.Euler(0, turnAmount, 0);
    }

    private void HandleDriving()
    {
        if (isAccelerating)
        {
            // Handle boost states with local variables instead of modifying kartData
            if (kartLongBoosted)
            {
                currentMaxSpeed = kartApex.kartData.maxBoostedSpeed;
                currentAcceleration = 100;
                boostTimer += Time.fixedDeltaTime;
                if (boostTimer >= kartApex.kartGameSettings.longBoostDuration)
                {
                    kartLongBoosted = false;
                    boostTimer = 0f;
                    currentMaxSpeed = kartApex.kartData.maxSpeed;
                    currentAcceleration = kartApex.kartData.acceleration;
                    kartApex.BL_particleSystem.SetActive(false);
                    kartApex.BR_particleSystem.SetActive(false);
                }
            }
            else if (kartShortBoosted)
            {
                currentMaxSpeed = kartApex.kartData.maxBoostedSpeed;
                currentAcceleration = 100;
                boostTimer += Time.fixedDeltaTime;
                if (boostTimer >= kartApex.kartGameSettings.shortBoostDuration)
                {
                    kartShortBoosted = false;
                    boostTimer = 0f;
                    currentMaxSpeed = kartApex.kartData.maxSpeed;
                    currentAcceleration = kartApex.kartData.acceleration;
                    kartApex.BR_particleSystem.SetActive(false);
                    kartApex.BL_particleSystem.SetActive(false);
                }
            }
            else
            {
                // Reset to base values when not boosting
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

        // Improved slope handling that prevents ground penetration during boosts
        float slopeFactor = (float)Math.Sin(currentSlope * Mathf.Deg2Rad);
        float weightInfluence = (kartApex.kartData.weight - kartApex.kartGameSettings.minKartWeight) * kartApex.kartGameSettings.slopeInfluence;
        float forwardDotNormal = Vector3.Dot(transform.forward, averagedNormal);
        
        // Only apply slope influence if we're not in a boost state and moving reasonably fast
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
        Vector3 forwardOnGround = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, averagedNormal);
        targetRotation = Quaternion.LookRotation(forwardOnGround, averagedNormal);
        
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
    
    private void CheckForLandingAssist()
    {
        if (airborneTimer < kartApex.kartGameSettings.hopGracePeriod)
        {
            return;
        }

        Vector3 raycastDirection = Vector3.Slerp(Vector3.down, transform.forward, kartApex.kartGameSettings.landingAssistLookahead).normalized;
        Debug.DrawRay(transform.position, raycastDirection * kartApex.kartGameSettings.landingRayLength, Color.red);

        RaycastHit hit;
        Vector3 colliderWorldCenter = kartApex.kartCollider.transform.position;
        Vector3 colliderForward = kartApex.kartCollider.transform.forward;
        Vector3 colliderRight = kartApex.kartCollider.transform.right;
        Vector3 colliderExtents = kartApex.kartCollider.size / 2f;

        Vector3 frontRayOrigin = colliderWorldCenter + colliderForward * colliderExtents.z + Vector3.down * 0.1f;
        Vector3 backRayOrigin = colliderWorldCenter - colliderForward * colliderExtents.z + Vector3.down * 0.1f;

        bool frontHit = Physics.Raycast(frontRayOrigin, Vector3.down, out hit, kartApex.kartGameSettings.landingRayLength, kartApex.kartData.groundLayer);
        bool backHit = Physics.Raycast(backRayOrigin, Vector3.down, out hit, kartApex.kartGameSettings.landingRayLength, kartApex.kartData.groundLayer);

        if (frontHit || backHit)
        {
            landingAssistHit = hit;
            if (!isTransitioningToGrounded)
            {
                StartLandingTransition();
            }
        }
    }

    private void StartLandingTransition()
    {
        if (isTransitioningToGrounded) return;
        isTransitioningToGrounded = true;
        kartApex.kartRigidbody.isKinematic = true;
        kartApex.kartRigidbody.useGravity = false;
        kartApex.kartRigidbody.Sleep();
        transitionStartPosition = transform.position;
        transitionStartRotation = transform.rotation;
        float finalOffset = kartApex.kartData.groundContactOffset + kartApex.kartGameSettings.landingHeightOffset;
        
        // Add extra offset during boosts to prevent ground penetration
        if (kartLongBoosted || kartShortBoosted)
        {
            finalOffset += 0.1f;
        }
        
        targetPosition = landingAssistHit.point + landingAssistHit.normal * finalOffset;
        targetRotation = Quaternion.FromToRotation(transform.up, landingAssistHit.normal) * transform.rotation;
        currentSpeed = kartApex.kartRigidbody.linearVelocity.magnitude;
        transitionTimer = 0;
    }

    private void HandleLandingTransition()
    {
        transitionTimer += Time.fixedDeltaTime;
        float t = Mathf.Clamp01(transitionTimer / kartApex.kartGameSettings.transitionDuration);
        transform.position = Vector3.Lerp(transitionStartPosition, targetPosition, t);
        transform.rotation = Quaternion.Slerp(transitionStartRotation, targetRotation, t);
        if (t >= 1.0f)
        {
            isTransitioningToGrounded = false;
            // Now that we have landed, change the state to grounded
            currentState = KartState.Grounded; 
            if (airborneTimer > 0.05f)
            {
                onLanding.Invoke();
            }
        }
    }

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

    private void HandleChassisVisuals()
    {
        if (kartApex.chassisVisuals != null)
        {
            Quaternion targetTilt = Quaternion.Euler(0, steerInput * kartApex.kartData.maxChassisTiltAngle, 0);
            kartApex.chassisVisuals.localRotation = Quaternion.Slerp(kartApex.chassisVisuals.localRotation, targetTilt, kartApex.kartData.tiltDampening * Time.fixedDeltaTime);
        }
        if (kartApex.kartVisualsRoot != null && currentState == KartState.Grounded)
        {
            float averageHitHeight = 0f;
            if (groundHitCount > 0)
            {
                for (int i = 0; i < groundHitCount; i++)
                {
                    averageHitHeight += groundHits[i].point.y;
                }
                averageHitHeight /= groundHitCount;
            }
            Vector3 newLocalPos = new Vector3(0, (averageHitHeight - transform.position.y) + kartApex.kartData.groundContactOffset, 0);
            if (Vector3.Distance(kartApex.kartVisualsRoot.localPosition, newLocalPos) > kartApex.kartData.yJitterThreshold)
            {
                kartApex.kartVisualsRoot.localPosition = Vector3.Lerp(kartApex.kartVisualsRoot.localPosition, newLocalPos, kartApex.kartData.tiltDampening * Time.fixedDeltaTime);
            }
        }
    }

    private void HandleWheelVisuals()
    {
        currentSteerAngle = steerInput * kartApex.kartData.maxSteerAngle;
        kartApex.leftFrontWheel.localRotation = Quaternion.Euler(0, currentSteerAngle, 0);
        kartApex.rightFrontWheel.localRotation = Quaternion.Euler(0, currentSteerAngle, 0);
        float wheelSpeed = currentState == KartState.Grounded ? currentSpeed : kartApex.kartRigidbody.linearVelocity.magnitude;
        float wheelRotationSpeed = wheelSpeed * 360f / (2f * Mathf.PI * kartApex.kartData.wheelRadius);
        kartApex.leftFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        kartApex.rightFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        kartApex.leftBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        kartApex.rightBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
    }

    private void HandleRampJump()
    {
        if (stateChangeTimer < STATE_CHANGE_COOLDOWN) return; // Prevent rapid state changes
        
        isJumping = true;
        kartApex.kartRigidbody.AddForce(transform.up * kartApex.kartGameSettings.jumpForce, ForceMode.VelocityChange);
        airborneTimer = 0f;
        // The state is now explicitly set to Jumping
        currentState = KartState.Jumping;
        stateChangeTimer = 0f;
    }

    public void StartShortBoost(){
        kartShortBoosted = true;
        boostTimer = 0f;
    }

    public void StartLongBoost(){
        kartLongBoosted = true;
        boostTimer = 0f;
    }
}