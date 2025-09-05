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
    }

    private void HandleAirborneMovement()
    {
        airborneTimer += Time.fixedDeltaTime;
        
        if (isPerformingArcJump)
        {
            HandleArcJump();
            return;
        }
        
        if (kartApex.kartRigidbody.isKinematic)
        {
            kartApex.kartRigidbody.isKinematic = false;
            kartApex.kartRigidbody.useGravity = true;
            kartApex.kartRigidbody.linearVelocity = transform.forward * currentSpeed;
        }
        
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

    public void StartShortBoost(){
        kartShortBoosted = true;
        boostTimer = 0f;
    }

    public void StartLongBoost(){
        kartLongBoosted = true;
        boostTimer = 0f;
    }
}