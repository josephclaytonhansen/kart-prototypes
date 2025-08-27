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
        float turnAmount = steerInput * kartApex.kartData.turnSpeed * effectiveSteeringFactor * Time.fixedDeltaTime;
        if (isDrifting || kartApex.autoDrift)
        {
            turnAmount *= kartApex.kartData.driftTurnBoost;
            driftTimer += Time.fixedDeltaTime;
        }
        targetRotation *= Quaternion.Euler(0, turnAmount, 0);
    }

    private void HandleDriving()
    {
        if (isAccelerating)
        {
            currentSpeed = Mathf.Lerp(currentSpeed, kartApex.kartData.maxSpeed, kartApex.kartData.acceleration * Time.fixedDeltaTime);
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
        float slopeFactor = (float)Math.Sin(currentSlope * Mathf.Deg2Rad);
        float weightInfluence = kartApex.kartData.weight;
        float forwardDotNormal = Vector3.Dot(transform.forward, averagedNormal);
        if (Math.Abs(currentSpeed) > 3f)
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
        float checkDistance = (currentSpeed * Time.fixedDeltaTime) + kartApex.kartGameSettings.collisionLookahead;
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
                targetPosition = hitInfo.point + hitInfo.normal * kartApex.kartGameSettings.bounceSafeDistance;
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
            newTargetPos.y = averageHitHeight + kartApex.kartData.groundContactOffset;
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
            Vector3 raycastOrigin = wheelTransforms[i].position - transform.up * kartApex.kartData.groundContactOffset;
            if (Physics.Raycast(raycastOrigin, -transform.up, out groundHits[groundHitCount], kartApex.kartData.groundCheckDistance, kartApex.kartData.groundLayer))
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
        for (int i = 0; i < wheelTransforms.Length; i++)
        {
            Vector3 raycastOrigin = wheelTransforms[i].position;
            if (Physics.Raycast(raycastOrigin, -transform.up, 1f, layer))
            {
                return true;
            }
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
        isJumping = true;
        kartApex.kartRigidbody.AddForce(transform.up * kartApex.kartGameSettings.jumpForce, ForceMode.VelocityChange);
        airborneTimer = 0f;
        // The state is now explicitly set to Jumping
        currentState = KartState.Jumping;
    }
}