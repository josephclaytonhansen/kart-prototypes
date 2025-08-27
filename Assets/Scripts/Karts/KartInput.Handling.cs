using UnityEngine;
using System;

public partial class KartInput
{
    private void HandleGroundedMovement()
    {
        if (!wasGrounded)
        {
            if (airborneTimer > kartGameSettings.hopGracePeriod)
            {
                onLanding.Invoke();
            }
            kartRigidbody.isKinematic = true;
            kartRigidbody.useGravity = false;
            kartRigidbody.Sleep();
            isJumping = false;
            isPerformingTrick = false;
        }
        airborneTimer = 0f;
        currentSlope = Vector3.Angle(averagedNormal, Vector3.up);
        HandleSteering();
        HandleDriving();
        HandleAlignment();
    }

    private void HandleAirborneMovement()
    {
        airborneTimer += Time.fixedDeltaTime;
        if (kartRigidbody.isKinematic)
        {
            kartRigidbody.isKinematic = false;
            kartRigidbody.useGravity = true;
            kartRigidbody.linearVelocity = transform.forward * currentSpeed;
        }
    }

    private void HandleSteering()
    {
        float effectiveSteeringFactor;
        float absoluteSpeed = Mathf.Abs(currentSpeed);
        if (absoluteSpeed <= kartData.maxSpeed / 2f)
        {
            effectiveSteeringFactor = Mathf.Lerp(0, 1, absoluteSpeed / (kartData.maxSpeed / 2f));
        }
        else
        {
            effectiveSteeringFactor = Mathf.Lerp(1, kartData.steeringDampenAtMaxSpeed, (absoluteSpeed - kartData.maxSpeed / 2f) / (kartData.maxSpeed - kartData.maxSpeed / 2f));
        }
        float turnAmount = steerInput * kartData.turnSpeed * effectiveSteeringFactor * Time.fixedDeltaTime;
        if (isDrifting || autoDrift)
        {
            turnAmount *= kartData.driftTurnBoost;
            driftTimer += Time.fixedDeltaTime;
        }
        targetRotation *= Quaternion.Euler(0, turnAmount, 0);
    }

    private void HandleDriving()
    {
        if (isAccelerating)
        {
            currentSpeed = Mathf.Lerp(currentSpeed, kartData.maxSpeed, kartData.acceleration * Time.fixedDeltaTime);
        }
        else if (brakeAction.IsPressed())
        {
            if (currentSpeed > 0.1f)
            {
                currentSpeed = Mathf.Max(currentSpeed - kartData.brakeForce * Time.fixedDeltaTime, 0f);
            }
            else
            {
                currentSpeed = Mathf.Lerp(currentSpeed, -kartData.maxReverseSpeed, kartData.reverseAcceleration * Time.fixedDeltaTime);
            }
        }
        else
        {
            currentSpeed = Mathf.Lerp(currentSpeed, 0, kartData.deceleration * Time.fixedDeltaTime);
        }
        float slopeFactor = Mathf.Sin(currentSlope * Mathf.Deg2Rad);
        float weightInfluence = kartData.weight;
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
        if (isDrifting || autoDrift)
        {
            Vector3 outwardsAgainstTurn = Vector3.ProjectOnPlane(targetRotation * Vector3.right * -steerInput, averagedNormal).normalized;
            movementDirection = (forwardOnGround.normalized + outwardsAgainstTurn * kartData.driftSlideAmount).normalized;
        }
        else
        {
            movementDirection = forwardOnGround.normalized;
        }
        float checkDistance = (currentSpeed * Time.fixedDeltaTime) + kartGameSettings.collisionLookahead;
        Vector3 checkDirection = movementDirection;
        RaycastHit hitInfo = new RaycastHit();
        bool hitFound = false;
        Vector3 colliderWorldCenter = kartCollider.transform.position;
        Vector3 colliderForward = kartCollider.transform.forward;
        Vector3 colliderRight = kartCollider.transform.right;
        Vector3 colliderExtents = kartCollider.size / 2f;
        Vector3 centerRayOrigin = colliderWorldCenter + colliderForward * colliderExtents.z;
        Vector3 leftRayOrigin = centerRayOrigin - colliderRight * colliderExtents.x;
        Vector3 rightRayOrigin = centerRayOrigin + colliderRight * colliderExtents.x;
        float originOffset = 0.05f;
        centerRayOrigin += checkDirection * originOffset;
        leftRayOrigin += checkDirection * originOffset;
        rightRayOrigin += checkDirection * originOffset;
        if (Physics.Raycast(centerRayOrigin, checkDirection, out hitInfo, checkDistance, kartData.groundLayer))
        {
            hitFound = true;
            Debug.DrawRay(centerRayOrigin, checkDirection * checkDistance, Color.cyan);
        }
        else if (Physics.Raycast(leftRayOrigin, checkDirection, out hitInfo, checkDistance, kartData.groundLayer))
        {
            hitFound = true;
            Debug.DrawRay(leftRayOrigin, checkDirection * checkDistance, Color.cyan);
        }
        else if (Physics.Raycast(rightRayOrigin, checkDirection, out hitInfo, checkDistance, kartData.groundLayer))
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
            if (wallAngle > kartGameSettings.wallAngleThreshold)
            {
                currentSpeed = -kartGameSettings.headOnBounceSpeed;
                targetPosition = hitInfo.point + hitInfo.normal * kartGameSettings.bounceSafeDistance;
                Debug.DrawLine(transform.position, hitInfo.point, Color.red);
            }
            else
            {
                Vector3 wallTangent = Vector3.Cross(hitInfo.normal, Vector3.up).normalized;
                if (Vector3.Dot(wallTangent, movementDirection) < 0)
                {
                    wallTangent *= -1;
                }
                movementDirection = Vector3.Slerp(wallTangent, hitInfo.normal, kartGameSettings.glancingBounceFactor).normalized;
                currentSpeed *= (1f - kartGameSettings.glancingSpeedLoss);
                targetPosition = hitInfo.point + hitInfo.normal * kartGameSettings.bounceSafeDistance;
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
            newTargetPos.y = averageHitHeight + kartData.groundContactOffset;
            targetPosition = newTargetPos;
        }
    }

    private void CheckForLandingAssist()
    {
        Vector3 raycastDirection = Vector3.Slerp(Vector3.down, transform.forward, kartGameSettings.landingAssistLookahead).normalized;
        Debug.DrawRay(transform.position, raycastDirection * kartGameSettings.landingRayLength, Color.red);
        if (Physics.Raycast(transform.position, raycastDirection, out landingAssistHit, kartGameSettings.landingRayLength, kartData.groundLayer) && airborneTimer > kartGameSettings.hopGracePeriod)
        {
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
        kartRigidbody.isKinematic = true;
        kartRigidbody.useGravity = false;
        kartRigidbody.Sleep();
        transitionStartPosition = transform.position;
        transitionStartRotation = transform.rotation;
        float finalOffset = kartData.groundContactOffset + kartGameSettings.landingHeightOffset;
        targetPosition = landingAssistHit.point + landingAssistHit.normal * finalOffset;
        targetRotation = Quaternion.FromToRotation(transform.up, landingAssistHit.normal) * transform.rotation;
        currentSpeed = kartRigidbody.linearVelocity.magnitude;
        transitionTimer = 0;
    }

    private void HandleLandingTransition()
    {
        transitionTimer += Time.fixedDeltaTime;
        float t = Mathf.Clamp01(transitionTimer / kartGameSettings.transitionDuration);
        transform.position = Vector3.Lerp(transitionStartPosition, targetPosition, t);
        transform.rotation = Quaternion.Slerp(transitionStartRotation, targetRotation, t);
        if (t >= 1.0f)
        {
            isTransitioningToGrounded = false;
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
            Vector3 raycastOrigin = wheelTransforms[i].position - transform.up * kartData.groundContactOffset;
            if (Physics.Raycast(raycastOrigin, -transform.up, out groundHits[groundHitCount], kartData.groundCheckDistance, kartData.groundLayer))
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

    private void HandleChassisVisuals()
    {
        if (chassisVisuals != null)
        {
            Quaternion targetTilt = Quaternion.Euler(0, steerInput * kartData.maxChassisTiltAngle, 0);
            chassisVisuals.localRotation = Quaternion.Slerp(chassisVisuals.localRotation, targetTilt, kartData.tiltDampening * Time.fixedDeltaTime);
        }
        if (kartVisualsRoot != null && currentState == KartState.Grounded)
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
            Vector3 newLocalPos = new Vector3(0, (averageHitHeight - transform.position.y) + kartData.groundContactOffset, 0);
            if (Vector3.Distance(kartVisualsRoot.localPosition, newLocalPos) > kartData.yJitterThreshold)
            {
                kartVisualsRoot.localPosition = Vector3.Lerp(kartVisualsRoot.localPosition, newLocalPos, kartData.tiltDampening * Time.fixedDeltaTime);
            }
        }
    }

    private void HandleWheelVisuals()
    {
        currentSteerAngle = steerInput * kartData.maxSteerAngle;
        leftFrontWheel.localRotation = Quaternion.Euler(0, currentSteerAngle, 0);
        rightFrontWheel.localRotation = Quaternion.Euler(0, currentSteerAngle, 0);
        float wheelSpeed = currentState == KartState.Grounded ? currentSpeed : kartRigidbody.linearVelocity.magnitude;
        float wheelRotationSpeed = wheelSpeed * 360f / (2f * Mathf.PI * kartData.wheelRadius);
        leftFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        rightFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        leftBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        rightBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
    }

    private void HandleRampJump()
    {
        isJumping = true;
        kartRigidbody.AddForce(transform.up * kartGameSettings.jumpForce, ForceMode.VelocityChange);
        airborneTimer = 0f;
    }
}