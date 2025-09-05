using UnityEngine;
using System;

public partial class KartInput
{
    private void HandleRampJump()
    {
        // IMPROVED: Mario Kart-style immediate responsiveness - no artificial delays
        
        // IMPROVED: Mario Kart Wii style graceful arc jump (NOT physics-based)
        isJumping = true;
        isPerformingArcJump = true;
        arcJumpTimer = 0f;
        
        // Store start position and rotation
        arcJumpStartPosition = transform.position;
        arcJumpStartRotation = transform.rotation;
        
        // Calculate target landing position
        Vector3 jumpDirection = transform.forward;
        arcJumpTargetPosition = arcJumpStartPosition + jumpDirection * kartApex.kartGameSettings.jumpArcDistance;
        
        // Calculate peak position (midpoint of arc)
        Vector3 midPoint = Vector3.Lerp(arcJumpStartPosition, arcJumpTargetPosition, 0.5f);
        arcJumpPeakPosition = midPoint + Vector3.up * kartApex.kartGameSettings.jumpArcHeight;
        
        // Calculate target rotation (maintain forward direction)
        arcJumpTargetRotation = Quaternion.LookRotation(jumpDirection, Vector3.up);
        
        // Set rigidbody to kinematic for controlled movement
        kartApex.kartRigidbody.isKinematic = true;
        kartApex.kartRigidbody.useGravity = false;
        
        // Reset timers and set state
        airborneTimer = 0f;
        // Set state based on whether trick was performed
        currentState = isPerformingTrick ? KartState.PerformingTrick : KartState.Jumping;
        
        Debug.Log("Mario Kart Wii style arc jump started - graceful and smooth!");
    }

    // IMPROVED: Handle the graceful arc jump movement like Mario Kart Wii
    private void HandleArcJump()
    {
        arcJumpTimer += Time.fixedDeltaTime;
        float normalizedTime = arcJumpTimer / kartApex.kartGameSettings.jumpArcDuration;
        
        if (normalizedTime >= 1f)
        {
            // IMPROVED: Check for ground/slope at landing position
            Vector3 finalLandingPos = arcJumpTargetPosition;
            if (Physics.Raycast(arcJumpTargetPosition + Vector3.up * 2f, Vector3.down, out RaycastHit landingHit, 5f, kartApex.kartData.groundLayer))
            {
                // Adapt landing to actual ground surface (including slopes)
                finalLandingPos = landingHit.point + landingHit.normal * kartApex.kartData.groundContactOffset;
                arcJumpTargetRotation = Quaternion.LookRotation(transform.forward, landingHit.normal);
            }
            
            // MARIO KART WII STYLE: Arc jump is COMPLETELY controlled - no gravity handoff
            // Land directly to grounded state with smooth positioning
            isPerformingArcJump = false;
            transform.position = finalLandingPos;
            transform.rotation = arcJumpTargetRotation;
            
            // Stay kinematic and transition directly to grounded
            currentState = KartState.Grounded;
            Debug.Log("Arc jump completed - landed directly to grounded state");
            return;
        }
        
        // Apply the jump arc curve for smooth motion
        float curveValue = kartApex.kartGameSettings.jumpArcCurve.Evaluate(normalizedTime);
        
        // Calculate position along the arc using quadratic bezier curve
        Vector3 currentPos = CalculateQuadraticBezierPoint(normalizedTime, arcJumpStartPosition, arcJumpPeakPosition, arcJumpTargetPosition);
        
        // IMPROVED: Check for wall collisions during arc jump
        Vector3 movementDirection = (currentPos - transform.position).normalized;
        float movementDistance = Vector3.Distance(currentPos, transform.position);
        
        if (Physics.Raycast(transform.position, movementDirection, out RaycastHit wallHit, movementDistance + 0.5f, kartApex.kartData.groundLayer))
        {
            float wallAngle = Vector3.Angle(wallHit.normal, Vector3.up);
            if (wallAngle > kartApex.kartGameSettings.wallAngleThreshold)
            {
                // Hit a wall during arc jump - end arc early and bounce
                isPerformingArcJump = false;
                transform.position = wallHit.point + wallHit.normal * kartApex.kartGameSettings.bounceSafeDistance;
                
                // Calculate bounce direction
                Vector3 bounceDirection = Vector3.Reflect(movementDirection, wallHit.normal);
                arcJumpTargetRotation = Quaternion.LookRotation(bounceDirection, Vector3.up);
                transform.rotation = arcJumpTargetRotation;
                
                // Restore physics and apply bounce
                kartApex.kartRigidbody.isKinematic = false;
                kartApex.kartRigidbody.useGravity = true;
                kartApex.kartRigidbody.linearVelocity = bounceDirection * currentSpeed * 0.5f; // Reduced speed from collision
                
                currentState = KartState.Airborne;
                onWallCollision.Invoke();
                Debug.Log("Wall collision during arc jump - bounced!");
                return;
            }
        }
        
        transform.position = currentPos;
        
        // Smoothly rotate during the jump
        transform.rotation = Quaternion.Slerp(arcJumpStartRotation, arcJumpTargetRotation, curveValue);
        
        // IMPROVED: Allow airborne steering during arc jump
        if (Mathf.Abs(steerInput) > kartApex.kartGameSettings.inputDeadzone)
        {
            float airborneControlStrength = kartApex.kartGameSettings.airborneControlStrength * 0.5f; // Reduced during arc
            float airborneSteer = steerInput * airborneControlStrength * kartApex.kartData.turnSpeed * Time.fixedDeltaTime;
            transform.Rotate(0, airborneSteer, 0);
            
            // Update target rotation to maintain new direction
            arcJumpTargetRotation *= Quaternion.Euler(0, airborneSteer, 0);
        }
    }
    
    // Helper method for smooth arc calculation
    private Vector3 CalculateQuadraticBezierPoint(float t, Vector3 p0, Vector3 p1, Vector3 p2)
    {
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        
        Vector3 point = uu * p0; // (1-t)^2 * P0
        point += 2 * u * t * p1; // 2(1-t)t * P1
        point += tt * p2; // t^2 * P2
        
        return point;
    }
}