using UnityEngine;
using System;

public partial class KartInput
{
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
        
        // IMPROVED: Better landing orientation - blend ground normal with upright preference
        Vector3 landingUp = landingAssistHit.normal;
        Vector3 uprightBlendedUp = Vector3.Slerp(landingUp, Vector3.up, 0.5f).normalized; // 50% upright bias
        Vector3 correctedForward = Vector3.ProjectOnPlane(transform.forward, uprightBlendedUp).normalized;
        
        if (correctedForward.magnitude < 0.1f)
        {
            correctedForward = Vector3.ProjectOnPlane(Vector3.forward, uprightBlendedUp).normalized;
        }
        
        targetRotation = Quaternion.LookRotation(correctedForward, uprightBlendedUp);
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
}