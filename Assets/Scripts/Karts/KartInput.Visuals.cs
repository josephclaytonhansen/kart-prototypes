using UnityEngine;
using System;

public partial class KartInput
{
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
}