using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
public class KartInput : MonoBehaviour
{
    [Header("Input Actions")]
    public InputAction steerAction;
    public InputAction accelerateAction;
    public InputAction brakeAction;
    public InputAction trickAction;

    [Header("Kart Components")]
    public Transform leftFrontWheel;
    public Transform rightFrontWheel;
    public Transform leftBackWheel;
    public Transform rightBackWheel;
    public KartData kartData;
    public Rigidbody kartRigidbody;
    public Transform kartVisualsRoot;

    [Header("Jump and Fall Settings")]
    [Tooltip("Controls the upward force of a player-initiated jump.")]
    public float jumpForce = 15f;
    [Tooltip("Controls the manual downward gravity applied during a jump arc.")]
    public float manualFallGravity = 20f;
    [Tooltip("Time the kart can be airborne on a hop before physics takes over.")]
    public float hopGracePeriod = 0.15f;
    [Tooltip("Intensity of the visual hop effect on small bumps.")]
    public float visualHopIntensity = 0.05f;

    [Header("Landing Assist")]
    public float landingAssistLookahead = 0.5f;
    public float landingRayLength = 5.0f;
    public float transitionDuration = 0.2f;
    [Tooltip("Extra vertical offset added during landing transition to ensure kart lands above the ground.")]
    public float landingHeightOffset = 0.1f;

    [Header("Suspension Visuals")]
    public Transform chassisVisuals;

    private bool isTransitioningToGrounded = false;
    private bool isJumping = false;
    private Vector3 transitionStartPosition;
    private Quaternion transitionStartRotation;
    private float transitionTimer = 0f;
    private RaycastHit landingAssistHit;

    private float steerInput = 0f;
    private float currentSteerAngle = 0f;
    private float currentSpeed = 0f;

    private RaycastHit lookaheadHitInfo;
    public BoxCollider kartCollider;

    private bool isGrounded = false;
    private bool wasGrounded = false;

    private float airborneTimer = 0f;

    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private Vector3 averagedNormal;

    private List<RaycastHit> groundHits = new List<RaycastHit>();

    [Header ("Events")]
    public UnityEvent onAccelerate;
    public UnityEvent onDecelerate;
    public UnityEvent onBrake;
    public UnityEvent onSteer;
    public UnityEvent onTrick;
    public UnityEvent onLanding;
    public UnityEvent onWallCollision;

    public void Awake()
    {
        steerAction.performed += OnSteer;
        steerAction.canceled += OnSteerCanceled;
        accelerateAction.performed += OnAccelerate;
        accelerateAction.canceled += OnAcceleratedCanceled;
        brakeAction.performed += OnBrake;
        trickAction.performed += OnTrick;

        kartRigidbody.isKinematic = true;
        kartRigidbody.useGravity = false;

        targetPosition = transform.position;
        targetRotation = transform.rotation;
    }

    private void OnBrake(InputAction.CallbackContext context)
    {
        onBrake.Invoke();
    }

    private void OnSteerCanceled(InputAction.CallbackContext context)
    {
        steerInput = 0f;
    }

    public void OnSteer(InputAction.CallbackContext context)
    {
        steerInput = context.ReadValue<float>();
    }

    private void OnAcceleratedCanceled(InputAction.CallbackContext context)
    {
        onDecelerate.Invoke();
    }

    public void OnAccelerate(InputAction.CallbackContext context)
    {
        onAccelerate.Invoke();
    }

    private void OnTrick(InputAction.CallbackContext context)
    {
        if (isGrounded)
        {
            isJumping = true;
            kartRigidbody.isKinematic = false;
            kartRigidbody.useGravity = false;
            kartRigidbody.linearVelocity = transform.forward * currentSpeed + Vector3.up * jumpForce;
            airborneTimer = 0f;
        }
    }

    void FixedUpdate()
    {
        wasGrounded = isGrounded;
        isGrounded = CheckGround();

        if (isGrounded)
        {
            if (!wasGrounded)
            {
                if (airborneTimer > hopGracePeriod)
                {
                    onLanding.Invoke();
                }

                kartRigidbody.isKinematic = true;
                kartRigidbody.useGravity = false;
                kartRigidbody.Sleep();
                isJumping = false;
            }

            airborneTimer = 0f;
            HandleGroundedMovementAndAlignment();
        }
        else
        {
            airborneTimer += Time.fixedDeltaTime;

            if (airborneTimer < hopGracePeriod && !isJumping)
            {
                Vector3 moveDirection = (targetRotation * Vector3.forward) * currentSpeed;
                targetPosition += moveDirection * Time.fixedDeltaTime;
                targetPosition.y += Physics.gravity.y * Time.fixedDeltaTime * Time.fixedDeltaTime;

                if (kartVisualsRoot != null)
                {
                    float hopLerp = Mathf.InverseLerp(0, hopGracePeriod, airborneTimer);
                    float yOffset = Mathf.Sin(hopLerp * Mathf.PI) * visualHopIntensity;
                    kartVisualsRoot.localPosition = Vector3.Lerp(kartVisualsRoot.localPosition, new Vector3(0, kartData.groundContactOffset + yOffset, 0), Time.fixedDeltaTime * 10f);
                }
            }
            else
            {
                if (isJumping)
                {
                    kartRigidbody.linearVelocity += Vector3.down * manualFallGravity * Time.fixedDeltaTime;
                }
                else
                {
                    if (kartRigidbody.isKinematic)
                    {
                        kartRigidbody.isKinematic = false;
                        kartRigidbody.useGravity = true;
                        kartRigidbody.linearVelocity = transform.forward * currentSpeed;
                    }
                }
            }

            Vector3 raycastDirection = Vector3.Slerp(Vector3.down, transform.forward, landingAssistLookahead).normalized;
            Debug.DrawRay(transform.position, raycastDirection * landingRayLength, Color.red);

            if (Physics.Raycast(transform.position, raycastDirection, out landingAssistHit, landingRayLength, kartData.groundLayer) && airborneTimer > hopGracePeriod)
            {
                if (!isTransitioningToGrounded)
                {
                    StartLandingTransition();
                }
            }
        }
    }

    void Update()
    {
        if (isTransitioningToGrounded)
        {
            HandleLandingTransition();
        }
        else if (isGrounded)
        {
            transform.position = Vector3.Lerp(transform.position, targetPosition, 25f * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 25f * Time.deltaTime);
        }

        HandleChassisVisuals();
        HandleWheelVisuals();
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

        float finalOffset = kartData.groundContactOffset + landingHeightOffset;
        targetPosition = landingAssistHit.point + landingAssistHit.normal * finalOffset;
        targetRotation = Quaternion.FromToRotation(transform.up, landingAssistHit.normal) * transform.rotation;

        currentSpeed = kartRigidbody.linearVelocity.magnitude;
        transitionTimer = 0;
    }

    private void HandleLandingTransition()
    {
        transitionTimer += Time.fixedDeltaTime;
        float t = Mathf.Clamp01(transitionTimer / transitionDuration);

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
        groundHits.Clear();
        int groundedCount = 0;

        RaycastHit hit;
        bool backWheelsGrounded = false;

        Vector3 raycastOrigin_FL = leftFrontWheel.position - transform.up * kartData.groundContactOffset;
        Vector3 raycastOrigin_FR = rightFrontWheel.position - transform.up * kartData.groundContactOffset;
        Vector3 raycastOrigin_BL = leftBackWheel.position - transform.up * kartData.groundContactOffset;
        Vector3 raycastOrigin_BR = rightBackWheel.position - transform.up * kartData.groundContactOffset;

        if (Physics.Raycast(raycastOrigin_FL, -transform.up, out hit, kartData.groundCheckDistance, kartData.groundLayer))
        {
            groundedCount++;
            groundHits.Add(hit);
        }
        if (Physics.Raycast(raycastOrigin_FR, -transform.up, out hit, kartData.groundCheckDistance, kartData.groundLayer))
        {
            groundedCount++;
            groundHits.Add(hit);
        }
        if (Physics.Raycast(raycastOrigin_BL, -transform.up, out hit, kartData.groundCheckDistance, kartData.groundLayer))
        {
            groundedCount++;
            groundHits.Add(hit);
            backWheelsGrounded = true;
        }
        if (Physics.Raycast(raycastOrigin_BR, -transform.up, out hit, kartData.groundCheckDistance, kartData.groundLayer))
        {
            groundedCount++;
            groundHits.Add(hit);
            backWheelsGrounded = true;
        }

        Vector3 tempAveragedNormal = Vector3.zero;
        foreach (var h in groundHits)
        {
            tempAveragedNormal += h.normal;
        }
        averagedNormal = tempAveragedNormal.normalized;

        return groundedCount >= 3 || (groundedCount >= 2 && backWheelsGrounded);
    }

    private void HandleGroundedMovementAndAlignment()
    {
        // Handle Steering and Acceleration/Deceleration
        HandleSteering();
        if (accelerateAction.IsPressed())
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

        // Project the forward movement onto the averaged ground normal's plane
        Vector3 forwardOnGround = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, averagedNormal);

        // Update target rotation to align with the ground
        targetRotation = Quaternion.LookRotation(forwardOnGround, averagedNormal);

        // Wall collision check
        bool lookaheadHitFound = false;
        if (kartCollider != null && Mathf.Abs(currentSpeed) > 0.1f)
        {
            Vector3 boxCastDirection = (targetRotation * Vector3.forward).normalized;
            if (currentSpeed < 0)
            {
                boxCastDirection = (targetRotation * Vector3.back).normalized;
            }

            float boxCastDistance = currentSpeed * Time.fixedDeltaTime;
            Vector3 castOrigin = transform.position + (transform.forward * 0.5f);

            // Use a specific layer mask for non-ground objects
            int wallLayer = ~(kartData.groundLayer);

            lookaheadHitFound = Physics.BoxCast(castOrigin, kartCollider.size * 0.5f, boxCastDirection, out lookaheadHitInfo, targetRotation, boxCastDistance, wallLayer);
        }

        if (lookaheadHitFound)
        {
            // The angle check is now based on your specific requirement
            float angleToUp = Vector3.Angle(lookaheadHitInfo.normal, Vector3.up);

            if (angleToUp > 45f) // This is a wall
            {
                onWallCollision.Invoke();
                
                Vector3 incomingVector = (targetRotation * Vector3.forward).normalized;
                if (currentSpeed < 0)
                {
                    incomingVector = (targetRotation * Vector3.back).normalized;
                }
                
                Vector3 reflectionVector = Vector3.Reflect(incomingVector, lookaheadHitInfo.normal);
                reflectionVector = Vector3.ProjectOnPlane(reflectionVector, averagedNormal).normalized;

                currentSpeed = currentSpeed * 0.8f;
                targetRotation = Quaternion.LookRotation(reflectionVector, averagedNormal);
                
                targetPosition += reflectionVector * (currentSpeed * Time.fixedDeltaTime);
            }
            else // This is a slope, let the regular movement logic handle it
            {
                Vector3 moveDirection = forwardOnGround.normalized * currentSpeed;
                targetPosition += moveDirection * Time.fixedDeltaTime;
            }
        }
        else
        {
            // Move the target position along the projected forward vector
            Vector3 moveDirection = forwardOnGround.normalized * currentSpeed;
            targetPosition += moveDirection * Time.fixedDeltaTime;
        }

        // Final height correction based on a central raycast
        RaycastHit heightHit;
        if (Physics.Raycast(targetPosition + Vector3.up * kartData.groundCheckDistance, Vector3.down, out heightHit, kartData.groundCheckDistance * 2, kartData.groundLayer))
        {
            // Set the final target position to be the hit point plus the offset, but maintain the horizontal movement
            Vector3 newTargetPos = targetPosition;
            newTargetPos.y = heightHit.point.y + kartData.groundContactOffset;
            targetPosition = newTargetPos;
        }
    }

    private void HandleSteering()
    {
        float effectiveSteeringFactor;
        float absoluteSpeed = Mathf.Abs(currentSpeed);

        if (absoluteSpeed <= kartData.maxSpeed / 2f)
        {
            float speedFactor = Mathf.InverseLerp(0, kartData.maxSpeed / 2f, absoluteSpeed);
            effectiveSteeringFactor = Mathf.Lerp(0, 1, speedFactor);
        }
        else
        {
            float speedFactor = Mathf.InverseLerp(kartData.maxSpeed / 2f, kartData.maxSpeed, absoluteSpeed);
            effectiveSteeringFactor = Mathf.Lerp(1, kartData.steeringDampenAtMaxSpeed, speedFactor);
        }

        float turnAmount = steerInput * kartData.turnSpeed * effectiveSteeringFactor * Time.fixedDeltaTime;
        targetRotation *= Quaternion.Euler(0, turnAmount, 0);
    }

    private void HandleChassisVisuals()
    {
        if (chassisVisuals != null)
        {
            Quaternion targetTilt = Quaternion.Euler(0, steerInput * kartData.maxChassisTiltAngle, 0);
            chassisVisuals.localRotation = Quaternion.Slerp(chassisVisuals.localRotation, targetTilt, kartData.tiltDampening * Time.fixedDeltaTime);
        }

        if (kartVisualsRoot != null && isGrounded)
        {
            float slopeAngle = Vector3.Angle(Vector3.up, averagedNormal);
            float adjustedOffset = kartData.groundContactOffset;

            if (Mathf.Abs(currentSpeed) > 0.1f)
            {
                adjustedOffset += (0.02f * slopeAngle);
            }

            Vector3 newLocalPos = new Vector3(0, adjustedOffset, 0);
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

        float wheelSpeed = isGrounded ? currentSpeed : kartRigidbody.linearVelocity.magnitude;
        float wheelRotationSpeed = wheelSpeed * 360f / (2f * Mathf.PI * kartData.wheelRadius);

        leftFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        rightFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        leftBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        rightBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
    }

    public void OnEnable()
    {
        steerAction.Enable();
        accelerateAction.Enable();
        brakeAction.Enable();
        trickAction.Enable();
    }

    public void OnDisable()
    {
        steerAction.Disable();
        accelerateAction.Disable();
        brakeAction.Disable();
        trickAction.Disable();
    }
}