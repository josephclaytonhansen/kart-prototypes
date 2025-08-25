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
    [Tooltip("Time the kart can be airborne on a hop before physics takes over.")]
    public float hopGracePeriod = 0.15f;
    [Tooltip("Intensity of the visual hop effect on small bumps.")]
    public float visualHopIntensity = 0.05f;

    private bool isOnRamp = false;
    private bool isPerformingTrick = false;
    private bool isJumping = false;

    [Header("Landing Assist")]
    public float landingAssistLookahead = 0.5f;
    public float landingRayLength = 5.0f;
    public float transitionDuration = 0.2f;
    [Tooltip("Extra vertical offset added during landing transition to ensure kart lands above the ground.")]
    public float landingHeightOffset = 0.1f;

    [Header("Suspension Visuals")]
    public Transform chassisVisuals;

    private bool isTransitioningToGrounded = false;
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

    [Header("Events")]
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
        if (isJumping)
        {
            isPerformingTrick = true;
            onTrick.Invoke();
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
                isPerformingTrick = false;
            }

            airborneTimer = 0f;
            HandleGroundedMovementAndAlignment();
        }
        else // Airborne State
        {
            airborneTimer += Time.fixedDeltaTime;

            // This is the core change: transition to a physics Rigidbody immediately.
            // This happens for both ramp jumps AND falls off an edge.
            if (kartRigidbody.isKinematic)
            {
                kartRigidbody.isKinematic = false;
                kartRigidbody.useGravity = true;
                // Preserve forward velocity. We only apply a new jump impulse in the jump method.
                kartRigidbody.linearVelocity = transform.forward * currentSpeed;
            }
        }

        // The remaining logic for landing assist and visuals is unchanged
        // as it was already working correctly.
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

        Vector3 raycastOrigin_FL = leftFrontWheel.position - transform.up * kartData.groundContactOffset;
        Vector3 raycastOrigin_FR = rightFrontWheel.position - transform.up * kartData.groundContactOffset;
        Vector3 raycastOrigin_BL = leftBackWheel.position - transform.up * kartData.groundContactOffset;
        Vector3 raycastOrigin_BR = rightBackWheel.position - transform.up * kartData.groundContactOffset;

        bool backWheelsGrounded = false;

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

        Vector3 forwardOnGround = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, averagedNormal);
        targetRotation = Quaternion.LookRotation(forwardOnGround, averagedNormal);

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
            int wallLayer = ~(kartData.groundLayer);
            lookaheadHitFound = Physics.BoxCast(castOrigin, kartCollider.size * 0.5f, boxCastDirection, out lookaheadHitInfo, targetRotation, boxCastDistance, wallLayer);
        }

        if (lookaheadHitFound)
        {
            float angleToUp = Vector3.Angle(lookaheadHitInfo.normal, Vector3.up);

            if (angleToUp > 45f)
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
            else
            {
                Vector3 moveDirection = forwardOnGround.normalized * currentSpeed;
                targetPosition += moveDirection * Time.fixedDeltaTime;
            }
        }
        else
        {
            Vector3 moveDirection = forwardOnGround.normalized * currentSpeed;
            targetPosition += moveDirection * Time.fixedDeltaTime;
        }

        RaycastHit heightHit;
        if (Physics.Raycast(targetPosition + Vector3.up * kartData.groundCheckDistance, Vector3.down, out heightHit, kartData.groundCheckDistance * 2, kartData.groundLayer))
        {
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
            float averageHitHeight = 0f;
            if (groundHits.Count > 0)
            {
                foreach (var hit in groundHits)
                {
                    averageHitHeight += hit.point.y;
                }
                averageHitHeight /= groundHits.Count;
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

        float wheelSpeed = isGrounded ? currentSpeed : kartRigidbody.linearVelocity.magnitude;
        float wheelRotationSpeed = wheelSpeed * 360f / (2f * Mathf.PI * kartData.wheelRadius);

        leftFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        rightFrontWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        leftBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
        rightBackWheel.Rotate(wheelRotationSpeed * Time.fixedDeltaTime, 0, 0);
    }
    
    private void HandleRampJump()
    {
        isJumping = true;
        // Core change: use AddForce() to add to existing velocity
        kartRigidbody.AddForce(transform.up * jumpForce, ForceMode.VelocityChange);
        airborneTimer = 0f;
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("JumpRamp"))
        {
            isOnRamp = true;
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("JumpRamp") && isOnRamp)
        {
            isOnRamp = false;
            HandleRampJump();
        }
        else
        {
            isOnRamp = false;
        }
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