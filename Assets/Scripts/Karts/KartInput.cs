using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;
using System;
using Unity.Cinemachine;

[RequireComponent(typeof(Rigidbody))]
public class KartInput : MonoBehaviour
{
    [Header("Input Actions")]
    public InputAction steerAction;
    public InputAction accelerateAction;
    public InputAction brakeAction;
    public InputAction trickAction;
    public InputAction driftAction;
    public InputAction lookBackAction;

    [Header("Camera Settings")]
    public CinemachineCamera kartCamera;
    public CinemachineCamera lookbackCamera;

    [Header("Kart Components")]
    public Transform leftFrontWheel;
    public Transform rightFrontWheel;
    public Transform leftBackWheel;
    public Transform rightBackWheel;
    public KartData kartData;
    public KartGameSettings kartGameSettings;
    public Rigidbody kartRigidbody;
    public Transform kartVisualsRoot;
    [Tooltip("The BoxCollider used for wall collision detection (drag from Chassis child)")]
    public BoxCollider kartCollider;

    public bool autoDrift = false;

    [Header("Tailwind Settings")]
    private float tailwindTimer = 0f;

    [Header("Jump and Fall Settings")]
    private bool isOnRamp = false;
    private bool isPerformingTrick = false;
    private bool isJumping = false;

    // Collision state variables
    private bool isBouncing = false;
    private float bounceTimer = 0f;
    private Vector3 bounceStartPosition;
    private Vector3 bounceTargetPosition;
    private Quaternion bounceStartRotation;
    private Quaternion bounceTargetRotation;
    private float preCollisionSpeed = 0f;
    private Vector3 bounceVelocity;

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

    private float currentSlope = 0f;

    private RaycastHit lookaheadHitInfo;

    private bool isGrounded = false;
    private bool wasGrounded = false;

    private bool isDrifting = false;
    private float driftTimer = 0f;

    private float airborneTimer = 0f;

    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private Vector3 averagedNormal;

    private readonly List<RaycastHit> groundHits = new List<RaycastHit>();
    private Transform[] wheelTransforms;

    [Header("Events")]
    public UnityEvent onAccelerate;
    public UnityEvent onDecelerate;
    public UnityEvent onBrake;
    public UnityEvent onSteer;
    public UnityEvent onTrick;
    public UnityEvent onLanding;
    public UnityEvent onWallCollision;
    public UnityEvent onDrift;
    public UnityEvent onDriftBlueBoost;
    public UnityEvent onDriftOrangeBoost;
    public UnityEvent onTailwind;
    public UnityEvent onTailwindBoost;

    public void Awake()
    {
        steerAction.performed += OnSteer;
        steerAction.canceled += OnSteerCanceled;
        accelerateAction.performed += OnAccelerate;
        accelerateAction.canceled += OnAcceleratedCanceled;
        brakeAction.performed += OnBrake;
        trickAction.performed += OnTrick;
        driftAction.performed += OnDrift;
        driftAction.canceled += OnDriftCanceled;
        lookBackAction.performed += OnLookBack;
        lookBackAction.canceled += OnLookBackCanceled;

        kartRigidbody.isKinematic = true;
        kartRigidbody.useGravity = false;

        targetPosition = transform.position;
        targetRotation = transform.rotation;

        // Cache wheel transforms for efficiency
        wheelTransforms = new Transform[]
        {
            leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel
        };
    }

    private void OnLookBack(InputAction.CallbackContext context)
    {
        Debug.Log("Look Back Activated");
        if (lookbackCamera != null && kartCamera != null)
        {
            lookbackCamera.Priority = 20;
            kartCamera.Priority = 10;
        }
    }

    private void OnLookBackCanceled(InputAction.CallbackContext context)
    {
        Debug.Log("Look Back Deactivated");
        if (lookbackCamera != null && kartCamera != null)
        {
            lookbackCamera.Priority = 10;
            kartCamera.Priority = 20;
        }
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
        onSteer.Invoke();
    }

    public void OnDrift(InputAction.CallbackContext context)
    {
        if (isGrounded && accelerateAction.IsPressed() && steerInput != 0 && !isDrifting)
        {
            isDrifting = true;
            driftTimer = 0f;
            onDrift.Invoke();
        }
    }

    public void OnDriftCanceled(InputAction.CallbackContext context)
    {
        if (isDrifting)
        {
            isDrifting = false;

            float driftDuration = driftTimer;
            driftTimer = 0f;

            if (driftDuration >= kartGameSettings.driftTimeToOrangeBoost && !autoDrift)
            {
                onDriftOrangeBoost.Invoke();
            }
            else if (driftDuration >= kartGameSettings.driftTimeToBlueBoost && !autoDrift)
            {
                onDriftBlueBoost.Invoke();
            }
        }
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
            HandleGroundedMovement();
        }
        else
        {
            HandleAirborneMovement();
        }

        CheckForLandingAssist();
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

        // PREDICTIVE COLLISION CHECK
        // Dynamically calculate the raycast distance based on the predicted movement speed and a safety factor.
        float checkDistance = (currentSpeed * Time.fixedDeltaTime) + kartGameSettings.collisionLookahead;
        Vector3 checkDirection = movementDirection;
        
        if (Physics.Raycast(transform.position, checkDirection, out lookaheadHitInfo, checkDistance, kartData.groundLayer))
        {
            onWallCollision.Invoke();
            float wallAngle = Vector3.Angle(lookaheadHitInfo.normal, Vector3.up);

            if (wallAngle > kartGameSettings.wallAngleThreshold)
            {
                // Head-On Collision: Stop and bounce backward
                currentSpeed = -kartGameSettings.headOnBounceSpeed;
                targetPosition = lookaheadHitInfo.point + lookaheadHitInfo.normal * kartGameSettings.bounceSafeDistance;
            }
            else
            {
                // Glancing Collision: Slide along the wall with a slight bounce
                Vector3 wallTangent = Vector3.Cross(lookaheadHitInfo.normal, Vector3.up).normalized;
                if (Vector3.Dot(wallTangent, movementDirection) < 0)
                {
                    wallTangent *= -1;
                }
                
                movementDirection = (wallTangent * (1 - kartGameSettings.glancingBounceFactor)) + (lookaheadHitInfo.normal * kartGameSettings.glancingBounceFactor);
                currentSpeed *= (1f - kartGameSettings.glancingSpeedLoss);
                targetPosition = lookaheadHitInfo.point + lookaheadHitInfo.normal * kartGameSettings.bounceSafeDistance;
            }
        }
        
        targetPosition += movementDirection * currentSpeed * Time.fixedDeltaTime;
    }

    private void HandleAlignment()
    {
        Vector3 forwardOnGround = Vector3.ProjectOnPlane(targetRotation * Vector3.forward, averagedNormal);
        targetRotation = Quaternion.LookRotation(forwardOnGround, averagedNormal);

        if (groundHits.Count > 0)
        {
            float averageHitHeight = 0f;
            foreach (var hit in groundHits)
            {
                averageHitHeight += hit.point.y;
            }
            averageHitHeight /= groundHits.Count;

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
        groundHits.Clear();
        int groundedCount = 0;
        Vector3 tempAveragedNormal = Vector3.zero;
        bool backWheelsGrounded = false;

        for (int i = 0; i < wheelTransforms.Length; i++)
        {
            RaycastHit hit;
            Vector3 raycastOrigin = wheelTransforms[i].position - transform.up * kartData.groundContactOffset;
            if (Physics.Raycast(raycastOrigin, -transform.up, out hit, kartData.groundCheckDistance, kartData.groundLayer))
            {
                groundedCount++;
                groundHits.Add(hit);
                tempAveragedNormal += hit.normal;

                if (i >= 2)
                {
                    backWheelsGrounded = true;
                }
            }
        }

        if (groundedCount > 0)
        {
            averagedNormal = tempAveragedNormal.normalized;
        }
        else
        {
            averagedNormal = Vector3.up;
        }

        return groundedCount >= 3 || (groundedCount >= 2 && backWheelsGrounded);
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
        kartRigidbody.AddForce(transform.up * kartGameSettings.jumpForce, ForceMode.VelocityChange);
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
        driftAction.Enable();
        lookBackAction.Enable();
    }

    public void OnDisable()
    {
        steerAction.Disable();
        accelerateAction.Disable();
        brakeAction.Disable();
        trickAction.Disable();
        driftAction.Disable();
        lookBackAction.Disable();
    }
}