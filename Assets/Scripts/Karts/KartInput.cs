using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;

public class KartInput : MonoBehaviour
{
    [Header("Input Actions")]
    public InputAction steerAction;
    public InputAction accelerateAction;
    public InputAction brakeAction;
    
    [Header("Kart Components")]
    public Transform leftFrontWheel;
    public Transform rightFrontWheel;
    public Transform leftBackWheel;
    public Transform rightBackWheel;
    public KartData kartData;
    public Rigidbody kartRigidbody;
    
    // The new variable for the visual part of the kart
    public Transform kartVisualsRoot;

    private float steerInput = 0f;
    private float currentSteerAngle = 0f;
    private float currentSpeed = 0f;
    
    private RaycastHit lookaheadHitInfo;
    public BoxCollider kartCollider;
    
    private bool isGrounded = false;
    private bool wasGrounded = false;
    private bool isBouncing = false;
    private float groundedTime = 0f;
    
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

    public void Awake()
    {
        steerAction.performed += OnSteer;
        steerAction.canceled += OnSteerCanceled;
        accelerateAction.performed += OnAccelerate;
        accelerateAction.canceled += OnAccelerateCanceled;
        brakeAction.performed += OnBrake;

        kartRigidbody.isKinematic = true;
        
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

    private void OnAccelerateCanceled(InputAction.CallbackContext context)
    {
        onDecelerate.Invoke();
    }

    public void OnAccelerate(InputAction.CallbackContext context)
    {
        onAccelerate.Invoke();
    }

    void FixedUpdate()
    {
        wasGrounded = isGrounded;
        bool rawGrounded = CheckGround();

        if (rawGrounded)
        {
            groundedTime += Time.fixedDeltaTime;
        }
        else
        {
            groundedTime = 0;
        }

        isGrounded = groundedTime > 0.05f;
        
        if (!isGrounded)
        {
            airborneTimer += Time.fixedDeltaTime;
        }

        // State Machine Logic
        // Transition from Airborne to Grounded
        if (!wasGrounded && isGrounded && airborneTimer > 0.1f)
        {
            // NEW: Get the average hit point from the raycasts to find the exact landing spot
            Vector3 averageHitPoint = Vector3.zero;
            foreach(var hit in groundHits)
            {
                averageHitPoint += hit.point;
            }
            averageHitPoint /= groundHits.Count;

            // NEW: Snap the target position to the ground to ensure a clean landing
            targetPosition = averageHitPoint + (transform.up * kartData.groundContactOffset);
            
            kartRigidbody.isKinematic = true;
            currentSpeed = kartRigidbody.linearVelocity.magnitude;
            kartRigidbody.Sleep();
            airborneTimer = 0f;
        }

        // Transition from Grounded to Airborne
        if (wasGrounded && !isGrounded)
        {
            kartRigidbody.isKinematic = false;
            // NEW: Use the kart's forward direction to preserve the pitch and prevent the nose-down effect
            kartRigidbody.linearVelocity = transform.forward * currentSpeed + Vector3.up * kartData.jumpForce;
            airborneTimer = 0f;
        }

        // Main logic for grounded state
        if (isGrounded)
        {
            HandleMovement();
            HandleSteering();
            
            if (!isBouncing)
            {
                HandleGroundAlignment();
            }
        }
        else // Main logic for airborne state
        {
            // The old AddForce logic caused jitter. The initial launch velocity is all that's needed.
        }
        
        if (kartVisualsRoot != null && isGrounded)
        {
            float slopeAngle = Vector3.Angle(Vector3.up, averagedNormal);
            float adjustedOffset = kartData.groundContactOffset;
            
            if (Mathf.Abs(currentSpeed) > 0.1f)
            {
                adjustedOffset += (0.02f * slopeAngle);
            }
            
            kartVisualsRoot.localPosition = new Vector3(0, adjustedOffset, 0);
        }

        HandleWheelVisuals();
    }
    
    private bool CheckGround()
    {
        RaycastHit hit;
        groundHits.Clear();
        int groundedCount = 0;

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
        }
        if (Physics.Raycast(raycastOrigin_BR, -transform.up, out hit, kartData.groundCheckDistance, kartData.groundLayer))
        {
            groundedCount++;
            groundHits.Add(hit);
        }
        
        Vector3 tempAveragedNormal = Vector3.zero;
        foreach (var h in groundHits)
        {
            tempAveragedNormal += h.normal;
        }
        averagedNormal = tempAveragedNormal.normalized;

        return groundedCount >= 3;
    }

    void Update()
    {
        if (kartRigidbody.isKinematic)
        {
            transform.position = Vector3.Lerp(transform.position, targetPosition, 25f * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 25f * Time.deltaTime);
        }
    }

    private void HandleMovement()
    {
        isBouncing = false;
        
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

        Vector3 moveDirection = (targetRotation * Vector3.forward) * currentSpeed;
        Vector3 newTargetPosition = targetPosition + moveDirection * Time.fixedDeltaTime;

        bool lookaheadHitFound = false;
        
        if (kartCollider != null && Mathf.Abs(currentSpeed) > 0.1f)
        {
            Vector3 boxCastDirection;
            if (currentSpeed > 0)
            {
                boxCastDirection = (targetRotation * Vector3.forward).normalized;
            }
            else
            {
                boxCastDirection = (targetRotation * Vector3.back).normalized;
            }
            
            float boxCastDistance = Vector3.Distance(targetPosition, newTargetPosition);
            
            lookaheadHitFound = Physics.BoxCast(targetPosition, kartCollider.size * 0.5f, boxCastDirection, out lookaheadHitInfo, targetRotation, boxCastDistance, kartData.groundLayer);
        }

        if (lookaheadHitFound)
        {
            float remainingDistance = lookaheadHitInfo.distance;
            targetPosition += moveDirection.normalized * remainingDistance;

            Vector3 kartForwardOnGround = Vector3.ProjectOnPlane(transform.forward, averagedNormal);
            Vector3 hitNormalOnGround = Vector3.ProjectOnPlane(lookaheadHitInfo.normal, averagedNormal);

            float angleToWall = Vector3.Angle(kartForwardOnGround, hitNormalOnGround);

            if (angleToWall > 70f)
            {
                Vector3 incomingVector = kartForwardOnGround.normalized;
                Vector3 reflectionVector = Vector3.Reflect(incomingVector, hitNormalOnGround);
                
                currentSpeed = currentSpeed * 0.8f;
                moveDirection = reflectionVector.normalized * currentSpeed;
                targetPosition += moveDirection * Time.fixedDeltaTime;
            }
            else
            {
                currentSpeed = 0f;
            }
        }
        else
        {
            targetPosition += moveDirection * Time.fixedDeltaTime;
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

    private void HandleGroundAlignment()
    {
        Vector3 tempAveragedNormal = Vector3.zero;
        
        foreach (var hit in groundHits)
        {
            tempAveragedNormal += hit.normal;
        }
        
        averagedNormal = tempAveragedNormal.normalized;

        targetRotation = Quaternion.FromToRotation(targetRotation * Vector3.up, averagedNormal) * targetRotation;
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
    }

    public void OnDisable()
    {
        steerAction.Disable();
        accelerateAction.Disable();
        brakeAction.Disable();
    }
}