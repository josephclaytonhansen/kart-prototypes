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
    
    private RaycastHit lookaheadHit;
    
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
            kartRigidbody.isKinematic = true;
            currentSpeed = kartRigidbody.linearVelocity.magnitude;
            kartRigidbody.Sleep();
            targetPosition = transform.position;
            airborneTimer = 0f;
        }

        // Transition from Grounded to Airborne
        if (wasGrounded && !isGrounded)
        {
            kartRigidbody.isKinematic = false;
            kartRigidbody.linearVelocity = (targetRotation * Vector3.forward) * currentSpeed + Vector3.up * kartData.jumpForce;
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
            Vector3 horizontalForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            kartRigidbody.AddForce(horizontalForward * currentSpeed * kartData.airborneSpeedFactor, ForceMode.Acceleration);
        }
        
        // This is the new, corrected way to apply the visual offset.
        if (kartVisualsRoot != null && isGrounded)
        {
            float slopeAngle = Vector3.Angle(Vector3.up, averagedNormal);
            float adjustedOffset = kartData.groundContactOffset + (0.005f * slopeAngle);
            kartVisualsRoot.localPosition = new Vector3(0, adjustedOffset, 0);
        }

        HandleWheelVisuals();
    }
    
    private bool CheckGround()
    {
        RaycastHit hit;
        groundHits.Clear();
        int groundedCount = 0;

        // Corrected raycast origins to account for the visual offset
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
        
        if (currentSpeed > 0.1f)
        {
            lookaheadHitFound = Physics.Raycast(targetPosition, targetRotation * Vector3.forward, out lookaheadHit, Vector3.Distance(targetPosition, newTargetPosition), kartData.groundLayer);
        }
        else if (currentSpeed < -0.1f)
        {
            lookaheadHitFound = Physics.Raycast(targetPosition, targetRotation * Vector3.back, out lookaheadHit, Vector3.Distance(targetPosition, newTargetPosition), kartData.groundLayer);
        }

        if (lookaheadHitFound)
        {
            float angle = Vector3.Angle(Vector3.up, lookaheadHit.normal);
            
            if (angle > 45f)
            {
                isBouncing = true;
                
                Vector3 incomingVector = (targetRotation * Vector3.forward).normalized;
                Vector3 reflectionVector = Vector3.Reflect(incomingVector, lookaheadHit.normal);
                reflectionVector.y = 0;
                moveDirection = reflectionVector.normalized * kartData.bounceStrength;
                
                targetPosition = lookaheadHit.point + lookaheadHit.normal * 0.1f;
            }
            else if (angle > 1f)
            {
                moveDirection = Vector3.ProjectOnPlane(moveDirection, lookaheadHit.normal);
                targetPosition += moveDirection * Time.fixedDeltaTime;
            }
        }
        else
        {
            targetPosition += moveDirection * Time.fixedDeltaTime;
        }
    }

    private void HandleSteering()
    {
        if (Mathf.Abs(currentSpeed) > 0.1f)
        {
            float turnAmount = steerInput * kartData.turnSpeed * Time.fixedDeltaTime;
            targetRotation *= Quaternion.Euler(0, turnAmount, 0);
        }
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