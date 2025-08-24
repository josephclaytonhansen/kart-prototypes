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
    
    [Header("Kart Components")] 
    public Transform leftFrontWheel; 
    public Transform rightFrontWheel; 
    public Transform leftBackWheel; 
    public Transform rightBackWheel; 
    public KartData kartData; 
    public Rigidbody kartRigidbody; 
    public Transform kartVisualsRoot; 

    [Header("Airborne Settings")]
    public float landingAssistLookahead = 0.5f;
    public float landingRayLength = 5.0f;
    public float transitionDuration = 0.2f;

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
    
    [Header ("Events")] 
    public UnityEvent onAccelerate; 
    public UnityEvent onDecelerate; 
    public UnityEvent onBrake; 
    public UnityEvent onSteer; 
    public UnityEvent onJump;
    public UnityEvent onLanding;
    public UnityEvent onWallCollision; // New event

    public void Awake() 
    { 
        steerAction.performed += OnSteer; 
        steerAction.canceled += OnSteerCanceled; 
        accelerateAction.performed += OnAccelerate; 
        accelerateAction.canceled += OnAccelerateCanceled; 
        brakeAction.performed += OnBrake; 

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
        isGrounded = CheckGround(); 
        
        // Handle transitions between states
        if (isGrounded && !wasGrounded) 
        {
            // Transition from Airborne to Grounded
            // This is now handled by the landing transition logic
            if (!isTransitioningToGrounded)
            {
                // Fallback for small hops where a transition wasn't needed
                kartRigidbody.isKinematic = true;
                kartRigidbody.useGravity = false;
                kartRigidbody.Sleep();
                
                if (airborneTimer > 0.05f) 
                {
                    onLanding.Invoke();
                }
            }
            airborneTimer = 0f;
        } 
        
        if (wasGrounded && !isGrounded) 
        { 
            // Transition from Grounded to Airborne
            kartRigidbody.isKinematic = false;
            kartRigidbody.useGravity = true;
            kartRigidbody.linearVelocity = transform.forward * currentSpeed + Vector3.up * kartData.jumpForce; 
            airborneTimer = 0f; 
            onJump.Invoke();
        } 

        // Main logic for grounded state
        if (isGrounded) 
        { 
            HandleMovement(); 
            HandleSteering(); 
            HandleGroundAlignment(); 
        } 
        else // Main logic for airborne state
        {
            airborneTimer += Time.fixedDeltaTime;
            
            // Landing Assist: Anticipate landing and start smooth transition
            Vector3 raycastDirection = Vector3.Slerp(Vector3.down, transform.forward, landingAssistLookahead).normalized;
            Debug.DrawRay(transform.position, raycastDirection * landingRayLength, Color.red);

            if (Physics.Raycast(transform.position, raycastDirection, out landingAssistHit, landingRayLength, kartData.groundLayer) && airborneTimer > 0.2f)
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
        // This is now our single point for applying the smooth position and rotation changes.
        if (isTransitioningToGrounded)
        {
            HandleLandingTransition();
        }
        else if (isGrounded)
        {
            // When grounded, smoothly lerp to the target
            transform.position = Vector3.Lerp(transform.position, targetPosition, 25f * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 25f * Time.deltaTime);
        }

        // Apply visual tilt and Y offset
        HandleChassisVisuals();
        
        // Handle wheel visuals independently of the main kart body
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
        
        // Use the landing assist hit for a precise landing position and rotation
        targetPosition = landingAssistHit.point + (Quaternion.FromToRotation(transform.up, landingAssistHit.normal) * transform.rotation * Vector3.up * kartData.groundContactOffset);
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

        if(t >= 1.0f)
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
        RaycastHit hit; 
        groundHits.Clear(); 
        int groundedCount = 0; 

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

        // Always be considered grounded if the back two wheels are.
        return groundedCount >= 3 || (groundedCount >= 2 && backWheelsGrounded); 
    } 

    private void HandleMovement() 
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
            
            lookaheadHitFound = Physics.BoxCast(targetPosition + (Vector3.up * 0.1f), kartCollider.size * 0.5f, boxCastDirection, out lookaheadHitInfo, targetRotation, boxCastDistance, kartData.groundLayer); 
        } 

        if (lookaheadHitFound) 
        { 
            float remainingDistance = lookaheadHitInfo.distance; 
            targetPosition += moveDirection.normalized * remainingDistance; 

            Vector3 kartForwardOnGround = Vector3.ProjectOnPlane(transform.forward, averagedNormal); 
            Vector3 hitNormalOnGround = Vector3.ProjectOnPlane(lookaheadHitInfo.normal, averagedNormal); 

            float angleToWall = Vector3.Angle(kartForwardOnGround, hitNormalOnGround); 

            // Only bounce if the angle is steep enough, otherwise stop.
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
        targetRotation = Quaternion.FromToRotation(targetRotation * Vector3.up, averagedNormal) * targetRotation; 
    } 

    private void HandleChassisVisuals()
    {
        // Apply visual tilt to the dedicated chassis object
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
    } 

    public void OnDisable() 
    { 
        steerAction.Disable(); 
        accelerateAction.Disable(); 
        brakeAction.Disable(); 
    } 
}