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
    
    public Transform kartVisualsRoot; 

    [Header("Airborne Settings")]
    public float landingAssistLookahead = 0.5f;
    public float landingRayLength = 5.0f;
    public float transitionDuration = 0.2f;

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

        kartRigidbody.isKinematic = false;
        kartRigidbody.useGravity = true;
        
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

        // Transition from Airborne to Grounded
        if (isGrounded && !wasGrounded && airborneTimer > 0.05f) 
        {
            kartRigidbody.isKinematic = true;
            kartRigidbody.useGravity = false;
            kartRigidbody.Sleep();
            airborneTimer = 0f;
        } 

        // Transition from Grounded to Airborne
        if (wasGrounded && !isGrounded) 
        { 
            kartRigidbody.isKinematic = false;
            kartRigidbody.useGravity = true;
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
            Vector3 raycastDirection = Vector3.Slerp(Vector3.down, transform.forward, landingAssistLookahead).normalized;
            Debug.DrawRay(transform.position, raycastDirection * landingRayLength, Color.red);

            if (Physics.Raycast(transform.position, raycastDirection, out landingAssistHit, landingRayLength, kartData.groundLayer) && airborneTimer > 0.2f)
            {
                if (!isTransitioningToGrounded)
                {
                    StartLandingTransition();
                }
            }

            if(Physics.Raycast(transform.position, raycastDirection, out landingAssistHit, landingRayLength, kartData.groundLayer))
            {
                targetRotation = Quaternion.Slerp(targetRotation, Quaternion.FromToRotation(targetRotation * Vector3.up, landingAssistHit.normal) * targetRotation, Time.fixedDeltaTime * 10f);
            }
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

    private void StartLandingTransition()
    {
        if (isTransitioningToGrounded) return;

        isTransitioningToGrounded = true;
        kartRigidbody.isKinematic = true;
        kartRigidbody.useGravity = false;
        kartRigidbody.Sleep();

        transitionStartPosition = transform.position;
        transitionStartRotation = transform.rotation;

        Vector3 averageHitPoint = Vector3.zero;
        if(groundHits.Count > 0)
        {
             foreach(var hit in groundHits)
             {
                averageHitPoint += hit.point;
             }
             averageHitPoint /= groundHits.Count;
        }
        else // Fallback if no groundHits
        {
             averageHitPoint = landingAssistHit.point;
        }
        
        // Use the average of the four ground checks for a precise landing position
        targetPosition = averageHitPoint + (transform.up * kartData.groundContactOffset);
        
        // This is still a good fallback for aligning the rotation
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
        // This is now our single point for applying the smooth position and rotation changes.
        if (isTransitioningToGrounded || isGrounded)
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