using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;

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

    private float steerInput = 0f;
    private float currentSteerAngle = 0f;
    private float currentSpeed = 0f;
    private RaycastHit groundHit;
    private RaycastHit lookaheadHit;
    private bool touchingGround = false;

    // These variables will store the target values calculated in FixedUpdate.
    private Vector3 targetPosition;
    private Quaternion targetRotation;

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
        
        // Initialize the targets to the current transform.
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
        touchingGround = Physics.Raycast(targetPosition, -transform.up, out groundHit, kartData.groundCheckDistance, kartData.groundLayer);

        if (touchingGround)
        {
            if (!kartRigidbody.isKinematic)
            {
                kartRigidbody.isKinematic = true;
                currentSpeed = kartRigidbody.linearVelocity.magnitude;
                kartRigidbody.linearVelocity = Vector3.zero;
                kartRigidbody.angularVelocity = Vector3.zero;
            }

            HandleMovement();
            HandleSteering();
            HandleGroundAlignment();
        }
        else
        {
            if (kartRigidbody.isKinematic)
            {
                kartRigidbody.isKinematic = false;
                kartRigidbody.linearVelocity = transform.forward * currentSpeed;
                currentSpeed = 0f;
            }
        }
        
        HandleWheelVisuals();
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

        bool lookaheadHitFound = false;
        
        if (currentSpeed > 0.1f)
        {
            lookaheadHitFound = Physics.Raycast(targetPosition, targetRotation * Vector3.forward, out lookaheadHit, 1f, kartData.groundLayer);
        }
        else if (currentSpeed < -0.1f)
        {
            lookaheadHitFound = Physics.Raycast(targetPosition, targetRotation * Vector3.back, out lookaheadHit, 1f, kartData.groundLayer);
        }

        if (lookaheadHitFound)
        {
            if (Vector3.Angle(Vector3.up, lookaheadHit.normal) > 1f)
            {
                moveDirection = Vector3.ProjectOnPlane(moveDirection, lookaheadHit.normal);
                if (Vector3.Angle(Vector3.up, lookaheadHit.normal) > 60f)
                {
                    touchingGround = false;
                }
            }
            else
            {
                currentSpeed = 0;
            }
        }
        
        targetPosition += moveDirection * Time.fixedDeltaTime;
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
        float angle = Vector3.Angle(Vector3.up, groundHit.normal);

        if (angle < 60f)
        {
            targetRotation = Quaternion.FromToRotation(targetRotation * Vector3.up, groundHit.normal) * targetRotation;
        }
        else
        {
            touchingGround = false;
        }
    }

    private void HandleWheelVisuals()
    {
        currentSteerAngle = steerInput * kartData.maxSteerAngle;
        leftFrontWheel.localRotation = Quaternion.Euler(0, currentSteerAngle, 0);
        rightFrontWheel.localRotation = Quaternion.Euler(0, currentSteerAngle, 0);
        
        float wheelRotationSpeed = currentSpeed * 360f / (2f * Mathf.PI * kartData.wheelRadius);
        
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