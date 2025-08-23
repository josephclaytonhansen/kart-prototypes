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
    public Transform kartParent;
    public KartData kartData;
    public Transform steeringWheel;
    public Rigidbody kartRigidbody;

    [Header("Smoothing Settings")]
    public float groundSmoothingStrength = 7.5f;
    public float velocitySmoothing = 20f;
    private float currentSpeed = 0f;
    private float steerInput = 0f;
    private float currentSteerAngle = 0f;
    private bool touchingGround = false;

    [Header ("Events")]
    public UnityEvent onAccelerate;
    public UnityEvent onDecelerate;
    public UnityEvent onBrake;
    public UnityEvent onSteer;

    private float targetAngle = 0f;
    private float realSpeed = 0f;
    private float steerAmount = 0f;

    public void Awake()
    {
        steerAction.performed += OnSteer;
        steerAction.canceled += OnSteerCanceled;
        accelerateAction.performed += OnAccelerate;
        accelerateAction.canceled += OnAccelerateCanceled;
        brakeAction.performed += OnBrake;
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
        HandleWheelVisuals();
        HandleMovement();
        HandleSteering();
        HandleGroundNormalRotation();
    }

    private void HandleWheelVisuals()
    {
        targetAngle = steerInput * kartData.maxSteerAngle;
        currentSteerAngle = Mathf.MoveTowards(currentSteerAngle, targetAngle, kartData.steerSpeed * Time.fixedDeltaTime * kartData.maxSteerAngle);
        currentSteerAngle = Mathf.Clamp(currentSteerAngle, kartData.minSteerAngle, kartData.maxSteerAngle);
        
        leftFrontWheel.localEulerAngles = new Vector3(0, 0, currentSteerAngle);
        rightFrontWheel.localEulerAngles = new Vector3(0, 0, currentSteerAngle);
    }

    private void HandleMovement()
    {
        realSpeed = transform.InverseTransformDirection(kartRigidbody.linearVelocity).z;

        if (brakeAction != null && brakeAction.IsPressed())
        {
            if (currentSpeed > 0f)
            {
                currentSpeed = Mathf.MoveTowards(currentSpeed, 0f, kartData.brakeSpeed * Time.fixedDeltaTime);
            }
            else
            {
                currentSpeed = Mathf.MoveTowards(currentSpeed, -kartData.maxSpeed, kartData.accelerateSpeed * Time.fixedDeltaTime);
            }
        }
        else if (accelerateAction.IsPressed())
        {
            currentSpeed = Mathf.MoveTowards(currentSpeed, kartData.maxSpeed, kartData.accelerateSpeed * Time.fixedDeltaTime);
            currentSpeed = Mathf.Min(currentSpeed, kartData.maxSpeed);
        }
        else
        {
            if (currentSpeed > 0f)
            {
                currentSpeed = Mathf.MoveTowards(currentSpeed, 0f, kartData.decelerateSpeed * Time.fixedDeltaTime);
                if (currentSpeed < 0.01f) currentSpeed = 0f;
            }
            else if (currentSpeed < 0f)
            {
                currentSpeed = Mathf.MoveTowards(currentSpeed, 0f, kartData.decelerateSpeed * Time.fixedDeltaTime);
                if (currentSpeed > -0.01f) currentSpeed = 0f;
            }
        }

        Vector3 targetVelocity = transform.forward * currentSpeed;
        targetVelocity.y = kartRigidbody.linearVelocity.y;
        
        Vector3 smoothedVelocity = Vector3.Lerp(kartRigidbody.linearVelocity, targetVelocity, Time.fixedDeltaTime * velocitySmoothing);
        smoothedVelocity.y = kartRigidbody.linearVelocity.y;
        
        kartRigidbody.linearVelocity = smoothedVelocity;
    }

    private void HandleSteering()
    {
        realSpeed = transform.InverseTransformDirection(kartRigidbody.linearVelocity).z;
        steerAmount = realSpeed / 1.5f * steerInput * kartData.turnRadius;

        if (Mathf.Abs(realSpeed) > 0.5f && Mathf.Abs(steerInput) > 0.01f)
        {
            Vector3 steerDirection = new Vector3(
                transform.eulerAngles.x, 
                transform.eulerAngles.y + steerAmount, 
                transform.eulerAngles.z
            );
            transform.eulerAngles = Vector3.Lerp(transform.eulerAngles, steerDirection, 3f * Time.fixedDeltaTime);
        }
    }

    private void HandleGroundNormalRotation()
    {
        RaycastHit hit;
        bool hitGround = Physics.Raycast(transform.position, -transform.up, out hit, 0.75f);
        
        if (hitGround)
        {
            
            transform.rotation = Quaternion.Lerp(
                transform.rotation, 
                Quaternion.FromToRotation(transform.up * 2, hit.normal) * transform.rotation, 
                groundSmoothingStrength * Time.fixedDeltaTime
            );
            touchingGround = true;

        }
        else
        {
            touchingGround = false;
        }
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