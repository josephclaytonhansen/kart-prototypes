using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;
using System;
using Unity.Cinemachine;

[RequireComponent(typeof(Rigidbody))]
public partial class KartInput : MonoBehaviour
{
    // Define the possible states of the kart's behavior.
    public enum KartState
    {
        Grounded,
        Airborne,
        // Add more states here as needed, e.g., Stunned, Boosting, etc.
    }

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

    // Internal state variables
    protected float tailwindTimer = 0f;
    protected bool isOnRamp = false;
    protected bool isPerformingTrick = false;
    protected bool isJumping = false;
    protected bool isBouncing = false;
    protected float bounceTimer = 0f;
    protected Vector3 bounceStartPosition;
    protected Vector3 bounceTargetPosition;
    protected Quaternion bounceStartRotation;
    protected Quaternion bounceTargetRotation;
    protected float preCollisionSpeed = 0f;
    protected Vector3 bounceVelocity;
    protected bool isTransitioningToGrounded = false;
    protected Vector3 transitionStartPosition;
    protected Quaternion transitionStartRotation;
    protected float transitionTimer = 0f;
    protected RaycastHit landingAssistHit;
    protected float steerInput = 0f;
    protected float currentSteerAngle = 0f;
    protected float currentSpeed = 0f;
    protected float currentSlope = 0f;
    protected RaycastHit lookaheadHitInfo;
    protected bool isGrounded = false;
    protected bool wasGrounded = false;
    protected bool isDrifting = false;
    protected float driftTimer = 0f;
    protected bool isAccelerating = false;
    protected float airborneTimer = 0f;
    protected Vector3 targetPosition;
    protected Quaternion targetRotation;
    protected Vector3 averagedNormal;
    protected RaycastHit[] groundHits = new RaycastHit[4];
    protected int groundHitCount;
    protected Transform[] wheelTransforms;
    private List<InputAction> allActions;

    // State Machine
    protected KartState currentState = KartState.Grounded;

    [Header("Suspension Visuals")]
    public Transform chassisVisuals;


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
        allActions = new List<InputAction>
        {
            steerAction, accelerateAction, brakeAction, trickAction, driftAction, lookBackAction
        };
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
        wheelTransforms = new Transform[]
        {
            leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel
        };
    }

    void FixedUpdate()
    {
        wasGrounded = isGrounded;
        isGrounded = CheckGround();

        if (isGrounded)
        {
            currentState = KartState.Grounded;
            HandleGroundedMovement();
        }
        else
        {
            currentState = KartState.Airborne;
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
        else if (currentState == KartState.Grounded)
        {
            transform.position = Vector3.Lerp(transform.position, targetPosition, 25f * Time.deltaTime);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 25f * Time.deltaTime);
        }
        HandleChassisVisuals();
        HandleWheelVisuals();
    }

    // Input callback methods
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
        if (currentState == KartState.Grounded && isAccelerating && steerInput != 0 && !isDrifting)
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
        isAccelerating = false;
        onDecelerate.Invoke();
    }
    public void OnAccelerate(InputAction.CallbackContext context)
    {
        isAccelerating = true;
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

    // Trigger methods
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

    // OnEnable/OnDisable methods
    public void OnEnable()
    {
        foreach (var action in allActions)
        {
            action.Enable();
        }
    }
    public void OnDisable()
    {
        foreach (var action in allActions)
        {
            action.Disable();
        }
    }
}