using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Events;
using System.Collections.Generic;
using System;
using Unity.Cinemachine;

[RequireComponent(typeof(Rigidbody))]
public partial class KartInput : MonoBehaviour
{
    public KartApex kartApex;
    // Define the possible states of the kart's behavior.
    public enum KartState
    {
        Grounded,
        Airborne,
        Jumping,
        PerformingTrick,
        Frozen // New state for when the kart cannot move.
    }

    public enum TerrainType{
        Mud,
        Ground,
        Ice,
        Sand,
        Death,
        OffRoad
    }

    public TerrainType terrainType;

    [Header("Input Actions")]
    public InputAction steerAction;
    public InputAction accelerateAction;
    public InputAction brakeAction;
    public InputAction trickAction;
    public InputAction driftAction;
    public InputAction lookBackAction;

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

    // New variables for last grounded position and death handling
    protected Vector3 lastGroundedPosition;

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
    [Tooltip("Invoked when the kart hits the death layer, sends the last non-death grounded position.")]
    public UnityEvent<Vector3> onGroundedOnDeathLayer;


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
        kartApex.kartRigidbody.isKinematic = true;
        kartApex.kartRigidbody.useGravity = false;
        targetPosition = transform.position;
        targetRotation = transform.rotation;
        wheelTransforms = new Transform[]
        {
            kartApex.leftFrontWheel, kartApex.rightFrontWheel, kartApex.leftBackWheel, kartApex.rightBackWheel
        };
    }

    void FixedUpdate()
    {
        if (kartApex.frozen)
        {
            return; // Exit FixedUpdate if the kart is frozen
        }
        
        wasGrounded = isGrounded;
        isGrounded = CheckGround();

        // Check if the kart is on a death layer
        bool onDeathLayer = CheckGroundOnLayer(kartApex.deathLayer);
        if (onDeathLayer)
        {
            kartApex.frozen = true;
            onGroundedOnDeathLayer.Invoke(lastGroundedPosition);
            return;
        }

        if (isGrounded)
        {
            // If the kart just became grounded, store a safe respawn position.
            if (!wasGrounded)
            {
                // Start with the current position.
                lastGroundedPosition = transform.position;

                Vector3 backwardsDirection = -transform.forward;
                // Subtract a small offset to ensure we don't collide with the ground immediately.
                lastGroundedPosition = lastGroundedPosition - (backwardsDirection * currentSpeed);
                // add a small y offset so it's slightly off the ground
                lastGroundedPosition.y += 1f;
            }
            
            if (currentState != KartState.Grounded)
            {
                currentState = KartState.Grounded;
                if (airborneTimer > kartApex.kartGameSettings.hopGracePeriod)
                {
                    onLanding.Invoke();
                }
            }
            HandleGroundedMovement();
        }
        else // Kart is not on the ground
        {
            if (currentState != KartState.Jumping && currentState != KartState.PerformingTrick)
            {
                currentState = KartState.Airborne;
            }
            HandleAirborneMovement();
        }
    }

    void Update()
    {
        if (kartApex.frozen)
        {
            return; // Exit Update if the kart is frozen
        }
        
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
        if (kartApex.frozen) return;
        Debug.Log("Look Back Activated");
        if (kartApex.lookbackCamera != null && kartApex.kartCamera != null)
        {
            kartApex.lookbackCamera.Priority = 20;
            kartApex.kartCamera.Priority = 10;
        }
    }
    private void OnLookBackCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        Debug.Log("Look Back Deactivated");
        if (kartApex.lookbackCamera != null && kartApex.kartCamera != null)
        {
            kartApex.lookbackCamera.Priority = 10;
            kartApex.kartCamera.Priority = 20;
        }
    }
    private void OnBrake(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        onBrake.Invoke();
    }
    private void OnSteerCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        steerInput = 0f;
    }
    public void OnSteer(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        steerInput = context.ReadValue<float>();
        onSteer.Invoke();
    }
    public void OnDrift(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        if (currentState == KartState.Grounded && isAccelerating && steerInput != 0 && !isDrifting)
        {
            isDrifting = true;
            driftTimer = 0f;
            onDrift.Invoke();
        }
    }
    public void OnDriftCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        if (isDrifting)
        {
            isDrifting = false;
            float driftDuration = driftTimer;
            driftTimer = 0f;
            if (driftDuration >= kartApex.kartGameSettings.driftTimeToOrangeBoost && !kartApex.autoDrift)
            {
                onDriftOrangeBoost.Invoke();
            }
            else if (driftDuration >= kartApex.kartGameSettings.driftTimeToBlueBoost && !kartApex.autoDrift)
            {
                onDriftBlueBoost.Invoke();
            }
        }
    }
    private void OnAcceleratedCanceled(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        isAccelerating = false;
        onDecelerate.Invoke();
    }
    public void OnAccelerate(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        isAccelerating = true;
        onAccelerate.Invoke();
    }
    private void OnTrick(InputAction.CallbackContext context)
    {
        if (kartApex.frozen) return;
        // Only allow a trick if the kart is in the Jumping state
        if (currentState == KartState.Jumping)
        {
            currentState = KartState.PerformingTrick;
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

    void OnTriggerStay(Collider other)
    {
        if (kartApex.frozen) return;
        // Predictive Jump Takeoff
        if (isOnRamp && other.CompareTag("JumpRamp"))
        {
            Vector3 rayOrigin = transform.position + transform.forward * 0.5f;
            RaycastHit hit;
            // Check if the raycast from the front of the kart leaves the ramp
            if (!Physics.Raycast(rayOrigin, transform.forward, out hit, kartApex.kartGameSettings.jumpRayLength, kartApex.kartData.groundLayer))
            {
                HandleRampJump();
            }
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("JumpRamp"))
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