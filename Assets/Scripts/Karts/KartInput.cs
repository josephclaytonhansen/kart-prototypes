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
    public enum KartState
    {
        Grounded,
        Airborne,
        Jumping,
        PerformingTrick,
        Frozen
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
    private TerrainType previousTerrainType;

    [Header("Input Actions")]
    public InputAction steerAction;
    public InputAction accelerateAction;
    public InputAction brakeAction;
    public InputAction trickAction;
    public InputAction driftAction;
    public InputAction lookBackAction;

    // Internal state variables
    protected float tailwindTimer = 0f;
    protected float boostTimer = 0f;
    protected bool kartLongBoosted = false;
    protected bool kartShortBoosted = false;
    protected float currentMaxSpeed;
    protected float currentAcceleration;
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

    protected bool isPerformingArcJump = false;
    protected float arcJumpTimer = 0f;
    protected Vector3 arcJumpStartPosition;
    protected Vector3 arcJumpTargetPosition;
    protected Vector3 arcJumpPeakPosition;
    protected Quaternion arcJumpStartRotation;
    protected Quaternion arcJumpTargetRotation;
    
    protected bool trickPerformedOnRamp = false;

    protected Dictionary<TerrainType, int> activeTerrainZones = new Dictionary<TerrainType, int>();

    protected KartState currentState = KartState.Grounded;

    protected Vector3 lastGroundedPosition;
    
    protected float terrainChangeSmoothing = 0f;
    protected float stateChangeTimer = 0f;
    protected const float STATE_CHANGE_COOLDOWN = 0.1f;
    protected const float TERRAIN_CHANGE_SMOOTHING_DURATION = 0.2f;

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
}