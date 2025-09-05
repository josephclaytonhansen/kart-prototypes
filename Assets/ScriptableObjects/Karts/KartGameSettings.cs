using UnityEngine;

[CreateAssetMenu(fileName = "New KartGameSettings", menuName = "Kart/Kart Game Settings")]
public class KartGameSettings : ScriptableObject
{
    [Header("Slope Settings")]
    public float slopeInfluence = 0.5f;
    public float minKartWeight = 50f;
    
    [Header("Drift Settings")]
    public float driftTimeToBlueBoost = 1.5f;
    public float driftTimeToOrangeBoost = 2.0f;

    [Header("Boost Settings")]
    public float shortBoostDuration = 1.0f;
    public float longBoostDuration = 2.0f;
    [Tooltip("IMPROVED: Configurable boost acceleration from MKW")]
    public float boostAcceleration = 100f;

    [Header("Tailwind Settings")]
    public float tailwindTimeToBoost = 3.0f;
    public float tailwindBoostMultiplier = 1.3f;

    [Header("Landing Assist")]
    public float landingAssistLookahead = 0.5f;
    public float landingRayLength = 5.0f;
    public float transitionDuration = 0.2f;
    [Tooltip("Extra vertical offset added during landing transition to ensure kart lands above the ground.")]
    public float landingHeightOffset = 0.1f;

    [Header("Collision Settings")]
    [Tooltip("IMPROVED: Distance ahead to check for collisions based on current speed (reduced to prevent over-sensitive detection)")]
    public float collisionLookahead = 0.2f;
    [Tooltip("Angle threshold to determine if surface is a wall (degrees)")]
    public float wallAngleThreshold = 45f;
    [Tooltip("The width of the predictive raycast sweep to detect glancing blows.")]
    public float predictiveRaycastWidth = 0.5f;

    [Tooltip("IMPROVED: The speed to bounce backward at for a head-on collision (increased for better control).")]
    public float headOnBounceSpeed = 4.0f;

    [Tooltip("The strength of the bounce away from the wall during a glancing collision.")]
    public float glancingBounceFactor = 0.2f;

    [Tooltip("The speed reduction factor for a glancing collision (0.0 to 1.0).")]
    public float glancingSpeedLoss = 0.2f;

    [Tooltip("IMPROVED: The distance to start the bounce transition from the wall (increased to prevent sticking).")]
    public float bounceSafeDistance = 1.5f;

    [Tooltip("IMPROVED: Minimum clearance distance for glancing collisions to prevent wall grinding penetration (reduced to be less aggressive).")]
    public float glancingClearanceDistance = 0.3f;

    [Header("Jump and Fall Settings")]
    [Tooltip("Controls the upward force of a player-initiated jump.")]
    public float jumpForce = 15f;
    [Tooltip("Time the kart can be airborne on a hop before physics takes over.")]
    public float hopGracePeriod = 0.15f;
    [Tooltip("Intensity of the visual hop effect on small bumps.")]
    public float visualHopIntensity = 0.05f;
    [Tooltip("Distance ahead to check for the end of a jump ramp")]
    public float jumpRayLength = 2f;
    
    [Header("Mario Kart Wii Jump Arc System")]
    [Tooltip("Duration of the jump arc in seconds")]
    public float jumpArcDuration = 1.2f;
    [Tooltip("Peak height of the jump arc")]
    public float jumpArcHeight = 8f;
    [Tooltip("Forward distance covered during jump")]
    public float jumpArcDistance = 15f;
    [Tooltip("How smoothly the jump transitions (higher = smoother)")]
    public AnimationCurve jumpArcCurve = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [Header("Mario Kart Wii Improvements")]
    [Tooltip("IMPROVED: Input deadzone to prevent tiny joystick movements")]
    public float inputDeadzone = 0.15f;
    [Tooltip("IMPROVED: Airborne control strength (rotation-based like MKW)")]
    public float airborneControlStrength = 0.5f;
    [Tooltip("IMPROVED: Minimum airborne time before trick can be performed")]
    public float trickMinAirborneTime = 0.2f;
    [Tooltip("IMPROVED: SphereCast ground ray origin offset")]
    public float groundRayOriginOffset = 0.05f;
    
    [Header("Particle Colors")]
    [Tooltip("IMPROVED: Modern ParticleSystem API color for blue boost")]
    public Color blueBoostColor = new Color(0f, 0.7f, 1f);
    [Tooltip("IMPROVED: Modern ParticleSystem API color for orange boost")]
    public Color orangeBoostColor = new Color(1f, 0.7f, 0f);


    public enum BounceType
    {
        HeadOn,
        Glancing
    }
}