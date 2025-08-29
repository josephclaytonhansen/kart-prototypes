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
    [Tooltip("Distance ahead to check for collisions based on current speed")]
    public float collisionLookahead = 0.1f;
    [Tooltip("Angle threshold to determine if surface is a wall (degrees)")]
    public float wallAngleThreshold = 45f;
    [Tooltip("The width of the predictive raycast sweep to detect glancing blows.")]
    public float predictiveRaycastWidth = 0.5f;

    [Tooltip("The speed to bounce backward at for a head-on collision.")]
    public float headOnBounceSpeed = 2.0f;

    [Tooltip("The strength of the bounce away from the wall during a glancing collision.")]
    public float glancingBounceFactor = 0.2f;

    [Tooltip("The speed reduction factor for a glancing collision (0.0 to 1.0).")]
    public float glancingSpeedLoss = 0.2f;

    [Tooltip("The distance to start the bounce transition from the wall.")]
    public float bounceSafeDistance = 0.2f;

    [Header("Jump and Fall Settings")]
    [Tooltip("Controls the upward force of a player-initiated jump.")]
    public float jumpForce = 15f;
    [Tooltip("Time the kart can be airborne on a hop before physics takes over.")]
    public float hopGracePeriod = 0.15f;
    [Tooltip("Intensity of the visual hop effect on small bumps.")]
    public float visualHopIntensity = 0.05f;
    [Tooltip("Distance ahead to check for the end of a jump ramp")]
    public float jumpRayLength = 2f;


    public enum BounceType
    {
        HeadOn,
        Glancing
    }
}