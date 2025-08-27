using UnityEngine;

[CreateAssetMenu(fileName = "New KartGameSettings", menuName = "Kart/Kart Game Settings")]
public class KartGameSettings : ScriptableObject
{
    [Header("Drift Settings")]
    public float driftTimeToBlueBoost = 1.5f;
    public float driftTimeToOrangeBoost = 2.0f;

    [Header("Tailwind Settings")]
    public float tailwindTimeToBoost = 3.0f;

    [Header("Landing Assist")]
    public float landingAssistLookahead = 0.5f;
    public float landingRayLength = 5.0f;
    public float transitionDuration = 0.2f;
    [Tooltip("Extra vertical offset added during landing transition to ensure kart lands above the ground.")]
    public float landingHeightOffset = 0.1f;

    [Header("Collision Settings")]
    public LayerMask selectableLayerMask;
    public float bounceDuration = 0.25f;
    [Tooltip("Distance ahead to check for collisions based on current speed")]
    public float collisionLookahead = 0.1f;
    [Tooltip("Minimum speed deceleration on head-on collision")]
    public float minCollisionDeceleration = 0.3f;
    [Tooltip("Maximum speed deceleration on head-on collision")]
    public float maxCollisionDeceleration = 0.8f;
    [Tooltip("Minimum bounce distance")]
    public float minBounceDistance = 0.5f;
    [Tooltip("Maximum bounce distance")]
    public float maxBounceDistance = 2.0f;
    [Tooltip("Safe distance from wall surface")]
    public float wallSafeDistance = 0.2f;
    [Tooltip("Angle threshold to determine if surface is a wall (degrees)")]
    public float wallAngleThreshold = 45f;
    [Tooltip("Weight range for kart data (min-max)")]
    public Vector2 weightRange = new Vector2(50f, 500f);

    [Header("Jump and Fall Settings")]
    [Tooltip("Controls the upward force of a player-initiated jump.")]
    public float jumpForce = 15f;
    [Tooltip("Time the kart can be airborne on a hop before physics takes over.")]
    public float hopGracePeriod = 0.15f;
    [Tooltip("Intensity of the visual hop effect on small bumps.")]
    public float visualHopIntensity = 0.05f;
}