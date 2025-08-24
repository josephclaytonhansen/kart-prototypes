using UnityEngine;

[CreateAssetMenu(fileName = "New KartData", menuName = "Kart/Kart Data")]
public class KartData : ScriptableObject
{
    [Header("Movement Stats")]
    public float maxSpeed = 10f;
    public float acceleration = 25f;
    public float deceleration = 10f;
    public float brakeForce = 20f;
    public float maxReverseSpeed = 5f;
    public float reverseAcceleration = 10f; // <-- This was the missing variable
    public float turnSpeed = 100f;
    
    [Header("Airborne & Collision")]
    public float groundCheckDistance = 1f;
    public LayerMask groundLayer;
    public float bounceStrength = 20f;
    public float jumpForce = 5f;
    [Range(0, 2f)] public float airborneSpeedFactor = 1.0f;

    [Header("Wheel Stats")]
    public float maxSteerAngle = 30f;
    public float wheelRadius = 0.3f;
}