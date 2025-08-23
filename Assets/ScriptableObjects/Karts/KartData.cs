using UnityEngine;

[CreateAssetMenu(fileName = "KartData", menuName = "Kart/KartData", order = 1)]
public class KartData : ScriptableObject
{
    [Header("Movement")]
    public float acceleration = 25f;
    public float deceleration = 15f;
    public float brakeForce = 30f;
    public float reverseAcceleration = 15f;
    public float maxSpeed = 35f;
    public float maxReverseSpeed = 15f;
    public float gravity = 20f;

    [Header("Steering")]
    public float turnSpeed = 150f;
    public float maxSteerAngle = 45f;
    public float steeringWheelMaxRotation = 360f;

    [Header("Physics")]
    public float groundAlignmentSpeed = 10f;
    public float groundCheckDistance = 1.2f;
    public LayerMask groundLayer;

    [Header("Visuals")]
    public float wheelRadius = 0.5f;
}