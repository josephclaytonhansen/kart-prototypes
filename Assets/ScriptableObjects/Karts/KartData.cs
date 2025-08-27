using UnityEngine;

// This attribute allows you to create a new KartData asset in the Unity project
[CreateAssetMenu(fileName = "New KartData", menuName = "Kart/Kart Data")]
public class KartData : ScriptableObject
{
    [Header("Movement Stats")]
    // The maximum forward speed the kart can reach.
    // Higher values make the kart faster.
    public float maxSpeed = 15f;

    // The weight of the kart, which affects collisions and slope slowdown
    public float weight = 200f;
    
    // The rate at which the kart accelerates to maxSpeed.
    // Higher values make acceleration faster and more snappy.
    public float acceleration = 15f;
    
    // The rate at which the kart naturally slows down when no input is given.
    // Lower values result in a longer "coasting" period.
    public float deceleration = 2.5f;
    
    // The rate at which the kart slows down when the brake is applied.
    // Higher values result in a more powerful, abrupt brake.
    public float brakeForce = 15f;
    
    // The maximum speed the kart can reach when reversing.
    // Higher values make reversing faster.
    public float maxReverseSpeed = 5f;
    
    // The rate at which the kart accelerates in reverse.
    // Higher values make reversing acceleration faster.
    public float reverseAcceleration = 15f;
    
    // The speed at which the kart turns.
    // Higher values make the turning more responsive.
    public float turnSpeed = 50f;

    [Tooltip("Steering will be this percentage of normal at max speed.")]
    [Range(0, 1)]
    public float steeringDampenAtMaxSpeed = 0.5f;

    [Header("Airborne & Collision")]
    // The distance used by the raycast to check if the kart is on the ground.
    // Adjust this based on your kart's size.
    public float groundCheckDistance = 0.5f;
    
    // The LayerMask used to identify what the kart can drive on.
    public LayerMask groundLayer;
    
    // The outward force applied when the kart collides with a steep object.
    // Higher values result in a more powerful bounce.
    public float bounceStrength = 20f;
    
    // The initial upward force applied when the kart leaves the ground.
    // Higher values make the kart jump higher.
    public float jumpForce = 10f;
    
    // A multiplier for the kart's speed when it's in the air.
    // Values > 1 make the kart faster in the air; < 1 make it slower.
    public float airborneSpeedFactor = 1f;

    // The vertical distance the kart hovers above the ground when grounded.
    // Use this to adjust the kart's visual height relative to the terrain.
    public float groundContactOffset = 0.2f;

    [Header("Wheel Stats")]
    // The maximum angle the front wheels can turn when steering.
    // Higher values allow for tighter turns.
    public float maxSteerAngle = 30f;
    
    // The radius of the wheels, used to calculate how fast they should rotate.
    // Must match the radius of your wheel mesh.
    public float wheelRadius = 0.5f;

    [Header("Wall Collision")]
    [Tooltip("How much speed is lost on a wall collision (1 = no loss, 0 = full stop).")]
    [Range(0, 1)]
    public float speedDampenOnWallCollision = 0.8f;
    
    [Tooltip("The strength of the smooth rotation bounce off a wall.")]
    [Range(0, 1)]
    public float wallBounceStrength = 0.5f;

    [Header("Visual Data")]
    public Mesh chassisMesh;
    public Texture2D chassisTexture;
    public Vector3 chassisLocalOffset;

    public Mesh frontLeftWheelMesh;
    public Texture2D frontLeftWheelTexture;
    public Vector3 frontLeftWheelLocalOffset;

    public Mesh frontRightWheelMesh;
    public Texture2D frontRightWheelTexture;
    public Vector3 frontRightWheelLocalOffset;

    public Mesh backLeftWheelMesh;
    public Texture2D backLeftWheelTexture;
    public Vector3 backLeftWheelLocalOffset;

    public Mesh backRightWheelMesh;
    public Texture2D backRightWheelTexture;
    public Vector3 backRightWheelLocalOffset;

    public float maxChassisTiltAngle = 10f;
    public float tiltDampening = 5f;
    public float yJitterThreshold = 0.01f;
}

