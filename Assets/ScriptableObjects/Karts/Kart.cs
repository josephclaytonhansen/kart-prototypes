using UnityEngine;

[CreateAssetMenu(fileName = "Kart", menuName = "Scriptable Objects/Kart")]
public class KartData : ScriptableObject
{
    [Header("Steering Settings")]
    public float maxSteerAngle = 30f;
    public float minSteerAngle = -30f;
    public float steerSpeed = 5f;

    public float turnRadius = 1f;

    [Header("Speed Settings")]
    public float accelerateSpeed = 5f;
    public float decelerateSpeed = 15f;

    public float brakeSpeed = 30f;
    public float maxSpeed = 10f;

    public float weight = 1000f;
    public float offRoadMultiplier = 1f;

}
