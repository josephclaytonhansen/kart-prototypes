using UnityEngine;
using Unity.Cinemachine;
using System.Collections.Generic;

public class KartApex : MonoBehaviour{
    [Header("Controller")]
    public bool playerControlled;
    public CPUKart cpuKart;
    public KartInput kartInput;
    public bool controllerActive;

    [Header("Layers")]
    public LayerMask deathLayer;
    
    [Header("Camera Settings")]
    public CinemachineCamera kartCamera;
    public CinemachineCamera lookbackCamera;

    [Header("Kart Components")]
    public Transform leftFrontWheel;
    public Transform rightFrontWheel;
    public Transform leftBackWheel;
    public Transform rightBackWheel;
    public KartData kartData;

    [Header("Visual Components")]
    public GameObject BL_particleSystem;
    public GameObject BR_particleSystem;
    public Rigidbody kartRigidbody;
    public Transform kartVisualsRoot;
    public BoxCollider kartCollider;

    [Header("Player Settings")]
    public bool autoDrift = false;

    [Header("Helpers")]
    public KartMeshUpdate kartMeshUpdate;
    public KartGameSettings kartGameSettings;

    [Header("Suspension Visuals")]
    public Transform chassisVisuals;

    [Header("Race Data")]
    public bool frozen;
    public int lapCount;
    public float[] lapTimes;
    public int trackPlacement;
    public int desiredPlacement;
    public Vector3 respawnPoint;
    public int startingPlacement;

    public void RecoverFromDeath(Vector3 lastPosition)
    {
        // IMPROVED: Ensure upright orientation on respawn
        kartRigidbody.position = lastPosition;
        kartRigidbody.linearVelocity = Vector3.zero;
        kartRigidbody.angularVelocity = Vector3.zero;
        
        // Force upright orientation to prevent spawning sideways/upside down
        Vector3 currentForward = transform.forward;
        Vector3 correctedForward = Vector3.ProjectOnPlane(currentForward, Vector3.up).normalized;
        if (correctedForward.magnitude < 0.1f)
        {
            correctedForward = Vector3.forward; // Fallback direction
        }
        transform.rotation = Quaternion.LookRotation(correctedForward, Vector3.up);
        
        frozen = false;
        Debug.Log($"Recovered from death at {lastPosition} with upright orientation");
    }
}