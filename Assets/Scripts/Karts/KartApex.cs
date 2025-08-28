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
    public LayerMask iceLayer;
    public LayerMask mudLayer;
    public LayerMask sandLayer;
    public LayerMask offRoadLayer;
    public LayerMask allTerrainLayers; 

    [Header("Camera Settings")]
    public CinemachineCamera kartCamera;
    public CinemachineCamera lookbackCamera;

    [Header("Kart Components")]
    public Transform leftFrontWheel;
    public Transform rightFrontWheel;
    public Transform leftBackWheel;
    public Transform rightBackWheel;
    public KartData kartData;
    
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
        kartRigidbody.position = lastPosition;
        kartRigidbody.linearVelocity = Vector3.zero;
        kartRigidbody.angularVelocity = Vector3.zero;
        frozen = false;
    }
}