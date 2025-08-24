using UnityEngine;

public class KartMeshUpdate : MonoBehaviour
{
    [Header("Hierarchy References")]
    [Tooltip("The parent transform containing the chassis mesh renderer.")]
    public Transform chassisPivot;
    [Tooltip("The pivot transforms for each of the four wheels.")]
    public Transform frontLeftWheelPivot;
    public Transform frontRightWheelPivot;
    public Transform backLeftWheelPivot;
    public Transform backRightWheelPivot;

    private MeshRenderer chassisMeshRenderer;
    private MeshRenderer frontLeftWheelRenderer;
    private MeshRenderer frontRightWheelRenderer;
    private MeshRenderer backLeftWheelRenderer;
    private MeshRenderer backRightWheelRenderer;
    
    void Awake()
    {
        if (chassisPivot != null) chassisMeshRenderer = chassisPivot.GetComponentInChildren<MeshRenderer>();
        if (frontLeftWheelPivot != null) frontLeftWheelRenderer = frontLeftWheelPivot.GetComponentInChildren<MeshRenderer>();
        if (frontRightWheelPivot != null) frontRightWheelRenderer = frontRightWheelPivot.GetComponentInChildren<MeshRenderer>();
        if (backLeftWheelPivot != null) backLeftWheelRenderer = backLeftWheelPivot.GetComponentInChildren<MeshRenderer>();
        if (backRightWheelPivot != null) backRightWheelRenderer = backRightWheelPivot.GetComponentInChildren<MeshRenderer>();
    }

    public void UpdateVisuals(KartData kartData)
    {
        if (chassisPivot != null)
        {
            UpdateMeshAndTexture(chassisMeshRenderer, kartData.chassisMesh, kartData.chassisTexture);
            chassisPivot.localPosition = kartData.chassisLocalOffset;
        }

        if (frontLeftWheelPivot != null)
        {
            UpdateMeshAndTexture(frontLeftWheelRenderer, kartData.frontWheelMesh, kartData.frontWheelTexture);
            frontLeftWheelPivot.localPosition = kartData.frontLeftWheelLocalOffset;
        }
        if (frontRightWheelPivot != null)
        {
            UpdateMeshAndTexture(frontRightWheelRenderer, kartData.frontWheelMesh, kartData.frontWheelTexture);
            frontRightWheelPivot.localPosition = kartData.frontRightWheelLocalOffset;
        }

        if (backLeftWheelPivot != null)
        {
            UpdateMeshAndTexture(backLeftWheelRenderer, kartData.backWheelMesh, kartData.backWheelTexture);
            backLeftWheelPivot.localPosition = kartData.backLeftWheelLocalOffset;
        }
        if (backRightWheelPivot != null)
        {
            UpdateMeshAndTexture(backRightWheelRenderer, kartData.backWheelMesh, kartData.backWheelTexture);
            backRightWheelPivot.localPosition = kartData.backRightWheelLocalOffset;
        }
    }
    
    private void UpdateMeshAndTexture(MeshRenderer renderer, Mesh mesh, Texture2D texture)
    {
        if (renderer != null)
        {
            MeshFilter filter = renderer.GetComponent<MeshFilter>();
            if (filter != null && mesh != null)
            {
                filter.mesh = mesh;
            }
            if (texture != null)
            {
                renderer.material.mainTexture = texture;
            }
        }
    }
}