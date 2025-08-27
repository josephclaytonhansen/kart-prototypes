using UnityEngine;
using Unity.Cinemachine;
using System.Collections.Generic;

public class KartManager : MonoBehaviour{

    public KartApex[] karts;
    public KartGameSettings kartGameSettings;

    public int totalLaps;

    public enum KartState
    {
        Preview,
        Waiting,
        Racing,
        Finished,
        Respawning,
    }

    public bool IsAnyFinished()
    {
        foreach (var state in kartStates.Values)
        {
            if (state == KartState.Finished)
            {
                return true;
            }
        }
        return false;
    }

    public bool AllVisualsUpdated()
    {
        foreach (var updated in kartVisualsUpdated.Values)
        {
            if (!updated)
            {
                return false;
            }
        }
        return true;
    }

    public bool AllControllersAssigned()
    {
        foreach (var assigned in controllerAssigned.Values)
        {
            if (!assigned)
            {
                return false;
            }
        }
        return true;
    }

    public bool IsRaceComplete(KartApex kartApex)
    {
        return kartStates[kartApex] == KartState.Finished;
    }

    public Dictionary<KartApex, KartState> kartStates = new Dictionary<KartApex, KartState>();
    public Dictionary<KartApex, bool> kartVisualsUpdated = new Dictionary<KartApex, bool>();
    public Dictionary<KartApex, bool> controllerAssigned = new Dictionary<KartApex, bool>();
}