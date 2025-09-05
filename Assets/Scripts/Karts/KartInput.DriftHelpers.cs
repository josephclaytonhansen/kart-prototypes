using UnityEngine;
using System;

public partial class KartInput
{
    public void StartShortBoost()
    {
        kartShortBoosted = true;
        boostTimer = 0f;
        Debug.Log($"SHORT BOOST ACTIVATED! Duration: {kartApex.kartGameSettings.shortBoostDuration}s");
    }

    public void StartLongBoost()
    {
        kartLongBoosted = true;
        boostTimer = 0f;
        Debug.Log($"LONG BOOST ACTIVATED! Duration: {kartApex.kartGameSettings.longBoostDuration}s");
    }

    // COMPLETELY REMOVED: ValidateDriftConditions
    // Unity was caching the old method, this ensures it's gone
    
    // Simple drift system: start on button press, end on button release only
    public void ForceEndDrift()
    {
        if (isDrifting)
        {
            EndDrift("forced end");
        }
    }

    private void EndDrift(string reason)
    {
        // IMPROVED: Prevent multiple EndDrift calls within a short time
        if (!isDrifting || (Time.time - lastDriftEndTime) < 0.1f) 
        {
            Debug.Log($"EndDrift called but not drifting or too soon - reason: {reason}, isDrifting: {isDrifting}, timeSinceLastEnd: {Time.time - lastDriftEndTime:F3}s");
            return;
        }

        lastDriftEndTime = Time.time;
        float driftDuration = driftTimer;
        
        Debug.Log($"Drift ending: {reason}, duration: {driftDuration:F2}s, autoDrift: {kartApex.autoDrift}, state: {currentState}, isGrounded: {isGrounded}");
        
        isDrifting = false;
        driftTimer = 0f;

        // Turn off drift particles
        kartApex.BL_particleSystem.SetActive(false);
        kartApex.BR_particleSystem.SetActive(false);

        // Apply drift boost if long enough
        if (driftDuration >= kartApex.kartGameSettings.driftTimeToOrangeBoost && !kartApex.autoDrift)
        {
            Debug.Log($"Orange drift boost from {reason}");
            StartLongBoost(); // FIXED: Actually apply the boost
            onDriftOrangeBoost.Invoke();
        }
        else if (driftDuration >= kartApex.kartGameSettings.driftTimeToBlueBoost && !kartApex.autoDrift)
        {
            Debug.Log($"Blue drift boost from {reason}");
            StartShortBoost(); // FIXED: Actually apply the boost  
            onDriftBlueBoost.Invoke();
        }

        Debug.Log($"Drift state cleared - isDrifting: {isDrifting}, autoDrift: {kartApex.autoDrift}");
    }
}
