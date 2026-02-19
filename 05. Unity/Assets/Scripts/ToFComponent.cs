using UnityEngine;
using Unity.MLAgents.Sensors;

public class ToFComponent : SensorComponent
{
    public TimeOfFlightCamera tof;

    public override ISensor[] CreateSensors()
    {
        if (tof == null)
            tof = GetComponent<TimeOfFlightCamera>();

        if (tof == null)
        {
            Debug.LogError("TimeOfFlightCamera NOT FOUND!");
        }

        return new ISensor[]
        {
            new Observations(tof, "ToF Sensor")
        };
    }
}
