using UnityEngine;
using Unity.MLAgents.Sensors;

public class Observations : ISensor
{
    private TimeOfFlightCamera tof;
    private string name;

    public Observations(TimeOfFlightCamera tof, string name)
    {
        this.tof = tof;
        this.name = name;
    }


    public string GetName() => name;

    public ObservationSpec GetObservationSpec()
    {
        return ObservationSpec.Visual(tof.height, tof.width, 1);
    }

    public int Write(ObservationWriter writer)
    {
        if (tof == null)
        {
            Debug.LogError("ToF is NULL!");
            return 0;
        }

        if (tof.distances == null)
        {
            Debug.LogError("Distances is NULL!");
            return 0;
        }

        for(int i = 0; i < tof.heightResized; i++)
        {
            for(int j = 0; j < tof.widthResized; j++)
            {
                writer[i, j, 0] = tof.distances[i, j];
            }
        }

        return tof.heightResized * tof.widthResized;
    }

    public byte[] GetCompressedObservation() => null;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    public void Update(){}

    public void Reset(){}
    public CompressionSpec GetCompressionSpec()
    {
        return CompressionSpec.Default();
    }
}
