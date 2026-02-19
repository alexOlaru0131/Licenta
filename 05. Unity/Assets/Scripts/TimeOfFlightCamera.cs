using UnityEngine;
using System.Buffers.Text;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;

public class TimeOfFlightCamera : MonoBehaviour
{
    [Header("Camera")]
    public Camera tofCamera;

    public float[,] distances;

    public int width = 48;
    public int height = 36;
    public int widthResized = 48;
    public int heightResized = 36;
    float minRange = 0.3f;
    float maxRange = 4.0f;
    float refreshRate = 60.0f;
    LayerMask layerMask = Physics.AllLayers;
    QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Collide;
    float nextFrameTime;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    
    void Start()
    {
        
    }

    void Awake()
    {
        tofCamera = GameObject.Find("Time-of-Flight").GetComponent<Camera>();
        distances = new float[height, width];

        float aspect = (float)width / height;
        tofCamera.aspect = aspect;

        float diagFovDeg = 70.0f;

        float diagRad = diagFovDeg * Mathf.Deg2Rad;
        float tanDiag = Mathf.Tan(diagRad * 0.5f);
        float tanVert = tanDiag / Mathf.Sqrt(1f + aspect * aspect);
        float vertRad = 2f * Mathf.Atan(tanVert);
        tofCamera.fieldOfView = vertRad * Mathf.Rad2Deg;
    }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log(tofCamera);

        // if(Time.time < nextFrameTime) return;
        nextFrameTime = Time.time + (1f / refreshRate);
        
        Scan();
    }

    void Scan()
    {
        var random = new System.Random();

        for (int i = 0 ; i < height ; i++)
        {
            float pixel_y = (i + 0.5f) / height;
            for (int j = 0; j < width ; j++)
            {
                float pixel_x = (j + 0.5f) / width;
                Ray ray = tofCamera.ViewportPointToRay(new Vector3(pixel_x, pixel_y, 0f));
                float value = 0f;

                if(Physics.Raycast(ray, out RaycastHit hit, maxRange, layerMask, triggerInteraction))
                {
                    float d = hit.distance;

                    if(d >= minRange && d <= maxRange) value = d;
                    double noise = random.NextDouble() / 5;
                    value += (float)noise;
                }

                // Debug.DrawRay(ray.origin, ray.direction * maxRange, UnityEngine.Color.red);

                distances[i,j] = value;
            }
        }
    }
}
