using UnityEngine;
using AustinHarris.JsonRpc;
using Unity.MLAgents;
using Unity.VisualScripting;
using System.Collections.Generic;
using System;
using Newtonsoft.Json.Linq;

class NewVector2
{
    public float x;
    public float y;
    public NewVector2(float x, float y)
    {
        this.x = x;
        this.y = y;
    }

    public Vector2 AsVector2()
    {
        return new Vector2(x, y);
    }
}

public class UnityToPython : MonoBehaviour
{
    private PointsRpc pointsRpc;
    private DistancesRpc distancesRpc;
    public object scanLock = new object();

    int width = 240;
    int height = 180;
    float minRange = 0.1f;
    float maxRange = 4.0f;
    float refreshRate = 60.0f;

    LayerMask layerMask = Physics.AllLayers;
    QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Collide;

    float noise = 0.0005f;
    float noisePerMeter = 0.002f;
    float quantStep = 0.001f;

    [HideInInspector]
    public float[,] distances;

    [SerializeField] public Camera tofCamera;
    float nextFrameTime;

    class PointsRpc : JsonRpcService
    {
        List<NewVector2> points = new List<NewVector2>(5);

        [JsonRpcMethod]
        void GetPoints(List<object> data)
        {

            List<NewVector2> result = new List<NewVector2>(data.Count);

            foreach(var item in data)
            {
                JObject obj = item as JObject;

                float x = obj["x"].Value<float>();
                float y = obj["y"].Value<float>();

                result.Add(new NewVector2(x, y));
            }

            this.points = result;
        }

        public List<NewVector2> ReturnPoints()
        {
            return points;
        }
    }

    public class DistRpc
    {
        public float[,] dist;

        public DistRpc(float[,] dist, int height, int width)
        {
            this.dist = new float[height, width];
            this.dist = dist;
        }
    }

    public class DistancesRpc : JsonRpcService
    {
        UnityToPython parent;
        int height, width;
        public float[,] distances;
        [JsonRpcMethod("SendRigid")]
        public DistRpc SendRigid()
        {
            lock (parent.scanLock)
            {
                DistRpc d = new DistRpc(distances, height, width);
                return d;
            }
        }

        public DistancesRpc(UnityToPython parent, float[,] distances, int height, int width)
        {
            this.parent = parent;
            this.height = height;
            this.width = width;
            this.distances = new float[height, width];
            this.distances = distances;
        }

    }

        // Start is called once before the first execution of Update after the MonoBehaviour is created
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

        void Start()
        {
            pointsRpc = new PointsRpc();
            distancesRpc = new DistancesRpc(this, distances, height, width);
        }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log(tofCamera);

        // if(Time.time < nextFrameTime) return;
        nextFrameTime = Time.time + (1f / refreshRate);

        var pts = pointsRpc.ReturnPoints();
        if(pts != null && pts.Count > 0)
        {
            Debug.Log(pts[0].y);
        }
        
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

                // Debug.DrawRay(ray.origin, ray.direction * maxRange, Color.red, 0.1f);

                distances[i,j] = value;
            }
        }
    }
}
