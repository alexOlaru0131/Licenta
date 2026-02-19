using System.Buffers.Text;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEditor.ShaderGraph;
using UnityEngine;
using UnityEngine.Splines;
using static UnityEditor.ShaderGraph.Internal.KeywordDependentCollection;
using static UnityEngine.GraphicsBuffer;

public class AgentScript : Agent
{
    // scripts
    [Header("MPC Script")]
    public MPC mpc;

    [Header("ToF Script")]
    public TimeOfFlightCamera tof;

    [Header("Generate boxes script")]
    [SerializeField]
    public GenerateBoxes gb;
    private GenerateBoxes gen;

    [Header("Generate track script")]
    [SerializeField]
    public GenerateTrack gt;
    private GenerateTrack genTr;

    [Header("Car")]
    public Rigidbody rigid;
    private Transform rigidTransform;

    [Header("Wheel Colliders")]
    public WheelCollider frontRightCol, frontLeftCol, backRightCol, backLeftCol;

    [Header("Wheel Meshes")]
    public Transform frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    [Header("Floor")]
    public BoxCollider floor;
    public PhysicsMaterial floorMaterial;
    private float staticFrictionCoef;
    private float dynamicFrictionCoef;

    [Header("Upper and lower collider")]
    [SerializeField] public Collider lowerCollider;
    [SerializeField] public Collider upperCollider;
    [SerializeField] public Collider leftCollider;
    [SerializeField] public Collider rightCollider;

    [Header("Materials")]
    public Material wallMaterial;

    [Header("Target")]
    public GameObject targetBox;

    private float timerActions = 0f;
    public float timerLimit;

    private float[] distance = new float[2];
    private float angle;
    private bool gotTarget = true;
    private int numberOfSteps = 0;
    private void FixedUpdate()
    {
        System.Random rnd = new System.Random();
        Vector3 agentPosition = rigid.position;

        double positionOnZ = agentPosition[2];
        agentPosition = rigid.transform.localPosition;

        timerActions += Time.fixedDeltaTime;

        if (timerActions >= timerLimit)
        {
            distance[1] = distance[0];
            distance[0] =  Vector3.Distance(rigid.transform.position, target);
            // Debug.Log(distance[0]);
            AddReward((distance[1] - distance[0]) * 0.5f);

            Vector3 dirToTarget = (target - rigid.transform.position).normalized;
            float alignment = Vector3.Dot(rigidTransform.forward, dirToTarget);
            angle = Vector3.SignedAngle(
                rigidTransform.forward,
                dirToTarget,
                Vector3.up
            ) * Mathf.Deg2Rad;

            AddReward(alignment * 0.01f);

            RequestDecision();
            timerActions = 0;

            if(distance[0] < 0.5f)
            {
                AddReward(5f);
                gotTarget = true;
                EndEpisode();
            }

            if(rigid.transform.position[1] < floor.transform.position[1] - 1) EndEpisode();

            numberOfSteps += 1;
        }
        else
        {
            RequestAction();
        }
    }
    void Awake()
    {

    }

    public void Start()
    {
        
    }

    // [Header("Motor torque")]

    // public float motorForce;
    // public float maxSteerAngle;

    public override void OnActionReceived(ActionBuffers actions)
    {
        float motorForce = 1;
        float maxSteerAngle = 45;

        // float steerInput = actions.ContinuousActions[0];
        // float throttleInput = actions.ContinuousActions[1];

        int dc = actions.DiscreteActions[0];
        float throttleInput;
        switch (dc)
        {
            case 0: throttleInput = 0; break;
            case 1: throttleInput = 0.4f * motorForce; break;
            case 2: throttleInput = 0.6f * motorForce; break;
            case 3: throttleInput = 0.8f * motorForce; break;
            case 4: throttleInput = motorForce; break;
            default: throttleInput = 0; break;
        }

        int action = actions.DiscreteActions[1];
        float steerInput;
        switch (action)
        {
            case 0: steerInput = 0; break;
            case 1: steerInput = -1; break;
            case 2: steerInput = 1; break;
            case 3:
                {
                    throttleInput *= -1;
                    steerInput = 0;
                    break;
                }
            case 4:
                {
                    throttleInput *= -1;
                    steerInput = -1;
                    break;
                }
            case 5:
                {
                    throttleInput *= -1;
                    steerInput = 1;
                    break;
                }
            default: {
                steerInput = 0;
                break;
                }
        }

        frontLeftCol.steerAngle = maxSteerAngle * steerInput;
        frontRightCol.steerAngle = maxSteerAngle * steerInput;
        
        backLeftCol.motorTorque = motorForce * throttleInput;
        backRightCol.motorTorque = motorForce * throttleInput;

        frontRightCol.GetWorldPose(out Vector3 pos1, out Quaternion rot1);
        frontRightWheel.transform.position = pos1;

        frontLeftCol.GetWorldPose(out Vector3 pos2, out Quaternion rot2);
        frontLeftWheel.transform.position = pos2;

        backRightCol.GetWorldPose(out Vector3 pos3, out Quaternion rot3);
        backRightWheel.transform.position = pos3;

        backLeftCol.GetWorldPose(out Vector3 pos4, out Quaternion rot4);
        backLeftWheel.transform.position = pos4;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation((target[0] - rigid.transform.position[0])/10f);
        sensor.AddObservation((target[2] - rigid.transform.position[2])/10f);
        sensor.AddObservation(distance[0]/10f);
        sensor.AddObservation(angle);

        float maxRange = 4.0f;
        for(int j = 0 ;j < 48; j++)
        {

            float dist = Mathf.Clamp01(tof.distances[18,j] / maxRange);
            if(tof.distances[18,j] == 0) sensor.AddObservation(1);
            else sensor.AddObservation(dist);
        }
    }


    [HideInInspector]
    public Vector3 target;
    Vector3 startPosition;
    public override void OnEpisodeBegin()
    {
        System.Random rnd = new System.Random();
        // floor.transform.localScale = new Vector3(numberOfSteps / 100000 + 5, 0.1f, numberOfSteps / 100000 + 5);
        Vector3 floorUpperLimits = floor.bounds.max;
        Vector3 floorLowerLimits = floor.bounds.min;
        Vector3 floorPosition = floor.transform.position;
        
        lowerCollider.name = "Lower collider";
        upperCollider.name = "Upper collider";
        leftCollider.name = "Left collider";
        rightCollider.name = "Right collider";
        rigidTransform = rigid.transform;

        genTr = gt.GetComponent<GenerateTrack>();
        if (genTr == null)
            genTr = gt.AddComponent<GenerateTrack>();
        genTr.Init(floor, rigid.transform.position);

        gen = gb.GetComponent<GenerateBoxes>();
        if (gen == null)
            gen = gb.AddComponent<GenerateBoxes>();
        gen.Init(floor, genTr.pathPoints, rigid);

        SetReward(0f);

        if (gotTarget){

            floor.material = floorMaterial;

            foreach (var box in gb.wallBoxList)
            {
                Destroy(box);
            }
            gb.wallBoxList.Clear();

            foreach(var col in gb.wallBoxColList)
            {
                Destroy(col);
            }
            gb.wallBoxColList.Clear();

            genTr.pathPoints.Clear();

            genTr.GeneratePathPoints(numberOfSteps / 100000 + 2);
            // gen.GenerateBoxesFcn();
            target = genTr.TargetCordinates(genTr.pathPoints);
            target[1] = floorPosition[1];
            targetBox.transform.position = target;
            // Debug.Log(target);

            gotTarget = false;

            startPosition = genTr.StartCordinates(genTr.pathPoints);
            startPosition[1] = floorPosition[1] + 0.5f;
            // Debug.Log(startPosition);
            // Debug.Log(Vector3.Distance(rigid.transform.position, target));
        }

        rigid.transform.position = startPosition;
        rigid.transform.rotation = Quaternion.LookRotation(target - startPosition);
        rigid.linearVelocity = Vector3.zero;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;

        continuousActions[0] = Input.GetAxisRaw("Horizontal");
        continuousActions[1] = Input.GetAxisRaw("Vertical");
    }


    public void OnTriggerEnter(Collider other)
    {
        string hitName = other.name;
        switch(hitName)
        {
            case ("wallbox"):
                floor.GetComponent<Renderer>().material.color = Color.red;
                EndEpisode();
                break;

            case ("Lower collider"):
                floor.GetComponent<Renderer>().material.color = Color.red;
                EndEpisode();
                break;

            case ("Upper collider"):
                floor.GetComponent<Renderer>().material.color = Color.red;
                EndEpisode();
                break;

            case ("Left collider"):
                floor.GetComponent<Renderer>().material.color = Color.red;
                EndEpisode();
                break;

            case ("Right collider"):
                floor.GetComponent<Renderer>().material.color = Color.red;
                EndEpisode();
                break;
        }
    }

}
