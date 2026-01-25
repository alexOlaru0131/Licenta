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

    [Header("Generate boxes script")]
    [SerializeField]
    public GenerateBoxes gb;
    private GenerateBoxes gen;

    [Header("Generate track script")]
    [SerializeField]
    public GenerateTrack gt;
    private GenerateTrack genTr;

    [Header("Unity to Python")]
    public UnityToPython utp;
    public List<Vector2> points = new List<Vector2>
    {
        new Vector2(0, 0),
        new Vector2(0, 0),
        new Vector2(0, 0),
        new Vector2(0, 0),
        new Vector2(0, 0)
    };

    [Header("Car")]
    public Rigidbody rigid;
    private Transform rigidTransform;

    [Header("Wheel Colliders")]
    public WheelCollider frontRightCol, frontLeftCol, backRightCol, backLeftCol;

    [Header("Wheel Meshes")]
    public Transform frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    [Header("Parameters")]
    public float drivespeed;

    bool gotTarget1 = true, gotTarget2 = true, gotTarget3 = true, gotTarget4 = true, gotTarget5 = true;
    private float[] distances = new float[5];
    private float[] angles = new float[5];

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

    private float timerActions = 0f;
    public float timerLimit = 1f;

    private void Update()
    {
        System.Random rnd = new System.Random();
        Vector3 agentPosition = rigid.position;

        double positionOnZ = agentPosition[2];
        agentPosition = rigid.transform.localPosition;

        timerActions += Time.deltaTime;

        if (timerActions >= timerLimit)
        {
            for (int i = 0; i < 5; i++)
            {
                distances[i] = (float)System.Math.Sqrt
                            (
                                System.Math.Pow(points[i][0] - agentPosition[0], 2) +
                                System.Math.Pow(points[i][1] - agentPosition[2], 2)
                            );
                //Debug.Log(distances[i]);
                Vector2 point = new Vector2
                    (
                    points[i][0] - agentPosition[0],
                    points[i][1] - agentPosition[2]
                    );

                angles[i] = Vector2.SignedAngle(rigidTransform.forward, point) * Mathf.Deg2Rad;
            }

            RequestDecision();
            timerActions = 0;
        }
        else
        {
            RequestAction();
        }

        if (gotTarget1 && gotTarget2 && gotTarget3 && gotTarget4 && gotTarget5)
        {
            EndEpisode();
        }
    }
    void Awake()
    {

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int discrete = actions.DiscreteActions[0];
        discrete = 1000;

        switch (discrete)
        {
        case 0:
            {
                rigid.linearVelocity = new Vector3(
                                (float)(rigid.linearVelocity[0] * 0.9),
                                (float)(rigid.linearVelocity[1] * 0.9),
                                (float)(rigid.linearVelocity[2] * 0.9)
                                );
                rigid.transform.rotation = new Quaternion(rigid.transform.rotation[0],
                                                    (float)(rigid.transform.rotation[1] + 0.03),
                                                    rigid.transform.rotation[2],
                                                    rigid.transform.rotation[3]);
                break;
            }

        case 1:
        {
                rigid.linearVelocity = new Vector3(
                            (float)(rigid.linearVelocity[0] * 0.9),
                            (float)(rigid.linearVelocity[1] * 0.9),
                            (float)(rigid.linearVelocity[2] * 0.9)
                            );
                rigid.transform.rotation = new Quaternion(rigid.transform.rotation[0],
                                                    (float)(rigid.transform.rotation[1] - 0.03),
                                                    rigid.transform.rotation[2],
                                                    rigid.transform.rotation[3]);
                break;
            }

        case 2:
            {
                frontRightCol.motorTorque = (int)(drivespeed / 10 * 2);
                backRightCol.motorTorque = (int)(drivespeed / 10 * 2);
                frontLeftCol.motorTorque = (int)(drivespeed / 10 * 2);
                backLeftCol.motorTorque = (int)(drivespeed / 10 * 2);
                break;
            }

        case 3:
            {
                frontRightCol.motorTorque = (int)drivespeed / 10 * 4;
                backRightCol.motorTorque = (int)drivespeed / 10 * 4;
                frontLeftCol.motorTorque = (int)drivespeed / 10 * 4;
                backLeftCol.motorTorque = (int)drivespeed / 10 * 4;
                break;
            }

        case 4:
            {
                frontRightCol.motorTorque = (int)(drivespeed / 10 * 6);
                backRightCol.motorTorque = (int)(drivespeed / 10 * 6);
                frontLeftCol.motorTorque = (int)(drivespeed / 10 * 6);
                backLeftCol.motorTorque = (int)(drivespeed / 10 * 6);
                break;
            }

        case 5:
            {
                frontRightCol.motorTorque = (int)(drivespeed / 10 * 8);
                backRightCol.motorTorque = (int)(drivespeed / 10 * 8);
                frontLeftCol.motorTorque = (int)(drivespeed / 10 * 8);
                backLeftCol.motorTorque = (int)(drivespeed / 10 * 8);
                break;
            }

        case 6:
            {
                frontRightCol.motorTorque = drivespeed;
                backRightCol.motorTorque = drivespeed;
                frontLeftCol.motorTorque = drivespeed;
                backLeftCol.motorTorque = drivespeed;
                break;
            }
        }

        frontRightCol.GetWorldPose(out Vector3 pos1, out Quaternion rot1);
        frontRightWheel.transform.position = pos1;
        frontRightWheel.transform.rotation = rot1;

        frontLeftCol.GetWorldPose(out Vector3 pos2, out Quaternion rot2);
        frontLeftWheel.transform.position = pos2;
        frontLeftWheel.transform.rotation = rot2;

        backRightCol.GetWorldPose(out Vector3 pos3, out Quaternion rot3);
        backRightWheel.transform.position = pos3;
        backRightWheel.transform.rotation = rot3;

        backLeftCol.GetWorldPose(out Vector3 pos4, out Quaternion rot4);
        backLeftWheel.transform.position = pos4;
        backLeftWheel.transform.rotation = rot4;
        
        // State state = new State
        // {
        //     pos1x = target1Position[0],
        //     pos1z = target1Position[2],
        //     pos2x = target2Position[0],
        //     pos2z = target2Position[2],
        //     pos3x = target3Position[0],
        //     pos3z = target3Position[2],
        //     pos4x = target4Position[0],
        //     pos4z = target4Position[2],
        //     pos5x = target5Position[0],
        //     pos5z = target5Position[2],
        //     distance1 = distances[0],
        //     distance2 = distances[1],
        //     distance3 = distances[2],
        //     distance4 = distances[3],
        //     distance5 = distances[4],
        //     angle1 = angles[0],
        //     angle2 = angles[1],
        //     angle3 = angles[2],
        //     angle4 = angles[3],
        //     angle5 = angles[4],
        //     got1 = gotTarget1,
        //     got2 = gotTarget2,
        //     got3 = gotTarget3,
        //     got4 = gotTarget4,
        //     got5 = gotTarget5,
        //     staticFrictionCoef = staticFrictionCoef,
        //     dynamicFrictionCoef = dynamicFrictionCoef,
        //     robotX = rigid.transform.position[0],
        //     robotZ = rigid.transform.position[2],
        //     robotRotation = rigid.rotation.eulerAngles.y * Mathf.Deg2Rad,
        // };

        // mpc.MPC_func(state, discrete);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        foreach (Vector2 point in points)
        {
            sensor.AddObservation(point[0]);
            sensor.AddObservation(point[1]);
            //Debug.Log(pos[0] + " " + pos[2]);
        }

        foreach (var distance in distances)
        {
            sensor.AddObservation(distance);
        }

        foreach (var angle in angles)
        {
            sensor.AddObservation(angle);
        }

        sensor.AddObservation(gotTarget1);
        sensor.AddObservation(gotTarget2);
        sensor.AddObservation(gotTarget3);
        sensor.AddObservation(gotTarget4);
        sensor.AddObservation(gotTarget5);
        sensor.AddObservation(staticFrictionCoef);
        sensor.AddObservation(dynamicFrictionCoef);

        //Debug.Log(sensor.ObservationSize());
    }

    public override void OnEpisodeBegin()
    {
        System.Random rnd = new System.Random();
        Vector3 floorUpperLimits = floor.bounds.max;
        Vector3 floorLowerLimits = floor.bounds.min;
        Vector3 floorPosition = floor.transform.position;
        lowerCollider.name = "Lower collider";
        upperCollider.name = "Upper collider";
        leftCollider.name = "Left collider";
        rightCollider.name = "Right collider";
        rigidTransform = rigid.transform;

        mpc.mpcLine.positionCount = 0;

        SetReward(0f);

        rigid.transform.position = new Vector3(
            floorPosition[0], floorUpperLimits[1] + 1, floorPosition[2] - 25
            );
        rigid.transform.rotation = Quaternion.identity;
        rigid.linearVelocity = Vector3.zero;

        if (gotTarget1 && gotTarget2 && gotTarget3 && gotTarget4 && gotTarget5)
        {
            staticFrictionCoef = (float)rnd.NextDouble() / 2;
            dynamicFrictionCoef = (float)rnd.NextDouble() / 2;

            floorMaterial.dynamicFriction = dynamicFrictionCoef;
            floorMaterial.staticFriction = staticFrictionCoef;

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
        }


        genTr = gt.GetComponent<GenerateTrack>();
        if (genTr == null)
            genTr = gt.AddComponent<GenerateTrack>();
        genTr.Init(floor, rigid.transform.position);

        gen = gb.GetComponent<GenerateBoxes>();
        if (gen == null)
            gen = gb.AddComponent<GenerateBoxes>();
        gen.Init(floor, genTr.pathPoints, rigid);

        if(utp.points != null && utp.points.Count > 0)
            points = utp.points;

        genTr.GeneratePathPoints();
        gen.GenerateBoxesFcn();

        // --------------------- RESETS FOR TARGET REACHED STATE
        gotTarget1 = false;
        gotTarget2 = false;
        gotTarget3 = false;
        gotTarget4 = false;
        gotTarget5 = false;

        mpc.drawnMPC1 = false;
        mpc.drawnMPC2 = false;
        mpc.drawnMPC3 = false;
        mpc.drawnMPC4 = false;
        mpc.drawnMPC5 = false;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // for user control
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

            case ("Target1Collider"):
                AddReward(1f);
                gotTarget1 = true;
                break;

            case ("Target2Collider"):
                AddReward(1f);
                gotTarget2 = true;
                break;

            case ("Target3Collider"):
                AddReward(1f);
                gotTarget3 = true;
                break;

            case ("Target4Collider"):
                AddReward(1f);
                gotTarget4 = true;
                break;

            case ("Target5Collider"):
                AddReward(1f);
                gotTarget5 = true;
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
