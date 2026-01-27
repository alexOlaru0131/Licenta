using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

public class MPC : MonoBehaviour
{
    [Header("Agent Script")]
    public AgentScript agentScript;

    [Header("mpc Path")]
    public LineRenderer mpcLine;
    private List<Vector3> mpcTrajectory = new List<Vector3>();

    [HideInInspector]
    public bool drawnMPC1 = false, drawnMPC2 = false, drawnMPC3 = false, drawnMPC4 = false, drawnMPC5 = false;

    [HideInInspector]
    public bool collided = false;

    private float ComputeTaskCost(State state)
    {
        float cost = 0f;
        if (!state.got1) cost += state.distance1;
        if (!state.got2) cost += state.distance2;
        if (!state.got3) cost += state.distance3;
        if (!state.got4) cost += state.distance4;
        if (!state.got5) cost += state.distance5;

        return cost;
    }

    private bool CheckTaskValid(State state)
    {
        foreach(Vector3 position in agentScript.gb.wallBoxPositions)
        {
            if (state.robotX >= position[0] &&
                state.robotX <= position[0] + 3 &&
                state.robotZ >= position[2] &&
                state.robotZ <= position[2] + 3
                ) return false;
        }

        return true;
    }

    public void MPC_func(State state, int discrete)
    {
        int samples = 10;

        int horizon = 12;
        if (state.got1 == true)
        {
            horizon = 10;
        }
        else if (state.got2 == true)
        {
            horizon = 8;
        }
        else if (state.got3 == true)
        {
            horizon = 6;
        }
        else if (state.got4 == true)
        {
            horizon = 4;
        }
        else if (state.got5 == true)
        {
            horizon = 2;
        }

        float bestCost = float.MaxValue;
        List<Vector3> bestPath = new List<Vector3>();

        for (int sample = 0; sample < samples; sample++)
        {
            int[] sequence = SampleActionSequence(horizon, discrete);
            State s = state;
            float totalCost = 0f;

            List<Vector3> mpcPath = new List<Vector3>();
            mpcPath.Add(new Vector3(s.robotX, agentScript.rigid.transform.position[1], s.robotZ));
            for (int i = 0; i < horizon; i++)
            {
                s = SimulateStep(s, sequence[i]);

                float taskCost = ComputeTaskCost(s);
                totalCost += taskCost;
                mpcPath.Add(new Vector3(s.robotX, agentScript.rigid.transform.position[1], s.robotZ));

                if (CheckTaskValid(s) == false) totalCost += 1000f;

            }

            if (totalCost < bestCost)
            {
                bestCost = totalCost;
                bestPath = mpcPath;
            }
        }

        if (bestPath != null)
        {
            mpcTrajectory = bestPath;
        }
    }

    private State SimulateStep(State state, int action)
    {
        State nextState = state;
        Vector3 robotPosition = agentScript.rigid.transform.position;

        switch (action)
        {
            case 0:
                {
                    nextState.robotRotation += (float)0.3;
                    break;
                }

            case 1:
                {
                    nextState.robotRotation -= (float)0.3;
                    break;
                }

            case 2:
                {
                    int speed = 1;
                    Vector2 vector = new Vector2
                        (
                        Mathf.Sin(nextState.robotRotation),
                        Mathf.Cos(nextState.robotRotation)
                        );
                    nextState.robotX += vector[0] * speed;
                    nextState.robotZ += vector[1] * speed;
                    break;
                }

            case 3:
                {
                    int speed = 2;
                    Vector2 vector = new Vector2
                        (
                        Mathf.Sin(nextState.robotRotation),
                        Mathf.Cos(nextState.robotRotation)
                        );
                    nextState.robotX += vector[0] * speed;
                    nextState.robotZ += vector[1] * speed;
                    break;
                }

            case 4:
                {
                    int speed = 3;
                    Vector2 vector = new Vector2
                        (
                        Mathf.Sin(nextState.robotRotation),
                        Mathf.Cos(nextState.robotRotation)
                        );
                    nextState.robotX += vector[0] * speed;
                    nextState.robotZ += vector[1] * speed;
                    break;
                }

            case 5:
                {
                    int speed = 4;
                    Vector2 vector = new Vector2
                        (
                        Mathf.Sin(nextState.robotRotation),
                        Mathf.Cos(nextState.robotRotation)
                        );
                    nextState.robotX += vector[0] * speed;
                    nextState.robotZ += vector[1] * speed;
                    break;
                }

            case 6:
                {
                    int speed = 5;
                    Vector2 vector = new Vector2
                        (
                        Mathf.Sin(nextState.robotRotation),
                        Mathf.Cos(nextState.robotRotation)
                        );
                    nextState.robotX += vector[0] * speed;
                    nextState.robotZ += vector[1] * speed;
                    break;
                }
        }

        nextState.distance1 = (float)System.Math.Sqrt
                    (
                        System.Math.Pow(nextState.pos1x - nextState.robotX, 2) +
                        System.Math.Pow(nextState.pos1z - nextState.robotZ, 2)
                    ) * 4 / 120;
        Vector2 point = new Vector2
            (
            nextState.pos1x - nextState.robotX,
            nextState.pos1z - nextState.robotZ
            ) * 4 / 120;
        Vector2 forward = new Vector2
                                    (
                                    Mathf.Sin(nextState.robotRotation),
                                    Mathf.Cos(nextState.robotRotation)
                                    );

        nextState.angle1 = Vector2.SignedAngle(forward, point) * Mathf.Deg2Rad;

        nextState.distance2 = (float)System.Math.Sqrt
                    (
                        System.Math.Pow(nextState.pos2x - nextState.robotX, 2) +
                        System.Math.Pow(nextState.pos2z - nextState.robotZ, 2)
                    ) * 4 / 120;
        point = new Vector2
            (
            nextState.pos2x - nextState.robotX,
            nextState.pos2z - nextState.robotZ
            ) * 4 / 120;
        nextState.angle2 = Vector2.SignedAngle(forward, point) * Mathf.Deg2Rad;

        nextState.distance3 = (float)System.Math.Sqrt
                    (
                        System.Math.Pow(nextState.pos3x - nextState.robotX, 2) +
                        System.Math.Pow(nextState.pos3z - nextState.robotZ, 2)
                    ) * 4 / 120;
        point = new Vector2
            (
            nextState.pos3x - nextState.robotX,
            nextState.pos3z - nextState.robotZ
            ) * 4 / 120;
        nextState.angle3 = Vector2.SignedAngle(forward, point) * Mathf.Deg2Rad;

        nextState.distance4 = (float)System.Math.Sqrt
                    (
                        System.Math.Pow(nextState.pos4x - nextState.robotX, 2) +
                        System.Math.Pow(nextState.pos4z - nextState.robotZ, 2)
                    ) * 4 / 120;
        point = new Vector2
            (
            nextState.pos4x - nextState.robotX,
            nextState.pos4z - nextState.robotZ
            ) * 4 / 120;
        nextState.angle4 = Vector2.SignedAngle(forward, point) * Mathf.Deg2Rad;

        nextState.distance5 = (float)System.Math.Sqrt
                    (
                        System.Math.Pow(nextState.pos5x - nextState.robotX, 2) +
                        System.Math.Pow(nextState.pos5z - nextState.robotZ, 2)
                    ) * 4 / 120;
        point = new Vector2
            (
            nextState.pos5x - nextState.robotX,
            nextState.pos5z - nextState.robotZ
            ) * 4 / 120;
        nextState.angle5 = Vector2.SignedAngle(forward, point) * Mathf.Deg2Rad;

        return nextState;

    }

    private int[] SampleActionSequence(int horizon, int discrete)
    {
        int[] sequence = new int[horizon];
        for (int i = 0; i < horizon; i++)
        {
            if (Random.value < 0.5f)
            {
                sequence[i] = discrete;
            }
            else
            {
                sequence[i] = Random.Range(0, 7);
            }
        }

        return sequence;
    }

    void Update()
    {
        
    }

    public void OnTriggerEnter(Collider other)
    {
        string hitName = other.name;
        switch(hitName)
        {
            case ("Target1Collider"):
                if (drawnMPC1 == false)
                {
                    drawnMPC1 = true;
                    mpcLine.positionCount = mpcTrajectory.Count;
                    for (int i = 0; i < mpcTrajectory.Count; i++)
                    {
                        mpcLine.SetPosition(i, mpcTrajectory[i]);
                    }
                }
                break;

            case ("Target2Collider"):
                if (drawnMPC2 == false)
                {
                    drawnMPC2 = true;
                    mpcLine.positionCount = mpcTrajectory.Count;
                    for (int i = 0; i < mpcTrajectory.Count; i++)
                    {
                        mpcLine.SetPosition(i, mpcTrajectory[i]);
                    }
                }
                break;

            case ("Target3Collider"):
                if (drawnMPC3 == false)
                {
                    drawnMPC3 = true;
                    mpcLine.positionCount = mpcTrajectory.Count;
                    for (int i = 0; i < mpcTrajectory.Count; i++)
                    {
                        mpcLine.SetPosition(i, mpcTrajectory[i]);
                    }
                }
                break;

            case ("Target4Collider"):
                if (drawnMPC4 == false)
                {
                    drawnMPC4 = true;
                    mpcLine.positionCount = mpcTrajectory.Count;
                    for (int i = 0; i < mpcTrajectory.Count; i++)
                    {
                        mpcLine.SetPosition(i, mpcTrajectory[i]);
                    }
                }
                break;

            case ("Target5Collider"):
                if (drawnMPC5 == false)
                {
                    drawnMPC5 = true;
                    mpcLine.positionCount = mpcTrajectory.Count;
                    for (int i = 0; i < mpcTrajectory.Count; i++)
                    {
                        mpcLine.SetPosition(i, mpcTrajectory[i]);
                    }
                }
                break;
        }
    }
}
