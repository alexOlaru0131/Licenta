using UnityEngine;

public struct State
{
    public float pos1x, pos1z, pos2x, pos2z, pos3x, pos3z, pos4x, pos4z, pos5x, pos5z;
    public float distance1, distance2, distance3, distance4, distance5;
    public float angle1, angle2, angle3, angle4, angle5;
    public bool got1, got2, got3, got4, got5;
    public float staticFrictionCoef, dynamicFrictionCoef;
    public float robotX, robotZ, robotRotation;
}
