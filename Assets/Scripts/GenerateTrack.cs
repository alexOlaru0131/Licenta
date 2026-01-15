using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;

public class GenerateTrack : MonoBehaviour
{
    // public hidden members
    [HideInInspector]
    public List<Vector3> pathPoints;

    // private members
    private Vector3 startPoint;
    private BoxCollider floor;

    public void Init(BoxCollider floor, Vector3 startPoint)
    {
        this.floor = floor;
        this.startPoint = startPoint;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void GeneratePathPoints()
    {
        System.Random rnd = new System.Random();
        Vector3 floorUpperLimits = floor.bounds.max;
        Vector3 floorLowerLimits = floor.bounds.min;
        Vector3 floorPosition = floor.transform.position;

        pathPoints.Add(startPoint);

        for(int i = 0; i < 9; i++)
        {
            int targetX = rnd.Next((int)floor.bounds.min[0], (int)floor.bounds.max[0]);
            int targetZ = rnd.Next((int)floor.bounds.min[2], (int)floor.bounds.max[2]);
            pathPoints.Add(new Vector3(targetX, 0, targetZ));
        }

        for(int i = 0; i < 10; i++)
        {
            for(int k = 0 ; k < 10 ; k++)
            {
                if(k == i) continue;
                Vector3 startPoint = pathPoints[i];
                Vector3 endPoint = pathPoints[k];
                bool connectedPoints = false;

                Vector3 currentPos = startPoint;
                List<Vector2> shiftingPos = new List<Vector2>();
                shiftingPos.Add(new Vector2(1, 0));
                shiftingPos.Add(new Vector2(-1, 0));
                shiftingPos.Add(new Vector2(0, 1));
                shiftingPos.Add(new Vector2(0, -1));
                shiftingPos.Add(new Vector2(1, 1));
                shiftingPos.Add(new Vector2(1, -1));
                shiftingPos.Add(new Vector2(-1, 1));
                shiftingPos.Add(new Vector2(-1, -1));

                int index = 0;
                int count = 0;
                while (!connectedPoints)
                {
                    float minDistance = float.MaxValue;
                    for(int j = 0 ; j < shiftingPos.Count; j++)
                    {
                        currentPos = new Vector3(currentPos[0] + shiftingPos[j][0],
                                                currentPos[1], currentPos[2] + shiftingPos[j][1]
                                                );
                        float distance = (float)System.Math.Sqrt
                                (
                                    System.Math.Pow(currentPos[0] - endPoint[0], 2) +
                                    System.Math.Pow(currentPos[2] - endPoint[2], 2)
                                );
                        if(distance < minDistance)
                        {
                            index = j;
                            minDistance = distance;
                        }
                    }

                    currentPos = new Vector3(currentPos[0] + shiftingPos[index][0],
                                            currentPos[1],
                                            currentPos[2] + shiftingPos[index][1]
                                            );
                    pathPoints.Add(currentPos);
                    
                    if(minDistance < 2) break;
                    count ++;
                    if(count == 10) break;
                }
            }

        }

        // foreach(var point in pathPoints)
        //     Debug.Log(point);
    }
}
