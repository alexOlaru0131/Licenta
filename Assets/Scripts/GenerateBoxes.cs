using System.Collections.Generic;
using UnityEngine;

public class GenerateBoxes : MonoBehaviour
{
    // public members
    [Header("Walls")]
    [SerializeField] public GameObject wallBox;

    // public hidden members
    [HideInInspector]
    public List<GameObject> wallBoxList = new List<GameObject>();
    [HideInInspector]
    public List<Collider> wallBoxColList = new List<Collider>();
    [HideInInspector]
    public List<Vector3> wallBoxPositions = new List<Vector3>();

    // private members
    private List<Vector3> targetPositions;
    private Transform wallBoxTransform;
    private Rigidbody rigid;
    private BoxCollider floor;
    public void Init(BoxCollider floor, List<Vector3> targetPositions, Rigidbody rigid)
    {
        this.floor = floor;
        this.targetPositions = targetPositions;
        this.rigid = rigid;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void GenerateBoxesFcn()
    {
        System.Random rnd = new System.Random();
        Vector3 floorUpperLimits = floor.bounds.max;
        Vector3 floorLowerLimits = floor.bounds.min;
        Vector3 floorPosition = floor.transform.position;

        wallBox = GameObject.Find("wallBox");
        wallBoxTransform = wallBox.GetComponent<Transform>();

        Vector3 startPos = new Vector3(
                                                    (float)(floorLowerLimits[0]),
                                                    floorLowerLimits[1] + 1,
                                                    (float)(floorLowerLimits[2])
                                                );

        int numberOfBoxesZ = (int)(floor.bounds.extents[2] * 2 );
        int numberOfBoxesX = (int)(floor.bounds.extents[0] * 2 );
        int[,] boxesMap = new int[numberOfBoxesX+1, numberOfBoxesZ+1];
        int boxesLimits = rnd.Next(2, 6);
        for (int i = 0; i <= numberOfBoxesX; i++)
        {
            for (int j = 0; j <= numberOfBoxesZ; j++)
            {
                bool canPlace = true;
                Vector3 expectedPosition = new Vector3(startPos[0] + i,
                                            startPos[1],
                                            startPos[2] + j
                                            );

                foreach (Vector3 targetPos in targetPositions)
                {
                    if (System.Math.Abs(expectedPosition[0] - targetPos[0]) < 3 + boxesLimits &&
                        System.Math.Abs(expectedPosition[0] - targetPos[0]) < 3 + boxesLimits &&
                        System.Math.Abs(expectedPosition[2] - targetPos[2]) < 3 + boxesLimits &&
                        System.Math.Abs(expectedPosition[2] - targetPos[2]) < 3 + boxesLimits)
                    {
                        canPlace = false;
                        boxesMap[i, j] = 0;
                    }
                    if (System.Math.Abs(expectedPosition[0] - rigid.transform.position[0]) < 3 &&
                        System.Math.Abs(expectedPosition[2] - rigid.transform.position[2]) < 3)
                    {
                        canPlace = false;
                        boxesMap[i, j] = 0;
                    }
                }

                if (canPlace)
                {
                    GameObject wallBoxTemp = Instantiate(wallBox);
                    wallBoxTemp.transform.position = expectedPosition;
                    Collider wallBoxCol = wallBoxTemp.GetComponent<Collider>();
                    wallBoxCol.name = "wallbox";
                    wallBoxCol.enabled = true;
                    wallBoxList.Add(wallBoxTemp);
                    wallBoxColList.Add(wallBoxCol);
                    boxesMap[i, j] = 1;
                    wallBoxPositions.Add(expectedPosition);
                }
            }
        }
    }
}
