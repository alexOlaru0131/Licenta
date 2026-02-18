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

public class Plant : MonoBehaviour
{
    public WheelCollider wheel1Col, wheel2Col, wheel3Col, wheel4Col;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

        // TO DO: ADAUGA ESANTIONARE

        Debug.Log("FR: " + wheel1Col.rpm / 60 * 2 * Mathf.PI);
        // Debug.Log("FL: " + wheel2Col.rpm);
        // Debug.Log("BR: " + wheel3Col.rpm);
        // Debug.Log("BL: " + wheel4Col.rpm);
    }
}
