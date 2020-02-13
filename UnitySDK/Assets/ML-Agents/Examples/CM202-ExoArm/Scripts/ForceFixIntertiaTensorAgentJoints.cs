using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ForceFixIntertiaTensorAgentJoints : MonoBehaviour
{
    public Vector3 tensorXYZ = new Vector3(1f,1f,1f);
    // Start is called before the first frame update
    void Start()
    {
        var rb = GetComponent<Rigidbody>();
        rb.inertiaTensor = tensorXYZ; //static values at
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
