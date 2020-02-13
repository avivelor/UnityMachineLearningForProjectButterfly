/* Dustin Halsey - dhalsey@ucsc.edu
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * faceCamera.cs
 * This script rotates the parent object to always face the player
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;




public class faceTowardsCamera : MonoBehaviour {

    public GameObject cameraHead; //the object that this object will face

    // Use this for initialization
    void Start () {
        if (cameraHead == null) {
            Debug.Log("WRNING: no object to face towards assigned in faceTowardsCamera.cs");
        }
	}
	
	// Update is called once per frame
	void Update () {
        if (cameraHead != null) {
            transform.LookAt(cameraHead.GetComponent<Transform>().position);
        }
	}
}
