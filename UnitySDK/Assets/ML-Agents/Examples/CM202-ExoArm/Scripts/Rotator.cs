/* Aviv Elor - aelor@ucsc.edu - avivelor1@gmail.com
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * Rotator.cs
 * This Script is attached to any object to make it rotate, also attached to pickups. 
*/

using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotator : MonoBehaviour {

	public Text dropSpeedSetup;
	public float rotationSpeed = 1;
	public int rotateX = 1;
	public int rotateY = 1;
	public int rotateZ = 1;

	void Update () { // Update is called once per frame by the unity physics engine
		transform.Rotate (new Vector3 (3*rotationSpeed*rotateX, 4*rotationSpeed*rotateY, 5*rotationSpeed*rotateZ) * Time.deltaTime * float.Parse (dropSpeedSetup.text)); //rotate the object based off of the speed and time per framerate
	}
}
