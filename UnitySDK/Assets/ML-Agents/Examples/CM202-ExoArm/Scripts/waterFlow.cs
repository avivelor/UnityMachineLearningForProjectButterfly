/* Dustin Halsey - dhalsey@ucsc.edu
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * RotationalLight.cs
 * This Script moves the offset of the material attached to the object.  THis is intended to create the water flow on the ball.
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class waterFlow : MonoBehaviour {
    private Vector2 waterOffset;
    private Renderer waterRend;
    private float rotateSpeed = 1f;
    private float scroll;
	// Use this for initialization
	void Start () {
        waterRend = GetComponent<Renderer>();
}
	
	// Update is called once per frame
	void Update () {
        scroll += 0.0008f;
        GetComponent<Transform>().rotation = Quaternion.Euler(0,0,0); //locks the rotation to improve the water effect
        waterRend.material.SetTextureOffset("_MainTex", new Vector2(0, scroll)); //offsets the texture to give the water flowing effect
	}
}
