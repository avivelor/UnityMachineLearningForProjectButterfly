/* Aviv Elor - aelor@ucsc.edu - avivelor1@gmail.com
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * CRUX_SpawnPointManager.cs
 * This Script is attached to the SpawnPointManager. It takes in locations of empty childs and spawns a given object at a publicly referenced location.
 * Desc
*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

//this script handles the spawning of the butterfly. Spawning differs on Scene name which will change it's movement for the desired excersize.
//this script is attached to "SpawnPointManager" under no other object and is attached to an empty child
public class CRUX_SpawnPointManager : MonoBehaviour {

    public ExoAgent AgentArm;
    public int ButterflyMovement = 0; //0 (HSR), 1 (FAR), 2 (SAR) cause enums aren't compiling idk
	public GameObject butterfly; //create a public reference to choose which object is being dropped
	public Transform headsetTransform;
	public float forwardOffset = 0; //x direction: + -> forward, - -> backwards
	public float headsetOffset = 0; //y direction: + -> up, - -> down
	public float horizontalOffset = 0; //z direction: + -> left, - -> right

	public int handDominance = 0; //1 true is left handed, 0 false is right handed.

    GameObject newButterfly;
    float armLength = 0.5f;

    void Awake()
    {   
        

        //spawn butterfly realitive to user headset
        handDominance = PlayerPrefs.GetInt("Hand Dominance");

        /*
        forwardOffset = float.Parse(PlayerPrefs.GetString("Arm Length")); //Define Forward Offset of Butterfly by Arm Length\
        
        headsetOffset = (-1 * (float.Parse(PlayerPrefs.GetString("Arm Length")) / 2)); //Define Forward Offset of Butterfly by Arm Length\

        f (SceneManager.GetActiveScene().name == "Side Arm Raise") { //if bubble lateral, remove forward offset (side raises)
            forwardOffset = 1 * (float.Parse(PlayerPrefs.GetString("Arm Length")) / 4);
            if (handDominance == 1) {
                horizontalOffset = (float.Parse(PlayerPrefs.GetString("Arm Length")) / 1);
            } else {
                horizontalOffset = -1 * (float.Parse(PlayerPrefs.GetString("Arm Length")) / 1);
            }
        } else {
            if (handDominance == 1) {
                horizontalOffset = (float.Parse(PlayerPrefs.GetString("Arm Length")) / 2);
            } else {
                horizontalOffset = -1 * (float.Parse(PlayerPrefs.GetString("Arm Length")) / 2);
            }
        } */

        if(SceneManager.GetActiveScene().name == "Custom Motion Primitives") {
			forwardOffset = 0;
            horizontalOffset = 0;
		}

        if(headsetTransform == null) headsetTransform = GameObject.Find(transform.root.name + "/BodyParts/Face").transform;

        newButterfly = Instantiate(butterfly, new Vector3(headsetTransform.position.x + forwardOffset, headsetTransform.position.y + headsetOffset,
            headsetTransform.position.z + horizontalOffset), new Quaternion(0.7071f, 0f, 0f, -0.7071f), this.gameObject.transform);
            newButterfly.GetComponent<UmbrellaHandeler>().ButterflyMovement = ButterflyMovement;
            newButterfly.GetComponent<UmbrellaHandeler>().shoulderOffset = new Vector3(forwardOffset,headsetOffset,horizontalOffset);
            if(AgentArm != null) newButterfly.GetComponent<UmbrellaHandeler>().MLagent = AgentArm.gameObject;
    }


	// Use this for initialization
	void Start () {
 
       armLength = float.Parse(PlayerPrefs.GetString("Arm Length"));
       if (SceneManager.GetActiveScene().name == "Main Loading Scene" || SceneManager.GetActiveScene().name == "Custom Motion Primitives") {

       } 
       if(AgentArm != null) AgentArm.goal = newButterfly;


    }
		
	// Update is called once per frame
	void Update () {
         if (SceneManager.GetActiveScene().name == "Main Loading Scene") {
             armLength = float.Parse(PlayerPrefs.GetString("Arm Length"));
             newButterfly.transform.position = new Vector3(headsetTransform.position.x, headsetTransform.position.y - 0.25f, headsetTransform.position.z - armLength);
         }
	}
}



