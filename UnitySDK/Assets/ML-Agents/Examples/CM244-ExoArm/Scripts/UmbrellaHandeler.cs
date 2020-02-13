/* Dustin Halsey - dhalsey@ucsc.edu
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * UmbrellaHandeler.cs
 * This Script is attached to the butterfly prefab. It controls score, and movement of butterfly.
 * NOTE: Need to update rotatePoint to be more accurate to the middle of the shoulders
*/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System;

//CODE BEING REWRITTEN. WIP
public class UmbrellaHandeler : MonoBehaviour {
    public int ButterflyMovement = 0; //0, 1, 2 cause enums aren't compiling idk
    
    public float armLength = 0.5f; //the arm length of the user. Will be updated on start
    public float speed = 0.1f; //the speed the butterfly moves
    public static int rainScore = 0; //??? What is this for??? was original used to tell difference between crystal catcher and bubble points

    public GameObject particleGood;
    public GameObject particleBad; 
    public Vector3 shoulderOffset = new Vector3(0,-0.2f,0); //the offset for the rotation point of the butterfly (shoulder point)


    private Rigidbody rb; //get the Rigidbody of the controller
    private GameObject cameraHead; //The eye object of the player
    private GameObject butterFly; //The butterfly gameObject
    private GameObject weakBubble;
    public GameObject MLagent;


    public bool weakProtectionFlag = false; //if the butterfly is protected by the weak bubble
    public bool strongProtectionFlag = false; //if the butterfly is protected by the strong bubble
    private float repsPerMin = 6; //the number of reps in each set
    private float setDuration = 60; // The length in seconds of each set
    

    private int handDominance; //left or right hand (0 = right, 1 = left);

    int mpNum = 0; //number of motion paths
	List<string> mpNames = new List<string>(); //names of motion paths
	List<int> mpCounts = new List<int>(); //counts of motion paths
	List<float> mpTimes = new List<float>(); //counts of motion paths
	List<List<Vector3>> mpPositions = new List<List<Vector3>>(); //list of motion paths

	void GetSavedMotionPathData() {
		mpNum = 0;
		mpNames.Clear();
		mpCounts.Clear();
		mpTimes.Clear();
		mpPositions.Clear();
		
		mpNum = PlayerPrefs.GetInt("CustomMotionPath_NumPaths");
		string pathId = "CustomMotionPath";

		for(int i = 0; i < mpNum; i++) {
			string pathName = pathId + i + "_";
			mpCounts.Add(PlayerPrefs.GetInt(pathName + "count"));
			mpNames.Add(PlayerPrefs.GetString(pathName + "name"));
			mpTimes.Add(PlayerPrefs.GetFloat(pathName + "time"));
			List<Vector3> positions = new List<Vector3>();
			for(int j = 0; j < mpCounts[i]; j++) {
				float x = PlayerPrefs.GetFloat(pathName + "x" + j);
				float y = PlayerPrefs.GetFloat(pathName + "y" + j);
				float z = PlayerPrefs.GetFloat(pathName + "z" + j);
				positions.Add(new Vector3(x,y,z));
			}
			mpPositions.Add(positions);
		}

        InterfacePopulatePreviewMotionPaths();
	}

    public Dropdown SavedMotionPaths;
    void InterfacePopulatePreviewMotionPaths() {
            if(SavedMotionPaths == null) {
                SavedMotionPaths = GameObject.Find("Dropdown_SavedMocapPaths").GetComponent<Dropdown>();
         }

		    SavedMotionPaths.ClearOptions();
		    List<string> pathOptions = new List<string>();
		
		    for(int i = 0; i < mpNum; i++) {
		    	pathOptions.Add(mpNames[i]);
		    }

		    if(pathOptions.Count > 0) {
			    SavedMotionPaths.AddOptions(pathOptions);
		    }

            //start at inital position
            this.transform.position = mpPositions[0][0];
	}

    bool isMotionPathBeingHandled = false;
    IEnumerator RenderMotionPath(List<Vector3> path, float recordTime, Transform butterfly) {
		Debug.Log("Rendering motion path of " + recordTime + " second length");
        isMotionPathBeingHandled = true;
	
		//lineRenderer.SetVertexCount(0);
		//Change how mant points based on the mount of positions is the List
		//lineRenderer.SetVertexCount(path.Count);
        cameraHead = GameObject.Find(transform.root.name + "/BodyParts/Face");
		for (int i = 0; i < path.Count; i++ )
		{
			yield return new WaitForSecondsRealtime(recordTime/path.Count);
  			//Change the postion of the lines
    		Vector3 pose =   (Quaternion.Euler(0,-90,0) * path[i]) + cameraHead.transform.position;
            butterfly.position = pose;
		}
        //+ new Vector3(armLength,0,0)
        for(int i = path.Count-1; i >= 0; i--) {
            yield return new WaitForSecondsRealtime(recordTime/path.Count);
            Vector3 pose =  (Quaternion.Euler(0,-90,0) * path[i]) + cameraHead.transform.position;
            butterfly.position = pose;
        }

        PlayerPrefs.SetInt("Reps", PlayerPrefs.GetInt("Reps") + 1); //Resets the number of reps completed by the player
        isMotionPathBeingHandled = false;
	}


    void Start() {
        
        handDominance = 0; //0 = right 1 = left
        speed = float.Parse(PlayerPrefs.GetString("Speed")); // updates the speed to be what the player has listed
        armLength = float.Parse(PlayerPrefs.GetString("Arm Length"));
        PlayerPrefs.SetInt("Butterfly Protected", 0); //Resets the butterfly to not protected
        PlayerPrefs.SetInt("Reps", 0); //Resets the number of reps completed by the player
        //Debug.Log(transform.root.name.ToString());
        cameraHead = GameObject.Find(transform.root.name + "/BodyParts/Face");
        butterFly = GameObject.Find(transform.root.name + "/SpawnPointManager/butterfly");
        // gets the reference of inactive game objects.  This is neccessary because 
        // Unity's "find" does not locate inactive objects
        /*foreach (GameObject go in Resources.FindObjectsOfTypeAll(typeof(GameObject)) as GameObject[]) {
            if (go.tag == "WeakBubble" && go.scene == SceneManager.GetActiveScene()) {
                weakBubble = go;
            }
        }*/
        weakBubble = GameObject.Find(transform.root.name + "/ExoAgent/Capsule (1)/Hand/Arm/BubbleWeak");

        if((SceneManager.GetActiveScene().name == "Custom Motion Primitives")) {
            GetSavedMotionPathData();
        }
    }
    void Update() {
        if (SceneManager.GetActiveScene().name != "Main Loading Scene") {
        getShoulderPos();
        RotateButterfly(); //moves the butterfly if the game has started
        }

        if((Time.timeScale ==1 && SceneManager.GetActiveScene().name == "Custom Motion Primitives") && isMotionPathBeingHandled == false) {
            int mpIndex = SavedMotionPaths.value;
            StartCoroutine(RenderMotionPath(mpPositions[mpIndex], mpTimes[mpIndex], this.transform));
        }
        collisionBubbles(weakBubble);
    }

    //checks for collisions when timeScale is 0
    //Unity by default will not run OnCollision when timeScale is 0
    void collisionBubbles(GameObject bubble) {
        if (bubble == null) return;
        if (Time.timeScale >= 1) return; //exits loop if game is started because this becomes unnceccesary
        Collider bubbleCollider = bubble.GetComponent<Collider>();
        
        //if the objects collide
        if (bubbleCollider.bounds.Intersects(butterFly.GetComponent<Collider>().bounds)) { 
            if (bubble.tag == "WeakBubble") {
                PlayerPrefs.SetInt("Butterfly Protected", 1);
            } 
        } else if (bubble.transform.parent.gameObject.activeSelf == true) {
            PlayerPrefs.SetInt("Butterfly Protected", 0);
        }
    }


    //calculates the position of the user's shoulders in relation to the HMD
    private Vector3 getShoulderPos() {
        Vector3 shoulderPos;
        shoulderPos = cameraHead.transform.position + shoulderOffset;
        return shoulderPos;
    }


    private float rotateAngle = 0;
    private int currentRep = 0;
    //please note that this function is currently reliant on a consistent rotation radius. to have this work
    //with a dynamic range of motion, changes will have to be made
    void RotateButterfly() {
        //float singleRepSpeed = setDuration / repsPerMin; // the time in seconds of a single rep
        float singleRepSpeed = setDuration / speed; //note that speed is entered as RepsPerMin. Updated in PausedGameController. Entered through Evaluator Interface GUI
        Vector3 rotatePoint = getShoulderPos();
        float posX = rotatePoint.x;
        float posY = rotatePoint.y;
        float posZ = rotatePoint.z;
        float max = 3.14f / 3f, min = 3.14f / 3f; //the max and min rotation angles in radians

        //Gets the rotation angles for each scence. Adjust these values to change the range of motion
        if (ButterflyMovement == 0) {
            max = 3.14f/3f;  //60 degrees
            min = -3.14f/3f; //-60 degrees
        } else if (ButterflyMovement == 1) {
            max = 3.14f/3f;  //60 degrees
            min = -3.14f/3f; //-60 degrees
        } else if (ButterflyMovement == 2) {
            max = 3.14f/3f;  //60 degrees
            min = -3.14f/3f; //-60 degrees
        }

        //prevents the possibility of double counting a rep
        if (currentRep != PlayerPrefs.GetInt("Reps")) {
            rotateAngle = min;
            currentRep = PlayerPrefs.GetInt("Reps");
        }
            
        if (Time.timeScale == 0) rotateAngle = min; //keeps the butterfly in place until the game starts

        //gets the butterfly's next position
        if (ButterflyMovement == 0) {
            posX = rotatePoint.x + Mathf.Cos(rotateAngle) * armLength;
            posZ = rotatePoint.z + Mathf.Sin(rotateAngle) * armLength;
            switchDirection(rotateAngle, min, max);
        } else if (ButterflyMovement == 1) {
            posX = rotatePoint.x + Mathf.Cos(rotateAngle) * armLength;
            posY = rotatePoint.y + Mathf.Sin(rotateAngle) * armLength;
            switchDirection(rotateAngle, min, max);
        } else if (ButterflyMovement == 2) {
            if (handDominance == 1) { //left hand
                posY = rotatePoint.y + Mathf.Sin(rotateAngle) * armLength;
                posZ = rotatePoint.z + (Mathf.Cos(rotateAngle) * armLength) + 0.15f; //offsets posZ to left side (the float is to adjsut for shoulder dist) 
                posX += 0.2f; //adds an offset for the shoulder
            } else if (handDominance == 0) { //right hand
                posY = rotatePoint.y + Mathf.Sin(rotateAngle) * armLength;
                posZ = rotatePoint.z - (Mathf.Cos(rotateAngle)*armLength) - 0.15f; //offsets posZ to left side (the float is to adjsut for shoulder dist)    
                posX += 0.2f; //adds an offset for the shoulder
            }      
            switchDirection(rotateAngle, min, max);
        }


        //Moves the butterfly at a set reps per min
        //                   //total rad for 1 rep       repsPerSec
        //float rotationPerSec = ((max - min) * 2f) * (repsPerMin / 60) ; //finds the radians to rotate per second
        float rotationPerSec = ((max - min) * 2f) * (speed / 60) ; //finds the radians to rotate per second
        rotateAngle += /*speed*/ rotationPerSec * Time.deltaTime * direction; //increments the rotation angle at a set speed

        transform.position = new Vector3(posX, posY, posZ);
    }



    private int direction = 1; //the path direction of the butterfly (-1 or 1)
    //checks to see if the butterfly's direction should be switched
    //usage: currentAngle = butterfly's current angle in radians; minAngle = smallest angle butterfly should reach before changing directions
    // maxAngle = largest angle butterfly should reach before changing directions
    void switchDirection(float currentAngle, float minAngle, float maxAngle) {
        //Half rep completed
        if(currentAngle > maxAngle) {
            direction = -1;
        //Full rep completed
        } else if (currentAngle < minAngle) {
            direction = 1;
            PlayerPrefs.SetInt("Reps", PlayerPrefs.GetInt("Reps") + 1); //Resets the number of reps completed by the player
        }
    }

    //Updates protection to true when the butterfly touches another object
    void OnTriggerStay(Collider other) { //whenever the catcher collides with another interactive object    
        if (other.gameObject.CompareTag("WeakBubble")) {
            weakProtectionFlag = true;
            PlayerPrefs.SetInt("Butterfly Protected", 1); //1 = weak

            //UNITY ML AGENT REWARD HERE
            if(MLagent != null) MLagent.GetComponent<ExoAgent>().AddReward(0.01f);
            //Debug.Log("Agent Recieves Reward!");
        }
        
    }

    //Updates protection to false when the butterfly touches another object
    //NOTE: may need to adjust how this works when both bubbles cover the butterfly
    void OnTriggerExit(Collider other) {
        if (other.gameObject.CompareTag("WeakBubble")) {
            weakProtectionFlag = false;
            PlayerPrefs.SetInt("Butterfly Protected", 0); //0 = no protection
        }
        
    }
}


		
