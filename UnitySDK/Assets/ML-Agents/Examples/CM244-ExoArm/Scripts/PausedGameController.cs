/* Aviv Elor - aelor@ucsc.edu - avivelor1@gmail.com
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * PausedGameController.cs
 * This Script is to pausedBackground. It enabled, disables, and controls the pause menu.
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

// this is a very sexy script that handles loading, pausing, and saving user preferences.
//this script is attached to an empty child in the Evaluator Interface, top level
public class PausedGameController : MonoBehaviour {
	//public GameObject objectWithholdingScript;
	//PickupManager scriptData;

	public Transform pauseMenu; //get the pause screen
	public Transform setupMenu;
	//public Transform dataExporter;
	public Transform howToPlayMenu;
	public Transform SpawnPointManager;
	public Transform InstructionText;
	public Transform ResetCalibrationButton;

	public InputField playerNameField;
	public InputField armLengthField;
	public InputField speedField;
	public InputField sampleRateHzField;
	public Toggle handDominance; //handDominance.isOn (returns true if left handed)

	public Toggle autoMode;
	bool isAutoMode = false;

	void Awake () { //update all input fields to previously set values
		if (!PlayerPrefs.HasKey ("Player Name")) {
			PlayerPrefs.SetString ("Player Name", "Username");
			PlayerPrefs.SetString ("Arm Length", "0.5");
			PlayerPrefs.SetString ("Speed", "5");
			PlayerPrefs.SetString ("Sample Rate Hz", "90");
			PlayerPrefs.SetInt ("Hand Dominance", 0);
			PlayerPrefs.SetInt ("Auto Mode", 0);
		}

		playerNameField.text = PlayerPrefs.GetString ("Player Name");
		armLengthField.text = PlayerPrefs.GetString ("Arm Length");
		speedField.text = PlayerPrefs.GetString ("Speed");
		sampleRateHzField.text = PlayerPrefs.GetString ("Sample Rate Hz");
		if (PlayerPrefs.GetInt ("Hand Dominance") == 1) {
			handDominance.isOn = true;
		} else {
			handDominance.isOn = false;
		}

		if (autoMode != null) {
			if (PlayerPrefs.GetInt ("Auto Mode") == 1) {
				autoMode.isOn = true;
			} else {
				autoMode.isOn = false;
			}
		}
	}

	// Use this for initialization
	void Start () {
		if (GameObject.Find ("SpawnPointManager") != null) SpawnPointManager = GameObject.Find ("SpawnPointManager").transform;

		if (PlayerPrefs.GetInt ("Hand Dominance") == 1) {
			handDominance.isOn = true;
		} else {
			handDominance.isOn = false;
		}

		if (autoMode != null) {
			if (PlayerPrefs.GetInt ("Auto Mode") == 1) {
				autoMode.isOn = true;
			} else {
				autoMode.isOn = false;
			}
		}

		if (PlayerPrefs.GetInt ("Auto Mode") == 1) {
			isAutoMode = true;
		}

		autoTimer = 0.0f;
		SavePlayerPreferences ();
		Time.timeScale = 0; //freeze all time, stop all motion in the world
	}

	// Update is called once per frame
	float autoTimer = 0.0f; //auto mode timer
	public float timePerAutoRound = 60.0f; //one minute each round
	void Update () {
		//Debug.Log(PlayerPrefs.GetString ("Target Therapy"));
		if (Input.GetKeyDown (KeyCode.Escape)) { //whenever escape is hit
			Pause ();
		}

		if (isAutoMode) {
			autoTimer += Time.deltaTime;
			if (autoTimer > timePerAutoRound) {

				if (Time.timeScale != 0) {
					Pause ();
				}

				StartCoroutine (AutoStart ());

			}
		}
	}

	IEnumerator AutoStart () {
		GameObject.Find ("Weak Score").GetComponent<TextMesh> ().text = "Great Job! Loading new round...";

		yield return new WaitForSecondsRealtime (5f); //delays the functionality of start by 0.1 seconds to make sure everything is loaded before getting object
		if (SceneManager.GetActiveScene ().name == "Main Loading Scene") { //load forward arm raise
			LoadBubble_Forward ();
		} else if (SceneManager.GetActiveScene ().name == "Forward Arm Raise") { //load horizontal shoulder rotation
			LoadButterfly_Rain ();
		} else if (SceneManager.GetActiveScene ().name == "Horizontal Shoulder Rotation") { //side arm raise
			LoadBubble_Lateral ();
		} else { //load main menu
			SaveAndSetup ();
		}

	}

	public void SetTargetTherapy () {
		PlayerPrefs.SetString ("Target Therapy", InstructionText.GetComponent<Text> ().text); //swap between MVFT and CIMT depending on drop menu text
	}

	public void ResetAllCalibration () {
		PlayerPrefs.SetString ("Arm Length", "0");
	}

	public void StartGame () {
		Time.timeScale = 1;
		setupMenu.gameObject.SetActive (false);
		InstructionText.gameObject.SetActive (false);
		SpawnPointManager.gameObject.SetActive (true);
		//dataExporter.gameObject.SetActive (true);
	}

	public void Pause () {
		if (pauseMenu.gameObject.activeInHierarchy == false) { //if the pause menu is not open
			pauseMenu.gameObject.SetActive (true); //open the pause menu
			Time.timeScale = 0; //freeze all time, stop all motion in the world

		} else {
			pauseMenu.gameObject.SetActive (false); //else close the pause menu
			Time.timeScale = 1; //return all time back to normal
		}
	}

	public void SaveAndSetup () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("Main Loading Scene");
	}

	bool HTCswitch = true;
	public void HowToPlayExit () {
		if (HTCswitch == false) {
			howToPlayMenu.gameObject.SetActive (false);
			HTCswitch = true;
		} else if (HTCswitch == true) {
			howToPlayMenu.gameObject.SetActive (true);
			HTCswitch = false;
		}
	}

	public void ReloadLevel () {
		//Application.LoadLevel (Application.loadedLevel); //reload everything
		UnityEngine.SceneManagement.SceneManager.LoadScene (UnityEngine.SceneManagement.SceneManager.GetActiveScene ().buildIndex);
	}

	public void Quit () {
		//Application.LoadLevel(0) //Main Menu
		SavePlayerPreferences ();
		Application.Quit ();
	}

	public void LoadButterfly_Crystal () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("Crystal Catcher");
	}

	public void Load_CustomMotions () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("ExoButterflyTraining");
	}

	public void Load_MotionRecordingMode () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("Motion Recording Scene");
	}

	public void LoadButterfly_Rain () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("Horizontal Shoulder Rotation");
	}

	public void LoadBubble_Forward () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("Forward Arm Raise");
	}

	public void LoadBubble_Lateral () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("Side Arm Raise");
	}

	public void LoadBubble_HumanVsAgent () {
		SavePlayerPreferences ();
		UnityEngine.SceneManagement.SceneManager.LoadScene ("HumanVsAgent");
	}

	void SavePlayerPreferences () {
		if (SceneManager.GetActiveScene ().name == "Main Loading Scene") {
			PlayerPrefs.SetString ("Target Therapy", InstructionText.GetComponent<Text> ().text);
		}
		PlayerPrefs.SetString ("Player Name", playerNameField.text);
		PlayerPrefs.SetString ("Arm Length", armLengthField.text);
		PlayerPrefs.SetString ("Speed", speedField.text);
		PlayerPrefs.SetString ("Sample Rate Hz", sampleRateHzField.text);
		if (handDominance.isOn == true) {
			PlayerPrefs.SetInt ("Hand Dominance", 1);
		} else {
			PlayerPrefs.SetInt ("Hand Dominance", 0);
		}

		if (autoMode != null) {
			if (autoMode.isOn == true) {
				PlayerPrefs.SetInt ("Auto Mode", 1);
			} else {
				PlayerPrefs.SetInt ("Auto Mode", 0);

			}
		}
	}

	public void saveArmLength () {
		PlayerPrefs.SetString ("Arm Length", armLengthField.text);
	}

	public void toggleRunAutoMode () {
		if (autoMode.isOn) {
			PlayerPrefs.SetInt ("Auto Mode", 1);
			Debug.Log ("auto on");
		} else {
			PlayerPrefs.SetInt ("Auto Mode", 0);
			Debug.Log ("auto off");
		}
		SavePlayerPreferences ();
	}

}