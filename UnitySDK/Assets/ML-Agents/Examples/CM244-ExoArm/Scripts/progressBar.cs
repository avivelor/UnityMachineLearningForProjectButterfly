using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class progressBar : MonoBehaviour {
    private int isProtected = 0;
    private Image progImage;
    private GameObject pausedManager; //used for startgame function
    private float fillRate = .4f; //the time in seconds it takes to fill the bar
    // Use this for initialization
    void Start () {
        progImage = GetComponent<Image> ();
        progImage.fillAmount = 0f;
        pausedManager = GameObject.Find ("PauseManager");
    }

    // Update is called once per frame
    void Update () {
        //Debug.Log(PlayerPrefs.GetInt("Butterfly Protected"));
        //checks to see if the butterfly is protected
        if (PlayerPrefs.GetInt ("Butterfly Protected") == 1) {
            isProtected = 1;
        } else if (PlayerPrefs.GetInt ("Butterfly Protected") == 2) {
            isProtected = 2;
        } else {
            isProtected = 0;
        }

        if (isProtected == 1 && transform.parent.parent.tag == "WeakBubble") {

            progImage.fillAmount += fillRate * Time.unscaledDeltaTime;
        } else if (isProtected == 2 && transform.parent.parent.tag == "StrongBubble") {
            progImage.fillAmount += fillRate * Time.unscaledDeltaTime;
        } else {
            progImage.fillAmount -= fillRate * Time.unscaledDeltaTime;
        }

        //starts the game when the bar fills
        if (progImage.fillAmount >= 1) {
            Time.timeScale = 1;
            progImage.fillAmount = 0;
            if (SceneManager.GetActiveScene ().name != "ExoButterflyTraining" && SceneManager.GetActiveScene ().name != "ExoArm") {
                pausedManager.GetComponent<PausedGameController> ().StartGame (); //starts the game via the UI

                if (SceneManager.GetActiveScene ().name == "Main Loading Scene" && PlayerPrefs.GetInt ("Auto Mode") == 1) {
                    UnityEngine.SceneManagement.SceneManager.LoadScene ("Forward Arm Raise");
                }
            }

        }

        //disables the progress wheel if the game has started
        if (Time.timeScale > 0) {
            progImage.enabled = false;
        } else {
            progImage.enabled = true;
        }
    }
}