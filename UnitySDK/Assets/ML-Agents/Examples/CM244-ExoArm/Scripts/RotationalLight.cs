/* Dustin Halsey - dhalsey@ucsc.edu
 * Created for DANSER and ISIS Labs at University of California, Santa Cruz
 * RotationalLight.cs
 * This Script is attached to a directional light in the scene.  It controls the transition from night to day based upon the butterfly's protection.
 * This script is reliant on the butterflyProt variable from the UmbrellaHandeler.cs script attached to butterfly 3(Clone)
*/
//Disabled for now
/*
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotationalLight : MonoBehaviour
{
    private Transform direction;  //angle of light for time of day
    private Light lighting; //intensity of light

    public bool butterflyProt; //the butterfly object (used to obtain the protectionFlag for time of day
    [Range(-20.0f, 80.0f)]
    public float timeOfDay;

    // Use this for initialization
    void Start()
    {
        direction = gameObject.GetComponent(typeof(Transform)) as Transform;
        lighting = gameObject.GetComponent(typeof(Light)) as Light;
        //butterflyProt = GameObject.Find("Bubble").GetComponent()
    }

    // Update is called once per frame
    void Update()
    {
        if (GameObject.Find("butterfly 3(Clone)") != null){ //gets the butterflyobject's butterflyProt variable Note this is done in the update due to the butterfly not existing until start
            butterflyProt = GameObject.Find("butterfly 3(Clone)").GetComponent<UmbrellaHandeler>().protectionFlag;
            //Debug.Log(butterflyProt);
        }
        
        if (butterflyProt == true){ //brightens light if the butterfly is protected
            if (timeOfDay < 80.0f) timeOfDay += 0.25f;
        } else{ //darkens light
            if (timeOfDay > -20.0f) timeOfDay -= 0.1f;
        }
        
        lighting.intensity = (timeOfDay + 20.0f) / 45.5f; //adjusts the intensity to make the time of day more drastic
        direction.rotation = Quaternion.Euler(timeOfDay, 0, 0); //changes the time of day
    }
}
*/
