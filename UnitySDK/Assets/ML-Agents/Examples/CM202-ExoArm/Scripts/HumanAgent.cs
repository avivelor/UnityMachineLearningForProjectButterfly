using MLAgents;
using UnityEngine;
using UnityEngine.UI;

public class HumanAgent : MonoBehaviour {
    private float maxTorque = 150f; //default is 150f
    public GameObject pendulumA;
    public GameObject pendulumB;
    public GameObject hand;

    float rbaTorqueX = 0f; //shoulder pitch
    float rbaTorqueZ = 0f; //shoulder roll
    float rbbTorqueX = 0f; //elbow pitch
    float rbbTorqueZ = 0f; //elbow roll

    public Text ShoulderPitchText; //shoulder pitch
    public Text ShoulderRollText; //shoulder roll
    public Text ElbowPitchText; //elbow pitch
    public Text ElbowRollText; //elbow roll

    Rigidbody m_RbA;
    Rigidbody m_RbB;

    float repSpeed = 5f;

    // Use this for initialization
    void Start () {
        repSpeed = float.Parse (PlayerPrefs.GetString ("Speed"));

        m_RbA = pendulumA.GetComponent<Rigidbody> ();
        m_RbB = pendulumB.GetComponent<Rigidbody> ();
    }

    bool flag1 = false, flag2 = false, flag3 = false, flag4 = false, flag5 = false, flag6 = false, flag7 = false, flag8 = false;
    float countdownToRemoveTorqueTimer = 0f, countdownTime = 0.5f;
    // Use this for initialization
    void Update () {
        //need to do agent action here
        DoAgentAction (rbaTorqueX, rbaTorqueZ, rbbTorqueX, rbbTorqueZ);

        if (Input.GetKeyUp (KeyCode.Alpha1)) flag1 = false;
        if (Input.GetKeyUp (KeyCode.Alpha2)) flag2 = false;
        if (Input.GetKeyUp (KeyCode.Alpha3)) flag3 = false;
        if (Input.GetKeyUp (KeyCode.Alpha4)) flag4 = false;
        if (Input.GetKeyUp (KeyCode.Q)) flag5 = false;
        if (Input.GetKeyUp (KeyCode.W)) flag6 = false;
        if (Input.GetKeyUp (KeyCode.E)) flag7 = false;
        if (Input.GetKeyUp (KeyCode.R)) flag8 = false;

        if (Input.GetKeyDown (KeyCode.Alpha1) || flag1) //shoulder pitch
        {
            flag1 = true;
            rbaTorqueX = rbaTorqueX + TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        if (Input.GetKeyDown (KeyCode.Alpha2) || flag2) //shoulder roll
        {
            flag2 = true;
            rbaTorqueZ = rbaTorqueZ + TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        if (Input.GetKeyDown (KeyCode.Alpha3) || flag3) //elbow pitch
        {
            flag3 = true;
            rbbTorqueX = rbbTorqueX + TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        if (Input.GetKeyDown (KeyCode.Alpha4) || flag4) // elbow roll
        {
            flag4 = true;
            rbbTorqueZ = rbbTorqueZ + TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

         if (Input.GetKeyDown (KeyCode.Q) || flag5) //shoulder pitch
        {
            flag5 = true;
            rbaTorqueX = rbaTorqueX - TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        if (Input.GetKeyDown (KeyCode.W) || flag6) //shoulder roll
        {
            flag6 = true;
            rbaTorqueZ = rbaTorqueZ - TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        if (Input.GetKeyDown (KeyCode.E) || flag7) //elbow pitch
        {
            flag7 = true;
            rbbTorqueX = rbbTorqueX - TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        if (Input.GetKeyDown (KeyCode.R) || flag8) // elbow roll
        {
            flag8 = true;
            rbbTorqueZ = rbbTorqueZ - TorqueButtonPressAmount / 10;
            countdownToRemoveTorqueTimer = countdownTime;
        }

        ShoulderPitchText.gameObject.transform.parent.gameObject.GetComponent<Button> ().interactable = !flag1 && !flag5;
        ShoulderRollText.gameObject.transform.parent.gameObject.GetComponent<Button> ().interactable = !flag2 && !flag6;
        ElbowPitchText.gameObject.transform.parent.gameObject.GetComponent<Button> ().interactable = !flag3 && !flag7;
        ElbowRollText.gameObject.transform.parent.gameObject.GetComponent<Button> ().interactable = !flag4 && !flag8;

        if (countdownToRemoveTorqueTimer < 0) {
            if (rbaTorqueX > 0) {
                rbaTorqueX = rbaTorqueX - TorqueButtonPressAmount;
                if (rbaTorqueX < 0) rbaTorqueX = 0;
            }

            if (rbaTorqueZ > 0) {
                rbaTorqueZ = rbaTorqueZ - TorqueButtonPressAmount;
                if (rbaTorqueZ < 0) rbaTorqueZ = 0;
            }

            if (rbbTorqueX > 0) {
                rbbTorqueX = rbbTorqueX - TorqueButtonPressAmount;
                if (rbbTorqueX < 0) rbbTorqueX = 0;
            }

            if (rbbTorqueZ > 0) {
                rbbTorqueZ = rbbTorqueZ - TorqueButtonPressAmount;
                if (rbbTorqueZ < 0) rbbTorqueZ = 0;
            }
        } else {
            countdownToRemoveTorqueTimer = countdownToRemoveTorqueTimer - Time.deltaTime;
        }
    }

    void DoAgentAction (float rbaTorqueX, float rbaTorqueZ, float rbbTorqueX, float rbbTorqueZ) {
        var torqueX = Mathf.Clamp (rbaTorqueX, -1f, 1f) * maxTorque;
        var torqueZ = Mathf.Clamp (rbaTorqueZ, -1f, 1f) * maxTorque;
        m_RbA.AddTorque (new Vector3 (torqueX, 0f, torqueZ));
        if (ShoulderPitchText != null) ShoulderPitchText.text = torqueX.ToString ();
        if (ShoulderRollText != null) ShoulderRollText.text = torqueZ.ToString ();

        torqueX = Mathf.Clamp (rbbTorqueX, -1f, 1f) * maxTorque;
        torqueZ = Mathf.Clamp (rbbTorqueZ, -1f, 1f) * maxTorque;
        m_RbB.AddTorque (new Vector3 (torqueX, 0f, torqueZ));
        if (ElbowPitchText != null) ElbowPitchText.text = torqueX.ToString ();
        if (ElbowRollText != null) ElbowRollText.text = torqueZ.ToString ();

    }

    public float TorqueButtonPressAmount = 0.003f;
    public void ButtonPressAddTorqueShoulderPitch () //rbaTorqueX
    {
        rbaTorqueX = rbaTorqueX + TorqueButtonPressAmount;
        countdownToRemoveTorqueTimer = countdownTime;
    }

    public void ButtonPressAddTorqueShoulderRoll () //rbaTorqueZ
    {
        rbaTorqueZ = rbaTorqueZ + TorqueButtonPressAmount;
        countdownToRemoveTorqueTimer = countdownTime;
    }

    public void ButtonPressAddTorqueElbowPitch () //rbbTorqueX
    {
        rbbTorqueX = rbbTorqueX + TorqueButtonPressAmount;
        countdownToRemoveTorqueTimer = countdownTime;
    }

    public void ButtonPressAddTorqueElbowRoll () //rbbTorqueZ
    {
        rbbTorqueZ = rbbTorqueZ + TorqueButtonPressAmount;
        countdownToRemoveTorqueTimer = countdownTime;
    }
}