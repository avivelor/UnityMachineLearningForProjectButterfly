using UnityEngine;

public class ExoGoal : MonoBehaviour
{
    public GameObject agent;
    public GameObject hand;
    public GameObject goalOn;

    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject == hand)
        {
            goalOn.transform.localScale = new Vector3(1f, 1f, 1f);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.gameObject == hand)
        {
            goalOn.transform.localScale = new Vector3(0f, 0f, 0f);
        }
    }

    void OnTriggerStay(Collider other)
    {
        if (other.gameObject == hand)
        {
            agent.GetComponent<ExoAgent>().AddReward(0.01f);
        }
    }
}
