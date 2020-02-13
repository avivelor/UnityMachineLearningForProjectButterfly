using UnityEngine;
using MLAgents;

public class ExoAcademy : Academy
{
    public override void AcademyReset()
    {
        FloatProperties.RegisterCallback("gravity", f => { Physics.gravity = new Vector3(0, -f, 0); });

    }

    public override void AcademyStep()
    {
    }
}
