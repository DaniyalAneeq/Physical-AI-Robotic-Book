// examples/module2/chapter1-unity-robotics/SimpleController.cs

using UnityEngine;

public class SimpleController : MonoBehaviour
{
    public ArticulationBody joint;
    public float targetVelocity = 100f;

    void FixedUpdate()
    {
        var drive = joint.xDrive;
        drive.targetVelocity = targetVelocity;
        joint.xDrive = drive;
    }
}
