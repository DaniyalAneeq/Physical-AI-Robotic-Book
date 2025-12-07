using UnityEngine;

public class EnvironmentController : MonoBehaviour
{
    public Light directionalLight;

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.L))
        {
            directionalLight.enabled = !directionalLight.enabled;
        }
    }
}
