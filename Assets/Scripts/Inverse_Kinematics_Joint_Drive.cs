using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Inverse_Kinematics_Joint_Drive : MonoBehaviour
{

    ArticulationBody This_Object_Body;
    ArticulationDriveAxis Joint_Axis = ArticulationDriveAxis.X;
    public GameObject End_Effector;
    
    // Start is called before the first frame update
    void Start()
    {
        This_Object_Body =  GetComponent<ArticulationBody>();
    }

    // Update is called once per frame
    void Update()
    {   
        // Angle of this joint [rad]
         ArticulationReducedSpace This_Joint_Angle = This_Object_Body.jointPosition;
        // Joint Position
        Vector3 jointPosition = This_Object_Body.transform.position;

        // Target mouse position
        Vector3 End_Effector_Position = End_Effector.transform.position; //mainCamera.ScreenToWorldPoint(new Vector3(Mouse_Position.x, 0, Mouse_Position.z));

        
        Vector3 directionToMouse = End_Effector_Position - jointPosition;
        float targetAngle = Mathf.Atan2(directionToMouse.y, directionToMouse.z) * Mathf.Rad2Deg;
        float currentJointAngle = This_Object_Body.jointPosition[0]; // Předpokládáme revolute joint s jedním stupněm volnosti

         float angleDifference = Mathf.DeltaAngle(currentJointAngle, targetAngle);

        //print(This_Object_Body.dofCount);
        //print(angleDifference);

        This_Object_Body.SetDriveTarget(Joint_Axis , angleDifference);

       // ArticulationDrive drive = This_Object_Body.xDrive;
       // drive.target = currentJointAngle + angleDifference; // Nový cílový úhel
       // This_Object_Body.xDrive = drive;

        //print(This_Joint_Angle[0] );
      //  ArticulationBody.SetDriveTarget(ArticulationDriveAxis.X,)
    }
}
