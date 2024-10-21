using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Joint_3 : MonoBehaviour
{
    // Start is called before the first frame update
    ArticulationBody This_Object_Body;
    ArticulationDriveAxis Joint_Axis = ArticulationDriveAxis.X;
    //public GameObject Base_Robot;
    IK_David Base_IK_Script;
    [SerializeField] GameObject Base_Robot;


    void Start()
    {
       Base_IK_Script =  Base_Robot.GetComponent<IK_David>();
      This_Object_Body =  GetComponent<ArticulationBody>();
        
    }

    // Update is called once per frame
    void Update()
    {
        //print(Base_IK_Script.Joints_Angles[2])  ;
        float Joint_Angle = Base_IK_Script.Joints_Angles[2];
        This_Object_Body.SetDriveTarget(Joint_Axis , Joint_Angle);
    }
}
