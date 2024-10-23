using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ur_kinematics;

public class IK_David : MonoBehaviour
{


    public Transform Target_Effector;

    
    
    robot_kinematics Robot_Kinematics ;  
    //Transform test = Target_Effector.Rotate(0, 0, 0);


    private double[] Coordinates_Matrix = new double[6];    // Matrix used for sending coordinates to "robot_kinematic" class. First 3 elemets are postitions, next 3 elemets are rotations
    private float[] Temp_coordinates = new float[6];
    public float[] Joints_Angles = new float[6];           // Output of script; values (Thetas) in Rads, float type
    private double[][] Theta     = new double[6][];                 // Theta field stores output theta from "inverse_kin" script in double type





    // Start is called before the first frame update
    void Start()
    {
        Robot_Kinematics = new robot_kinematics(RobotType.UR10e, coordinateSystem.xzy_L);
    }

    // Update is called once per frame
    void Update()
    {
        // Fill up positions
        Temp_coordinates[0] = Target_Effector.transform.position.x;
        Temp_coordinates[1] = Target_Effector.transform.position.y;
        Temp_coordinates[2] = Target_Effector.transform.position.z;
        // Fill up rotations
        Temp_coordinates[3] = Target_Effector.transform.rotation.eulerAngles.x * Mathf.Deg2Rad; 
        Temp_coordinates[4] = Target_Effector.transform.rotation.eulerAngles.y * Mathf.Deg2Rad; 
        Temp_coordinates[5] = Target_Effector.transform.rotation.eulerAngles.z * Mathf.Deg2Rad; 

        for (int ii = 0; ii<= 5; ii ++)
        {
         Coordinates_Matrix[ii] = Temp_coordinates[ii];
        }



        if (Robot_Kinematics != null)
        {
            Theta = Robot_Kinematics.inverse_kin(Coordinates_Matrix);
        }
        else
        {
        Robot_Kinematics = new robot_kinematics(RobotType.UR10e, coordinateSystem.xzy_L);
        }


        for (int i = 0; i <= 5 ; i++)
        {
            Joints_Angles[i] = Mathf.PI - (float)Theta[i][0];
            Joints_Angles[i] = Joints_Angles[i] * Mathf.Rad2Deg;
        }

    }
}
