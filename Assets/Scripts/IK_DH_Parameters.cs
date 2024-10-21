using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UIElements;

public class IK_DH_Parameters : MonoBehaviour
{

    // DH parameters for UR10e from https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    static float[] theta = new float[5];
    public float[] a = {0, -0.6127f, -0.3922f, 0, 0, 0};
    public float[] d = {0.1625f, 0, 0, 	0.1333f, 0.0997f, 0.0996f};
    public double[] alpha = {Mathf.PI/2.0, 0, 0 , Mathf.PI/2.0, -Mathf.PI/2.0,0} ;
    public  float[] Joints_Angles = new float[5];


    public Transform Target_Effector;


    private Vector3 Position_Matrix;
    private Matrix4x4 Rotation_Matrix;


   private void Theta_First_Joint()
   {
        theta[0] = Mathf.Atan2(Position_Matrix.y, Position_Matrix.x);
   }

   private void Position_Second_Joint()
   {
        float[] P2 = {Position_Matrix.x - d[5], Position_Matrix.y - d[5], Position_Matrix.z};
        float R =  Mathf.Sqrt(MathF.Pow(P2[0],2) + MathF.Pow(P2[1],2) + MathF.Pow(P2[2],2));       
        theta[1] = Mathf.Atan2(P2[2], Mathf.Sqrt(Mathf.Pow(P2[0],2) +  Mathf.Pow(P2[1],2) )) ; 
        theta[2] = Mathf.Acos(Mathf.Abs(Mathf.Pow(R, 2) - Mathf.Pow(a[1],2) - Mathf.Pow(a[2],2)))/(2 * -a[1] * -a[2]);
   }

   private void Position_Third_Joint()
   {
       
   }
    

    // Used for transformation of theta 
    private void Rad_To_Deg()
    { 
      //  print(theta[0]);
        for(int i = 0; i < Joints_Angles.Length; i++)
        {
        Joints_Angles[i] = theta[i] * Mathf.Rad2Deg;
        Console.WriteLine("joint 1{0}", Joints_Angles[1]);
        }
    }


    // Start is called before the first frame update
    void Start()
    {
        Position_Matrix = Target_Effector.position;
        Rotation_Matrix = Matrix4x4.Rotate(Target_Effector.rotation);
        
    }

    // Update is called once per frame
    void Update()
    {
        Position_Matrix = Target_Effector.position;
        
        Theta_First_Joint();
        Position_Second_Joint();
            

        Rad_To_Deg();
        print (Joints_Angles[2]);
    //    print(Joints_Angles[0]);


    }
}
