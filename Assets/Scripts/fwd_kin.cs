using UnityEngine;
using ur_kinematics;

public class fwd_kin : MonoBehaviour
{
    [SerializeField] GameObject Base_Robot;
    robot_kinematics Robot_Kinematics;
    IK_David Base_IK_Script;
    private Transform thiObj_transform;
    void Start()
    {
        Base_IK_Script = Base_Robot.GetComponent<IK_David>();
        Robot_Kinematics = new robot_kinematics(RobotType.UR10e, coordinateSystem.xzy_L);
    }

    // Update is called once per frame
    void Update()
    {
        var trans = Robot_Kinematics.forward_kin(
            new double[] 
            {
                Base_IK_Script.Joints_Angles[0],
                Base_IK_Script.Joints_Angles[1],
                Base_IK_Script.Joints_Angles[2],
                Base_IK_Script.Joints_Angles[3],
                Base_IK_Script.Joints_Angles[4],
                Base_IK_Script.Joints_Angles[5],
            }

        );

        this.transform.position = new Vector3( (float)trans[0], (float)trans[1], (float)trans[2]);
        this.transform.rotation = Quaternion.Euler((float)trans[3], (float)trans[4], (float)trans[5]);
    }
}
