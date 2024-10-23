using System;
using System.Collections.Generic;
//using System.ComponentModel.DataAnnotations.Schema;
using System.Data;
using System.Linq;
//using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;
using System.Transactions;
using mitoSoft.Matrices;
namespace ur_kinematics
{
    public enum RobotType
    {
        UR5,
        UR10,
        UR20,
        UR10e
    }

    public enum coordinateSystem
    {
        xyz,
        xzy_L
    }

    public class robot_kinematics
    {

        private double[] theta;
        private double[] alpha;
        private double[] a;
        private double[] d;
        private string coordinate_sys = "";
        double[][] R_matrix = new double[3][];


        private List<double[][]> t_matrices = new List<double[][]>();
        double[][] eye_matrix = new double[3][]
        {
            new double[] {0,0,0},
            new double[] {0,0,0},
            new double[] {0,0,0}
        };
        double[][] m0 = new double[3][]
        {
            new double[] {0},
            new double[] {0},
            new double[] {0},
        };
        double[][] m1 = new double[3][]
        {
            new double[] {0},
            new double[] {0},
            new double[] {1},
        };
        matrix_math matrixM;

        public robot_kinematics(RobotType type, coordinateSystem coordinate_system)
        {
            switch (coordinate_system)
            {
                case coordinateSystem.xzy_L:
                    R_matrix = new double[3][]
                    {
                        //new double[3]{ 1, 0, 0},
                        //new double[3]{0, 0, -1},
                        //new double[3]{0, 1, 0}
                        new double[3]{ 1, 0, 0},
                        new double[3]{0, 0, 1},
                        new double[3]{0, 1, 0}
                    };
                    break;
                case coordinateSystem.xyz:
                    R_matrix = new double[3][]
                    {
                        new double[3]{ 1, 0, 0},
                        new double[3]{0, 1, 0},
                        new double[3]{0, 0, 1}
                    };
                    break;
            }
            switch (type)
            {
                case RobotType.UR20:
                    this.theta = new double[6] { 0, 0, 0, 0, 0, 0 };
                    this.alpha = new double[6] { Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0 };
                    this.a = new double[6] { 0, -0.8620, -0.7287, 0, 0, 0 };
                    this.d = new double[6] { 0.2363, 0, 0, 0.2010, 0.1593, 0.1543 };
                    break;
                case RobotType.UR10e:
                    this.theta = new double[6] { 0, 0, 0, 0, 0, 0 };
                    this.alpha = new double[6] { Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0 };
                    this.a = new double[6] { 0, -0.6127, -0.57155, 0, 0, 0 };
                    this.d = new double[6] { 0.1807, 0, 0, 0.17415, 0.11985, 0.11655 };
                    break;
            }

            matrixM = new matrix_math();
        }

        private double[][] get_homog_t(double[] in_vector)
        {
            double[][] out_transform = new double[4][]
            {
                new double[4]{0,0,0,0},
                new double[4]{0,0,0,0},
                new double[4]{0,0,0,0},
                new double[4]{0,0,0,0}
            };

            double[][] Rx = new double[3][]
            {
                new double[3]{ 1, 0, 0},
                new double[3]{0, Math.Cos(in_vector[5]), -Math.Sin(in_vector[5])},
                new double[3]{0, Math.Sin(in_vector[5]), Math.Cos(in_vector[5])}
            };
            double[][] Ry = new double[3][]
            {
                new double[3]{ Math.Cos(in_vector[4]), 0, Math.Sin(in_vector[4]) },
                new double[3]{ 0, 1, 0},
                new double[3]{ -Math.Sin(in_vector[4]), 0, Math.Cos(in_vector[4]) }
            };
            double[][] Rz = new double[3][]
            {
                new double[3]{ Math.Cos(in_vector[3]), -Math.Sin(in_vector[3]), 0},
                new double[3]{ Math.Sin(in_vector[3]), Math.Cos(in_vector[3]), 0},
                new double[3]{ 0, 0, 1}
            };

            var Rzy = matrixM.MultiplyMatrices(Rz, Ry);
            double[][] Rot_matrix = matrixM.MultiplyMatrices(Rzy, Rx);

            ////Switch basis
            //Rot_matrix = switch_basis(Rot_matrix);

            //out_transform[0][0] = Rot_matrix[0][0];
            //out_transform[0][1] = Rot_matrix[0][1];
            //out_transform[0][2] = Rot_matrix[0][2];
            //out_transform[0][3] = in_vector[0];

            //out_transform[1][0] = Rot_matrix[1][0];
            //out_transform[1][1] = Rot_matrix[1][1];
            //out_transform[1][2] = Rot_matrix[1][2];
            //out_transform[1][3] = in_vector[2];

            //out_transform[2][0] = Rot_matrix[2][0];
            //out_transform[2][1] = Rot_matrix[2][1];
            //out_transform[2][2] = Rot_matrix[2][2];
            //out_transform[2][3] = in_vector[1];

            //out_transform[3][3] = 1.0;


            //return out_transform;

            var homog_t = new double[4][]
            {
                new double[4]{ Rot_matrix[0][0], Rot_matrix[0][1], Rot_matrix[0][2], in_vector[0] },
                new double[4]{ Rot_matrix[1][0], Rot_matrix[1][1], Rot_matrix[1][2], in_vector[1] },
                new double[4]{ Rot_matrix[2][0], Rot_matrix[2][1], Rot_matrix[2][2], in_vector[2] },
                new double[4]{0, 0, 0, 1}
            };

            return switch_transform_basis(homog_t);

        }

        double[][] switch_basis(double[][] in_matrix)
        {
            var r_1 = matrixM.InvertMatrix(R_matrix);
            var ar = matrixM.MultiplyMatrices(in_matrix, R_matrix);
            return matrixM.MultiplyMatrices(r_1, ar);
        }

        double[] switch_basis(double[] in_vector)
        {
            var r_1 = matrixM.InvertMatrix(R_matrix);
            return matrixM.MultiplyMatrices(r_1, in_vector);
        }

        private double[][] switch_transform_basis(double[][] transform)
        {
            double[][] rot = new double[3][]
            {
                new double[3]{ transform[0][0], transform[0][1], transform[0][2] },
                new double[3]{ transform[1][0], transform[1][1], transform[1][2] },
                new double[3]{ transform[2][0], transform[2][1], transform[2][2] }
            };
            double[] trans = new double[3]
            {
                transform[0][3],
                transform[1][3],
                transform[2][3]
            };
            var transform_rot = switch_basis(rot); //matrix
            var transform_trans = switch_basis(trans); //vector

            return new double[4][]
            {
                new double[] { transform_rot[0][0], transform_rot[0][1], transform_rot[0][2], transform_trans[0] },
                new double[] { transform_rot[1][0], transform_rot[1][1], transform_rot[1][2], transform_trans[1] },
                new double[] { transform_rot[2][0], transform_rot[2][1], transform_rot[2][2], transform_trans[2] },
                new double[] { 0, 0, 0, 1}
            };
        }

        private double[] get_pose_vector(double[][] homog_t)
        {
            double rot_y = Math.Atan2(Math.Sqrt(Math.Pow(homog_t[2][1], 2) + Math.Pow(homog_t[2][0], 2)), -homog_t[2][0]);
            double rot_x = Math.Atan2(homog_t[0][0] / Math.Cos(rot_y), homog_t[1][0] / Math.Cos(rot_y));
            double rot_z = Math.Atan2(homog_t[2][2] / Math.Cos(rot_y), homog_t[2][1] / Math.Cos(rot_y));
            return new double[]
           {
                homog_t[0][3],
                homog_t[1][3],
                homog_t[2][3],
                rot_x * 180 / Math.PI,
                rot_y * 180 / Math.PI,
                rot_z * 180 / Math.PI
           };
        }



        public double[] forward_kin(double[] jointsRot_deg)
        {
            //double[][] t01 = new double[4][]
            //{
            //    new double[] { Math.Cos(jointsRot_deg[0] * Math.PI / 180), -Math.Sin(jointsRot_deg[0] * Math.PI / 180), 0, 0 },
            //    new double[] { Math.Sin(jointsRot_deg[0] * Math.PI / 180), Math.Cos(jointsRot_deg[0] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 1, 0, },
            //    new double[] { 0, 0, 0, 1 }
            //};
            //double[][] t12 = new double[4][]
            //{
            //    new double[] { -Math.Sin(jointsRot_deg[1] * Math.PI / 180), -Math.Cos(jointsRot_deg[1] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, -1, 0, },
            //    new double[] { Math.Cos(jointsRot_deg[1] * Math.PI / 180), -Math.Sin(jointsRot_deg[1] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 0, 1 }
            //};
            //double[][] t23 = new double[4][]
            //{
            //    new double[] { Math.Cos(jointsRot_deg[2] * Math.PI / 180), -Math.Sin(jointsRot_deg[2] * Math.PI / 180), 0, a[1] },
            //    new double[] { Math.Sin(jointsRot_deg[2] * Math.PI / 180), Math.Cos(jointsRot_deg[2] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 1, 0, },
            //    new double[] { 0, 0, 0, 1 }
            //};
            //double[][] t34 = new double[4][]
            //{
            //    new double[] { Math.Sin(jointsRot_deg[3] * Math.PI / 180), Math.Cos(jointsRot_deg[3] * Math.PI / 180), 0, a[2] },
            //    new double[] { -Math.Cos(jointsRot_deg[3] * Math.PI / 180), Math.Sin(jointsRot_deg[3] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 1, d[3] },
            //    new double[] { 0, 0, 0, 1 }
            //};
            //double[][] t45 = new double[4][]
            //{
            //    new double[] { Math.Cos(jointsRot_deg[4] * Math.PI / 180), -Math.Sin(jointsRot_deg[4] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 1, d[4] },
            //    new double[] { -Math.Sin(jointsRot_deg[4] * Math.PI / 180), -Math.Cos(jointsRot_deg[4] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 0, 1 }
            //};
            //double[][] t56 = new double[4][]
            //{
            //    new double[] { Math.Cos(jointsRot_deg[5] * Math.PI / 180), -Math.Sin(jointsRot_deg[5] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, -1, 0 },
            //    new double[] { Math.Sin(jointsRot_deg[5] * Math.PI / 180), Math.Cos(jointsRot_deg[5] * Math.PI / 180), 0, 0 },
            //    new double[] { 0, 0, 0, 1 }
            //};

            //List<double[][]> transforms = new List<double[][]>();
            //transforms.Add(t01);
            //transforms.Add(t12);
            //transforms.Add(t23);
            //transforms.Add(t34);
            //transforms.Add(t45);
            //transforms.Add(t56);
            //double[][] outMat = switch_transform_basis(transforms[0]);
            //for (int i = 1; i < theta.Length; i++)
            //{
            //    outMat = matrixM.MultiplyMatrices(outMat, switch_transform_basis(transforms[i]));
            //}

            //return get_pose_vector(outMat);

            double c1 = Math.Cos(jointsRot_deg[0] * Math.PI / 180);
            double c2 = Math.Cos(jointsRot_deg[1] * Math.PI / 180);
            double c3 = Math.Cos(jointsRot_deg[2] * Math.PI / 180);
            double c4 = Math.Cos(jointsRot_deg[3] * Math.PI / 180);
            double c5 = Math.Cos(jointsRot_deg[4] * Math.PI / 180);
            double c6 = Math.Cos(jointsRot_deg[5] * Math.PI / 180);

            double s1 = Math.Sin(jointsRot_deg[0] * Math.PI / 180);
            double s2 = Math.Sin(jointsRot_deg[1] * Math.PI / 180);
            double s3 = Math.Sin(jointsRot_deg[2] * Math.PI / 180);
            double s4 = Math.Sin(jointsRot_deg[3] * Math.PI / 180);
            double s5 = Math.Sin(jointsRot_deg[4] * Math.PI / 180);
            double s6 = Math.Sin(jointsRot_deg[5] * Math.PI / 180);

            double s23 = Math.Sin((jointsRot_deg[1] * Math.PI / 180) + (jointsRot_deg[2] * Math.PI / 180));
            double c23 = Math.Cos((jointsRot_deg[1] * Math.PI / 180) + (jointsRot_deg[2] * Math.PI / 180));

            double s234 = Math.Sin((jointsRot_deg[1] * Math.PI / 180) + (jointsRot_deg[2] * Math.PI / 180) + (jointsRot_deg[3] * Math.PI / 180));
            double c234 = Math.Cos((jointsRot_deg[1] * Math.PI / 180) + (jointsRot_deg[2] * Math.PI / 180) + (jointsRot_deg[3] * Math.PI / 180));

            var r11 = -s1 * s5 * c6 + c1 * (-s234 * s6 + c234 * c5 * c6);
            var r21 = c1 * s5 * c6 + s1 * (-s234 * s6 + c234 * c5 * c6);
            var r31 = c234 * s6 + s234 * c5 * c6;
            var r12 = s1 * s5 * s6 - c1 * (s234 * c6 + c234 * c5 * s6);
            var r22 = -c1 * s5 * s6 - s1 * (s234 * c6 + c234 * c5 * s6);
            var r32 = c234 * c6 - s234 * c5 * s6;
            var r13 = s1 * c5 + c1 * c234 * s5;
            var r23 = -c1 * c5 + s1 * s234 * s5;
            var r33 = s234 * s5;

            var x = d[3] * s1 - c1 * (a[1] * s2 + a[2] * s23 + d[4] + s234);
            var y = -d[3] * c1 - s1 * (a[1] * s2 + a[2] * s23 + d[4] * s234);
            var z = a[1] * c2 + a[2] * c23 + d[4] * c234;

            double[][] transform_matrix = new double[4][]
            {
                new double[] { r11, r12, r13, x },
                new double[] { r21, r22, r23, y },
                new double[] { r31, r32, r33, z },
                new double[] { 0, 0, 0, 1 }
            };

            //return get_pose_vector(switch_transform_basis(transform_matrix));

            return get_pose_vector(transform_matrix);
        }


        //https://people.ohio.edu/williams/html/PDF/UniversalRobotKinematics.pdf

        public double[][] inverse_kin(double[] endPoint)
        {
            double[][] jointAngles = new double[theta.Length][];
            double[][] endPoint_transform = new double[theta.Length][];
            double[][] Th0 = new double[3][];

            endPoint_transform = get_homog_t(endPoint);
            Th0 = get_homog_t(new double[] { 0, 0.187, 0, 0, 0, 0 });

            endPoint_transform = matrixM.MultiplyMatrices(matrixM.InvertMatrix(Th0), endPoint_transform);

            var x6 = -endPoint_transform[0][3];
            var y6 = endPoint_transform[1][3];
            var d6 = d[3];


            double[] th12 = new double[]
            {
                (-x6 + Math.Sqrt(Math.Pow(y6, 2) + Math.Pow(x6, 2) - Math.Pow(d6, 2))) / (d6 - y6),
                (-x6 - Math.Sqrt(Math.Pow(y6, 2) + Math.Pow(x6, 2) - Math.Pow(d6, 2))) / (d6 - y6)
            };

            double[] theta1 = new double[]
            {
                2 * Math.Atan(th12[0]),
                2 * Math.Atan(th12[1])
            }; // THETA1 has 2 options

            //-----------------

            var th6_1_th0 = endPoint_transform[0][1] * Math.Sin(theta1[0]) - endPoint_transform[1][1] * Math.Cos(theta1[0]);
            var th6_2_th0 = endPoint_transform[1][0] * Math.Cos(theta1[0]) - endPoint_transform[0][0] * Math.Sin(theta1[0]);
            var th6_1_th1 = endPoint_transform[0][1] * Math.Sin(theta1[1]) - endPoint_transform[1][1] * Math.Cos(theta1[1]);
            var th6_2_th1 = endPoint_transform[1][0] * Math.Cos(theta1[1]) - endPoint_transform[0][0] * Math.Sin(theta1[1]);

            double[] theta6 = new double[]
            {
                Math.Atan2(th6_1_th0, th6_2_th0),
                Math.Atan2(th6_1_th1, th6_2_th1)
            };

            //------------------

            var th5_1_th0 = (endPoint_transform[1][0] * Math.Cos(theta1[0]) - endPoint_transform[0][0] * Math.Sin(theta1[0])) * Math.Cos(theta6[0]) + (endPoint_transform[0][1] * Math.Sin(theta1[0]) - endPoint_transform[1][1] * Math.Cos(theta1[0])) * Math.Sin(theta6[0]);
            var th5_2_th0 = endPoint_transform[0][2] * Math.Sin(theta1[0]) - endPoint_transform[1][2] * Math.Cos(theta1[0]);

            var th5_1_th1 = (endPoint_transform[1][0] * Math.Cos(theta1[1]) - endPoint_transform[0][0] * Math.Sin(theta1[1])) * Math.Cos(theta6[1]) + (endPoint_transform[0][1] * Math.Sin(theta1[1]) - endPoint_transform[1][1] * Math.Cos(theta1[1])) * Math.Sin(theta6[1]);
            var th5_2_th1 = endPoint_transform[0][2] * Math.Sin(theta1[1]) - endPoint_transform[1][2] * Math.Cos(theta1[1]);

            double[] theta5 = new double[]
            {
                Math.Atan2(th5_1_th0, th5_2_th0),
                Math.Atan2(th5_1_th1, th5_2_th1)
            };


            //-------------

            var A_1 = (endPoint_transform[2][0] * Math.Cos(theta6[0]) - endPoint_transform[2][1] * Math.Sin(theta6[0])) / Math.Cos(theta5[0]);
            var A_2 = (endPoint_transform[2][0] * Math.Cos(theta6[1]) - endPoint_transform[2][1] * Math.Sin(theta6[1])) / Math.Cos(theta5[1]);

            var B_1 = endPoint_transform[2][1] * Math.Cos(theta6[0]) + endPoint_transform[2][0] * Math.Sin(theta6[0]);
            var B_2 = endPoint_transform[2][1] * Math.Cos(theta6[1]) + endPoint_transform[2][0] * Math.Sin(theta6[1]);

            var alp_1 = -endPoint_transform[0][3] * Math.Cos(theta1[0]) - endPoint_transform[1][3] * Math.Sin(theta1[0]) - d[4] * A_1;
            var alp_2 = -endPoint_transform[0][3] * Math.Cos(theta1[1]) - endPoint_transform[1][3] * Math.Sin(theta1[1]) - d[4] * A_2;

            var bet_1 = endPoint_transform[2][3] - d[4] * B_1;
            var bet_2 = endPoint_transform[2][3] - d[4] * B_2;

            var E_1 = -2 * a[1] * bet_1;
            var E_2 = -2 * a[1] * bet_2;

            var F_1 = -2 * a[1] * alp_1;
            var F_2 = -2 * a[1] * alp_2;


            var G_1 = Math.Pow(a[1], 2) + Math.Pow(alp_1, 2) + Math.Pow(bet_1, 2) - Math.Pow(a[2], 2);
            var G_2 = Math.Pow(a[1], 2) + Math.Pow(alp_2, 2) + Math.Pow(bet_2, 2) - Math.Pow(a[2], 2);

            double[] th23 = new double[]
            {
                (-F_1 + Math.Sqrt(Math.Pow(E_1, 2) + Math.Pow(F_1, 2) - Math.Pow(G_1, 2))) / (G_1 - E_1),
                (-F_1 - Math.Sqrt(Math.Pow(E_1, 2) + Math.Pow(F_1, 2) - Math.Pow(G_1, 2))) / (G_1 - E_1),
                (-F_2 + Math.Sqrt(Math.Pow(E_2, 2) + Math.Pow(F_2, 2) - Math.Pow(G_2, 2))) / (G_2 - E_2),
                (-F_2 - Math.Sqrt(Math.Pow(E_2, 2) + Math.Pow(F_2, 2) - Math.Pow(G_2, 2))) / (G_2 - E_2)
            };


            double[] theta2 = new double[]
            {
                2 * Math.Atan(th23[0]),
                2 * Math.Atan(th23[1]),
                2 * Math.Atan(th23[2]),
                2 * Math.Atan(th23[3])
            };



            //-----------------

            var th3_1_a = alp_1 - a[1] * Math.Sin(theta2[0]);
            var th3_1_b = bet_1 - a[1] * Math.Cos(theta2[0]);

            var th3_2_a = alp_1 - a[1] * Math.Sin(theta2[1]);
            var th3_2_b = bet_1 - a[1] * Math.Cos(theta2[1]);

            var th3_3_a = alp_2 - a[1] * Math.Sin(theta2[2]);
            var th3_3_b = bet_2 - a[1] * Math.Cos(theta2[2]);

            var th3_4_a = alp_2 - a[1] * Math.Sin(theta2[3]);
            var th3_4_b = bet_2 - a[1] * Math.Cos(theta2[3]);

            double[] theta3 = new double[]
            {
                Math.Atan2(th3_1_a, th3_1_b) - theta2[0],
                Math.Atan2(th3_2_a, th3_2_b) - theta2[1],
                Math.Atan2(th3_3_a, th3_3_b) - theta2[2],
                Math.Atan2(th3_4_a, th3_4_b) - theta2[3]
            };


            //-----------------

            double[] theta4 = new double[]
            {
                Math.Atan2(A_1, B_1) - theta2[0] - theta3[0],
                Math.Atan2(A_1, B_1) - theta2[1] - theta3[1],
                Math.Atan2(A_2, B_2) - theta2[2] - theta3[2],
                Math.Atan2(A_2, B_2) - theta2[3] - theta3[3]
            };


            jointAngles[0] = new double[theta1.Length];
            jointAngles[1] = new double[theta2.Length];

            jointAngles[2] = new double[theta3.Length];
            jointAngles[3] = new double[theta4.Length];

            jointAngles[4] = new double[theta5.Length];
            jointAngles[5] = new double[theta6.Length];

            jointAngles[0] = theta1;
            jointAngles[1] = theta2;
            jointAngles[2] = theta3;
            jointAngles[3] = theta4;
            jointAngles[4] = theta5;
            jointAngles[5] = theta6;

            return jointAngles;
        }
    }
}


