#include <kuka_arm/trajectory_sampler.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

kuka_arm::CalculateIK srv;

bool  handle_calculate_IK(std::vector<geometry_msgs::Pose>& poses, std::vector<double>& response);

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "IK_server");
    ros::NodeHandle nh;
    //ros::ServiceClient client = nh.serviceClient<kuka_arm::CalculateIK>("calculate_ik");
    ros::ServiceServer service = nh.advertiseService("calculate_ik", handle_calculate_IK);
    //s = ros.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    ROS_INFO("Ready to receive an IK request");
    
    ros::spin();

    return 0;
}


bool  handle_calculate_IK(std::vector<geometry_msgs::Pose>& poses, std::vector<trajectory_msgs::JointTrajectoryPoint>& response)
{
    //std::vector<geometry_msgs::Pose>& request, std::vector<double>& response
    //geometry_msgs::Pose& request, trajectory_msgs::JointTrajectory& response)
    ROS_INFO("Received %d eef-poses from the plan",poses.size());
    if (poses.size() < 1){
        ROS_INFO("No valid poses received");
        return false;
    }
    else
    {
        std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_list;
        double pi = 3.1415;
        for (std::size_t x = 0; x < poses.size(); ++x)
        {
            trajectory_msgs::JointTrajectoryPoint joint_trajectory_point = trajectory_msgs::JointTrajectoryPoint();
            
            double px = poses[x].position.x;
            double py = poses[x].position.y;
            double pz = poses[x].position.z;
            double theta1, theta2, theta3, theta4, theta5, theta6;
            double yaw, pitch, roll;
            /*
            IK code starts here
            */
            double p[4] = {poses[x].orientation.x, poses[x].orientation.y, poses[x].orientation.z, poses[x].orientation.w};

            //creating 
            double pdotp = p[0]*p[0] + p[1]*p[1] +p[2]*p[2] +p[3]*p[3];
            double M[4][4] = {{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,1.0}};
            if(pdotp < 0.00000000001){
                M[0][0] = 1.0;
                M[1][1] = 1.0;
                M[2][2] = 1.0;
                M[3][3] = 1.0;
            }
            else{
                double nq = sqrt(2.0 / pdotp);
                p[0] *= nq;
                p[1] *= nq;            
                p[2] *= nq;
                p[3] *= nq;
            
                /* more memory usage and population time overheads
                double q[4][4] = {
                    {p[0]*p[0], p[0]*p[1], p[0]*p[2], p[0]*p[3] },
                    {p[1]*p[0], p[1]*p[1], p[1]*p[2], p[1]*p[3] },
                    {p[2]*p[0], p[2]*p[1], p[2]*p[2], p[2]*p[3] },
                    {p[3]*p[0], p[3]*p[1], p[3]*p[2], p[3]*p[3] }
                }

                double q_matrix[4][4] = {
                    {1.0-q[1][1]-q[2][2],     q[0][1]-q[2][3],     q[0][2]+q[1][3], 0.0},
                    {    q[0][1]+q[2][3], 1.0-q[0][0]-q[2][2],     q[1][2]-q[0][3], 0.0},
                    {    q[0][2]-q[1][3],     q[1][2]+q[0][3], 1.0-q[0][0]-q[1][1], 0.0},
                    {                0.0,                 0.0,                 0.0, 1.0}
                }
                */
                M[0][0] = 1.0-p[1]*p[1]-p[2]*p[2];
                M[0][1] = p[0]*p[1]-p[2]*p[3];
                M[0][2] = p[0]*p[2]+p[1]*p[3];
                M[1][0] = p[0]*p[1]+p[2]*p[3];
                M[1][1] = 1.0-p[0]*p[0]-p[2]*p[2];
                M[1][2] = p[1]*p[2]-p[0]*p[3];
                M[2][0] = p[0]*p[2]-p[1]*p[3];
                M[2][1] = p[1]*p[2]+p[0]*p[3];
                M[2][2] = 1.0-p[0]*p[0]-p[1]*p[1];

            }

                double cy = sqrt(M[0][0]*M[0][0] + M[1][0]*M[1][0]);
                if (cy > 0.00000000001){
                    roll = atan2( M[2][1],  M[2][2]);
                    pitch = atan2(-M[2][0],  cy);
                    yaw = atan2( M[1][0],  M[0][0]);
                }
                else{
                    roll = atan2(-M[1][2],  M[1][1]);
                    pitch = atan2(-M[2][0],  cy);
                    yaw = 0.0;
                }
                 
            //port this one  errors here python code
            // (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            //        [poses[x].orientation.x, poses[x].orientation.y,
            //            poses[x].orientation.z, poses[x].orientation.w])
            //Arrays
            double d6_g = 0.303;

            double R_ypr_adjusted[3][3] = {
                {sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw), -sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll), cos(pitch)*cos(yaw)},
                {sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw), -sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw), sin(yaw)*cos(pitch)},
                {cos(pitch)*cos(roll), -sin(roll)*cos(pitch), -sin(pitch)}
            };

            double wc_x = px - d6_g *  R_ypr_adjusted[0][2];
            double wc_y = py - d6_g * R_ypr_adjusted[1][2];
            double wc_z = pz - d6_g * R_ypr_adjusted[2][2];

            theta1 = atan2(wc_y, wc_x);
            double a1 = 0.35;
            double a2 = 1.25;
            double a3 = -0.054;
            double d1 = 0.75;

            double distance_j3_j4 = sqrt(pow(0.96,2) + pow((-0.054),2));
            double distance_j4_j5 = 0.54;

            double angle_j3_j4 = pi - asin(a3 / distance_j3_j4);
            
            double distance_j3_j5 = sqrt(pow(distance_j3_j4,2) + pow(distance_j4_j5,2) - 2*distance_j3_j4*distance_j4_j5*cos(angle_j3_j4));

            double j2_x = a1 * cos(theta1);

            double j2_y = a1 * sin(theta1);

            double distance_j2_j5 = sqrt(pow((wc_x - j2_x),2) + pow((wc_y - j2_y),2) + pow((wc_z - d1),2));

            theta2 = pi/2 -  acos((pow(distance_j3_j5,2) - pow(a2,2) - pow(distance_j2_j5,2)/(-2*a2*distance_j2_j5))) - atan2(wc_z - d1, sqrt(pow((wc_x - j2_x),2) + pow((wc_y - j2_y),2)));

            theta3 = pi/2 - asin(a3/distance_j3_j5) - acos((pow(distance_j2_j5,2) - pow(a2,2) - pow(distance_j3_j5,2))/(-2*a2*distance_j3_j5));

            double R3_6 [3][3]= {
                        {(1.0*sin(yaw)*sin(roll) + sin(pitch)*cos(yaw)*cos(roll))*sin(theta2 + theta3)*cos(theta1) + (sin(yaw)*sin(pitch)*cos(roll) - 1.0*sin(roll)*cos(yaw))*sin(theta1)*sin(theta2 + theta3) + 1.0*cos(pitch)*cos(roll)*cos(theta2 + theta3), -1.0*(-1.0*sin(yaw)*cos(roll) + sin(pitch)*sin(roll)*cos(yaw))*sin(theta2 + theta3)*cos(theta1) - 1.0*(sin(yaw)*sin(pitch)*sin(roll) + 1.0*cos(yaw)*cos(roll))*sin(theta1)*sin(theta2 + theta3) - 1.0*sin(roll)*cos(pitch)*cos(theta2 + theta3), 1.0*sin(yaw)*sin(theta1)*sin(theta2 + theta3)*cos(pitch) - 1.0*sin(pitch)*cos(theta2 + theta3) + 1.0*sin(theta2 + theta3)*cos(yaw)*cos(pitch)*cos(theta1)},
                        {(1.0*sin(yaw)*sin(roll) + sin(pitch)*cos(yaw)*cos(roll))*cos(theta1)*cos(theta2 + theta3) + (sin(yaw)*sin(pitch)*cos(roll) - 1.0*sin(roll)*cos(yaw))*sin(theta1)*cos(theta2 + theta3) - 1.0*sin(theta2 + theta3)*cos(pitch)*cos(roll), -1.0*(-1.0*sin(yaw)*cos(roll) + sin(pitch)*sin(roll)*cos(yaw))*cos(theta1)*cos(theta2 + theta3) - 1.0*(sin(yaw)*sin(pitch)*sin(roll) + 1.0*cos(yaw)*cos(roll))*sin(theta1)*cos(theta2 + theta3) + 1.0*sin(roll)*sin(theta2 + theta3)*cos(pitch), 1.0*sin(yaw)*sin(theta1)*cos(pitch)*cos(theta2 + theta3) + 1.0*sin(pitch)*sin(theta2 + theta3) + 1.0*cos(yaw)*cos(pitch)*cos(theta1)*cos(theta2 + theta3)},
                        {-(1.0*sin(yaw)*sin(roll) + sin(pitch)*cos(yaw)*cos(roll))*sin(theta1) + (sin(yaw)*sin(pitch)*cos(roll) - 1.0*sin(roll)*cos(yaw))*cos(theta1), 1.0*(-1.0*sin(yaw)*cos(roll) + sin(pitch)*sin(roll)*cos(yaw))*sin(theta1) - 1.0*(sin(yaw)*sin(pitch)*sin(roll) + 1.0*cos(yaw)*cos(roll))*cos(theta1), 1.0*sin(yaw)*cos(pitch)*cos(theta1) - 1.0*sin(theta1)*cos(yaw)*cos(pitch)}
            };

            //mathmodule?
            theta4 = atan2(R3_6[2][2], -R3_6[0][2]);
            theta5 = atan2(sqrt(pow(R3_6[0][2],2) + pow(R3_6[2][2],2)) , R3_6[1][2]);
            theta6 = atan2(-R3_6[1][1], R3_6[1][0]);

            //mod this errors here python code
            joint_trajectory_point.positions = {theta1, theta2, theta3, theta4, theta5, theta6};
            joint_trajectory_list.push_back(joint_trajectory_point);
        }
        ROS_INFO("length of Joint Trajectory List: %d", joint_trajectory_list.size());
    }
    return true;
}