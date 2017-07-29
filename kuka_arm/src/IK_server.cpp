#include <kuka_arm/trajectory_sampler.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

kuka_arm::CalculateIK srv;

bool  handle_calculate_IK(srv.request.poses& poses, srv.response.points& points)
{
    //std::vector<geometry_msgs::Pose>& request, std::vector<double>& response
    //geometry_msgs::Pose& request, trajectory_msgs::JointTrajectory& response)
    ROS_INFO("Received %s eef-poses from the plan" % poses.size());
    if (poses.size() < 1){
        ROS_INFO("No valid poses received");
        return false;
    }
    else{
    std::vector<double> joint_trajectory_list;
    double pi = 3.1415;
    for (std::size_t x = 0; x < srv.response.points.size(); ++x)
    {
        trajectory_msgs::JointTrajectoryPoint * joint_trajectory_point = new JointTrajectoryPoint();
        
        double px = poses[x].position.x;
        double py = poses[x].position.y;
        double pz = poses[x].position.z;
        double theta1, theta2, theta3, theta4, theta5, theta6;
        double yaw, pitch, roll;
        /*
        IK code starts here
        */
        //Arrays
        double d6_g = 0.303;

        double R_ypr_adjusted[3][3] = {
            {sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw), -sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll), cos(pitch)*cos(yaw)},
            {sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw), -sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw), sin(yaw)*cos(pitch)},
            {cos(pitch)*cos(roll), -sin(roll)*cos(pitch), -sin(pitch)}
        };

        wc_x = px - d6_g *  R_ypr_adjusted[0][2];
        wc_y = py - d6_g * R_ypr_adjusted[1][2];
        wc_z = pz - d6_g * R_ypr_adjusted[2][2];

        theta1 = atan2(wc_y, wc_x);
        double a1 = 0.35;
        double a2 = 1.25;
        double d1 = 0.75;

        double distance_j3_j4 = sqrt(0.96**2 + (-0.054)**2);
        double distance_j4_j5 = 0.54

        double a3 = -0.054;

        double angle_j3_j4 = pi - asin(a3 / distance_j3_j4);
        
        double distance_j3_j5 = sqrt(distance_j3_j4**2 + distance_j4_j5**2 - 2*distance_j3_j4*distance_j4_j5*cos(angle_j3_j4));

        double j2_x = a1 * cos(theta1);

        double j2_y = a1 * sin(theta1);

        double distance_j2_j5 = sqrt((wc_x - j2_x)**2 + (wc_y - j2_y)**2 + (wc_z - d1)**2);

        theta2 = pi/2 -  acos((distance_j3_j5**2 - a2**2 - distance_j2_j5**2)/(-2*a2*distance_j2_j5)) - atan2(wc_z - d1, sqrt((wc_x - j2_x)**2 + (wc_y - j2_y)**2));

        theta3 = pi/2 - asin(a3/distance_j3_j5) - acos((distance_j2_j5**2 - a2**2 - distance_j3_j5**2)/(-2*a2*distance_j3_j5));

        double R36 [3][3]= {
            {-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)},
            {                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)},
            {-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)}
        };

        //mathmodule?
        theta4 = atan2(R3_6[2][2], -R3_6[0][2]);
        theta5 = atan2(sqrt(R3_6[0][2]**2 + R3_6[2][2]**2) , R3_6[1][2]);
        theta6 = atan2(-R3_6[1][1], R3_6[1][0]);

        //mod this
        joint_trajectory_point->popositions = [theta1, theta2, theta3, theta4, theta5, theta6];

        joint_trajectory_list.append(joint_trajectory_point);
    }
    ROS_INFO("length of Joint Trajectory List: %s" % len(joint_trajectory_list));
    return true;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "IK_server");
    ros::NodeHandle nh;
    //ros::ServiceClient client = nh.serviceClient<kuka_arm::CalculateIK>("calculate_ik");
    ros::ServiceServer service = nh.advertiseService("calculate_ik", handle_calculate_IK);
    //s = ros.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    ROS_INFO("Ready to receive an IK request")
    
    ros::spin();

    return 0;
}