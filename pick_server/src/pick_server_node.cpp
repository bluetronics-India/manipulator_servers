
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_server/PickAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>

#define TARGET_NAME "can"

// server wrapper for pick goals
typedef actionlib::SimpleActionServer<pick_server::PickAction> pick_server_t;

// moveit clients
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> pick_client_t;

moveit::planning_interface::PlanningSceneInterface *planning_scene_ptr;

moveit_msgs::PickupGoal BuildPickGoal(const std::string &objectName)
{
    moveit_msgs::PickupGoal goal;
    goal.target_name = objectName;
    goal.group_name = "arm";
    goal.end_effector = "eef";
    goal.allowed_planning_time = 15.0;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.replan=true;
    goal.planning_options.replan_attempts=5;
    goal.planner_id = "RRTConnectkConfigDefault";

    goal.minimize_object_distance = true;
    moveit_msgs::Grasp g;
    g.max_contact_force = 1.0; //0.01
    g.grasp_pose.header.frame_id = goal.target_name;
    g.grasp_pose.pose.position.x = -0.02;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = 0.0;
    g.grasp_pose.pose.orientation.x = 0.0;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = "/base_footprint"; //gripper_link
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.post_grasp_retreat.direction.header.frame_id = "/base_footprint"; //gripper_link
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size());
    g.pre_grasp_posture.points[0].positions[0] = 0.14;

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].effort[0] = 0.6;
    goal.possible_grasps.push_back(g);
    return goal;
}


void executeCB(const pick_server::PickGoalConstPtr& goal, pick_server_t* as)
{

    pick_client_t pick_client("pickup", true);

    moveit_msgs::PickupGoal pickGoal = BuildPickGoal("can");
    actionlib::SimpleClientGoalState pickStatus = pick_client.sendGoalAndWait(pickGoal);
/*
    ROS_INFO("[pick_server]: handling client request...");

    pick_client_t pick_client("pickup", true);
    ROS_INFO("[pick_server]: waiting for Moveit pickup server");
    pick_client.waitForServer();
    ROS_INFO("[pick_server]: got Moveit pickup server");

    moveit_msgs::PickupGoal pu_goal;

    std::vector<moveit_msgs::CollisionObject> col_objects;
    moveit_msgs::CollisionObject target_obj;
    target_obj.id = TARGET_NAME;

    // define object primitives (size and shape)
    shape_msgs::SolidPrimitive cylinder_primitives;
    cylinder_primitives.type = cylinder_primitives.CYLINDER;
    cylinder_primitives.dimensions.resize(2);
    cylinder_primitives.dimensions[0] = 0.145;
    cylinder_primitives.dimensions[1] = 0.03;

    target_obj.primitives.push_back(cylinder_primitives);

    // define object position and orientation
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.7; //goal->x;
    target_pose.position.y = 0.0; //goal->y;
    target_pose.position.z = 0.4; //goal->z;

    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

    target_obj.primitive_poses.push_back(target_pose);

    target_obj.operation = target_obj.ADD;
    target_obj.header.frame_id = "/base_footprint";

    col_objects.push_back(target_obj);
    //planning_scene_ptr->addCollisionObjects(col_objects);

// continue here:
    //https://github.com/robotican/armadillo_2.0/blob/master/armadillo2_demos/object_detection/src/pick_and_place/objects_handler.cpp#L154

    pu_goal.target_name = TARGET_NAME;
    pu_goal.group_name = "arm";
    pu_goal.end_effector = "eef";
    pu_goal.allowed_planning_time = 15.0;
    pu_goal.planning_options.replan_delay = 2.0;
    pu_goal.planning_options.planning_scene_diff.is_diff = true;
    pu_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    pu_goal.planning_options.replan=true;
    pu_goal.planning_options.replan_attempts=5;
    pu_goal.planner_id = "RRTConnectkConfigDefault";
    pu_goal.minimize_object_distance = true;

    moveit_msgs::Grasp grasp;
    grasp.max_contact_force = 1.0; //0.01

    grasp.pre_grasp_approach.direction.header.frame_id = "/base_footprint"; //gripper_link
    grasp.pre_grasp_approach.direction.vector.x = 1.0;
    grasp.pre_grasp_approach.min_distance = 0.01;
    grasp.pre_grasp_approach.desired_distance = 0.2;

    grasp.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    grasp.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    grasp.pre_grasp_posture.points.resize(1);
    grasp.pre_grasp_posture.points[0].positions.resize(grasp.pre_grasp_posture.joint_names.size());
    grasp.pre_grasp_posture.points[0].positions[0] = 0.14;

    grasp.grasp_pose.header.frame_id = "/base_footprint";
    grasp.grasp_pose.pose.position.x = -0.12;
    grasp.grasp_pose.pose.position.y = 0.0;
    grasp.grasp_pose.pose.position.z = 0.0;
    grasp.grasp_pose.pose.orientation.x = 0.0;
    grasp.grasp_pose.pose.orientation.y = 0.0;
    grasp.grasp_pose.pose.orientation.z = 0.0;
    grasp.grasp_pose.pose.orientation.w = 1.0;

    grasp.post_grasp_retreat.direction.header.frame_id = "/base_footprint"; //gripper_link
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.min_distance = 0.1;
    grasp.post_grasp_retreat.desired_distance = 0.2;

    grasp.grasp_posture.joint_names = grasp.pre_grasp_posture.joint_names;
    grasp.grasp_posture.points.resize(1);
    grasp.grasp_posture.points[0].positions.resize(grasp.grasp_posture.joint_names.size());
    grasp.grasp_posture.points[0].positions[0] = 0.01;
    grasp.grasp_posture.points[0].effort.resize(grasp.grasp_posture.joint_names.size());
    grasp.grasp_posture.points[0].effort[0] = 0.6;

    pu_goal.possible_grasps.push_back(grasp);

    actionlib::SimpleClientGoalState pickStatus = pick_client.sendGoalAndWait(pu_goal);
    if(pickStatus != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("[pick_server]: goal execution succeeded");
    }
    else
    {
        ROS_WARN("[pick_server]: goal execution failed");
    }









    pick_server::PickFeedback feedback;
    pick_server::PickResult result;

    int x = 0;
    while(!as->isPreemptRequested() || ros::ok())
    {
        x++;
        feedback.state = std::to_string(x);

        as->publishFeedback(feedback);
        ros::Duration(1).sleep();

        if (x > 3)
            break;
    }
    // if preempted:
    //as->setPreempted(result);

    result.x = x;

    as->setSucceeded(result);
    */

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_server_node");
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroupInterface group("arm");

    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(50);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");

    group.setStartStateToCurrentState();

    pick_server_t pick_server(nh, "pick", boost::bind(&executeCB, _1, &pick_server), false);
    pick_server.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_ptr = &planning_scene_interface;
    ROS_INFO("[pick_server]: ready");

    ros::spin();

    return 0;
}