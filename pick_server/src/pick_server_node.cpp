
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pick_server/PickAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// server wrapper for pick goals
typedef actionlib::SimpleActionServer<pick_server::PickAction> pick_server_t;

// moveit clients
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> pick_client_t;

moveit::planning_interface::PlanningSceneInterface *planning_scene_ptr;

tf::TransformListener *transformer;

void addCylinderToScene(std::string name,
                        double x,
                        double y,
                        double z,
                        double height,
                        double width)
{
    std::vector<moveit_msgs::CollisionObject> col_objects;
    moveit_msgs::CollisionObject target_obj;
    target_obj.id = name;

    // define object primitives (size and shape)
    shape_msgs::SolidPrimitive cylinder_primitives;
    cylinder_primitives.type = cylinder_primitives.CYLINDER;
    cylinder_primitives.dimensions.resize(2);
    cylinder_primitives.dimensions[0] = height;
    cylinder_primitives.dimensions[1] = width;

    target_obj.primitives.push_back(cylinder_primitives);

    // define object position and orientation
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

    target_obj.primitive_poses.push_back(target_pose);

    target_obj.operation = target_obj.ADD;
    target_obj.header.frame_id = "/base_footprint";

    col_objects.push_back(target_obj);
    planning_scene_ptr->addCollisionObjects(col_objects);
}

moveit_msgs::PickupGoal buildPickGoal(const std::string& obj_name)
{
    moveit_msgs::PickupGoal pu_goal;
    pu_goal.target_name = obj_name;
    pu_goal.group_name = "arm";
    pu_goal.end_effector = "eef";
    pu_goal.allowed_planning_time = 15.0;
    pu_goal.planner_id = "RRTConnectkConfigDefault";
    pu_goal.minimize_object_distance = true;

    pu_goal.planning_options.replan_delay = 2.0;
    pu_goal.planning_options.planning_scene_diff.is_diff = true;
    pu_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    pu_goal.planning_options.replan=true;
    pu_goal.planning_options.replan_attempts=5;

    moveit_msgs::Grasp g;
    g.max_contact_force = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = "/base_footprint";
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size());
    g.pre_grasp_posture.points[0].positions[0] = 0.14;

    g.grasp_pose.header.frame_id = pu_goal.target_name;
    g.grasp_pose.pose.position.x = -0.02;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = 0.0;
    g.grasp_pose.pose.orientation.x = 0.0;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].effort[0] = 0.6;

    g.post_grasp_retreat.direction.header.frame_id = "/base_footprint";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    pu_goal.possible_grasps.push_back(g);

    return pu_goal;
}


void executeCB(const pick_server::PickGoalConstPtr& goal, pick_server_t* as)
{

    pick_client_t pick_client("pickup", true);
    ROS_INFO("[pick_server]: waiting for Moveit pickup server");
    pick_client.waitForServer();
    ROS_INFO("[pick_server]: got Moveit pickup server");

    //todo: print incoming request args

    geometry_msgs::PointStamped origin_goal;
    origin_goal.header.frame_id = goal->frame_id;
    origin_goal.point.x = goal->x;
    origin_goal.point.y = goal->y;
    origin_goal.point.z = goal->z;

    geometry_msgs::PointStamped transformed_goal;

    try
    {
        transformer->transformPoint("/base_footprint", origin_goal, transformed_goal);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    addCylinderToScene(goal->obj_name,
                       transformed_goal.point.x,
                       transformed_goal.point.y,
                       transformed_goal.point.z,
                       goal->h,
                       goal->w);

    moveit_msgs::PickupGoal pick_goal = buildPickGoal(goal->obj_name);
    actionlib::SimpleClientGoalState pick_status = pick_client.sendGoalAndWait(pick_goal);

    if(pick_status != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("[pick_server]: goal execution succeeded");
    }
    else
    {
        ROS_WARN("[pick_server]: goal execution failed");
    }


    // todo: listen to pick results, and publish to my client
    // todo: add table to planning scene

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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_server_node");
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroupInterface group("arm");

    /*group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(50);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");

    group.setStartStateToCurrentState();*/

    pick_server_t pick_server(nh, "pick", boost::bind(&executeCB, _1, &pick_server), false);
    pick_server.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_ptr = &planning_scene_interface;

    tf::TransformListener tf_trans;
    transformer = &tf_trans;

    ROS_INFO("[pick_server]: ready");

    ros::spin();

    return 0;
}