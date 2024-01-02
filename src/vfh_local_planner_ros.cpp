#include "vfh_local_planner/vfh_local_planner_ros.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vfh_local_planner::VFHPlannerRos, nav_core::BaseLocalPlanner)

namespace vfh_local_planner
{
    VFHPlannerRos::VFHPlannerRos(): costmap_ros_(NULL), tf_(NULL), initialized_(false), odom_helper_("odom") {};

    // VFHPlannerRos::VFHPlannerRos(std::string name, tf2_ros::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    //     : costmap_ros_(NULL), tf_(NULL), initialized_(false), odom_helper_("odom")
    VFHPlannerRos::VFHPlannerRos(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false), odom_helper_("odom")
    {
        // initialize planner
		initialize(name, tf, costmap_ros);
    };

    VFHPlannerRos::~VFHPlannerRos() {};


    // void VFHPlannerRos::initialize(std::string name, tf2_ros::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    void VFHPlannerRos::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // check if plugin initialized
        if(!initialized_)
		{
            ROS_INFO("Initializing VFH Planner");

            ros::NodeHandle private_nh("~/" + name);

            g_plan_pub_= private_nh.advertise<nav_msgs::Path>("global_plan",1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan",1);

            //Parameter for dynamic reconfigure
            dsrv_ = new dynamic_reconfigure::Server<vfh_local_plannerConfig>(private_nh);
            dynamic_reconfigure::Server<vfh_local_plannerConfig>::CallbackType cb = boost::bind(&VFHPlannerRos::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            xy_goal_latch_ = false;
            rotating_to_goal_ = false;
            finding_alternative_way_ = false;

            costmap_ros_ = costmap_ros;
            tf_ = tf;

            global_frame_ = costmap_ros_->getGlobalFrameID();
            const char* temp_char = global_frame_.c_str();
            ROS_INFO("%s",temp_char);
            costmap_ = costmap_ros_->getCostmap();
            initialized_ = true;

            vfh_planner.Initialize(costmap_);     
            ros::Duration(3).sleep();
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
        
    }

    void VFHPlannerRos::reconfigureCB(vfh_local_plannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
        vfh_planner.Reconfigure(config);
        goal_reached_ = false;
    }

    bool VFHPlannerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

        ROS_INFO("Got new plan!");
        //reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        xy_goal_latch_ = false;
        rotating_to_goal_ = true;
        goal_reached_ = false;
        finding_alternative_way_ = false;
            
        return true;
    };

    void VFHPlannerRos::publishGlobalPath(std::vector<geometry_msgs::PoseStamped>& path)
    {
        base_local_planner::publishPlan(path,g_plan_pub_);
    }

    void VFHPlannerRos::publishLocalPath(std::vector<geometry_msgs::PoseStamped>& path)
    {
        base_local_planner::publishPlan(path,l_plan_pub_);
    }

    bool VFHPlannerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        //Check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
        
        //Get the pose of the robot in the global frame of the costmap
        tf2::Stamped<tf2::Transform> current_pose;
        geometry_msgs::PoseStamped robot_pose;
        if (!costmap_ros_->getRobotPose(robot_pose)) {
            std::cout << "cant get current pose" << std::endl;
            return false;
        }
        tf2::convert(robot_pose,current_pose);

        //Get current robot velocity
        tf2::Stamped<tf2::Transform> current_vel;
        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);
        tf2::convert(robot_vel,current_vel);

        //Create transform used to transform coordinates from global planner frame to costmap frame
        // tf::StampedTransform frame_transform;
        // geometry_msgs::TransformStamped frame_transform;
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener tf_(tfBuffer);

        tf2::Stamped<tf2::Transform> frame_transform;
        geometry_msgs::TransformStamped transform_stamped;
        // tf2_ros::Buffer tfBuffer;

        try
        {
            // tf_->canTransform(global_frame_,global_plan_.back().header.frame_id, ros::Time::now());
            transform_stamped = tf_->lookupTransform("odom","map", ros::Time::now());
            tf2::fromMsg(transform_stamped,frame_transform);
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("warning: %s",ex.what());
            // return false;
        }
        

        // frame_transform.lookupTransform(global_frame_, ros::Time(), global_plan_.back().header.frame_id, global_plan_.back().header.stamp, 
        //   global_plan_.back().header.frame_id, frame_transform);
        // global_plan_.back().header.stamp=ros::Time::now();
        // tfBuffer.lookupTransform(global_frame_,global_plan_.back().header.frame_id,global_plan_.back().header.stamp,ros::Duration(5));
        // tfBuffer.lookupTransform("odom","map",ros::Time(0),ros::Duration(5));
        // if(tfBuffer.canTransform(global_frame_,global_plan_.back().header.frame_id,ros::Time::now(),ros::Duration(5)))
        // {
        //     tfBuffer.lookupTransform(global_frame_,global_plan_.back().header.frame_id,ros::Time(),ros::Duration(5));
        // }
        // else
        // {
        //     ROS_ERROR("transform error");
        // }

        //Transform plan goal from the global planner frame to the frame of the costmap
        const geometry_msgs::PoseStamped& plan_goal = global_plan_.back();
        tf2::Stamped<tf2::Transform> global_goal;
        tf2::convert(plan_goal, global_goal);
        // global_goal.setData(frame_transform * global_goal);
        global_goal.setData(global_goal);
        global_goal.stamp_ = transform_stamped.header.stamp;
        global_goal.frame_id_ = global_frame_;
        // publishGlobalPath
        //Transforms the global plan of the robot from the global planner frame to the frame of the costmap
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        // ROS_INFO("%zu",transformed_plan.size());
        // ROS_INFO("%f",robot_pose.pose.position.x);
        // if(!planner_util_.getLocalPlan(robot_pose,transformed_plan));
        // {
        //     ROS_ERROR("cannot get local plan");
        //     return false;
        // }
        // ros::Duration(5).sleep();
        // ROS_INFO("3");

        if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, transformed_plan)) {
            
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            if (!config_.ignore_global_plan_updates)
                return false;
        }

        // std::vector<geometry_msgs::PoseStamped> local_plan;
        // base_local_planner::Trajectory path;

        // std::cout<<path.getPointsSize()<<std::endl;
        // for(int i=0;i<path.getPointsSize();i++)
        // {
        //     double p_x,p_y,p_th;
        //     geometry_msgs::PoseStamped p;
        //     path.getPoint(i,p_x,p_y,p_th);
        //     p.header.frame_id=costmap_ros_->getGlobalFrameID();
        //     p.header.stamp=ros::Time::now();
        //     p.pose.position.x=p_x;
        //     p.pose.position.y=p_y;
        //     p.pose.position.z=0.0;
        //     tf2::Quaternion q;
        //     q.setRPY(0,0,p_th);

        //     tf2::convert(q,p.pose.orientation);
        //     local_plan.push_back(p);
        // }

        tf2::Stamped<tf2::Transform> intermediary_goal_point;
        //Check if the modified plan is not empty
        if (!transformed_plan.empty())
        {
            //Trim off parts of the global plan that are far enough behind the robot
            //base_local_planner::prunePlan(current_pose, transformed_plan, global_plan_);

            //Get intermediary goal point in the transformed plan
            int point_index = GetPlanPoint(transformed_plan, global_plan_,current_pose);
            // tf::poseStampedMsgToTF(transformed_plan.at(point_index), intermediary_goal_point);
            tf2::fromMsg(transformed_plan.at(point_index), intermediary_goal_point);
            // publishLocalPath(local_plan);
            // publishGlobalPath(transformed_plan);
            ROS_INFO("plan send");
        }
        else
        {
            ROS_ERROR("Plan is empty");
            if (!config_.ignore_global_plan_updates)
                return false;
        }
        
        //Update VFH histogram with new costmap
        if (!vfh_planner.UpdateHistogram(costmap_ros_->getCostmap()))
        {
            ROS_WARN("Could not find clear direction");
            return false;
        }

        //#############################################################################################################################################
        //########################################### Check if the robot reached the goal position ####################################################

        //Check if the robot is at the goal coordinate x y
        double goal_x = global_goal.getOrigin().getX();
        double goal_y = global_goal.getOrigin().getY();
        if (base_local_planner::getGoalPositionDistance(robot_pose, goal_x, goal_y) <= config_.xy_goal_tolerance_ || xy_goal_latch_)
        {
            std::cout << "goal reached" << std::endl;
            xy_goal_latch_ = true;
            //Check if the robot is at the same orientation of the goal
            double goal_th = tf2::getYaw(global_goal.getRotation());
            if (fabs(base_local_planner::getGoalOrientationAngleDifference(robot_pose, goal_th)) <= config_.yaw_goal_tolerance_)
            {
                std::cout << "end" << std::endl;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                goal_reached_ = true;
                return true;
            }
            else{
                std::cout << "rotating to goal to end" << std::endl;
                vfh_planner.RotateToGoal(current_pose, current_vel, goal_th, cmd_vel);
                return true;
            }
        }
        //#############################################################################################################################################
        //######################################################## Drive to the goal ##################################################################
        double direction_to_follow;
        double goal_distance;
        if(finding_alternative_way_)
        {
            double global_plan_goal_angle = atan2((global_goal.getOrigin().getY()-current_pose.getOrigin().getY()), (global_goal.getOrigin().getX()-current_pose.getOrigin().getX()));
            goal_distance = 0.2;
            if (vfh_planner.DirectionIsClear(global_plan_goal_angle))
            {
                direction_to_follow = global_plan_goal_angle;
                goal_distance = std::min(sqrt(pow((global_goal.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((global_goal.getOrigin().getY()-current_pose.getOrigin().getY()),2)), 0.3);
            }
            else
            {
                direction_to_follow = vfh_planner.GetNewDirection(global_plan_goal_angle, tf2::getYaw(current_pose.getRotation()),previews_direction);
            }
        }
        else
        {
            double intermediary_goal_orientation = atan2((intermediary_goal_point.getOrigin().getY()-current_pose.getOrigin().getY()), (intermediary_goal_point.getOrigin().getX()-current_pose.getOrigin().getX()));

            //Check if the path is free
            if (!vfh_planner.DirectionIsClear(intermediary_goal_orientation))
            {
                finding_alternative_way_ = true;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }
            else
            {
                direction_to_follow = intermediary_goal_orientation;
                goal_distance = sqrt(pow((intermediary_goal_point.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((intermediary_goal_point.getOrigin().getY()-current_pose.getOrigin().getY()),2));
            }
        }

        previews_direction = direction_to_follow;

        std::cout << "Going to direction: " << radToDeg(direction_to_follow) << std::endl;
        if (rotating_to_goal_)
        {
            std::cout << "rotating to goal to start" << std::endl;
            vfh_planner.RotateToGoal(current_pose, current_vel, direction_to_follow, cmd_vel);
            if (fabs(base_local_planner::getGoalOrientationAngleDifference(robot_pose, direction_to_follow)) < config_.yaw_goal_tolerance_)
            {
                rotating_to_goal_ = false;
            }
            return true;
        }
        //Check if the robot is deviating too much from the plan
        if (fabs(base_local_planner::getGoalOrientationAngleDifference(robot_pose, direction_to_follow)) > M_PI/3)
        {
            std::cout << "rotating to goal to start" << std::endl;
            rotating_to_goal_ = true;
            cmd_vel.linear.x = 0.05;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        //Drive toward the plan
        else
        {
            std::cout << "driving to goal" << std::endl;
            vfh_planner.DriveToward(angles::shortest_angular_distance(tf2::getYaw(current_pose.getRotation()),direction_to_follow), goal_distance, cmd_vel);
        }

        return true;
    };

    bool VFHPlannerRos::isGoalReached()
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
        return goal_reached_;
    };

    int VFHPlannerRos::GetPlanPoint(std::vector<geometry_msgs::PoseStamped> transformed_plan, std::vector<geometry_msgs::PoseStamped> &global_plan, tf2::Stamped<tf2::Transform> current_pose)
    {
        int point = 0;
        for (int i = 0; i < transformed_plan.size(); i++)
        {
            double point_distance = sqrt(pow((transformed_plan.at(i).pose.position.x-current_pose.getOrigin().getX()),2)+pow((transformed_plan.at(i).pose.position.y-current_pose.getOrigin().getY()),2));
            if (point_distance < 0.6)
            {
                point = i;
            }
            else
            {
                if(point)
                    break;
            }
            
        }
        return point;
    }
}