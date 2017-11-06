#include <controller/cmd_val_controller.h>
#include <iostream>


namespace swarm_navigator {

    CmdValController::CmdValController(ros::NodeHandle &nh,std::string topic,std::string base_link,std::string odom_link){
        nh_ = nh;
        liner_velocity_ = 0.25;
        angular_velocity_ = 0.75;
        base_link_ = base_link;
        odom_link_ = odom_link;
        //set up the publisher for the cmd_vel topic
        publisher_ = nh_.advertise<geometry_msgs::Twist>(topic, 1);        
    }

    CmdValController::CmdValController(ros::NodeHandle &nh,std::string topic){
        nh_ = nh;
        liner_velocity_ = 0.25;
        angular_velocity_ = 0.75;
        base_link_ = "base_link"; //"base_footprint"
        odom_link_ = "odom";
        //set up the publisher for the cmd_vel topic
        publisher_ = nh_.advertise<geometry_msgs::Twist>(topic, 1);        
    }
    

    void CmdValController::stop(){
        geometry_msgs::Twist command;
        command.linear.x = command.linear.y = command.angular.z = 0; 
        publisher_.publish(command);  
    }

    void CmdValController::foward(){
        stop();

        geometry_msgs::Twist command;
        command.linear.x = liner_velocity_;
        publisher_.publish(command);  
    }

    void CmdValController::backward(){
        stop();

        geometry_msgs::Twist command;
        command.linear.x = -liner_velocity_;
        publisher_.publish(command);  
    }

    void CmdValController::left(){
        stop();
        
        geometry_msgs::Twist command;
        command.angular.z = angular_velocity_;
        command.linear.x = liner_velocity_;
        publisher_.publish(command); 
    }

    void CmdValController::right(){
        stop();
        
        geometry_msgs::Twist command;
        command.angular.z = -angular_velocity_;
        command.linear.x = liner_velocity_;
        publisher_.publish(command); 
    }

    void CmdValController::setLinerVelocity(float velocity){
        liner_velocity_ = velocity;
    }

    void CmdValController::setAngularVelocity(float velocity){
        angular_velocity_ = velocity;
    }

    bool CmdValController::ready(){
        return nh_.ok();
    }

    bool CmdValController::driveForward(double distance){

        ros::Time time = ros::Time::now();//ros::Time(0)
        //wait for the listener to get the first message
        listener_.waitForTransform(base_link_, odom_link_, 
                                time, ros::Duration(1.0));
        
        //we will record transforms here
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;

        //record the starting transform from the odometry to the base frame
        listener_.lookupTransform(base_link_, odom_link_,  
                                time, start_transform);
        
        //we will be sending commands of type "twist"
        geometry_msgs::Twist base_cmd;
        //the command will be to go forward at 0.25 m/s
        base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = 0.1;
        
        ros::Rate rate(10.0);
        bool done = false;
        while (!done && nh_.ok())
        {
            //send the drive command
            publisher_.publish(base_cmd);
            rate.sleep();
            //get the current transform
            try
            {
                listener_.lookupTransform(base_link_, odom_link_,  
                                        ros::Time(0), current_transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                break;
            }
            //see how far we've traveled
            tf::Transform relative_transform = 
                start_transform.inverse() * current_transform;
            double dist_moved = relative_transform.getOrigin().length();

            if(dist_moved > distance){
                 done = true;
                 stop();
            }
        }
        if (done) return true;
        return false;
    }

    bool CmdValController::turn(bool clockwise, double radians){

        while(radians < 0) radians += 2*M_PI;
        while(radians > 2*M_PI) radians -= 2*M_PI;
    
        ros::Time time = ros::Time::now();//ros::Time(0)
        //wait for the listener to get the first message
        listener_.waitForTransform(base_link_, odom_link_,  
                                    time, ros::Duration(1.0));
        
        //we will record transforms here
        tf::StampedTransform start_transform;
        tf::StampedTransform current_transform;
    
        //record the starting transform from the odometry to the base frame
        listener_.lookupTransform(base_link_, odom_link_, 
                                    time, start_transform);
        
        //we will be sending commands of type "twist"
        geometry_msgs::Twist base_cmd;
        //the command will be to turn at 0.75 rad/s
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = 0.75;
        if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
        
        //the axis we want to be rotating by
        tf::Vector3 desired_turn_axis(0,0,1);
        if (!clockwise) desired_turn_axis = -desired_turn_axis;
        
        ros::Rate rate(10.0);
        bool done = false;
        while (!done && nh_.ok()){
            //send the drive command
            publisher_.publish(base_cmd);
            rate.sleep();
            //get the current transform
            try
            {
            listener_.lookupTransform(base_link_, odom_link_, 
                                        ros::Time(0), current_transform);
            }
            catch (tf::TransformException ex)
            {
            ROS_ERROR("%s",ex.what());
            break;
            }
            tf::Transform relative_transform = 
            start_transform.inverse() * current_transform;
            tf::Vector3 actual_turn_axis = 
            relative_transform.getRotation().getAxis();
            double angle_turned = relative_transform.getRotation().getAngle();
            if ( fabs(angle_turned) < 1.0e-2) continue;
    
            if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
            angle_turned = 2 * M_PI - angle_turned;
    
            if (angle_turned > radians){
                 done = true;
                 stop();
            }
        }
        if (done) return true;
        return false;
    }

    bool CmdValController::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const {
        global_pose.setIdentity();
        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = base_link_;
        robot_pose.stamp_ = ros::Time();
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

        listener_.waitForTransform(base_link_, odom_link_, 
            current_time, ros::Duration(1.0));
        // get the global pose of the robot
        try
        {
            listener_.transformPose(odom_link_, robot_pose, global_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Error: %s\n", ex.what());
            return false;
        }
       
        return true;
    }

    bool CmdValController::achieveGoal(const geometry_msgs::PoseStamped& goal){
        
        // ros::Time time = ros::Time::now();//ros::Time(0)
        // //wait for the listener to get the first message
        // listener_.waitForTransform(base_link_, odom_link_, 
        //                             ros::Time::now(), ros::Duration(1.0));
        
        // //we will record transforms here
        // tf::StampedTransform robot_transform;        
    
        // //record the starting transform from the odometry to the base frame
        // listener_.lookupTransform(base_link_, odom_link_, 
        //                             ros::Time::now(), robot_transform);
        ROS_INFO("Goal came...");
        tf::Stamped<tf::Pose> global_pose;
        getRobotPose(global_pose);
        geometry_msgs::PoseStamped current_position;
        tf::poseStampedTFToMsg(global_pose, current_position);

        double dx = goal.pose.position.x - current_position.pose.position.x;
        double dy = goal.pose.position.y - current_position.pose.position.y;
        double dz = goal.pose.position.z - current_position.pose.position.z;

        double yaw_goal = atan2(dy,dx);
        //double pitch_goal = atan2(sqrt(dy * dy + dx * dx), dy) + M_PI;

        // geometry_msgs::PoseStamped robot_pose;
        // robot_pose.pose.orientation.x = robot_transform.getRotation().getX();
        // robot_pose.pose.orientation.y = robot_transform.getRotation().getY();
        // robot_pose.pose.orientation.z = robot_transform.getRotation().getZ();
        // robot_pose.pose.orientation.w = robot_transform.getRotation().getW();
        
        double yaw_robot(50);
        yaw_robot = tf::getYaw(current_position.pose.orientation);  

        double yaw_goal_robot_diff(50);
        yaw_goal_robot_diff = yaw_goal-yaw_robot;  

        ROS_INFO("robot: [%f]", yaw_robot);
        ROS_INFO("goal: [%f]", yaw_goal);
        ROS_INFO("diff: [%f]", yaw_goal_robot_diff);
        ROS_INFO("Robot:(x,y,z) - ([%f],[%f],[%f])", 
                                        current_position.pose.position.x,
                                        current_position.pose.position.y,
                                        current_position.pose.position.z);

        bool clockwise = true;

        if (yaw_goal_robot_diff > M_PI)
                yaw_goal_robot_diff -= 2*M_PI;
        if (yaw_goal_robot_diff <= -M_PI)
                yaw_goal_robot_diff += 2*M_PI;


        if (yaw_goal_robot_diff > 0){     
            // ROS_INFO("clockwise");       
            clockwise = false;
        }else{
            // ROS_INFO("not clockwise"); 
            yaw_goal_robot_diff*=-1;
            clockwise = true;
        }
        ROS_INFO("angle: [%f]", yaw_goal_robot_diff);
        turn(clockwise,yaw_goal_robot_diff);
        double distance = hypot(current_position.pose.position.x - goal.pose.position.x
                    , current_position.pose.position.y - goal.pose.position.y);
        ROS_INFO("distance: [%f]", distance);
        driveForward(distance);        
        //turn(true,2*M_PI);

        return true;
    }

    bool CmdValController::followPath(const std::vector<geometry_msgs::PoseStamped>& plan){
    
        for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.end ()-1; 
                                it != plan.begin (); --it){
            ROS_INFO("cordinates %f %f %f",(*it).pose.position.x,(*it).pose.position.y,(*it).pose.position.z);
            // achieveGoal(*it);  
        }

        return true;
    }
}