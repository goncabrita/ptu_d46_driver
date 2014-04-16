#include <string>
#include <ros/ros.h>
#include <ptu_d46/ptu_d46_driver.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

/**
 * PTU46 ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class PTU46_Node
{
    public:
        PTU46_Node(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
        ~PTU46_Node();

        // Service Control
        void Connect();
        bool Ok()
        {
            return m_pantilt != NULL;
        }
        void Disconnect();

        // Service Execution
        void SpinOnce();

        // Callback Methods
        void SetGoal(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

        void jointStateSpinner();

    protected:
        PTU46::PTU46* m_pantilt;
        ros::NodeHandle m_node;
        ros::NodeHandle m_private_node;
        ros::Publisher  m_joint_state_pub;
        ros::Publisher  m_joint_pub;
        ros::Subscriber m_joint_sub;

        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> m_as;
        bool as_active_;
        void actionServerCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

    private:
        bool Write(double pan_pos, double pan_vel, double tilt_pos, double tilt_vel);
        bool CheckJointNames(const std::vector<std::string> *joint_names);
        bool CheckGoal(const std::vector<trajectory_msgs::JointTrajectoryPoint> *points);

        double pan_;
        double tilt_;
        double pan_speed_;
        double tilt_speed_;
        double min_pan_;
        double max_pan_;
        double min_pan_speed_;
        double max_pan_speed_;
        double pan_step_;
        double min_tilt_;
        double max_tilt_;
        double min_tilt_speed_;
        double max_tilt_speed_;
        double tilt_step_;
        double pan_tolerance_;
        double tilt_tolerance_;
        ros::Duration time_tolerance_;

        int pan_index_;
        int tilt_index_;

        std::string pan_joint_;
        std::string tilt_joint_;

        ros::Time start_time_;
        ros::Time goal_start_time_;
        std::list<trajectory_msgs::JointTrajectoryPoint> trajectory_;
        std::vector<control_msgs::JointTolerance> path_tolerance_;
        std::vector<control_msgs::JointTolerance> goal_tolerance_;
        ros::Duration goal_time_tolerance_;

        double joint_state_rate_;

        bool got_new_goal_;
};

PTU46_Node::PTU46_Node(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle) :
    m_pantilt(NULL),
    m_node(node_handle), m_private_node(private_node_handle),
    m_as(node_handle, "/ptu_d46_controller/follow_joint_trajectory", boost::bind(&PTU46_Node::actionServerCallback, this, _1), false)
{
    got_new_goal_ = false;
}

PTU46_Node::~PTU46_Node()
{
    Disconnect();
}

/** Opens the connection to the PTU and sets appropriate parameters.
    Also manages subscriptions/publishers */
void PTU46_Node::Connect()
{
    // If we are reconnecting, first make sure to disconnect
    if(Ok())
    {
        Disconnect();
    }

    // Query for serial configuration
    std::string port;
    m_private_node.param<std::string>("port", port, PTU46_DEFAULT_PORT);
    int baud;
    m_private_node.param("baud", baud, PTU46_DEFAULT_BAUD);

    // Connect to the PTU
    ROS_INFO("PTU_D46 - %s - Attempting to connect to [%s]...", __FUNCTION__, port.c_str());
    m_pantilt = new PTU46::PTU46(port.c_str(), baud);
    ROS_ASSERT(m_pantilt != NULL);
    if(!m_pantilt->isOpen())
    {
        Disconnect();
        ROS_FATAL("PTU_D46 - %s - Could not connect to [%s]", __FUNCTION__, port.c_str());
        ROS_BREAK();
        return;
    }
    ROS_INFO("PTU_D46 - %s - Connected!", __FUNCTION__);

    m_private_node.param<std::string>("pan_joint", pan_joint_, "ptu_d46_pan_joint");
    m_private_node.param<std::string>("tilt_joint", tilt_joint_, "ptu_d46_tilt_joint");

    m_private_node.param<double>("min_tilt", min_tilt_, m_pantilt->GetMin(PTU46_TILT));
    m_private_node.param<double>("max_tilt", max_tilt_, m_pantilt->GetMax(PTU46_TILT));
    m_private_node.param<double>("min_tilt_speed", min_tilt_speed_, m_pantilt->GetMinSpeed(PTU46_TILT));
    m_private_node.param<double>("max_tilt_speed", max_tilt_speed_, m_pantilt->GetMaxSpeed(PTU46_TILT));
    m_private_node.param<double>("tilt_step", tilt_step_, m_pantilt->GetResolution(PTU46_TILT));

    m_private_node.param<double>("min_pan", min_pan_, m_pantilt->GetMin(PTU46_PAN));
    m_private_node.param<double>("max_pan", max_pan_, m_pantilt->GetMax(PTU46_PAN));
    m_private_node.param<double>("min_pan_speed", min_pan_speed_, m_pantilt->GetMinSpeed(PTU46_PAN));
    m_private_node.param<double>("max_pan_speed", max_pan_speed_, m_pantilt->GetMaxSpeed(PTU46_PAN));
    m_private_node.param<double>("pan_step", pan_step_, m_pantilt->GetResolution(PTU46_PAN));

    m_private_node.param("pan_tolerance", pan_tolerance_, pan_step_*10.0);
    m_private_node.param("tilt_tolerance", tilt_tolerance_, tilt_step_*10.0);

    m_private_node.param("joint_state_rate", joint_state_rate_, 50.0);

    // Publishers : Only publish the most recent reading
    m_joint_pub = m_node.advertise<control_msgs::JointTrajectoryControllerState>("/ptu_d46_controller/state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub = m_node.subscribe("/ptu_d46_controller/command", 1, &PTU46_Node::SetGoal, this);

    m_joint_state_pub = m_node.advertise<sensor_msgs::JointState>("/joint_states", 100);

    m_as.start();
    as_active_ = false;
}

/** Disconnect */
void PTU46_Node::Disconnect()
{
    if(m_pantilt != NULL)
    {
        delete m_pantilt;   // Closes the connection
        m_pantilt = NULL;   // Marks the service as disconnected
    }
}

/** Write positions and velocities to the PTU **/
bool PTU46_Node::Write(double pan_pos, double pan_vel, double tilt_pos, double tilt_vel)
{
    m_pantilt->SetPosition(PTU46_PAN, pan_pos);
    m_pantilt->SetPosition(PTU46_TILT, tilt_pos);
    m_pantilt->SetSpeed(PTU46_PAN, pan_vel);
    m_pantilt->SetSpeed(PTU46_TILT, tilt_vel);
    start_time_ = ros::Time::now();
}

/** Check joint names **/
bool PTU46_Node::CheckJointNames(const std::vector<std::string> *joint_names)
{
    int found_joint = 0;
    for(int i=0 ; i<joint_names->size() ; i++)
    {
        if(pan_joint_.compare(joint_names->at(i)) == 0)
        {
            pan_index_ = i;
            found_joint++;
        }
        else if(tilt_joint_.compare(joint_names->at(i)) == 0)
        {
            tilt_index_ = i;
            found_joint++;
        }
    }
    return (joint_names->size() == found_joint);
}

/** Check the joint limits **/
bool PTU46_Node::CheckGoal(const std::vector<trajectory_msgs::JointTrajectoryPoint> *points)
{
    for(int i=0 ; i<points->size() ; i++)
    {
        if(     points->at(i).positions[pan_index_] < min_pan_ ||
                points->at(i).positions[pan_index_] > max_pan_ ||
                points->at(i).positions[tilt_index_] < min_tilt_ ||
                points->at(i).positions[tilt_index_] > max_tilt_ ||
                points->at(i).velocities[pan_index_] > max_pan_speed_ ||
                points->at(i).velocities[tilt_index_] > max_tilt_speed_) return false;
    }
    return true;
}

/** Callback for getting new Goal JointState */
void PTU46_Node::SetGoal(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if(!Ok() || !CheckJointNames(&msg->joint_names) || !CheckGoal(&msg->points))
        return;

    if(as_active_)
    {
        ROS_WARN("PTU_D46 - %s - Goal preempted.", __FUNCTION__);
        // set the action state to preempted
        m_as.setPreempted();
        as_active_ = false;
    }

    trajectory_.clear();
    for(int i=0 ; i<msg->points.size() ; i++)
    {
        trajectory_.push_back(msg->points[i]);
    }

    got_new_goal_ = true;
    //Write(trajectory_.front().positions[pan_index_], trajectory_.front().velocities[pan_index_], trajectory_.front().positions[tilt_index_], trajectory_.front().velocities[tilt_index_]);
}

/** Callback for the action server */
void PTU46_Node::actionServerCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    if(!Ok())
        return;

    if(!CheckJointNames(&goal->trajectory.joint_names))
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        m_as.setAborted(result);
        ROS_ERROR("PTU-D46 - %s - Invalid joints!", __FUNCTION__);
        return;
    }

    if(!CheckGoal(&goal->trajectory.points))
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        m_as.setAborted(result);
        ROS_ERROR("PTU-D46 - %s - Invalid goal!", __FUNCTION__);
        return;
    }

    as_active_ = true;

    trajectory_.clear();
    for(int i=0 ; i<goal->trajectory.points.size() ; i++)
    {
        trajectory_.push_back(goal->trajectory.points[i]);
    }

    path_tolerance_ = goal->path_tolerance;
    goal_tolerance_ = goal->goal_tolerance;
    goal_time_tolerance_ = goal->goal_time_tolerance;
    got_new_goal_ = true;
    //Write(trajectory_.front().positions[pan_index_], trajectory_.front().velocities[pan_index_], trajectory_.front().positions[tilt_index_], trajectory_.front().velocities[tilt_index_]);

    goal_start_time_ = ros::Time::now();

    ros::Rate r(10.0);
    // start executing the action
    while(ros::ok() && Ok() && as_active_)
    {
        // check that preempt has not been requested by the client
        if(m_as.isPreemptRequested())
        {
            ROS_WARN("PTU_D46 - %s - Goal preempted.", __FUNCTION__);
            // set the action state to preempted
            m_as.setPreempted();
            as_active_ = false;
            break;
        }

        // Read Position & Speed
        control_msgs::FollowJointTrajectoryFeedback feedback;
        feedback.header.stamp = ros::Time::now();
        feedback.joint_names.push_back(pan_joint_);
        feedback.desired.positions.push_back(trajectory_.front().positions[pan_index_]);
        feedback.desired.velocities.push_back(trajectory_.front().velocities[pan_index_]);
        feedback.actual.positions.push_back(pan_);
        feedback.actual.velocities.push_back(pan_speed_);
        feedback.error.positions.push_back(trajectory_.front().positions[pan_index_] - pan_);
        feedback.error.velocities.push_back(trajectory_.front().velocities[pan_index_] - pan_speed_);
        feedback.joint_names.push_back(tilt_joint_);
        feedback.desired.positions.push_back(trajectory_.front().positions[tilt_index_]);
        feedback.desired.velocities.push_back(trajectory_.front().velocities[tilt_index_]);
        feedback.actual.positions.push_back(tilt_);
        feedback.actual.velocities.push_back(tilt_speed_);
        feedback.error.positions.push_back(trajectory_.front().positions[tilt_index_] - tilt_);
        feedback.error.velocities.push_back(trajectory_.front().velocities[tilt_index_] - tilt_speed_);
        feedback.desired.time_from_start = trajectory_.front().time_from_start;
        feedback.actual.time_from_start = ros::Time::now() - start_time_;
        feedback.error.time_from_start = ros::Time::now() - start_time_ - trajectory_.front().time_from_start;
        // publish the feedback
        m_as.publishFeedback(feedback);

        r.sleep();
    }

    as_active_ = false;
}

void PTU46_Node::jointStateSpinner()
{
    ros::Rate r(joint_state_rate_);
    while(ros::ok())
    {
        sensor_msgs::JointState msg;

        msg.header.stamp = ros::Time::now();
        msg.name.push_back(pan_joint_);
        msg.position.push_back(pan_);
        msg.velocity.push_back(pan_speed_);
        msg.effort.push_back(0.0);
        msg.name.push_back(tilt_joint_);
        msg.position.push_back(tilt_);
        msg.velocity.push_back(tilt_speed_);
        msg.effort.push_back(0.0);

        m_joint_state_pub.publish(msg);
        r.sleep();
    }
}

/**
 * Publishes a joint_state message with position and speed.
 * Also sends out updated TF info.
 */
void PTU46_Node::SpinOnce() {
    if (! Ok())
        return;

    // Read Position & Speed
    pan_  = m_pantilt->GetPosition(PTU46_PAN);
    tilt_ = m_pantilt->GetPosition(PTU46_TILT);

    pan_speed_  = m_pantilt->GetSpeed(PTU46_PAN);
    tilt_speed_ = m_pantilt->GetSpeed(PTU46_TILT);

    // Publish Position & Speed
    control_msgs::JointTrajectoryControllerState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.joint_names.push_back(pan_joint_);
    joint_state.actual.positions.push_back(pan_);
    joint_state.actual.velocities.push_back(pan_speed_);
    joint_state.joint_names.push_back(tilt_joint_);
    joint_state.actual.positions.push_back(tilt_);
    joint_state.actual.velocities.push_back(tilt_speed_);
    if(trajectory_.size() > 0)
    {
        joint_state.desired.positions.push_back(trajectory_.front().positions[pan_index_]);
        joint_state.desired.velocities.push_back(trajectory_.front().velocities[pan_index_]);
        joint_state.error.positions.push_back(trajectory_.front().positions[pan_index_] - pan_);
        joint_state.error.velocities.push_back(trajectory_.front().velocities[pan_index_] - pan_speed_);
        joint_state.desired.positions.push_back(trajectory_.front().positions[tilt_index_]);
        joint_state.desired.velocities.push_back(trajectory_.front().velocities[tilt_index_]);
        joint_state.error.positions.push_back(trajectory_.front().positions[tilt_index_] - tilt_);
        joint_state.error.velocities.push_back(trajectory_.front().velocities[tilt_index_] - tilt_speed_);
        joint_state.desired.time_from_start = trajectory_.front().time_from_start;
        joint_state.actual.time_from_start = ros::Time::now() - start_time_;
        joint_state.error.time_from_start = ros::Time::now() - start_time_ - trajectory_.front().time_from_start;
    }
    m_joint_pub.publish(joint_state);

    // If there is stuff on the queue deal with it
    if(trajectory_.size() > 0)
    {
        if(got_new_goal_)
        {
            Write(trajectory_.front().positions[pan_index_], trajectory_.front().velocities[pan_index_], trajectory_.front().positions[tilt_index_], trajectory_.front().velocities[tilt_index_]);
            got_new_goal_ = false;
        }

        double pan_tolerance = pan_tolerance_;
        double tilt_tolerance = tilt_tolerance_;
        ros::Duration time_tolerance = time_tolerance_;

        if(as_active_)
        {
            if(trajectory_.size() == 1 && goal_tolerance_[pan_index_].position > 0 && goal_tolerance_[tilt_index_].position > 0)
            {
                pan_tolerance = goal_tolerance_[pan_index_].position;
                tilt_tolerance = goal_tolerance_[tilt_index_].position;
            }
            else if(path_tolerance_[pan_index_].position > 0 && path_tolerance_[tilt_index_].position > 0)
            {
                pan_tolerance = path_tolerance_[pan_index_].position;
                tilt_tolerance = path_tolerance_[tilt_index_].position;
            }

            if(goal_time_tolerance_ > time_tolerance) time_tolerance = goal_time_tolerance_;
        }

        if(fabs(trajectory_.front().positions[pan_index_] - pan_) <= pan_tolerance && fabs(trajectory_.front().positions[tilt_index_] - tilt_) <= tilt_tolerance)
        {
            // Remove it from the queue
            trajectory_.pop_front();
            if(as_active_ && trajectory_.size() == 0)
            {
                // And set the AS to successful!
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                m_as.setSucceeded(result);
                as_active_ = false;
            }
            else if(trajectory_.size() > 0)
            {
                // And keep on going!
                Write(trajectory_.front().positions[pan_index_], trajectory_.front().velocities[pan_index_], trajectory_.front().positions[tilt_index_], trajectory_.front().velocities[tilt_index_]);
            }
        }
        // If we timed out...
        else if(as_active_ && ros::Time::now() > goal_start_time_ + goal_time_tolerance_)
        {
            // Clear the trajectory queue
            trajectory_.clear();
            // And abort the AS.
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
            m_as.setAborted(result);
            as_active_ = false;
        }
        else if(ros::Time::now() > start_time_ + time_tolerance)
        {
            if(as_active_ && goal_tolerance_[pan_index_].position != -1 && goal_tolerance_[tilt_index_].position != -1)
            {
                trajectory_.clear();

                // And abort the AS.
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
                m_as.setAborted(result);
                as_active_ = false;
            }
            else if(!as_active_)
            {
                trajectory_.clear();
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ptu_d46");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Connect to PTU
    PTU46_Node ptu_node(n, pn);
    ptu_node.Connect();

    // Query for polling frequency
    int hz;
    pn.param("hz", hz, 10);
    ros::Rate loop_rate(hz);

    ros::AsyncSpinner spinner(5);
    spinner.start();

    boost::thread joint_state_thread(&PTU46_Node::jointStateSpinner, &ptu_node);

    while(ros::ok() && ptu_node.Ok())
    {
        ptu_node.SpinOnce();
        loop_rate.sleep();
    }

    if(!ptu_node.Ok())
    {
        ROS_ERROR("PTU_D46 -- pan/tilt unit disconnected prematurely");
        return -1;
    }

    joint_state_thread.join();

    spinner.stop();

    return 0;
}
