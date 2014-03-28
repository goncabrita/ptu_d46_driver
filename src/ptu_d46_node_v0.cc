#include <string>
#include <ros/ros.h>
#include <ptu_d46/ptu_d46_driver.h>
#include <sensor_msgs/JointState.h>

#include <actionlib/server/simple_action_server.h>
#include <ptu_d46_driver/GotoAction.h>

namespace PTU46 {

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
class PTU46_Node {
    public:
        PTU46_Node(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
        ~PTU46_Node();

        // Service Control
        void Connect();
        bool ok() {
            return m_pantilt != NULL;
        }
        void Disconnect();

        // Service Execution
        void spinOnce();

        // Callback Methods
        void SetGoal(const sensor_msgs::JointState::ConstPtr& msg);

    protected:
        PTU46* m_pantilt;
        ros::NodeHandle m_node;
        ros::NodeHandle m_private_node;
        ros::Publisher  m_joint_pub;
        ros::Subscriber m_joint_sub;

        actionlib::SimpleActionServer<ptu_d46_driver::GotoAction> m_as;

        void actionServerCallback(const ptu_d46_driver::GotoGoalConstPtr &goal);

    private:
        double pan_;
        double tilt_;

        double pan_speed_;
        double tilt_speed_;

        int hz;
        ros::Duration timeout;
        double goal_tolerance;

        std::string base_frame_id;
        std::string pan_frame_id;
        std::string tilt_frame_id;

        std::string pan_joint;
        std::string tilt_joint;
};

PTU46_Node::PTU46_Node(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle)
    :m_pantilt(NULL), m_node(node_handle), m_private_node(private_node_handle), m_as(node_handle, "ptu_d46", boost::bind(&PTU46_Node::actionServerCallback, this, _1), false) {

    m_private_node.param<std::string>("base_frame_id", base_frame_id, "ptu_d46_base_link");
    m_private_node.param<std::string>("pan_frame_id", pan_frame_id, "ptu_d46_pan_link");
    m_private_node.param<std::string>("tilt_frame_id", tilt_frame_id, "ptu_d46_tilt_link");

    m_private_node.param<std::string>("pan_joint", pan_joint, "ptu_d46_pan_joint");
    m_private_node.param<std::string>("tilt_joint", tilt_joint, "ptu_d46_tilt_joint");

    m_private_node.param("goal_tolerance", goal_tolerance, 0.01);
    double t;
    m_private_node.param("timeout", t, 5.0);
    timeout = ros::Duration(t);
    m_private_node.param("hz", hz, PTU46_DEFAULT_HZ);
}

PTU46_Node::~PTU46_Node() {
    Disconnect();
}

/** Opens the connection to the PTU and sets appropriate parameters.
    Also manages subscriptions/publishers */
void PTU46_Node::Connect() {
    // If we are reconnecting, first make sure to disconnect
    if (ok()) {
        Disconnect();
    }

    // Query for serial configuration
    std::string port;
    m_private_node.param<std::string>("port", port, PTU46_DEFAULT_PORT);
    int baud;
    m_private_node.param("baud", baud, PTU46_DEFAULT_BAUD);

    // Connect to the PTU
    ROS_INFO("Attempting to connect to %s...", port.c_str());
    m_pantilt = new PTU46(port.c_str(), baud);
    ROS_ASSERT(m_pantilt != NULL);
    if (! m_pantilt->isOpen()) {
        ROS_ERROR("Could not connect to pan/tilt unit [%s]", port.c_str());
        Disconnect();
        return;
    }
    ROS_INFO("Connected!");

    m_private_node.setParam("min_tilt", m_pantilt->GetMin(PTU46_TILT));
    m_private_node.setParam("max_tilt", m_pantilt->GetMax(PTU46_TILT));
    m_private_node.setParam("min_tilt_speed", m_pantilt->GetMinSpeed(PTU46_TILT));
    m_private_node.setParam("max_tilt_speed", m_pantilt->GetMaxSpeed(PTU46_TILT));
    m_private_node.setParam("tilt_step", m_pantilt->GetResolution(PTU46_TILT));

    m_private_node.setParam("min_pan", m_pantilt->GetMin(PTU46_PAN));
    m_private_node.setParam("max_pan", m_pantilt->GetMax(PTU46_PAN));
    m_private_node.setParam("min_pan_speed", m_pantilt->GetMinSpeed(PTU46_PAN));
    m_private_node.setParam("max_pan_speed", m_pantilt->GetMaxSpeed(PTU46_PAN));
    m_private_node.setParam("pan_step", m_pantilt->GetResolution(PTU46_PAN));


    // Publishers : Only publish the most recent reading
    m_joint_pub = m_node.advertise
                  <sensor_msgs::JointState>("joint_states", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub = m_node.subscribe
                  <sensor_msgs::JointState>("cmd", 1, &PTU46_Node::SetGoal, this);

    m_as.start();

}

/** Disconnect */
void PTU46_Node::Disconnect() {
    if (m_pantilt != NULL) {
        delete m_pantilt;   // Closes the connection
        m_pantilt = NULL;   // Marks the service as disconnected
    }
}

/** Callback for getting new Goal JointState */
void PTU46_Node::SetGoal(const sensor_msgs::JointState::ConstPtr& msg) {
    if (! ok())
        return;
    double pan = msg->position[0];
    double tilt = msg->position[1];
    double panspeed = msg->velocity[0];
    double tiltspeed = msg->velocity[1];
    m_pantilt->SetPosition(PTU46_PAN, pan);
    m_pantilt->SetPosition(PTU46_TILT, tilt);
    m_pantilt->SetSpeed(PTU46_PAN, panspeed);
    m_pantilt->SetSpeed(PTU46_TILT, tiltspeed);
}

/** Callback for the action server */
void PTU46_Node::actionServerCallback(const ptu_d46_driver::GotoGoalConstPtr &goal) {
    if (!ok())
        return;

    // helper variables
    ros::Rate loop_rate(hz);

    ros::Time start_time = ros::Time::now();

    double pan = goal->joint.position[0];
    double tilt = goal->joint.position[1];
    double panspeed = goal->joint.velocity[0];
    double tiltspeed = goal->joint.velocity[1];
    m_pantilt->SetPosition(PTU46_PAN, pan);
    m_pantilt->SetPosition(PTU46_TILT, tilt);
    m_pantilt->SetSpeed(PTU46_PAN, panspeed);
    m_pantilt->SetSpeed(PTU46_TILT, tiltspeed);

    // start executing the action
    while(ros::ok() && ok())
    {
        // check that preempt has not been requested by the client
        if(m_as.isPreemptRequested())
        {
            ROS_INFO("PTU_D46 - %s - Goal preempted", __FUNCTION__);
            // set the action state to preempted
            m_as.setPreempted();
            break;
        }

        // Read Position & Speed
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = base_frame_id;
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.velocity.resize(2);
        joint_state.name[0] = pan_joint;
        joint_state.position[0] = pan_;
        joint_state.velocity[0] = pan_speed_;
        joint_state.name[1] = tilt_joint;
        joint_state.position[1] = tilt_;
        joint_state.velocity[1] = tilt_speed_;

        if(ros::Time::now() - start_time > timeout)
        {
            ptu_d46_driver::GotoResult result;
            result.joint = joint_state;
            ROS_INFO("PTU-D46 - %s - Goal timeout", __FUNCTION__);
            // set the action state to failed
            m_as.setAborted();
            break;
        }

        if( fabs(goal->joint.position[0] - pan_) <= goal_tolerance  && fabs(goal->joint.position[1] - tilt_) <= goal_tolerance )
        {
            ptu_d46_driver::GotoResult result;
            result.joint = joint_state;
            // set the action state to succeeded
            m_as.setSucceeded(result);
            break;
        }

        ptu_d46_driver::GotoFeedback feedback;
        feedback.joint = joint_state;
        // publish the feedback
        m_as.publishFeedback(feedback);

        loop_rate.sleep();
    }
}

/**
 * Publishes a joint_state message with position and speed.
 * Also sends out updated TF info.
 */
void PTU46_Node::spinOnce() {
    if (! ok())
        return;

    // Read Position & Speed
    pan_  = m_pantilt->GetPosition(PTU46_PAN);
    tilt_ = m_pantilt->GetPosition(PTU46_TILT);

    pan_speed_  = m_pantilt->GetSpeed(PTU46_PAN);
    tilt_speed_ = m_pantilt->GetSpeed(PTU46_TILT);

    // Publish Position & Speed
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = base_frame_id;
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = pan_joint;
    joint_state.position[0] = pan_;
    joint_state.velocity[0] = pan_speed_;
    joint_state.name[1] = tilt_joint;
    joint_state.position[1] = tilt_;
    joint_state.velocity[1] = tilt_speed_;
    m_joint_pub.publish(joint_state);
}

} // PTU46 namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "ptu_d46");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Connect to PTU
    PTU46::PTU46_Node ptu_node(n, pn);
    ptu_node.Connect();
    if (! ptu_node.ok())
        return -1;

    // Query for polling frequency
    int hz;
    pn.param("hz", hz, PTU46_DEFAULT_HZ);
    ros::Rate loop_rate(hz);

    while (ros::ok() && ptu_node.ok()) {
        // Publish position & velocity
        ptu_node.spinOnce();

        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    if (! ptu_node.ok()) {
        ROS_ERROR("pan/tilt unit disconnected prematurely");
        return -1;
    }

    return 0;
}
