/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */


#include <ros/ros.h>
#include <publisher.h>
#include <subscriber.h>
#include <px4_msgs/DebugVect.h>
#include <px4_msgs/OffboardControlMode.h>
#include <px4_msgs/TrajectorySetpoint.h>
#include <px4_msgs/VehicleCommand.h>
#include <px4_msgs/VehicleControlMode.h>
#include <px4_msgs/Timesync.h>
#include <chrono>
#include <iostream>
#include <atomic>
#include <thread>
#include <cmath>
#include <stdint.h>
#include <climits>

class OffboardControl
{
public:
    OffboardControl(ros::NodeHandle &nh) {
        // Publishers
        offboard_control_mode_publisher_ = nh->advertise<px4_msgs::OffboardControlMode>("fmu/offboard_control_mode/in", 10);
        trajectory_setpoint_publisher_ = nh->advertise<px4_msgs::TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
        vehicle_command_publisher_ = nh->advertise<px4_msgs::VehicleCommand>("fmu/vehicle_command/in", 10);

        // Subscribers
        timesync_sub_ = nh.subscribe("/fmu/timesync/out", 10, &OffboardControl::timesyncCallback, this);
        subscription_ = nh.subscribe("vel_ctrl_vect_topic", 10, &OffboardControl::OnVelVect, this);

        // Timer
        timer_ = nh.createTimer(ros::Duration(0.05), &OffboardControl::controlCallback, this);

        ROS_INFO("OffboardControl Node Initialized");
    }

    ~OffboardControl() {
        ROS_INFO("Shutting down offboard control...");
        publish_vehicle_command(px4_msgs::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        ros::Duration(0.01).sleep();
    }

    void arm();
    void disarm();

private:
    ros::Timer timer_;
    ros::Subscriber subscription_;
    ros::Publisher offboard_control_mode_publisher_;
    ros::Publisher trajectory_setpoint_publisher_;
    ros::Publisher vehicle_command_publisher_;
    ros::Subscriber timesync_sub_;

    std::atomic<uint64_t> timestamp_;
    bool armed = false;
    float xv = NAN, yv = NAN, zv = NAN;
    bool hasReceivedLaserScanMsgs = false;
    int sensorMsgsCallbacks = 0, sensorMsgsCallbacksPrev = 0, sensorMsgsCallbacksSamePrev = 0;

    void OnVelVect(const px4_msgs::DebugVect::ConstPtr& msg);
    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint(float x, float y, float z, float yaw, float yawspeed, float vx, float vy, float vz) const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0,
                                 float param3 = 0.0,
                                 float param4 = 0.0,
                                 float param5 = 0.0,
                                 float param6 = 0.0,
                                 float param7 = 0.0) const;

    void timesyncCallback(const px4_msgs::Timesync::ConstPtr& msg) {
        timestamp_.store(msg->timestamp);
    }

    void controlCallback(const ros::TimerEvent&) {
        if (hasReceivedLaserScanMsgs && !armed) {
            ROS_INFO("Control messages received, attempting takeoff...");
            publish_vehicle_command(px4_msgs::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }

        // Offboard control mode needs to be paired with trajectory setpoint
        publish_offboard_control_mode();
        publish_trajectory_setpoint(NAN, NAN, NAN, NAN, NAN, xv, yv, zv);

        // Detect if no messages are received when drone is armed, land if true
        if (armed) {
            if (sensorMsgsCallbacks == sensorMsgsCallbacksPrev) {
                sensorMsgsCallbacksSamePrev++;
            } else {
                sensorMsgsCallbacksSamePrev = 0;
            }

            if (sensorMsgsCallbacksSamePrev == 10) {
                ROS_INFO("Connection to control publisher lost, landing...");
                publish_vehicle_command(px4_msgs::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            }
            sensorMsgsCallbacksPrev = sensorMsgsCallbacks;
        }
    }
};

	

/**
 * @brief Control drone velocity based on ROS2 advertiser
 */
void OffboardControl::OnVelVect(const px4_msgs::DebugVect::Ptr msg) {
	OffboardControl::xv = msg->x;
	OffboardControl::yv = msg->y;
	OffboardControl::zv = msg->z;
	OffboardControl::hasReceivedLaserScanMsgs = true;
	if(OffboardControl::sensorMsgsCallbacks < INT_MAX){
		OffboardControl::sensorMsgsCallbacks++;
	} else {
		OffboardControl::sensorMsgsCallbacks = 0;
	}
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() {
	armed = true;
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() {
	armed = false;
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yaw, float yawspeed, float vx, float vy, float vz) const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.position = { x, y, z }; 	// in meters NED
	msg.yaw = yaw; 					// in radians NED -PI..+PI
	msg.yawspeed = yawspeed; 		// in radians/sec
	msg.velocity = { vx, vy, vz } ; // in meters/sec
	RCLCPP_INFO(this->get_logger(),  "\n Velocity vector: \n vx: %f, vy: %f, vz: %f'", vx, vy, vz);
	//msg.acceleration	// in meters/sec^2
	//msg.jerk		// in meters/sec^3
	//msg.thrust		// normalized thrust vector in NED

	trajectory_setpoint_publisher_->publish(msg);

}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2, float param3, float param4,
					      float param5, float param6,
					      float param7) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char** argv) {
    std::cout << "Starting offboard control node..." << std::endl;
    ros::init(argc, argv, "offboard_control");
    ros::NodeHandle nh;

    OffboardControl offboard_control = OffboardControl(&nh);

    ros::spin();
    return 0;
}
