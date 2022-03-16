/*!********************************************************************************
 * \brief     Thrust controller implementation
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef _THRUST_CONTROLLER_H
#define _THRUST_CONTROLLER_H

// Define DRONE_DYNAMIC_TUNING in order to Tune PID Controller on the fly
//#define DRONE_DYNAMIC_TUNING

#include <iostream>
#include <string>
#include <vector>

// ROS
#include "ros/ros.h"
#include <robot_process.h>
#include "ThrustController.h"
// ROS msgs
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "sensor_msgs/Imu.h"
// Aerostack msgs
#include "geometry_msgs/Vector3Stamped.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "aerostack_msgs/FlightActionCommand.h"

#include <tf/transform_datatypes.h>
#include "tf_conversions/tf_eigen.h"
#include <eigen3/Eigen/Dense>
#include "eigen_conversions/eigen_msg.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "mavros/mavros_uas.h"
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "thrust_controller_process/thrustControllerConfig.h"

#include "control/filtered_derivative_wcb.h"

const double PELICAN_MID_LEVEL_AUTOPILOT_RATE = 30.0;
// #define VERBOSE_PELICAN_DMLA_ROS_MODULE

class ThrustControllerProcess : public RobotProcess
{

public: // Constructors and destructors
    ThrustControllerProcess();
    ~ThrustControllerProcess();
    double get_moduleRate();  

private:
    ThrustController MyThrustController;
    bool resetValues();
    std::string configFile;
    std::string robot_namespace;
    std::string robot_config_path;

public: // Init and close, related to Constructors and destructors
    void init();
    void close();

private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    double rate;
 //Topic names
public:
    std::string estimated_speed_topic;
    std::string estimated_pose_topic;
    std::string imu_topic;
    std::string roll_pitch_topic;
    std::string altitude_yaw_rate_topic;
    std::string flight_action_topic;
    std::string command_rpy_thrust_topic;

private:
    // Member variables
    double estimated_yaw_;
    float altitude_speed;
    ros::Subscriber self_localization_pose_sub;
    void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped& msg);
    ros::Subscriber self_localization_speed_sub;
    void selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped& msg);
    ros::Subscriber sensor_measurement_imu_sub;
    void sensorImuCallback(const sensor_msgs::Imu& msg);
    ros::Subscriber roll_pitch_sub;
    void rollPitchCallback(const geometry_msgs::PoseStamped& msg);
    ros::Subscriber altitude_rate_yaw_rate_sub;
    void altitudeRateYawRateCallback(const geometry_msgs::TwistStamped& msg);
    ros::Subscriber flight_action_sub;
    void flightActionCallback(const aerostack_msgs::FlightActionCommand& msg);

    ros::Publisher command_rpy_thrust_pub;

    void toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw);
    bool publishCommandRPYThrust();

    CVG_BlockDiagram::FilteredDerivativeWCB filtered_derivative_wcb;
public:
    void parametersCallback(thrustController::thrustControllerConfig &config, uint32_t level);
};

#endif

