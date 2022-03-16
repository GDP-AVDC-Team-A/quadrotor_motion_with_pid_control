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
 
#include "thrust_controller_process.h"

ThrustControllerProcess::ThrustControllerProcess(){
}

ThrustControllerProcess::~ThrustControllerProcess(){
}

void ThrustControllerProcess::ownSetUp()
{
    ros::param::get("~robot_namespace", robot_namespace);
    ros::param::get("~frequency", rate);
    ros::param::get("~robot_config_path", robot_config_path);
    ros::param::get("~config_file", configFile);

    //Topics
    ros::param::get("~estimated_speed", estimated_speed_topic);
    ros::param::get("~estimated_pose", estimated_pose_topic);
    ros::param::get("~imu", imu_topic);
    ros::param::get("~roll_pitch",roll_pitch_topic);
    ros::param::get("~altitude_yaw_rate",altitude_yaw_rate_topic);
    ros::param::get("~flight_action",flight_action_topic);
    ros::param::get("~command_rpy_thrust",command_rpy_thrust_topic);

    MyThrustController.init(robot_config_path+"/"+configFile);
    filtered_derivative_wcb.setTimeParameters(0.02,0.02,0.200,1.0,100.000);
    filtered_derivative_wcb.reset();
}

double ThrustControllerProcess::get_moduleRate(){
    return rate;
}

void ThrustControllerProcess::close(){
    MyThrustController.close();
}


//Reset
bool ThrustControllerProcess::resetValues()
{
    return true;
}

//Stop
void ThrustControllerProcess::ownStop()
{
   MyThrustController.stop();
}


//Run
void ThrustControllerProcess::ownRun()
{
    //Run Pelican Middle Level Autopilot
    MyThrustController.run();

    //Publish
    publishCommandRPYThrust();
}

void ThrustControllerProcess::ownStart()
{
    //Node
    ros::NodeHandle n;

    //configure droneArucoEye
    if(!MyThrustController.configureAutopilot(robot_config_path+"/pelican_autopilot.xml"))
    {
        #ifdef VERBOSE_PELICAN_DMLA_ROS_MODULE
                std::cout<<"[PelicanDMLA-ROS] Error reading configuration file"<<std::endl;
        #endif
    }

    //Standard topics
    //Subscribers
    self_localization_pose_sub = n.subscribe(estimated_pose_topic, 1, &ThrustControllerProcess::selfLocalizationPoseCallback, this);
    self_localization_speed_sub = n.subscribe(estimated_speed_topic, 1, &ThrustControllerProcess::selfLocalizationSpeedCallback, this);
    sensor_measurement_imu_sub = n.subscribe(imu_topic, 1, &ThrustControllerProcess::sensorImuCallback, this);
    roll_pitch_sub = n.subscribe(roll_pitch_topic, 1, &ThrustControllerProcess::rollPitchCallback, this);
    altitude_rate_yaw_rate_sub = n.subscribe(altitude_yaw_rate_topic, 1, &ThrustControllerProcess::altitudeRateYawRateCallback, this);
    flight_action_sub = n.subscribe(flight_action_topic, 1, &ThrustControllerProcess::flightActionCallback, this);
    //Publishers
    command_rpy_thrust_pub = n.advertise<mav_msgs::RollPitchYawrateThrust>(command_rpy_thrust_topic, 1, true);

    MyThrustController.start();
}

#ifdef DRONE_DYNAMIC_TUNING
    void ThrustControllerProcess::parametersCallback(thrustController::thrustControllerConfig &config, uint32_t level)
    {
        MyThrustController.setMidlevelDynamicGains(config.altitude_feedforward, config.altitude_kp, config.altitude_ki, config.altitude_kd);
    }
#endif

bool ThrustControllerProcess::publishCommandRPYThrust()
{
    DroneState::ModeType current_drone_mode = MyThrustController.getDroneMLAutopilotStatus();

    switch (current_drone_mode) {
    case DroneState::INIT:
    case DroneState::LOOPING:
        // Do not publish a CtrlInput command
        return false;
    break;
    case DroneState::LANDED:
    case DroneState::TAKING_OFF:
    case DroneState::HOVERING:
    case DroneState::FLYING:
    case DroneState::LANDING:
    {
        float pitch, roll, dyaw, thrust;
        MyThrustController.getDroneLLAutopilotCommand(&pitch, &roll, &dyaw, &thrust);

        mav_msgs::RollPitchYawrateThrust command_rpy_thrust_msg;
		command_rpy_thrust_msg.header.stamp = ros::Time::now();
        //Convert to radians
        command_rpy_thrust_msg.pitch = (-1)*pitch*(M_PI/180.0);
        command_rpy_thrust_msg.roll  = (+1)*roll*(M_PI/180.0);
        command_rpy_thrust_msg.yaw_rate  = (-1)*dyaw*(M_PI/180.0);
        command_rpy_thrust_msg.thrust.z = thrust;

        command_rpy_thrust_pub.publish( command_rpy_thrust_msg );
        return true;
    } break;
    }
    return false;
}

//Callbacks
void ThrustControllerProcess::selfLocalizationPoseCallback(const geometry_msgs::PoseStamped& msg){
    double yaw, p, r;
    toEulerianAngle(msg, &r, &p, &yaw);
    estimated_yaw_ = yaw;

    //Altitude
    ros::Time current_timestamp = ros::Time::now();

    double zraw_t = (-1.0) * msg.pose.position.z;

    time_t tv_sec; suseconds_t tv_usec;
    {
    tv_sec  = current_timestamp.sec;
    tv_usec = current_timestamp.nsec / 1000.0;
    filtered_derivative_wcb.setInput( zraw_t, tv_sec, tv_usec);
    }

    double z_t, dz_t;
    filtered_derivative_wcb.getOutput( z_t, dz_t);

    MyThrustController.setDroneMeasurementAltitude( z_t, dz_t);
}

void ThrustControllerProcess::selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped& msg){
  //This was a bug, as the velocities to the midlevel should be in body frame and the estimated pose topic pubs in world frame
  double y, p, r;
  y = estimated_yaw_;
  p = 0;
  r = 0;

  altitude_speed = msg.twist.linear.z;

  Eigen::Vector3f BodyFrame;
  Eigen::Vector3f GlobalFrame;
  Eigen::Matrix3f RotationMat;

  GlobalFrame(0) = (+1)*msg.twist.linear.x;
  GlobalFrame(1) = (+1)*msg.twist.linear.y;
  GlobalFrame(2) = 0;

  RotationMat(0,0) = cos(y);
  RotationMat(1,0) = -sin(y);
  RotationMat(2,0) = 0;

  RotationMat(0,1) = sin(y);
  RotationMat(1,1) = cos(y);
  RotationMat(2,1) = 0;

  RotationMat(0,2) = 0;
  RotationMat(1,2) = 0;
  RotationMat(2,2) = 1;

  BodyFrame = RotationMat*GlobalFrame;

  MyThrustController.setDroneMeasurementGroundOpticalFlow(BodyFrame(0), - BodyFrame(1));
}

void ThrustControllerProcess::sensorImuCallback(const sensor_msgs::Imu& msg){
    geometry_msgs::Vector3Stamped rotation_angles_msg;
    tf::Quaternion quaterniond;
    geometry_msgs::Quaternion quaternion;
    double roll,pitch,yaw;

    //convert quaternion msg to eigen
    tf::quaternionMsgToTF(msg.orientation, quaterniond);

//------ The below code converts ENU (Mavros) frame to NED (Aerostack) frame ------- ///

    //Rotating the frame in x-axis by 180 degrees
    //Eigen::Quaterniond BASE_LINK_TO_AIRCRAFT = mavros::UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
    tf::Quaternion BASE_LINK_TO_AIRCRAFT = tf::createQuaternionFromRPY(M_PI,0.0,0.0);

    quaterniond = quaterniond*BASE_LINK_TO_AIRCRAFT;

    //Rotating the frame in x-axis by 180 deg and in z-axis by 90 axis (accordance to the new mavros update)
    //Eigen::Quaterniond ENU_TO_NED = mavros::UAS::quaternion_from_rpy(M_PI, 0.0, M_PI_2);
    tf::Quaternion ENU_TO_NED = tf::createQuaternionFromRPY(M_PI,0.0,M_PI_2);
    quaterniond = ENU_TO_NED*quaterniond;
//-----------------------------------------------------------------------------------///

    //converting back quaternion from eigen to msg
    tf::quaternionTFToMsg(quaterniond, quaternion);

    tf::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Matrix3x3 m(q);

    //mavros::UAS::quaternion_to_rpy(quaterniond, roll, pitch, yaw);
    //quaterniond.getRPY(roll, pitch, yaw);
    
   //convert quaternion to euler angels
    m.getEulerYPR(yaw, pitch, roll);

    rotation_angles_msg.vector.x = (double)(+1)*(roll)*180/M_PI;
    rotation_angles_msg.vector.y = (double)(+1)*(pitch)*180/M_PI;
    rotation_angles_msg.vector.z = (double)(+1)*(yaw)*180/M_PI;


    //converting the yaw which is in the range from -phi to +phi to 0 to 360 degrees
    if( rotation_angles_msg.vector.z < 0 )
        rotation_angles_msg.vector.z += 360.0;

    MyThrustController.setDroneMeasurementRotationAngles(rotation_angles_msg.vector.x, rotation_angles_msg.vector.y, rotation_angles_msg.vector.z);
}

void ThrustControllerProcess::rollPitchCallback(const geometry_msgs::PoseStamped& msg){
    double r,p,yaw;
    toEulerianAngle(msg, &r, &p, &yaw);
    MyThrustController.setAutopilotCommandData_PitchRoll(p,r);
}

void ThrustControllerProcess::altitudeRateYawRateCallback(const geometry_msgs::TwistStamped& msg){
    MyThrustController.setAutopilotCommandData_DYaw( msg.twist.angular.z );
    MyThrustController.setAutopilotCommandData_DAltitude( msg.twist.linear.z );
}

void ThrustControllerProcess::flightActionCallback(const aerostack_msgs::FlightActionCommand& msg){
    DroneStateCommand::StateCommand last_drone_state_command = DroneStateCommand::IDLE;
    bool pass_command_to_midlevel_autopilot = true;
    switch (msg.action) {
    case aerostack_msgs::FlightActionCommand::TAKE_OFF:
        last_drone_state_command = DroneStateCommand::TAKE_OFF;
        break;
    case aerostack_msgs::FlightActionCommand::HOVER:
        last_drone_state_command = DroneStateCommand::HOVER;
        break;
    case aerostack_msgs::FlightActionCommand::LAND:
        last_drone_state_command = DroneStateCommand::LAND;
        break;
    case aerostack_msgs::FlightActionCommand::MOVE:
        last_drone_state_command = DroneStateCommand::MOVE;
        break;
    case aerostack_msgs::FlightActionCommand::UNKNOWN:
    default:
        pass_command_to_midlevel_autopilot = false;
        break;
    }

    if (pass_command_to_midlevel_autopilot) {
        MyThrustController.setDroneFlyingModeCommand(last_drone_state_command);
    }
}

void ThrustControllerProcess::toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw){
    if(q.pose.orientation.w == 0 && q.pose.orientation.x == 0 && q.pose.orientation.y == 0 && q.pose.orientation.z == 0){
        *roll   = 0; 
        *pitch = 0;
        *yaw = 0;
    }else{
        *roll  = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.y + q.pose.orientation.w * q.pose.orientation.x) , 1.0 - 2.0 * (q.pose.orientation.x * q.pose.orientation.x + q.pose.orientation.y * q.pose.orientation.y));
        *pitch = asin(2.0 * (q.pose.orientation.y * q.pose.orientation.w - q.pose.orientation.z * q.pose.orientation.x));
        *yaw   = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.y) , - 1.0 + 2.0 * (q.pose.orientation.w * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.x));    
    }
}