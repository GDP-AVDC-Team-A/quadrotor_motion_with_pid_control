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

#ifndef THRUSTCONTROLLER_H
#define THRUSTCONTROLLER_H

#include <math.h>

#include "drone_utils/drone_state_command_enum.h"
#include "drone_utils/drone_state_enum.h"
#include "DMLA_YawController.h"
#include "DMLA_AltSpeedController.h"
#include "DMLA_HoverController.h"
#include <iostream>
#include <string>
#include "xmlfilereader.h"

//#define EMERGENCY_MODE_ENABLE

class ThrustController
{
public:
    ThrustController();
    ~ThrustController();
    int configureAutopilot(std::string autopilot_configuration_file);

public:
    bool close();
    bool reset();
    bool start();
    bool stop();
    bool run();
    void init(std::string configFile);

public:
    bool readConfigs(std::string configFile);

public:
    // Drone Middle Level Autopilot Commands
    void setAutopilotCommandData_PitchRoll( const double &pitch_cmd, const double &roll_cmd);
    void setAutopilotCommandData_DYaw( const double &dyaw_cmd);
    void setAutopilotCommandData_DAltitude( const double &dalt_cmd);

    // Sensor Measurements
    int setDroneMeasurementRotationAngles(float roll_deg, float pitch_deg, float yaw_deg);
    int setDroneMeasurementAltitude(float z_mps, float dz_mps);
    int setDroneMeasurementGroundOpticalFlow(float vx_mps, float vy_mps);
    // Drone Flying Mode
    int  setDroneFlyingModeCommand( DroneStateCommand::StateCommand last_drone_mode_command_in );
    void getDroneFlyingMode( DroneState::ModeType *const pcurrent_drone_status );
    // Drone Low Level Autopilot Commands
    void getDroneLLAutopilotCommand( float *const pitch, float *const roll, float *const dyaw, float *const thrust);
    inline DroneState::ModeType getDroneMLAutopilotStatus() { return current_drone_mode; }

public:
    void setMidlevelDynamicGains(double feedforward, double kp, double ki, double kd){
        altSpeedController.setAltitudeDynamicGains(feedforward,kp, ki, kd);
    }

private:
    void preconditionCheck();
    bool stateTransitionCheck();
    bool processState();
    void computeDroneAutopilotCommand();

private:
    bool autopilot_is_started;
    bool first_feedback_received,
         first_altitude_feedback_received,
         first_ground_speed_feedback_received,
         first_rotation_angles_feedback_received;

    DroneStateCommand::StateCommand last_drone_mode_command;
    bool                            flag_received_drone_mode_command;
    DroneState::ModeType            current_drone_mode;
    // Sensor feedback
    double last_z_m;    // m
    double last_dz_mps; // m/s
    double last_yaw_measurement;            // deg
    double last_pitch_measurement;          // deg
    double last_roll_measurement;           // deg
    double last_ground_speed_X_measurement; // m/s
    double last_ground_speed_Y_measurement; // m/s
    // Mid-Level_DroneAutopilot_Commands, MLDAC received from other modules
    double last_MLDAC_pitch_m1top1;
    double last_MLDAC_roll_m1top1;
    double last_MLDAC_alt_speed_m1top1;
    double last_MLDAC_dyaw_m1top1;
    // drone_autopilot_command, cur_DA_ = current_drone_autopilot_
    double cur_DA_alt_speed_command_m1top1;
    double cur_DA_dyaw_command_m1top1;
    double cur_DA_dyaw_command_m1top1_aux;
    double cur_DA_pitch_command_m1top1;
    double cur_DA_roll_command_m1top1;
    double cur_DA_thrust_command_N;
    double cur_DA_dyaw_command_degps;
    double cur_DA_pitch_command_deg;
    double cur_DA_roll_command_deg;

    CVG::MAV::AltSpeedController altSpeedController;
    CVG::MAV::HoverController hoverController;
    bool is_last_ground_speed_measurement_valid;
    CVG::MAV::YawController yawController;
    Timer yawRateTimer;
    bool firstCommand;
    Timer landingTimer;
    bool landingWithTime;
    Timer takingoffTimer;

    // Configuration parameters
    double ON_GROUND_ALTITUDE;

    double TAKEOFF_SPEED;
    double TAKEOFF_FINAL_ALTITUDE;
    double TAKINGOFF_ALT_SPEED_COS_GAMMA;
    double MAX_TAKINGOFF_TIME_S;

    double LANDING_SPEED;
    double ALTITUDE_FOR_LANDING_WITH_TIME;
    double LANDING_WITH_TIME_MAX_SECONDS;
    double LANDING_WITH_TIME_FIXED_THRUST;

    double MAX_ALTITUDE;
    double MAX_ALTITUDE_DESCENDING_SPEED;
    double MAX_ALTITUDE_TO_CONSIDER_UNKNOWN_STATE_AS_LANDED;

    double ML_AUTOPILOT_MAX_ALLOWED_YAW_SPEED;
    double ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED;

    double LL_AUTOPILOT_COMMAND_TILT_MIN_VALUE;
    double LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
    double LL_AUTOPILOT_COMMAND_DYAW_MIN_VALUE;
    double LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE;
    double LL_AUTOPILOT_COMMAND_THRUST_MIN_VALUE;
    double LL_AUTOPILOT_COMMAND_THRUST_MAX_VALUE;
};

#endif // THRUSTCONTROLLER_H
