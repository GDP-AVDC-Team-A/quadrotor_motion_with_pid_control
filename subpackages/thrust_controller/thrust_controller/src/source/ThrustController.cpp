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

#include "ThrustController.h"

ThrustController::ThrustController(){
    altSpeedController = CVG::MAV::AltSpeedController();
    hoverController = CVG::MAV::HoverController();
    yawController = CVG::MAV::YawController();
}

bool ThrustController::readConfigs(std::string configFile)
{
  try {
      XMLFileReader my_xml_reader(configFile);

      ON_GROUND_ALTITUDE              = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:ON_GROUND_ALTITUDE");

      TAKEOFF_SPEED                   = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:TAKE_OFF:TAKEOFF_SPEED");
      TAKEOFF_FINAL_ALTITUDE          = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:TAKE_OFF:TAKEOFF_FINAL_ALTITUDE");
      TAKINGOFF_ALT_SPEED_COS_GAMMA   = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:TAKE_OFF:TAKINGOFF_ALT_SPEED_COS_GAMMA");
      MAX_TAKINGOFF_TIME_S            = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:TAKE_OFF:MAX_TAKINGOFF_TIME_S");

      LANDING_SPEED                   = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LANDING:LANDING_SPEED");
      ALTITUDE_FOR_LANDING_WITH_TIME  = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LANDING:ALTITUDE_FOR_LANDING_WITH_TIME");
      LANDING_WITH_TIME_MAX_SECONDS   = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LANDING:LANDING_WITH_TIME_MAX_SECONDS");

      double THRUSTER_GAIN, FEEDFORWARD_M, LANDING_WITH_TIME_FIXED_THRUST_KG;

      FEEDFORWARD_M                                     = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:FEEDFORWARD_M");
      THRUSTER_GAIN                                     = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:THRUSTER_GAIN");
      LANDING_WITH_TIME_FIXED_THRUST_KG                 = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LANDING:LANDING_WITH_TIME_FIXED_THRUST_KG");
      LANDING_WITH_TIME_FIXED_THRUST                    = (FEEDFORWARD_M - LANDING_WITH_TIME_FIXED_THRUST_KG) * THRUSTER_GAIN;

      MAX_ALTITUDE                                      = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:MAX_ALTITUDE:MAX_ALTITUDE");
      MAX_ALTITUDE_DESCENDING_SPEED                     = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:MAX_ALTITUDE:MAX_ALTITUDE_DESCENDING_SPEED");
      MAX_ALTITUDE_TO_CONSIDER_UNKNOWN_STATE_AS_LANDED  = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:MAX_ALTITUDE:MAX_ALTITUDE_TO_CONSIDER_UNKNOWN_STATE_AS_LANDED");

      ML_AUTOPILOT_MAX_ALLOWED_YAW_SPEED		= my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:CONTROLLER_SATURATIONS:ML_AUTOPILOT_MAX_ALLOWED_YAW_SPEED");
      ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED           = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:CONTROLLER_SATURATIONS:ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED");

      LL_AUTOPILOT_COMMAND_TILT_MIN_VALUE               = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LL_AUTOPILOT:LL_AUTOPILOT_COMMAND_TILT_MIN_VALUE");
      LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE               = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LL_AUTOPILOT:LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE");
      LL_AUTOPILOT_COMMAND_DYAW_MIN_VALUE               = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LL_AUTOPILOT:LL_AUTOPILOT_COMMAND_DYAW_MIN_VALUE");
      LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE               = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LL_AUTOPILOT:LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE");
      LL_AUTOPILOT_COMMAND_THRUST_MIN_VALUE             = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LL_AUTOPILOT:LL_AUTOPILOT_COMMAND_THRUST_MIN_VALUE");
      LL_AUTOPILOT_COMMAND_THRUST_MAX_VALUE             = my_xml_reader.readDoubleValue("midlevel_autopilot_config:midlevel_autopilot:LL_AUTOPILOT:LL_AUTOPILOT_COMMAND_THRUST_MAX_VALUE");

  } catch ( cvg_XMLFileReader_exception &e) {
      throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
  }

}

ThrustController::~ThrustController()
{
}

int ThrustController::configureAutopilot(std::string autopilot_configuration_file)
{
    return (int)true;
}

void ThrustController::init(std::string configFile)
{
    readConfigs(configFile);
    altSpeedController.init(configFile);
    hoverController.init(configFile);
    yawController.init(configFile);

    autopilot_is_started    = false;
    first_feedback_received = false;
    first_altitude_feedback_received = false;
    first_ground_speed_feedback_received = false;
    first_rotation_angles_feedback_received = false;
    is_last_ground_speed_measurement_valid = false;
    current_drone_mode = DroneState::INIT;
    flag_received_drone_mode_command = false;

    last_MLDAC_pitch_m1top1     = 0.0f;
    last_MLDAC_roll_m1top1      = 0.0f;
    last_MLDAC_alt_speed_m1top1 = 0.0f;
    last_MLDAC_dyaw_m1top1      = 0.0f;

    std::cout << "ThrustController(...), exit" << std::endl;


}

bool ThrustController::run()
{
    if (!first_feedback_received)
    {
      if(first_altitude_feedback_received && first_ground_speed_feedback_received && first_rotation_angles_feedback_received)
      {
       first_feedback_received = true;
      }
      else
      {
       return false;
      }
    }

    if(!autopilot_is_started)
    {
      return false;
    }
    // This function requires the following continuous feedback:
    //      Sensor measrurements and interaction with other nodes:
    //          - last autopilot_feedback_data: altitude, IMU rotation angles, and ground speed measurements
    //          - last_drone_mode_command
    //      Internal autopilot variables:
    //          - current_drone_mode

    preconditionCheck();
    /* bool state_change_ocurred = false;
    state_change_ocurred = */
    stateTransitionCheck();
    bool is_successful = true;
    is_successful = processState();
    computeDroneAutopilotCommand();

    return is_successful;
}

#define KEEP_IN_RANGE(a, min, max) if (a < min) a = min; else if (a > max) a = max;

void ThrustController::setAutopilotCommandData_PitchRoll(const double &pitch_cmd, const double &roll_cmd)
{
    if(!autopilot_is_started)
     return;

    if(current_drone_mode ==  DroneState::FLYING)
    {
      last_MLDAC_pitch_m1top1 = pitch_cmd;
      last_MLDAC_roll_m1top1 = roll_cmd;
    }

    KEEP_IN_RANGE( last_MLDAC_pitch_m1top1, -1.0, 1.0)
    KEEP_IN_RANGE( last_MLDAC_roll_m1top1 , -1.0, 1.0)
}

void ThrustController::setAutopilotCommandData_DYaw(const double &dyaw_cmd)
{
    if(!autopilot_is_started)
      return;

    if(current_drone_mode ==  DroneState::FLYING)
    {
      last_MLDAC_dyaw_m1top1 = dyaw_cmd;
    }

    KEEP_IN_RANGE( last_MLDAC_dyaw_m1top1, -1.0, 1.0)
}

void ThrustController::setAutopilotCommandData_DAltitude(const double &dalt_cmd)
{
    if(!autopilot_is_started)
      return;
    if(current_drone_mode ==  DroneState::FLYING)
    {
      last_MLDAC_alt_speed_m1top1 = dalt_cmd;
    }

    KEEP_IN_RANGE( last_MLDAC_alt_speed_m1top1, -1.0, 1.0)
}

int ThrustController::setDroneMeasurementRotationAngles(float roll_deg, float pitch_deg, float yaw_deg)
{
    last_roll_measurement  = roll_deg;
    last_pitch_measurement = pitch_deg;
    last_yaw_measurement   = yaw_deg;

    first_rotation_angles_feedback_received = true;
    return 1;
}

int ThrustController::setDroneMeasurementAltitude(float z_mps, float dz_mps)
{
    last_z_m    = z_mps;
    last_dz_mps = dz_mps;

    first_altitude_feedback_received = true;
    return 1;
}

int ThrustController::setDroneMeasurementGroundOpticalFlow(float vx_mps, float vy_mps)
{
    last_ground_speed_X_measurement = vx_mps;
    last_ground_speed_Y_measurement = vy_mps;

    is_last_ground_speed_measurement_valid = true;
    first_ground_speed_feedback_received   = true;
    return 1;
}

int ThrustController::setDroneFlyingModeCommand(DroneStateCommand::StateCommand last_drone_mode_command_in)
{
    last_drone_mode_command = last_drone_mode_command_in;
    flag_received_drone_mode_command = true;

    return 1;
}

void ThrustController::getDroneFlyingMode(DroneState::ModeType *const pcurrent_drone_status)
{
    (*pcurrent_drone_status) = current_drone_mode;
}

void ThrustController::getDroneLLAutopilotCommand(float *const pitch, float *const roll, float *const dyaw, float *const thrust)
{
    (*pitch)  = cur_DA_pitch_command_deg;
    (*roll)   = cur_DA_roll_command_deg;
    (*dyaw)   = cur_DA_dyaw_command_degps;
    (*thrust) = cur_DA_thrust_command_N;
}

void ThrustController::preconditionCheck()
{
    // Usually this function would modify the value of specific boolean monitonring variables.
    // This bool values can be checked later in the run() function, or in this function directly.

    switch (current_drone_mode)
    {
    case DroneState::INIT:
    {
    } break;

    case DroneState::LANDED:
    {
    } break;

    case DroneState::TAKING_OFF:
    {
    } break;

    case DroneState::HOVERING:
    {
    } break;

    case DroneState::FLYING:
    {
    } break;

    case DroneState::LANDING:
    {
    } break;

    case DroneState::EMERGENCY:
    {
    //case DroneState::UNKNOWN:
    } break;

    default:
    {
    //case DroneState::LOOPING:
    } break;
    }
}

bool ThrustController::stateTransitionCheck()
{
    DroneState::ModeType previous_drone_mode = current_drone_mode;

//  std::cout << "stateTransitionCheck(), -last_z_m:" << -last_z_m << std::endl;

#ifdef EMERGENCY_MODE_ENABLE
    if (last_drone_mode_command == DroneStateCommand::EMERGENCY_STOP)
        current_drone_mode = DroneState::EMERGENCY;
    else {
#endif
        switch (current_drone_mode) {

        case DroneState::INIT: {
            // This is the the first time feedback and mode must be determined. Are we landed or flying?
            // If we are flying, make it hover for safety.
            if(-last_z_m <= MAX_ALTITUDE_TO_CONSIDER_UNKNOWN_STATE_AS_LANDED)
            {
              current_drone_mode = DroneState::LANDED;
            }
            else
            {
              current_drone_mode = DroneState::HOVERING;
            }
        } break;

        case DroneState::LANDED: {
            if(last_drone_mode_command == DroneStateCommand::TAKE_OFF)
            {
              current_drone_mode = DroneState::TAKING_OFF;
              takingoffTimer.restart(false);
            }
        } break;

        case DroneState::TAKING_OFF: {
            if(-last_z_m > fabs(TAKEOFF_FINAL_ALTITUDE) || takingoffTimer.getElapsedSeconds() >= MAX_TAKINGOFF_TIME_S  )
            {
              current_drone_mode = DroneState::HOVERING;
            }
            if(last_drone_mode_command == DroneStateCommand::LAND)
               current_drone_mode = DroneState::LANDING;
            else if (last_drone_mode_command == DroneStateCommand::HOVER)
               current_drone_mode = DroneState::HOVERING;
        } break;

        case DroneState::HOVERING: {
            if(last_drone_mode_command == DroneStateCommand::MOVE)
              current_drone_mode = DroneState::FLYING;
            else if (last_drone_mode_command == DroneStateCommand::LAND)
              current_drone_mode = DroneState::LANDING;
        } break;

        case DroneState::FLYING: {
            if((last_drone_mode_command == DroneStateCommand::HOVER) || (last_drone_mode_command == DroneStateCommand::LAND))
            {
              last_MLDAC_pitch_m1top1     = 0.0f;
              last_MLDAC_roll_m1top1      = 0.0f;
              last_MLDAC_dyaw_m1top1      = 0.0f;
              last_MLDAC_alt_speed_m1top1 = 0.0f;

              switch (last_drone_mode_command) {
              case DroneStateCommand::HOVER: {
                    current_drone_mode = DroneState::HOVERING;
                } break;

              case DroneStateCommand::LAND: {
                    current_drone_mode = DroneState::LANDING;
                } break;

              default: { /* avoid compilation warnings */
                } break;
               }
            }
        } break;

        case DroneState::LANDING: {
            if(!landingWithTime)
            {
              if (-last_z_m < fabs(ALTITUDE_FOR_LANDING_WITH_TIME))
              {
                landingWithTime = true;
                landingTimer.restart(false);
              }
            }
            else
            {
             if(landingTimer.getElapsedSeconds() > LANDING_WITH_TIME_MAX_SECONDS)
             {
               current_drone_mode = DroneState::LANDED;
               landingWithTime = false;
             }
            }

            if(last_drone_mode_command == DroneStateCommand::TAKE_OFF)
            {
              current_drone_mode = DroneState::TAKING_OFF;
              takingoffTimer.restart(false);
              landingWithTime = false;
            }
        } break;

        case DroneState::EMERGENCY: {
#ifdef EMERGENCY_MODE_ENABLE
#else
#endif
        } break;

        default: {
            //        case DroneState::UNKNOWN:
            //        case DroneState::LOOPING:
        } break;
        }
#ifdef EMERGENCY_MODE_ENABLE
    }
#endif
    flag_received_drone_mode_command = false;

    return (previous_drone_mode != current_drone_mode);
}

bool ThrustController::processState()
{
    switch (current_drone_mode) {

    case DroneState::INIT: {
        altSpeedController.enable(false);
        hoverController.enable(false);
        yawController.enable(false);

        cur_DA_pitch_command_m1top1     = 0.0f;
        cur_DA_roll_command_m1top1      = 0.0f;
        cur_DA_alt_speed_command_m1top1 = 0.0f;
        cur_DA_dyaw_command_m1top1      = 0.0f;
    } break;

    case DroneState::LANDED: {
        altSpeedController.enable(false);
        hoverController.enable(false);
        yawController.enable(false);

        cur_DA_pitch_command_m1top1     = last_pitch_measurement / LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
        cur_DA_roll_command_m1top1      = last_roll_measurement  / LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
        cur_DA_alt_speed_command_m1top1 = 0.0f;
        cur_DA_dyaw_command_m1top1      = 0.0f;
    } break;

    case DroneState::TAKING_OFF: {
        altSpeedController.enable(true);
        hoverController.enable(true);
        yawController.enable(true);

        // [set by hoverController] cur_DA_pitch_command_m1top1;
        // [set by hoverController] cur_DA_roll_command_m1top1 ;
        if (-last_z_m < fabs(TAKEOFF_FINAL_ALTITUDE))
        {
         if (-last_z_m > ON_GROUND_ALTITUDE)
         {
           // A=altFactor    is in [0, 1.0]. init_altitude:0, final_altitude:1
           double altFactor = ((last_z_m + ON_GROUND_ALTITUDE) / (-TAKEOFF_FINAL_ALTITUDE + ON_GROUND_ALTITUDE));
           // B=altFactorCos is in [0, 0.5]. init_altitude:0, final altitude:0.5. Note that B has -cos() shape/concavity.
           double altFactorCos = 1.0 - 0.5 * (1 + cos(altFactor * M_PI));
           // vh = vh0*(1-B)^gamma, where gamma is around 2. Then vh is in take-offspeed*[1, 0.25]. init_altitude: vh0, final altitude:0.25*vh0
           cur_DA_alt_speed_command_m1top1 = TAKEOFF_SPEED * pow(1.0 -  altFactorCos, TAKINGOFF_ALT_SPEED_COS_GAMMA) / ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED;
         }
         else
         {
           cur_DA_alt_speed_command_m1top1 = TAKEOFF_SPEED / ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED;
         }
        }
        else
        {
          cur_DA_alt_speed_command_m1top1 = 0.0f;
        }

        cur_DA_dyaw_command_m1top1 = 0.0;
    } break;

    case DroneState::HOVERING: {
        altSpeedController.enable(true);
        hoverController.enable(true);
        yawController.enable(true);

        // [set by hoverController] cur_DA_pitch_command_m1top1;
        // [set by hoverController] cur_DA_roll_command_m1top1 ;
        cur_DA_alt_speed_command_m1top1 = 0.0f;
        cur_DA_dyaw_command_m1top1      = 0.0;
    } break;

    case DroneState::FLYING: {
        altSpeedController.enable(true);
        hoverController.enable(false); //Estos controladores se podrian activar para evitar que se vaya
        yawController.enable(false);   //el Pelican en las pruebas de vuelo con DMC

        // set by other module
        cur_DA_pitch_command_m1top1     = last_MLDAC_pitch_m1top1;
        cur_DA_roll_command_m1top1      = last_MLDAC_roll_m1top1;
        cur_DA_alt_speed_command_m1top1 = last_MLDAC_alt_speed_m1top1;
        cur_DA_dyaw_command_m1top1      = last_MLDAC_dyaw_m1top1;

        // check whether the drone is flying too high
        if(-last_z_m > fabs(MAX_ALTITUDE))
          cur_DA_alt_speed_command_m1top1 = -fabs(MAX_ALTITUDE_DESCENDING_SPEED / ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED);
    } break;

    case DroneState::LANDING: {
        altSpeedController.enable(true);
        hoverController.enable(true);
        yawController.enable(true);

        // [set by hoverController] cur_DA_pitch_command_m1top1;
        // [set by hoverController] cur_DA_roll_command_m1top1 ;
        cur_DA_alt_speed_command_m1top1 = -fabs(LANDING_SPEED / ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED);
        cur_DA_dyaw_command_m1top1      = 0.0;
    } break;

    case DroneState::EMERGENCY: {
//        case DroneState::UNKNOWN:
#ifdef EMERGENCY_MODE_ENABLE
        hoverController.enable(false);
        altSpeedController.enable(false);
        yawController.enable(false);
        cur_DA_pitch_command_m1top1     = last_pitch_measurement / LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
        cur_DA_roll_command_m1top1      = last_roll_measurement  / LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
        cur_DA_dyaw_command_m1top1      = 0;
        cur_DA_alt_speed_command_m1top1 = 0;
        // Added to re-enable take off
//        current_drone_mode = DroneState::LANDED;
#else
#endif
    } break;

    default: {
//      case DroneState::LOOPING:
        altSpeedController.enable(false);
        hoverController.enable(false);
        yawController.enable(false);
        cur_DA_pitch_command_m1top1     = 0.0f;
        cur_DA_roll_command_m1top1      = 0.0f;
        cur_DA_alt_speed_command_m1top1 = 0.0f;
        cur_DA_dyaw_command_m1top1      = 0.0f;
    } break;

    }

    return true;
}

void ThrustController::computeDroneAutopilotCommand()
{
//char str[512];
//sprintf(str, "y: %.2f, p: %.2f (%.2f), r: %.2f (%.2f), t: %.2f\n", cur_DA_dyaw_command_degps * LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE,
//                                                                   cur_DA_pitch_command_m1top1 * LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE,
//                                                                   last_ground_speed_X_measurement,
//                                                                   cur_DA_roll_command_m1top1 * LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE,
//                                                                   last_ground_speed_Y_measurement,
//                                                                   cur_DA_alt_speed_command_m1top1);
//std::cout << str;

    // *** Altitude Speed controller ***
    //altSpeedController.updateAltitude( -last_z_m, ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED);
    altSpeedController.updateAltitude( -last_z_m,  -last_dz_mps, ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED);
    altSpeedController.setReference(cur_DA_alt_speed_command_m1top1 * ML_AUTOPILOT_MAX_ALLOWED_ALTITUDE_SPEED);
    if(!landingWithTime)
    {
      cur_DA_thrust_command_N  = altSpeedController.getOutput() * LL_AUTOPILOT_COMMAND_THRUST_MAX_VALUE;
      cur_DA_thrust_command_N /= ( cos(last_pitch_measurement*(M_PI/180.0))*cos(last_roll_measurement*(M_PI/180.0)) );
    }
//    else if (emergency_flag_)
//    {
//      cur_DA_thrust_command_N = 0;
//    }
    else
    {
      cur_DA_thrust_command_N = LANDING_WITH_TIME_FIXED_THRUST * LL_AUTOPILOT_COMMAND_THRUST_MAX_VALUE;
    }

    // *** Hover controller ***
    if (hoverController.isEnabled())
    {
      if (is_last_ground_speed_measurement_valid)
      {
        float pitchCmdDeg = cur_DA_pitch_command_deg, rollCmdDeg = cur_DA_roll_command_deg;
        hoverController.run(last_ground_speed_X_measurement, last_ground_speed_Y_measurement, -last_z_m, &pitchCmdDeg, &rollCmdDeg);
        cur_DA_pitch_command_m1top1 = pitchCmdDeg / LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
        cur_DA_roll_command_m1top1  = rollCmdDeg  / LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
      }
      else
      {
        cur_DA_pitch_command_m1top1 = 0.0f;
        cur_DA_roll_command_m1top1  = 0.0f;
      }
    }

    // *** Yaw controller ***
    cur_DA_dyaw_command_m1top1_aux = 0.0f;
    if (yawController.isEnabled())
    {
      // If there's a recent measure, feed it back to the control loop
      yawController.setFeedback(last_yaw_measurement);
      // What is the expected yaw position, according to the commanded yaw rate?
      double elapsed = yawRateTimer.getElapsedSeconds();
      yawRateTimer.restart(true);
      double newYawRef = yawController.getReference() + elapsed * (cur_DA_dyaw_command_m1top1 * LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE);
      yawController.setReference(newYawRef);
      cur_DA_dyaw_command_m1top1_aux = yawController.getOutput() / LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE;
    }
    else
    {
      yawRateTimer.restart(false);
      yawController.setReference(last_yaw_measurement);
      cur_DA_dyaw_command_m1top1_aux = cur_DA_dyaw_command_m1top1;
    }

    cur_DA_pitch_command_deg  =  cur_DA_pitch_command_m1top1     * LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
    cur_DA_roll_command_deg   =  cur_DA_roll_command_m1top1      * LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE;
    cur_DA_dyaw_command_degps =  cur_DA_dyaw_command_m1top1_aux  * LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE;
    cur_DA_thrust_command_N   =  cur_DA_thrust_command_N;

    KEEP_IN_RANGE( cur_DA_pitch_command_deg,  LL_AUTOPILOT_COMMAND_TILT_MIN_VALUE,   LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE  )
    KEEP_IN_RANGE( cur_DA_roll_command_deg,   LL_AUTOPILOT_COMMAND_TILT_MIN_VALUE,   LL_AUTOPILOT_COMMAND_TILT_MAX_VALUE  )
    KEEP_IN_RANGE( cur_DA_dyaw_command_degps, LL_AUTOPILOT_COMMAND_DYAW_MIN_VALUE,   LL_AUTOPILOT_COMMAND_DYAW_MAX_VALUE  )
    KEEP_IN_RANGE( cur_DA_thrust_command_N,   LL_AUTOPILOT_COMMAND_THRUST_MIN_VALUE, LL_AUTOPILOT_COMMAND_THRUST_MAX_VALUE)
}


bool ThrustController::close()
{
    stop();

    return true;
}

bool ThrustController::start()
{
    if (autopilot_is_started) return true;

    first_feedback_received = false;
    first_altitude_feedback_received = false;
    first_ground_speed_feedback_received = false;
    first_rotation_angles_feedback_received = false;
    is_last_ground_speed_measurement_valid = false;
    last_ground_speed_X_measurement = 0.0f;
    last_ground_speed_Y_measurement = 0.0f;
    last_pitch_measurement = 0.0f;
    last_roll_measurement = 0.0f;
    last_yaw_measurement = 0.0f;
    last_z_m = ON_GROUND_ALTITUDE;

    // start-up flags
    current_drone_mode = DroneState::INIT;              // UNKNOWN until first sample is received
    last_drone_mode_command = DroneStateCommand::INIT;  // Initial value
    landingWithTime = false;
    // In case the initial mode is determined as HOVERING, set yaw and altitude rate to 0
    cur_DA_alt_speed_command_m1top1 = 0.0f;
    cur_DA_dyaw_command_m1top1      = 0.0f;
    cur_DA_pitch_command_m1top1     = 0.0f;
    cur_DA_roll_command_m1top1      = 0.0f;
    cur_DA_thrust_command_N   = 0.0f;
    cur_DA_dyaw_command_degps = 0.0f;
    cur_DA_pitch_command_deg  = 0.0f;
    cur_DA_roll_command_deg   = 0.0f;

    yawController.setMaxAllowedOutput(ML_AUTOPILOT_MAX_ALLOWED_YAW_SPEED);

    flag_received_drone_mode_command = false;
    autopilot_is_started = true;

    return true;
}

bool ThrustController::stop()
{
  // The drone can be stoped using the radio control. This "stop" function is more dangerous
 //  than useful. Which is why is left empty.

//    if (!autopilot_is_started) return;

//    autopilot_is_started = false;
//    first_feedback_received = false;
//    is_last_ground_speed_measurement_valid = false;

    return true;
}
