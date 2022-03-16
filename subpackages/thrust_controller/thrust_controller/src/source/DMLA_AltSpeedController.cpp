/*!********************************************************************************
 * \brief     Thrust controller implementation
 * \authors   Ignacio Mellado, Jesus Pestana, Miguel Gomez
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

#include <iostream>
#include "DMLA_AltSpeedController.h"

namespace CVG
{
namespace MAV
{

AltSpeedController::AltSpeedController()
{

}

bool AltSpeedController::readConfigs(std::string configFile)
{
  try {
      XMLFileReader my_xml_reader(configFile);

      // 21 July 2014, Pelican (Full) Weight: 2.065 Kg
      //               convert to counts using asctec's conversion factor:
      //               2.065 * 9.81/(4095/32)            = 2600 counts
      //               experimental feedforward constant = 2650 counts
      double FEEDFORWARD_M;
      FEEDFORWARD_M                       = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:FEEDFORWARD_M");
      THRUSTER_GAIN                       = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:THRUSTER_GAIN");
      FEEDFORWARD                         = FEEDFORWARD_M * THRUSTER_GAIN;
      GAIN_P                              = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:GAIN_P");
      GAIN_I                              = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:GAIN_I");
      GAIN_D                              = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:GAIN_D");

      ALT_SPEED_MEASURE_SATURATION_FACTOR = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:ALT_SPEED_MEASURE_SATURATION_FACTOR");

      COMMAND_UPPER_LIMIT                 = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:COMMAND_UPPER_LIMIT");
      COMMAND_LOWER_LIMIT                 = my_xml_reader.readDoubleValue("midlevel_autopilot_config:altitude_speed_controller:COMMAND_LOWER_LIMIT");
      SUBSAMPLING_DIVIDER                 = my_xml_reader.readIntValue("midlevel_autopilot_config:altitude_speed_controller:SUBSAMPLING_DIVIDER");

  } catch ( cvg_XMLFileReader_exception &e) {
      throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
  }

}

void AltSpeedController::init(std::string configFile)
{
      readConfigs(configFile);

    //    std::cout << "FEEDFORWARD:" << FEEDFORWARD << std::endl;
    //    std::cout << "GAIN_P:" <<  GAIN_P << std::endl;
    //    std::cout << "GAIN_I:" <<  GAIN_I << std::endl;
    //    std::cout << "GAIN_D:" <<  GAIN_D << std::endl;

    //    std::cout << "ALT_SPEED_MEASURE_SATURATION_FACTOR:" <<  ALT_SPEED_MEASURE_SATURATION_FACTOR << std::endl;

    //    std::cout << "COMMAND_UPPER_LIMIT:" <<  COMMAND_UPPER_LIMIT << std::endl;
    //    std::cout << "COMMAND_LOWER_LIMIT:" <<  COMMAND_LOWER_LIMIT << std::endl;
    //    std::cout << "SUBSAMPLING_DIVIDER:" <<  SUBSAMPLING_DIVIDER << std::endl;

      controller_module.setGains(GAIN_P, GAIN_I, GAIN_D);

      controller_module.enableAntiWindup(true, COMMAND_LOWER_LIMIT, COMMAND_UPPER_LIMIT);
      controller_module.enableMaxOutput( true, COMMAND_LOWER_LIMIT, COMMAND_UPPER_LIMIT);
      started = false;
      debugCount = 0;
      enabled = false;
      repeatCount = 0;
      lastSpeed = lastSpeedF = 0;

    //	float num[] = { 0.079955585370677, 0.0 };
    //	float den[] = { 1.0, -0.920044414629323 };

      float num[] = { 0.153518275109386, 0.0 };
      float den[] = { 1.0, -0.846481724890614 };
      outputFilter.create(1, num, den);

      std::cout << "AltSpeedController(...), exit "<< std::endl;

}

void AltSpeedController::setReference(double ref)
{
#ifdef SHOW_DEBUG
    debugRef(std::string("AS_ref:") + ref);
#endif
    controller_module.setReference(ref);
}

double AltSpeedController::getOutput()
{
      if (!enabled)
      {
#ifdef SHOW_DEBUG
        debugRef(std::string("AS_cmd:") + 0.0);
#endif
      return 0.0;
      }

//    if (repeatCount == 0) {

     double Dthrust_1p1 = controller_module.getOutput();
     thrust_1p1 = FEEDFORWARD + Dthrust_1p1;
//    }
#ifdef SHOW_DEBUG
    debugRef(std::string("AS_cmd:") + thrust_1p1);
#endif
    //std::cout << "thrust_1p1:" << thrust_1p1 << std::endl;

    return thrust_1p1;
}

void AltSpeedController::updateAltitude(double curAlt, double maxAltSpeedCommand)
{
    if (!enabled) return;

    double fAlt = curAlt;

    repeatCount++;

    if(repeatCount >= SUBSAMPLING_DIVIDER)
      repeatCount = 0;

    double altSpeed, altSpeedF;
    if(repeatCount == 0)
      {
        if(!started)
        {
          started = true;
          lastAlt = fAlt;
        }

	double elapsed = dTimer.getElapsedSeconds();
	dTimer.restart();
	altSpeed = (fAlt - lastAlt) / elapsed;

        // Altitude speed saturation to avoid altitude sensor peaks
        double speedLimit = fabs(maxAltSpeedCommand * ALT_SPEED_MEASURE_SATURATION_FACTOR);
        if (altSpeed > speedLimit)
          altSpeed = speedLimit;
        else if (altSpeed < -speedLimit)
          altSpeed = -speedLimit;

        lastAlt = fAlt;
        lastSpeed = altSpeed;
    }
    else
    {
        altSpeed = lastSpeed;
    }

    altSpeedF = outputFilter.filter(altSpeed);

#ifdef SHOW_DEBUG
    debugInfo(std::string("A_measure:") + curAlt + " A_filt:" + fAlt + " AS_calc:" + altSpeed + " AS_filt:" + altSpeedF);
#endif
    controller_module.setFeedback(altSpeedF);
}

void AltSpeedController::updateAltitude(double altitude, double altitude_speed, double maxAltSpeedCommand)
{
    lastAlt   = altitude;
    lastSpeed = altitude_speed;

    if (!enabled) return;

    controller_module.setFeedback(altitude_speed);
}

void AltSpeedController::enable(bool e)
{
    if(enabled && !e)
    {
      controller_module.reset();
      started = false;
    }

    else if(!enabled && e)
    {
      controller_module.reset();
      started = false;
    }

    enabled = e;
}

void AltSpeedController::debugRef(const std::string &str)
{
    if (debugCount > 0)
      std::cout << " ";
    else
      std::cout << debugTimer.getElapsedSeconds() << " [cmd] ";

    std::cout << str;
    debugCount++;
    if (debugCount >= 2)
    {
      std::cout << std::endl;
      debugCount = 0;
    }

}

void AltSpeedController::debugInfo(const std::string &str)
{
    std::cout << debugTimer.getElapsedSeconds() << " [info] " << str.c_str() << std::endl;
}

}
}
