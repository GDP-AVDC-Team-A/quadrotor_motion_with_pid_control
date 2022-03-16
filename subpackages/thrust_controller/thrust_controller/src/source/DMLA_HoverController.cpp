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

#include "DMLA_HoverController.h"

namespace CVG
{
namespace MAV
{

HoverController::HoverController(){
}

bool HoverController::readConfigs(std::string configFile)
{
  try {
      XMLFileReader my_xml_reader(configFile);

      GAIN_P                 = my_xml_reader.readDoubleValue("midlevel_autopilot_config:hover_controller:GAIN_P");
      GAIN_I                 = my_xml_reader.readDoubleValue("midlevel_autopilot_config:hover_controller:GAIN_I");
      GAIN_D                 = my_xml_reader.readDoubleValue("midlevel_autopilot_config:hover_controller:GAIN_D");
      MAX_OUTPUT             = my_xml_reader.readDoubleValue("midlevel_autopilot_config:hover_controller:MAX_OUTPUT");
      ALTITUDE_FOR_NOMINAL_D = my_xml_reader.readDoubleValue("midlevel_autopilot_config:hover_controller:ALTITUDE_FOR_NOMINAL_D");

  } catch ( cvg_XMLFileReader_exception &e) {
      throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
  }

}

void HoverController::init(std::string configFile)
{
    readConfigs(configFile);

    pidVx.setReference(0.0);
    pidVy.setReference(0.0);

    pidVx.setGains(-GAIN_P, -GAIN_I, -GAIN_D);
    pidVy.setGains( GAIN_P,  GAIN_I,  GAIN_D);
    pidVx.enableMaxOutput(true, MAX_OUTPUT);
    pidVy.enableMaxOutput(true, MAX_OUTPUT);

    pidVx.enableAntiWindup(true, MAX_OUTPUT);
    pidVy.enableAntiWindup(true, MAX_OUTPUT);

    enabled = false;

    std::cout << "HoverController(...), exit" << std::endl;

}

void HoverController::run(double Vx_measure, double Vy_measure, double altitude, float *pitchCmdDeg, float *rollCmdDeg)
{
    if(!isEnabled())
    {
      (*pitchCmdDeg) = 0.0f;
      (*rollCmdDeg)  = 0.0f;
      return;
    }

    double D = GAIN_D;
    if(altitude <= ALTITUDE_FOR_NOMINAL_D)
    {
      if(altitude > ALTITUDE_FOR_NOMINAL_D * 0.5)
      {
        D -= 2.0 * GAIN_D * (ALTITUDE_FOR_NOMINAL_D - altitude) /  ALTITUDE_FOR_NOMINAL_D;
        if (D <= 0.0) D = 0.0;
      }
      else
        D = 0.0;
    }

    pidVx.setGains( GAIN_P,  GAIN_I,  D);
    pidVy.setGains( GAIN_P,  GAIN_I,  D);
    pidVx.setFeedback(Vx_measure);
    pidVy.setFeedback(Vy_measure);

    (*pitchCmdDeg) = (-1.0) * pidVx.getOutput();
    (*rollCmdDeg)  =          pidVy.getOutput();
}

void HoverController::enable(bool e)
{
    if (enabled != e)
    {
      pidVx.reset();
      pidVy.reset();
    }

    enabled = e;
}

}
}
