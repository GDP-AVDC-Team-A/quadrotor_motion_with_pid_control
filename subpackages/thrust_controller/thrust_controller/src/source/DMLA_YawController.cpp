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
#include "DMLA_YawController.h"

namespace CVG
{
namespace MAV
{

YawController::YawController(){
}

bool YawController::readConfigs(std::string configFile)
{
  try {
      XMLFileReader my_xml_reader(configFile);

      GAIN_P = my_xml_reader.readDoubleValue("midlevel_autopilot_config:yaw_controller:GAIN_P");
      GAIN_I = my_xml_reader.readDoubleValue("midlevel_autopilot_config:yaw_controller:GAIN_I");
      GAIN_D = my_xml_reader.readDoubleValue("midlevel_autopilot_config:yaw_controller:GAIN_D");

  } catch ( cvg_XMLFileReader_exception &e) {
      throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
  }
}

void YawController::init(std::string configFile)
{
    readConfigs(configFile);

    yawPid.setGains(GAIN_P, GAIN_I, GAIN_D);
    enabled = false;
    reference = feedback = 0.0;

    std::cout << "YawController(...), exit "<< std::endl;
}

void YawController::setReference(double yawRef_deg)
{
    reference = yawRef_deg;
    calcError();
}

void YawController::setFeedback(double yawMeasure_deg)
{
    feedback = yawMeasure_deg;
    calcError();
}

void YawController::calcError()
{
    double err = fmod(reference - feedback, 360.0);
    if (err < -180) err += 360;
    else if (err > 180) err -= 360;
    yawPid.setReference( 0.0 );
    yawPid.setFeedback( -err );
}

double YawController::getOutput()
{
    return yawPid.getOutput();
}

void YawController::enable(bool e)
{
    if (enabled != e)
    {
      yawPid.reset();
    }

    enabled = e;
}

}
}

