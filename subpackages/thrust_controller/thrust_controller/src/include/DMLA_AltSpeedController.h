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

#ifndef ALTSPEEDCONTROLLER_H_
#define ALTSPEEDCONTROLLER_H_

#include <string>
#include <math.h>

#include "control/Filter.h"
#include <iostream>
#include "xmlfilereader.h"
#include "Timer.h"

//#define ALTSPEEDCONTROLLER_USE_DMC_CONTROL_INSTEAD_OF_PID
#ifndef ALTSPEEDCONTROLLER_USE_DMC_CONTROL_INSTEAD_OF_PID
#include "control/PID.h"
#else
#include "control/DMC.h"
#endif


namespace CVG {
namespace MAV {

class AltSpeedController {
private:
#ifndef ALTSPEEDCONTROLLER_USE_DMC_CONTROL_INSTEAD_OF_PID
    CVG_BlockDiagram::PID controller_module;
#else
    CVG_BlockDiagram::DMC controller_module;
#endif
    double thrust_1p1;
    double lastAlt;
    bool started;
    Timer dTimer;
    bool enabled;

    int debugCount;
    Timer debugTimer;
    int repeatCount;
    double lastSpeed, lastSpeedF;
    CVG_BlockDiagram::Filter outputFilter;

public:

    void init(std::string configFile);
    bool readConfigs(std::string configFile);

public:
    AltSpeedController();

    void updateAltitude(double altitude, double maxAltSpeedCommand); // Nacho's Function
    void updateAltitude(double altitude, double altitude_speed, double maxAltSpeedCommand);
    void setReference(double ref);
    double getOutput();

    void debugRef(const std::string &str);
    void debugInfo(const std::string &str);

    void enable(bool e);

private:
    // Configuration parameters
    double FEEDFORWARD, THRUSTER_GAIN;
    double GAIN_P, GAIN_I, GAIN_D;

    double ALT_SPEED_MEASURE_SATURATION_FACTOR;

    double COMMAND_UPPER_LIMIT;
    double COMMAND_LOWER_LIMIT;
    int    SUBSAMPLING_DIVIDER;

public:
    void setAltitudeDynamicGains(double feedforward, double kp, double ki, double kd){
        controller_module.setGains(kp, ki, kd);
        FEEDFORWARD = feedforward * THRUSTER_GAIN;
    }

};

}
}

#endif /* ALTSPEEDCONTROLLER_H_ */
