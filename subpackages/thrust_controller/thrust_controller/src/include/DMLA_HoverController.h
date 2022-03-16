/*!********************************************************************************
 * \brief     Thrust controller implementation
 * \authors   Ignacio Mellado
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

#ifndef HOVERCONTROLLER_H_
#define HOVERCONTROLLER_H_

#include "control/PID.h"
#include <string>
#include <iostream>
#include "xmlfilereader.h"

namespace CVG {
namespace MAV {

class HoverController {

public:
    HoverController();
    void run(double Vx_measure, double Vy_measure, double altitude, float *pitchCmdDeg, float *rollCmdDeg);
    void enable(bool e);
    inline bool isEnabled() { return enabled; }

public:
    void init(std::string configFile);
    bool readConfigs(std::string configFile);

private:
    CVG_BlockDiagram::PID pidVx, pidVy;
    bool enabled;

    // Configuration parameters
    double GAIN_P, GAIN_I, GAIN_D;
    double MAX_OUTPUT;
    double ALTITUDE_FOR_NOMINAL_D;
};

}
}

#endif /* HOVERCONTROLLER_H_ */
