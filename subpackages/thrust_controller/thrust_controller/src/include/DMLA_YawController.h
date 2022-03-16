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

#ifndef YAWCONTROLLER_H_
#define YAWCONTROLLER_H_

#include <math.h>
#include "control/PID.h"
#include <string>
#include <iostream>
#include "xmlfilereader.h"

namespace CVG {
namespace MAV {

class YawController {
public:
    YawController();
    void setFeedback(double yawMeasure_deg);
    double getOutput();
    void setReference(double yawRef_deg);
    void enable(bool e);
    inline bool isEnabled() { return enabled; }
    inline void setMaxAllowedOutput(double maxOutput) { yawPid.enableMaxOutput(true, maxOutput); }
    inline double getReference() { return reference; }
    double mapAngleToMinusPlusPi(double angleRads);

public:
    void init(std::string configFile);
    bool readConfigs(std::string configFile);

protected:
    void calcError();

private:
    CVG_BlockDiagram::PID yawPid;
    bool enabled;
    double reference, feedback;

    // Configuration parameters
    double GAIN_P, GAIN_I, GAIN_D;
};

}
}

#endif /* YAWCONTROLLER_H_ */
