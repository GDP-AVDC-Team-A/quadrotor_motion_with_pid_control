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

int main(int argc,char **argv)
{
    // Ros Init
    ros::init(argc, argv, "THRUST CONTROLLER");
    ros::NodeHandle n;

    std::cout << "[ROSNODE] Starting " << "THRUST CONTROLLER" << std::endl;

    // Vars
    ThrustControllerProcess thrust_controller;
    thrust_controller.setUp();
    //thrust_controller.start();

    ros::Rate loop_rate(thrust_controller.get_moduleRate());

#ifdef DRONE_DYNAMIC_TUNING
    //dynamic Reconfigure
    dynamic_reconfigure::Server<thrustController::thrustControllerConfig> server;
    dynamic_reconfigure::Server<thrustController::thrustControllerConfig>::CallbackType f;

    f = boost::bind(&ThrustControllerProcess::parametersCallback, &thrust_controller, _1, _2);
    server.setCallback(f);
#endif

	try
	{
        while(ros::ok())
		{
            //Read messages
            ros::spinOnce();

            //Run retina
            thrust_controller.run();

            //Sleep
            loop_rate.sleep();
		}
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}
