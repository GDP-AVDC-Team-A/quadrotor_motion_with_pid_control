/*!********************************************************************************
 * \brief     quadrotor_pid_thrust_control implementation
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

#include "../include/behavior_quadrotor_pid_thrust_control.h"

namespace quadrotor_motion_with_pid_control
{
BehaviorQuadrotorPidThrustControl::BehaviorQuadrotorPidThrustControl() : BehaviorExecutionController() { 
  setName("quadrotor_pid_thrust_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorQuadrotorPidThrustControl::~BehaviorQuadrotorPidThrustControl() {}

void BehaviorQuadrotorPidThrustControl::onConfigure()
{

  node_handle = getNodeHandle();
  nspace = getNamespace();
}

bool BehaviorQuadrotorPidThrustControl::checkSituation()
{
  return true;
}

void BehaviorQuadrotorPidThrustControl::onActivate()
{
  std::cout<<"ACTIVATED THRUST CONTROLLER"<<std::endl;
  ros::ServiceClient start_controller=node_handle.serviceClient<std_srvs::Empty>("/"+nspace+"/thrust_controller/start");
  std_srvs::Empty req;
  start_controller.call(req);
}

void BehaviorQuadrotorPidThrustControl::onDeactivate()
{
  std::cout<<"DEACTIVATED THRUST CONTROLLER"<<std::endl;
  ros::ServiceClient stop_controller=node_handle.serviceClient<std_srvs::Empty>("/"+nspace+"/thrust_controller/stop");
  std_srvs::Empty req;
  stop_controller.call(req);
}

void BehaviorQuadrotorPidThrustControl::onExecute()
{ 

}

void BehaviorQuadrotorPidThrustControl::checkGoal(){}


void BehaviorQuadrotorPidThrustControl::checkProgress() {

}


void BehaviorQuadrotorPidThrustControl::checkProcesses() 
{ 
 
}

}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_pid_control::BehaviorQuadrotorPidThrustControl, nodelet::Nodelet)