//##################################################
//# PROJECT: P1MC PID
//# AUTHOR : li chunjing
//##################################################
/*******************************************************************************
* Copyright (c) 2017, UBT CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of UBT nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <stdio.h>
#include <algorithm>
#include <controller/pid.h>


Pid_Controller::Pid_Controller(float value)
{
	Kp = value;
	Ki = value;
	Kd1 = value;
	Kd2 = value;

	Ui = value;

	Ud1 = value;
	Ud2 = value;

	refValue = value;
	fbcakValue = value;
	fbcakValue_d = value;    //differential of feed back value

	outMin = value;
	outMax = value;

	Out = value;
}

Pid_Controller::Pid_Controller()
{
	Kp = 0;
	Ki = 0;
	Kd1 = 0;
	Kd2 = 0;

	Ui = 0;

	Ud1 = 0;
	Ud2 = 0;

	refValue = 0;
	refValue_1=0;
	fbcakValue = 0;
	fbcakValue_d = 0;    //differential of feed back value

	outMin = 0;
	outMax = 0;

	Out = 0;
}

void Pid_Controller::PID_setgains(float kp, float ki, float kd1, float kd2)
{
	Kp = kp;
	Ki = ki;
	Kd1 = kd1;
	Kd2 = kd2;
}
	
void Pid_Controller::PID_setrefValue(float refvalue)
{
	refValue = refvalue;
}

void Pid_Controller::PID_setfbackValue(float fbcakvalue)
{
	fbcakValue = fbcakvalue;
}

void Pid_Controller::PID_setfbackValue_d(float fbcakvalue_d)
{
	fbcakValue_d = fbcakvalue_d;
}

void Pid_Controller::PID_setoutMin(float outmin)
{
	outMin = outmin;
}

void Pid_Controller::PID_setoutMax(float outmax)
{
	outMax = outmax;
}

void Pid_Controller::PID_run()
{
    float Error; 
	float Up;

	Error = refValue - fbcakValue;
	
	Up = Kp * Error;
	Ui = Ui + Ki * Error;
	Ud1 = -Kd1 * fbcakValue_d;
	Ud2 = Kd2 * (refValue - refValue_1) / CTRL_PERIOD_S;

	Out = Up + Ui + Ud1 +Ud2;

	if(Out <= outMin) Out = outMin;
	if(Out >= outMax) Out = outMax;
	refValue_1 = refValue;
}

float Pid_Controller::PID_getOut()
{
   return Out;
}

float Pid_Controller::PID_getrefValue()
{
   return refValue;
}

float Pid_Controller::PID_getfbackValue()
{
   return fbcakValue;
}

float Pid_Controller::PID_getfbackValue_d()
{
   return fbcakValue_d;
}