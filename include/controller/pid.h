//##################################################
//# PROJECT: P1MC Makefile
//# AUTHOR : li chunjing
//##################################################
/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
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
* * Neither the name of ROBOTIS nor the names of its
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
#ifndef _PID_H_
#define _PID_H_

#include <algorithm>
#include <stdint.h>
#define CTRL_PERIOD_S   0.005

class  Pid_Controller
{
	private:
		float Kp;
		float Ki;
		float Kd1;
		float Kd2;

		float Ui;

		float Ud1;
		float Ud2;

		float refValue;
		float refValue_1;
		float fbcakValue;
		float fbcakValue_d;

		float outMin;
		float outMax;
		float Out;

	public:
		Pid_Controller(float value);
		Pid_Controller();

		void PID_setgains(float kp, float ki, float kd1, float kd2);
		void PID_setrefValue(float refvalue);
		void PID_setfbackValue(float fbcakvalue);
		void PID_setfbackValue_d(float fbcakvalue_d);
		void PID_setoutMin(float outmin);
		void PID_setoutMax(float outmax);

		void PID_run();
		
		float PID_getOut();
		float PID_getrefValue();
	    float PID_getfbackValue();
	    float PID_getfbackValue_d();

};

#endif