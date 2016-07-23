// PID.cpp

#include "PID.h"

extern "C" {
#include "stm32f4xx_hal.h"
#include "math_ext.h"
}

float PID::get_pid(float error, float scaler)
{
	uint32_t currTime = HAL_GetTick();  
	uint32_t dT = currTime - m_lastTime;
	float output = 0;

	// Check if the last update exceeds the update time, and prevent Integration buildup error
	if (m_lastTime == 0 || dT > 1000) 
	{
		dT = 0;
		reset_I();
	}
	
	m_lastTime = currTime;
	float fDT = (float)dT / 1000.0f;

	// Compute proportional component
	m_pidInfo.P = error * m_kp;
	output += m_pidInfo.P;

	// Compute derivative component if time has elapsed
	if ((fabsf(m_kd) > 0) && (dT > 0))
	{
		float derivative;

		if (__isnanf(m_lastDerivative))
		{
			// Since we reset, suppress the first derivative term as we don't want a sudden change in input to cause a large D output change			
			derivative = 0;
			m_lastDerivative = 0;
		}
		else
		{
			derivative = (error - m_lastError) / fDT;
		}

        // discrete low pass filter, cuts out the high frequency noise that can drive the controller crazy
		float RC = 1 / (2 * M_PI * m_fCut);
		derivative = m_lastDerivative + ((fDT / (RC + fDT)) * (derivative - m_lastDerivative));

		                      // update state
		m_lastError = error;
		m_lastDerivative = derivative;

		        // add in derivative component
		m_pidInfo.D = m_kd * derivative;
		output += m_pidInfo.D;
	}

	    // scale the P and D components
	output *= scaler;
	m_pidInfo.D *= scaler;
	m_pidInfo.P *= scaler;

	// Compute integral component if time has elapsed
	if ((fabsf(m_ki) > 0) && (dT > 0))
	{
		m_integrator += (error * m_ki) * scaler * fDT;
		if (m_integrator < -m_imax) 
		{
			m_integrator = -m_imax;
		}
		else if (m_integrator > m_imax)
		{
			m_integrator = m_imax;
		}
		m_pidInfo.I = m_integrator;
		output += m_integrator;
	}

	m_pidInfo.desired = output;
	return output;
}

void PID::reset_I()
{
	m_integrator = 0;
	// we use NAN (Not A Number) to indicate that the last 
	// derivative value is not valid
	m_lastDerivative = NAN;
	m_pidInfo.I = 0;
}

void PID::load_gains()
{
//	_kp.load();
//	_ki.load();
//	_kd.load();
//	_imax.load();
}

void PID::save_gains()
{
//	_kp.save();
//	_ki.save();
//	_kd.save();
//	_imax.save();
}