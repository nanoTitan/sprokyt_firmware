// PID.h

#pragma once

#include <stdint.h>
#include "math.h"

#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5
#define PID_COUNT 6

struct PIDInfo
{
	float P;
	float I;
	float D;
	float desired;
};

class PID
{
public:
	PID(const float& _p = 0.0f, const float& _i = 0.0f, const float& _d = 0.0f, const int16_t& _imax = 0)
	:m_kp(_p),
	m_ki(_i),
	m_kd(_d),
	m_imax(_imax),
	m_integrator(0),
	m_lastError(0),
	m_lastDerivative(NAN),
	m_lastTime(0)
	{
		m_pidInfo.P =  m_pidInfo.I = m_pidInfo.D = m_pidInfo.desired = 0;
	}

	    /// Iterate the PID, return the new control value
	    ///
	    /// Positive error produces positive output.
	    ///
	    /// @param error	The measured error value
	    /// @param scaler	An arbitrary scale factor
	    ///
	    /// @returns		The updated control output.
	    ///
	float get_pid(float error, float scaler = 1.0);

		// get_pid() constrained to +/- 4500
	int16_t get_pid_4500(float error, float scaler = 1.0);

	    /// Reset the PID integrator
	    ///
	void reset_I();

	    /// Load gain properties
	    ///
	void load_gains();

	    /// Save gain properties
	    ///
	void save_gains();

	    /// @name	parameter accessors
	    //@{

	/// Overload the function call operator to permit relatively easy initialisation
	void operator()(
		const float    p,
	    const float    i,
	    const float    d,
	    const int16_t  imaxval) 
	{
		m_kp = p; m_ki = i; m_kd = d; m_imax = imaxval;
	}

	inline float kP() const { return m_kp; }
	inline float kI() const { return m_ki;	}
	inline float kD() const { return m_kd;	}
	inline int16_t imax() const { return m_imax; }

	inline void kP(const float v) { m_kp = v; }
	inline void kI(const float v) { m_ki = v; }
	inline void kD(const float v) { m_kd = v; }
	inline void imax(const int16_t v) { m_imax = fabs(v); }
	inline float get_integrator() const { return m_integrator;	}

private:
	PIDInfo			m_pidInfo;
	float	        m_kp;
	float	        m_ki;
	float	        m_kd;
	float	        m_imax;

	float           m_integrator;///< integrator value
	float           m_lastError;///< last error for derivative
	float           m_lastDerivative;///< last derivative for low-pass filter
	uint32_t		m_lastTime;

	float           _get_pid(float error, uint16_t dt, float scaler);

	    /// Low pass filter cut frequency for derivative calculation.
	    ///
	    /// 20 Hz because anything over that is probably noise, see
	    /// http://en.wikipedia.org/wiki/Low-pass_filter.
	    ///
	static const uint8_t m_fCut = 20;
};