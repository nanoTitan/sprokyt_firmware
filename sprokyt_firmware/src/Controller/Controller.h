#pragma once

#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5
#define PID_COUNT 6

class Controller
{
public:
	Controller() {}
	virtual ~Controller() {}
	
	virtual void Update(float yaw, float pitch, float roll) = 0;
	
private:
	
};