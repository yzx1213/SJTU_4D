/***************************************************************************

file : driver_cruise.cpp
description : user module for CyberFollow

***************************************************************************/

/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_follow.h"
double constrain(double lowerBoundary, double upperBoundary, double input);
void updateGear(int *cmdGear);
const int topGear = 6;
static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_follow";	// name of the module (short).
	modInfo[0].desc = "user module for CyberFollower";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

/*
WARNING!

DO NOT MODIFY CODES ABOVE!
*/

/*
define your variables here.
following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;
float leaderspeed, leaderAcc, lastTimeLeaderSpeed = 0;
float s, lastTimeDisctance = 20;
float speedErr, D_err, D_errDiff, Tmp;
static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	/* you can modify the print code here to show what you want */
	//printf("speed %.3f Leader XY(%.3f, %.3f) gearbox:%d fw_speed:%f brake:%f\n", _speed, _Leader_X, _Leader_Y,_gearbox,leaderspeed,*cmdBrake);
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	s = sqrt(_Leader_X * _Leader_X + _Leader_Y * _Leader_Y);
	leaderspeed = _speed + (s - lastTimeDisctance) * 180;
	lastTimeDisctance = s;
	leaderAcc = (leaderspeed - lastTimeLeaderSpeed) / 0.02;
	lastTimeLeaderSpeed = leaderspeed;
	speedErr = leaderspeed - _speed;

	D_err = 0.5 * atan2(_Leader_X, _Leader_Y) + 0.5 * _Leader_X + 0.0016*(fabs(_midline[0][0]))*(fabs(_midline[0][0]));//方向偏差模型
	D_errDiff = D_err - Tmp;
	Tmp = D_err;

	*cmdSteer = -0.8 * D_err - 0.25 *D_errDiff;


	if (_Leader_Y < 12) {
		*cmdAcc = 0;
		*cmdBrake = 1;
		//*cmdSteer = _yaw * 1.5;

		*cmdGear = 1;
	}



	if (speedErr > 0) {//头车速度大



		if (_Leader_Y > 13) {
			*cmdAcc = (_Leader_Y - 10) * 0.3 + 0.18 * speedErr + leaderAcc * 0.3;
			*cmdBrake = 0;
		}
		if (_Leader_Y < 13) {
			*cmdAcc = (_Leader_Y - 10) *0.1 + leaderAcc * 0.1;
			*cmdBrake = 0;
		}
	}

	if (speedErr < 0 && speedErr > -20) {//头车速度小，且头车跟车速度相差不大

		if (_Leader_Y < 14 && leaderAcc > -20) {//距离近 头车减速不明显
			*cmdAcc = 0;
			*cmdBrake = -0.2 * speedErr - 0.01 * leaderAcc;
		}
		else if (_Leader_Y < 14 && leaderAcc < -30) {//距离近 头车减速明显
			*cmdAcc = 0;
			*cmdBrake = -0.5 * speedErr - 0.1 * leaderAcc;
		}

		else if (_Leader_Y > 14 && leaderAcc < -30) {//距离远 头车明显减速
			*cmdAcc = 0;
			*cmdBrake = -speedErr - 0.1 * leaderAcc;
		}



		else if (_Leader_Y > 14 && leaderAcc > -20) {//距离远 头车减速不明显
			*cmdAcc = 0.1 * (_Leader_Y - 10);
			*cmdBrake = 0;
		}

	}

	if (speedErr < -20) {//头车速度小，且头车跟车速度相差巨大，即需要紧急刹车

		*cmdAcc = 0;
		*cmdBrake = 1;
	}
	


	updateGear(cmdGear);
	//printf("speed %.3f Leader XY(%.3f, %.3f) gearbox:%d leaderspeed:%f leaderAcc:%f cmdAcc:%f brake:%f\n", _speed, _Leader_X, _Leader_Y, _gearbox, leaderspeed, leaderAcc, *cmdAcc, *cmdBrake);
	printf("speed %.3f Leader XY(%.3f, %.3f) D_err:%f, steer:%f, leaderspeed:%f leaderAcc:%f cmdAcc:%f brake:%f midline:%f\n", _speed, _Leader_X, _Leader_Y, D_err, *cmdSteer, leaderspeed, leaderAcc, *cmdAcc, *cmdBrake, _midline[0][0]);

}











void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear > 1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear > 2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear > 3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear > 4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear > 5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

