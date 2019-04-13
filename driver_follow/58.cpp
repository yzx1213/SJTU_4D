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
static float error, errorSum = 0;
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
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;
circle c;
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;
static float leaderspeed, leaderAcc, lastTimeLeaderSpeed = 0;
static float lastTimeDisctance = 20, last_speed = 0;
static float D_errDiff;
float acc, s, speedErr, D_err, Tmp;
float safe_distance;
int count = 0;
float state = 0;
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	c = getR(_midline[0][0], _midline[0][1], _midline[7][0], _midline[7][1], _midline[14][0], _midline[14][1]);
	count++;
	error = sqrt(25 * _Leader_X*_Leader_X + (_Leader_Y - 9.9) * (_Leader_Y - 9.9));
	errorSum += error;
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
	acc = (_speed - last_speed) / 0.02;
	last_speed = _speed;

	//D_err 为负，头车在左侧 需左转
	D_err = 1.6 * _Leader_X;//方向偏差模型
	D_errDiff = D_err - Tmp;
	Tmp = D_err;

	float D = fabs(D_err < 1.1) ? 0 : 2 * fabs(D_err) - 2.2;
	float S = leaderspeed >= 60 ? 2.9*log(leaderspeed) - 1.14 : 10.6;
	if (leaderAcc > 0)	safe_distance = constrain(10.5, 25, S + D -speedErr*0.3-0.6);
	if (leaderAcc <= 0)	safe_distance = constrain(10.5, 25, S - int(leaderAcc*0.03) + D - speedErr*0.2 - 0.5);
	if (leaderspeed > 200) safe_distance += 0.3;
	if (leaderspeed < 100) safe_distance -= 0.2;
	//*cmdSteer = (- 0.8 * D_err - 0.25 *D_errDiff) - 0.8*atan(4 * _Leader_X / _speed) + 0.016*fabs(_midline[0][0]);

	 *cmdSteer = -0.51* D_err - 1.8 * D_errDiff;
	 if(c.r<120 && _midline[0][0]<=-3) *cmdSteer +=0.2;

	if (_Leader_Y < 12) {
		*cmdAcc = 0;
		*cmdBrake = 1;
		//*cmdSteer = _yaw * 1.5;

		*cmdGear = 1;
	}

	if (speedErr > 0) {//头车速度大

		if (_Leader_Y > safe_distance) {
			*cmdAcc = (_Leader_Y - safe_distance) * 0.3 + 0.1* speedErr + leaderAcc * 0.05;
			*cmdBrake = 0;state = 1.1;

		}
		//else if (_Leader_Y > safe_distance && fabs(*cmdSteer) > 0.5) {
		//	*cmdAcc = 0.4;
		//	*cmdBrake = 0;state = 1.2;
		//}

		else if (_Leader_Y < safe_distance) {
			*cmdAcc = (_Leader_Y - safe_distance) * 0.1 + leaderAcc * 0.1;;
			*cmdBrake = 0;state = 1.3;

		}
		else state = -1;
	}


	if (speedErr < 0 && speedErr > -20) {//头车速度小，且头车跟车速度相差不大

		if (_Leader_Y < safe_distance && leaderAcc > -30 && _speed < 150) {//距离近 头车减速不明显 速度较慢
			*cmdAcc = 0;
			*cmdBrake = -0.2 * speedErr - 0.01 * leaderAcc;state = 2.1;
		}
		else if (_Leader_Y < safe_distance && leaderAcc > -30 && _speed > 150) {//距离近 头车减速不明显 速度较快
			*cmdAcc = 0;
			*cmdBrake = -2 * speedErr - 1 * leaderAcc;state = 2.2;
		}

		else if (_Leader_Y < safe_distance && leaderAcc < -30) {//距离近 头车减速明显
			*cmdAcc = 0;
			*cmdBrake = -0.5 * speedErr - 0.1 * leaderAcc;state = 2.3;
		}

		else if (_Leader_Y > safe_distance + 1 && leaderAcc < -50) {//距离很远 头车特别明显减速
			*cmdAcc = 0;
			*cmdBrake = 0.1*-speedErr - 0.1 * leaderAcc;state = 2.4;
		}

		else if (_Leader_Y > safe_distance + 1 && leaderAcc < -30) {//距离很远 头车较明显减速
			*cmdAcc = 0;
			*cmdBrake = 0.1*-speedErr - 0.001 * leaderAcc;state = 2.5;
		}


		else if (_Leader_Y > safe_distance && leaderAcc < -30) {//距离较远 头车明显减速
			*cmdAcc = 0;
			*cmdBrake = -speedErr - 0.01 * leaderAcc;state = 2.6;
		}

		else if (_Leader_Y > safe_distance && leaderAcc > -20 && fabs(D_err) < 0.4&&safe_distance > 10.6) {//距离远 头车减速不明显 且转向不明显
			*cmdAcc = 0.5 * (_Leader_Y - safe_distance);
			*cmdBrake = 0;state = 2.7;
		}
		else if (_Leader_Y > safe_distance && leaderAcc > -20 && fabs(D_err) < 0.4&&safe_distance < 10.6) {//距离远 头车减速不明显 且转向不明显
			*cmdAcc = 0.2 * (_Leader_Y - safe_distance);
			*cmdBrake = 0;state = 2.8;
		}
		else if (_Leader_Y > safe_distance && leaderAcc > -20 && fabs(D_err) > 0.4) {//距离远 头车减速不明显 但转向明显
			*cmdAcc = 0;
			*cmdBrake = 0.1;state = 2.9;
		}
		else state = -2;
	}

	if (speedErr < -20) {//头车速度小，且头车跟车速度相差巨大，即需要紧急刹车

		*cmdAcc = 0;
		*cmdBrake = 1;state = 3;
	}


	if (*cmdBrake < 0.1&&*cmdAcc == 0 && (_midline[0][0] < -6 || _midline[0][0]>6))
		*cmdBrake = 0.15;


	updateGear(cmdGear);
	//printf("speed %.3f Leader XY(%.3f, %.3f) gearbox:%d leaderspeed:%f leaderAcc:%f cmdAcc:%f brake:%f\n", _speed, _Leader_X, _Leader_Y, _gearbox, leaderspeed, leaderAcc, *cmdAcc, *cmdBrake);
	printf("speed %.3f XY(%.3f, %.3f) D_err:%.3f,count:%d steer:%.3f, l_s:%.3f l_A:%.3f cmdA:%.3f br:%.3f acc:%.3f safe:%f er:%.3f,erS:%.3f,state: %f\n", _speed, _Leader_X, _Leader_Y, D_err, count, *cmdSteer, leaderspeed, leaderAcc, *cmdAcc, *cmdBrake, acc, safe_distance, error, errorSum / count, state);
	
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
circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b*f - e * c) / (b*d - e * a);
	y = (d*c - a * f) / (b*d - e * a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}
