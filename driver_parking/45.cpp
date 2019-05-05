/***************************************************************************

	file                 : user3.cpp
	author            : Xuangui Huang
	email              : stslxg@gmail.com
	description    :  user module for CyberParking

 ***************************************************************************/

 /*
	  WARNING !

	  DO NOT MODIFY CODES BELOW!
 */

#ifdef _WIN32
#include <windows.h>
#endif
void updateGear(int *cmdGear);
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;

#include <math.h>
#include "driver_parking.h"
#include <cmath>
circle c;
static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);
const int topGear = 6;
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);

double constrain(double lowerBoundary, double upperBoundary, double input);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_parking";	// name of the module (short).
	modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
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
	printf("OK!\n");
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
static float line[11][2];
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
float s, last_s = 100, s_diff, y, last_y, y_diff;
float last_tmp = 0, tmp, tmp_diff;
int state;
float pos_distance = 7.8;
float start_p;
float a = 8.35 + 0.118 - 0.481;
float k1,k2;
int flagout = 0;


static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	s = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));
	tmp = fabs(fabs(_caryaw - _lotAngle) - PI);
	tmp_diff = tmp - last_tmp;
	last_tmp = tmp;

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
	//printf("lotX %.6f  lotY %.6f", _lotX, _lotY);
}


float sign_y;
static int flag = 1;
static float k, b, dist;
static int flagt = 0;
float dtmp = 0,D_errD;

float offset = _width / 3;
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {

	if (abs(_lotAngle) > (PI / 2 - 0.0005) && abs(_lotAngle) < (PI / 2 + 0.0005))                       //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
		dist = (_carX - _lotX);
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = (k*_carX - _carY + b) / sqrt(k*k + 1);
	}
	c = getR(_midline[0][0], _midline[0][1], _midline[2][0], _midline[2][1], _midline[4][0], _midline[4][1]);
	float y = sqrt(s*s - dist * dist);
	y_diff = y - last_y;
	last_y = y;

	float phi = _lotAngle - _caryaw - asin(-dist / s);
	if (_lotAngle < 0) _lotAngle += 2 * PI;
	if (_caryaw < 0) _caryaw += 2 * PI;
	float theta = (_lotAngle - _caryaw);
	float rel_x = s * sin(phi);
	float rel_y = -s * cos(phi);

	/*for (int i = 0; i < 11;i++) {
		line[i][0] = -dist * cos(PI + _lotAngle - _caryaw) + i * 1 * sin(PI + _lotAngle - _caryaw);
		line[i][1] = -dist * sin(PI + _lotAngle - _caryaw) - i * 1 * cos(PI + _lotAngle - _caryaw);
	}
	*/
	for (int i = 0; i < 11; i++) {
		line[i][0] = -dist * cos(theta) + 0.2 * i * sin(theta);
		line[i][1] = -dist * sin(theta) - 0.2 * i * cos(theta);
	}
	line[11][0] = -dist * cos(theta) + y * sin(theta);
	line[11][1] = -dist * sin(theta) - y * cos(theta);
	float D_err = -5.5 * theta - 1.4* dist;
	D_errD = D_err - dtmp;
	dtmp = D_err;

	//if (_lotX<169 && _lotX > 168) start_p = 7.53;
	//else if (_lotX < 150 && _lotX > 149) start_p = 7.515;
	//else if (_lotX < 32 && _lotX > 31) start_p = 8.723;
	//else if (_lotX < 30 && _lotX > 29) start_p = 8.268;
	//else if (_lotX < 45 && _lotX > 44) start_p = 8.127;


	float rel_X, rel_Y, transx, transy, temp;

	rel_X = _carX * cos(_lotAngle) + _carY * sin(_lotAngle);
	rel_Y = _carY * cos(_lotAngle) - _carX * sin(_lotAngle);
	transx = _lotX * cos(_lotAngle) + _lotY * sin(_lotAngle);
	transy = _lotY * cos(_lotAngle) - _lotX * sin(_lotAngle);
	rel_X -= transx;
	rel_Y -= transy;
	temp = rel_X;
	rel_X = -rel_Y;
	rel_Y = temp;




	s_diff = s - last_s;
	last_s = s;








	//if (!*bFinished) {
	//	if (s < 1 && _speed < 0.16) *bFinished = 1;

	//	if (s > 70 || fabs(dist) > 70) {
	//		*cmdAcc = 1;//油门给100%
	//		*cmdBrake = 0;//无刹车
	//		*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
	//		*cmdGear = 1;//档位始终挂1
	//		state = 1;
	//	}
	//	else if ((fabs(dist) < start_p && fabs(fabs(_yaw) - PI / 2) < 0.05 && s_diff > -0.075 && y<0.02) || state == 5) {
	//		*cmdSteer = 0;
	//		*cmdBrake = 1;*cmdAcc = 0;

	//		state = 5;
	//	}
	//	else if (fabs(dist) < start_p  && fabs(tmp)<0.07&&tmp_diff>=-0.08 && y >= 0.02 && state!=2) {
	//		*cmdSteer = 0;
	//		if (_speed > 10 * y) { *cmdBrake = 0.1 + (_speed - 4 * y)*0.05;*cmdAcc = 0; *cmdSteer = 1.628 * D_err; }
	//		else { *cmdBrake = 0;*cmdAcc = 0.2;*cmdSteer = 1.628 * D_err;
	//		}
	//		state = 4;
	//	}

	//	else if (fabs(dist) < start_p&& fabs(fabs(_yaw) - PI / 2) > 0.05 &&state!=4) {
	//		*cmdSteer = 1;
	//		if (_speed > 25) { *cmdBrake = 0.1;*cmdAcc = 0; }
	//		else { *cmdBrake = 0;*cmdAcc = 0.2; }
	//		state = 3;
	//	}

	//	else if (s < 80 && s > 10&&fabs(dist) < 80) {
	//		*cmdSteer = (_yaw - atan2(_midline[10][0] + offset , _midline[10][1])) / 3.14;
	//		if (_speed > 30) { *cmdBrake = 0.1;*cmdAcc = 0; }
	//		else { *cmdBrake = 0;*cmdAcc = 0.2; }
	//		state = 2;
	//		pos_distance = y;
	//		start_p = 1.9*a*sin(atan(a/(c.r-5))) + a;
	//		offset = a + sqrt(a*a + (c.r - 5)*(c.r - 5)) - c.r;
	//	}
	//}

	if (!*bFinished) {
		if (s < 1 && fabs(_speed) < 0.1) *bFinished = 1;//除了速度，可辅助以精度的判断

		if (s > 70 || fabs(dist) > 70) {//巡线段
			*cmdAcc = 1;//油门给100%
			*cmdBrake = 0;//无刹车
			*cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;//设定舵机方向
			*cmdGear = 1;//档位始终挂1
			state = 1;
		}

		else if (s < 70 && fabs(dist) < 70 && fabs(dist) > 18.3 - 140 / c.r) {//靠左侧行驶巡线
			*cmdSteer = (_yaw - atan2(_midline[10][0] -2.5, _midline[10][1])) / 3.14;//中线左边3米处
			updateGear(cmdGear);
			if (_speed > 80) { *cmdBrake = 0.2; *cmdAcc = 0; }
			else { *cmdBrake = 0; *cmdAcc = 0.8; }//控制速度为100，高速以便飘逸
			state = 2;
		}

		else if (fabs(dist) < 18.3 - 140 / c.r && fabs(fabs(_yaw) - PI / 2) >= 1.33 - 7 / c.r && state != 4) {//开始飘逸
			*cmdGear = -1;
			*cmdAcc = 1;
			*cmdBrake = 0;
			*cmdSteer = -1;
			state = 3;
		}

		else if (_speed < 2) {
			*cmdGear = -1;
			if (fabs(_speed) > 15 && rel_Y> 0.6) { *cmdAcc = 0; *cmdBrake = 0.2; }
			else { *cmdAcc = 0.4; *cmdBrake = 0; }
			
			if (rel_Y< 0.6 ){ *cmdAcc = 0; *cmdBrake = 1; }
			
			*cmdSteer = 2.5*D_err+5*D_errD;
			state = 5;
		}

		else if (fabs(dist) < 18.3 - 140 / c.r && fabs(fabs(_yaw) - PI / 2) < 1.33 - 7 / c.r) {//当角度到达一定值时往回拉车头
			*cmdGear = -1;
			*cmdAcc = 0.5;
			*cmdBrake = 1;
			*cmdSteer = 0;
			state = 4;
		}





	}














	if (*bFinished)
	{
		if (flagout == 0)
		{
			*cmdGear = 1; *cmdAcc = 1; *cmdSteer = 1; *cmdBrake = 0;
			if (abs(_yaw) < 0.5)  flagout = 1;
		}
		else if (flagout == 1)
		{
			*cmdGear = 1; *cmdAcc = 0.5; *cmdSteer = -1; *cmdBrake = 0;
			if (fabs(_yawrate) < 0.1)  flagout = 2;
		}
		else if (flagout == 2)
		{
			*cmdGear = 1; *cmdAcc = 1; *cmdSteer = (_yaw - 8 * atan2(_midline[30][0], _midline[30][1])) / 3.14;
			*cmdBrake = 0;
		}

	}

	printf("line[11][1]%f,y%f,theta%f,dist%f\n", line[11][1],y,theta,dist);
	/*printf("theta:%f\n", theta);
	printf("line0(%f,%f)\n", line[0][0], line[0][1]);
	printf("line1(%f,%f)\n", line[1][0], line[1][1]);
	printf("line2(%f,%f)\n", line[2][0], line[2][1]);
	printf("line3(%f,%f)\n", line[3][0], line[3][1]);
	printf("D_err:%f 1:%f, 2:%f, 3:%f\n", D_err, theta, 5 * atan2(line[5][0], line[5][1]), 2.0* atan(20 * line[3][0] / _speed));
	printf("dist%f,y%f\n", dist, y);
	printf("lotangle:%f,caryaw:%f,dist:%f,s:%f,asin(-dist/s):%f\n", _lotAngle, _caryaw, dist, asin(-dist / s));
	printf("rel_x,rel_y(%f,%f)\n", rel_x, rel_y);
	printf("car(%.2f,%.2f),lot(%.2f,%.2f), lotangle:%f\n", _carX, _carY, _lotX, _lotY, _lotAngle);
	printf("tmp:%f tmp_diff %f\n", tmp, tmp_diff);
	printf("finished:%d, yaw:%f, c.r:%f,  start_p%.2f,  offset:%f\n", *bFinished, _yaw, c.r, start_p, offset);
	printf("speed:%f,  state:%d,   Acc:%.2f,  Brake:%.2f,   steer:%.2f,  gear:%d, l_agl:%f,c_agl:%f,  dist:%.3f,  s:%.3f,  y:%.2f\n\n", _speed, state, *cmdAcc, *cmdBrake, *cmdSteer, *cmdGear, _lotAngle, _caryaw, dist, s, y);*/
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
	x = constrain(-1000000.0, 1000000.0, x);
	y = constrain(-1000000.0, 1000000.0, y);
	r = constrain(1.0, 1000000, r);
	int sign = (x > 0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}
