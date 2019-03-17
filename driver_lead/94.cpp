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
#include <cmath> 
#endif

#include "driver_lead.h"

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_lead(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_lead";	// name of the module (short).
	modInfo[0].desc    =  "leader module for CyberFollow" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tLeaderItf *itf = (tLeaderItf *)pt;
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
static float _yaw, _yawrate, _speed, _acc, _width, _rpm,  _DistanceFromStart;
static int _gearbox, _laps;
static int _counter=0;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm,  float DistanceFromStart, int laps){
	/* write your own code here */
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	_DistanceFromStart = DistanceFromStart;
	_laps = laps;

	printf("speed %f DFS %f lap %d 10m far target(%f, %f)\n", _speed, _DistanceFromStart, _laps, _midline[10][0], _midline[10][1]);
	
}


static float ki,k; 
static int counter=0;//定义变量

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	/* write your own code here */
	/*******************************路径设计********************************/
	/*switch(_laps)
	{
	    case 1: ki = 0;break;           //第一圈正常行驶
	    case 2: ki += 0.01 * PI; break; //第二圈曲线行驶
		case 3: ki = 0;//第三圈的时候进行一定的保护
	    default: break;
	}
	k = sin(ki)/2.0;
	*/
	switch (_laps)
	{
	case 1: ki = 0;k = 0;break;           //第一圈正常行驶
	case 2: ki += 0.01; counter++ ;break; //第二圈曲线行驶
	case 3: ki = 0;//第三圈的时候进行一定的保护
	default: break;
	}
	
	
	if (ki == 0)
	{
		k = 0;
	}
	else
	{
		if(counter<=2380) k = (abs(ki - counter / 100) - 1) ;
		else k = (abs(ki - counter / 100))-0.9;
	}
		
	/*******************************车辆控制********************************/
	
	//油门给80%
	/*if ((counter%300)<200)
	{
		*cmdAcc = 1;
		*cmdBrake = 0;
	}
	else
	{
		*cmdAcc = 0;
		*cmdBrake = 1;
	}*/
	*cmdAcc = 1;
	*cmdBrake=0;
	*cmdSteer = (_yaw -8*atan2( _midline[30][0]+k*_width ,_midline[30][1]))/3.14 ;//设定舵机方向
	*cmdGear = 1;//档位始终挂1
	printf("lap %d counter %d\n",_laps, counter);
	
}
