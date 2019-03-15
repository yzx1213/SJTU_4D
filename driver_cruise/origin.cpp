/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_cruise";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberCruise" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
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

//**********Global variables for vehicle states*********//
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
//******************************************************//

//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //
// Direction Control Variables						         //
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum=0;//sum of direction error(Integration)      //
// Speed Control Variables								     //
circle c;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum=0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta=20;												 //
//***********************************************************//

//*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag=true;											//
double offset=0;										//
double Tmp = 0;
//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int *cmdGear);													//
// Function constrain:															//
//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary,double input);		//
// Function getR:																//
//		Given three points ahead, outputs a struct circle.						//
//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);		//
//******************************************************************************//

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	if(parameterSet==false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		// Speed Control
		/*
		You can modify the limited speed in this module
		Enjoy  -_-  
		*/
		startPoint = _speed * 0.445;
		c = getR(_midline[startPoint][0],_midline[startPoint][1],_midline[startPoint+delta][0],_midline[startPoint+delta][1],_midline[startPoint+2*delta][0],_midline[startPoint+2*delta][1]);
		if (c.r<=60)
		{
			expectedSpeed = constrain(45,200,c.r*c.r*(-0.046)+c.r*5.3-59.66);
		}
		else
		{
			expectedSpeed = constrain(100,200,c.r*1.4);
		}
		curSpeedErr = expectedSpeed - _speed;
		speedErrSum = 0.1 * speedErrSum + curSpeedErr;
		if (curSpeedErr > 0)
		{
			
			if (abs(*cmdSteer)<0.6)
			{
				*cmdAcc = constrain(0.0,1.0,kp_s * curSpeedErr + ki_s * speedErrSum + offset);
				*cmdBrake = 0;
			}
			else if (abs(*cmdSteer)>0.70)
			{
				*cmdAcc = 0.005 + offset;
				*cmdBrake = 0;
			}
			else
			{
				*cmdAcc = 0.11 + offset;
				*cmdBrake = 0;
			}
		
		}
		else if (curSpeedErr < 0)
		{
			*cmdBrake = constrain(0.0,0.8,-kp_s *curSpeedErr/5 - offset/3);
			*cmdAcc = 0;
		}

		updateGear(cmdGear);
		
		/******************************************Modified by Yuan Wei********************************************/
		/*
		Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
		Once you have chose the error model , you can rectify the value of PID to improve your control performance.
		Enjoy  -_-  
		*/
		// Direction Control		
		//set the param of PID controller
        kp_d = 1;
        ki_d = 0;
		kd_d = 0;

		//get the error 
		D_err = -atan2(_midline[5][0],_midline[5][1]);//only track the aiming point on the middle line

		//the differential and integral operation 
		D_errDiff = D_err - Tmp;
		D_errSum = D_errSum + D_err;
		Tmp = D_err;

		//set the error and get the cmdSteer
		*cmdSteer =constrain(-1.0,1.0,kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);

		//print some useful info on the terminal
		printf("D_err : %f \n", D_err);
		printf("cmdSteer %f \n", *cmdSteer);	
		/******************************************End by Yuan Wei********************************************/
	}
}

void PIDParamSetter()
{
	
		kp_s=0.02;
		ki_s=0;
		kd_s=0;
		kp_d=1.35;
		ki_d=0.151;
		kd_d=0.10;
		parameterSet = true;
	
}

void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear >1)
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
		else if (_speed >=105 && topGear >2)
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
		else if (_speed >= 145 && topGear >3)
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
		else if (_speed >= 187 && topGear >4)
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
		else if (_speed >= 234 && topGear >5)
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

double constrain(double lowerBoundary, double upperBoundary,double input)
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
	double a,b,c,d,e,f;
    double r,x,y;
	
	a=2*(x2-x1);
    b=2*(y2-y1);
    c=x2*x2+y2*y2-x1*x1-y1*y1;
    d=2*(x3-x2);
    e=2*(y3-y2);
    f=x3*x3+y3*y3-x2*x2-y2*y2;
    x=(b*f-e*c)/(b*d-e*a);
    y=(d*c-a*f)/(b*d-e*a);
    r=sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
	x=constrain(-1000.0,1000.0,x);
	y=constrain(-1000.0,1000.0,y);
	r=constrain(1.0,500.0,r);
	int sign = (x>0)?1:-1;
	circle tmp = {r,sign};
	return tmp;
}

