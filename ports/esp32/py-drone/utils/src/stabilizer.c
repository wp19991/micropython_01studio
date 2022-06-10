// #include "system.h"
#include "stabilizer.h"
#include "sensors.h"
#include "sensfusion6.h"
#include "commander.h"
#include "anomal_detec.h"
#include "state_control.h"
#include "state_estimator.h"
#include "power_distribution.h"
#include "position_pid.h"
#include "flip.h"
// #include "optical_flow.h"
// #include "vl53lxx.h"
#include "maths.h"
#include "ledseq.h"
#include "sensfusion6.h"

#include "sensors_mpu6050_spl06.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool isInit = false;

static setpoint_t 	setpoint;	/*设置目标状态*/
static sensorData_t sensorData;	/*传感器数据*/
static state_t 		state;		/*四轴姿态*/
static control_t 	control;	/*四轴控制参数*/

static uint16_t velModeTimes = 0;		/*速率模式次数*/
static uint16_t absModeTimes = 0;		/*绝对值模式次数*/
static float setHeight = 0.f;		/*设定目标高度 单位cm*/
static float baroLast = 0.f;
static float baroVelLpf = 0.f;

void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit) return;
	stateControlInit();		/*姿态PID初始化*/
	powerDistributionInit();		/*电机初始化*/
	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);	
	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= stateControlTest();
	pass &= powerDistributionTest();

	return pass;
}

/*设置快速调整参数*/	
void setFastAdjustPosParam(uint16_t velTimes, uint16_t absTimes, float height)
{
	if(velTimes != 0 && velModeTimes == 0)
	{
		baroLast = sensorData.baro.asl;
		baroVelLpf = 0.f;

		velModeTimes = velTimes;
	}
	if(absTimes != 0 && absModeTimes ==0)
	{
		setHeight = height;
		absModeTimes = absTimes;
	}		
}

/*快速调整高度*/
static void fastAdjustPosZ(void)
{	
	if(velModeTimes > 0)
	{
		velModeTimes--;
		estRstHeight();	/*复位估测高度*/
		
		float baroVel = (sensorData.baro.asl - baroLast) / 0.004f;	/*250Hz*/
		baroLast = sensorData.baro.asl;
		baroVelLpf += (baroVel - baroVelLpf) * 0.35f;

		setpoint.mode.z = modeVelocity;
		state.velocity.z = baroVelLpf;		/*气压计融合*/
		setpoint.velocity.z = -1.0f * baroVelLpf;
		
		if(velModeTimes == 0)
		{
			//if(getModuleID() == OPTICAL_FLOW)
				//setHeight = getFusedHeight();
			//else
				setHeight = state.position.z;
		}		
	}
	else if(absModeTimes > 0)
	{
		absModeTimes--;
		estRstAll();	/*复位估测*/
		setpoint.mode.z = modeAbs;		
		setpoint.position.z = setHeight;
	}	
}

void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime = xTaskGetTickCount();//getSysTickCnt();
	
	ledseqRun(SYS_LED, seq_alive);

	while(!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
	}

	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, 1);//MAIN_LOOP_DT);		/*1ms周期延时*/

		//获取6轴和气压数据（500Hz）
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			sensorsAcquire(&sensorData, tick);				/*获取6轴和气压数据*/
		}

		//四元数和欧拉角计算（250Hz）
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
			//printf("state->:roll:%f,pitch:%f,yaw:%f,%d\r\n",state.attitude.roll,state.attitude.pitch,state.attitude.yaw,state.attitude.timestamp);
		}

		//位置预估计算（250Hz）
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{
			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
		}

		//目标姿态和飞行模式设定（100Hz）	
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated()==true)
		{
			commanderGetSetpoint(&setpoint, &state);	/*目标数据和飞行模式设定*/	
		}

		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
		{
			fastAdjustPosZ();	/*快速调整高度*/
		}		
#if 0
		/*读取光流数据(100Hz)*/
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			getOpFlowData(&state, 0.01f);	
		}
#endif
		/*翻滚检测(500Hz) 非定点模式*/
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick) && (getCommanderCtrlMode() != 0x03))
		{
			flyerFlipCheck(&setpoint, &control, &state);	
		}

		/*异常检测*/
		anomalDetec(&sensorData, &state, &control);			
		
		/*PID控制*/	

		stateControl(&control, &sensorData, &state, &setpoint, tick);

		//控制电机输出（500Hz）
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			powerDistribution(&control);
		}
		tick++;
	}
}

void getAttitudeData(attitude_t* get)
{
	get->pitch = state.attitude.pitch;
	get->roll = state.attitude.roll;
	get->yaw = state.attitude.yaw;
}
void getControlledQuantiy(control_t* get)
{
	get->roll = control.roll;
	get->pitch = control.pitch;
	get->yaw = control.yaw;
	get->thrust = control.thrust;
	get->flipDir = control.flipDir;
}
bool getLockStatus(void)
{
	return state.isRCLocked;
}

float getBaroData(void)
{
	return sensorData.baro.asl;
}

float getBaroTemp(void)
{
	return sensorData.baro.temperature;
}

void getSensorData(sensorData_t* get)
{
	*get = sensorData;
}

void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos)
{
	acc->x = 1.0f * state.acc.x;
	acc->y = 1.0f * state.acc.y;
	acc->z = 1.0f * state.acc.z;
	vel->x = 1.0f * state.velocity.x;
	vel->y = 1.0f * state.velocity.y;
	vel->z = 1.0f * state.velocity.z;
	pos->x = 1.0f * state.position.x;
	pos->y = 1.0f * state.position.y;
	pos->z = 1.0f * state.position.z;
}


