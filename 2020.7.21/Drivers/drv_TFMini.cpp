#include "drv_Uart7.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

#define SensorInd 2

//maxi：结合TFmini的datesheet来看就很多明白了！！！！！！

typedef struct
{
	int16_t Dist;	//Dist距离（30-1200cm）
	int16_t Strength;	//Strength信号强度（20-2000可信）
	uint8_t Mode;	//Mode测距档位
	uint8_t Rsv;	//预留
}__PACKED _TfMini;
static const unsigned char packet_ID[2] = { 0x59 , 0x59 };


//maxi:下面这个仔细读读还是可以看懂的。
static void TFMini_Server(void* pvParameters)
{
	/*状态机*/
	  _TfMini  SensorD;  //maxi:SensorD就是在这里被定义的嘛，就是上面那个结构体
		unsigned char rc_counter = 0;
		unsigned char sum = 0;
	/*状态机*/
	
	while(1)
	{
		uint8_t rdata;
		if( Read_Uart7( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter == 0 )
				sum = 0;
			if( rc_counter < 2 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					sum += rdata;
					++rc_counter;
				}
			}
			else if( rc_counter < 8 )
			{	//接收数据
				( (unsigned char*)&SensorD )[ rc_counter - 2 ] = rdata;   //maxi:应该是这里获取数据
				sum += (unsigned char)rdata;
				++rc_counter;
			}
			else
			{	//校验
				if( sum == rdata )
				{	//校验成功
					if( SensorD.Strength>20 && SensorD.Strength<3000 && SensorD.Dist>30 && SensorD.Dist<1200 )  //maxi:这里是判断信号强度和高度是不是在区间范围内。
					{
						vector3<double> position;
						position.z = SensorD.Dist;   //maxi:SensorD也是最上面定义的激光测高的结构体
						//获取倾角
						Quaternion quat;
						get_Airframe_quat( &quat );
						double lean_cosin = quat.get_lean_angle_cosin();
						//更新
						position.z *= lean_cosin;
						PositionSensorUpdatePosition( SensorInd, position, true );     //maxi:传感器更新函数
					}
					else
						PositionSensorSetInavailable( SensorInd );
				}
				rc_counter = 0;
			}
		}
	}
}

void init_drv_TFMini()
{
	//波特率115200
	SetBaudRate_Uart7( 115200, 2, 2 );
	//注册传感器
	bool res = PositionSensorRegister( SensorInd , \
																			Position_Sensor_Type_RangePositioning , \
																			Position_Sensor_DataType_s_z , \
																			Position_Sensor_frame_ENU , \
																			0.05 , //延时
																			0 ,	//xy信任度
																			0 //z信任度
																			);
	if(res)
		xTaskCreate( TFMini_Server, "TFMini", 1024, NULL, SysPriority_ExtSensor, NULL);  //maxi：任务创建
}