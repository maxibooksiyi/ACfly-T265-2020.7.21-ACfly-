#include "StorageSystem.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "fatfs.h"
#include <stdio.h>

#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "Sensors.hpp"
#include "semphr.h"
#include "stream_buffer.h"
#include "message_buffer.h"

/*SD Log*/
	//Log������
	#define BufSize 10240
	Static_DTCMBuf uint8_t LogStreamBufferStorage[ BufSize+1 ];
	Static_DTCMBuf StaticMessageBuffer_t LogStreamBufferStruct;
	Static_DTCMBuf MessageBufferHandle_t LogStreamBuffer = xMessageBufferCreateStatic( BufSize,
                                                 LogStreamBufferStorage,
                                                 &LogStreamBufferStruct );;
	static SemaphoreHandle_t LogSemphr = xSemaphoreCreateMutex();
	static bool Lock_SDLog( double TIMEOUT = -1 )
	{
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake( LogSemphr , TIMEOUT_Ticks ) )
			return true;
		return false;
	}
	static void UnLock_SDLog()
	{
		xSemaphoreGive(LogSemphr);
	}
	
	/*Log����*/
		bool SDLog_Msg_PosSensor( uint8_t ind, Position_Sensor sensor, double SyncTIMEOUT )
		{
			struct MsgS
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//ΪPosition_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//��������ţ�bit7Ϊ�Ƿ���ã�
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[6];
				double posx;
				double posy;
				double posz;
			}__attribute__((__packed__));
			struct MsgV
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//ΪPosition_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;	
				uint8_t sensor;	//��������ţ�bit7Ϊ�Ƿ���ã�
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[6];				
				double velx;
				double vely;
				double velz;
			}__attribute__((__packed__));
			struct MsgSV
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//ΪPosition_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//��������ţ�bit7Ϊ�Ƿ���ã�
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[6];				
				double posx;
				double posy;
				double posz;
				double velx;
				double vely;
				double velz;
			}__attribute__((__packed__));
			struct MsgGS
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//ΪPosition_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//��������ţ�bit7Ϊ�Ƿ���ã�
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[6];
				double OriginLat;
				double OriginLon;
				double Lat;
				double Lon;
				double posx;
				double posy;
				double posz;
			}__attribute__((__packed__));
			struct MsgGSV
			{
				uint8_t msg_type;
				uint8_t length;
				uint8_t SensorType;	//ΪPosition_Sensor_Type
				uint8_t DataType;	//Position_Sensor_DataType
				uint32_t Time;
				uint8_t sensor;	//��������ţ�bit7Ϊ�Ƿ���ã�
				uint8_t frame;	//Position_Sensor_frame
				uint8_t rsv1[6];
				double OriginLat;
				double OriginLon;
				double Lat;
				double Lon;
				double posx;
				double posy;
				double posz;
				double velx;
				double vely;
				double velz;
			}__attribute__((__packed__));
			
			if( sensor.sensor_DataType < 8 )
			{	//s������
				if( sensor.sensor_type == Position_Sensor_Type_GlobalPositioning )
				{	//GS
					MsgGS msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.sensor_type;
					msg.DataType = sensor.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.available ? (1<<7)|ind : ind;
					msg.frame = sensor.velocity_data_frame;
					msg.OriginLat = rad2degree(sensor.mp.lat0_rad);
					msg.OriginLon = rad2degree(sensor.mp.lon0_rad);
					msg.Lat = sensor.position_Global.x;
					msg.Lon = sensor.position_Global.y;
					msg.posx = sensor.position.y*0.01;
					msg.posy = sensor.position.x*0.01;
					msg.posz = -sensor.position.z*0.01;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
				else
				{	//S
					MsgS msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.sensor_type;
					msg.DataType = sensor.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.available ? (1<<7)|ind : ind;
					msg.frame = sensor.velocity_data_frame;
					msg.posx = sensor.position.y*0.01;
					msg.posy = sensor.position.x*0.01;
					msg.posz = -sensor.position.z*0.01;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
			}
			else if( sensor.sensor_DataType < 16 )
			{	//v������
				MsgV msg;
				msg.msg_type = LogMsg_PosSensor;
				msg.length = sizeof(msg);
				msg.SensorType = sensor.sensor_type;
				msg.DataType = sensor.sensor_DataType;
				msg.Time = TIME::get_System_Run_Time()*1e+4;
				msg.sensor = sensor.available ? (1<<7)|ind : ind;
				msg.frame = sensor.velocity_data_frame;
				msg.velx = sensor.velocity.y*0.01;
				msg.vely = sensor.velocity.x*0.01;
				msg.velz = -sensor.velocity.z*0.01;
				if( Lock_SDLog(SyncTIMEOUT) )
				{
					xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
					
					UnLock_SDLog();
					return true;
				}
			}
			if( sensor.sensor_DataType < 24 )
			{	//sv������
				if( sensor.sensor_type == Position_Sensor_Type_GlobalPositioning )
				{	//GSV
					MsgGSV msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.sensor_type;
					msg.DataType = sensor.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.available ? (1<<7)|ind : ind;
					msg.frame = sensor.velocity_data_frame;
					msg.OriginLat = rad2degree(sensor.mp.lat0_rad);
					msg.OriginLon = rad2degree(sensor.mp.lon0_rad);
					msg.Lat = sensor.position_Global.x;
					msg.Lon = sensor.position_Global.y;
					msg.posx = sensor.position.y*0.01;
					msg.posy = sensor.position.x*0.01;
					msg.posz = -sensor.position.z*0.01;
					msg.velx = sensor.velocity.y*0.01;
					msg.vely = sensor.velocity.x*0.01;
					msg.velz = -sensor.velocity.z*0.01;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
				else
				{	//S
					MsgSV msg;
					msg.msg_type = LogMsg_PosSensor;
					msg.length = sizeof(msg);
					msg.SensorType = sensor.sensor_type;
					msg.DataType = sensor.sensor_DataType;
					msg.Time = TIME::get_System_Run_Time()*1e+4;
					msg.sensor = sensor.available ? (1<<7)|ind : ind;
					msg.frame = sensor.velocity_data_frame;
					msg.posx = sensor.position.y*0.01;
					msg.posy = sensor.position.x*0.01;
					msg.posz = -sensor.position.z*0.01;
					msg.velx = sensor.velocity.y*0.01;
					msg.vely = sensor.velocity.x*0.01;
					msg.velz = -sensor.velocity.z*0.01;
					if( Lock_SDLog(SyncTIMEOUT) )
					{
						xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
						
						UnLock_SDLog();
						return true;
					}
				}
			}
			
			return false;
		}
		bool SDLog_Msg_Attitude( double SyncTIMEOUT )
		{
			Quaternion airframe_quat;
			if( get_AirframeY_quat(&airframe_quat,SyncTIMEOUT) == false )
				return false;
			vector3<double> angular_rate;
			if( get_AngularRate_Ctrl(&angular_rate,SyncTIMEOUT) == false )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				uint16_t rsv;
				uint32_t Time;			
				double roll;
				double pitch;
				double yaw;
				double rollspeed;
				double pitchspeed;
				double yawspeed;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_Attitude;
			msg.length = sizeof(msg);
			msg.Time = TIME::get_System_Run_Time()*1e+4;			
			msg.roll = airframe_quat.getRoll();
			msg.pitch = airframe_quat.getPitch();
			msg.yaw = airframe_quat.getYaw();
			msg.rollspeed = angular_rate.x;
			msg.pitchspeed = angular_rate.y;
			msg.yawspeed = angular_rate.z;
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_AttitudeQuaternion( double SyncTIMEOUT )
		{
			Quaternion airframe_quat;
			if( get_AirframeY_quat(&airframe_quat,SyncTIMEOUT) == false )
				return false;
			vector3<double> angular_rate;
			if( get_AngularRate_Ctrl(&angular_rate,SyncTIMEOUT) == false )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				uint16_t rsv;
				uint32_t Time;
				double q0;
				double q1;
				double q2;
				double q3;
				double rollspeed;
				double pitchspeed;
				double yawspeed;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_AttitudeQuaternion;
			msg.length = sizeof(msg);
			msg.Time = TIME::get_System_Run_Time()*1e+4;			
			msg.q0 = airframe_quat.get_qw();
			msg.q1 = airframe_quat.get_qx();
			msg.q2 = airframe_quat.get_qy();
			msg.q3 = airframe_quat.get_qz();
			msg.rollspeed = angular_rate.x;
			msg.pitchspeed = angular_rate.y;
			msg.yawspeed = angular_rate.z;
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_LocalPositionNed( double SyncTIMEOUT )
		{
			vector3<double> pos;
			vector3<double> vel;
			vector3<double> acc;
			if( get_Position( &pos, SyncTIMEOUT ) == false )
				return false;
			if( get_VelocityENU( &vel, SyncTIMEOUT ) == false )
				return false;
			if( get_AccelerationENU_Ctrl( &acc, SyncTIMEOUT ) == false )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				int8_t XYSensor;
				int8_t ZSensor;
				uint32_t Time;			
				double posx;
				double posy;
				double posz;
				double xspeed;
				double yspeed;
				double zspeed;
				double accx;
				double accy;
				double accz;
			}__attribute__((__packed__));
			Msg msg;
			
			msg.msg_type = LogMsg_LocalPositionNed;
			msg.length = sizeof(msg);
			msg.XYSensor = get_Current_XYSensor();
			msg.ZSensor = get_Current_ZSensor();
			msg.Time = TIME::get_System_Run_Time()*1e+4;		
			msg.posx = pos.y * 0.01;
			msg.posy = pos.x * 0.01;
			msg.posz = -pos.z * 0.01;
			msg.xspeed = vel.y * 0.01;
			msg.yspeed = vel.x * 0.01;
			msg.zspeed = -vel.z * 0.01;
			msg.accx = acc.y * 0.01;
			msg.accy = acc.x * 0.01;
			msg.accz = -acc.z * 0.01;
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, &msg, sizeof(msg), 0 );
				
				UnLock_SDLog();
				return true;
			}
			return false;
		}
		bool SDLog_Msg_DebugVect( const char* name, double vect[], uint8_t length, double SyncTIMEOUT )
		{
			if( length > 20 )
				return false;
			struct Msg
			{
				uint8_t msg_type;
				uint8_t length;
				uint16_t rsv;
				uint32_t Time;
				char name[10];
			}__attribute__((__packed__));

			uint8_t* buf = new uint8_t[sizeof(Msg)+length*8];
			Msg* msg = (Msg*)buf;
			msg->msg_type = LogMsg_DebugVect;
			msg->length = sizeof(Msg)+length*8;
			msg->Time = TIME::get_System_Run_Time()*1e+4;
			for(uint8_t i = 0; i < 10 ; ++i )
			{
				msg->name[i] = name[i];
				if( name[i] == 0 )
					break;
			}
			memcpy( &buf[sizeof(Msg)], vect, length*8 );
			
			if( Lock_SDLog(SyncTIMEOUT) )
			{
				xMessageBufferSend( LogStreamBuffer, buf, msg->length, 0 );
								
				UnLock_SDLog();
				delete[] buf;
				return true;
			}
			delete[] buf;
			return false;
		}
	/*Log����*/
/*SD Log*/

static void SDS_Task(void* pvParameters)
{
reload_SD:
	//�ȴ�SD�����뿨��
	do
		os_delay(2.0);
	while( BSP_SD_IsDetected() == false );
		
	//����SD��
	if( BSP_SD_Init() != MSD_OK )
		goto reload_SD;
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) == FR_OK)
	{
		//�ļ�ָ��
		static FIL SDFile;
		//�ļ�����״̬
		FRESULT fres;
		//�ļ����У�״̬
		FILINFO finfo;
		//�ļ����У���
		char filename[50];
		uint8_t filename_len;
		//д�ļ���ʱ����
		__attribute__ ((aligned (4))) Static_AXIDMABuf char buf[BufSize];
		UINT length; 
		
		//����ACFlyĿ¼
		filename[0] = 0;
		strcat( filename, SDPath );
		strcat( filename, "ACFly" );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//ɾ��ͬ���ļ�
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);		
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto reload_SD;
		
		//����logĿ¼
		filename[0] = 0;
		strcat( filename, SDPath );
		strcat( filename, "ACFly/Log" );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//ɾ��ͬ���ļ�
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);		
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto reload_SD;
		
		//����log����Ŀ¼
		RTC_TimeStruct RTC_Time;
		RTC_Time = Get_RTC_Time();
		filename_len = strlen(filename);
		sprintf( &filename[filename_len], "/%d%02d%02d" , RTC_Time.Year, RTC_Time.Month, RTC_Time.Date );
		fres = f_stat( filename, &finfo );
		if( fres!=FR_NO_FILE && (finfo.fattrib&AM_DIR)==0 )
		{	//ɾ��ͬ���ļ�
			if( (finfo.fattrib&AM_RDO)!=0 )
				f_chmod( filename, 0, AM_RDO );
			f_unlink(filename);
		}
		fres = f_mkdir(filename);
		if( fres!=FR_OK && fres!=FR_EXIST )
			goto reload_SD;
		
		//����Log�ļ�
		filename_len = strlen(filename);
		sprintf( &filename[filename_len], "/%02d%02d.aclog" , RTC_Time.Hours, RTC_Time.Minutes );
		fres = f_stat( filename, &finfo );
		fres = f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
		//д������ͷ�Ͱ汾��Ϣ
		buf[0] = 'A';	buf[1] = 'C'; buf[2] = 0; buf[3] = (1<<1) | 1;
		f_write( &SDFile, buf, 4, &length );
		//д�����ַ�
		sprintf( buf, "Prophet%d%02d%02d%02d%02d", RTC_Time.Year, RTC_Time.Month, RTC_Time.Date, RTC_Time.Hours, RTC_Time.Minutes );
		f_write( &SDFile, buf, 24, &length );
		f_close(&SDFile);
		char LogFilename[30];
		strcpy( LogFilename, filename );
		fres = f_open(&SDFile, LogFilename, FA_OPEN_APPEND | FA_WRITE);
		
		TickType_t xLastWakeTime;
		TIME last_flush_TIME;
		while(1)
		{
			/*д��Log*/
				//�ӻ�������ȡҪд��SD��������
				uint32_t t_length = 0;
				while( BufSize > t_length + 2 )
				{
					buf[t_length+0] = 'A';	buf[t_length+1] = 'C';
					length = xMessageBufferReceive( LogStreamBuffer, &buf[t_length+2], BufSize - t_length - 2, 0 );
					if( length > 0 )
					{
						t_length += 2 + length;
					}
					else
						break;
				}

				//����д���ļ�
				if( t_length > 0 )
					f_write( &SDFile, buf, t_length, &length );
				//ÿ��һ��ʱ��ˢ������
				if( last_flush_TIME.get_pass_time() > 2 )
				{
					f_sync(&SDFile);
					last_flush_TIME = TIME::now();
				}
			/*д��Log*/
				
			vTaskDelayUntil( &xLastWakeTime, 1 );
		}
	}
	else	//SD������ʧ���ٴγ���
		goto reload_SD;
}

void init_SDStorage()
{
	//���ݼ�¼����
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//��̬��¼��Ƶ
		MAV_PARAM_TYPE_UINT8 ,	//λ���ٶȼ�¼��Ƶ
		MAV_PARAM_TYPE_UINT8 ,	//ԭʼIMU��¼��Ƶ
		MAV_PARAM_TYPE_UINT8 ,	//��������¼��Ƶ
		MAV_PARAM_TYPE_UINT8 ,	//λ�ô�������¼
		MAV_PARAM_TYPE_UINT8 ,	//���ջ��źż�¼
	};
	SName param_names[] = {
		"SDLog_Att" ,	//��̬��¼��Ƶ
		"SDLog_LocalNed" ,	//λ���ٶȼ�¼��Ƶ
		"SDLog_RawIMU" ,	//ԭʼIMU��¼��Ƶ
		"SDLog_AttCtrl" ,	//��̬��������¼��Ƶ
		"SDLog_PosCtrl" ,	//λ�ÿ�������¼��Ƶ
		"SDLog_PosSensor" ,	//λ�ô�������¼
		"SDLog_Receiver" ,	//���ջ��źż�¼
	};
	uint64_t initial_cfg[] = {
		5 ,	//��̬��¼��Ƶ
		5 ,	//λ���ٶȼ�¼��Ƶ
		0 ,	//ԭʼIMU��¼��Ƶ
		2 ,	//��������¼��Ƶ
		1 ,	//λ�ô�������¼
		1 ,	//���ջ��źż�¼
	};
	ParamGroupRegister( "SDLog", 2, 6, param_types, param_names, (uint64_t*)&initial_cfg );
	
	xTaskCreate( SDS_Task , "SDS_Task" ,2048 , NULL , SysPriority_UserTask , NULL );
}