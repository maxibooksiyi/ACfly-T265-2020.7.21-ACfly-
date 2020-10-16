#include "drv_Uart3.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"

static void SDI_Server(void* pvParameters)
{
	/*״̬������*/
		static uint8_t rc_step1 = 0;	//0�����հ�ͷ'A' 'C'
																	//1������1�ֽ���Ϣ���
																	//2������1�ֽ���Ϣ����
																	//3���������ݰ�����
																	//4������2�ֽ�У��
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 4*6
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sumA;
		static uint8_t sumB;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*״̬������*/
	
	while(1)
	{
		uint8_t r_data;
		if( Read_Uart3( &r_data, 1, 2, 0.5 ) )
		{
			switch( rc_step1 )
			{
				case 0 :
					//���հ�ͷ'A''C'
					if( rc_step2 == 0 )
					{
						if( r_data == 'A' )
							rc_step2 = 1;
					}
					else
					{
						if( r_data == 'C' )
						{
							rc_step1 = 1;
							rc_step2 = 0;
							sumA = sumB = 0;
						}
						else
							rc_step2 = 0;
					}
					break;
					
				case 1:
					//������Ϣ���
					msg_type = r_data;
					sumA += r_data;
					sumB += sumA;
					rc_step1 = 2;
					rc_step2 = 0;
					break;
				
				case 2:
					//������Ϣ����
					if( r_data > MAX_SDI_PACKET_SIZE )
					{
						reset_SDI_RC;
						break;
					}
					msg_length = r_data;
					sumA += r_data;
					sumB += sumA;
					if( msg_length == 0 )
						rc_step1 = 4;
					else
						rc_step1 = 3;
					rc_step2 = 0;
					break;
					
				case 3:
					//�������ݰ�
					msg_pack[ rc_step2 ] = r_data;				
					sumA += r_data;
					sumB += sumA;
					++rc_step2;
					if( rc_step2 >= msg_length )
					{
						rc_step1 = 4;
						rc_step2 = 0;
					}
					break;
					
				case 4:
					//����У��λ
					if( rc_step2 == 0 )
					{
						if( sumA != r_data )
						{
							reset_SDI_RC;
							break;
						}
						rc_step2 = 1;
					}
					else
					{
						if( sumB == r_data )
						{
							ModeMsg msg;
							msg.cmd = msg_type;
							for( uint16_t i = 0; i < 8*4; ++i )
							{
								if( i < msg_length )
									((uint8_t*)(&msg.params[0]))[i] = msg_pack[i];
								else
									((uint8_t*)(&msg.params[0]))[i] = 0;
							}
							SendMsgToMode( msg, 0 );
						}
						reset_SDI_RC;
					}
					break;					
			}					
		}
	}
}

void init_drv_SDI()
{
	//������115200
	SetBaudRate_Uart3( 115200, 2, 2 );
	xTaskCreate( SDI_Server, "SDI", 1024, NULL, SysPriority_UserTask, NULL);
}