#include "drv_USB.hpp"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "Basic.hpp"
#include "CommuLink.hpp"

//USB�豸����
USBD_HandleTypeDef hUsbDeviceFS;

//USB���ͻ�����
static SemaphoreHandle_t USBD_VCOM_TxSemphr;
//USB������Ϣ������
#define TxStreamBufferSize 1024
#define TxBufferSize 256
static uint8_t USBD_VCOM_TxBuffer[TxBufferSize];
static StreamBufferHandle_t USBD_VCOM_TxStreamBuffer;

//USB���ջ�����
static SemaphoreHandle_t USBD_VCOM_RxSemphr;
//USB���ջ�����
#define RxStreamBufferSize 1024
static StreamBufferHandle_t USBD_VCOM_RxStreamBuffer;

/*��������
	������֤����������
	����֮��������
*/
bool Lock_USBD_VCOM( double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks > 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	if( xSemaphoreTakeRecursive( USBD_VCOM_TxSemphr , Sync_waitTicks ) == pdTRUE )
		return true;
	return false;
}
void Unlock_USBD_VCOM()
{
	xSemaphoreGiveRecursive( USBD_VCOM_TxSemphr );
}

/*USBD���⴮�ڷ��ͺ�������Ҫ���͵�����ѹ�뻺������
	data:Ҫ���͵�����ָ��
	length:Ҫ���͵����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Send_waitTime:�ȴ��������пռ�����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʷ��͵��ֽ���
	���ͣ������ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ͻ�����λ�ò��㣩
				��ֻ����ǰ������ݣ�ֻ��ǰ�������ѹ�뻺������
*/
uint16_t Write_USBD_VCOM( const uint8_t *data, uint16_t length, double Send_waitTime, double Sync_waitTime )
{
	if( length == 0 )
		return 0;
	uint32_t Sync_waitTicks;
	if( Sync_waitTicks >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Send_waitTicks;
	if( Send_waitTime >= 0 )
		Send_waitTicks = Send_waitTime*configTICK_RATE_HZ;
	else
		Send_waitTicks = portMAX_DELAY;
	//��ȡ�ź���
	if( xSemaphoreTakeRecursive( USBD_VCOM_TxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
		uint16_t data_sent = xStreamBufferSend( USBD_VCOM_TxStreamBuffer , data , length , Send_waitTicks );
		USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
		if( hcdc->TxState == 0 )
		{
			//USB���пɷ���
			uint16_t sd_length;
			sd_length = xStreamBufferReceive( USBD_VCOM_TxStreamBuffer , USBD_VCOM_TxBuffer , TxBufferSize , 0 );
			if( sd_length > 0 )
				CDC_Transmit_FS( USBD_VCOM_TxBuffer , sd_length );
		}
		//�ͷ��ź���
		xSemaphoreGiveRecursive( USBD_VCOM_TxSemphr );
		return data_sent;
	}
	else
		return 0;
}

/*USBD���⴮�ڽ��պ�������Ҫ���͵�����ѹ�뻺������
	data:��������ָ��
	length:Ҫ���յ����ݳ��ȣ��ֽڣ�
	Sync_waitTime:�߳�ͬ�����ȴ�ʱ�䣨s��
	Rc_waitTime:�ȴ����ݵ����ȴ�ʱ�䣨s��
	����ֵ��ʵ�ʽ��յ����ֽ���
	����:�����ָ���ȴ�ʱ���ڽ��ղ����㹻���ݣ����ջ�������û��ô�����ݣ�
				�ͽ��վ����������
*/
uint16_t Read_USBD_VCOM( uint8_t *data, uint16_t length, double Rc_waitTime, double Sync_waitTime )
{
	uint32_t Sync_waitTicks;
	if( Sync_waitTime >= 0 )
		Sync_waitTicks = Sync_waitTime*configTICK_RATE_HZ;
	else
		Sync_waitTicks = portMAX_DELAY;
	uint32_t Rc_waitTicks;
	if( Rc_waitTime >= 0 )
		Rc_waitTicks = Rc_waitTime*configTICK_RATE_HZ;
	else
		Rc_waitTicks = portMAX_DELAY;
	//��ȡ�ź���
	if( xSemaphoreTake( USBD_VCOM_RxSemphr , Sync_waitTicks ) == pdTRUE )
	{	
//		if( length >= xStreamBufferBytesAvailable(USBD_VCOM_RxStreamBuffer) )
//			xEventGroupClearBits( CommuPortsRxEvents , CommuPort_USBD_VCOM );
		uint16_t rc_length = xStreamBufferReceive( USBD_VCOM_RxStreamBuffer , data , length , Rc_waitTicks );
//		if( xStreamBufferBytesAvailable(USBD_VCOM_RxStreamBuffer) > 0 )
//			xEventGroupSetBits( CommuPortsRxEvents , CommuPort_USBD_VCOM );
		
		//�ͷ��ź���
		xSemaphoreGive( USBD_VCOM_RxSemphr );
		return rc_length;
	}
	else
		return 0;
}

void init_drv_USB(void)
{
	//���ͺ���������
	USBD_VCOM_TxSemphr = xSemaphoreCreateRecursiveMutex();
	//���պ���������
	USBD_VCOM_RxSemphr = xSemaphoreCreateMutex();
	//���ͻ�����
	USBD_VCOM_TxStreamBuffer = xStreamBufferCreate( TxStreamBufferSize , 1 );
	//���ջ�����
	USBD_VCOM_RxStreamBuffer = xStreamBufferCreate( RxStreamBufferSize , 1 );
		
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    while(1);
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    while(1);
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    while(1);
  }
	//USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USBD_VCOM_RxBuffer);
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    while(1);
  }
  //HAL_PWREx_EnableUSBVoltageDetector();
	
	Port USBDVCOM;
	USBDVCOM.write = Write_USBD_VCOM;
	USBDVCOM.lock = Lock_USBD_VCOM;
	USBDVCOM.unlock = Unlock_USBD_VCOM;
	USBDVCOM.read = Read_USBD_VCOM;
	CommuPortRegister(USBDVCOM);
}


/*USB�����жϷ������*/
	BaseType_t USBD_INT_HigherPriorityTaskWoken = pdFALSE;
	static volatile bool Tx_Complete = false;
	extern "C" void USBD_VCOM_TX_Complete_Callback()
	{
		Tx_Complete = true;
	}

	extern "C" void OTG_FS_IRQHandler(void)
	{
		extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
		HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
		
		BaseType_t HigherPriorityTaskWoken = USBD_INT_HigherPriorityTaskWoken;
		USBD_INT_HigherPriorityTaskWoken = pdFALSE;
		if( Tx_Complete )
		{
			uint16_t length = xStreamBufferReceiveFromISR( USBD_VCOM_TxStreamBuffer , USBD_VCOM_TxBuffer , TxBufferSize , &HigherPriorityTaskWoken );
			if( length > 0 )
				CDC_Transmit_FS( USBD_VCOM_TxBuffer , length );
			Tx_Complete = false;
		}
		portYIELD_FROM_ISR(HigherPriorityTaskWoken);
	}
/*USB�����жϷ������*/
	
/*USB���շ������*/
	extern "C" void CDC_Receive_FS_Callback(uint8_t* pbuf, uint32_t Len)
	{
		if( Len > 0 )
			xStreamBufferSendFromISR( USBD_VCOM_RxStreamBuffer , pbuf , Len , &USBD_INT_HigherPriorityTaskWoken );
		//xEventGroupSetBitsFromISR( CommuPortsRxEvents , CommuPort_USBD_VCOM , &USBD_INT_HigherPriorityTaskWoken );
	}
/*USB���շ������*/

