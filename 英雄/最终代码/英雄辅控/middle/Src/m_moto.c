#include "m_moto.h"
#include "m_remote.h" 
#include "main.h"
#include "stdint.h"
#include "stdio.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"

 u8 i;
moto_measure_t moto_chassis[6];//3508测量量
moto_measure_t moto_gym[2];//6623测量量
moto_measure_t moto_toggle;
moto_measure_t moto_3508_info;//作用？？？？
moto_measure_t moto_6623_info;//作用？？？？
moto_measure_t moto_info;//作用？？？？


void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
void get_moto2006_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

HAL_StatusTypeDef can_send_msg()
{ 
//	if(_hcan->Instance->ESR){
//		//can error occured, sleep can and reset!
//		_hcan->Instance->MCR |= 0x02;
//		_hcan->Instance->MCR &= ~(0x02);
//	}//这个是zw试过的可以解决can错误  有待验证！
	return HAL_OK;
}

float ZGyroModuleAngle;


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	//ignore can1 or can2.
		switch(_hcan->pRxMsg->StdId)
		{
				case CAN_YAW_FEEDBACK_ID:
				case CAN_PIT_FEEDBACK_ID:
				{
					i = _hcan->pRxMsg->StdId - CAN_6623Yaw_ID;
				
					moto_gym[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_gym[i], _hcan) : get_moto_measure(&moto_gym[i], _hcan);
				}
				break;
				
				case CAN_3508Moto1_ID:
				case CAN_3508Moto2_ID:
				case CAN_3508Moto3_ID:
				case CAN_3508Moto4_ID:
					{
						i = _hcan->pRxMsg->StdId - CAN_3508Moto1_ID;
				
						moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
					}
						break;	
				
//				case CAN_2006Moto_ID:
//						moto_toggle.msg_cnt++ <= 50	?	get_moto_offset(&moto_toggle, _hcan) : get_moto2006_measure(&moto_toggle, _hcan);
//						break;
				
				case CAN_3508Moto5_ID:
						moto_chassis[4].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[4], _hcan) : get_moto_measure(&moto_chassis[4], _hcan);
						break;
					
				case CAN_3508Moto6_ID:
						moto_chassis[5].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[5], _hcan) : get_moto_measure(&moto_chassis[5], _hcan);
						break;
	  }
		

	//hcan1.Instance->IER|=0x00008F02;
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	

}


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->real_current  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->speed_rpm = ptr->real_current ;	//这里是因为两种电调对应位不一样的信息
	ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])/-5;
	ptr->hall = hcan->pRxMsg->Data[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle ;//- ptr->offset_angle;
}


void get_moto2006_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	static short speedrpm;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	
	speedrpm = ptr->speed_rpm ;
	ptr->speed_rpm  = (int16_t)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->speed_rpm	=	0.1*speedrpm + (1-0.1)*ptr->speed_rpm;
	
	ptr->real_current = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])*10.f/10000.f;

	ptr->hall = hcan->pRxMsg->Data[6];
	
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;// - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void Set_3508_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

	hcan->pTxMsg->StdId = 0x200;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;
	hcan->pTxMsg->Data[6] = iq4 >> 8;
	hcan->pTxMsg->Data[7] = iq4;
	
	HAL_CAN_Transmit(hcan, 0);
}	


void Set_6623_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

	hcan->pTxMsg->StdId = 0x1FF;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = iq1 >> 8;
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = iq2 >> 8;
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;
	hcan->pTxMsg->Data[6] = iq4 >> 8;
	hcan->pTxMsg->Data[7] = iq4;
	
	HAL_CAN_Transmit(hcan, 0);
}

