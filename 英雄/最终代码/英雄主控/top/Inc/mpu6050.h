#ifndef __MPU6050_H
#define __MPU6050_H

extern unsigned char uart3_rx_buff[22];
extern float mpua[3];
extern float mpuw[3];
extern float mpuang[3];
extern float mpuT;

extern void MPUDataProcess(void);

#endif
