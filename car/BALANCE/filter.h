#ifndef __FILTER_H
#define __FILTER_H
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
extern float angle,angle1, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void First_order_Filter(float angle_m, float gyro_m);
#endif
