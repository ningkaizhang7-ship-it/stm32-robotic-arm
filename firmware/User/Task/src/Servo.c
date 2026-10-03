#include "data.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "Servo.h"
#include "thread_rccu.h"
#include "SLAVE_SteeringEngine_3CH/SLAVE_SteeringEngine_3CH.h"
typedef struct{
	int16_t Num1;//舵机ID
	int16_t Num2;
	int16_t Num3;
	int16_t Num4;
}Servo;
//int16_t Red_1 = 1912;
//int16_t Green_1 = 3293;
//int16_t Blue_1 = 582;


int16_t Open_Claw = 800;
int16_t Close_Claw = 1370;
/* ------------------------------------------------------------*/
Servo bowl_1 		= {//1号碗
								797,
								2097,
								2148,//>>>>>2050
								};
Servo bowl_2 		= {//2号碗
								1017,
								2083,
								2154,//>>>>>2050
								};
Servo bowl_3 		= {//3号碗
								1257,
								2111,
								2147,//>>>>>2050
								};
/* ------------------------------------------------------------*/
Servo Red 		= {//红一层
								3633,
								2752,
								1717
								};
Servo Green 	= {//绿一层
								3207,
								2521,
								1443
								};
Servo Blue 		= {//蓝一层
								2716,
								2614,
								1553
								};
/* ------------------------------------------------------------*/
Servo Red_Stacking 		= {//红码垛
								3633,
								2598,
								1996	
								};
Servo Green_Stacking 	= {//绿码垛
								3227,
								2216,
								1866
								};
Servo Blue_Stacking 	= {//蓝码垛
								2728,
								2352,
								1866
								};
/* ------------------------------------------------------------*/

		void Rest_action()//归位动作
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S, 2, 1835, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S, 3, 2382, 3250, 60);
			My_mDelay(800);
		}
		void Rest_action_S2()//初始动作
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, 2140, 3250, 60);		//2号舵机控制前后
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 2280, 3250, 60);		//3号舵机控制上下
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, 1040, 2500,60);	
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);		//4号舵机手爪
			HAL_Delay(800);
		}
		
		void QR_action()//识别二维码动作
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S, 1, 3108, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S, 2, 1919, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S, 3, 2373, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);		//4号舵机手爪
			My_mDelay(800);
		}
		
		void IdentifyColor_action()//识别原料颜色动作
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, 3000, 2500,60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, 2165, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 2165, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);
			HAL_Delay(800);
		}
		void IdentifyCircle_action()//粗加工区识别圆环动作
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, 3100, 2500,60);
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, 2500, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 1918, 3250, 60);//2154
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);
			HAL_Delay(800);
		}
		void Get_A()//抓取原料区物料
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, 3136, 2500,60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, 2263, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 1920, 3250, 60);
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw,  3250, 60);//抓
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Put_1()//放到1号碗（红）
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, bowl_1.Num1, 2500,60);//转到1号碗方向
			HAL_Delay(1200);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, bowl_1.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, bowl_1.Num3, 1897, 30);//向下伸
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//放
			HAL_Delay(300);
		}
		void Put_2()//放到2号碗（绿）
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, bowl_2.Num1, 2500,60);//转到2号碗方向
			HAL_Delay(1000);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, bowl_2.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, bowl_2.Num3, 1897, 30);//向下伸
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//放
			HAL_Delay(300);
		}
		void Put_3()//放到3号碗（蓝）
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, bowl_3.Num1, 2500,60);//转到3号碗方向
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, bowl_3.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, bowl_3.Num3, 1897, 30);//向下伸	
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//放
			HAL_Delay(300);
		}
		void Get_1()//抓1号碗物料（红）
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//放
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, bowl_1.Num1, 2500,60);//转到1号碗方向
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, bowl_1.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, bowl_1.Num3, 3250, 60);//向下伸
			HAL_Delay(400);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);//抓
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 2250, 1897, 30);//向上伸			
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Get_2()//抓2号碗物料（绿）
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//放
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, bowl_2.Num1, 2500,60);//转到2号碗方向
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, bowl_2.Num2, 3250, 60);//向前伸		
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, bowl_2.Num3, 3250, 60);//向下伸			
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);//抓
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 2250, 1897, 30);//向上伸			
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Get_3()//抓3号碗物料（蓝）
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//放
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, bowl_3.Num1, 2500,60);//转到3号碗方向
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, bowl_3.Num2, 3250, 60);//向前伸		
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, bowl_3.Num3, 3250, 60);//向下伸		
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);//抓
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, 2250, 1897, 30);//向上伸
			HAL_Delay(300);
			Rest_action();//归位
		}
		void Put_Red()//放红环——第一层
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Red.Num1, 2500,60);//转到红环方向
			HAL_Delay(1200);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Red.Num2, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Red.Num3, 3250, 60);
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(800);//300
			Rest_action();
		}
		void Put_Green()//放绿环——第一层
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Green.Num1, 2500,60);//转到绿环方向
			HAL_Delay(1000);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Green.Num3, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Green.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Put_Blue()//放蓝环——第一层
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Blue.Num1, 2500,60);//转到蓝环方向
			HAL_Delay(1000);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Blue.Num2, 3250, 60);//向前伸
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Blue.Num3, 3250, 60);
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Put_Red_Stacking()//放红环——码垛
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Red_Stacking.Num1, 2500,60);//转到红环方向
			HAL_Delay(1200);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Red_Stacking.Num2, 3250, 60);//向前伸
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Red_Stacking.Num3, 3250, 60);
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);//300
			Rest_action();
		}
		void Put_Green_Stacking()//放绿环——码垛
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Green_Stacking.Num1, 2500,60);//转到绿环方向
			HAL_Delay(1000);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Green_Stacking.Num3, 4250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Green_Stacking.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Put_Blue_Stacking()//放蓝环——码垛
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Blue_Stacking.Num1, 2500,60);//转到蓝环方向
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Blue_Stacking.Num2, 3250, 60);//向前伸
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Blue_Stacking.Num3, 3250, 60);
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			Rest_action();
		}
		void Get_Red()//抓红环物料
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Red.Num1, 2500,60);//转到红环方向
			HAL_Delay(1200);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Red.Num2, 3250, 60);//向前伸
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Red.Num3, 3250, 60);
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);//抓
			HAL_Delay(500);
			Rest_action();
		}
		void Get_Green()//抓绿环物料
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Green.Num1, 2500,60);//转到绿环方向
			HAL_Delay(1000);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Green.Num3, 3250, 60);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Green.Num2, 3250, 60);//向前伸
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);//抓
			HAL_Delay(500);
			Rest_action();
		}
		void Get_Blue()//抓蓝环物料
		{
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Open_Claw, 3250, 60);//爪子松开
			HAL_Delay(300);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,1, Blue.Num1, 2500,60);//转到蓝环方向
			HAL_Delay(800);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,2, Blue.Num2, 3250, 60);//向前伸
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,3, Blue.Num3, 3250, 60);			
			HAL_Delay(500);
			SLAVE_SteeringEngine3CH_BUSMotorControl(&Servo_S,4, Close_Claw, 3250, 60);//抓
			HAL_Delay(500);
			Rest_action();
		}