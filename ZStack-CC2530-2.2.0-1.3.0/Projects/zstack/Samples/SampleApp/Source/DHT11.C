#include <ioCC2530.h>
#include "OnBoard.h"

#define uint unsigned int
#define uchar unsigned char

#define wenshi P0_6

//温湿度定义
uchar ucharFLAG,uchartemp;
uchar shidu_shi,shidu_ge,wendu_shi,wendu_ge=4;
uchar ucharT_data_H,ucharT_data_L,ucharRH_data_H,ucharRH_data_L,ucharcheckdata;
uchar ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_temp,ucharRH_data_L_temp,ucharcheckdata_temp;
uchar ucharcomdata;

/*******函数声明*********/
void Delay_us(void); //1 us延时
void Delay_10us(void); //10 us延时
void Delay_ms(uint Time);//n ms延时
void COM(void);	// 温湿写入
void DHT11_TEST(void) ;  //温湿传感启动

uchar temp[2]={0,0}; 
uchar temp1[5]="temp=";
uchar humidity[2]={0,0};
uchar humidity1[9]="humidity=";
 
/****************************
        延时函数
*****************************/
void Delay_us(void) //1 us延时
{
   MicroWait(1);   
}

void Delay_10us(void) //10 us延时
{
  MicroWait(10); 
}

void Delay_ms(uint Time)//n ms延时
{
  unsigned char i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delay_10us();
  }
}


/***********************
   温湿度传感
***********************/
void COM(void)	// 温湿写入
{     
    uchar i;         
    for(i=0;i<8;i++)    
    {
     ucharFLAG=2; 
     while((!wenshi)&&ucharFLAG++);//等待总线被拉高
     Delay_10us();
     Delay_10us();
     Delay_10us();  
     uchartemp=0;
     if(wenshi)uchartemp=1; //总线拉高后等待30us，如果总线仍然为高电平，则采集到的数据位为1，否则为0
     ucharFLAG=2;
     while((wenshi)&&ucharFLAG++);   //等待总线被拉低
     if(ucharFLAG==1)break;    //超时则跳出for循环
     
     //判断数据位是0还是1
     ucharcomdata<<=1;
     ucharcomdata|=uchartemp; 
     }    
}

void DHT11_TEST(void)   //温湿传感启动
{
    wenshi=0;
    Delay_ms(19);  //>18MS
    wenshi=1; 
    P0DIR &= ~0x40; //重新配置IO口方向 输入
    Delay_10us();
    Delay_10us();						
    Delay_10us();
    Delay_10us();  
     if(!wenshi) //如果输入是0 成功 传感器写入数据
     {
      ucharFLAG=2; 
      while((!wenshi)&&ucharFLAG++);//如果总线一直是低电平则循环等待拉高
      ucharFLAG=2;
      while((wenshi)&&ucharFLAG++); //如果总线一直是高电平则循环等待拉低，开始向主机发送数据
      COM();
      ucharRH_data_H_temp=ucharcomdata;//湿度高8位（整数部分）
      COM();
      ucharRH_data_L_temp=ucharcomdata;//湿度低8位（小数部分）
      COM();
      ucharT_data_H_temp=ucharcomdata;//温度高8位
      COM();
      ucharT_data_L_temp=ucharcomdata;//温度低8位
      COM();
      ucharcheckdata_temp=ucharcomdata;//校验和末8位
      wenshi=1;   //总线拉高
      uchartemp=(ucharT_data_H_temp+ucharT_data_L_temp+ucharRH_data_H_temp+ucharRH_data_L_temp);
       if(uchartemp==ucharcheckdata_temp)
      {
          ucharRH_data_H=ucharRH_data_H_temp;
          ucharRH_data_L=ucharRH_data_L_temp;
          ucharT_data_H=ucharT_data_H_temp;
          ucharT_data_L=ucharT_data_L_temp;
          ucharcheckdata=ucharcheckdata_temp;
       }
         wendu_shi=ucharT_data_H/10; 
         wendu_ge=ucharT_data_H%10;
	 
         shidu_shi=ucharRH_data_H/10; 
         shidu_ge=ucharRH_data_H%10;        
    } 
    else //没用成功读取，返回0
    {
         wendu_shi=0; 
         wendu_ge=0;
	 
         shidu_shi=0; 
         shidu_ge=0;  
    } 
    
    P0DIR |= 0x40; //IO口需要重新配置 
}
