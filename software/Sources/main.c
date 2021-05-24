#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


uint chargetime=50,dischargetime=220;//设置充放电时间初始值
ulong injectionwidth1=5,injectionwidth2=5,injectionwidth3=5,injectionwidth4=5,injectionwidth5=5;//喷油脉宽
ulong period1=5,period2=5,period3=5,period4=5,period5=5;  //喷油间隔，这些表示时间的变量初值不能设置为0，否则单片机再复位后按照最大值65536来处理这些变量

uint data[12];
uint count=0;  //通讯接受数据计数
uchar flag=0;  //(一组数据发送完毕标记)
uint times=1;     //(喷射次数)以及作为发生喷油干涉时的标识
uint n=0,m=0;
uint pre1=0,pre2=0;

ulong single=0,startT=0;
ulong time1=0,time2=0,time3=0,time4=0;

uint min=0;//用来表示脉宽和喷油间隔中的最小值
uint shuzu[5];
int c=0;




/*********************总线始时钟设置：外部晶振16MHz,单片机总线时钟设置为16MHz*********************/
void Busclock_Init(void)
{
 CLKSEL=0x00;//将PLL锁相环脱离系统从而进行配置
 PLLCTL_PLLON=1;//打开PLL锁相环
 SYNR=0X00;
 REFDV=0X00;//这两个寄存器设置参考计算总线频率的公式
 while(!(CRGFLG_LOCK==1));//等待PLL锁相环稳定
 CLKSEL_PLLSEL=1;//在系统中使用PLL锁相环
}

/*********************SCI模块的初始化*********************/ 
void SCI_Init(void) 
{                  //有两个SCI模块，SCI0和SCI1
 SCI0BD=104;//设置波特率，总线时钟16MHZ，波特率为16000000/（16*104）=9600bit/s，或者分开写SCI0BDL=104，SCI0BDH=0
 SCI0CR1=0x00;//LOOPS=0,RSRC=0,普通操作模式；SCIWAI=0，SCI在等待模式下使能；M=0，1位起始位，8位数据位，1位停止位；PE=0，禁止奇偶校验
 SCI0CR2=0x2C;//TIE=0，禁止发送器中断使能; TCIE=0，禁止发送完成中断使能 ;RIE=1，允许接收器满中断使能 ;ILIE=0，禁止空闲线中断使能 ;TE=1，允许使用发送器 ;RE=1，允许使用接收器;
  
}

/*********************ECT初始化函数*********************/
void ECT_Init(void) 
{
 /*配置自由运行定时器的分频系数*/
 //ECT_TSCR2=ECT_TSCR2&0XF8|0x04;//预分频系数为16，即定时器频率为总线频率的1/16,即1MHZ，&0xF8是清除后三位，|0x04是设定分频系数
 ECT_TSCR2=0x04;
 ECT_TSCR2_TOI=1;//允许定时器溢出中断
 ECT_TSCR2_TCRE=0;//不允许OC7通道的输出比较事件复位寄存器
 
 /*输出比较相关寄存器设置*/
 ECT_TIOS_IOS4=1;//通道4设置为输出比较功能
 ECT_TIOS_IOS5=1;//通道5设置为输出比较功能
 ECT_TIOS_IOS6=1;//通道6设置为输出比较功能
 ECT_TIOS_IOS7=1;//通道7设置为输出比较功能
   
 ECT_TCTL2 = 0x00;  // 前四个通道设置为定时器与输出引脚断开
 ECT_TCTL1 = 0x00;  // 后四个通道设置为定时器与输出引脚断开
 //ECT_TCTL1=ECT_TCTL1&0x00|0x55;//通道4，5，6，7输出比较翻转模式
 
 ECT_TIE_C4I=0;//禁止通道4中断使能
 ECT_TIE_C5I=0;
 ECT_TIE_C6I=0;
 ECT_TIE_C7I=0;
 
 ECT_TTOV=0;//禁止在定时器溢出时产生比较动作
 //ECT_OCPD_OCPD4=0;//允许输出比较动作出现在定时器通道端口引脚上
 //ECT_OCPD_OCPD5=0;
 //ECT_OCPD_OCPD6=0;
 //ECT_OCPD_OCPD7=0;
 
  /*输入捕捉相关寄存器设置*/ 
  
 ECT_TIOS_IOS0=0;//通道0设置为输入捕捉功能
 ECT_TIOS_IOS1=0;
 ECT_TIOS_IOS2=0;
 ECT_TIOS_IOS3=0;
 
 ECT_TCTL4=0xff;//前四个通道捕捉上升沿和下降沿
 ECT_ICOVW_NOVW0=0; //通道0的捕捉寄存器可以被覆盖
                    //如果某位NOVWx=0，新的捕捉事件或转移动作发生时，不论捕捉或保持寄存器是否为空，均直接保存新的结果
                     //如果某位NOVWx=1，对应的寄存器不允许被覆盖，但如若处于空白状态则任然允许覆盖
 ECT_ICOVW_NOVW1=0;
 ECT_ICOVW_NOVW2=0;
 ECT_ICOVW_NOVW3=0;
 
 ECT_ICSYS_SH04=0;
 ECT_ICSYS_SH15=0;//通道1和5各自使用自己的端口引脚，而不共享
 ECT_ICSYS_SH26=0;
 ECT_ICSYS_SH37=0; 
 
 ECT_TIE_C0I=1;//通道0的输入捕捉中断允许 
 ECT_TIE_C1I=1; 
 ECT_TIE_C2I=1; 
 ECT_TIE_C3I=1;  
    
 ECT_TSCR1_TEN=1;  //开启自由运行定时器
                                             
}




/*********************延时函数*********************/
 void delayms(uint ms) //延时
 {
  uint i,j;
  for(i=0;i<ms;i++)
     for(j=0;j<2000;j++);    
}

/*********************延时函数*********************/
 void delayus(uint us) //延时
 {
  uint a,b;
  for(a=0;a<us;a++)
     for(b=0;b<2;b++);    
}



/********************串口接收中断服务函数，中断向量20********************/

#pragma CODE_SEG NON_BANKED//以下放入非分页区
void interrupt 20 SCI_GETDATA(void) 
{
  if(SCI0SR1_RDRF==1) //检测接收数据寄存器是否接受到数据
  {
                   
    data[count]=SCI0DRL;
    if(data[count]>0x63) data[count]=0x63;//保证上位机每个框中输入的数据小于100，当大于100时，取99（十进制99表示为十六进制就是0x63）
    count++;
          
    if(count>11)    //检查11个数据是否全部发送完成
    {
      count=0;
      flag=1;
    }
                  
  }
}

/********************ECT定时器溢出中断服务函数，中断向量16********************/

 void interrupt 16 OVERFLOW_ECT(void) 
 {
  ECT_TFLG2_TOF=1;//清除中断标识   不让打断
  single++;
 }



/********************输入捕捉中断服务函数，中断向量8，9，10，11  通道0，1，2，3*******************/

 void interrupt 8 CAPTURE0_ISR(void) 
{
  ECT_TFLG1_C0F=1;//清除中断标识   不让被打断
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT0==1)   
    {   
      n=0; 
      PORTA_PA2=1;  //选缸开启
      ECT_TIE_C4I=1;//允许通道4中断使能
      ECT_TC4=ECT_TCNT+10;//试验测得从MOTEC触发信号到喷油器动作延时大概200us
    }  

    if(PTT_PTT0==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA2=0;  //选缸关闭
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  

void interrupt 9 CAPTURE1_ISR(void) 
{
  ECT_TFLG1_C1F=1;//清除中断标识   不让被打断
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT1==1)   
    {   
      n=0; 
      PORTA_PA3=1;  //选缸开启
      ECT_TIE_C4I=1;//允许通道4中断使能
      ECT_TC4=ECT_TCNT+10;//试验测得从MOTEC触发信号到喷油器动作延时大概200us
    }  

    if(PTT_PTT1==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA3=0;  //选缸关闭
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  
 
void interrupt 10 CAPTURE2_ISR(void) 
{
  ECT_TFLG1_C2F=1;//清除中断标识   不让被打断
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT2==1)   
    {   
      n=0; 
      PORTA_PA4=1;  //选缸开启
      ECT_TIE_C4I=1;//允许通道4中断使能
      ECT_TC4=ECT_TCNT+10;//试验测得从MOTEC触发信号到喷油器动作延时大概200us
    }  

    if(PTT_PTT2==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA4=0;  //选缸关闭
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  
   
void interrupt 11 CAPTURE3_ISR(void) 
{
  ECT_TFLG1_C3F=1;//清除中断标识   不让被打断
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT3==1)   
    {   
      n=0; 
      PORTA_PA5=1;  //选缸开启
      ECT_TIE_C4I=1;//允许通道4中断使能
      ECT_TC4=ECT_TCNT+10;//试验测得从MOTEC触发信号到喷油器动作延时大概200us
    }  

    if(PTT_PTT3==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA5=0;  //选缸关闭
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  
 

/*********************输出比较中断服务函数，中断向量12，13  通道4，5*********************/

 void interrupt 12 COMPARE4_ISR(void)   //充电中断函数
{
 ECT_TFLG1_C4F=1;//标志位写1清零 
 //ECT_TIE_C0I=0;//禁止通道0输入捕捉中断使能 
  
 switch(n) 
 {
   case 0:
          
          PORTA_PA1=0;
          ECT_TC4=ECT_TC4+chargetime; //设置充电时间60us（ECT时钟频率为1MHz，所以每隔1us计数一次），chargetime初值不能是0，否则复位时chargetime会当做最大值65536来处理，同理其余变量
          break;
           
   case 1:
          PORTA_PA1=1;
          ECT_TIE_C4I=0;//禁止通道4中断使能
          break;
          
                         
   default:break;                  
             
  }   
  n++;   
}       

/*

 void interrupt 13 COMPARE5_ISR(void)    //放电中断函数
{
 ECT_TFLG1_C5F=1;//标志位写1清零
 //ECT_TIE_C1I=0;//禁止通道1输入捕捉中断使能 
 //if(dischargetime>180) dischargetime=150;
  
 switch(m) 
 {          
   case 0:
          PORTA_PA0=0;
          ECT_TC5=ECT_TC5+dischargetime; //设置放电时间60us（ECT时钟频率为1MHz，所以每隔1us计数一次)
          break;
               
   case 1:
          PORTA_PA0=1; 
          ECT_TIE_C5I=0;//禁止通道5中断使能
          break;
                                            
   default:break;                  
             
  }   
  m++;   
}      

*/

#pragma CODE_SEG DEFAULT//以下放入分页区


void main(void) 
{
  /* put your own code here */
  
  Busclock_Init();//调用总线时钟设置函数
  SCI_Init(); //调用SCI初始化函数 
  ECT_Init(); //调用ECT初始化函数
  
  DDRT_DDRT0=0;//定义T0口为触发输入
  DDRT_DDRT1=0;//定义T1口为触发输入
  DDRT_DDRT2=0;//定义T2口为触发输入
  DDRT_DDRT3=0;//定义T3口为触发输入
 // PTT_PTT1=0;//定义T1口输出为低电平
  
  DDRA_DDRA0=1;//定义A0口为输出
  PORTA_PA0=1;//定义A0口输出为高电平
  
  DDRA_DDRA1=1;//定义A1口为输出
  PORTA_PA1=1;//定义A1口输出为高电平
  
  DDRA_DDRA2=1;//定义A2口为输出
  PORTA_PA2=0;//定义A2口输出为低电平
  
  DDRA_DDRA3=1;//定义A3口为输出
  PORTA_PA3=0;//定义A3口输出为低电平
  
  DDRA_DDRA4=1;//定义A4口为输出
  PORTA_PA4=0;//定义A4口输出为低电平
  
  DDRA_DDRA5=1;//定义A5口为输出
  PORTA_PA5=0;//定义A5口输出为低电平
  
 
  
	EnableInterrupts; //打开全局中断
  delayms(10);
  
  for(;;)
  {
    _FEED_COP(); /* feeds the dog */
    
     if(flag==1)
   {
    flag=0;
    
    if((data[0]==0x0a)&&(data[11]==0x0a)) //检测喷油脉宽设定指令，第一个数据和最后一个数据是指令码，如果检测到两个数据是十进制数据10（十六进制0x0a），则证明传输过来的是喷油脉宽设定数据
    {
      injectionwidth1=data[1]*100+data[2]; //将五个数分别赋值给5个变量
      injectionwidth2=data[3]*100+data[4];//乘以100的原因是因为在上位机中将千百十个分开传输的，因为ECT模块的时钟为1MHz，每隔1us计数一次
      injectionwidth3=data[5]*100+data[6];
      injectionwidth4=data[7]*100+data[8];
      injectionwidth5=data[9]*100+data[10];
      
      if(injectionwidth1==0) injectionwidth1=5;
      if(injectionwidth2==0) injectionwidth2=5;
      if(injectionwidth3==0) injectionwidth3=5;
      if(injectionwidth4==0) injectionwidth4=5;
      if(injectionwidth5==0) injectionwidth5=5; //设定脉宽的最小值，如果为0，单片机将按照65536计算
      
      
    
      
      SCI0DRL=data[0];  //如果是喷油脉宽设定指令，则向上位机返回data[1]
      while(!SCI0SR1_TC) 
      {
       ;
      } 
     
    }
    
    
     if((data[0]==0x14)&&(data[11]==0x14))  //检测喷油间隔设定指令，第一个数据和最后一个数据是指令码，如果检测到两个数据是十进制数据20（十六进制0x14），则证明传输过来的是喷油间隔时间设定数据
    {
      period1=data[1]*100+data[2];  //将五个数分别幅赋值给5个变量 ,16位计数器最大计数65535us
      period2=data[3]*100+data[4];  
      period3=data[5]*100+data[6];
      period4=data[7]*100+data[8];
      period5=data[9]*100+data[10];
      
      if(period1==0) period1=5;
      if(period2==0) period2=5;
      if(period3==0) period3=5;
      if(period4==0) period4=5;
      if(period5==0) period5=5; //设定间隔的最小值，如果为0，单片机将按照65536计算
      
            
/*    shuzu[0]=period1;
      shuzu[1]=period2;
      shuzu[2]=period3;
      shuzu[3]=period4; 
      shuzu[4]=period5;  
      min=shuzu[0];
      for(c=0;c<=4;c++) 
      {
       if(min>=shuzu[c]) min=shuzu[c];     //用来得到最小的喷油间隔，在设置最大放电时间时使用
      }  */
      
      
      SCI0DRL=data[0];   //如果是喷油间隔设定指令，向上位机返回data[2]
      while(!SCI0SR1_TC) //等待发送完成
      {
       ;
      }                                                                                                         
      
     
    }
    
    
    
     if((data[0]==0x1e)&&(data[11]==0x1e))   //检测充放电时间及次数设定指令，第一个数据和最后一个数据是指令码，如果检测到两个数据是十进制数据30（十六进制0x1e），则证明传输过来的是充放电时间及次数设定数据
    {
      chargetime=data[1]*100+data[2];  //将五个数分别幅赋值给3个变量
      dischargetime=data[3]*100+data[4];//乘以100的原因是因为在上位机中将千百十个分开传输的，但是计时用的ECT模块的时钟为1，即每隔1us计数一次。而变量times不是作为计数定时用的
      times=data[5];
      if(chargetime>=0x82) chargetime=0x82;//设定最大充电时间130us
      if(chargetime==0) chargetime=3; //设定最小充电时间，不能为0，否则单片机按照65536计算
      
      
      if(dischargetime==0) dischargetime=3; //设定最小放电时间，不能为0，否则单片机按照65536计算
      
     
     
      SCI0DRL=data[0];      //如果是充放电时间及次数设定指令，则向上位机返回data[3]
      while(!SCI0SR1_TC) 
      {
       ;
      }
         
    }
    
     else 
     {
      count=0;
     }
   }
  
  } /* loop forever */
  /* please make sure that you never leave main */
}
