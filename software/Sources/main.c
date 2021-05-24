#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */


uint chargetime=50,dischargetime=220;//���ó�ŵ�ʱ���ʼֵ
ulong injectionwidth1=5,injectionwidth2=5,injectionwidth3=5,injectionwidth4=5,injectionwidth5=5;//��������
ulong period1=5,period2=5,period3=5,period4=5,period5=5;  //���ͼ������Щ��ʾʱ��ı�����ֵ��������Ϊ0������Ƭ���ٸ�λ�������ֵ65536��������Щ����

uint data[12];
uint count=0;  //ͨѶ�������ݼ���
uchar flag=0;  //(һ�����ݷ�����ϱ��)
uint times=1;     //(�������)�Լ���Ϊ�������͸���ʱ�ı�ʶ
uint n=0,m=0;
uint pre1=0,pre2=0;

ulong single=0,startT=0;
ulong time1=0,time2=0,time3=0,time4=0;

uint min=0;//������ʾ��������ͼ���е���Сֵ
uint shuzu[5];
int c=0;




/*********************����ʼʱ�����ã��ⲿ����16MHz,��Ƭ������ʱ������Ϊ16MHz*********************/
void Busclock_Init(void)
{
 CLKSEL=0x00;//��PLL���໷����ϵͳ�Ӷ���������
 PLLCTL_PLLON=1;//��PLL���໷
 SYNR=0X00;
 REFDV=0X00;//�������Ĵ������òο���������Ƶ�ʵĹ�ʽ
 while(!(CRGFLG_LOCK==1));//�ȴ�PLL���໷�ȶ�
 CLKSEL_PLLSEL=1;//��ϵͳ��ʹ��PLL���໷
}

/*********************SCIģ��ĳ�ʼ��*********************/ 
void SCI_Init(void) 
{                  //������SCIģ�飬SCI0��SCI1
 SCI0BD=104;//���ò����ʣ�����ʱ��16MHZ��������Ϊ16000000/��16*104��=9600bit/s�����߷ֿ�дSCI0BDL=104��SCI0BDH=0
 SCI0CR1=0x00;//LOOPS=0,RSRC=0,��ͨ����ģʽ��SCIWAI=0��SCI�ڵȴ�ģʽ��ʹ�ܣ�M=0��1λ��ʼλ��8λ����λ��1λֹͣλ��PE=0����ֹ��żУ��
 SCI0CR2=0x2C;//TIE=0����ֹ�������ж�ʹ��; TCIE=0����ֹ��������ж�ʹ�� ;RIE=1��������������ж�ʹ�� ;ILIE=0����ֹ�������ж�ʹ�� ;TE=1������ʹ�÷����� ;RE=1������ʹ�ý�����;
  
}

/*********************ECT��ʼ������*********************/
void ECT_Init(void) 
{
 /*�����������ж�ʱ���ķ�Ƶϵ��*/
 //ECT_TSCR2=ECT_TSCR2&0XF8|0x04;//Ԥ��Ƶϵ��Ϊ16������ʱ��Ƶ��Ϊ����Ƶ�ʵ�1/16,��1MHZ��&0xF8���������λ��|0x04���趨��Ƶϵ��
 ECT_TSCR2=0x04;
 ECT_TSCR2_TOI=1;//����ʱ������ж�
 ECT_TSCR2_TCRE=0;//������OC7ͨ��������Ƚ��¼���λ�Ĵ���
 
 /*����Ƚ���ؼĴ�������*/
 ECT_TIOS_IOS4=1;//ͨ��4����Ϊ����ȽϹ���
 ECT_TIOS_IOS5=1;//ͨ��5����Ϊ����ȽϹ���
 ECT_TIOS_IOS6=1;//ͨ��6����Ϊ����ȽϹ���
 ECT_TIOS_IOS7=1;//ͨ��7����Ϊ����ȽϹ���
   
 ECT_TCTL2 = 0x00;  // ǰ�ĸ�ͨ������Ϊ��ʱ����������ŶϿ�
 ECT_TCTL1 = 0x00;  // ���ĸ�ͨ������Ϊ��ʱ����������ŶϿ�
 //ECT_TCTL1=ECT_TCTL1&0x00|0x55;//ͨ��4��5��6��7����ȽϷ�תģʽ
 
 ECT_TIE_C4I=0;//��ֹͨ��4�ж�ʹ��
 ECT_TIE_C5I=0;
 ECT_TIE_C6I=0;
 ECT_TIE_C7I=0;
 
 ECT_TTOV=0;//��ֹ�ڶ�ʱ�����ʱ�����Ƚ϶���
 //ECT_OCPD_OCPD4=0;//��������Ƚ϶��������ڶ�ʱ��ͨ���˿�������
 //ECT_OCPD_OCPD5=0;
 //ECT_OCPD_OCPD6=0;
 //ECT_OCPD_OCPD7=0;
 
  /*���벶׽��ؼĴ�������*/ 
  
 ECT_TIOS_IOS0=0;//ͨ��0����Ϊ���벶׽����
 ECT_TIOS_IOS1=0;
 ECT_TIOS_IOS2=0;
 ECT_TIOS_IOS3=0;
 
 ECT_TCTL4=0xff;//ǰ�ĸ�ͨ����׽�����غ��½���
 ECT_ICOVW_NOVW0=0; //ͨ��0�Ĳ�׽�Ĵ������Ա�����
                    //���ĳλNOVWx=0���µĲ�׽�¼���ת�ƶ�������ʱ�����۲�׽�򱣳ּĴ����Ƿ�Ϊ�գ���ֱ�ӱ����µĽ��
                     //���ĳλNOVWx=1����Ӧ�ļĴ������������ǣ����������ڿհ�״̬����Ȼ������
 ECT_ICOVW_NOVW1=0;
 ECT_ICOVW_NOVW2=0;
 ECT_ICOVW_NOVW3=0;
 
 ECT_ICSYS_SH04=0;
 ECT_ICSYS_SH15=0;//ͨ��1��5����ʹ���Լ��Ķ˿����ţ���������
 ECT_ICSYS_SH26=0;
 ECT_ICSYS_SH37=0; 
 
 ECT_TIE_C0I=1;//ͨ��0�����벶׽�ж����� 
 ECT_TIE_C1I=1; 
 ECT_TIE_C2I=1; 
 ECT_TIE_C3I=1;  
    
 ECT_TSCR1_TEN=1;  //�����������ж�ʱ��
                                             
}




/*********************��ʱ����*********************/
 void delayms(uint ms) //��ʱ
 {
  uint i,j;
  for(i=0;i<ms;i++)
     for(j=0;j<2000;j++);    
}

/*********************��ʱ����*********************/
 void delayus(uint us) //��ʱ
 {
  uint a,b;
  for(a=0;a<us;a++)
     for(b=0;b<2;b++);    
}



/********************���ڽ����жϷ��������ж�����20********************/

#pragma CODE_SEG NON_BANKED//���·���Ƿ�ҳ��
void interrupt 20 SCI_GETDATA(void) 
{
  if(SCI0SR1_RDRF==1) //���������ݼĴ����Ƿ���ܵ�����
  {
                   
    data[count]=SCI0DRL;
    if(data[count]>0x63) data[count]=0x63;//��֤��λ��ÿ���������������С��100��������100ʱ��ȡ99��ʮ����99��ʾΪʮ�����ƾ���0x63��
    count++;
          
    if(count>11)    //���11�������Ƿ�ȫ���������
    {
      count=0;
      flag=1;
    }
                  
  }
}

/********************ECT��ʱ������жϷ��������ж�����16********************/

 void interrupt 16 OVERFLOW_ECT(void) 
 {
  ECT_TFLG2_TOF=1;//����жϱ�ʶ   ���ô��
  single++;
 }



/********************���벶׽�жϷ��������ж�����8��9��10��11  ͨ��0��1��2��3*******************/

 void interrupt 8 CAPTURE0_ISR(void) 
{
  ECT_TFLG1_C0F=1;//����жϱ�ʶ   ���ñ����
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT0==1)   
    {   
      n=0; 
      PORTA_PA2=1;  //ѡ�׿���
      ECT_TIE_C4I=1;//����ͨ��4�ж�ʹ��
      ECT_TC4=ECT_TCNT+10;//�����ô�MOTEC�����źŵ�������������ʱ���200us
    }  

    if(PTT_PTT0==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA2=0;  //ѡ�׹ر�
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  

void interrupt 9 CAPTURE1_ISR(void) 
{
  ECT_TFLG1_C1F=1;//����жϱ�ʶ   ���ñ����
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT1==1)   
    {   
      n=0; 
      PORTA_PA3=1;  //ѡ�׿���
      ECT_TIE_C4I=1;//����ͨ��4�ж�ʹ��
      ECT_TC4=ECT_TCNT+10;//�����ô�MOTEC�����źŵ�������������ʱ���200us
    }  

    if(PTT_PTT1==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA3=0;  //ѡ�׹ر�
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  
 
void interrupt 10 CAPTURE2_ISR(void) 
{
  ECT_TFLG1_C2F=1;//����жϱ�ʶ   ���ñ����
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT2==1)   
    {   
      n=0; 
      PORTA_PA4=1;  //ѡ�׿���
      ECT_TIE_C4I=1;//����ͨ��4�ж�ʹ��
      ECT_TC4=ECT_TCNT+10;//�����ô�MOTEC�����źŵ�������������ʱ���200us
    }  

    if(PTT_PTT2==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA4=0;  //ѡ�׹ر�
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  
   
void interrupt 11 CAPTURE3_ISR(void) 
{
  ECT_TFLG1_C3F=1;//����жϱ�ʶ   ���ñ����
  
  if((times>0)&&(times<2))
  {
    if(PTT_PTT3==1)   
    {   
      n=0; 
      PORTA_PA5=1;  //ѡ�׿���
      ECT_TIE_C4I=1;//����ͨ��4�ж�ʹ��
      ECT_TC4=ECT_TCNT+10;//�����ô�MOTEC�����źŵ�������������ʱ���200us
    }  

    if(PTT_PTT3==0) 
    {
      PORTA_PA0=0;
      delayus(dischargetime);
      PORTA_PA0=1;
      delayus(70);
      PORTA_PA5=0;  //ѡ�׹ر�
    }
  }
  else 
  {
    ECT_TIE_C4I=0;
  }
}  
 

/*********************����Ƚ��жϷ��������ж�����12��13  ͨ��4��5*********************/

 void interrupt 12 COMPARE4_ISR(void)   //����жϺ���
{
 ECT_TFLG1_C4F=1;//��־λд1���� 
 //ECT_TIE_C0I=0;//��ֹͨ��0���벶׽�ж�ʹ�� 
  
 switch(n) 
 {
   case 0:
          
          PORTA_PA1=0;
          ECT_TC4=ECT_TC4+chargetime; //���ó��ʱ��60us��ECTʱ��Ƶ��Ϊ1MHz������ÿ��1us����һ�Σ���chargetime��ֵ������0������λʱchargetime�ᵱ�����ֵ65536������ͬ���������
          break;
           
   case 1:
          PORTA_PA1=1;
          ECT_TIE_C4I=0;//��ֹͨ��4�ж�ʹ��
          break;
          
                         
   default:break;                  
             
  }   
  n++;   
}       

/*

 void interrupt 13 COMPARE5_ISR(void)    //�ŵ��жϺ���
{
 ECT_TFLG1_C5F=1;//��־λд1����
 //ECT_TIE_C1I=0;//��ֹͨ��1���벶׽�ж�ʹ�� 
 //if(dischargetime>180) dischargetime=150;
  
 switch(m) 
 {          
   case 0:
          PORTA_PA0=0;
          ECT_TC5=ECT_TC5+dischargetime; //���÷ŵ�ʱ��60us��ECTʱ��Ƶ��Ϊ1MHz������ÿ��1us����һ��)
          break;
               
   case 1:
          PORTA_PA0=1; 
          ECT_TIE_C5I=0;//��ֹͨ��5�ж�ʹ��
          break;
                                            
   default:break;                  
             
  }   
  m++;   
}      

*/

#pragma CODE_SEG DEFAULT//���·����ҳ��


void main(void) 
{
  /* put your own code here */
  
  Busclock_Init();//��������ʱ�����ú���
  SCI_Init(); //����SCI��ʼ������ 
  ECT_Init(); //����ECT��ʼ������
  
  DDRT_DDRT0=0;//����T0��Ϊ��������
  DDRT_DDRT1=0;//����T1��Ϊ��������
  DDRT_DDRT2=0;//����T2��Ϊ��������
  DDRT_DDRT3=0;//����T3��Ϊ��������
 // PTT_PTT1=0;//����T1�����Ϊ�͵�ƽ
  
  DDRA_DDRA0=1;//����A0��Ϊ���
  PORTA_PA0=1;//����A0�����Ϊ�ߵ�ƽ
  
  DDRA_DDRA1=1;//����A1��Ϊ���
  PORTA_PA1=1;//����A1�����Ϊ�ߵ�ƽ
  
  DDRA_DDRA2=1;//����A2��Ϊ���
  PORTA_PA2=0;//����A2�����Ϊ�͵�ƽ
  
  DDRA_DDRA3=1;//����A3��Ϊ���
  PORTA_PA3=0;//����A3�����Ϊ�͵�ƽ
  
  DDRA_DDRA4=1;//����A4��Ϊ���
  PORTA_PA4=0;//����A4�����Ϊ�͵�ƽ
  
  DDRA_DDRA5=1;//����A5��Ϊ���
  PORTA_PA5=0;//����A5�����Ϊ�͵�ƽ
  
 
  
	EnableInterrupts; //��ȫ���ж�
  delayms(10);
  
  for(;;)
  {
    _FEED_COP(); /* feeds the dog */
    
     if(flag==1)
   {
    flag=0;
    
    if((data[0]==0x0a)&&(data[11]==0x0a)) //������������趨ָ���һ�����ݺ����һ��������ָ���룬�����⵽����������ʮ��������10��ʮ������0x0a������֤����������������������趨����
    {
      injectionwidth1=data[1]*100+data[2]; //��������ֱ�ֵ��5������
      injectionwidth2=data[3]*100+data[4];//����100��ԭ������Ϊ����λ���н�ǧ��ʮ���ֿ�����ģ���ΪECTģ���ʱ��Ϊ1MHz��ÿ��1us����һ��
      injectionwidth3=data[5]*100+data[6];
      injectionwidth4=data[7]*100+data[8];
      injectionwidth5=data[9]*100+data[10];
      
      if(injectionwidth1==0) injectionwidth1=5;
      if(injectionwidth2==0) injectionwidth2=5;
      if(injectionwidth3==0) injectionwidth3=5;
      if(injectionwidth4==0) injectionwidth4=5;
      if(injectionwidth5==0) injectionwidth5=5; //�趨�������Сֵ�����Ϊ0����Ƭ��������65536����
      
      
    
      
      SCI0DRL=data[0];  //��������������趨ָ�������λ������data[1]
      while(!SCI0SR1_TC) 
      {
       ;
      } 
     
    }
    
    
     if((data[0]==0x14)&&(data[11]==0x14))  //������ͼ���趨ָ���һ�����ݺ����һ��������ָ���룬�����⵽����������ʮ��������20��ʮ������0x14������֤����������������ͼ��ʱ���趨����
    {
      period1=data[1]*100+data[2];  //��������ֱ����ֵ��5������ ,16λ������������65535us
      period2=data[3]*100+data[4];  
      period3=data[5]*100+data[6];
      period4=data[7]*100+data[8];
      period5=data[9]*100+data[10];
      
      if(period1==0) period1=5;
      if(period2==0) period2=5;
      if(period3==0) period3=5;
      if(period4==0) period4=5;
      if(period5==0) period5=5; //�趨�������Сֵ�����Ϊ0����Ƭ��������65536����
      
            
/*    shuzu[0]=period1;
      shuzu[1]=period2;
      shuzu[2]=period3;
      shuzu[3]=period4; 
      shuzu[4]=period5;  
      min=shuzu[0];
      for(c=0;c<=4;c++) 
      {
       if(min>=shuzu[c]) min=shuzu[c];     //�����õ���С�����ͼ�������������ŵ�ʱ��ʱʹ��
      }  */
      
      
      SCI0DRL=data[0];   //��������ͼ���趨ָ�����λ������data[2]
      while(!SCI0SR1_TC) //�ȴ��������
      {
       ;
      }                                                                                                         
      
     
    }
    
    
    
     if((data[0]==0x1e)&&(data[11]==0x1e))   //����ŵ�ʱ�估�����趨ָ���һ�����ݺ����һ��������ָ���룬�����⵽����������ʮ��������30��ʮ������0x1e������֤������������ǳ�ŵ�ʱ�估�����趨����
    {
      chargetime=data[1]*100+data[2];  //��������ֱ����ֵ��3������
      dischargetime=data[3]*100+data[4];//����100��ԭ������Ϊ����λ���н�ǧ��ʮ���ֿ�����ģ����Ǽ�ʱ�õ�ECTģ���ʱ��Ϊ1����ÿ��1us����һ�Ρ�������times������Ϊ������ʱ�õ�
      times=data[5];
      if(chargetime>=0x82) chargetime=0x82;//�趨�����ʱ��130us
      if(chargetime==0) chargetime=3; //�趨��С���ʱ�䣬����Ϊ0������Ƭ������65536����
      
      
      if(dischargetime==0) dischargetime=3; //�趨��С�ŵ�ʱ�䣬����Ϊ0������Ƭ������65536����
      
     
     
      SCI0DRL=data[0];      //����ǳ�ŵ�ʱ�估�����趨ָ�������λ������data[3]
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
