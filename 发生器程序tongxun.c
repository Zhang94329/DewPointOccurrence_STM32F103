#include<regx52.h>
#include<math.h>
/*流量控制器1-8的测量值*/
sbit AD7705_U2_Rst=P1^0;
sbit AD7705_U2_DIN_DOUT=P1^1;
sbit AD7705_U2_SCLK=P1^2;
sbit AD7705_U2_DRDY=P1^3;
/*压力变送器，温度变送器，湿度变送器，呼吸机温度变送器，预留4路变送器*/
sbit AD7705_U11_Rst=P1^4;
sbit AD7705_U11_DIN_DOUT=P1^5;
sbit AD7705_U11_SCLK=P1^6;
sbit AD7705_U11_DRDY=P1^7;
/*流量控制器1-8的控制量*/
sbit MAX531_U6_DIN=P3^3;
sbit MAX531_U6_SCLK=P2^4;
sbit MAX531_U6_CS=P2^3;
/*IV型测压电磁阀的控制量*/
sbit DCF1389=P0^3;
sbit DCF4567=P0^7;
/*IV型吹洗电磁阀的控制量*/
sbit DCF2=P3^2;
/****74LS138控制8路质量流量输出切换****/
sbit a_bit=P2^5;
sbit b_bit=P2^6;
sbit c_bit=P2^7;
sbit en_bit=P3^5;

unsigned char OutputData[5];/*向串口发送的每位数据*/
unsigned char Input_ch[24];/*IV型 串口接收的每位数据，8个变量*3=24*/
unsigned char ch_No,ch;
unsigned int InputData[8]; /*IV型 接收上位机的8个完整信息*/
unsigned int AD7705_Data;  /*AD转换后的数字量*/
unsigned int controlData[6];  /*IV型 6个流量控制DA值*/
unsigned char CD4051_U1_U13[8]={0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff}; /*U1,U13--CD4051通道切换的地址信息*/
unsigned char CD4051_U9[8]={0x8f,0x9f,0xaf,0xbf,0xcf,0xdf,0xef,0xff};  /*U9--CD4051通道切换的地址信息*/
float templiuliang;		  /*用于计算质量控制器流量的临时变量*/
float flag;				 /*上位机是否开始通讯判断位*/


/***向MAX531_U6和U12写2字节数据***/
void MAX531_U6_Write(unsigned int SetWord)
{
	unsigned char i;
	for(i=0;i<4;i++)
	{
		SetWord=SetWord<<1;
	}

	MAX531_U6_SCLK=1;
	MAX531_U6_CS=0;
	for(i=0;i<12;i++)
	{
		MAX531_U6_SCLK=0;
		if(SetWord&0x8000)
			MAX531_U6_DIN=1;
		else
			MAX531_U6_DIN=0;
		SetWord=SetWord<<1;
		MAX531_U6_SCLK=1;
	}
	MAX531_U6_CS=1;
	MAX531_U6_SCLK=1;
	MAX531_U6_DIN=0;
}
/***串口初始化***/
void UartInit()
{
	SCON=0x50;   	//SCON: serail mode 1, 8-bit UART
	TMOD=0x20;      //TMOD: timer 1, mode 2, 8-bit reload
	PCON=0x00;      //SMOD="1";
	TH1=0xFD;       //Baud:0XFA:4800;0XFD9600 fosc="11".0592MHz
	TL1=0xFD;
	IE=0x90;	//串口中断允许
	TR1=1;          //启动TIMER1
}

/***向AD7705_U2写8位数据***/
void AD7705_U2_Write(unsigned char SetWord)
{
	unsigned char i;
	AD7705_U2_SCLK=1;
	for(i=0;i<8;i++)
	{
		AD7705_U2_SCLK=0;
		if(SetWord&0x80)
			AD7705_U2_DIN_DOUT=1;
		else
			AD7705_U2_DIN_DOUT=0;
		SetWord=SetWord<<1;
		AD7705_U2_SCLK=1;
	}
	AD7705_U2_SCLK=1;
}
/***向AD7705_U11写8位数据***/
void AD7705_U11_Write(unsigned char SetWord)
{
	unsigned char i;
	AD7705_U11_SCLK=1;
	for(i=0;i<8;i++)
	{
		AD7705_U11_SCLK=0;
		if(SetWord&0x80)
			AD7705_U11_DIN_DOUT=1;
		else
			AD7705_U11_DIN_DOUT=0;
		SetWord=SetWord<<1;
		AD7705_U11_SCLK=1;
	}
	AD7705_U11_SCLK=1;
}

/***AD7705_U2设置***/
void AD7705_U2_Set(void)
{	//设置通道1
	AD7705_U2_Write(0x20);
	AD7705_U2_Write(0x0c);		//时钟寄存器
	AD7705_U2_Write(0x10);
	AD7705_U2_Write(0x40);		//设置寄存器，增益为1，双极性
}
/***AD7705_U11设置***/
void AD7705_U11_Set(void)
{	//设置通道1
	AD7705_U11_Write(0x20);
	AD7705_U11_Write(0x0c);		//时钟寄存器
	AD7705_U11_Write(0x10);
	AD7705_U11_Write(0x40);		//设置寄存器，增益为1，双极性
}

/***从AD7705_U2读取16位数据***/
unsigned int AD7705_U2_Read(void)
{
	unsigned char i;
	unsigned int ADData;
	AD7705_U2_Write(0x38);
	while(AD7705_U2_DRDY==1);
	ADData=0;
	AD7705_U2_SCLK=1;
	for(i=0;i<16;i++)
	{
		AD7705_U2_DIN_DOUT=1;
		AD7705_U2_SCLK=0;
		ADData=ADData<<1;
		ADData=(ADData|AD7705_U2_DIN_DOUT);
		AD7705_U2_SCLK=1;
	}
	AD7705_U2_SCLK=1;
	return(ADData);
}
/***从AD7705_U11读取16位数据***/
unsigned int AD7705_U11_Read(void)
{
	unsigned char i;
	unsigned int ADData;
	AD7705_U11_Write(0x38);
	while(AD7705_U11_DRDY==1);
	ADData=0;
	AD7705_U11_SCLK=1;
	for(i=0;i<16;i++)
	{
		AD7705_U11_DIN_DOUT=1;
		AD7705_U11_SCLK=0;
		ADData=ADData<<1;
		ADData=(ADData|AD7705_U11_DIN_DOUT);
		AD7705_U11_SCLK=1;
	}
	AD7705_U11_SCLK=1;
	return(ADData);
}

/***复位AD7705_U2***/
void AD7705_U2_Reset(void)
{
	unsigned char i;
	AD7705_U2_SCLK=1;
	AD7705_U2_DIN_DOUT=1;
	AD7705_U2_Rst=1;
	for(i=0;i<56;i++)
	{
		AD7705_U2_SCLK=0;
		AD7705_U2_DIN_DOUT=1;
		AD7705_U2_SCLK=1;
	}
	AD7705_U2_SCLK=1;
}
/***复位AD7705_U11***/
void AD7705_U11_Reset(void)
{
	unsigned char i;
	AD7705_U11_SCLK=1;
	AD7705_U11_DIN_DOUT=1;
	AD7705_U11_Rst=1;
	for(i=0;i<56;i++)
	{
		AD7705_U11_SCLK=0;
		AD7705_U11_DIN_DOUT=1;
		AD7705_U11_SCLK=1;
	}
	AD7705_U11_SCLK=1;
}

/***发送单个字符***/
void UartSendChar(unsigned char ch)
{
    SBUF=ch;
    while(TI==0);
    TI=0;
}
void delay(unsigned int n)
{unsigned int i,j;
   for(i=0;i<n;i++){j=0;}
}
/***接受、处理字符串***/
void UartSrv()interrupt 4
{
	ch=SBUF;
    if((0x39>=ch)&&(ch>=0x30)) {ch=ch-0x30;}
    if(ch==0x20) {ch=0;}
	if(RI==1)
	{
		RI=0;
		if(ch_No>=24) {ch_No=0;}	//判断接收数据个数
		Input_ch[ch_No]=ch;
		ch_No++;
	}
	flag=0;
}
/***主程序***/
void main(void)
{
  unsigned char i,tempp2,tempp0;
 	for(i=0;i<24;i++)
      {Input_ch[i]=0;}/*检测值清0*/
	ch_No=0;
    for(i=0;i<8;i++)
      {InputData[i]=0;}
	for(i=0;i<5;i++)
	  {OutputData[i]=0;}
    UartInit();
    en_bit=0;
	a_bit=0;
	b_bit=0;
	c_bit=0;
   	DCF1389=1; //IV型默认阀断电，通气
	DCF4567=1; //IV型默认阀断电，通气
	DCF2=1;	   //IV型默认阀断电，不通气	
	flag=0;
	while(1)
	{
		   templiuliang=0.0;
		   for(i=0;i<8;i++)
           {InputData[i]=Input_ch[i*3]*100+Input_ch[i*3+1]*10+Input_ch[i*3+2];
		   }
		   if(InputData[0]==222){
		   DCF2=0;
		   DCF1389=1;
		   DCF4567=1;
		   flag=1;}	//IV型吹洗功能 DCF2上电，通气
		   if(InputData[0]==555){
		   DCF1389=0;
		   DCF4567=0;
		   DCF2=1;
		   flag=1;}  //IV型测试压力功能 DCF1389上电，不通气
		   if(InputData[0]==111){
		   DCF2=1;
		   DCF1389=1;
		   DCF4567=1;
		   flag=1;} //IV型测量功能，DCF2断电不通气，其他阀断电通气
		   for(i=2;i<4;i++)
           {templiuliang=(float)(InputData[i]);		  //2.3	 IV型对应0——100ml	毒气流量、毒气浓度流量
            templiuliang=templiuliang*40.94+0.5;
            controlData[i-2]=(int)(templiuliang);		 //0,1
           }
           for(i=4;i<6;i++)			//4.5   IV型对应0-50L	流量1、流量2
           {templiuliang=(float)(InputData[i]);
            templiuliang=templiuliang*8.188+0.5;
            controlData[i-2]=(int)(templiuliang);//2.3
           }
           for(i=6;i<8;i++)					 //6.7	 IV型对应0-100L	  干气、湿气
           {templiuliang=(float)(InputData[i]);
            templiuliang=templiuliang*4.094+0.5;
            controlData[i-2]=(int)(templiuliang);	  //4.5
           }
           /*实现上位机前面板上的8个流量控制*/
		   for(i=0;i<1;i++)//IV型FIC3——毒气流量控制
           {
	            tempp2=P2|0x07;
	            P2=tempp2&CD4051_U1_U13[i];
				MAX531_U6_Write(controlData[i]);
			   	c_bit=0;
				b_bit=0;
				a_bit=0;
				en_bit=1;//允许74LS138工作，选通译码器			 
				delay(26666);		 //	延时26666
				en_bit=0;			 // 保持
	       }
           for(i=1;i<2;i++)//IV型FIC4——毒气浓度流量控制
           {
	            tempp2=P2|0x07;
	            P2=tempp2&CD4051_U1_U13[i];
				MAX531_U6_Write(controlData[i]);
			   	c_bit=0;
				b_bit=0;
				a_bit=1;
				en_bit=1;//允许74LS138工作，选通译码器			 
				delay(26666);		 //	延时26666
				en_bit=0;			 // 保持
	       }
		   for(i=2;i<3;i++)//IV型FIC5——流量1控制
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=0;
			b_bit=1;
			a_bit=0;
			en_bit=1;//允许74LS138工作，选通译码器			 
			delay(26666);		 //	延时26666
			en_bit=0;			 // 保持
	       }

		   for(i=3;i<4;i++)//IV型FIC6——流量2控制
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=0;
			b_bit=1;
			a_bit=1;
			en_bit=1;//允许74LS138工作，选通译码器			 
			delay(26666);		 //	延时26666
			en_bit=0;			  // 保持
	       }
		   for(i=4;i<5;i++)//IV型FIC1——干气流量控制
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=1;
			b_bit=0;
			a_bit=0;
			en_bit=1;//允许74LS138工作，选通译码器			 
			delay(26666);		 //	延时26666
			en_bit=0;			 // 保持
	       }
		   for(i=5;i<6;i++)//IV型FIC2——湿气流量控制
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=1;
			b_bit=0;
			a_bit=1;
			en_bit=1;//允许74LS138工作，选通译码器		
			delay(26666);		 //	延时26666
			en_bit=0;			 // 保持
	       }
	    while(flag) { 
	    for(i=0;i<6;i++)		 //IV型J1-J6六路流量采集并发送
	    {
	    tempp0=P0|0x07;
      P0=tempp0&CD4051_U1_U13[i];
      AD7705_U2_Reset();
			AD7705_U2_Set();
			AD7705_Data=AD7705_U2_Read();
			AD7705_U2_Rst=0;
      OutputData[0]=AD7705_Data/10000;
	 		AD7705_Data=AD7705_Data%10000;
			OutputData[1]=AD7705_Data/1000;
      AD7705_Data=AD7705_Data%1000;
			OutputData[2]=AD7705_Data/100;
			AD7705_Data=AD7705_Data%100;
			OutputData[3]=AD7705_Data/10;
			OutputData[4]=AD7705_Data%10;
			UartSendChar(OutputData[0]+0x30);
			UartSendChar(OutputData[1]+0x30);
			UartSendChar(OutputData[2]+0x30);
			UartSendChar(OutputData[3]+0x30);
			UartSendChar(OutputData[4]+0x30);
	  	 }	  
       	for(i=0;i<3;i++)	  //IV型压力、温度、湿度变送器采集并发送
	   	 {
	    	tempp0=P0|0x70;
        	P0=tempp0&CD4051_U9[i];
        	AD7705_U11_Reset();
			AD7705_U11_Set();
			AD7705_Data=AD7705_U11_Read();
			AD7705_U11_Rst=0;
        	OutputData[0]=AD7705_Data/10000;
	 		AD7705_Data=AD7705_Data%10000;
			OutputData[1]=AD7705_Data/1000;
        	AD7705_Data=AD7705_Data%1000;
			OutputData[2]=AD7705_Data/100;
			AD7705_Data=AD7705_Data%100;
			OutputData[3]=AD7705_Data/10;
			OutputData[4]=AD7705_Data%10;
			UartSendChar(OutputData[0]+0x30);
			UartSendChar(OutputData[1]+0x30);
			UartSendChar(OutputData[2]+0x30);
			UartSendChar(OutputData[3]+0x30);
			UartSendChar(OutputData[4]+0x30);
	   	 }
	   }
	 }
}
