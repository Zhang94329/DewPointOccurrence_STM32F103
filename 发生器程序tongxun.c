#include<regx52.h>
#include<math.h>
/*����������1-8�Ĳ���ֵ*/
sbit AD7705_U2_Rst=P1^0;
sbit AD7705_U2_DIN_DOUT=P1^1;
sbit AD7705_U2_SCLK=P1^2;
sbit AD7705_U2_DRDY=P1^3;
/*ѹ�����������¶ȱ�������ʪ�ȱ��������������¶ȱ�������Ԥ��4·������*/
sbit AD7705_U11_Rst=P1^4;
sbit AD7705_U11_DIN_DOUT=P1^5;
sbit AD7705_U11_SCLK=P1^6;
sbit AD7705_U11_DRDY=P1^7;
/*����������1-8�Ŀ�����*/
sbit MAX531_U6_DIN=P3^3;
sbit MAX531_U6_SCLK=P2^4;
sbit MAX531_U6_CS=P2^3;
/*IV�Ͳ�ѹ��ŷ��Ŀ�����*/
sbit DCF1389=P0^3;
sbit DCF4567=P0^7;
/*IV�ʹ�ϴ��ŷ��Ŀ�����*/
sbit DCF2=P3^2;
/****74LS138����8·������������л�****/
sbit a_bit=P2^5;
sbit b_bit=P2^6;
sbit c_bit=P2^7;
sbit en_bit=P3^5;

unsigned char OutputData[5];/*�򴮿ڷ��͵�ÿλ����*/
unsigned char Input_ch[24];/*IV�� ���ڽ��յ�ÿλ���ݣ�8������*3=24*/
unsigned char ch_No,ch;
unsigned int InputData[8]; /*IV�� ������λ����8��������Ϣ*/
unsigned int AD7705_Data;  /*ADת�����������*/
unsigned int controlData[6];  /*IV�� 6����������DAֵ*/
unsigned char CD4051_U1_U13[8]={0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff}; /*U1,U13--CD4051ͨ���л��ĵ�ַ��Ϣ*/
unsigned char CD4051_U9[8]={0x8f,0x9f,0xaf,0xbf,0xcf,0xdf,0xef,0xff};  /*U9--CD4051ͨ���л��ĵ�ַ��Ϣ*/
float templiuliang;		  /*���ڼ���������������������ʱ����*/
float flag;				 /*��λ���Ƿ�ʼͨѶ�ж�λ*/


/***��MAX531_U6��U12д2�ֽ�����***/
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
/***���ڳ�ʼ��***/
void UartInit()
{
	SCON=0x50;   	//SCON: serail mode 1, 8-bit UART
	TMOD=0x20;      //TMOD: timer 1, mode 2, 8-bit reload
	PCON=0x00;      //SMOD="1";
	TH1=0xFD;       //Baud:0XFA:4800;0XFD9600 fosc="11".0592MHz
	TL1=0xFD;
	IE=0x90;	//�����ж�����
	TR1=1;          //����TIMER1
}

/***��AD7705_U2д8λ����***/
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
/***��AD7705_U11д8λ����***/
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

/***AD7705_U2����***/
void AD7705_U2_Set(void)
{	//����ͨ��1
	AD7705_U2_Write(0x20);
	AD7705_U2_Write(0x0c);		//ʱ�ӼĴ���
	AD7705_U2_Write(0x10);
	AD7705_U2_Write(0x40);		//���üĴ���������Ϊ1��˫����
}
/***AD7705_U11����***/
void AD7705_U11_Set(void)
{	//����ͨ��1
	AD7705_U11_Write(0x20);
	AD7705_U11_Write(0x0c);		//ʱ�ӼĴ���
	AD7705_U11_Write(0x10);
	AD7705_U11_Write(0x40);		//���üĴ���������Ϊ1��˫����
}

/***��AD7705_U2��ȡ16λ����***/
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
/***��AD7705_U11��ȡ16λ����***/
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

/***��λAD7705_U2***/
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
/***��λAD7705_U11***/
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

/***���͵����ַ�***/
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
/***���ܡ������ַ���***/
void UartSrv()interrupt 4
{
	ch=SBUF;
    if((0x39>=ch)&&(ch>=0x30)) {ch=ch-0x30;}
    if(ch==0x20) {ch=0;}
	if(RI==1)
	{
		RI=0;
		if(ch_No>=24) {ch_No=0;}	//�жϽ������ݸ���
		Input_ch[ch_No]=ch;
		ch_No++;
	}
	flag=0;
}
/***������***/
void main(void)
{
  unsigned char i,tempp2,tempp0;
 	for(i=0;i<24;i++)
      {Input_ch[i]=0;}/*���ֵ��0*/
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
   	DCF1389=1; //IV��Ĭ�Ϸ��ϵ磬ͨ��
	DCF4567=1; //IV��Ĭ�Ϸ��ϵ磬ͨ��
	DCF2=1;	   //IV��Ĭ�Ϸ��ϵ磬��ͨ��	
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
		   flag=1;}	//IV�ʹ�ϴ���� DCF2�ϵ磬ͨ��
		   if(InputData[0]==555){
		   DCF1389=0;
		   DCF4567=0;
		   DCF2=1;
		   flag=1;}  //IV�Ͳ���ѹ������ DCF1389�ϵ磬��ͨ��
		   if(InputData[0]==111){
		   DCF2=1;
		   DCF1389=1;
		   DCF4567=1;
		   flag=1;} //IV�Ͳ������ܣ�DCF2�ϵ粻ͨ�����������ϵ�ͨ��
		   for(i=2;i<4;i++)
           {templiuliang=(float)(InputData[i]);		  //2.3	 IV�Ͷ�Ӧ0����100ml	��������������Ũ������
            templiuliang=templiuliang*40.94+0.5;
            controlData[i-2]=(int)(templiuliang);		 //0,1
           }
           for(i=4;i<6;i++)			//4.5   IV�Ͷ�Ӧ0-50L	����1������2
           {templiuliang=(float)(InputData[i]);
            templiuliang=templiuliang*8.188+0.5;
            controlData[i-2]=(int)(templiuliang);//2.3
           }
           for(i=6;i<8;i++)					 //6.7	 IV�Ͷ�Ӧ0-100L	  ������ʪ��
           {templiuliang=(float)(InputData[i]);
            templiuliang=templiuliang*4.094+0.5;
            controlData[i-2]=(int)(templiuliang);	  //4.5
           }
           /*ʵ����λ��ǰ����ϵ�8����������*/
		   for(i=0;i<1;i++)//IV��FIC3����������������
           {
	            tempp2=P2|0x07;
	            P2=tempp2&CD4051_U1_U13[i];
				MAX531_U6_Write(controlData[i]);
			   	c_bit=0;
				b_bit=0;
				a_bit=0;
				en_bit=1;//����74LS138������ѡͨ������			 
				delay(26666);		 //	��ʱ26666
				en_bit=0;			 // ����
	       }
           for(i=1;i<2;i++)//IV��FIC4��������Ũ����������
           {
	            tempp2=P2|0x07;
	            P2=tempp2&CD4051_U1_U13[i];
				MAX531_U6_Write(controlData[i]);
			   	c_bit=0;
				b_bit=0;
				a_bit=1;
				en_bit=1;//����74LS138������ѡͨ������			 
				delay(26666);		 //	��ʱ26666
				en_bit=0;			 // ����
	       }
		   for(i=2;i<3;i++)//IV��FIC5��������1����
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=0;
			b_bit=1;
			a_bit=0;
			en_bit=1;//����74LS138������ѡͨ������			 
			delay(26666);		 //	��ʱ26666
			en_bit=0;			 // ����
	       }

		   for(i=3;i<4;i++)//IV��FIC6��������2����
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=0;
			b_bit=1;
			a_bit=1;
			en_bit=1;//����74LS138������ѡͨ������			 
			delay(26666);		 //	��ʱ26666
			en_bit=0;			  // ����
	       }
		   for(i=4;i<5;i++)//IV��FIC1����������������
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=1;
			b_bit=0;
			a_bit=0;
			en_bit=1;//����74LS138������ѡͨ������			 
			delay(26666);		 //	��ʱ26666
			en_bit=0;			 // ����
	       }
		   for(i=5;i<6;i++)//IV��FIC2����ʪ����������
           {
            tempp2=P2|0x07;
            P2=tempp2&CD4051_U1_U13[i];
			MAX531_U6_Write(controlData[i]);
		   	c_bit=1;
			b_bit=0;
			a_bit=1;
			en_bit=1;//����74LS138������ѡͨ������		
			delay(26666);		 //	��ʱ26666
			en_bit=0;			 // ����
	       }
	    while(flag) { 
	    for(i=0;i<6;i++)		 //IV��J1-J6��·�����ɼ�������
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
       	for(i=0;i<3;i++)	  //IV��ѹ�����¶ȡ�ʪ�ȱ������ɼ�������
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
