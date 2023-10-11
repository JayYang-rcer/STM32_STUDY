#include "ioi2c.h"
#include "sys.h"
#include "delay.h"
#include "stdio.h"


//IIC��ʼ��
void IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
}


//ģ��IIC��ʼ�ź�
int IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0; //START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}



//ģ��IIC�����ź� 
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}



//IIC�ȴ�Ӧ���ź�
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 1;  
} 


//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}


//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}



//����һ���ֽ� 
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1; 	  
			delay_us(1);   
			IIC_SCL=1;
			delay_us(1); 
			IIC_SCL=0;	
			delay_us(1);
    }	 
} 


//IICд���ݵ��Ĵ���
//����  addr���豸��ַ��reg���Ĵ�����ַ��len;�ֽ�����data������
//����  ֵ��0���ɹ�д�룻1��û�гɹ�д��
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
	if(!IIC_Start()) return 1;
	
	IIC_Send_Byte(addr << 1 );
	if(!IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	
	for(i=0; i<len; i++)
	{
		IIC_Send_Byte(data[i]);
		if(!IIC_Wait_Ack())
		{
			IIC_Stop();
			return 1;
		}
	}
	
	IIC_Stop();
	return 0;
}


//IIC���Ĵ���������
//������addr���豸��ַ��reg���Ĵ�����ַ��len;�ֽ�����*buf���������ݻ���
//����  ֵ��0���ɹ�������1��û�гɹ�����
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if (!IIC_Start()) return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
	{
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
	
    while (len) 
	{
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
	
    IIC_Stop();
    return 0;
}


u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	 {
			IIC_SCL=0; 
			delay_us(2);
			IIC_SCL=1;
			receive<<=1;
			if(READ_SDA)receive++;   
			delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}


unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
  IIC_Stop();//����һ��ֹͣ����

	return res;
}
 


u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte(dev+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 else                  data[count]=IIC_Read_Byte(0);  //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}



u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
  IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
	 }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}


u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}


u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}


u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}



