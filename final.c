/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Advanced robotics 
Version : 1.0 
Date    : 7/19/2017
Author  : 
Company : 
Comments: 


Chip type               : ATmega16A
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

#include <mega16a.h>
#include <stdlib.h>
#include <stdio.h>
#include <delay.h>

// Graphic Display functions
#include <glcd.h>

// Font used for displaying text
// on the graphic display
#include <font5x7.h>
//----------dinh nghia---------
#define TRIGGER PORTC.6
#define ECHO PINC.7
#define LED_1 PORTB.0
#define LED_2 PORTB.1
#define PWM_1 OCR1B
#define PWM_2 OCR1A
#define DIR_1 PORTD.6
#define DIR_2 PORTD.7
#define motor_1 1
#define motor_2 2
#define run_thuan 0
#define run_nguoc 1
#define in_1 PORTB.3
#define in_2 PORTB.4
#define button_adc PINB.7
#define CT_1 PIND.0
#define CT_2 PINA.7
#define CT_3 PINA.6
#define servo_1 PORTA.4
#define servo_2 PORTA.5
#define BL_Nokia PORTC.5
#define chon_san PORTD.1

// Declare your global variables here
//-------------khai bao bien toan cuc---
int change, count, dem, RC_1,RC_2;
unsigned int en_1, en_2, en_3;
unsigned char k = 0x0;
float distance;
char buff[20];
unsigned int min_adc[4];
unsigned int max_adc[4];
unsigned char ngatu=0,nho=0;
unsigned char set_adc=0;
eeprom unsigned int nguong_adc[4];
interrupt [EXT_INT0] void ext_int0_isr(void)
{
en_1++;
}
// Su dung ngat ngoai thu hai cho encoder banh phai
// External Interrupt 1 service routine
interrupt [EXT_INT1] void ext_int1_isr(void)
{
en_2++;
}

// External Interrupt 2 service routine
// Su dung ngat ngoai thu ba cho encoder tay nang _ cencter
interrupt [EXT_INT2] void ext_int2_isr(void)
{
en_3++;
}
// Timer 0 overflow interrupt service routine
// su dung timer 0 de dieu khien servo
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
// Reinitialize Timer 0 value
TCNT0=0x9C; // chu ky 0.1ms
dem++;
if(dem==200) {dem = 0;} // servo chi hoat dong khi cap xung 20ms vao, dem den 200 thi reset
if(dem<RC_1){servo_1 = 1;} else{servo_1 = 0;}
if(dem<RC_2){servo_2 = 1;} else{servo_2=0;}
}
// Timer2 overflow interrupt service routine
// su dung timer 2 de tinh thoi gian truyen ve cua cam bien sieu am
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
// Reinitialize Timer2 value
TCNT2=0x9C;
if(count>0){count++;}
if(ECHO == 0 && change == 1)
   {
    distance = count*0.1*3.432*5; 
    count = 0;
    change = 0; 
   }
}

// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}
//--------ham doc gia tri ADC cua san thi dau va tinh nguong trung binh
void set_nguong_adc(void)
{unsigned char i=0;
if((button_adc==0)&&(set_adc==0))
    {
        while(button_adc==0)
        { 
            for(i=0;i<4;i++)
            {
             min_adc[i]=read_adc(i);
            }
             sprintf(buff,"%u %u %u %u",min_adc[0],min_adc[1],min_adc[2],min_adc[3]);
             glcd_moveto(0,0);
             glcd_outtext("ADC vach trang");
             glcd_moveto(0,7);
             glcd_outtext(buff);
        }     
        set_adc=1;
} 
if((button_adc==0)&&(set_adc==1))
    {
       while(button_adc==0)
        { 
            for(i=0;i<4;i++)
            {
             max_adc[i]=read_adc(i);
            }
             sprintf(buff,"%u %u %u %u",max_adc[0],max_adc[1],max_adc[2],max_adc[3]);
             glcd_moveto(0,14);
             glcd_outtext("ADC vach den");
             glcd_moveto(0,21);
             glcd_outtext(buff);
        }  
        for(i=0;i<4;i++)
            {
            nguong_adc[i]=((min_adc[i]+max_adc[i]))/2;
            } 
    set_adc=2;
}
if((button_adc==0)&&(set_adc==2))
        {
            sprintf(buff,"%u %u %u %u",nguong_adc[0],nguong_adc[1],nguong_adc[2],nguong_adc[3]);
         glcd_moveto(0,28);
         glcd_outtext("trung binh");
         glcd_moveto(0,35);
         glcd_outtext(buff);   
         delay_ms(1000);
         glcd_clear();
        }
}
//-------------ham xu ly vach trang/den va nhan biet nga tu-----
void read_line(){
    if(read_adc(0) > nguong_adc[0]) k |= 0b0001;
    if(read_adc(1) > nguong_adc[1]) k |= 0b0010;
    if(read_adc(2) > nguong_adc[2]) k |= 0b0100;
    if(read_adc(3) > nguong_adc[3]) k |= 0b1000; 
    
    if((read_adc(0)>nguong_adc[0])&&(read_adc(1)>nguong_adc[1])&&(read_adc(2)>nguong_adc[2])&&(read_adc(3)>nguong_adc[3])&&(nho==0))
        {
        ngatu++;
        nho=1;
        }
    else if((read_adc(0)<nguong_adc[0])||(read_adc(1)<nguong_adc[1])||(read_adc(2)<nguong_adc[2])||(read_adc(3)<nguong_adc[3]))
        {
        nho=0;
        }     
}
//-------------ham dieu khien toc do, chieu quay cua dong co-----------------------
void control_motor(unsigned char motor,unsigned char dir_motor, unsigned char speed)  
{
    switch(motor)
    {
        case 1:
        {   if(dir_motor==0)
            {      
             DIR_1 =  dir_motor;
             PWM_1 = speed;  
             break;
            }
            else
            {
             DIR_1 =  dir_motor;
             PWM_1 =255- speed;  
             break;
            }
        } 
        case 2:
        {         
           
            if(dir_motor==0)
            {      
             DIR_2 =  dir_motor;
             PWM_2 = speed;  
             break;
            }
            else
            {
             DIR_2 =  dir_motor;
             PWM_2 =255- speed;  
             break;
            }
        }    
      
    }
}
//-----------ham do duong-------
void do_line(int kp, int speed_t, int speed_p)
{
    int error; 
    int speed_1=0, speed_2=0;
    read_line();
    if(k==0b0001) error = 3;
    if(k==0b0011) error = 2;
    if(k==0b0010) error = 1;
    if(k==0b0110) error = 0;
    if(k==0b0100) error = -1;
    if(k==0b1100) error = -2;
    if(k==0b1000) error = -3;
    speed_1 = speed_t + kp*error;
    speed_2 = speed_p - kp*error;
    if(speed_1>255){speed_1=255;}
    if(speed_2>255){speed_2=255;}
    if(speed_1<00){speed_1=00;}
    if(speed_2<00){speed_2=00;}
    control_motor(1,0,speed_1);
    control_motor(2,0,speed_2);
    k = 0x0;
}
//-------------ham nang tay len xuong--------
void tay_nang(int direction){
    if(direction == 0){
        in_1 = 0;
        in_2 = 1;
    }else{
        in_1 = 1;
        in_2 = 0;
    }
}
//--------------ham khoi dong cam bien sieu am---------
void set_up_sieu_am(){
  
    TRIGGER = 0;
    delay_us(2);
    TRIGGER = 1;
    delay_us(10);
    TRIGGER = 0;
    while(ECHO == 0);
    count = 1;    
    change = 1;
}

void main(void)
{
// Declare your local variables here
// Variable used to store graphic display
// controller initialization data
GLCDINIT_t glcd_init_data;

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (1<<DDA5) | (1<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=P Bit6=P Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(1<<PORTA7) | (1<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=Out Bit3=Out Bit2=In Bit1=Out Bit0=Out 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (1<<DDB4) | (1<<DDB3) | (0<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=P Bit6=T Bit5=T Bit4=0 Bit3=1 Bit2=T Bit1=0 Bit0=0 
PORTB=(1<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (1<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (1<<DDC6) | (1<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=1 Bit6=1 Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=P Bit0=P 
PORTD=(1<<PORTD7) | (1<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFF
// OC0 output: Disconnected
// Timer Period: 0.1 ms
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (1<<CS01) | (0<<CS00);
TCNT0=0x9C;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Fast PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Non-Inverted PWM
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.256 ms
// Output Pulse(s):
// OC1A Period: 0.256 ms Width: 0 us// OC1B Period: 0.256 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFF
// OC2 output: Disconnected
// Timer Period: 0.1 ms
ASSR=0<<AS2;
TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (1<<CS21) | (0<<CS20);
TCNT2=0x9C;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (1<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (1<<TOIE0);

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Rising Edge
// INT1: On
// INT1 Mode: Rising Edge
// INT2: On
// INT2 Mode: Rising Edge
GICR|=(1<<INT1) | (1<<INT0) | (1<<INT2);
MCUCR=(1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00);
MCUCSR=(1<<ISC2);
GIFR=(1<<INTF1) | (1<<INTF0) | (1<<INTF2);

// USART initialization
// USART disabled
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

// ADC initialization
// ADC Clock frequency: 1000.000 kHz
// ADC Voltage Reference: AREF pin
// ADC Auto Trigger Source: ADC Stopped
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
SFIOR=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Graphic Display Controller initialization
// The PCD8544 connections are specified in the
// Project|Configure|C Compiler|Libraries|Graphic Display menu:
// SDIN - PORTC Bit 3
// SCLK - PORTC Bit 4
// D /C - PORTC Bit 2
// /SCE - PORTC Bit 1
// /RES - PORTC Bit 0

// Specify the current font for displaying text
glcd_init_data.font=font5x7;
// No function is used for reading
// image data from external memory
glcd_init_data.readxmem=NULL;
// No function is used for writing
// image data to external memory
glcd_init_data.writexmem=NULL;
// Set the LCD temperature coefficient
glcd_init_data.temp_coef=PCD8544_DEFAULT_TEMP_COEF;
// Set the LCD bias
glcd_init_data.bias=PCD8544_DEFAULT_BIAS;
// Set the LCD contrast control voltage VLCD
glcd_init_data.vlcd=PCD8544_DEFAULT_VLCD;

glcd_init(&glcd_init_data);

// Global enable interrupts
#asm("sei")
glcd_moveto(0,0);
glcd_outtext("Advanced");
glcd_moveto(35,24);
glcd_outtext("Robotics");
glcd_moveto(25,40);
glcd_outtext("DIY-HUS");
delay_ms(1000);
control_motor(1,1,0); 
control_motor(2,1,0);  
while (1)
      { 
      set_nguong_adc();
      if(chon_san ==0){glcd_moveto(0,0);glcd_outtext("SAN XANH"); LED_1=1;} 
      if(chon_san ==1){glcd_moveto(0,0);glcd_outtext("SAN DO"); LED_2=1;}
      sprintf(buff,"ent=%u enp=%u ",en_1,en_2);
      glcd_moveto(0,0);
      glcd_outtext("encoder trai - phai");
      glcd_moveto(0,8);  
      glcd_outtext(buff);
      sprintf(buff,"en_c=%u ",en_3);
      glcd_moveto(0,16);
      glcd_outtext("encoder giua");
      glcd_moveto(0,24);  
      glcd_outtext(buff);
      if(CT_1==0)
        {
            en_1=0; en_2=0; en_3=0; 
            while((en_1<72)||(en_2<72))      //thu di xem co thang hang khong
            {                      
                control_motor(1,0,150);
                control_motor(2,1,150);
            } 
        }
      if(CT_2==0)
        {
            en_1=0; en_2=0; en_3=0;  
            while(1)
            {do_line(24,100,100);}
        }
      if(CT_3==0)
        {
            en_1=0; en_2=0; en_3=0; 
            while(en_3<72)
            {tay_nang(1);}
        }
      }
}
/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Advanced robotics 
Version : 1.0 
Date    : 7/19/2017
Author  : 
Company : 
Comments: 


Chip type               : ATmega16A
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

#include <mega16a.h>
#include <stdlib.h>
#include <stdio.h>
#include <delay.h>

// Graphic Display functions
#include <glcd.h>

// Font used for displaying text
// on the graphic display
#include <font5x7.h>
//----------dinh nghia---------
#define TRIGGER PORTC.6
#define ECHO PINC.7
#define LED_1 PORTB.0
#define LED_2 PORTB.1
#define PWM_1 OCR1B
#define PWM_2 OCR1A
#define DIR_1 PORTD.6
#define DIR_2 PORTD.7
#define motor_1 1
#define motor_2 2
#define run_thuan 0
#define run_nguoc 1
#define in_1 PORTB.3
#define in_2 PORTB.4
#define button_adc PINB.7
#define CT_1 PIND.0
#define CT_2 PINA.7
#define CT_3 PINA.6
#define servo_1 PORTA.4
#define servo_2 PORTA.5
#define BL_Nokia PORTC.5
#define chon_san PORTD.1

// Declare your global variables here
//-------------khai bao bien toan cuc---
int change, count, dem, RC_1,RC_2;
unsigned int en_1, en_2, en_3;
unsigned char k = 0x0;
float distance;
char buff[20];
unsigned int min_adc[4];
unsigned int max_adc[4];
unsigned char ngatu=0,nho=0;
unsigned char set_adc=0;
eeprom unsigned int nguong_adc[4];
interrupt [EXT_INT0] void ext_int0_isr(void)
{
en_1++;
}
// Su dung ngat ngoai thu hai cho encoder banh phai
// External Interrupt 1 service routine
interrupt [EXT_INT1] void ext_int1_isr(void)
{
en_2++;
}

// External Interrupt 2 service routine
// Su dung ngat ngoai thu ba cho encoder tay nang _ cencter
interrupt [EXT_INT2] void ext_int2_isr(void)
{
en_3++;
}
// Timer 0 overflow interrupt service routine
// su dung timer 0 de dieu khien servo
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
// Reinitialize Timer 0 value
TCNT0=0x9C; // chu ky 0.1ms
dem++;
if(dem==200) {dem = 0;} // servo chi hoat dong khi cap xung 20ms vao, dem den 200 thi reset
if(dem<RC_1){servo_1 = 1;} else{servo_1 = 0;}
if(dem<RC_2){servo_2 = 1;} else{servo_2=0;}
}
// Timer2 overflow interrupt service routine
// su dung timer 2 de tinh thoi gian truyen ve cua cam bien sieu am
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
// Reinitialize Timer2 value
TCNT2=0x9C;
if(count>0){count++;}
if(ECHO == 0 && change == 1)
   {
    distance = count*0.1*3.432*5; 
    count = 0;
    change = 0; 
   }
}

// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}
//--------ham doc gia tri ADC cua san thi dau va tinh nguong trung binh
void set_nguong_adc(void)
{unsigned char i=0;
if((button_adc==0)&&(set_adc==0))
    {
        while(button_adc==0)
        { 
	        for(i=0;i<4;i++)
            {
             min_adc[i]=read_adc(i);
            }
             sprintf(buff,"%u %u %u %u",min_adc[0],min_adc[1],min_adc[2],min_adc[3]);
             glcd_moveto(0,0);
             glcd_outtext("ADC vach trang");
			 glcd_moveto(0,7);
			 glcd_outtext(buff);
        }     
        set_adc=1;
} 
if((button_adc==0)&&(set_adc==1))
    {
       while(button_adc==0)
        { 
            for(i=0;i<4;i++)
            {
             max_adc[i]=read_adc(i);
            }
             sprintf(buff,"%u %u %u %u",max_adc[0],max_adc[1],max_adc[2],max_adc[3]);
             glcd_moveto(0,14);
             glcd_outtext("ADC vach den");
			 glcd_moveto(0,21);
			 glcd_outtext(buff);
        }  
        for(i=0;i<4;i++)
            {
			nguong_adc[i]=((min_adc[i]+max_adc[i]))/2;
			} 
    set_adc=2;
}
if((button_adc==0)&&(set_adc==2))
        {
       	 sprintf(buff,"%u %u %u %u",nguong_adc[0],nguong_adc[1],nguong_adc[2],nguong_adc[3]);
		 glcd_moveto(0,28);
         glcd_outtext("trung binh");
		 glcd_moveto(0,35);
		 glcd_outtext(buff);   
         delay_ms(1000);
         glcd_clear();
	    }
}
//-------------ham xu ly vach trang/den va nhan biet nga tu-----
void read_line(){
    if(read_adc(0) > nguong_adc[0]) k |= 0b0001;
    if(read_adc(1) > nguong_adc[1]) k |= 0b0010;
    if(read_adc(2) > nguong_adc[2]) k |= 0b0100;
    if(read_adc(3) > nguong_adc[3]) k |= 0b1000; 
    
    if((read_adc(0)>nguong_adc[0])&&(read_adc(1)>nguong_adc[1])&&(read_adc(2)>nguong_adc[2])&&(read_adc(3)>nguong_adc[3])&&(nho==0))
        {
        ngatu++;
        nho=1;
        }
    else if((read_adc(0)<nguong_adc[0])||(read_adc(1)<nguong_adc[1])||(read_adc(2)<nguong_adc[2])||(read_adc(3)<nguong_adc[3]))
        {
        nho=0;
        }     
}
//-------------ham dieu khien toc do, chieu quay cua dong co-----------------------
void control_motor(unsigned char motor,unsigned char dir_motor, unsigned char speed)  
{
    switch(motor)
    {
        case 1:
        {   if(dir_motor==0)
            {      
             DIR_1 =  dir_motor;
             PWM_1 = speed;  
             break;
            }
            else
            {
             DIR_1 =  dir_motor;
             PWM_1 =255- speed;  
             break;
            }
        } 
        case 2:
        {         
           
            if(dir_motor==0)
            {      
             DIR_2 =  dir_motor;
             PWM_2 = speed;  
             break;
            }
            else
            {
             DIR_2 =  dir_motor;
             PWM_2 =255- speed;  
             break;
            }
        }    
      
    }
}
//-----------ham do duong-------
void do_line(int kp, int speed_t, int speed_p)
{
    int error; 
    int speed_1=0, speed_2=0;
    read_line();
    if(k==0b0001) error = 3;
    if(k==0b0011) error = 2;
    if(k==0b0010) error = 1;
    if(k==0b0110) error = 0;
    if(k==0b0100) error = -1;
    if(k==0b1100) error = -2;
    if(k==0b1000) error = -3;
    speed_1 = speed_t + kp*error;
    speed_2 = speed_p - kp*error;
	if(speed_1>255){speed_1=255;}
    if(speed_2>255){speed_2=255;}
    if(speed_1<00){speed_1=00;}
    if(speed_2<00){speed_2=00;}
    control_motor(1,0,speed_1);
    control_motor(2,0,speed_2);
    k = 0x0;
}
//-------------ham nang tay len xuong--------
void tay_nang(int direction){
    if(direction == 0){
        in_1 = 0;
        in_2 = 1;
    }else{
        in_1 = 1;
        in_2 = 0;
    }
}
//--------------ham khoi dong cam bien sieu am---------
void set_up_sieu_am(){
  
    TRIGGER = 0;
    delay_us(2);
    TRIGGER = 1;
    delay_us(10);
	TRIGGER = 0;
	while(ECHO == 0);
    count = 1;    
    change = 1;
}

void main(void)
{
// Declare your local variables here
// Variable used to store graphic display
// controller initialization data
GLCDINIT_t glcd_init_data;

// Input/Output Ports initialization
// Port A initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (1<<DDA5) | (1<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=P Bit6=P Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(1<<PORTA7) | (1<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=Out Bit3=Out Bit2=In Bit1=Out Bit0=Out 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (1<<DDB4) | (1<<DDB3) | (0<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=P Bit6=T Bit5=T Bit4=0 Bit3=1 Bit2=T Bit1=0 Bit0=0 
PORTB=(1<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (1<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=In Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC7) | (1<<DDC6) | (1<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit7=T Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=1 Bit6=1 Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=P Bit0=P 
PORTD=(1<<PORTD7) | (1<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (1<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFF
// OC0 output: Disconnected
// Timer Period: 0.1 ms
TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<CS02) | (1<<CS01) | (0<<CS00);
TCNT0=0x9C;
OCR0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Fast PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Non-Inverted PWM
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0.256 ms
// Output Pulse(s):
// OC1A Period: 0.256 ms Width: 0 us// OC1B Period: 0.256 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFF
// OC2 output: Disconnected
// Timer Period: 0.1 ms
ASSR=0<<AS2;
TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (1<<CS21) | (0<<CS20);
TCNT2=0x9C;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (1<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<OCIE0) | (1<<TOIE0);

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Rising Edge
// INT1: On
// INT1 Mode: Rising Edge
// INT2: On
// INT2 Mode: Rising Edge
GICR|=(1<<INT1) | (1<<INT0) | (1<<INT2);
MCUCR=(1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00);
MCUCSR=(1<<ISC2);
GIFR=(1<<INTF1) | (1<<INTF0) | (1<<INTF2);

// USART initialization
// USART disabled
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

// ADC initialization
// ADC Clock frequency: 1000.000 kHz
// ADC Voltage Reference: AREF pin
// ADC Auto Trigger Source: ADC Stopped
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
SFIOR=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Graphic Display Controller initialization
// The PCD8544 connections are specified in the
// Project|Configure|C Compiler|Libraries|Graphic Display menu:
// SDIN - PORTC Bit 3
// SCLK - PORTC Bit 4
// D /C - PORTC Bit 2
// /SCE - PORTC Bit 1
// /RES - PORTC Bit 0

// Specify the current font for displaying text
glcd_init_data.font=font5x7;
// No function is used for reading
// image data from external memory
glcd_init_data.readxmem=NULL;
// No function is used for writing
// image data to external memory
glcd_init_data.writexmem=NULL;
// Set the LCD temperature coefficient
glcd_init_data.temp_coef=PCD8544_DEFAULT_TEMP_COEF;
// Set the LCD bias
glcd_init_data.bias=PCD8544_DEFAULT_BIAS;
// Set the LCD contrast control voltage VLCD
glcd_init_data.vlcd=PCD8544_DEFAULT_VLCD;

glcd_init(&glcd_init_data);

// Global enable interrupts
#asm("sei")
glcd_moveto(0,0);
glcd_outtext("Advanced");
glcd_moveto(35,24);
glcd_outtext("Robotics");
glcd_moveto(25,40);
glcd_outtext("DIY-HUS");
delay_ms(1000);
control_motor(1,1,0); 
control_motor(2,1,0);  
while (1)
      { 
      set_nguong_adc();
      if(chon_san ==0){glcd_moveto(0,0);glcd_outtext("SAN XANH"); LED_1=1;} 
      if(chon_san ==1){glcd_moveto(0,0);glcd_outtext("SAN DO"); LED_2=1;}
      sprintf(buff,"ent=%u enp=%u ",en_1,en_2);
      glcd_moveto(0,0);
      glcd_outtext("encoder trai - phai");
      glcd_moveto(0,8);  
      glcd_outtext(buff);
      sprintf(buff,"en_c=%u ",en_3);
      glcd_moveto(0,16);
      glcd_outtext("encoder giua");
      glcd_moveto(0,24);  
      glcd_outtext(buff);
      if(CT_1==0)
        {
            en_1=0; en_2=0; en_3=0; 
            while((en_1<72)||(en_2<72))      //thu di xem co thang hang khong
            {                      
                control_motor(1,0,150);
                control_motor(2,1,150);
            } 
        }
      if(CT_2==0)
        {
            en_1=0; en_2=0; en_3=0;  
            while(1)
            {do_line(24,100,100);}
        }
      if(CT_3==0)
        {
            en_1=0; en_2=0; en_3=0; 
            while(en_3<72)
            {tay_nang(1);}
        }
      }
}
