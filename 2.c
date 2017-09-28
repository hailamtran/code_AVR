#include <mega8.h>
#include <glcd.h>
#include <stdlib.h>
#include <delay.h>
// Font used for displaying text
// on the graphic display
#include <font5x7.h>

void inso(int a) {
    char c[10];
    int dem = 0,i;
    
    while(a) {
        c[dem] = (a%10)+48;
        a=a/10;
        dem++;
    }
    
    glcd_moveto(36,0);
    for(i=dem-1; i>=0; i--) {
        glcd_putchar(c[i]);
    }
     
}

void chay() {
    int j;
    for(j = 80; j>0; j--) {   
            glcd_clear();
            glcd_line(j,43,j,50);
            glcd_line(j-1,43,j-1,50);
            glcd_line(j-2,43,j-2,50);
        
    }
}

void main(void)
{

int a,i,res=0,dem = 0;
GLCDINIT_t glcd_init_data;

DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

TCCR0=(0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;

TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

ASSR=0<<AS2;
TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<TOIE0);
MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);

UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
SFIOR=(0<<ACME);

ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

glcd_init_data.font=font5x7;
glcd_init_data.readxmem=NULL;
glcd_init_data.writexmem=NULL;
glcd_init_data.temp_coef=188;
glcd_init_data.bias=3;
glcd_init_data.vlcd=47;

glcd_init(&glcd_init_data);

DDRD.1 = 0;
PORTD.1 = 1;

while (1)
      {    
      chay();
      /*
        glcd_moveto(0,0);
        glcd_outtext("score:0");
        inso(dem);
        
        if(PIND.1 == 0) {
        
            if(res == 1) 
            res = 0, dem++;
            
            for(i= 42; i>=30; i--) {
                delay_ms(40);
                glcd_moveto(15,i);
                glcd_outtext("*");     
            }    
            
            for(i= 30; i<=42; i++) {
                delay_ms(40);
                glcd_moveto(15,i);
                glcd_outtext("*");     
            }
        }
        
        else {
            res = 1;
        }  
        
        */               
      }
}

