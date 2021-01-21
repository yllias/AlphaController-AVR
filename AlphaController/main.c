/*
 * AlphaController.c
 *
 * Created: 08.10.2020 11:02:15
 * Author : Yll Kryeziu
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

//-----------------------------------------alpha angle percentage-------------------------------------
double fireAngleTable [101]={1, 0.99, 0.98, 0.97, 0.96, 0.95, 0.94, 0.93, 0.92, 0.91, 0.9, 0.89, 0.88, 0.87, 0.86, 0.85, 0.84, 0.83, 0.82, 0.81, 0.8, 0.79, 0.78, 0.77, 0.76, 0.75, 0.74, 0.73, 0.72, 0.71, 0.7, 0.69, 0.68, 0.67, 0.66, 0.65, 0.64, 0.63, 0.62, 0.61, 0.6, 0.59, 0.58, 0.57, 0.56, 0.55, 0.54, 0.53, 0.52, 0.51, 0.5, 0.49, 0.48, 0.47, 0.46, 0.45, 0.44, 0.43, 0.42, 0.41, 0.4, 0.39, 0.38, 0.37, 0.36, 0.35, 0.34, 0.33, 0.32, 0.31, 0.3, 0.29, 0.28, 0.27, 0.26, 0.25, 0.24, 0.23, 0.22, 0.21, 0.2, 0.19, 0.18, 0.17, 0.16, 0.15, 0.14, 0.13, 0.12, 0.11, 0.1, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0};
double fireAngleTableV [101]={1, 0.975, 0.96, 0.95, 0.935, 0.925, 0.92, 0.91, 0.9, 0.89, 0.885, 0.875, 0.87, 0.86, 0.855, 0.845, 0.84, 0.835, 0.825, 0.82, 0.815, 0.805, 0.8, 0.795, 0.79, 0.78, 0.775, 0.77, 0.765, 0.755, 0.75, 0.745, 0.74, 0.735, 0.725, 0.72, 0.715, 0.71, 0.705, 0.695, 0.69, 0.685, 0.68, 0.675, 0.67, 0.66, 0.655, 0.65, 0.645, 0.64, 0.63, 0.625, 0.62, 0.615, 0.61, 0.6, 0.595, 0.59, 0.585, 0.575, 0.57, 0.565, 0.56, 0.55, 0.545, 0.54, 0.53, 0.525, 0.52, 0.51, 0.505, 0.5, 0.49, 0.485, 0.475, 0.47, 0.46, 0.455, 0.445, 0.435, 0.43, 0.42, 0.41, 0.405, 0.395, 0.385, 0.375, 0.365, 0.355, 0.34, 0.33, 0.315, 0.305, 0.29, 0.275, 0.255, 0.235, 0.215, 0.185, 0.145, 0};  
double fireAngleTableP [101]={1, 0.885, 0.855, 0.83, 0.815, 0.8, 0.785, 0.77, 0.76, 0.75, 0.74, 0.73, 0.725, 0.715, 0.705, 0.7, 0.69, 0.685, 0.675, 0.67, 0.665, 0.655, 0.65, 0.645, 0.64, 0.63, 0.625, 0.62, 0.615, 0.61, 0.605, 0.6, 0.595, 0.585, 0.58, 0.575, 0.57, 0.565, 0.56, 0.555, 0.55, 0.545, 0.54, 0.535, 0.53, 0.525, 0.52, 0.515, 0.51, 0.505, 0.5, 0.495, 0.49, 0.485, 0.48, 0.475, 0.47, 0.465, 0.46, 0.455, 0.45, 0.445, 0.44, 0.435, 0.43, 0.425, 0.42, 0.415, 0.405, 0.4, 0.395, 0.39, 0.385, 0.38, 0.375, 0.37, 0.36, 0.355, 0.35, 0.345, 0.335, 0.33, 0.325, 0.315, 0.31, 0.3, 0.295, 0.285, 0.275, 0.27, 0.26, 0.25, 0.24, 0.23, 0.215, 0.2, 0.185, 0.17, 0.145, 0.115, 0};

int netzT = 10000;

int mode = 0x00;
int steuerWert = 0;
int steuerWert2 = 0;
int steuerWert3 = 0;
int burstCounter = 0;
int intCounter = 1;
int timerCounter = 0;
int pinStatus = 0;
int alphaUs;
int clockCnt;
double alphaRad;

void SSRon();
void SSRoff();
void Init_Int0();
void Init_Timer_0();
void getControlVals();


int main(void)
{
	sei();
    Init_Int0();
	Init_Timer_0();
	DDRB = 0xFF;
	
	while(1){
		switch(mode){
			case 0x00: //permanent LOW
				SSRoff();
			break;
			case 0x01: //permanent HIGH
				SSRon();
			break;
			case 0x02: //uncorrected phase angle
				if(timerCounter > (fireAngleTable[steuerWert]*netzT)/50){
					SSRoff();
				}else{
					SSRon();
				}
			break;
			case 0x03: //phase angle power
				if(timerCounter > (fireAngleTableP[steuerWert]*netzT)/50){
					SSRoff();
				}else{
					SSRon();
				}
			break;
			case 0x04: //phase angle effective voltage
				if(timerCounter > (fireAngleTableV[steuerWert]*netzT)/50){
					SSRoff();
				}else{
					SSRon();
				}
			break;
			case 0x05: //burst fire low momentum 
				if(burstCounter > steuerWert){
					SSRoff();
				}else{
					SSRon();
				}
			break;
			case 0x06: //burst fire high momentum
				if(pinStatus){
					SSRon();
				}else{
					SSRoff();
				}
			break;
			
		}
		
	}
}

ISR (INT0_vect){
		if (intCounter == 200){
			intCounter = 0;
			getControlVals();
		}
		intCounter++;
		burstCounter = (intCounter+1)/2;
	timerCounter = 0;
	
}

ISR (TIMER0_COMPA_vect){
	timerCounter++;
}

void Init_Int0(){
	EIMSK |= (1<<0);
	EICRA |= (1<<ISC00);
	intCounter = 1;
	burstCounter = 0;
}

void Init_Timer_0(){
	TCCR0A = (1<<WGM01);	//ctc mode
	TCCR0B |= (1<<CS01);	//prescaler 8
	OCR0A = 0x63;			//fill OCR0A with 99 for 50us angle steps
	TIMSK0 |= (1<<1);		//enable compare interrupt
}

void SSRon (){
	PORTB |= (1<<PB0);		//set PB0 HIGH
}

void SSRoff (){
	PORTB &= ~(1<<PB0);		//clear PB0 aka.  set PB0 LOW
}

void USART_Init()
{
	/* Set baud rate */
	UBRR0 = 103;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0C &= ~(1<<USBS0);
}

unsigned char USART_Receive()
{
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)))
	{
		
		
	}
	/* Get and return received data from buffer */
	return UDR0;
}

void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!( UCSR0A & (1<<UDRE0)))
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
