/*
* AlphaController.c
*
* Created: 08.10.2020 11:02:15
* Author : Yll Kryeziu
*/
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

//-----------------------alpha angle percentage arrays modes 3, 4, 5-------------------------------------
int algoA[101] = {0};
int algoB[101] = {0};
int algoC[101] = {0};

double fireAngleTable [101]= {20000, 19800, 19600, 19400, 19200, 19000, 18800, 18600, 18400, 18200, 18000, 17800, 17600, 17400, 17200, 17000, 16800, 16600, 16400, 16200, 16000, 15800, 15600, 15400, 15200, 15000, 14800, 14600, 14400, 14200, 14000, 13800, 13600, 13400, 13200, 13000, 12800, 12600, 12400, 12200, 12000, 11800, 11600, 11400, 11200, 11000, 10800, 10600, 10400, 10200, 10000, 9800, 9600, 9400, 9200, 9000, 8800, 8600, 8400, 8200, 8000, 7800, 7600, 7400, 7200, 7000, 6800, 6600, 6400, 6200, 6000, 5800, 5600, 5400, 5200, 5000, 4800, 4600, 4400, 4200, 4000, 3800, 3600, 3400, 3200, 3000, 2800, 2600, 2400, 2200, 2000, 1800, 1600, 1400, 1200, 1000, 800, 600, 400, 200, 0};
double fireAngleTableV [101]= {20000, 19500, 19200, 19000, 18700, 18500, 18400, 18200, 18000, 17800, 17700, 17500, 17400, 17200, 17100, 16900, 16800, 16700, 16500, 16400, 16300, 16100, 16000, 15900, 15800, 15600, 15500, 15400, 15300, 15100, 15000, 14900, 14800, 14700, 14500, 14400, 14300, 14200, 14100, 13900, 13800, 13700, 13600, 13500, 13400, 13200, 13100, 13000, 12900, 12800, 12600, 12500, 12400, 12300, 12200, 12000, 11900, 11800, 11700, 11500, 11400, 11300, 11200, 11000, 10900, 10800, 10600, 10500, 10400, 10200, 10100, 10000, 9800, 9700, 9500, 9400, 9200, 9100, 8900, 8700, 8600, 8400, 8200, 8100, 7900, 7700, 7500, 7300, 7100, 6800, 6600, 6300, 6100, 5800, 5500, 5100, 4700, 4300, 3700, 2900, 0};
double fireAngleTableP [101]= {20000, 17700, 17100, 16600, 16300, 16000, 15700, 15400, 15200, 15000, 14800, 14600, 14500, 14300, 14100, 14000, 13800, 13700, 13500, 13400, 13300, 13100, 13000, 12900, 12800, 12600, 12500, 12400, 12300, 12200, 12100, 12000, 11900, 11700, 11600, 11500, 11400, 11300, 11200, 11100, 11000, 10900, 10800, 10700, 10600, 10500, 10400, 10300, 10200, 10100, 10000, 9900, 9800, 9700, 9600, 9500, 9400, 9300, 9200, 9100, 9000, 8900, 8800, 8700, 8600, 8500, 8400, 8300, 8100, 8000, 7900, 7800, 7700, 7600, 7500, 7400, 7200, 7100, 7000, 6900, 6700, 6600, 6500, 6300, 6200, 6000, 5900, 5700, 5500, 5400, 5200, 5000, 4800, 4600, 4300, 4000, 3700, 3400, 2900, 2300, 0};

int netzT = 10000;

//-----------------------control values---------------------------
volatile int mode = 1;
volatile int ctrlVal1 = 0;
volatile int ctrlVal2 = 0;
volatile int ctrlVal3 = 0;

//-----------------------variables for timing and synchronisation--------
volatile int intCounter = 0;
volatile int intCounterPhaseA = 0;
volatile int intCounterPhaseB = 0;
volatile int burstCounter = 0;
volatile int burstCounterA = 0;
volatile int burstCounterB = 0;
volatile int timerCounter = 0;
volatile int timerCounterA = 0;
volatile int timerCounterB = 0;
volatile int bufferIdx = 0;
volatile int uartFlag = 0;
volatile int shiftFlag = 0;
volatile int algoFlag = 1;
volatile char uartBuffer;

char input[50];

//function declarations
void SSR1on();
void SSR1off();
void SSR2on();
void SSR2off();
void SSR3Aon();
void SSR3Aoff();
void SSR3Bon();
void SSR3Boff();
void SSR3Con();
void SSR3Coff();

void Init_Int0();
void Init_Timer_0();
void Init_Timer_1();
void USART_Init();
unsigned char uart_getc();
void alphaAlgo(int* array, int percent);
void phaseShiftAngle();
void phaseShiftBurst();
void phaseShiftAlgo();
void uart_sendc(char c);

int main(void) {
    Init_Int0();
    Init_Timer_0();
    Init_Timer_1();
    USART_Init();
    sei();
    DDRB = 0xFF;
    while(1) {
        if (intCounter == 200) {
            intCounter = 0;
            burstCounter = 0;
            uart_sendc('r'); //Request Control Values
        }
        if(uartFlag == 1 && uartBuffer != '#') {
            input[bufferIdx] = uartBuffer;
            bufferIdx++;
            uartFlag = 0;
        } else if (uartFlag == 1 && uartBuffer == '#') {
            bufferIdx++;
            input[bufferIdx] = '\0';
            char* token = strtok(input, "-");
            mode = atoi(token);
            token = strtok(0, "-");
            ctrlVal1 = atoi(token);
            token = strtok(0, "-");
            ctrlVal2 = atoi(token);
            token = strtok(0, "-");
            ctrlVal3 = atoi(token);
            bufferIdx = 0;
            uartFlag = 0;
            algoFlag = 1;
        }
        switch(mode) {
        case 1: //permanent LOW	//WORKING
            SSR1off();
            SSR2off();
            SSR3Aoff();
            SSR3Boff();
            SSR3Coff();
            break;
        case 2: //permanent HIGH //WORKING
            SSR1on();
            SSR2on();
            SSR3Aon();
            SSR3Bon();
            SSR3Con();
            break;
        case 3: //uncorrected phase angle //WORKING
            if(TCNT1 > fireAngleTable[ctrlVal1]) {
                SSR1on();
            } else {
                SSR1off();
            }
            if(TCNT1 > fireAngleTable[ctrlVal2]) {
                SSR2on();
            } else {
                SSR2off();
            }
            if(TCNT1 > fireAngleTable[ctrlVal3]) {
                SSR3Aon();
            } else {
                SSR3Aoff();
            }
            phaseShiftAngle();
            if(timerCounterA > fireAngleTable[ctrlVal3]) {
                SSR3Bon();
            } else {
                SSR3Boff();
            }
            if(timerCounterB > fireAngleTable[ctrlVal3]) {
                SSR3Con();
            } else {
                SSR3Coff();
            }
            break;
        case 4: //phase angle effective voltage //WORKING
            if(TCNT1 > fireAngleTableV[ctrlVal1]) {
                SSR1on();
            } else {
                SSR1off();
            }
            if(TCNT1 > fireAngleTableV[ctrlVal2]) {
                SSR2on();
            } else {
                SSR2off();
            }
            if(TCNT1 > fireAngleTableV[ctrlVal3]) {
                SSR3Aon();
            } else {
                SSR3Aoff();
            }
            phaseShiftAngle();
            if(timerCounterA > fireAngleTableV[ctrlVal3]) {
                SSR3Bon();
            } else {
                SSR3Boff();
            }
            if(timerCounterB > fireAngleTableV[ctrlVal3]) {
                SSR3Con();
            } else {
                SSR3Coff();
            }
            break;
        case 5: //phase angle power	//WORKING
            if(TCNT1 > fireAngleTableP[ctrlVal1]) {
                SSR1on();
            } else {
                SSR1off();
            }
            if(TCNT1 > fireAngleTableP[ctrlVal2]) {
                SSR2on();
            } else {
                SSR2off();
            }
            if(TCNT1 > fireAngleTableP[ctrlVal3]) {
                SSR3Aon();
            } else {
                SSR3Aoff();
            }
            phaseShiftAngle();
            if(timerCounterA > fireAngleTableP[ctrlVal3]) {
                SSR3Bon();
            } else {
                SSR3Boff();
            }
            if(timerCounterB > fireAngleTableP[ctrlVal3]) {
                SSR3Con();
            } else {
                SSR3Coff();
            }
            break;
        case 6: //burst fire low momentum //WORKING
            if(intCounter+1 > 2*ctrlVal1) {
                SSR1off();
            } else {
                SSR1on();
            }
            if(intCounter+1 > 2*ctrlVal2) {
                SSR2off();
            } else {
                SSR2on();
            }
            if(intCounter+1 > 2*ctrlVal3) {
                SSR3Aoff();
            } else {
                SSR3Aon();
            }
            if(intCounterPhaseA+1 > 2*ctrlVal3) {
                SSR3Boff();
            } else {
                SSR3Bon();
            }
            if(intCounterPhaseB+1 > 2*ctrlVal3) {
                SSR3Coff();
            } else {
                SSR3Con();
            }
            break;
        case 7: //burst fire high momentum
            if(algoFlag == 1) {
                alphaAlgo(algoA, ctrlVal1);
                alphaAlgo(algoB, ctrlVal2);
                alphaAlgo(algoC, ctrlVal3);
                algoFlag = 0;
            }
            if(algoA[burstCounter]==1) {
                SSR1on();
            } else {
                SSR1off();
            }
            if(algoB[burstCounter]==1) {
                SSR2on();
            } else {
                SSR2off();
            }
            if(algoC[burstCounter]==1) {
                SSR3Aon();
            } else {
                SSR3Aoff();
            }
            if(algoC[burstCounterA]==1) {
                SSR3Bon();
            } else {
                SSR3Boff();
            }
            if(algoC[burstCounterB]==1) {
                SSR3Con();
            } else {
                SSR3Coff();
            }
        }
    }
}
void alphaAlgo(int* array, int percent) {
    int diffpercent = 100-percent;
    int algoCounter = 0;
    int control = -(100-percent);
    int oldControl = 0;
    while(algoCounter < 100) {
        if(oldControl < 0) {
            array[algoCounter] = 0;
        } else {
            array[algoCounter] = 1;
        }
        oldControl = control;
        if(control >= 0) {
            control -= diffpercent;
        } else {
            control += percent;
        }
        algoCounter++;
    }
}
void phaseShiftAngle() {
    if(TCNT1+6667 <= 20000) {
        timerCounterA = TCNT1+6667;
    } else {
        timerCounterA = TCNT1-13333;
    }
    if(TCNT1+13333 <= 20000) {
        timerCounterB = TCNT1+13333;
    } else {
        timerCounterB = TCNT1-6667;
    }
}
ISR (INT0_vect) {
    TCNT1 = 0;
    TCNT0 = 0;
    shiftFlag = 1 - shiftFlag;
    if(shiftFlag == 1) {
        OCR0A = 52;
    } else {
        OCR0A = 104;
    }
    intCounter++;
    if((intCounter%2==0)&&(intCounter!=0)) {
        burstCounter++;
    }
}
ISR(USART0_RX_vect) {
    uartBuffer = uart_getc();
    uartFlag = 1;
}
ISR (TIMER0_COMPA_vect) {
    if(shiftFlag == 0) {
        intCounterPhaseA = intCounter;
        burstCounterA = burstCounter;
    } else {
        intCounterPhaseB = intCounter;
        burstCounterB = burstCounter;
        OCR0A = 255;
    }
}
unsigned char uart_getc() {
    while (!(UCSR0A & (1<<RXC0))) {
    }
    return UDR0;
}
void uart_sendc(char c) {
    while (!(UCSR0A & (1<<UDRE0))) {
    }
    UDR0 = c;
}
void SSR1on () {
    PORTB |= (1<<PB0);		//set PB0 HIGH
}
void SSR1off () {
    PORTB &= ~(1<<PB0);		//clear PB0 aka.  set PB0 LOW
}
void SSR2on () {
    PORTB |= (1<<PB1);
}
void SSR2off () {
    PORTB &= ~(1<<PB1);
}
void SSR3Aon () {
    PORTB |= (1<<PB2);
}
void SSR3Aoff () {
    PORTB &= ~(1<<PB2);
}
void SSR3Bon () {
    PORTB |= (1<<PB3);
}
void SSR3Boff () {
    PORTB &= ~(1<<PB3);
}
void SSR3Con () {
    PORTB |= (1<<PB4);
}
void SSR3Coff () {
    PORTB &= ~(1<<PB4);
}
void Init_Int0() {
    EIMSK |= (1<<INT0);
    EICRA |= (1<<ISC00);
}
void Init_Timer_0() {
    TCCR0A = (1<<WGM01);			//ctc mode
    TCCR0B |= (1<<CS00)|(1<<CS02);	//prescaler 8
    OCR0A = 103;					//fill OCR0A with 99 for 50us angle steps
    TIMSK0 |= (1<<OCIE1A);				//enable compare interrupt
}
void Init_Timer_1() {
    TCCR1A |= 0;   				//normal mode
    TCCR1B |= (1<<CS11);		//prescaler 8
    TIMSK1 |= (1<<TOIE1);		//enable overflow interrupt
}
void USART_Init() {
    /* Set baud rate */
    UBRR0 = 25;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    /* Set frame format: 8data, 1stop bit */
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
    UCSR0C &= ~(1<<USBS0);
}