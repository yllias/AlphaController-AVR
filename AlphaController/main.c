/*
* AlphaController.c
*
* Created: 08.10.2020 11:02:15
* Author : Yll Kryeziu
*/
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

//-----------------------alpha angle percentage arrays modes 3, 4, 5-------------------------------------
double fireAngleTable [101]= {1, 0.99, 0.98, 0.97, 0.96, 0.95, 0.94, 0.93, 0.92, 0.91, 0.9, 0.89, 0.88, 0.87, 0.86, 0.85, 0.84, 0.83, 0.82, 0.81, 0.8, 0.79, 0.78, 0.77, 0.76, 0.75, 0.74, 0.73, 0.72, 0.71, 0.7, 0.69, 0.68, 0.67, 0.66, 0.65, 0.64, 0.63, 0.62, 0.61, 0.6, 0.59, 0.58, 0.57, 0.56, 0.55, 0.54, 0.53, 0.52, 0.51, 0.5, 0.49, 0.48, 0.47, 0.46, 0.45, 0.44, 0.43, 0.42, 0.41, 0.4, 0.39, 0.38, 0.37, 0.36, 0.35, 0.34, 0.33, 0.32, 0.31, 0.3, 0.29, 0.28, 0.27, 0.26, 0.25, 0.24, 0.23, 0.22, 0.21, 0.2, 0.19, 0.18, 0.17, 0.16, 0.15, 0.14, 0.13, 0.12, 0.11, 0.1, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0};
double fireAngleTableV [101]= {1, 0.975, 0.96, 0.95, 0.935, 0.925, 0.92, 0.91, 0.9, 0.89, 0.885, 0.875, 0.87, 0.86, 0.855, 0.845, 0.84, 0.835, 0.825, 0.82, 0.815, 0.805, 0.8, 0.795, 0.79, 0.78, 0.775, 0.77, 0.765, 0.755, 0.75, 0.745, 0.74, 0.735, 0.725, 0.72, 0.715, 0.71, 0.705, 0.695, 0.69, 0.685, 0.68, 0.675, 0.67, 0.66, 0.655, 0.65, 0.645, 0.64, 0.63, 0.625, 0.62, 0.615, 0.61, 0.6, 0.595, 0.59, 0.585, 0.575, 0.57, 0.565, 0.56, 0.55, 0.545, 0.54, 0.53, 0.525, 0.52, 0.51, 0.505, 0.5, 0.49, 0.485, 0.475, 0.47, 0.46, 0.455, 0.445, 0.435, 0.43, 0.42, 0.41, 0.405, 0.395, 0.385, 0.375, 0.365, 0.355, 0.34, 0.33, 0.315, 0.305, 0.29, 0.275, 0.255, 0.235, 0.215, 0.185, 0.145, 0};
double fireAngleTableP [101]= {1, 0.885, 0.855, 0.83, 0.815, 0.8, 0.785, 0.77, 0.76, 0.75, 0.74, 0.73, 0.725, 0.715, 0.705, 0.7, 0.69, 0.685, 0.675, 0.67, 0.665, 0.655, 0.65, 0.645, 0.64, 0.63, 0.625, 0.62, 0.615, 0.61, 0.605, 0.6, 0.595, 0.585, 0.58, 0.575, 0.57, 0.565, 0.56, 0.555, 0.55, 0.545, 0.54, 0.535, 0.53, 0.525, 0.52, 0.515, 0.51, 0.505, 0.5, 0.495, 0.49, 0.485, 0.48, 0.475, 0.47, 0.465, 0.46, 0.455, 0.45, 0.445, 0.44, 0.435, 0.43, 0.425, 0.42, 0.415, 0.405, 0.4, 0.395, 0.39, 0.385, 0.38, 0.375, 0.37, 0.36, 0.355, 0.35, 0.345, 0.335, 0.33, 0.325, 0.315, 0.31, 0.3, 0.295, 0.285, 0.275, 0.27, 0.26, 0.25, 0.24, 0.23, 0.215, 0.2, 0.185, 0.17, 0.145, 0.115, 0};

int netzT = 10000;

//-----------------------control values---------------------------
volatile int mode = 6;
volatile int ctrlVal1 = 20;
volatile int ctrlVal2 = 20;
volatile int ctrlVal3 = 20;
volatile int ctrlAdc = 0;

//-----------------------variables for timing and synchronisation--------
volatile int intCounter = 0;
volatile int intCounterPhaseA = 67;
volatile int intCounterPhaseB = 133;
volatile int timerCounter = 0;
volatile int timerCounterA = 0;
volatile int timerCounterB = 0;

//-----------------------variables for mode 7 control
int lNumber, sNumber;
int burstCh1 [5]= {0, 0, 0, 0, 0}; //{sTime, sNumber, lTime, lNumber, rhythm}
int burstCh2 [5]= {0, 0, 0, 0, 0};
int burstCh3 [5]= {0, 0, 0, 0, 0};
char delimiter[] = "-";
volatile char input[50];

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
void USART_Init();
unsigned char uart_getc();
void uart_gets( char* Buffer, uint8_t MaxLen, char endChar);
void requestControlVals();
void getAdcVals();
void calcBurstVals();
int uart_sendc(unsigned char c);
void uart_sends(char *s);


int main(void) {
    sei();
    Init_Int0();
    Init_Timer_0();
    USART_Init();
    DDRB = 0xFF;
    while(1) {
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
            if(timerCounter+1 > (fireAngleTable[ctrlVal1]*netzT)/50) {
                SSR1on();
            } else {
                SSR1off();
            }
            break;
        case 4: //phase angle power	//WORKING
            if(timerCounter+1 > (fireAngleTableP[ctrlVal1]*netzT)/50) {
                SSR1on();
            } else {
                SSR1off();
            }
            break;
        case 5: //phase angle effective voltage //WORKING
            if(timerCounter+1 > (fireAngleTableV[ctrlVal1]*netzT)/50) {
                SSR1on();
            } else {
                SSR1off();
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
            if(intCounter+1 > 2*ctrlVal1) {
                SSR1on();
            } else {
                SSR1off();
            }
            break;
        }
    }
}
ISR (INT0_vect) {
    timerCounter = 0;
    TCNT0 = 0;
    intCounter++;
    //intCounterPhaseA++;
    //intCounterPhaseB++;
    if (intCounter == 200) {
        intCounter = 0;
        //uart_sends("REQ"); //Request Control Values
        //if(ctrlAdc == 1){
        //	getAdcVals();
        //}
        //if(mode == 7){
        //calcBurstVals();
        //}
    }
    if (intCounterPhaseA == 200) {
        intCounterPhaseA = 0;
    }
    if (intCounterPhaseB == 200) {
        intCounterPhaseB = 0;
    }
}
ISR(USART0_RX_vect) { //Wenn empfangen->wird das ausgeführt
    /*uart_gets(input, sizeof(input),'#');
    //received STRING EX. "1-0-100-099-098"
    char* token = strtok(input, "-");
    mode = atoi(token);
    token = strtok(0, "-");
    ctrlAdc = atoi(token);
    token = strtok(0, "-");
    ctrlVal1 = atoi(token);
    token = strtok(0, "-");
    ctrlVal2 = atoi(token);
    token = strtok(0, "-");
    ctrlVal3 = atoi(token);*/
    ctrlVal1 = (uart_getc()-'0')*10;
    ctrlVal2 = (uart_getc()-'0')*10;
}
ISR (TIMER0_COMPA_vect) {
    timerCounter++;
}
void calcBurstVals(int steuerWert, int channel) {
    float quot;
    float hundert = 100.00;
    int sPercent, sTime, lPercent, lTime;
    int rhythm;
    quot = roundf((hundert/steuerWert)*100)/100;
    sPercent = 100-(quot-floor(quot))*100;
    lPercent = 100-sPercent;
    sTime = floor(quot);
    if(lPercent != 0) {
        lTime = sTime + 1;
    } else {
        lTime = 0;
    }
    sNumber = round((sPercent * steuerWert)/100);
    lNumber = round((lPercent * steuerWert)/100);
    //rhythm 0 = S, 1 = L
    if(lNumber>sNumber) {
        rhythm = 1;
    } else {
        rhythm = 0;
    }
}
unsigned char uart_getc() {
    /* Wait for data to be received */
    while (!(UCSR0A & (1<<RXC0))) {
    }
    /* Get and return received data from buffer */
    return UDR0;
}
void uart_gets( char* Buffer, uint8_t MaxLen, char endChar ) {
    uint8_t NextChar;
    uint8_t StringLen = 0;
    NextChar = uart_getc();         // Warte auf und empfange das nächste Zeichen
    // Sammle solange Zeichen, bis:
    // * entweder das String Ende Zeichen kam
    // * oder das aufnehmende Array voll ist
    while( NextChar != endChar && StringLen < MaxLen - 1 ) {
        *Buffer++ = NextChar;
        StringLen++;
        NextChar = uart_getc();
    }
    // Noch ein '\0' anhängen um einen Standard
    // C-String daraus zu machen
    *Buffer = '\0';
}
int uart_sendc(unsigned char c) {
    while (!(UCSR0A & (1<<UDRE0))) { /* warten bis Senden moeglich */
    }
    UDR0 = c;                      /* sende Zeichen */
    return 0;
}
/* puts ist unabhaengig vom Controllertyp */
void uart_sends (char *s) {
    while (*s) {
        /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
        uart_sendc(*s);
        s++;
    }
}
void SSR1on () {
    PORTB |= (1<<PB0);		//set PB0 HIGH
}
void SSR1off () {
    PORTB &= ~(1<<PB0);		//clear PB0 aka.  set PB0 LOW
}
void SSR2on () {
    PORTB |= (1<<PB1);		//set PB0 HIGH
}
void SSR2off () {
    PORTB &= ~(1<<PB1);		//clear PB0 aka.  set PB0 LOW
}
void SSR3Aon () {
    PORTB |= (1<<PB2);		//set PB0 HIGH
}
void SSR3Aoff () {
    PORTB &= ~(1<<PB2);		//clear PB0 aka.  set PB0 LOW
}
void SSR3Bon () {
    PORTB |= (1<<PB3);		//set PB0 HIGH
}
void SSR3Boff () {
    PORTB &= ~(1<<PB3);		//clear PB0 aka.  set PB0 LOW
}
void SSR3Con () {
    PORTB |= (1<<PB4);		//set PB0 HIGH
}
void SSR3Coff () {
    PORTB &= ~(1<<PB4);		//clear PB0 aka.  set PB0 LOW
}
void Init_Int0() {
    EIMSK |= (1<<0);
    EICRA |= (1<<ISC00);
}
void Init_Timer_0() {
    TCCR0A = (1<<WGM01);	//ctc mode
    TCCR0B |= (1<<CS01);	//prescaler 8
    OCR0A = 0x63;			//fill OCR0A with 99 for 50us angle steps
    TIMSK0 |= (1<<1);		//enable compare interrupt
}
void USART_Init() {
    /* Set baud rate */
    UBRR0 = 103;
    /* Enable receiver and transmitter */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0);
    /* Set frame format: 8data, 1stop bit */
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
    UCSR0C &= ~(1<<USBS0);
}