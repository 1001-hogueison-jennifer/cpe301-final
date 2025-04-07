/*
CPE 301 ~ Final Project

Team members:
    Jennifer Hogueison
    Enekoitz Irujo

Requirements:
    Create an evaporation cooling system
    
    This entails:
    - Monitor and display the current air temperature and humidity on an LCD screen
    - Monitor water levels in a reservoir and print an alert when the level is too low
    - Start or stop a fan's motor based on temperature being too high (start) or low (stop) 
    - Allow vent direction to be adjusted by either buttons or a potentiometer
    - Record the time and date when the motor is started or stopped, and transmit it using USB
    - Record the time and date of any state transition, and transmit it using USB
    - Record the time of any changes to the vent direction, and transmit it using USB

    States:
        - Disabled
            - Yellow LED
            - Fan off
            - Vent direction change disallowed
            Waits:
                - Start -> Idle

        - Idle
            - Green LED
            - Fan off
            - Vent direction change allowed
            - Display temp & humidity on LCD. Update this once per minute
            Waits:
                - Temp > threshold -> Running
                - Water level <= threshold -> Error
                - Stop -> Disabled

        - Running
            - Blue LED
            - Fan on
            - Vent direction change allowed
            - Display temp & humidity on LCD. Update this once per minute
            Waits:
                - Temp <= threshold -> Idle
                - Water level < threshold -> Error
                - Stop -> Disabled

        - Error
            - Red LED
            - Fan off
            - Vent direction change allowed
            - Display "Water level is too low" on LCD.
            Waits:
                - Stop -> Disabled
                - Reset -> Check water level, Idle if and only if > threshold

    Libraries allowed:
        - Stepper motor (for vent direction control)
        - LCD
        - Clock
        - Temperature and humidity sensor

    Libraries disallowed: 
        - ADC library for sampling
        - Serial library
        - pinMode()
        - digitalRead()
        - digitalWrite()
        - delay()
        - analogRead()
        - Any library not explicitly allowed

    Submission details:
        Final report:
            - Detailed description of project including components, states, & functionalities
            - Details and descriptions of every component's functionality
            - System overview with any constraints like operating temperature, power requirements, etc.
            - Circuit image
            - Schematic diagram of circuit
            - System demonstration images
        System demonstration video:
            - Demonstrate all operational states
            - Include narration
            - Upload to GitHub alongside everything else
        GitHub project:
            - Make public before submitting
            - Include link on Canvas
            - Comments should be professional and meaningful, no "asdf" commit comments
            - Commits will be reviewed and assessed based on contributions from *all* team members

*/

//Initialize macro definitions
#define DISABLED 0
#define IDLE 1
#define RUNNING 2
#define ERROR 3

#define RDA 0x80
#define TBE 0x20


//Initialize global registers
    //serial
volatile unsigned char  *UCSR0A     = (unsigned char *) 0x00C0;
volatile unsigned char  *UCSR0B     = (unsigned char *) 0x00C1;
volatile unsigned char  *UCSR0C     = (unsigned char *) 0x00C2;
volatile unsigned int   *UBRR0      = (unsigned int *)  0x00C4;
volatile unsigned char  *UDR0       = (unsigned char *) 0x00C6;
    //ADC
volatile unsigned char  *ADMUX      = (unsigned char*)  0x7C;
volatile unsigned char  *ADCSRB     = (unsigned char*)  0x7B;
volatile unsigned char  *ADCSRA     = (unsigned char*)  0x7A;
volatile unsigned int   *ADC_DATA   = (unsigned int*)   0x78;
    //timer
volatile unsigned char  *TIFR1      = (unsigned char*)  0x36;
volatile unsigned char  *TIMSK1     = (unsigned char*)  0x6F;
volatile unsigned char  *TCCR1A     = (unsigned char*)  0x80;
volatile unsigned char  *TCCR1B     = (unsigned char*)  0x81;
volatile unsigned char  *TCCR1C     = (unsigned char*)  0x82;
volatile unsigned int   *TCNT1      = (unsigned int*)   0x84;
    //GPIO
volatile unsigned char  *PINB       = (unsigned char*)  0x23;
volatile unsigned char  *DDRB       = (unsigned char*)  0x24;
volatile unsigned char  *PORTB      = (unsigned char*)  0x25;


//Initialize global variables
int state = DISABLED;
unsigned long FCPU = 16000000;          //CPU frequency of ATMEGA is 16 MHz
double clk_period = 0.0000000625;       //Period of 1 clock cycle for CPU
unsigned int timer_maxticks = 65536;    //set TCNT1 to (this - ticks needed)


//Function prototypes
    //serial
void U0init( int U0baud );
unsigned char U0kbhit();
unsigned char U0getchar();
void U0putchar( unsigned char U0pdata );
    //ADC
void adc_init();
unsigned int adc_read( unsigned char adc_channel_num );
    //timer
void timer_reset();


//Arduino init function
void setup() {
    U0init(9600);       //initialize serial
    adc_init();         //initialize ADC
}

//Arduino loop function
void loop() {

}


//Function definitions

/*
    void U0init( int U0baud );
    Initializes a serial connection at a specified baud rate.
*/
void U0init( int U0baud ) {
    unsigned int tbaud;
    tbaud = (FCPU / 16 / U0baud - 1);
    *UCSR0A = 0x20;     //set 7:0 to 00100000
    *UCSR0B = 0x18;     //set 7:0 to 00011000
    *UCSR0C = 0x06;     //set 7:0 to 00000110
    *UBRR0  = tbaud;
}

/*
    unsigned char U0kbhit();
    Returns whether or not the RDA bit in UCSR0A is set
*/
unsigned char U0kbhit() {
    return *UCSR0A & RDA;
}

/*
    unsigned char U0getchar();
    Returns a char received over serial in UDR0
*/
unsigned char U0getchar() {
    return *UDR0;
}

/* 
    void U0putchar( unsigned char U0pdata );
    Waits for UCSR0A's TBE bit to be set, then places the char U0pdata
    in the UDR0 register to be sent over serial 
*/
void U0putchar( unsigned char U0pdata ) {
    while ( (*UCSR0A & TBE) == 0 ) {
        //wait
    }
    *UDR0 = U0pdata;
}

/*
    void adc_init();
    Initializes using the ADC to monitor analog data
*/
void adc_init() {
    *ADCSRA |= 0x80;    //set bit 7
    *ADCSRA &= 0xDF;    //clear bit 5
    *ADCSRA &= 0xF7;    //clear bit 3
    *ADCSRA &= 0xF8;    //clear bits 2:0

    *ADCSRB &= 0xF7;    //clear bit 3
    *ADCSRB &= 0xF8;    //clear bits 2:0

    *ADMUX  &= 0x7F;    //clear bit 7
    *ADMUX  |= 0x40;    //set bit 6
    *ADMUX  &= 0xDF;    //clear bit 5
    *ADMUX  &= 0xE0;    //clear bits 4:0
}

/*
    unsigned int adc_read( unsigned char adc_channel_num );
    Clears the current ADC channel, then sets the channel
    Receives data from the ADC and returns it
*/
unsigned int adc_read( unsigned char adc_channel_num ) {
    *ADMUX  &= 0xE0;    //clear bits 4:0
    *ADCSRB &= 0xF7;    //clear bit 3

    //set bits for ADMUX based on selected channel
    switch (adc_channel_num) {
        case '0': {
            *ADMUX |= 0x00;
            break;
        }
        case '1': {
            *ADMUX |= 0x01;
            break;
        }
        case '2': {
            *ADMUX |= 0x02;
            break;
        }
        case '3': {
            *ADMUX |= 0x03;
            break;
        }
        case '4': {
            *ADMUX |= 0x04;
            break;
        }
        case '5': {
            *ADMUX |= 0x05;
            break;
        }
        case '6': {
            *ADMUX |= 0x06;
            break;
        }
        case '7': {
            *ADMUX |= 0x07;
            break;
        }
        default: {
            break;
        }
    }
 
    *ADCSRA |= 0x40;    //set bit 6

    while ( (*ADCSRA & 0x40) != 0 ) {
        //wait
    }

    //check the data register and return it
    unsigned short *portADCDataRegister = (unsigned short *) 0x78;
    unsigned int val = ( *portADCDataRegister & 0x03FF );
    return val;
}

/*
    timer_reset()
    Stops the timer and resets any overflow state
*/
void timer_reset() {
    *TCCR1B &= 0xF8;
    if ( ( *TIFR1 & 0x01 ) == 1 ) {
        *TIFR1 |= 0x01;
    }
}