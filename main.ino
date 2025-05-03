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
            - System overview with any constraints like operating temperature, power reqs, etc.
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
            - Commits will be reviewed and assessed based on contributions from *all* members

    ADDED:
        Timer functions
        Serial functions
        LCD handling
        RTC module functions
        ADC and water level functions
    NEED
        Air temperature and humidity
        Fan motor functions
        Vent stepper motor functions
        Main
*/

//Library includes
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <RTClib.h>
#include <DHT.h>
#include <Stepper.h>

//Initialize macro definitions
#define DISABLED 0
#define IDLE 1
#define RUNNING 2
#define ERROR 3
#define RDA 0x80
#define TBE 0x20
#define DHT11_PIN 10


//Initialize global registers
    //serial
volatile unsigned char  *UCSR0A     = (unsigned char *) 0xC0;   //USB, no pin-out
volatile unsigned char  *UCSR0B     = (unsigned char *) 0xC1;
volatile unsigned char  *UCSR0C     = (unsigned char *) 0xC2;
volatile unsigned int   *UBRR0      = (unsigned int *)  0xC4;
volatile unsigned char  *UDR0       = (unsigned char *) 0xC6;
    //ADC
volatile unsigned char  *ADMUX      = (unsigned char*)  0x7C;   //port f:0 (pin A0)
volatile unsigned char  *ADCSRB     = (unsigned char*)  0x7B;
volatile unsigned char  *ADCSRA     = (unsigned char*)  0x7A;
volatile unsigned int   *ADC_DATA   = (unsigned int*)   0x78;
    //timer
volatile unsigned char  *TIFR1      = (unsigned char*)  0x36;   //no pin-out
volatile unsigned char  *TIMSK1     = (unsigned char*)  0x6F;
volatile unsigned char  *TCCR1A     = (unsigned char*)  0x80;
volatile unsigned char  *TCCR1B     = (unsigned char*)  0x81;
volatile unsigned char  *TCCR1C     = (unsigned char*)  0x82;
volatile unsigned int   *TCNT1      = (unsigned int*)   0x84;
    //Vent
volatile unsigned char  *PORTH      = (unsigned char*)  0x102;  //digital 6 - 9 (port h:3-6)
volatile unsigned char  *DDRH       = (unsigned char*)  0x101;  //uses library
volatile unsigned char  *PINH       = (unsigned char*)  0x100;
volatile unsigned char  *PORTC      = (unsigned char*)  0x28;   //stepper motor buttons:
volatile unsigned char  *DDRC       = (unsigned char*)  0x27;       //digital 37 (port c:0) 
volatile unsigned char  *PINC       = (unsigned char*)  0x26;       //digital 36 (port c:1)
    //Motor
volatile unsigned char  *PORTL      = (unsigned char*)  0x10B;  //motor: digital 49 (port l:0)
volatile unsigned char  *DDRL       = (unsigned char*)  0x10A;
volatile unsigned char  *PINL       = (unsigned char*)  0x109;
    //buttons and LEDs
volatile unsigned char  *PORTA      = (unsigned char*)  0x22;   //on-off: digital 22 (port a:0)
volatile unsigned char  *DDRA       = (unsigned char*)  0x21;   //reset: digital 23 (port a:1)
volatile unsigned char  *PINA       = (unsigned char*)  0x20;   //LEDS: digital 26-29 (port a:4-7)
//other pins in use:
    //display: digital 2, 3, 4, 5, 11, 12
    //RTC:     digital 20, 21
    //DHT11:   digital 10


//Initialize global variables
int state = DISABLED;
    //timer
unsigned long FCPU = 16000000;          //CPU frequency of ATMEGA is 16 MHz
double clk_period = 0.0000000625;       //Period of 1 clock cycle for CPU
unsigned int currentTicks = 65535;      //set TCNT1 to (this - ticks needed)
unsigned char timer_running = 0;
    //LCD
int right = 0;
int up = 0;
int dir1 = 0;
int dir2 = 0;
const int RS = 11;
const int EN = 12;
const int D4 = 2;
const int D5 = 3;
const int D6 = 4;
const int D7 = 5;
LiquidCrystal lcd( RS, EN, D4, D5, D6, D7 );
int lcd_x_min = 0;
int lcd_x_max = 15;
int lcd_y_min = 0;
int lcd_y_max = 1;
    //RTC
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    //humidity/temp sensor
DHT dht11( DHT11_PIN, DHT11 );
const long interval = 60000;        //60 second interval for humidity sensor
unsigned long previous_millis = 0;
float humidity = 0.0;
float temperature = 0.0;
    //fan
float temperature_threshold = 25.0;
int water_low_threshold = 100;

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
void timer_setup();
void timer_start( unsigned int ticks );
void timer_stop();
    //LCD
void display_update();
    //RTC
void time_to_serial();
    //DHT11
void readHumiTemp( float* humi, float* temp );

    //vent stepper motor
Stepper ventStepper(12, 3, 4);


//Arduino init function
void setup() {
    adc_init();                                 //initialize ADC
    timer_setup();                              //initialize timer
    U0init(9600);                               //initialize serial

    ventStepper.setSpeed(1);
    int ventCount = 0;


    //set vent button pin to INPUT
//    *ddr_k &= 0xFB;                        //pin name and registers have changed 
    //enable pull-up resistor on vent button
//    *port_k |= 0x04;                       //pin name and registers have changed

    lcd.begin( lcd_x_max + 1, lcd_y_max + 1 );  //initialize LCD
    rtc.begin();                                //initialize RTC
    dht11.begin();                              //initialize DHT11
}

//Arduino loop function
void loop() {
    unsigned long current_millis = millis();
    if ( current_millis - previous_millis >= interval ) {
        previous_millis = current_millis;
        if ( state != DISABLED ) {
            readHumiTemp( &humidity, &temperature );
        }
    }

    // vent moving loop
    if (*PINC & 0x04) {                    //pin name and registers have changed
        //if vent button's pin is high and its not at the highest point
        if(ventCount < 6){
            if( timer_running == 0){
                timer_start(5000);
            }
            ventStepper.step(1);
        }

    }

    if (*DDRC & 0x04) {                    //pin name and registers have changed
        //if vent button's pin is high and its not at the lowest point
        if(ventCount > 1){
            if( timer_running == 0){
                timer_start(5000);
            }
            ventStepper.step(-1);
        }

    }

    // vent stopping loop
    if (!(*pin_k & 0x04)) {                 //pin name and registers have changed
        //if vent button's pin is not high
        if( timer_running == 1){
            timer_stop();
            time_to_serial();
        }
    }


    // motor moving loop
    if (/*motor pin is high*/) {
        //if motors pin is high
        if( timer_running == 0){
            timer_start(5000);
        }
    }

    // motor stopping loop
    if (/*motor pin no high*/) {
        //if motors pin is not high
        if( timer_running == 1){
            timer_stop();
            time_to_serial();
        }
    }

    //update display
    display_update();
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
    void timer_setup();
    Initializes registers to use the timer with interrupts
*/
void timer_setup() {
    *TCCR1A = 0x00;
    *TCCR1B = 0x00;
    *TCCR1C = 0x00;
    *TIFR1 |= 0x01;
    *TIMSK1 |= 0x01;
}

/*
    void timer_start( unsigned int ticks );
    Starts the timer for the specified number of ticks
*/
void timer_start( unsigned int ticks ) {
    currentTicks = (unsigned int) ticks;
    *TCCR1B &= 0xF8;            //stop current timer
    *TCNT1 = (unsigned int) (65535 - (unsigned long) (currentTicks) );
    *TCCR1C &= 0x1F;
    *TCCR1B |= 0x01;            //start timer
    timer_running = 1;
}

/*
    void timer_stop();
    Stops the timer and resets any overflow state
*/
void timer_stop() {
    *TCCR1B &= 0xF8;
    if ( ( *TIFR1 & 0x01 ) == 1 ) {
        *TIFR1 |= 0x01;
    }
    timer_running = 0;
    currentTicks = 65535;
}

/*
    void time_to_serial()
    Prints a string containing the current timestamp to the serial monitor one character at a time
*/
void time_to_serial() {
    DateTime now = rtc.now();
    char date_string[] = now.toString( "DDD, DD MMM YYYY hh:mm:ss" );
    int n = sizeof( date_string ) / sizeof( date_string[0] );
    for (int i = 0; i < len; i++) {
        U0putchar( date_string[i] );
    }
}

/*
    void display_update()
    Clears the LCD display and redraws information on it
*/
void display_update() {
    lcd.clear();
    //place any items that need to be displayed on the LCD here
    if ( state != DISABLED && state != ERROR ) {
        lcd.setCursor( 0, 0 );
        lcd.print( "T: " );
        lcd.print( temperature );
        lcd.print( " H: " );
        lcd.print( humidity );
    }
    if ( state == ERROR ) {
        lcd.setCursor( 0, 0 );
        lcd.print( "Water level is" );
        lcd.setCursor( 0, 1 );
        lcd.print( "too low. ");
    }
}

/*
    void readHumidity( float* humi, float* temp )
    Reads the current humidity and temperature and passes them back by reference
*/
void readHumiTemp( float* humi, float* temp ) {
    *humi = dht11.readHumidity();
    *temp = dht11.readTemperature();
    if ( isnan(*humi) || isnan(*temp) ) {
        char message[] = "Could not read from DHT11 sensor\n";
        int n = sizeof( message ) / sizeof( message[0] );
        for( int i = 0; i < n; i++ ) {
            U0putchar( message[i] );
        }
    }
}


//Interrupt service routines

/*
    ISR( TIMER1_OVF_vect );
    Timer overflow interrupt service routine.
*/
ISR( TIMER1_OVF_vect ) {
    *TCCR1B &= 0xF8;          //stop current timer

    // from lab 8, performs timer restart for currentTicks. change as needed.
    *TCNT1 = (unsigned int) (65535 - (unsigned long) (currentTicks) );
    *TCCR1C &= 0x1F;
    *TCCR1B |= 0x01;
    
    // add functions to perform when timer overflows here
}
