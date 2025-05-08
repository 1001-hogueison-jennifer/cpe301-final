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
        Air temperature and humidity
    NEED
        Fan motor functions
        Vent stepper motor functions
        Main
*/

//Library includes
#include <LiquidCrystal.h>
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <Stepper.h>
#include <Adafruit_Sensor.h>

//Initialize macro definitions
#define DISABLED 0
#define IDLE 1
#define RUNNING 2
#define ERROR 3
#define RDA 0x80
#define TBE 0x20
#define DHT11_PIN 42


//Initialize global registers
    //serial
volatile unsigned char  *_UCSR0A     = (unsigned char *) 0x00C0;   //USB, no pin-out
volatile unsigned char  *_UCSR0B     = (unsigned char *) 0x00C1;
volatile unsigned char  *_UCSR0C     = (unsigned char *) 0x00C2;
volatile unsigned int   *_UBRR0      = (unsigned int *)  0x00C4;
volatile unsigned char  *_UDR0       = (unsigned char *) 0x00C6;
    //ADC
volatile unsigned char  *_ADMUX      = (unsigned char*)  0x7C;   //port f:0 (pin A0)
volatile unsigned char  *_ADCSRB     = (unsigned char*)  0x7B;
volatile unsigned char  *_ADCSRA     = (unsigned char*)  0x7A;
volatile unsigned int   *_ADC_DATA   = (unsigned int*)   0x78;
    //timer
volatile unsigned char  *_TIFR1      = (unsigned char*)  0x36;   //no pin-out
volatile unsigned char  *_TIMSK1     = (unsigned char*)  0x6F;
volatile unsigned char  *_TCCR1A     = (unsigned char*)  0x80;
volatile unsigned char  *_TCCR1B     = (unsigned char*)  0x81;
volatile unsigned char  *_TCCR1C     = (unsigned char*)  0x82;
volatile unsigned int   *_TCNT1      = (unsigned int*)   0x84;
    //Vent
//volatile unsigned char  *_PORTH      = (unsigned char*)  0x102;  //digital 6 - 9 (port h:3-6)
//volatile unsigned char  *_DDRH       = (unsigned char*)  0x101;  //uses library
//volatile unsigned char  *_PINH       = (unsigned char*)  0x100;
volatile unsigned char  *_PORTC      = (unsigned char*)  0x28;   //stepper motor buttons:
volatile unsigned char  *_DDRC       = (unsigned char*)  0x27;       //digital 37 (port c:0) 
volatile unsigned char  *_PINC       = (unsigned char*)  0x26;       //digital 36 (port c:1)
    //Fan Motor
volatile unsigned char  *_PORTL      = (unsigned char*)  0x10B;  //motor: enable digital 49 (port l:0)
volatile unsigned char  *_DDRL       = (unsigned char*)  0x10A;    //forward digital 47 (port l:2)
volatile unsigned char  *_PINL       = (unsigned char*)  0x109;    //reverse digital 46 (port l:3)
    //buttons and LEDs
volatile unsigned char  *_PORTA      = (unsigned char*)  0x22;
volatile unsigned char  *_DDRA       = (unsigned char*)  0x21;   //reset: digital 23 (port a:1)
volatile unsigned char  *_PINA       = (unsigned char*)  0x20;   //LEDS: digital 26-29 (port a:4-7)
volatile unsigned char  *_PORTD      = (unsigned char*)  0x2B;   //power button: digital 19 (port d:2)
volatile unsigned char  *_DDRD       = (unsigned char*)  0x2A;
volatile unsigned char  *_PIND       = (unsigned char*)  0x29;
//other pins in use:
    //display: digital 2, 3, 4, 5, 11, 12
    //RTC:     digital 20, 21
    //DHT11:   digital 42


//Initialize global variables
int state = DISABLED;
int tts_flag = 0;
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
const long disp_interval = 1000;           //1 second interval for display refresh rate
unsigned long prev_millis_disp = 0;
    //RTC
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    //humidity/temp sensor
DHT dht11( DHT11_PIN, DHT11 );
const long interval = 6000;        //60 second interval for humidity sensor
unsigned long previous_millis = 0;
float humidity = 0.0;
float temperature = 0.0;
    //water sensor
int adc_channel = 0;
    //fan
float temperature_threshold = 25.0;
int water_low_threshold = 35;
const byte interruptPin = 19;
    //stepper motor
int ventCount = 0;
int steps_per_rev = 2048;
int step_pin1 = 6;
int step_pin2 = 7;
int step_pin3 = 8;
int step_pin4 = 9;


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
    //power button
void power();


//Arduino init function
void setup() {
    U0init(9600);                               //initialize serial
    adc_init();                                 //initialize ADC
    timer_setup();                              //initialize timer
    lcd.begin( lcd_x_max + 1, lcd_y_max + 1 );  //initialize LCD
    dht11.begin();                              //initialize DHT11

    rtc.begin();
    if (!rtc.isrunning()) {
      char message[] = "RTC not running, setting RTC to compilation time and starting it.\n";
      int n = sizeof( message ) / sizeof( message[0] );
      for( int i = 0; i < n-1; i++ ) {
          U0putchar( message[i] );
      }

      rtc.adjust(DateTime(__DATE__, __TIME__));
    }

    int checkPin = digitalPinToInterrupt( interruptPin ) ;
    if (checkPin == -1) {
        char message[] = "Pin doesn't work for interrupts\n";
        int n = sizeof( message ) / sizeof( message[0] );
        for( int i = 0; i < n-1; i++ ) {
            U0putchar( message[i] );
        }
    } else {
        char message[] = "Pin works for interrupts\n";
        int n = sizeof( message ) / sizeof( message[0] );
        for( int i = 0; i < n-1; i++ ) {
            U0putchar( message[i] );
        }
        attachInterrupt(digitalPinToInterrupt( interruptPin ), power, RISING );
    }

    *_DDRA &= 0xFD;                      //set reset button pin to INPUT
    *_PORTA |= 0x02;                     //enable pull-up resistor on reset button
    *_DDRD &= 0xFB;                      //set power button pin to INPUT
    *_PORTD |= 0x04;                     //enable pull-up resistor on power button
    *_DDRL |= 0x01;                      //set fan motor to OUTPUT
    *_DDRA |= 0x10;                      //set yellow LED to OUTPUT
    *_DDRA |= 0x20;                      //set green LED to OUTPUT
    *_DDRA |= 0x40;                      //set blue LED to OUTPUT
    *_DDRA |= 0x80;                      //set red LED to OUTPUT

    *_PORTL |= 0x04;                     //set fan motor to forward
    *_PORTL &= 0xF7;
}

//Arduino loop function
void loop() {
    //update temperature and humidity
    unsigned long current_millis = millis();
    if ( current_millis - previous_millis >= interval ) {
        previous_millis = current_millis;
        if ( state != DISABLED ) {
            readHumiTemp( &humidity, &temperature );
        }
    }

    //vent moving loop
    if (state != DISABLED) {
      if (*_PINC & 0x01 && timer_running == 0) {
        //if vent button's pin is high 
        timer_start(50000);
        Stepper ventStepper(steps_per_rev, step_pin1, step_pin3, step_pin2, step_pin4);
        ventStepper.setSpeed(5);
        ventStepper.step(50);
        tts_flag = 1;
      } else {
        if (*_PINC & 0x02 && timer_running == 0) {
          //if vent button's pin is high 
          timer_start(50000);
          Stepper ventStepper(steps_per_rev, step_pin1, step_pin3, step_pin2, step_pin4);
          ventStepper.setSpeed(5);
          ventStepper.step(-50);
          tts_flag = 1;
        }
      }
    }

    //state machine
    if (state == DISABLED) {
        //ensure fan is stopped
        *_PORTL &= 0xFE;

        *_PORTA |= 0x10;         //drive b:4 high (yellow LED)
        *_PORTA &= 0x1F;         //drive b:5-7 low
    }

    if (state == IDLE) {
        if (temperature > temperature_threshold ) {
            tts_flag = 1;
            state = RUNNING;
            //start the fan
            *_PORTL |= 0x01;

        }
        if ( adc_read(adc_channel) < water_low_threshold) {
            tts_flag = 1;
            state = ERROR;
        }
    
        *_PORTA |= 0x20;         //drive b:5 high (green LED)
        *_PORTA &= 0x2F;         //drive b:4, 6-7 low
    }
    
    if (state == RUNNING) {
        //check for temperature threshold
        if (temperature < temperature_threshold ) {
            tts_flag = 1;
            state = IDLE;
            //stop the fan
            *_PORTL &= 0xFE;
        }
        //check for water low
        if ( adc_read(adc_channel) < water_low_threshold ) {
            tts_flag = 1;
            state = ERROR;
        }
    
        *_PORTA |= 0x40;         //drive b:6 high (blue LED)
        *_PORTA &= 0x4F;         //drive b:4-5, 7 low
    }
    
    if (state == ERROR) {
        //ensure fan is stopped
        *_PORTL &= 0xFE;
    
        //check for reset button
        if ( *_PINA & 0x02 ) {
            if ( adc_read(adc_channel) > water_low_threshold ) {
                tts_flag = 1;
                state = IDLE;
            }
        }
    
        *_PORTA |= 0x80;         //drive b:7 high (red LED)
        *_PORTA &= 0x8F;         //drive b:4-6 low
    }
    
    //write time to serial
    if (tts_flag == 1) {
      tts_flag = 0;
      time_to_serial();
    }

    //update display
    unsigned long curr_millis_disp = millis();
    if ( curr_millis_disp - prev_millis_disp >= disp_interval ) {
        prev_millis_disp = curr_millis_disp;
        display_update();
    }
}


//Function definitions

/*
    void U0init( int U0baud );
    Initializes a serial connection at a specified baud rate.
*/
void U0init( int U0baud ) {
    unsigned int tbaud;
    tbaud = (FCPU / 16 / U0baud) - 1;
    *_UCSR0A = 0x20;     //set 7:0 to 00100000
    *_UCSR0B = 0x18;     //set 7:0 to 00011000
    *_UCSR0C = 0x06;     //set 7:0 to 00000110
    *_UBRR0  = tbaud;
}

/*
    unsigned char U0kbhit();
    Returns whether or not the RDA bit in UCSR0A is set
*/
unsigned char U0kbhit() {
    return *_UCSR0A & RDA;
}

/*
    unsigned char U0getchar();
    Returns a char received over serial in UDR0
*/
unsigned char U0getchar() {
    return *_UDR0;
}

/* 
    void U0putchar( unsigned char U0pdata );
    Waits for UCSR0A's TBE bit to be set, then places the char U0pdata
    in the UDR0 register to be sent over serial 
*/
void U0putchar( unsigned char U0pdata ) {
    while ( (*_UCSR0A & TBE) == 0 ) {
        //wait
    }
    *_UDR0 = U0pdata;
}

/*
    void adc_init();
    Initializes using the ADC to monitor analog data
*/
void adc_init() {
    *_ADCSRA |= 0x80;    //set bit 7
    *_ADCSRA &= 0xDF;    //clear bit 5
    *_ADCSRA &= 0xF7;    //clear bit 3
    *_ADCSRA &= 0xF8;    //clear bits 2:0

    *_ADCSRB &= 0xF7;    //clear bit 3
    *_ADCSRB &= 0xF8;    //clear bits 2:0

    *_ADMUX  &= 0x7F;    //clear bit 7
    *_ADMUX  |= 0x40;    //set bit 6
    *_ADMUX  &= 0xDF;    //clear bit 5
    *_ADMUX  &= 0xE0;    //clear bits 4:0
}

/*
    unsigned int adc_read( unsigned char adc_channel_num );
    Clears the current ADC channel, then sets the channel
    Receives data from the ADC and returns it
*/
unsigned int adc_read( unsigned char adc_channel_num ) {
    *_ADMUX  &= 0xE0;    //clear bits 4:0
    *_ADCSRB &= 0xF7;    //clear bit 3

    //set bits for ADMUX based on selected channel
    switch (adc_channel_num) {
        case '0': {
            *_ADMUX |= 0x00;
            break;
        }
        case '1': {
            *_ADMUX |= 0x01;
            break;
        }
        case '2': {
            *_ADMUX |= 0x02;
            break;
        }
        case '3': {
            *_ADMUX |= 0x03;
            break;
        }
        case '4': {
            *_ADMUX |= 0x04;
            break;
        }
        case '5': {
            *_ADMUX |= 0x05;
            break;
        }
        case '6': {
            *_ADMUX |= 0x06;
            break;
        }
        case '7': {
            *_ADMUX |= 0x07;
            break;
        }
        default: {
            break;
        }
    }
 
    *_ADCSRA |= 0x40;    //set bit 6

    while ( (*_ADCSRA & 0x40) != 0 ) {
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
    *_TCCR1A = 0x00;
    *_TCCR1B = 0x00;
    *_TCCR1C = 0x00;
    *_TIFR1 |= 0x01;
    *_TIMSK1 |= 0x01;
}

/*
    void timer_start( unsigned int ticks );
    Starts the timer for the specified number of ticks
*/
void timer_start( unsigned int ticks ) {
    currentTicks = (unsigned int) ticks;
    *_TCCR1B &= 0xF8;            //stop current timer
    *_TCNT1 = (unsigned int) (65535 - (unsigned long) (currentTicks) );
    *_TCCR1C &= 0x1F;
    *_TCCR1B |= 0x01;            //start timer
    timer_running = 1;
}

/*
    void timer_stop();
    Stops the timer and resets any overflow state
*/
void timer_stop() {
    *_TCCR1B &= 0xF8;
    if ( ( *_TIFR1 & 0x01 ) == 1 ) {
        *_TIFR1 |= 0x01;
    }
    timer_running = 0;
    currentTicks = 65535;
}

/*
    void time_to_serial()
    Prints a string containing the current timestamp to the serial monitor one character at a time
*/
void time_to_serial() {
    char date_string[] = "DD-MM-YYYY hh:mm:ss\n";
    int len = sizeof( date_string ) / sizeof( date_string[0] );
    DateTime current_time = rtc.now();
    strcpy(date_string, current_time.toString( date_string ) );
    for (int i = 0; i < len-1; i++) {
      while ( (*_UCSR0A & TBE) == 0 ) {
        //wait
      }
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
    if ( state != DISABLED) {
      if (state != ERROR ) {
        lcd.setCursor( 0, 0 );
        lcd.print( "T: " );
        lcd.print( temperature );
        lcd.setCursor( 0, 1 );
        lcd.print( "H: " );
        lcd.print( humidity );
      } else {
        lcd.setCursor( 0, 0 );
        lcd.print( "Water level is" );
        lcd.setCursor( 0, 1 );
        lcd.print( "too low. ");
      }
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
        for( int i = 0; i < n-1; i++ ) {
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
    *_TCCR1B &= 0xF8;          //stop current timer
    if ( ( *_TIFR1 & 0x01 ) == 1 ) {
        *_TIFR1 |= 0x01;
    }
    timer_running = 0;
    currentTicks = 65535;
}

/*
    void power();
    Power button interrupt service routine
*/
void power() {
    if (state == DISABLED) {
        state = IDLE;
        tts_flag = 1;
    } else {
        state = DISABLED;
        tts_flag = 1;
    }

}