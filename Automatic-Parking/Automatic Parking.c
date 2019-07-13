/*
 * Automatic_Parking.c
 *
 *  Author: Resham Panth
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"
#include <limits.h>
#include <stdbool.h>
#include "Servo.h"
#define V 16

int point_east = 0; //variable check if robot is pointing in north direction
int point_west = 1;	//variable check if robot is pointing in east direction
int point_north = 2; //variable check if robot is pointing in south direction
int point_south = 3; //variable check if robot is pointing in south direction
int threshold = 10; //Threshold value for line follower
int src=1 , dest; //Source and destination nodes
int prev_node= 24; //Variable to store previous node information
int direction=0; //6 Directions. 0=East 1=West 2=North-East 3=North-West 4=South-East 5=South-West //direction 5
int next_direction;
int next_note;
int turn;
int turn_info;
int half_turn=55, full_turn = 180;
int SlotFive=0, SlotNine=0, SlotThirteen=0;
int slavePosition;
int slaveReached, masterReached;
int masterDist; //Distance between master and it's next note
int slaveDist; //Distance between slave and it's next note
int slaveGo; //1 When the slave has to travel to a note. 0 When master has to travel to a note.
int path_index; //Used to store index of required destination point
int shortest_path[V]={0}; //Contains the shortest path as given by Djikstra function
int dist[V]; // The output array. dist[i] will hold the shortest distance from src to dest
int reset_path=0; //If Obstacle is found in the path, set variable to 1 and recreate the shortest path.
int orientation=0; //2 Orientations. 0=Clockwise 1=AntiClockwise
int base_pos=90;//Variable to store base servo positions
int strike_pos=90;//Variable to store striker servo position
int task_status=0;// When task is complete, set variable to 1
unsigned char ADC_Conversion(unsigned char);//variable used for ADC conversion
unsigned char ADC_Value; //variable for storing ADC value
unsigned char Left_white_line = 0; //variables to keep track of left white line sensor
unsigned char Center_white_line = 0; //variables to keep track of center white line sensor
unsigned char Right_white_line = 0; //variables to keep track of right white line sensor
volatile unsigned int sequence[2] = {0};
volatile unsigned int sequenceIndex = 0;
volatile unsigned int note_index = 0;
volatile unsigned int slaveData[2] = {0}; //This array will store the data received from the rescue robot
volatile unsigned int slaveDataIndex = 0; //This variable will keep track of index of array mentioned above
volatile unsigned int slaveSrc = 0; //variable to store the current source point of slave robot
volatile unsigned int slaveDest = 0; //variable to store the current destination point of slave robot
volatile unsigned int stopMasterRobot = 0; //flag variable indicating search robot to stop or not. 1 = stop, 0 = do not stop
volatile unsigned int Degrees; //to accept angle in degrees for turning
volatile unsigned long int ShaftCountLeft = 0;	//to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0;	//to keep track of right position encoder
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
int graph [16][16]= {{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
					{1,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0},
					{0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
					{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
					{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
					{0,1,0,0,1,0,1,0,0,1,0,0,0,0,0,0},
					{0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0},
					{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
					{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
					{0,0,0,0,0,1,0,0,1,0,1,0,0,1,0,0},
					{0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0},
					{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
					{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
					{0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0},
					{0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1},
					{0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0}
					}; //Adjacency Matrix used for finding shortest path
					
int directionGraph [16][4] = {{0,0,2,0},
						{6,0,3,1},
						{0,0,4,2},
						{0,0,0,3},
						{0,0,6,0},
						{10,2,7,5},
						{0,0,8,6},
						{0,0,0,7},
						{0,0,10,0},
						{14,6,11,9},
						{0,0,12,10},
						{0,0,0,11},
						{0,0,14,0},
						{0,10,15,13},
						{0,0,16,14},
						{0,0,0,15},
						}; //Matrix to store directions and neighboring points


//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;			//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

 //Interrupt 4 enable
void left_position_encoder_interrupt_init (void)
{
	cli();						//Clears the global interrupt
	EICRB = EICRB | 0x02;		// INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10;		// Enable Interrupt INT4 for left position encoder
	sei();						// Enables the global interrupt
}

 //Interrupt 5 enable
void right_position_encoder_interrupt_init (void)
{
	cli();						//Clears the global interrupt
	EICRB = EICRB | 0x08;		// INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20;		// Enable Interrupt INT5 for right position encoder
	sei();						// Enables the global interrupt
}
//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}
//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/*
*Function Name: uart0_init
*Input:None
*Output: UART0 is initialized
*Logic:Function To Initialize UART0;desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
*/
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	//UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hz set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}


//Function To Initialize UART2
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
	UCSR2B = 0x00; //disable while setting baud rate
	UCSR2A = 0x00;
	UCSR2C = 0x06;
	UBRR2L = 0x5F; //set baud rate lo
	UBRR2H = 0x00; //set baud rate hi
	UCSR2B = 0x98;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	char data = UDR0;//making copy of data from UDR2 in 'data' variable
	UDR0 = data;
	
	slaveData[slaveDataIndex] = data;  //store the received data in int array

	if(slaveData[slaveDataIndex] == 'A')  // Slave is sending distance to next note
	{
		SlotFive = 1;
		slaveDataIndex = -1; //reset index of rescue data array to -1 so next when it receives data it will store from 0th position
		//-1 because at the end of the ISR rescueDataIndex is getting incremented so it will become 0 automatically
	}
	if(slaveData[slaveDataIndex] == 'B')
	{
		SlotNine = 1;
		slaveDataIndex = -1;
	}
	if(slaveData[slaveDataIndex] == 'C')  // Slave is sending distance to next note
	{
		SlotThirteen =1;
		slaveDataIndex = -1;
	}
	
	slaveDataIndex++; //increment the rescueDataIndex after receiving packet
	
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

//Both wheels forward
void forward (void) 
{
	motion_set(0x06);
}

//Both wheels backward
void back (void)
{
	motion_set(0x09);
}

//Left wheel backward, Right wheel forward
void left (void) 
{
	motion_set(0x05);
}

//Left wheel forward, Right wheel backward
void right (void) 
{
	motion_set(0x0A);
}

//Left wheel forward, Right wheel stationary
void soft_right(void)
{
	motion_set(0x02);
}

//Right wheel forward, Left wheel stationary
void soft_left()
{
	motion_set(0x04);
}
//stops the both wheels
void stop (void)
{
	motion_set(0x00);
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
			break;
	}
	//stop(); //Stop robot
}

//Function used for moving robot forward by specified distance
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	//stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void backward_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}

//Function to Initialize PORTS
void port_init()
{
	motion_pin_config();			//robot motion pins config
	left_encoder_pin_config();		//left encoder pin config
	right_encoder_pin_config();	    //right encoder pin config
	buzzer_pin_config();			//buzzer pin config
	lcd_port_config();				//lcd pin config
	adc_pin_config();				//adc pin config
	base_servo_pin_config();
	strike_servo_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Pre-scale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sensor Values At Desired Row And Column Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	int line_follow;	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);//change
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Initialize the devices
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	uart0_init();
	uart2_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

//Function to turn bot left
void turn_left(unsigned int deg)
{
	velocity(200, 200);
	left_degrees(deg); //turn left by degree specified
	Center_white_line = ADC_Conversion(2); //take readings of white line sensor
	while (Center_white_line <= threshold)
	{
		left();
		Center_white_line = ADC_Conversion(2);
	}
	_delay_ms(60); //wait for some time
	stop(); //stop after turning left
}

//Function to turn bot right
void turn_right(unsigned int deg)
{
	velocity(200, 200);
	right_degrees(deg); //turn right by degree specified
	Center_white_line = ADC_Conversion(2); //take readings of white line sensor
	while (Center_white_line <= threshold)
	{
		right();
		Center_white_line = ADC_Conversion(2);
	}
	_delay_ms(60); //wait for some time
	stop(); //stop after turning right
}

/*
 *
 * Function Name: East
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in east and has to move east then it must go forward
 *		  2) If the bot faces in east and has to move north then it must go left
 *		  3) If the bot faces in east and has to move south then it must go right		
 * Example Call: east(direction);
 *
 */
void east(unsigned int previous_direction) 
{ 
    if (previous_direction == point_west) 
	{    
		velocity(200,200);
	    turn_left(full_turn);//180 degree turn
		turn_info = 1800;
    } 
	else if (previous_direction == point_north) 
	{ 
		velocity(200,200);
        turn_right(half_turn);//Right
		turn_info = 901;     
	}    
	else if (previous_direction == point_south)
	{
		velocity(200,200);
		turn_left(half_turn);//Left
		turn_info = 902;
	}
	else if (previous_direction == point_east)
	{
		turn_info = 0;
	}
	
    direction = point_east;
}

/*
 *
 * Function Name: West
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in east and has to move east then it must go forward
 *		  2) If the bot faces in east and has to move north then it must go left
 *		  3) If the bot faces in east and has to move south then it must go right		
 * Example Call: west(direction);
 *
 */
void west(unsigned int previous_direction) 
{ 
    if (previous_direction == point_east) 
	{    
		velocity(200,200);
	    turn_left(full_turn);//180 degree turn
		turn_info = 1800;
    } 
	else if (previous_direction == point_south) 
	{ 
		velocity(200,200);
        turn_right(half_turn);//Right
		turn_info = 901;     
	}    
	else if (previous_direction == point_north)
	{
		velocity(200,200);
		turn_left(half_turn);//Left
		turn_info = 902;
	}
	else if (previous_direction == point_west)
	{
		turn_info = 0;
	}
	
    direction = point_west;
}

/*
 *
 * Function Name: North
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in east and has to move east then it must go forward
 *		  2) If the bot faces in east and has to move north then it must go left
 *		  3) If the bot faces in east and has to move south then it must go right		
 * Example Call: north(direction);
 *
 */
void north(unsigned int previous_direction) 
{ 
    if (previous_direction == point_south) 
	{    
		velocity(200,200);
	    turn_left(full_turn);//180 degree turn
		turn_info = 1800;
    } 
	else if (previous_direction == point_west) 
	{ 
		velocity(200,200);
        turn_right(half_turn);//Right
		turn_info = 901;     
	}    
	else if (previous_direction == point_east)
	{
		velocity(200,200);
		turn_left(half_turn);//Left
		turn_info = 902;
	}
	else if (previous_direction == point_north)
	{
		turn_info = 0;
	}
	
    direction = point_north;
}

/*
 *
 * Function Name: South
 * Input: current orientation of the robot
 * Output: void
 * Logic: 1) If the bot faces in east and has to move east then it must go forward
 *		  2) If the bot faces in east and has to move north then it must go left
 *		  3) If the bot faces in east and has to move south then it must go right		
 * Example Call: south(direction);
 *
 */
void south(unsigned int previous_direction) 
{ 
    if (previous_direction == point_north) 
	{    
		velocity(200,200);
	    turn_left(full_turn);//180 degree turn
		turn_info = 1800;
    } 
	else if (previous_direction == point_east) 
	{ 
		velocity(200,200);
        turn_right(half_turn);//Right
		turn_info = 901;     
	}    
	else if (previous_direction == point_west)
	{
		velocity(200,200);
		turn_left(half_turn);//Left
		turn_info = 902;
	}
	else if (previous_direction == point_south)
	{
		turn_info = 0;
	}
	
    direction = point_south;
}

/*
*
* Function Name:  follow
* Input: 		  void
* Output: 		  void
* Logic: 		  this function is used to follow line. It takes the readings from white line sensor and check for the detection of nodes
*				  or else it will follow the line
* Example Call:	  follow();
*
*/
void follow()
{
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line
		
		int sum = Left_white_line + Center_white_line + Right_white_line;
		
		if(sum > 150)  //if sum value is greater than 150 then bot is on the node
		{	
			stop();
			buzzer_on();
			_delay_ms(500);		//delay
			buzzer_off();
			_delay_ms(500);
			velocity(220,220);
			forward_mm(50);
			break;  //break the line following loop after encountering node to perform further actions
		}
		else  //if no node is encounter then simply follow the line
		{
			if(Center_white_line>threshold)
			{
				forward();
				velocity(220,220);
			}

			else if(Left_white_line>threshold)
			{
				stop();
				velocity(180,180);
				soft_left();
			}

			else if(Right_white_line>threshold)
			{
				stop();
				velocity(180,180);
				soft_right();
			}
			
			else if(Center_white_line<=threshold && Left_white_line<=threshold && Right_white_line<=threshold)
			{
				stop();
			}
		}
	}
}

void followback()
{
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line
		
		int sum = Left_white_line + Center_white_line + Right_white_line;
		
		if(sum > 150)  //if sum value is greater than 150 then bot is on the node
		{	
			stop();
			buzzer_on();
			_delay_ms(500);		//delay
			buzzer_off();
			_delay_ms(500);
			break;  //break the line following loop after encountering node to perform further actions
		}
		else  //if no node is encounter then simply follow the line
		{
			if(Center_white_line>threshold)
			{
				back();
				velocity(220,220);
				back();
			}

			else if(Left_white_line>threshold)
			{
				stop();
				velocity(160,200);
				back();
			}

			else if(Right_white_line>threshold)
			{
				stop();
				velocity(200,160);
				back();
			}
			
			else if(Center_white_line<=threshold && Left_white_line<=threshold && Right_white_line<=threshold)
			{
				stop();
			}
		}
	}
}

/*
 *
 * Function Name: travel(from , to)
 * Input: (from_co-ordinate , to_co-ordinate)
 * Output: void
 * Logic: Split from and to co-ordinate value into x and y co-ordinate.
 *		  travel node to node until from(x co-ordinate) = to (x co-ordinate)
 *		  travel node to node until from(y co-ordinate) = to (y co-ordinate)	 	
 * Example Call: travel(12,13);
 *					OR
 *				  travel(current_position,next_position)
 */
void travel(int from, int to)
	{ 
		int i;
		for (i=0;i<4;i++)
		{
			if (directionGraph[from-1][i]== to)
			{
				break; //Break out of If loop
			}
		}
		next_direction=i;
		
		if (next_direction==0)
		{
			east(direction);
		}
		if (next_direction==1)
		{
			west(direction);
		}
		if (next_direction==2)
		{
			north(direction);
		}
		if (next_direction==3)
		{
			south(direction);
		}
		stop();
	
		follow();		
	}


/*
 *
 * Function Name: djikstra(graph , source)
 * Input: (graph , src)
 * Output: void
 * Logic: Gives the shortest path and distance between two nodes using Djiktra's Algorith	 	
 * Example Call: djikstra(arena[48],13);
 *					OR
 *				  djikstra(arena[48],src)
 */

// A utility function to find the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
	int min = INT_MAX, min_index; // Initialize min value
	int v;
	for (v = 0; v < V; v++)
	if (sptSet[v] == false && dist[v] <= min)
	min = dist[v], min_index = v;

	return min_index;
}

void printPath(int parent[], int j) // Function to print shortest path from source to j using parent array
{
	if (parent[j]==-1) 	// Base Case : If j is source
	return;

	printPath(parent, parent[j]);

	shortest_path[0]=src;
	shortest_path[path_index]=j+1;
	path_index++;
}

void djikstra(int graph[V][V], int src) // Function that implements Dijkstra's single source shortest path
{
	bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest path tree or shortest distance from src to i is finalized
	int parent[V],i, u; // Parent array to store shortest path tree
	src=src-1;
	
	for (i = 0; i < V; i++) // Initialize all distances as INFINITE and stpSet[] as false
	{
		parent[i] = -1;
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}
	
	dist[src] = 0;	// Distance of source vertex from itself is always 0
	int count;
	
	for (count = 0; count < V-1; count++) // Find shortest path for all vertices
	{
		u = minDistance(dist, sptSet);// U contains the index of the minimum distance from source to dest
		sptSet[u] = true; // Mark the picked vertex as processed
		int v;
		for (v = 0; v < V; v++) 		// Update dist value of the adjacent vertices of the picked vertex.
		if (!sptSet[v] && graph[u][v] &&      // Update dist[v] only if is not in sptSet, there is
		dist[u] + graph[u][v] < dist[v])  // an edge from u to v, and total weight of path from
		{									  // src to v through u is smaller than current value of
			parent[v] = u;					  // dist[v]
			dist[v] = dist[u] + graph[u][v];
		}
	}
	printPath(parent, dest-1);
	path_index=1;
}


//Function that will call the travel function and have the bot travel from one node to another.
void travel_path(int shortest_path[V])
{
	int i=1;
	while(1)
	{
		if (shortest_path[i] == 0)
		{
			break;
		}
		dest= shortest_path[i];
		if (src == dest)
		{
			break;
		}
		lcd_print(2,12, src,2);
		lcd_print(2, 15, dest,2);
		travel(src, dest);
		prev_node= src;
		src= dest;
		if(shortest_path[i+1] == 0)
		{
			lcd_print(2, 10 , orientation, 1);
			lcd_print(2, 12, direction , 1);
			break;
		}
		dest= shortest_path[i+1];
		lcd_print(2,12, src,2);
		lcd_print(2,15, dest,2);
		i++;
	}
}

void beep_beep_buzzer()
{
	buzzer_on();
	_delay_ms(200);
	buzzer_off();
	_delay_ms(50);
	buzzer_on();
	_delay_ms(200);
	buzzer_off();
	_delay_ms(50);
	buzzer_on();
	_delay_ms(200);
	buzzer_off();
	_delay_ms(50);
}


int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	//Setting up Servos for Default positions
	base_servo(30);
	_delay_ms(1000);
	servo_base_free();
	
	follow();//follow line
	while (1)
	{
		SlotFive = 0;
		SlotNine = 0;
		SlotThirteen = 0;
		src=2;
		stop();//stop
		base_servo(30);
		_delay_ms(1000);
		UDR0 = 'S';
		_delay_ms(15);
		if (SlotFive == 0)
		{
			dest = 5;
		}
		else if (SlotNine == 0)
		{
			dest = 9;
		}
		else if (SlotThirteen == 0)
		{
			dest = 13;
		}
		else 
		{
			stop();
			beep_beep_buzzer();
			break;
		}
		src=2;
		stop();
		lcd_print(2,12, src,2);
		lcd_print(2,15, dest,2);
		if (src == 2 && dest == 5)
		{
			backward_mm(30);
			turn_right(half_turn);
			follow();
			stop();
			beep_beep_buzzer();
			base_servo(90);
			_delay_ms(1000);
			memset(shortest_path,0,V);
			back();
			velocity(200,200);
			backward_mm(120);
			followback();
			forward_mm(70);
			turn_right(half_turn);
			forward();
			follow();
			forward_mm(50);
		}
		
		if (src == 2 && dest == 7)
		{
			turn_left(half_turn);
			follow();
			stop();
			beep_beep_buzzer();
			base_servo(90);
			_delay_ms(1000);
			memset(shortest_path,0,V);
			back();
			velocity(200,200);
			backward_mm(120);
			followback();
			forward_mm(70);
			turn_left(half_turn);
			forward();
			follow();
			forward_mm(50);
		}
		else 
		{
			memset(shortest_path,0,V); //Fill Shortest path with Zeroes
			djikstra(graph,src);
			travel_path(shortest_path);
			stop();
			beep_beep_buzzer();
			base_servo(90);
			_delay_ms(1000);
			memset(shortest_path,0,V);
			back();
			velocity(200,200);
			backward_mm(120);
			followback();
			forward_mm(70);
			lcd_print(2,12, src,2);
			lcd_print(2,15, dest,2);
			lcd_print(2,9,8,1);
			stop();
		}		
		switch(src)
		{	
			case 9:
			turn_right(half_turn);
			forward();
			follow();
			follow();
			forward_mm(50);
			break;
			
			case 11:
			turn_left(half_turn);
			forward();
			follow();
			follow();
			forward_mm(50);
			break;
			
			case 13:
			turn_right(half_turn);
			forward();
			follow();
			follow();
			follow();
			forward_mm(50);
			break;
			
			case 15:
			turn_right(half_turn);
			forward();
			follow();
			follow();
			follow();
			forward_mm(50);
			break;
		}	
		turn_right(180);
		stop();
	}
}
