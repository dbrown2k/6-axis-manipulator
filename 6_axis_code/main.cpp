/*
 * 6_axis_code.cpp
 *
 * Created: 10/09/2017 12:13:09
 * Author : david
 */ 

#include <avr/io.h> 
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h> //F_CPU="16000000" or 16MHz clock

//define function prototypes
uint8_t calc_legs();
void move_legs();
void read_switches();
void zero_legs();
float c(float deg);
float s(float deg);
void en_motors(uint8_t enable);
void step_motor(uint8_t motor, uint8_t dir);

//Define pins
//Leg_0
// step
#define leg0_step_mask (1<<PINH0)
#define leg0_step_port PORTH
#define leg0_step_dir DDRH
// dir
#define leg0_dir_mask (1<<PINH1)
#define leg0_dir_port PORTH
#define leg_dir_dir DDRH
// en
#define leg0_en_mask (1<<PINL1)
#define leg0_en_port PORTL
#define leg0_en_dir DDRL
//stop button
#define leg0_stop_mask (1<<PINC3)
#define leg0_stop_pin PINC
#define leg0_stop_port PORTC

//Leg_1
// step
#define leg1_step_mask (1<<PINF0)
#define leg1_step_port PORTF
#define leg1_step_dir DDRF
// dir
#define leg1_dir_mask (1<<PINL2)
#define leg1_dir_port PORTL
#define leg1_dir_dir DDRL
// en
#define leg1_en_mask (1<<PINF1)
#define leg1_en_port PORTF
#define leg1_en_dir DDRF
//stop button
#define leg1_stop_mask (1<<PINC2)
#define leg1_stop_pin PINC
#define leg1_stop_port PORTC

//Leg_2
// step
#define leg2_step_mask (1<<PINF3)
#define leg2_step_port PORTF
#define leg2_step_dir DDRF
// dir
#define leg2_dir_mask (1<<PINF2)
#define leg2_dir_port PORTF
#define leg2_dir_dir DDRF
// en
#define leg2_en_mask (1<<PINK0)
#define leg2_en_port PORTK
#define leg2_en_dir DDRK
//stop button
#define leg2_stop_mask (1<<PINC1)
#define leg2_stop_pin PINC
#define leg2_stop_port PORTC

//Leg_3
// step
#define leg3_step_mask (1<<PINA1)
#define leg3_step_port PORTA
#define leg3_step_pin PINA
#define leg3_step_dir DDRA
// dir
#define leg3_dir_mask (1<<PINA0)
#define leg3_dir_port PORTA
#define leg3_dir_dir DDRA
// en
#define leg3_en_mask (1<<PINA2)
#define leg3_en_port PORTA
#define leg3_en_dir DDRA
//stop button
#define leg3_stop_mask (1<<PINC0)
#define leg3_stop_pin PINC
#define leg3_stop_port PORTC

//Leg_4
// step
#define leg4_step_mask (1<<PINA4)
#define leg4_step_port PORTA
#define leg4_step_dir DDRA
// dir
#define leg4_dir_mask (1<<PINA3)
#define leg4_dir_port PORTA
#define leg4_dir_dir DDRA
// en
#define leg4_en_mask (1<<PINA5)
#define leg4_en_port PORTA
#define leg4_en_dir DDRA
//stop button
#define leg4_stop_mask (1<<PINC5)
#define leg4_stop_pin PINC
#define leg4_stop_port PORTC

//Leg_5
// step
#define leg5_step_mask (1<<PINA7)
#define leg5_step_port PORTA
#define leg5_step_dir DDRA
// dir
#define leg5_dir_mask (1<<PINA6)
#define leg5_dir_port PORTA
#define leg5_dir_dir DDRA
// en
#define leg5_en_mask (1<<PING2)
#define leg5_en_port PORTG
#define leg5_en_dir DDRG
//stop button
#define leg5_stop_mask (1<<PINC4)
#define leg5_stop_pin PINC
#define leg5_stop_port PORTC


//machine parameters
#define step_delay_us 8		//DRV8825 max step frequency is 250kHz = 4us
#define micro_steps 16		//control board setup
#define mm_steps 88.419413	//number of whole steps per mm for mechanism
#define leg_range 2.0		//range of mm travel

//base anchor point local coordinates (X, Y, Z) (mm)
const float base[6][3] = {
	{-3.00, 28.00, 0},
	{3.00, 28.00, 0},
	{25.75, -11.40, 0},
	{22.75, -16.60, 0},
	{-22.75, -16.60, 0},
	{-25.75, -11.40, 0}			
};		
//platform anchor point local coordinates (X, Y, Z)
const float platform[6][3] = {
	{-5.50, 5.48, 0},
	{5.50, 5.48, 0},
	{7.50, 2.02, 0},
	{2.00, -7.50, 0},
	{-2.00, -7.50, 0},
	{-7.50, 2.02, 0},
};	
//point of interest offset from platform coordinates
const float offset[3] = {0, 0, 0};		//X, Y, Z
const float z_mid = 20.0; 
const float	leg_mid = 30.00;


//table of sin values for 0 to 5deg in 0.1deg increments
const float sin_table[51] = {
	0,
	0.001745,
	0.003491,
	0.005236,
	0.006981,
	0.008727,
	0.010472,
	0.012217,
	0.013962,
	0.015707,
	0.017452,
	0.019197,
	0.020942,
	0.022687,
	0.024432,
	0.026177,
	0.027922,
	0.029666,
	0.031411,
	0.033155,
	0.034899,
	0.036644,
	0.038388,
	0.040132,
	0.041876,
	0.043619,
	0.045363,
	0.047106,
	0.04885,
	0.050593,
	0.052336,
	0.054079,
	0.055822,
	0.057564,
	0.059306,
	0.061049,
	0.062791,
	0.064532,
	0.066274,
	0.068015,
	0.069756,
	0.071497,
	0.073238,
	0.074979,
	0.076719,
	0.078459,
	0.080199,
	0.081939,
	0.083678,
	0.085417,
	0.087156
};
//table for cos values for 0 to 5deg in 0.1deg increments
const float cos_table[51] = {
	1,
	0.999998,
	0.999994,
	0.999986,
	0.999976,
	0.999962,
	0.999945,
	0.999925,
	0.999903,
	0.999877,
	0.999848,
	0.999816,
	0.999781,
	0.999743,
	0.999701,
	0.999657,
	0.99961,
	0.99956,
	0.999507,
	0.99945,
	0.999391,
	0.999328,
	0.999263,
	0.999194,
	0.999123,
	0.999048,
	0.998971,
	0.99889,
	0.998806,
	0.998719,
	0.99863,
	0.998537,
	0.998441,
	0.998342,
	0.99824,
	0.998135,
	0.998027,
	0.997916,
	0.997801,
	0.997684,
	0.997564,
	0.997441,
	0.997314,
	0.997185,
	0.997053,
	0.996917,
	0.996779,
	0.996637,
	0.996493,
	0.996345,
	0.996195
};


//end stops bits 0 to 5, 1 = set, 0 = clear
volatile uint8_t end_stops;

//number of steps that need to be taken
volatile int16_t leg_steps[6];
volatile int16_t last_steps[6] = {0,0,0,0,0,0};

//required position
volatile float new_offset[6];
volatile float curr_offset[6] = {0,0,0,0,0,0};



int main(void)
{
   //set output bits
   //leg0
   leg0_en_dir |= leg0_en_mask;
   leg0_step_dir |= leg0_step_mask;
   leg0_en_dir |= leg0_en_mask;
   //leg1
   leg1_en_dir |= leg1_en_mask;
   leg1_step_dir |= leg1_step_mask;
   leg1_en_dir |= leg1_en_mask;
   //leg2
   leg2_en_dir |= leg2_en_mask;
   leg2_step_dir |= leg2_step_mask;
   leg2_en_dir |= leg2_en_mask;
   //leg3
   leg3_en_dir |= leg3_en_mask;
   leg3_step_dir |= leg3_step_mask;
   leg3_en_dir |= leg3_en_mask;
   //leg4
   leg4_en_dir |= leg4_en_mask;
   leg4_step_dir |= leg4_step_mask;
   leg4_en_dir |= leg4_en_mask;
   //leg5
   leg5_en_dir |= leg5_en_mask;
   leg5_step_dir |= leg5_step_mask;
   leg5_en_dir |= leg5_en_mask;
	
	//set pull up on pinouts
	leg0_stop_port |= leg0_stop_mask;
	leg1_stop_port |= leg1_stop_mask;
	leg2_stop_port |= leg2_stop_mask;
	leg3_stop_port |= leg3_stop_mask;
	leg4_stop_port |= leg4_stop_mask;
	leg5_stop_port |= leg5_stop_mask;
   
   en_motors(1);
   
   
   //zero arms
   zero_legs();
   _delay_ms(500);
   
    while (1) 
    {
		
		for (uint8_t leg = 0; leg <= 2; leg++)
		{
		_delay_ms(500);
		new_offset[leg] = 0.5;
		move_legs();
		_delay_ms(500);
		new_offset[leg] = -0.5;
		move_legs();
		_delay_ms(500);
		new_offset[leg] = 0;
		move_legs();
		}
		
		for (uint8_t leg = 3; leg <= 5; leg++)
		{
			_delay_ms(500);
			new_offset[leg] = 3;
			move_legs();
			_delay_ms(500);
			new_offset[leg] = -3;
			move_legs();
			_delay_ms(500);
			new_offset[leg] = 0;
			move_legs();
		}
		
		_delay_ms(500);
		zero_legs();
		
		//en_motors(0);
	
    }
}


//calculate leg 
uint8_t calc_legs()
{
	//make it easier to understand the formula for the Gough-Stuart platform
	const uint8_t X = 0;
	const uint8_t Y = 1;
	const uint8_t Z = 2;	
	const uint8_t RX = 3;
	const uint8_t RY = 4;
	const uint8_t RZ = 5;
	
	
	//translation array
	float t[3] = {
		new_offset[X] + offset[X],
		new_offset[Y]  + offset[Y],
		new_offset[Z]  + z_mid + offset[Z]
	};
	
	//calculate rotation sin/cosin values
	float s_roll = s(new_offset[RX]);
	float s_pitch = s(new_offset[RY]);
	float s_yaw = s(new_offset[RZ]);
	float c_roll = c(new_offset[RX]);
	float c_pitch = c(new_offset[RY]);
	float c_yaw = c(new_offset[RZ]);
	
	
	float leg_change[6];
	float p[3];
	float leg_vector[6][3];	
	
	//iterate though the legs
	for (uint8_t leg = 0; leg <= 5; leg++)
	{
		//leg platform coordinates
		p[0] = platform[leg][X] - offset[X];
		p[1] = platform[leg][Y] - offset[Y];
		p[2] = platform[leg][Z] - offset[Z];
		
		
		//calculate leg vector
		leg_vector[leg][0] = (t[X] + c_yaw * c_pitch * p[X]) + ((-s_yaw * c_roll + c_yaw * s_pitch * s_roll) * p[Y]) + ((s_yaw * s_roll + c_yaw * s_pitch * c_roll) * p[Z]) - base[leg][X];
		leg_vector[leg][1] = (t[Y] + s_yaw * c_pitch * p[X]) + ((c_yaw * c_roll + s_yaw * s_pitch * s_roll) * p[Y]) + ((-c_yaw *s_roll + s_yaw * s_pitch * c_roll) * p[Z]) - base[leg][Y];
		leg_vector[leg][2] = (t[Z] + -s_pitch * p[X]) + (c_pitch * s_roll * p[Y]) + (c_pitch * c_roll * p[Z]) - base[leg][Z];

		
		//calculate leg length
		leg_change[leg] = sqrtf(leg_vector[leg][X]*leg_vector[leg][X] + leg_vector[leg][Y]*leg_vector[leg][Y] + leg_vector[leg][Z]*leg_vector[leg][Z]) - leg_mid;
		
		//check if out of range (leg length)
		if (leg_change[leg] <= -leg_range || leg_change[leg] >= leg_range)
		{
			return 0;
		}
		
		
	}
	
	
	//calculate number of steps do this after all legs have been successfully evaluated
	for (uint8_t leg = 0; leg <= 5; leg++)
	{
		last_steps[leg] = leg_steps[leg]; //update previous steps once we know that it is valid
		
		leg_steps[leg] = int16_t(leg_change[leg] * mm_steps * micro_steps);
	}
	
	return 1;
	
}


//move legs based on steps required
void move_legs()
{
	//check to see if there is a difference in leg position
	if (new_offset != curr_offset)
	{
		uint8_t leg = 0;
		
		//calculate the new position
		calc_legs();	
		
		//temp value
		int16_t temp_steps[6];
		
		
		
		//attempt to even out the effects of rounding errors
		temp_steps[0] = (leg_steps[0] - last_steps[0] +1);
		temp_steps[1] = (leg_steps[1] - last_steps[1] -1);
		temp_steps[2] = (leg_steps[2] - last_steps[2] +1);
		temp_steps[3] = (leg_steps[3] - last_steps[3] -1);
		temp_steps[4] = (leg_steps[4] - last_steps[4] +1);
		temp_steps[5] = (leg_steps[5] - last_steps[5] -1);
		
		
		//calculate absolute value for further calculations
		uint16_t uleg_steps[6];
		uint8_t leg_dirs = 0;
		uint16_t max_steps = 0;
		
		
		
		for (leg = 0; leg <= 5; leg++)
		{
			uleg_steps[leg] = abs(temp_steps[leg]); //absolute value - unsigned
			
			if (temp_steps[leg] >= 0)
			{
				leg_dirs |= (1<<leg); //is positive so set bit
			}
			//calculate max number of steps for any leg
			if (uleg_steps[leg] >= max_steps)
			{
				max_steps = uleg_steps[leg];
			}
		}
		
		//calculate intervals to step intermediate legs
		
		
		
		//iterate and step motors to move to new position
		for (uint16_t i = max_steps; i > 0; i--)
		{
			for (leg = 0; leg <= 5; leg++)
			{
				if (uleg_steps[leg] != 0)
				{
					step_motor(leg, uint8_t(leg_dirs & (1<<leg)));
					uleg_steps[leg]--;
				}
				
				curr_offset[leg] = new_offset[leg];
			}	
		}
		
	}
	
	
}

//calculate backlash in system


//read end stop switches and update switch value
void read_switches()
{
	//check if button is low (pulled high pin connected to ground)
	if ((leg0_stop_pin & leg0_stop_mask) == 0) 
		{ end_stops |= (1<<0); } //if button closed then set appropriate bit in the end_stop register
	else { end_stops &= ~(1<<0); }
		
	if ((leg1_stop_pin & leg1_stop_mask) == 0) 
		{ end_stops |= (1<<1); } 
	else { end_stops &= ~(1<<1); }
		
	if ((leg2_stop_pin & leg2_stop_mask) == 0) 
		{ end_stops |= (1<<2); } 
	else { end_stops &= ~(1<<2); }
		
	if ((leg3_stop_pin & leg3_stop_mask) == 0) 
		{ end_stops |= (1<<3); } 
	else { end_stops &= ~(1<<3); }
		
	if ((leg4_stop_pin & leg4_stop_mask) == 0) 
		{ end_stops |= (1<<4); } 
	else { end_stops &= ~(1<<4); }
		
	if ((leg5_stop_pin & leg5_stop_mask) == 0) 
		{ end_stops |= (1<<5); } 
	else { end_stops &= ~(1<<5); }	
		
}

//zero legs
void zero_legs()
{
	read_switches(); 
	
	while (end_stops < 0b00111111) //as the end stops are a binary value, 1 = set, stop when last leg set
	{
		read_switches(); //make sure that its not already in the stop position before moving motors
		
		//use end_stops as a mask for stepping motors
		//run one round of stepping and then re-check stops
		for (uint8_t leg = 0; leg <= 5; leg++)
		{
			if ((end_stops & (1<<leg)) == 0)
			{
				step_motor(leg, 0);
				_delay_us(50);
			}
		}
		
	}
	
	//move to mid position
	for (uint16_t i = 0; i <= (leg_range/2)*mm_steps*micro_steps; i++)
	{
		step_motor(0, 1);
		step_motor(1, 1);
		step_motor(2, 1);
		step_motor(3, 1);
		step_motor(4, 1);
		step_motor(5, 1);
		_delay_us(50);
	}
	
	
	//zero leg steps
	for (uint8_t leg = 0; leg <= 5; leg++)
	{
		last_steps[leg] = 0;
	}
	
}

//cos lookup
float c(float deg)
{
	return cos_table[abs(deg*10)];
}

//sin lookup
float s(float deg)
{
	float abs_sin = sin_table[abs(deg*10)];
	
	if (deg < 0) //check for negative
	{
		abs_sin = 0 - abs_sin;
	}
	
	return abs_sin;
}

//enable / disable motors
void en_motors(uint8_t enable)
{
	if (enable == 0)
	{
		leg0_en_port |= (leg0_en_mask);
		leg1_en_port |= (leg1_en_mask);
		leg2_en_port |= (leg2_en_mask);
		leg3_en_port |= (leg3_en_mask);
		leg4_en_port |= (leg4_en_mask);
		leg5_en_port |= (leg5_en_mask);
	} 
	else
	{
		leg0_en_port &= ~(leg0_en_mask);
		leg1_en_port &= ~(leg1_en_mask);
		leg2_en_port &= ~(leg2_en_mask);
		leg3_en_port &= ~(leg3_en_mask);
		leg4_en_port &= ~(leg4_en_mask);
		leg5_en_port &= ~(leg5_en_mask);
	}
}

//step specific motor
void step_motor(uint8_t motor, uint8_t dir)
{


	if (motor == 0)
	{
		if (dir == 0)
		{	leg0_dir_port |= (leg0_dir_mask);	}
		else
		{	leg0_dir_port &= ~(leg0_dir_mask);	}
	}
	
	if (motor == 1)
	{
		if (dir == 0)
		{	leg1_dir_port &= ~(leg1_dir_mask);	}
		else
		{	leg1_dir_port |= (leg1_dir_mask);	}
	}
	
	if (motor == 2)
	{
		if (dir == 0)
		{	leg2_dir_port |= (leg2_dir_mask);	}
		else
		{	leg2_dir_port &= ~(leg2_dir_mask);	}
	}
	
	if (motor == 3)
	{
		if (dir == 0)
		{	leg3_dir_port &= ~(leg3_dir_mask);	}
		else
		{	leg3_dir_port |= (leg3_dir_mask);	}
	}
	
	if (motor == 4)
	{
		if (dir == 0)
		{	leg4_dir_port |= (leg4_dir_mask);	}
		else
		{	leg4_dir_port &= ~(leg4_dir_mask);	}
	}
	
	if (motor == 5)
	{
		if (dir == 0)
		{	leg5_dir_port &= ~(leg5_dir_mask);	}
		else
		{	leg5_dir_port |= (leg5_dir_mask);	}
	}
	else
	{	}
	
uint8_t delay = step_delay_us * (32/micro_steps);
// rising edge causes driver index to move one step
	if (motor == 0)
	{
		leg0_step_port |= (leg0_step_mask);
		_delay_us(delay);
		leg0_step_port &= ~(leg0_step_mask);
	}
	if (motor == 1)
	{
		leg1_step_port |= (leg1_step_mask);
		_delay_us(delay);
		leg1_step_port &= ~(leg1_step_mask);
	}
	if (motor == 2)
	{
		leg2_step_port |= (leg2_step_mask);
		_delay_us(delay);
		leg2_step_port &= ~(leg2_step_mask);
	}
	if (motor == 3)
	{
		leg3_step_port |= (leg3_step_mask);
		_delay_us(delay);
		leg3_step_port &= ~(leg3_step_mask);
	}
	if (motor == 4)
	{
		leg4_step_port |= (leg4_step_mask);
		_delay_us(delay);
		leg4_step_port &= ~(leg4_step_mask);
	}
	if (motor == 5)
	{
		leg5_step_port |= (leg5_step_mask);
		_delay_us(delay);
		leg5_step_port &= ~(leg5_step_mask);
	} 
	else
	{	}

}

