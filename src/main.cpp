#include <Arduino.h>
#include "IRLine.h"
#include "state_machines.h"
#include "robot.h"
#include "config.h"
#include "path_handler.h"

//#define DEBUG



void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
int read_PIO_encoder(int sm);

volatile long enc_left = 0;
volatile long enc_right = 0;

// ---------------------------------------------------------------------------
// Quadrature encoder decoding via lookup table
// Each ISR fires on CHANGE of either pin A or B.
// next_state = (A << 1) | B  →  2-bit value (0..3)
// table_input = (prev_state << 2) | next_state  →  4-bit index (0..15)
// encoder_table maps every possible transition to -1, 0 or +1
// This catches all 4 edges per cycle, giving full quadrature resolution.
// ---------------------------------------------------------------------------
static const int encoder_table[16] = {0, 1,-1, 0,
                                      -1, 0, 0, 1,
                                       1, 0, 0,-1,
                                       0,-1, 1, 0};

#define pinIsHigh(pin, pins) (((1 << (pin)) & (pins)) >> (pin))

volatile int encoder1_state = 0;
volatile int encoder2_state = 0;

void enc_left_ISR() {
    int pins       = sio_hw->gpio_in;                    // single atomic read of all GPIOs
    int next_state = pinIsHigh(ENC1_A, pins) << 1
                   | pinIsHigh(ENC1_B, pins);
    enc_left      += encoder_table[(encoder1_state << 2) | next_state];
    encoder1_state = next_state;
}

void enc_right_ISR() {
    int pins       = sio_hw->gpio_in;
    int next_state = pinIsHigh(ENC2_A, pins) << 1
                   | pinIsHigh(ENC2_B, pins);
    enc_right     -= encoder_table[(encoder2_state << 2) | next_state]; // '-=' corrige montagem invertida
    encoder2_state = next_state;
}


void setup()
{

  Serial.begin();
  
  pinMode (START_BUTTON,INPUT_PULLUP);
  pinMode (RESET_BUTTON,INPUT_PULLUP);

  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc_left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc_left_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc_right_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc_right_ISR, CHANGE);


 

  // Motor driver pins
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);
 
  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);


   // ADC mux pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  
  //init_PIO_dual_encoders(ENC1_A, ENC2_A);

  
  analogReadResolution(10);

  //Initialize the robot stopped
 
  robot.stop();
 
}


void loop() {
		
    
  // Read and print sensors
    
    robot.IRLine.readIRSensors();

    //robot.IRLine.printIRLine();
    robot.IRLine.detectNode();

    Main_FSM_Handler();
    Map_FSM_Handler();

    robot.setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    robot.setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

    robot.setMotorPWM(80, MOTOR1A_PIN, MOTOR1B_PIN);
    robot.setMotorPWM(80, MOTOR2A_PIN, MOTOR2B_PIN);
    delay(2000);
    robot.setMotorPWM(0, MOTOR1A_PIN, MOTOR1B_PIN);
    robot.setMotorPWM(0, MOTOR2A_PIN, MOTOR2B_PIN);
    delay(1000);
  
    //Encoder reading
	  // edge_detection();
    // Serial.print("L = ");
    // Serial.print(enc_left);
    // Serial.print("   R = ");
    // Serial.println(enc_right);
    // delay(150);


  /**
   * State Machines Handlers
   */
    // Solve_FSM_Handler();
    //Test_FSM_Handler();
    //FodaseFMSHandler();
  //** End of State Machines Handlers
  

  #ifdef DEBUG
    // Serial.printf("PWM1: %d\n",robot.PWM_1);
    // Serial.printf("PWM2%d\n",robot.PWM_2);
  #endif
}