/**
  *******************
    SONIC HC-SR04
      5V - 5V
      GND - GND
      TRIG - [C4] TIM1 CH4 PWM
      ECHO - [C3]

    SERVO SG-90
      5V - 5V
      GND - MOSFET DRAIN
      PWM - [A3] TIM2 CH3 PWM

    MOSFET
      DRAIN - SERVO GND
      SOURCE - GND
      GATE - [D3]

    SWITCH
      UP POSITION - [C6]
      DOWN POSITION - [C7]
      MIDDLE - GND

    GREEN LED
      5V - [D5]
      GND - GND

    RED LED
      5V - [D6]
      GND - GND
  ******************
*/

#include "stm8s.h"

#define ECHO_PORT GPIOC
#define ECHO_PIN GPIO_PIN_3
#define ECHO_EXTI_PORT EXTI_PORT_GPIOC
#define ECHO_IRQ_NAME EXTI_PORTC_IRQHandler
#define ECHO_IRQ_NUM 5

#define SERVO GPIOD,GPIO_PIN_3

#define SW_UP GPIOC,GPIO_PIN_6
#define SW_DOWN GPIOC,GPIO_PIN_7

#define INIT_LED_PORT GPIOB
#define INIT_LED_PIN GPIO_PIN_5

#define LED_G1 GPIOA, GPIO_PIN_1
#define LED_G2 GPIOC, GPIO_PIN_5
#define LED_Y3 GPIOD, GPIO_PIN_5
#define LED_Y4 GPIOD, GPIO_PIN_6
#define LED_R5 GPIOD, GPIO_PIN_2
#define LED_R6 GPIOD, GPIO_PIN_4

#define G_LED_PORT GPIOD
#define G_LED_PIN GPIO_PIN_5
#define R_LED_PORT GPIOD
#define R_LED_PIN GPIO_PIN_6


#define OPEN_TIME 350 // open cap time OPEN_TIME*20mS
#define ACTIVE_DISTANCE 45 // distance for activate in cm
#define OPENING_TIME 50 // time for turn on servo while cap is moving to opened position
#define CLOSING_TIME 120 // time for slowly closing cap
#define ACT_DIS_CNTR 2 // for exception false activate, activation mus be ACT_DIS_CNTR times
#define SERVO_OPENED 700 // counter for TIM1 PWM
#define SERVO_CLOSED 2200 // counter for TIM1 PWM

short servoIsActive=0;
unsigned short distance=0; // distanse from HC-SR04 in centimeters
int delay=0; // delay for open cap time
short openingDelay=0; // delay for turn on servo while cap is moving to opened posotion
short closingDelta;
unsigned short initBlinking=250; // delay for init blinking
short ledFlush=0;
short ledDelaySeg; // segment of delay for sequentially turning off LEDs
short distanceCntr=ACT_DIS_CNTR;

void tim1Init(){ // TIM1 generate PWM with 10uS pulse per 60mS for HC-SR06 trigger
  TIM1_TimeBaseInit(15,TIM1_COUNTERMODE_UP,60000,0);
  TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, 11, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_SET);
  TIM1_Cmd(ENABLE);
  TIM1_CtrlPWMOutputs(ENABLE);
}

void tim2Init(){ // TIM2 generate PWM 50Hz for servo, peruod 20mS
  TIM2_TimeBaseInit(TIM2_PRESCALER_16,20000);
  TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,SERVO_CLOSED, TIM2_OCPOLARITY_HIGH); //500-2000
  TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);
  TIM2_OC3PreloadConfig(ENABLE);
  TIM2_Cmd(ENABLE);
}

void gpioInit(){
  GPIO_Init(ECHO_PORT,ECHO_PIN,GPIO_MODE_IN_FL_IT);
  GPIO_Init(SERVO,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(SW_UP,GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(SW_DOWN,GPIO_MODE_IN_PU_NO_IT);
  
  GPIO_Init(LED_G1,GPIO_MODE_OUT_OD_HIZ_SLOW);
  GPIO_Init(LED_G2,GPIO_MODE_OUT_OD_LOW_SLOW);
  GPIO_Init(LED_Y3,GPIO_MODE_OUT_OD_HIZ_SLOW);
  GPIO_Init(LED_Y4,GPIO_MODE_OUT_OD_LOW_SLOW); 
  GPIO_Init(LED_R5,GPIO_MODE_OUT_OD_HIZ_SLOW);
  GPIO_Init(LED_R6,GPIO_MODE_OUT_OD_LOW_SLOW);

}

void interruptConfig(){
  EXTI_SetExtIntSensitivity(ECHO_EXTI_PORT,EXTI_SENSITIVITY_FALL_ONLY);
  enableInterrupts();
}

// open servo's MOSFET
void servoOn(){
  GPIO_WriteHigh(SERVO);
  servoIsActive=1;
}

// close servo's MOSFET
void servoOff(){
  GPIO_WriteLow(SERVO);
  servoIsActive=0;
}

// turn off all G,R,Y LEDs
void ledAllOff(){
    GPIO_WriteHigh(LED_G1);
    GPIO_WriteHigh(LED_G2);
    GPIO_WriteHigh(LED_Y3);
    GPIO_WriteHigh(LED_Y4);
    GPIO_WriteHigh(LED_R5);
    GPIO_WriteHigh(LED_R6);
}

//turn on all LEDs
void ledAllOn(){
    GPIO_WriteLow(LED_G1);
    GPIO_WriteLow(LED_G2);
    GPIO_WriteLow(LED_Y3);
    GPIO_WriteLow(LED_Y4);
    GPIO_WriteLow(LED_R5);
    GPIO_WriteLow(LED_R6);
}
void moveServo(){
    if(openingDelay){ //time for opening cap
        TIM2_SetCompare3(SERVO_OPENED); //opened cap position
        servoOn();
        openingDelay--;
    }else{ 
        if(delay>CLOSING_TIME) servoOff();
        else{
          TIM2_SetCompare3(SERVO_CLOSED-delay*closingDelta);
          servoOn();
        }
    }
}

INTERRUPT_HANDLER(ECHO_IRQ_NAME, ECHO_IRQ_NUM)
{
    distance=(TIM1_GetCounter()-550)/58; // get distance in cm from TIM1 counter 
   
    if(!servoIsActive && distance<ACTIVE_DISTANCE) // get distance when servo turned off
        distanceCntr=!distanceCntr?0:distanceCntr-1;
    else
        distanceCntr=ACT_DIS_CNTR;
    
    if(!distanceCntr && GPIO_ReadInputPin(SW_UP)){ // legal activation
        initBlinking=0;
        if(!delay) openingDelay=OPENING_TIME;
        delay=OPEN_TIME;
    }
    if(!GPIO_ReadInputPin(SW_UP))
        if(!distanceCntr){
            GPIO_WriteLow(LED_R5);
            GPIO_WriteLow(LED_R6);
        }else if(!initBlinking){
            GPIO_WriteHigh(LED_R5);
            GPIO_WriteHigh(LED_R6);
        }
}

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13){
    if(initBlinking){
        if(initBlinking%10==0){
            GPIO_WriteReverse(LED_G1);
            GPIO_WriteReverse(LED_G2);
            GPIO_WriteReverse(LED_Y3);
            GPIO_WriteReverse(LED_Y4);
            GPIO_WriteReverse(LED_R5);
            GPIO_WriteReverse(LED_R6);
        }
        initBlinking--;
    }
    if(delay){ // cap open or opening or closing
        if(delay==OPEN_TIME) GPIO_WriteLow(LED_R6);
        if(delay==OPEN_TIME-1) GPIO_WriteLow(LED_R5);
        if(delay==OPEN_TIME-2) GPIO_WriteLow(LED_Y4);
        if(delay==OPEN_TIME-3) GPIO_WriteLow(LED_Y3);
        if(delay==OPEN_TIME-4) GPIO_WriteLow(LED_G2);
        if(delay==OPEN_TIME-5) GPIO_WriteLow(LED_G1);
        
        if(delay==OPEN_TIME-ledDelaySeg) GPIO_WriteHigh(LED_G1);
        if(delay==OPEN_TIME-ledDelaySeg*2) GPIO_WriteHigh(LED_G2);
        if(delay==OPEN_TIME-ledDelaySeg*3) GPIO_WriteHigh(LED_Y3);
        if(delay==OPEN_TIME-ledDelaySeg*4) GPIO_WriteHigh(LED_Y4);
        if(delay==OPEN_TIME-ledDelaySeg*5) GPIO_WriteHigh(LED_R5);
        if(delay==OPEN_TIME-ledDelaySeg*6) GPIO_WriteHigh(LED_R6);
        moveServo();
        delay--;
    }else{ //cap closed
        servoOff();
        if(!initBlinking) ledAllOff();
    }
    TIM2_ClearITPendingBit(TIM2_FLAG_UPDATE);
}

void main() {
    CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
    gpioInit();
    tim1Init();
    tim2Init();
    interruptConfig(); 
    ledDelaySeg=(OPEN_TIME-CLOSING_TIME)/6;
    closingDelta=(SERVO_CLOSED-SERVO_OPENED+100)/CLOSING_TIME; // 100 for firs long move
    while (1) {
      if(!servoIsActive && !GPIO_ReadInputPin(SW_DOWN)){
            if(!delay) openingDelay=OPENING_TIME;
            delay=OPEN_TIME;
            ledAllOn();
      }
       
  } 
}