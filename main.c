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
  ******************
*/

#include "stm8s.h"

#define ECHO_PORT GPIOC
#define ECHO_PIN GPIO_PIN_3
#define ECHO_EXTI_PORT EXTI_PORT_GPIOC
#define ECHO_IRQ_NAME EXTI_PORTC_IRQHandler
#define ECHO_IRQ_NUM 5

#define SERVO_PORT GPIOD
#define SERVO_PIN GPIO_PIN_3

#define SW_UP_PORT GPIOC
#define SW_UP_PIN GPIO_PIN_6

#define SW_DOWN_PORT GPIOC
#define SW_DOWN_PIN GPIO_PIN_7

#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_5

#define OPEN_TIME 300
#define ACTIVE_DISTANCE 25
#define CAP_MASS_TIME 50

unsigned int distance=0;
unsigned int delay=0;
unsigned int initDelay=3000; //60sec delay for start LED

void tim1Init(){ //TIM1 generate PWM with 10uS pulse per 60mS for HC-SR06 trigger
  TIM1_TimeBaseInit(1,TIM1_COUNTERMODE_UP,60000,0);
  TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, 11, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_SET);
  TIM1_Cmd(ENABLE);
  TIM1_CtrlPWMOutputs(ENABLE);
}

void tim2Init(){ //TIM2 generate PWM 50Hz for servo
  TIM2_TimeBaseInit(TIM2_PRESCALER_2,20000);
  TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,1500, TIM2_OCPOLARITY_HIGH); //500-2000
  TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);
  TIM2_OC3PreloadConfig(ENABLE);
  TIM2_Cmd(ENABLE);
}

void gpioInit(){
  GPIO_Init(ECHO_PORT,ECHO_PIN,GPIO_MODE_IN_FL_IT);
  GPIO_Init(SERVO_PORT,SERVO_PIN,GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(SW_UP_PORT,SW_UP_PIN,GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(SW_DOWN_PORT,SW_DOWN_PIN,GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(LED_PORT,LED_PIN,GPIO_MODE_OUT_PP_LOW_SLOW);
}

void interruptConfig(){
  EXTI_SetExtIntSensitivity(ECHO_EXTI_PORT,EXTI_SENSITIVITY_FALL_ONLY);
  __enable_interrupt();
}

void moveServo(){
  if(delay>(OPEN_TIME-CAP_MASS_TIME)){
    TIM2_SetCompare3(500);
    GPIO_WriteHigh(SERVO_PORT,SERVO_PIN);
  }else 
        if(delay>50) GPIO_WriteLow(SERVO_PORT,SERVO_PIN);
        else{
          TIM2_SetCompare3((50-delay)*20+700);
          GPIO_WriteHigh(SERVO_PORT,SERVO_PIN);
        }
}

void main() {
  
  //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  gpioInit();
  tim1Init();
  tim2Init();
  interruptConfig();
  while (1) {
    
  } 
}

char distanceCntr=5;
INTERRUPT_HANDLER(ECHO_IRQ_NAME, ECHO_IRQ_NUM)
{
  unsigned int newDistance=0;
  newDistance=(TIM1_GetCounter()-550)/58;
  if(newDistance>11) distance=newDistance; 
  if(distance<ACTIVE_DISTANCE) distanceCntr--;
    else distanceCntr=5;
    if(!distanceCntr){
      if(!delay) delay=OPEN_TIME; //TODO buttons
      if(delay<OPEN_TIME-CAP_MASS_TIME) delay=OPEN_TIME-CAP_MASS_TIME;
    }
}

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13){
  if(initDelay)
    if(!--initDelay) GPIO_WriteHigh(LED_PORT,LED_PIN);
  if(delay){
    moveServo();
    delay--;
  } else{
    GPIO_WriteLow(SERVO_PORT,SERVO_PIN);
  }
  TIM2_ClearITPendingBit(TIM2_FLAG_UPDATE);
}