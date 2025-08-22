#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

// Pins (assume mapped appropriately)


// Constants
const int setpoint = 2000;
const float kp = 8.0;
const float kd = 6.0;
const TickType_t controlTaskPeriod = pdMS_TO_TICKS(10);

// Variables
int prevDisplacement = 0;
volatile int encoderCount = 0;

// Task handles
TaskHandle_t controlTaskHandle;

// Function prototypes
//void SystemClock_Config(void);
//void GPIO_Init(void);

//int map(int x, int in_min, int in_max, int out_min, int out_max) ;
void ADC1_Init(void);
void PWM_Init(void);
void controlTask(void * argument);
//void EXTI2_IRQHandler(void);
//void EXTI3_IRQHandler(void);

int main(void) {
    // System configuration
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    ADC1_Init();
    PWM_Init();

    // Create FreeRTOS tasks
    xTaskCreate(controlTask, "Control Task", 128, NULL, 1, &controlTaskHandle);

    // Start scheduler
    vTaskStartScheduler();

    while (1) {}
}


void ADC1_Init(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC1
    ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC1
    ADC1->SMPR2 |= ADC_SMPR2_SMP0; // Sample time for channel 0
    ADC1->SQR3 |= ADC_SQR3_SQ1_0; // Channel 0 is first in the regular sequence

    // Calibrate ADC
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL); // Wait for calibration to complete
}

void PWM_Init() {
     // Enable TIM3 clock
     RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

     // Set prescaler value
     TIM3->PSC = 0; // No prescaler

     // Set auto-reload register for 1 kHz PWM frequency
     TIM3->ARR = 4000;

     // Configure TIM3 Channel 2 in PWM mode 1
     TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1 ****OC1M
     TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Enable output compare preload **** OC1PE

     // Enable capture/compare2 output
     TIM3->CCER |= TIM_CCER_CC1E;

     // Configure TIM3 for auto-reload preload and center-aligned mode 2
     TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CMS_1;

     // Generate an update event to load the prescaler value
     TIM3->EGR |= TIM_EGR_UG;

     // Enable TIM3
     TIM3->CR1 |= TIM_CR1_CEN;
 }


void controlTask(void * argument) {
    for(;;) {
        // Start ADC conversion
        ADC1->CR2 |= ADC_CR2_SWSTART; // Start ADC conversion
        while (!(ADC1->SR & ADC_SR_EOC)); // Wait for conversion to complete
        int potentiometerDisplacement = ADC1->DR; // Read ADC value

         // Read current angular displacement from the encoder
        int error = setpoint - potentiometerDisplacement; // Calculate error
        int derivative = error - prevDisplacement; // Correctly calculate the derivative term
        int16_t controlSignal = kp * error + kd * derivative; // Calculate control signal
        int motorSpeed = controlSignal*265/65535; // Map the control signal to motor speed

        if (motorSpeed < 0)
        	motorSpeed = 0;
        if (motorSpeed > 255)
        	motorSpeed = 255;

        TIM2->CCR2 = motorSpeed; // Output PWM signal to motor

        // Motor Driver control
        if (error > 0) {
            GPIOA->ODR|=(1<<2);
            GPIOA->ODR&=~(1<<3);
        } else if (error < 0) {
        	GPIOA->ODR&=~(1<<2);
        	GPIOA->ODR|=(1<<3);
        } else {
        	GPIOA->ODR|=(1<<2);
        	GPIOA->ODR|=(1<<3);
        }

        prevDisplacement = error; // Update previous error
        vTaskDelay(controlTaskPeriod); // Delay to control task period
    }
}


