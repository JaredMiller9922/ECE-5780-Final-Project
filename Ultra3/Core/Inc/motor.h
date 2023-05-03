
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */



/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the entire rover motor drive system
void motor_init(void);

// Set the duty cycle of the PWM for MTR_DRV_1, accepts (0-100)
void pwm_setDutyCycle_LMTR(uint8_t duty);
void pwm_setDutyCycle_RMTR(uint8_t duty);

// These methods control the rovers direction
void reverse(void);
void forward(void);
void rotate90Left(void);
void rotate90Right(void);

void rotate45Left(void);
void rotate45Right(void);
void rotate180Left(void);
void rotate180Right(void);

void LED_init(void);

/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void);


#endif /* MOTOR_H_ */
