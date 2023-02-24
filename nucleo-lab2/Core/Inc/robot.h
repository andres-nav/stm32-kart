#ifndef __ROBOT_H
#define __ROBOT_H

#include "stm32l1xx_hal.h"

extern struct Robot g_robot;

enum StatusGPIOPin { GPIO_PIN_UP, GPIO_PIN_DOWN };
enum StatusMotor { MOTOR_STOPPED, MOTOR_FORWARD, MOTOR_BACKWARD };
enum StatusRobot { ROBOT_STOPPED, ROBOT_FORWARD, ROBOT_BACKWARD, ROBOT_RIGHT, ROBOT_LEFT };

// TODO Remove status of gpiopin and status

struct GPIOPin {
  GPIO_TypeDef *gpio;
  char pin;
};

struct Timer {
  TIM_TypeDef *tim;
  char pin;
};

struct Motor {
  struct GPIOPin pin_1;
  struct GPIOPin pin_2;
};

struct Ultrasound {
  struct Timer trigger;

};

struct Robot {
  enum StatusRobot status;
  struct Motor motor_right;
  struct Motor motor_left;
  struct GPIOPin buzzer;
  struct Ultrasound ultrasound;
};

void createRobot();

void initGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin);
void initTimer(struct Timer *timer, TIM_TypeDef *tim, char pin);

void initMotor(struct Motor *motor, struct GPIOPin *pin_1, struct GPIOPin *pin_2);

void initUltrasound(struct Ultrasound *ultrasound);

void updateStatusRobot(enum StatusRobot status);
void updateRobot();

void updateStatusMotor(struct Motor *motor, enum StatusMotor status);
void updateMotor(struct Motor *motor);

void updateStatusGPIOPin(struct GPIOPin *gpio_pin, enum StatusGPIOPin status);

#endif /* __ROBOT_H */
