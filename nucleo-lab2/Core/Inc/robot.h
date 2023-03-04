#ifndef __ROBOT_H
#define __ROBOT_H

#include "stm32l1xx_hal.h"

extern struct Robot g_robot;

enum StatusGPIOPin { GPIO_PIN_UP, GPIO_PIN_DOWN };
enum StatusMotor { MOTOR_STOPPED, MOTOR_FORWARD, MOTOR_BACKWARD };
enum StatusRobot { ROBOT_STOPPED, ROBOT_FORWARD, ROBOT_BACKWARD, ROBOT_RIGHT, ROBOT_LEFT };

struct GPIOPin {
  GPIO_TypeDef *gpio;
  char pin;
};

struct Motor {
  struct GPIOPin pin_1;
  struct GPIOPin pin_2;
};

struct Ultrasound {
  struct GPIOPin *trigger;
  TIM_TypeDef *timer;
};

struct Robot {
  struct Motor motor_right;
  struct Motor motor_left;
  struct GPIOPin buzzer;
  struct Ultrasound ultrasound;
};

void createRobot(void);

void initGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin);
void initOutputGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin);
void initAFGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin, char af);

void updateStatusRobot(enum StatusRobot status);
void updateStatusMotor(struct Motor *motor, enum StatusMotor status);
void updateStatusGPIOPin(struct GPIOPin *gpio_pin, enum StatusGPIOPin status);

#endif /* __ROBOT_H */
