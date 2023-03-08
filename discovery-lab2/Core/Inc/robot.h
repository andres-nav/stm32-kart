#ifndef __ROBOT_H
#define __ROBOT_H

#include "stm32l1xx_hal.h"

extern struct Robot g_robot;

enum StatusGPIOPin { GPIO_PIN_UP, GPIO_PIN_DOWN};
enum StatusMotor { MOTOR_STOPPED, MOTOR_FORWARD, MOTOR_BACKWARD };
enum StatusUltrasound { ULTRASOUND_STOPPED, ULTRASOUND_TRIGGER_START, ULTRASOUND_TRIGGER_ON, ULTRASOUND_TRIGGER_SENT, ULTRASOUND_MEASURING };
enum StatusUltrasoundDistance { DISTANCE_CHANGED, DISTANCE_DID_NOT_CHANGE };
enum StatusBuzzer { BUZZER_OFF, BUZZER_ON, BUZZER_BEEPING };
enum StatusRobot { ROBOT_STOPPED, ROBOT_FORWARD, ROBOT_BACKWARD, ROBOT_RIGHT, ROBOT_LEFT };

struct GPIOPin {
  GPIO_TypeDef *gpio;
  unsigned char pin;
};

struct Motor {
  struct GPIOPin pin_1;
  struct GPIOPin pin_2;
};

struct Ultrasound {
  enum StatusUltrasound status;
  struct GPIOPin *trigger;
  struct GPIOPin *echo;
  uint16_t time_init;
  enum StatusUltrasoundDistance status_distance;
  int distance;
};

struct Buzzer {
  enum StatusBuzzer status;
  struct GPIOPin *gpio_pin;
};

struct Robot {
  struct Motor *motor_right;
  struct Motor *motor_left;
  struct Buzzer *buzzer;
  struct Ultrasound *ultrasound;
};

void createRobot(void);

void updateStatusGPIOPin(struct GPIOPin *gpio_pin, enum StatusGPIOPin status);
void toggleGPIOPin(struct GPIOPin *gpio_pin);

void updateStatusBuzzer(enum StatusBuzzer status);
void updateStatusRobot(enum StatusRobot status);



#endif /* __ROBOT_H */
