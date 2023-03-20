#ifndef __ROBOT_H
#define __ROBOT_H

#include "stm32l1xx_hal.h"

#define MAX_SPEED 100

extern struct Robot g_robot;

enum StatusGPIOPin { GPIO_PIN_UP, GPIO_PIN_DOWN};
enum StatusMotor { MOTOR_STOPPED, MOTOR_FORWARD, MOTOR_BACKWARD };
enum StatusUltrasound { ULTRASOUND_STOPPED, ULTRASOUND_TRIGGER_START, ULTRASOUND_TRIGGER_ON, ULTRASOUND_TRIGGER_SENT, ULTRASOUND_MEASURING };
enum StatusBuzzer { BUZZER_OFF, BUZZER_ON, BUZZER_BEEPING };
enum StatusDelay {DELAY_START, DELAY_WAITING, DELAY_OFF };
enum StatusRobot { ROBOT_STOPPED, ROBOT_FORWARD, ROBOT_BACKWARD, ROBOT_RIGHT, ROBOT_LEFT, ROBOT_BACKWARD_RIGHT, ROBOT_BACKWARD_LEFT};
enum StatusObstacle { OBSTACLE_NONE, OBSTACLE_IN_FRONT, OBSTACLE_RIGHT, OBSTACLE_RIGHT_MEASURE, OBSTACLE_RIGHT_BACK,
    OBSTACLE_LEFT, OBSTACLE_LEFT_MEASURE, OBSTACLE_LEFT_BACK };

struct GPIOPin {
  GPIO_TypeDef *gpio;
  unsigned char pin;
};

struct Motor {
  struct GPIOPin pin_speed;
  struct GPIOPin pin_direction;
  unsigned char channel;
};

struct Ultrasound {
  enum StatusUltrasound status;
  struct GPIOPin *trigger;
  struct GPIOPin *echo;
  uint16_t time_init;
  int distance;
};

struct Buzzer {
  enum StatusBuzzer status;
  struct GPIOPin *gpio_pin;
};

struct Robot {
  enum StatusRobot status;
  enum StatusObstacle status_obstacle;
  enum StatusDelay delay;
  unsigned char speed;
  struct Motor *motor_right;
  struct Motor *motor_left;
  struct Buzzer *buzzer;
  struct Ultrasound *ultrasound;
};

void createRobot(void);

void updateStatusGPIOPin(struct GPIOPin *gpio_pin, enum StatusGPIOPin status);
void toggleGPIOPin(struct GPIOPin *gpio_pin);

void updateStatusBuzzer(enum StatusBuzzer status);
void updateBuzzer();

void updateStatusRobot(enum StatusRobot status);
void updateSpeedRobot(unsigned char speed);

void updateRobot();



#endif /* __ROBOT_H */
