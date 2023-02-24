#include "robot.h"

struct Robot g_robot;

static struct GPIOPin s_pin_1_motor_right, s_pin_2_motor_right;
static struct Motor s_motor_right;

static struct GPIOPin s_pin_1_motor_left, s_pin_2_motor_left;
static struct Motor s_motor_left;

static struct GPIOPin s_buzzer;

static struct Ultrasound s_ultrasound;

/*
 * Creates a new robot and initializes all global and static variables
 */
void createRobot() {
  // Motor right is set to the output of the left driver (due to the placement
  // of the driver)
  initGPIOPin(&s_pin_1_motor_right, GPIOB, 9);
  initGPIOPin(&s_pin_2_motor_right, GPIOA, 12);
  initMotor(&s_motor_right, &s_pin_1_motor_right, &s_pin_2_motor_right);
  g_robot.motor_right = s_motor_right;

  // Motor left is set to the output of the right driver (due to the placement
  // of the driver)
  initGPIOPin(&s_pin_1_motor_left, GPIOB, 8);
  initGPIOPin(&s_pin_2_motor_left, GPIOA, 11);
  initMotor(&s_motor_left, &s_pin_1_motor_left, &s_pin_2_motor_left);
  g_robot.motor_left = s_motor_left;

  initGPIOPin(&s_buzzer, GPIOA, 1);
  g_robot.buzzer = s_buzzer;

  initUltrasound(&s_ultrasound);

  updateStatusRobot(ROBOT_STOPPED);
}

/*
 * Initializes the basic structure of a gpio pin with the default speed
 */
void initGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin) {
  gpio_pin->gpio = gpio;
  gpio_pin->pin = pin;

  gpio->MODER &= ~(1 << (pin * 2 + 1));
  gpio->MODER |= (1 << (pin * 2));

  gpio->OTYPER &= ~(1 << pin);

  gpio->OSPEEDR &= ~(1 << (pin * 2 + 1));
  gpio->OSPEEDR &= ~(1 << (pin * 2));
}

/*
 * Initializes the timer based on its characteristics
 */
void initTimer(struct Timer *timer, TIM_TypeDef *tim, char pin) {
  // TODO join the init gpio and timer with a common enum in TIM_TypeDef
  timer->tim = tim;
  timer->pin = pin;
}

/*
 * Initializes the motor with the corresponding pins.
 */
void initMotor(struct Motor *motor, struct GPIOPin *pin_1,
               struct GPIOPin *pin_2) {
  motor->pin_1 = *pin_1;
  motor->pin_2 = *pin_2;
}

void initUltrasound(struct Ultrasound *ultrasound) {

}

/*
 * Updates the status of the motor and calls to implement the status.
 */
void updateStatusRobot(enum StatusRobot status) {
  g_robot.status = status;
  updateRobot();
}

/*
 * Updates the robot to its according status
 *  All the movements are with respect to the whole robot.
 */
void updateRobot() {
  enum StatusMotor status_motor_right, status_motor_left;

  switch (g_robot.status) {
  case ROBOT_STOPPED:
    status_motor_right = MOTOR_STOPPED;
    status_motor_left = MOTOR_STOPPED;

    break;
  case ROBOT_FORWARD:
    status_motor_right = MOTOR_FORWARD;
    status_motor_left = MOTOR_FORWARD;

    break;
  case ROBOT_BACKWARD:
    status_motor_right = MOTOR_BACKWARD;
    status_motor_left = MOTOR_BACKWARD;

    break;
  case ROBOT_RIGHT:
    status_motor_right = MOTOR_BACKWARD;
    status_motor_left = MOTOR_FORWARD;

    break;
  case ROBOT_LEFT:
    status_motor_right = MOTOR_FORWARD;
    status_motor_left = MOTOR_BACKWARD;

    break;
  }

  updateStatusMotor(&(g_robot.motor_right), status_motor_right);
  updateStatusMotor(&(g_robot.motor_left), status_motor_left);
}

/*
 * Updates the motor depending of its status
 *  MOTOR_STOPPED: sets the motor to stop
 *  MOTOR_FORWARD: sets the motor to forward with respect to the whole robot
 *  MOTOR_BACKWARD: sets the motor to backward with respect to the whole robot
 *
 *  This is important as one motor cables are swapped to correct that it is
 * flipped. (hardware)
 */
void updateStatusMotor(struct Motor *motor, enum StatusMotor status) {
  enum StatusGPIOPin status_gpio_pin_1, status_motor_pin_2;

    switch (status) {
    case MOTOR_STOPPED:
      status_gpio_pin_1 = GPIO_PIN_UP;
      status_motor_pin_2 = GPIO_PIN_UP;

      break;
    case MOTOR_FORWARD:
      status_gpio_pin_1 = GPIO_PIN_UP;
      status_motor_pin_2 = GPIO_PIN_DOWN;

      break;
    case MOTOR_BACKWARD:
      status_gpio_pin_1 = GPIO_PIN_DOWN;
      status_motor_pin_2 = GPIO_PIN_UP;

      break;
    }

    updateStatusGPIOPin(&(motor->pin_1), status_gpio_pin_1);
    updateStatusGPIOPin(&(motor->pin_2), status_motor_pin_2);
}

/*
 * Updates the motor pin depending of its status
 *  GPIO_PIN_UP: sets the BSRR register to set the pin to 1
 *  GPIO_PIN_DOWN: sets the BSRR register to set the pin to 0
 */
void updateStatusGPIOPin(struct GPIOPin *gpio_pin, enum StatusGPIOPin status) {
  switch (status) {
    case GPIO_PIN_UP:
      gpio_pin->gpio->BSRR |= (1 << gpio_pin->pin);

      break;
    case GPIO_PIN_DOWN:
      gpio_pin->gpio->BSRR |= (1 << gpio_pin->pin) << 16;

      break;
    }
}
