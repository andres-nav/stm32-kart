#include "robot.h"

struct Robot g_robot;

static struct GPIOPin s_pin_1_motor_right, s_pin_2_motor_right;
static struct Motor s_motor_right;

static struct GPIOPin s_pin_1_motor_left, s_pin_2_motor_left;
static struct Motor s_motor_left;

static struct GPIOPin s_pin_buzzer;

static struct GPIOPin s_pin_ultrasound_trigger;
static struct Ultrasound s_ultrasound;

/*
 * Creates a new robot and initializes all global and static variables
 */
void createRobot() {
  // Motor right is set to the output of the left driver (due to the placement
  // of the driver)
  initOutputGPIOPin(&s_pin_1_motor_right, GPIOB, 9);
  initOutputGPIOPin(&s_pin_2_motor_right, GPIOA, 12);
  s_motor_right.pin_1 = s_pin_1_motor_right;
  s_motor_right.pin_2 = s_pin_2_motor_right;
  g_robot.motor_right = s_motor_right;

  // Motor left is set to the output of the right driver (due to the placement
  // of the driver)
  initOutputGPIOPin(&s_pin_1_motor_left, GPIOB, 8);
  initOutputGPIOPin(&s_pin_2_motor_left, GPIOA, 11);
  s_motor_left.pin_1 = s_pin_1_motor_left;
  s_motor_left.pin_2 = s_pin_2_motor_left;
  g_robot.motor_left = s_motor_left;

  // Initializes the buzzer
  initOutputGPIOPin(&s_pin_buzzer, GPIOA, 1);
  g_robot.buzzer = s_pin_buzzer;

  // Initializes the ultrasonic sensor
  initAFGPIOPin(&s_pin_ultrasound_trigger, GPIOB, 6, 2);
  s_ultrasound.trigger = &s_pin_ultrasound_trigger;
  s_ultrasound.timer = TIM4; // will use PA5 that is associated to TIM2_CH1
  TIM4->CR1 = 0x0000; // ARPE = 0, CEN = 0
  TIM4->CR2 = 0x0000;
  TIM4->SMCR = 0x0000;
  TIM4->PSC = 32000 - 1;
  TIM4->CNT = 0;
  TIM4->ARR = 0xFFFF;
  TIM4->CCR1 = 1000;
  TIM4->DIER |= (1 << 1); // channel 1
  TIM4->CCMR1 &= ~(0x00FF); // CCyS = 0; OCyM = 000; OCyPE = 0
  TIM4->CCMR1 |= 0x0030;
  TIM4->CCER &= ~(0x000F); // CCyP = 0; CCyE = 0
  TIM4->CCER |= 0x0001;

  //enableling the counter
  TIM4->CR1 |= 0x0001;
  TIM4->EGR |= 0x0001;
  TIM4->SR = 0;

  NVIC->ISER[0] |= (1 << 30);
}

/*
 * Initializes the basic structure of a gpio pin with the default speed
 */
void initGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin) {
  gpio_pin->gpio = gpio;
  gpio_pin->pin = pin;

  gpio->OTYPER &= ~(1 << pin);

  gpio->OSPEEDR &= ~(1 << (pin * 2 + 1));
  gpio->OSPEEDR &= ~(1 << (pin * 2));
}

void initOutputGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin) {
  initGPIOPin(gpio_pin, gpio, pin);

  // 01 the register
  gpio->MODER &= ~(1 << (pin * 2 + 1));
  gpio->MODER |= (1 << (pin * 2));
}

void initAFGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin, char af) {
  initGPIOPin(gpio_pin, gpio, pin);

  // 10 the register
  gpio->MODER |= (1 << (pin * 2 + 1));
  gpio->MODER &= ~(1 << (pin * 2));

  char afr = pin < 8 ? 0 : 1;
  gpio->AFR[afr] |= (af << (pin * 4));
}


/*
 * Updates the status of the motor and calls to implement the status.
 *    All the movements are with respect to the whole robot.
 *
 */
void updateStatusRobot(enum StatusRobot status) {
  enum StatusMotor status_motor_right, status_motor_left;

  switch (status) {
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
