#include "robot.h"

struct Robot g_robot;

static struct GPIOPin s_pin_speed_motor_right, s_pin_direction_motor_right;
static struct Motor s_motor_right;

static struct GPIOPin s_pin_speed_motor_left, s_pin_direction_motor_left;
static struct Motor s_motor_left;

static struct GPIOPin s_pin_buzzer;
static struct Buzzer s_buzzer;

static struct GPIOPin s_pin_ultrasound_trigger, s_pin_ultrasound_echo;
static struct Ultrasound s_ultrasound;

/*
 * Initializes the basic structure of a gpio pin with the default speed
 */
static void initGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin) {
  gpio_pin->gpio = gpio;
  gpio_pin->pin = pin;

  gpio->OTYPER &= ~(1 << pin);

  gpio->OSPEEDR &= ~(1 << (pin * 2 + 1));
  gpio->OSPEEDR &= ~(1 << (pin * 2));
}

static void initOutputGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin) {
  initGPIOPin(gpio_pin, gpio, pin);

  // 01 the register
  gpio->MODER &= ~(1 << (pin * 2 + 1));
  gpio->MODER |= (1 << (pin * 2));
}

static void initAFGPIOPin(struct GPIOPin *gpio_pin, GPIO_TypeDef *gpio, char pin, char af) {
  initGPIOPin(gpio_pin, gpio, pin);

  // 10 the register
  gpio->MODER |= (1 << (pin * 2 + 1));
  gpio->MODER &= ~(1 << (pin * 2));

  unsigned char afr = pin < 8 ? 0 : 1;
  pin = (pin - (8 * afr));
  gpio->AFR[afr] &= ~(0xF << (pin * 4));
  gpio->AFR[afr] |= (af << (pin * 4));
}

static void initTimer4(void) {
  // ------------- Motor Speed Timer -----------------------
  // Channel 3 for PB8
  // Channel 4 for PB9
  TIM4->CR1 = 0x0080;
  TIM4->CR2 = 0x0000;
  TIM4->SMCR = 0x0000;

  TIM4->PSC = 64000 - 1; // T = 2 ms
  TIM4->CNT = 0;
  TIM4->ARR = MAX_SPEED - 1; // Tpwm = 1s
  TIM4->CCR3 = 40;
  TIM4->CCR4 = 40; // DC = 10%

  TIM4->CCMR2 &= ~(0xFFFF); // Clear all channel 4 information
  TIM4->CCMR2 |= 0x6868; // CCyS = 0 (TOC) OCyM = 110 (PPM starting in 1) OC1PE = 1 (Preload enable for PWM)

  TIM4->CCER |= 0xBB00; // CC4NP:CC4P = 11 (rising and falling edge active) CC4E
}

static void initDriveModule(void) {
  // Motor right is set to the output of the left driver (due to the placement
  // of the driver)
  initAFGPIOPin(&s_pin_speed_motor_right, GPIOB, 9, 2);
  initOutputGPIOPin(&s_pin_direction_motor_right, GPIOA, 12);
  s_motor_right.pin_speed = s_pin_speed_motor_right;
  s_motor_right.pin_direction = s_pin_direction_motor_right;
  s_motor_right.channel = 4;
  g_robot.motor_right = &s_motor_right;

  // Motor left is set to the output of the right driver (due to the placement
  // of the driver)
  initAFGPIOPin(&s_pin_speed_motor_left, GPIOB, 8, 2);
  initOutputGPIOPin(&s_pin_direction_motor_left, GPIOA, 11);
  s_motor_left.pin_speed = s_pin_speed_motor_left;
  s_motor_left.pin_direction = s_pin_direction_motor_left;
  s_motor_left.channel = 3;
  g_robot.motor_left = &s_motor_left;

  g_robot.speed = MAX_SPEED;
  g_robot.status = ROBOT_FORWARD;
  g_robot.status_obstacle = OBSTACLE_NONE;

  initTimer4();
}

static void initTimer2(void) {
  // ------------- Echo Timer (Channel 1) ---------------
  TIM2->CR1 = 0x0000;
  TIM2->CR2 = 0x0000;
  TIM2->SMCR = 0x0000;

  TIM2->PSC = 32 - 1;
  TIM2->CNT = 0;
  TIM2->ARR = 0xFFFF;

  TIM2->DIER |= (1 << 1); // IRQ when CCR1 is reached -> CCyIE = 1

  TIM2->CCMR1 = 0x0001;  // CCyS = 1 (TIC); OCyM = 000 y OCyPE = 0 (always in TIC)

  TIM2->CCER = 0x0001; // CCyNP:CCyP = 11 (rising and falling edge active)
  TIM2->CCER |= (1 << 1);
  TIM2->CCER |= (1 << 3);

  // ------------- Trigger Timer (Channel 2) -----------
  TIM2->CCR2 = 10;

  TIM2->DIER |= (1 << 2); // IRQ when CCR2 is reached

  TIM2->CCMR1 &= ~(0xFF00); // Clear all channel 2 information
  TIM2->CCMR1 |= 0x3000;    // CC2S = 0 (TOC, PWM) OC2M = 011 (Toggle) OC2PE = 0  (without preload)

  TIM2->CCER &= ~(0x00F0);
  TIM2->CCER |= 0x0010; // CC2P = 0   (always)

  NVIC->ISER[0] |= (1 << 28);
}

static void initTimer3(void) {
  // ------------- Toggle Buzzer Timer -----------------------
  TIM3->CR1 = 0x0000;
  TIM3->CR2 = 0x0000;
  TIM3->SMCR = 0x0000;

  TIM3->PSC = 400 - 1;
  TIM3->CNT = 0;
  TIM3->ARR = 0xFFFF;
  TIM3->CCR1 = 500;

  TIM3->DIER |= (1 << 1); // IRQ when CCR1 is reached -> CCyIE = 1

  TIM3->CCMR1 &= ~(0x00FF);
  TIM3->CCMR1 |= 0x0030;    // CC1S = 0 (TOC, PWM) OC1M = 011 (Toggle) OC1PE = 0  (without preload)

  TIM3->CCER &= ~(0x000F);
  TIM3->CCER |= 0x0001; // CC1P = 0 (always) CC1E = 1   (hardware output activated)

  NVIC->ISER[0] |= (1 << 29);
}

static void initUltrasonicAndBuzzerModule(void) {
  initOutputGPIOPin(&s_pin_buzzer, GPIOA, 1);
  s_buzzer.gpio_pin = &s_pin_buzzer;
  s_buzzer.status = BUZZER_ON;
  g_robot.buzzer = &s_buzzer;

  initOutputGPIOPin(&s_pin_ultrasound_trigger, GPIOD, 2);
  s_ultrasound.trigger = &s_pin_ultrasound_trigger;

  initAFGPIOPin(&s_pin_ultrasound_echo, GPIOA, 5, 1);
  s_ultrasound.echo = &s_pin_ultrasound_echo;

  s_ultrasound.status = ULTRASOUND_STOPPED;
  s_ultrasound.distance = 0;
  s_ultrasound.status_distance = DISTANCE_DID_NOT_CHANGE;
  g_robot.ultrasound = &s_ultrasound;

  initTimer2();
  initTimer3();
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

/*
 * Updates the motor depending of its status
 *  MOTOR_STOPPED: sets the motor to stop
 *  MOTOR_FORWARD: sets the motor to forward with respect to the whole robot
 *  MOTOR_BACKWARD: sets the motor to backward with respect to the whole robot
 *
 *  This is important as one motor cables are swapped to correct that it is
 * flipped. (hardware)
 */
static void updateStatusMotor(struct Motor *motor, enum StatusMotor status) {
  enum StatusGPIOPin status_motor_pin_direction;
  unsigned char offset_enable = ((motor->channel - 1) * 4);
  unsigned char offset_pwm_mode = ((motor->channel - 3) * 8) + 4;

  switch (status) {
  case MOTOR_STOPPED:
    TIM4->CCER &= ~(11 << offset_enable); // Turn off PWM
    status_motor_pin_direction = GPIO_PIN_DOWN;
    break;
  case MOTOR_FORWARD:
    TIM4->CCER |= (11 << offset_enable); // Turn on PWM
    TIM4->CCMR2 |= (1 << offset_pwm_mode); // OCyM = 111
    status_motor_pin_direction = GPIO_PIN_DOWN;
    break;
  case MOTOR_BACKWARD:
    TIM4->CCER |= (11 << offset_enable);
    TIM4->CCMR2 &= ~(1 << offset_pwm_mode); // OCyM = 110
    status_motor_pin_direction = GPIO_PIN_UP;

    break;
  }
  TIM4->CNT = 0;
  updateStatusGPIOPin(&(motor->pin_direction), status_motor_pin_direction);
}



/*
 * Creates a new robot and initializes all global and static variables
 */
void createRobot(void) {
  initDriveModule();
  initUltrasonicAndBuzzerModule();

  TIM2->CR1 |= 0x0001; // CEN = 1 -> Start counter
  TIM2->SR = 0; // Clear flags

  TIM3->CR1 |= 0x0001; // CEN = 1 -> Start counter
  TIM3->SR = 0; // Clear flags

  TIM4->CR1 |= 0x0001;
  TIM4->SR = 0; // Clear flags

  updateSpeedRobot(0);
  updateStatusRobot(ROBOT_STOPPED);
  updateStatusBuzzer(BUZZER_OFF);
}

void toggleGPIOPin(struct GPIOPin *gpio_pin) {
  if ((gpio_pin->gpio->IDR & (1 << gpio_pin->pin)) == 0) {
    gpio_pin->gpio->BSRR |= (1 << gpio_pin->pin);
  } else {
    gpio_pin->gpio->BSRR |= (1 << gpio_pin->pin) << 16;
  }
}

void updateStatusBuzzer(enum StatusBuzzer status) {
  if (g_robot.buzzer->status == status) {
    return;
  }

  g_robot.buzzer->status = status;
  switch (g_robot.buzzer->status) {
  case BUZZER_ON:
    updateStatusGPIOPin(g_robot.buzzer->gpio_pin, GPIO_PIN_UP);
    break;
  case BUZZER_OFF:
    updateStatusGPIOPin(g_robot.buzzer->gpio_pin, GPIO_PIN_DOWN);
    break;
  case BUZZER_BEEPING:
    break;
  }
}

void updateBuzzer() {
  enum StatusBuzzer status_buzzer;

  if (g_robot.ultrasound->distance < 10) {
    status_buzzer = BUZZER_ON;
  } else if (g_robot.ultrasound->distance < 20) {
    status_buzzer = BUZZER_BEEPING;
  } else {
    status_buzzer = BUZZER_OFF;
  }

  updateStatusBuzzer(status_buzzer);
}



/*
 * Updates the status of the motor and calls to implement the status.
 *    All the movements are with respect to the whole robot.
 *
 */
void updateStatusRobot(enum StatusRobot status) {
  if (g_robot.status == status) {
    return;
  }

  g_robot.status = status;
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
    status_motor_right = MOTOR_STOPPED;
    status_motor_left = MOTOR_FORWARD;

    break;
  case ROBOT_LEFT:
    status_motor_right = MOTOR_FORWARD;
    status_motor_left = MOTOR_STOPPED;

    break;
  }

  updateStatusMotor(g_robot.motor_right, status_motor_right);
  updateStatusMotor(g_robot.motor_left, status_motor_left);
}

void updateSpeedRobot(unsigned char speed) {
  if (g_robot.speed == speed) {
    return;
  }

  if (speed > MAX_SPEED) {
    speed = MAX_SPEED;
  }

  g_robot.speed = speed;

  TIM4->CCR3 = speed;
  TIM4->CCR4 = speed;

}
