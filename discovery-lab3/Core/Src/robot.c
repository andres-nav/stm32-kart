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

static struct GPIOPin s_pin_speed_selector;
static struct SpeedSelector s_speed_selector;

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

  TIM4->PSC = TIMER_4_PSC - 1; // T = 2 ms
  TIM4->CNT = 0;
  TIM4->ARR = MAX_SPEED - 1; // Tpwm = 1s
  TIM4->CCR3 = MAX_SPEED;
  TIM4->CCR4 = MAX_SPEED; // DC = 10%

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
  s_motor_right.status = MOTOR_FORWARD;
  g_robot.motor_right = &s_motor_right;

  // Motor left is set to the output of the right driver (due to the placement
  // of the driver)
  initAFGPIOPin(&s_pin_speed_motor_left, GPIOB, 8, 2);
  initOutputGPIOPin(&s_pin_direction_motor_left, GPIOA, 11);
  s_motor_left.pin_speed = s_pin_speed_motor_left;
  s_motor_left.pin_direction = s_pin_direction_motor_left;
  s_motor_left.channel = 3;
  s_motor_right.status = MOTOR_FORWARD;
  g_robot.motor_left = &s_motor_left;

  g_robot.speed = MAX_SPEED;
  g_robot.status = ROBOT_FORWARD;
  g_robot.status_obstacle = OBSTACLE_NONE;
  g_robot.delay = DELAY_OFF;

  initTimer4();
}

static void initTimer2(void) {
  // ------------- Echo Timer (Channel 1) ---------------
  TIM2->CR1 = 0x0000;
  TIM2->CR2 = 0x0000;
  TIM2->SMCR = 0x0000;

  TIM2->PSC = TIMER_2_PSC - 1;
  TIM2->CNT = 0;
  TIM2->ARR = 0xFFFF;

  TIM2->DIER |= (1 << 1); // IRQ when CCR1 is reached -> CCyIE = 1

  TIM2->CCMR1 = 0x0001;  // CCyS = 1 (TIC); OCyM = 000 y OCyPE = 0 (always in TIC)

  TIM2->CCER = 0x0001; // CCyNP:CCyP = 11 (rising and falling edge active)
  TIM2->CCER |= (1 << 1);
  TIM2->CCER |= (1 << 3);

  // ------------- Trigger Timer (Channel 2) -----------
  TIM2->CCR2 = TIMER_2_CH_2_CNT;

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

  TIM3->PSC = TIMER_3_PSC - 1;
  TIM3->CNT = 0;
  TIM3->ARR = TIMER_3_CH_1_CNT;
  TIM3->CCR1 = TIMER_3_CH_1_CNT;
  TIM3->CCR2 = TIMER_3_CH_2_CNT;
  TIM3->CCR3 = TIMER_3_CH_3_CNT;

  TIM3->DIER |= (1 << 1); // IRQ when CCR1 is reached -> CCyIE = 1
  TIM3->DIER |= (1 << 2);
  TIM3->DIER |= (1 << 3);

  TIM3->CCMR1 &= ~(0xFFFF);
  TIM3->CCMR1 |= 0x3030;    // CC1S = 0 (TOC, PWM) OC1M = 011 (Toggle) OC1PE = 0  (without preload)

  TIM3->CCMR2 &= ~(0x00FF);
  TIM3->CCMR2 |= 0x0030;

  TIM3->CCER &= ~(0x0FFF);
  TIM3->CCER |= 0x0111; // CC1P = 0 (always) CC1E = 1   (hardware output activated)

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
  s_ultrasound.distance = 100;
  g_robot.ultrasound = &s_ultrasound;

  initTimer2();
  initTimer3();
}

static void initSpeedSelectorModule(void) {
  initGPIOPin(&s_pin_speed_selector, GPIOA, 4);
  s_speed_selector.adc = ADC1;

  s_pin_speed_selector.gpio->MODER |= (3 << (s_pin_speed_selector.pin * 2));

  s_speed_selector.adc->CR2 &= ~(1 << 0);
  s_speed_selector.adc->CR1 = 0;  // OVRIE = 0 (overrun IRQ disabled) RES = 00 (resolution = 12 bits) SCAN = 0 (scan mode disabled) EOCIE = 0 (EOC IRQ disabled)
  s_speed_selector.adc->CR2 = 0x00000412; // EOCS = 1 (EOC is activated after each conv.) DELS = 001 (delay till data is read) CONT = 1 (continuous conversion)

  s_speed_selector.adc->SQR1 = 0x00000000;       // 1 channel in the sequence
  s_speed_selector.adc->SQR5 = 0x00000004;       // The selected channel is AIN4

  s_speed_selector.adc->CR2 |= (1 << 0);       // ADON = 1 (ADC powered on)

  while ((s_speed_selector.adc->SR & 0x0040) == 0);  // If ADCONS = 0, I wait till converter is ready
  s_speed_selector.adc->CR2 |= 0x40000000;       // When ADCONS = 1, I start conv. (SWSTART = 1)

  s_speed_selector.gpio_pin = &s_pin_speed_selector;
  s_speed_selector.max_speed = MAX_SPEED;

  g_robot.speed_selector = s_speed_selector;
}

static void updateStatusBuzzer(enum StatusBuzzer status) {
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
  if (motor->status == status) {
    return;
  }

  motor->status = status;

  enum StatusGPIOPin status_motor_pin_direction;
  unsigned char offset_enable = ((motor->channel - 1) * 4);
  unsigned char offset_pwm_mode = ((motor->channel - 3) * 8) + 4;

  switch (status) {
  case MOTOR_STOPPED:
    TIM4->CCER &= ~(11 << offset_enable); // Turn off PWM
    status_motor_pin_direction = GPIO_PIN_DOWN;
    TIM4->CNT = 0;
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
  updateStatusGPIOPin(&(motor->pin_direction), status_motor_pin_direction);
}

/*
 * Updates the status of the motor and calls to implement the status.
 *    All the movements are with respect to the whole robot.
 *
 */
static void updateStatusRobot(enum StatusRobot status) {
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
  case ROBOT_BACKWARD_RIGHT:
    status_motor_right = MOTOR_BACKWARD;
    status_motor_left = MOTOR_STOPPED;

    break;
  case ROBOT_BACKWARD_LEFT:
    status_motor_right = MOTOR_STOPPED;
    status_motor_left = MOTOR_BACKWARD;

    break;
  }

  // TODO enable motor
  updateStatusMotor(g_robot.motor_left, status_motor_left);
  updateStatusMotor(g_robot.motor_right, status_motor_right);

}

static void updateSpeedRobot(unsigned char speed) {
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




/*
 * Creates a new robot and initializes all global and static variables
 */
void createRobot(void) {
  initDriveModule();
  initUltrasonicAndBuzzerModule();
  initSpeedSelectorModule();

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

void updateBuzzer() {
  enum StatusBuzzer status_buzzer;

  if (g_robot.ultrasound->distance < SHORT_DISTANCE) {
    status_buzzer = BUZZER_ON;
  } else if (g_robot.ultrasound->distance < LONG_DISTANCE) {
    status_buzzer = BUZZER_BEEPING;
  } else {
    status_buzzer = BUZZER_OFF;
  }

  updateStatusBuzzer(status_buzzer);
}

void updateMaxSpeed() {
  uint32_t value = g_robot.speed_selector.adc->DR;

  float percentage = (((float) value * 0.5) / MAX_VALUE_ADC) + 0.5;

  g_robot.speed_selector.max_speed = (int) (MAX_SPEED * percentage);
}

/*
 * It uses a TURN_SPEED so the angle is constant for a 0.5 second delay
 * Otherwise depending on the speed will make different angle of turns.
 */
void updateRobot() {
  enum StatusRobot status_robot = ROBOT_STOPPED;
  unsigned char speed = 0;
  unsigned char do_wait = 0;

  switch(g_robot.status_obstacle) {
  case OBSTACLE_NONE:
    if (g_robot.ultrasound->distance < LONG_DISTANCE) {
      /*
       * SHORT_DISTANCE + 2 is done so when the robot is at 10 the speed is not 0
       * Therefore the speed is from 20% to 100% the maximum
       */
      float percentage = (g_robot.ultrasound->distance - (SHORT_DISTANCE + 2)) * 0.1;
      speed = (int) (g_robot.speed_selector.max_speed * percentage) + MIN_SPEED;
    } else {
      speed = g_robot.speed_selector.max_speed;
    }

    if (speed < MIN_SPEED) {
      speed = MIN_SPEED;
    }

    status_robot = ROBOT_FORWARD;
    do_wait = 0;

    break;

  case OBSTACLE_IN_FRONT:
  case OBSTACLE_RIGHT_MEASURE:
  case OBSTACLE_LEFT_MEASURE:
  case OBSTACLE_FINAL:
    speed = 0;
    status_robot = ROBOT_STOPPED;
    do_wait = 1;
    break;

  case OBSTACLE_RIGHT:
    speed = TURN_SPEED;
    status_robot = ROBOT_BACKWARD_RIGHT;
    do_wait = 1;
    break;

  case OBSTACLE_RIGHT_BACK:
  case OBSTACLE_LEFT_BACK:
    speed = TURN_SPEED;
    status_robot = ROBOT_LEFT;
    do_wait = 1;
    break;

  case OBSTACLE_LEFT:
    speed = TURN_SPEED;
    status_robot = ROBOT_BACKWARD_LEFT;
    do_wait = 1;
    break;

  }

  updateSpeedRobot(speed);
  updateStatusRobot(status_robot);

  while (do_wait != 0) {
    g_robot.delay = DELAY_START;
    while(g_robot.delay != DELAY_OFF);
    do_wait --;
  }
}
