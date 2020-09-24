#include <Arduino.h>
#include <ledc.h>

#define LEDC_CHANNEL_0     0

// use 10 bit precission for LEDC timer
#define LEDC_TIMER_10_BIT  10

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     1

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            26

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button = {18, 0, false};

volatile bool FirstFall;
volatile uint64_t before_time = 0 , duty = 0;

void IRAM_ATTR Pin_isr() {
  if (!FirstFall)
  {
    before_time = timerRead(timer);
    FirstFall = true;
  }
  else
  {
    duty = timerRead(timer) - before_time;
    timerStop(timer);
    FirstFall = false;
    button.pressed = true;
  }
  
    
}
/*************************/
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
void setup() {
  Serial.begin(115200);

  pinMode(button.PIN, INPUT_PULLUP);
  attachInterrupt(button.PIN, Pin_isr, FALLING);

  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_10_BIT);
  ledcWriteTone(LEDC_CHANNEL_0,5);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, false);
  timerAlarmEnable(timer);
}
void loop() {
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;
    Serial.print("An Timer over. Total number: ");
    Serial.println(totalInterruptCounter);
  }

    if (button.pressed) {
        Serial.printf("duty time %u \n", duty);
        button.pressed = false;
    }

}