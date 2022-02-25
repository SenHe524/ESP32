#include <Arduino.h>
int interruptCounter = 0;
hw_timer_t *timer = NULL;

void IRAM_ATTR TimerEvent()
{
    Serial.println(interruptCounter++);
    if(interruptCounter > 5)
    {
        interruptCounter = 1;
    }

}
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    timer = timerBegin(0,80,true);
    timerAttachInterrupt(timer, &TimerEvent, true);
    timerAlarmWrite(timer, 1000000,true);
    timerAlarmEnable(timer);

}
void loop() {
  // put your main code here, to run repeatedly:
}