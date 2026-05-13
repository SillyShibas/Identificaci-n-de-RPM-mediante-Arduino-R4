// Functional Arduino R4 Code (load this one for serial port messages)
#include <Arduino_CAN.h>

uint8_t msg_OBD2[8] = {0x02,0x01,0x0C,0x0D,0,0,0,0};
uint32_t tant=millis();

void setup()
{
  Serial.begin(115200);
  Serial.flush();
  while (!Serial) { }
  delay(2000);

  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
}

void loop()
{
  if (CAN.available()){
    CanMsg const msgCAN = CAN.read();
    if (msgCAN.id >= 0x7DF){
      Serial.print(msgCAN);
      Serial.print(",");
      Serial.println((uint16_t)millis());
  }

  if(millis()-tant >= 100){
    CanMsg const msg(CanStandardId(0x7DF), 8, msg_OBD2);
    if (int const rc = CAN.write(msg); rc < 0){
      Serial.println ("CAN.Write(...) failed with error code");
    }
    tant = millis();  
    }


  }
}