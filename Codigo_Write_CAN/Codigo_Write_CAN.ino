// CAN WRITE

//

#include <Arduino_CAN.h>

static uint32_t const CAN_ID = 0x20;
uint8_t const msg_data[8] = {0x03,0x01,0x0C,0x0D,0,0,0}; //uint = unsigned int y 8_t = 8 bits
                                                        // leer por OBDII RPM y mensaje lineal
uint32_t t_prev = millis();


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

static uint32_t msg_cnt = 0;

void loop(){

  if (CAN.available()){
    CanMsg const msg = CAN.read();
    // si es mayor o igual que 7DF enviar el mensaje serial.println
    Serial.println(msg);  // me arrojara todos los mensajes continuos
  }

  if(millis() - t_prev>=100){

    CanMsg const msg(CanStandardId(0x111), 8, msg_data);
    if (int const rc = CAN.write(msg); rc < 0){
      
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.println(rc);
    for (;;) { }
    }
    t_prev = millis();
  }
}
/*
  memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
  CanMsg const msg(CanStandardId(0x111), 8, msg_data);



  /* Increase the message counter. 
  msg_cnt++;

  /* Only send one message per second. 
  delay(1000)
*/