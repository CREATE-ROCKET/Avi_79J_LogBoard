// CAN CREATE Library

#include "CAN.h"

//----------------------------
// return 0:ACK ERROR
//----------------------------
uint8_t CAN_CREATE::sendPacket(int id, char data) {
    // set data
    beginPacket(id);
    write(data);
    //send data
    return endPacket();
}

int CAN_CREATE::available(){
    int packetSize = parsePacket();// Datafield value (DLC)
    if (packetSize == 0){
        return 0;
    }
    if(packetRtr()){// RTR 0:data 1:remote
        return 0;
    }
    return CANControllerClass::available();//  number of bytes available for reading
}

int CAN_CREATE::read()
{
  if (!CANControllerClass::available()) {
    return -1;
  }

  return _rxData[_rxIndex++];
}