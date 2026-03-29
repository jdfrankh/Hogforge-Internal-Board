#include "I2CBus.h"

void I2CBus::init(int _SDA , int _SCL ){

    Wire.setPins(_SDA, _SCL);
    Wire.onReceive(onReceiveStatic);
    Wire.onRequest(onRequestStatic);
    Wire.begin((uint8_t)thisAddress); // join i2c bus

}

//============

// Static instance pointer used by the static callback wrappers
namespace {
    static I2CBus* s_instance = nullptr;
}

// Register instance so static callbacks can forward to it
// Note: if multiple instances are used, this will only forward to the last constructed instance.
I2CBus::I2CBus(){
    s_instance = this;
}

// Forwarding static callbacks
void I2CBus::onReceiveStatic(int length) {
    if (!s_instance) return;


    Serial.printf("onReceive[%d]: ", length);
    Serial.println();
    int i = 0;
    std::vector<int> dataVec;
    
    // We should receive a 7 byte address
    if(length == 7 || length == 6){
        while (Wire.available()) {
            int highByte = Wire.read() - '0';
            int lowByte = Wire.read() - '0';

            dataVec.push_back(((highByte * 10) + lowByte));
        // Serial.printf("%d : %d : %d ", highByte, lowByte, dataVec[i]);
        // Serial.println();
        // s_instance->data[i] = Wire.read();
        // Serial.write(s_instance->data[i]);
            i++;

            Serial.printf("%d %d ", highByte, lowByte);
            Wire.read(); // There is an additional bit stored into the system for some reaosn
        }
        Serial.println(); 
        s_instance->receivedQueue.push(dataVec);
        Serial.printf(" Queue Size: %d", s_instance->receivedQueue.size());
        Serial.println();
    }
}

void I2CBus::onRequestStatic() {
    if (!s_instance) return;

    s_instance->debugPackets++;

    for(int i = 0; i < 16; i++){
        Wire.print(s_instance->allData[i]); 
    }
    

    /* Original i2c send
    Wire.print(s_instance->debugPackets++);
    Wire.print(" P.");
    Serial.println("onRequest");
    */ 
    /*
    // When master requests data, write a simple status or the txData buffer.
    // This is a minimal implementation — adapt to the protocol you need.
    uint8_t outBuf[32];
    int outLen = 0;
    for (int i = 0; i < 16 && outLen < (int)sizeof(outBuf); ++i) {
        outBuf[outLen++] = (uint8_t)(s_instance->txData.valueSent[i] & 0xFF);
    }
    // Use Wire.write to send bytes
    Wire.write(outBuf, outLen);
    */
}

bool I2CBus::isNewData(){
    if(!s_instance) return false;
    //Serial.println("Finding new data");

    return(!s_instance->receivedQueue.empty());
}

std::vector<int> I2CBus::getNewData(){
    s_instance->topData = s_instance->receivedQueue.front();
    s_instance->receivedQueue.pop();

    return s_instance->topData;
}