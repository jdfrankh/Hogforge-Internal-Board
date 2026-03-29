#include <BuildPlates.h>
#include <LoaderBar.h>
#include <iostream>
#include <queue>
#include <vector>

        // I2C control stuff
#include <Wire.h>

//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================

// https://forum.arduino.cc/t/use-i2c-for-communication-between-arduinos/653958/4

class I2CBus{


    private:

        static void onRequestStatic();
        static void onReceiveStatic(int data);
        // void onReceive(int len);

        int debugPackets = 0;

        
        int allData[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; // All the data to send to the mainboard

        std::queue<std::vector<int>> receivedQueue;

        std::vector<int> topData;

        const byte thisAddress = 0x55; // these need to be swapped for the other Arduino
        const byte otherAddress = 0x7C;

    public:

        std::vector<int> getNewData(); // Return the top data received
        I2CBus();
        bool isNewData();
        void init(int _SDA, int _SCL);


    


    



};