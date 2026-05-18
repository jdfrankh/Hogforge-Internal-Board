#ifndef UART_COMMS_H
#define UART_COMMS_H

#include <Arduino.h>

//===================
// UARTComms - point-to-point serial communication between two boards.
//   Mirrors the API of I2CBus (init / sendData / requestData / update)
//   but uses a hardware UART instead of I2C.
//
//   Framing for each packet:
//     [START=0xAA][ADDR][LEN][payload...][CHECKSUM][END=0x55]
//   where CHECKSUM = XOR of ADDR, LEN, and all payload bytes.
//
//   Hard-coded peer address is UART_PEER_ADDR. Packets whose address
//   does not match are discarded.
//
//   Master/Slave:
//     - Master typically calls sendData() / requestData().
//     - Slave typically calls update() in its main loop and answers
//       REQUEST packets automatically with the last queued response.
//===================

#define UART_PEER_ADDR     0x55
#define UART_START_BYTE    0xAA
#define UART_END_BYTE      0x55
#define UART_MAX_PAYLOAD   32

// Packet "kinds" encoded in the LEN high nibble? Keep it simple: use ADDR field
// only for routing and a dedicated type byte at the start of the payload.
enum UARTPacketType : uint8_t {
    UART_PKT_DATA    = 0x01, // payload = 12 bytes (3 x int32, big-endian)
    UART_PKT_REQUEST = 0x02, // payload = 0 bytes, asks peer to send DATA
};

enum UARTRole : uint8_t {
    UART_ROLE_SLAVE  = 0,
    UART_ROLE_MASTER = 1,
};

class UARTComms {

    private:
        HardwareSerial* _port;
        uint32_t _baud;
        UARTRole _role;

        // RX state machine
        enum RxState : uint8_t {
            RX_WAIT_START = 0,
            RX_ADDR,
            RX_LEN,
            RX_PAYLOAD,
            RX_CHECKSUM,
            RX_END,
        };
        RxState  _rxState;
        uint8_t  _rxAddr;
        uint8_t  _rxLen;
        uint8_t  _rxIdx;
        uint8_t  _rxChecksum;
        uint8_t  _rxBuf[UART_MAX_PAYLOAD];

        // Last received DATA payload (decoded)
        int32_t  _rxData1;
        int32_t  _rxData2;
        int32_t  _rxData3;
        bool     _hasNewData;

        // Queued response for slave to send when a REQUEST is received
        int32_t  _txData1;
        int32_t  _txData2;
        int32_t  _txData3;

        void writePacket(uint8_t type, const uint8_t* payload, uint8_t len);
        void handlePacket();

    public:
        UARTComms();

        // port: e.g. &Serial1, &Serial2, ...
        // baud: e.g. 115200
        // role: UART_ROLE_MASTER or UART_ROLE_SLAVE
        // rxPin/txPin: optional pin remap (ESP32 only). Pass -1 to use the port's
        //   default pins (required on Teensy / AVR, where pins are fixed by hardware).
        void init(HardwareSerial* port, uint32_t baud = 115200,
                  UARTRole role = UART_ROLE_MASTER,
                  int rxPin = -1, int txPin = -1);

        void setRole(UARTRole role) { _role = role; }
        UARTRole role() const { return _role; }

        // Send three ints to the peer (mirrors I2CBus::sendData)
        void sendData(int data1, int data2 = 0, int data3 = 0);

        // Master: ask the peer to send its latest data.
        // Slave: pre-load the values that will be returned on REQUEST.
        void requestData();
        void setResponseData(int data1, int data2 = 0, int data3 = 0);

        // Pump the RX state machine. Returns true when a new DATA packet
        // has been fully received since the last call.
        bool update();

        // Connectivity check: send a REQUEST and wait up to timeoutMs for the
        // peer to reply with a DATA packet. Returns 1 if the peer responded,
        // 0 otherwise. Mirrors the int return of I2CBus::scan() so existing
        // call sites (e.g. menu->printFoundI2C(connected)) keep working.
        int scan(uint32_t timeoutMs = 250);

        // Accessors for the most recently received data packet
        int32_t lastData1() const { return _rxData1; }
        int32_t lastData2() const { return _rxData2; }
        int32_t lastData3() const { return _rxData3; }
};

#endif // UART_COMMS_H
