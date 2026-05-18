#include "UARTComms.h"

UARTComms::UARTComms()
    : _port(nullptr),
      _baud(115200),
      _role(UART_ROLE_MASTER),
      _rxState(RX_WAIT_START),
      _rxAddr(0),
      _rxLen(0),
      _rxIdx(0),
      _rxChecksum(0),
      _rxData1(0),
      _rxData2(0),
      _rxData3(0),
      _hasNewData(false),
      _txData1(0),
      _txData2(0),
      _txData3(0) {
}

void UARTComms::init(HardwareSerial* port, uint32_t baud, UARTRole role,
                     int rxPin, int txPin) {
    _port = port;
    _baud = baud;
    _role = role;
    _rxState = RX_WAIT_START;
    if (_port != nullptr) {
#if defined(ESP32) || defined(ESP_PLATFORM)
        if (rxPin >= 0 && txPin >= 0) {
            _port->begin(_baud, SERIAL_8N1, rxPin, txPin);
        } else {
            _port->begin(_baud);
        }
#else
        (void)rxPin; (void)txPin; // pins are fixed by hardware on Teensy/AVR
        _port->begin(_baud);
#endif
    }
}

void UARTComms::writePacket(uint8_t type, const uint8_t* payload, uint8_t len) {
    if (_port == nullptr) return;
    if (len > UART_MAX_PAYLOAD - 1) return; // 1 byte reserved for type

    uint8_t addr = UART_PEER_ADDR;
    uint8_t totalLen = len + 1; // +1 for type byte
    uint8_t checksum = addr ^ totalLen ^ type;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }

    _port->write(UART_START_BYTE);
    _port->write(addr);
    _port->write(totalLen);
    _port->write(type);
    if (len > 0) {
        _port->write(payload, len);
    }
    _port->write(checksum);
    _port->write(UART_END_BYTE);
    _port->flush();
}

void UARTComms::sendData(int data1, int data2, int data3) {
    int32_t d1 = (int32_t)data1;
    int32_t d2 = (int32_t)data2;
    int32_t d3 = (int32_t)data3;

    // 3 x int32, big-endian (MSB first).
    uint8_t payload[12];
    payload[0]  = (uint8_t)((d1 >> 24) & 0xFF);
    payload[1]  = (uint8_t)((d1 >> 16) & 0xFF);
    payload[2]  = (uint8_t)((d1 >> 8)  & 0xFF);
    payload[3]  = (uint8_t)( d1        & 0xFF);
    payload[4]  = (uint8_t)((d2 >> 24) & 0xFF);
    payload[5]  = (uint8_t)((d2 >> 16) & 0xFF);
    payload[6]  = (uint8_t)((d2 >> 8)  & 0xFF);
    payload[7]  = (uint8_t)( d2        & 0xFF);
    payload[8]  = (uint8_t)((d3 >> 24) & 0xFF);
    payload[9]  = (uint8_t)((d3 >> 16) & 0xFF);
    payload[10] = (uint8_t)((d3 >> 8)  & 0xFF);
    payload[11] = (uint8_t)( d3        & 0xFF);

    writePacket(UART_PKT_DATA, payload, sizeof(payload));
}

void UARTComms::requestData() {
    // Master asks the peer to push its latest data.
    writePacket(UART_PKT_REQUEST, nullptr, 0);
}

int UARTComms::scan(uint32_t timeoutMs) {
    if (_port == nullptr) return 0;

    // Drain any stale RX bytes so a previous unsolicited packet doesn't give
    // a false positive.
    while (_port->available() > 0) { (void)_port->read(); }
    _rxState = RX_WAIT_START;
    _hasNewData = false;

    // Ping the peer.
    writePacket(UART_PKT_REQUEST, nullptr, 0);

    // Poll for a reply.
    uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        if (update()) {
            return 1;
        }
    }
    return 0;
}

void UARTComms::setResponseData(int data1, int data2, int data3) {
    _txData1 = (int32_t)data1;
    _txData2 = (int32_t)data2;
    _txData3 = (int32_t)data3;
}

void UARTComms::handlePacket() {
    // _rxBuf[0] = type, remaining bytes = payload
    if (_rxLen < 1) return;
    uint8_t type = _rxBuf[0];

    switch (type) {
        case UART_PKT_DATA: {
            if (_rxLen >= 13) { // type + 12 payload bytes (3 x int32, big-endian)
                _rxData1 = (int32_t)(((uint32_t)_rxBuf[1]  << 24) |
                                     ((uint32_t)_rxBuf[2]  << 16) |
                                     ((uint32_t)_rxBuf[3]  << 8)  |
                                     ((uint32_t)_rxBuf[4]));
                _rxData2 = (int32_t)(((uint32_t)_rxBuf[5]  << 24) |
                                     ((uint32_t)_rxBuf[6]  << 16) |
                                     ((uint32_t)_rxBuf[7]  << 8)  |
                                     ((uint32_t)_rxBuf[8]));
                _rxData3 = (int32_t)(((uint32_t)_rxBuf[9]  << 24) |
                                     ((uint32_t)_rxBuf[10] << 16) |
                                     ((uint32_t)_rxBuf[11] << 8)  |
                                     ((uint32_t)_rxBuf[12]));
                _hasNewData = true;
            }
            break;
        }
        case UART_PKT_REQUEST: {
            // Auto-respond with the currently queued response data.
            sendData(_txData1, _txData2, _txData3);
            break;
        }
        default:
            break;
    }
}

bool UARTComms::update() {
    if (_port == nullptr) return false;

    _hasNewData = false;

    while (_port->available() > 0) {
        uint8_t b = (uint8_t)_port->read();

        switch (_rxState) {
            case RX_WAIT_START:
                if (b == UART_START_BYTE) {
                    _rxState = RX_ADDR;
                }
                break;

            case RX_ADDR:
                _rxAddr = b;
                _rxState = RX_LEN;
                break;

            case RX_LEN:
                _rxLen = b;
                _rxIdx = 0;
                if (_rxLen == 0 || _rxLen > UART_MAX_PAYLOAD) {
                    // Either malformed or nothing to read; jump straight to checksum
                    if (_rxLen > UART_MAX_PAYLOAD) {
                        _rxState = RX_WAIT_START;
                    } else {
                        _rxState = RX_CHECKSUM;
                    }
                } else {
                    _rxState = RX_PAYLOAD;
                }
                break;

            case RX_PAYLOAD:
                _rxBuf[_rxIdx++] = b;
                if (_rxIdx >= _rxLen) {
                    _rxState = RX_CHECKSUM;
                }
                break;

            case RX_CHECKSUM:
                _rxChecksum = b;
                _rxState = RX_END;
                break;

            case RX_END:
                if (b == UART_END_BYTE && _rxAddr == UART_PEER_ADDR) {
                    // Verify checksum
                    uint8_t calc = _rxAddr ^ _rxLen;
                    for (uint8_t i = 0; i < _rxLen; i++) {
                        calc ^= _rxBuf[i];
                    }
                    if (calc == _rxChecksum) {
                        handlePacket();
                    }
                }
                _rxState = RX_WAIT_START;
                break;
        }
    }

    return _hasNewData;
}
