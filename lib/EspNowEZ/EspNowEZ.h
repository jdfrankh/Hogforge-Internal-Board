
#ifndef ESPNOW_HPP
#define ESPNOW_HPP
#include <Arduino.h>
#include <stdint.h>
#include <string>
#include <iostream>
#include <memory>
#ifdef ESP32
#include <esp_now.h>
#elif ESP8266
#include <c_types.h>
#include <espnow.h>
#endif



struct Container {
    uint8_t* bytes;
    size_t length;

    template<typename T>
    void store(T& s){
        bytes = reinterpret_cast<uint8_t*>(&s);
        length = sizeof(s);
    }

    void printBytes() {
        Serial.print("Bytes: ");
        for (size_t i = 0; i < length; i++) {
            Serial.printf("%02X ", bytes[i]);
        }
        Serial.printf("Length: %d", length);
        Serial.println();
    }

};

class EspNowEZ {
private:

    uint8_t* raw;

    uint8_t _broadcastAll[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t currentMac[6] = {0, 0, 0, 0, 0, 0};

    // --- Fixed-peer support ------------------------------------------------
    uint8_t _fixedPeer[6]      = {0, 0, 0, 0, 0, 0};
    bool    _fixedPeerSet      = false;
    int     _fixedPeerChannel  = 0;

protected:
    // Updated by the internal send-status callback registered in
    // beginTracking(). Volatile because the callback runs on the WiFi task.
    static volatile bool _lastSendDone;
    static volatile bool _lastSendOk;
    static uint8_t       _lastSendPeer[6];

    static void _onSendStatusInternal(const uint8_t* mac,
                                      esp_now_send_status_t status);

public:



    

    EspNowEZ() {}
    ~EspNowEZ() {}

    virtual int add_peer(uint8_t *mac, int channel = 0) = 0;
    virtual int remove_peer(uint8_t *mac) = 0;
    virtual int send_message(uint8_t *mac, Container d) = 0;
    
    virtual int set_mac(uint8_t *mac) = 0;
    virtual int init() { return esp_now_init(); }
    int reg_send_cb(esp_now_send_cb_t cb) {
        return esp_now_register_send_cb(cb);
    }
    int reg_recv_cb(esp_now_recv_cb_t cb) {
        return esp_now_register_recv_cb(cb);
    }
    int unreg_send_cb() { return esp_now_unregister_send_cb(); }
    int unreg_recv_cb() { return esp_now_unregister_recv_cb(); }
    int broadcastMessage(Container d) {
        return send_message(_broadcastAll, d);
    }
    void readMacAddress(); 

    // ------------------------------------------------------------------
    // Fixed-peer convenience layer
    // ------------------------------------------------------------------
    // Configure (but do NOT yet add) a single hard-coded peer that the
    // application talks to. Call after init().
    void setFixedPeer(const uint8_t mac[6], int channel = 0) {
        memcpy(_fixedPeer, mac, 6);
        _fixedPeerChannel = channel;
        _fixedPeerSet = true;
    }
    bool hasFixedPeer() const { return _fixedPeerSet; }
    const uint8_t* fixedPeer() const { return _fixedPeer; }

    // Registers the library's internal send-status callback so
    // connectFixed() / sendToFixedConfirmed() can observe link-layer ACKs.
    // Must be called after esp_now_init() (i.e. after init()).
    int beginTracking() {
        return reg_send_cb(_onSendStatusInternal);
    }

    // Add the configured fixed peer (if not added already) and send a small
    // probe packet to it. Blocks up to timeoutMs waiting for the WiFi MAC
    // layer to confirm delivery via the send callback.
    // Returns true on confirmed delivery, false on timeout / no peer.
    bool connectFixed(uint32_t timeoutMs = 1000);

    // Send `d` to the fixed peer and block up to timeoutMs for the send
    // callback to report ESP_NOW_SEND_SUCCESS. Returns true on confirmed
    // delivery.
    bool sendToFixedConfirmed(Container d, uint32_t timeoutMs = 200);
};

extern EspNowEZ &ESPNow;
#endif