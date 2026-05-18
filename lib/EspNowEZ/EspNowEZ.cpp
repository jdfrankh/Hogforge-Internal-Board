#include "ESPNowEZ.h"
#ifdef ESP32
#include <cstring>
#include <list>
#include <esp_system.h>
#include <esp_wifi.h>

// --- Base-class static state for send-status tracking --------------------
volatile bool EspNowEZ::_lastSendDone = false;
volatile bool EspNowEZ::_lastSendOk   = false;
uint8_t       EspNowEZ::_lastSendPeer[6] = {0,0,0,0,0,0};

void EspNowEZ::_onSendStatusInternal(const uint8_t* mac,
                                     esp_now_send_status_t status) {
    if (mac) memcpy(_lastSendPeer, mac, 6);
    _lastSendOk   = (status == ESP_NOW_SEND_SUCCESS);
    _lastSendDone = true;
}

bool EspNowEZ::connectFixed(uint32_t timeoutMs) {
    if (!_fixedPeerSet) return false;

    // Add the peer if not already present. add_peer is idempotent-ish: a
    // failure here usually means "already exists", which is fine.
    add_peer(_fixedPeer, _fixedPeerChannel);

    // Tiny probe payload — just a marker byte. The peer doesn't have to do
    // anything with it; the link-layer MAC ACK is what we're waiting on.
    uint8_t probe = 0xC0;
    Container c;
    c.bytes  = &probe;
    c.length = 1;

    _lastSendDone = false;
    _lastSendOk   = false;
    if (send_message(_fixedPeer, c) != 0) {
        return false;
    }

    uint32_t t0 = millis();
    while (!_lastSendDone && (millis() - t0) < timeoutMs) {
        delay(1);
    }
    return _lastSendDone && _lastSendOk;
}

bool EspNowEZ::sendToFixedConfirmed(Container d, uint32_t timeoutMs) {
    if (!_fixedPeerSet) return false;

    _lastSendDone = false;
    _lastSendOk   = false;
    if (send_message(_fixedPeer, d) != 0) {
        return false;
    }

    uint32_t t0 = millis();
    while (!_lastSendDone && (millis() - t0) < timeoutMs) {
        delay(1);
    }
    return _lastSendDone && _lastSendOk;
}

class ESPNowW32 : public EspNowEZ {
  public:
    virtual esp_err_t add_peer(uint8_t *mac, int channel);
    virtual esp_err_t remove_peer(uint8_t *mac);
    virtual esp_err_t send_message(uint8_t *mac, Container d);
    virtual esp_err_t set_mac(uint8_t *mac);

  private:
    std::list<esp_now_peer_info_t> peers;
};

esp_err_t ESPNowW32::add_peer(uint8_t *mac, int channel) {
    peers.push_back(esp_now_peer_info_t{});
    esp_now_peer_info_t &peer = peers.back();
    std::memcpy(peer.peer_addr, mac, sizeof(peer.peer_addr));
    peer.channel = channel;
    peer.ifidx = WIFI_IF_STA;
    auto success = esp_now_add_peer(&peers.back());
    if (success != ESP_OK)
        remove_peer(mac);
    return success;
}
esp_err_t ESPNowW32::remove_peer(uint8_t *mac) {
    // find peer in peers
    for (auto it = peers.begin(); it != peers.end();) {
        if (0 == memcmp(mac, it->peer_addr, sizeof(it->peer_addr))) {
            it = peers.erase(it); // remove it
        }
    }
    return esp_now_del_peer(mac);
}
/*esp_err_t ESPNowW32::send_message(uint8_t *mac, Container *d) {
    return esp_now_send(mac, (uint8_t*) &d, d->getRawSize());
}*/

esp_err_t ESPNowW32::send_message(uint8_t *mac, Container d) {
    return esp_now_send(mac, d.bytes, d.length);
}

esp_err_t ESPNowW32::set_mac(uint8_t *mac) {
    // Set the MAC address for the station interface using esp_wifi API
    // esp_wifi_set_mac returns esp_err_t which matches the function signature
    return esp_wifi_set_mac(WIFI_IF_STA, mac);
}
ESPNowW32 espnow;
EspNowEZ &ESPNow = espnow;
#endif