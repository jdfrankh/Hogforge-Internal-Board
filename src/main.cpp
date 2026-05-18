#include <Arduino.h>
//#include <StepperMotor.h>
#include <BuildPlates.h>
#include <LoaderBar.h>
#include <Axis.h>
#include "main.h"
#include <Wire.h>
#include <vector>
#include <queue>
#include <cstring>
#include <Fan.h> 
#include <O2Sensor.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>

// ---- ESP-NOW link to Hogforge_Xiao_Transmitter bridge --------------------
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "EspNowEZ.h"


// ---- TMC2209 UART for the BuildPlate (broken STEP/DIR -> driven via VACTUAL) ----
// UART2 is unused on the XIAO ESP32-S3 and can be remapped to any GPIO.
HardwareSerial TMCSerial(2);
TMC2209Stepper drvBuild(&TMCSerial, TMC_RSENSE, BUILD_TMC_ADDR);
TMC2209Stepper drvLoad (&TMCSerial, TMC_RSENSE, LOAD_TMC_ADDR);

static void configureTMC(TMC2209Stepper& d) {
    d.begin();
    d.toff(5);
    d.blank_time(24);
    d.rms_current(300, 0.5f);
    d.microsteps(16);
    d.intpol(true);
    d.pdn_disable(true);       // REQUIRED so PDN_UART acts as UART, not standby
    d.mstep_reg_select(true);  // microsteps come from UART, not MS1/MS2 pins
    d.I_scale_analog(false);   // use internal Vref, ignore VREF pin
    d.en_spreadCycle(false);   // StealthChop
}



// ---- ESP-NOW bridge configuration ---------------------------------------
// MAC of the Hogforge_Xiao_Transmitter (ESP32-C3) that sits between us and
// the Teensy 4.1. Update to match the Xiao's printed STA MAC.
static uint8_t XIAO_BRIDGE_MAC[6] = { 0x98, 0x3D, 0xAE, 0xAA, 0xF3, 0xE4 };
//This mac address -> b8:f8:62:cb:e7:d4
static constexpr int XIAO_BRIDGE_CHANNEL = 1;  // fixed channel — must match Xiao bridge

// Packet layout shared with the Xiao bridge (see main.cpp on the Xiao side).
struct __attribute__((packed)) BridgePacket {
    uint8_t tag;   // 0xD1 = data forwarded from Teensy
    int32_t d1;
    int32_t d2;
    int32_t d3;
};
static constexpr uint8_t BRIDGE_TAG_DATA = 0xD1;

bool execute[3] = {false};
bool setPlate = true;

std::queue<std::vector<int>> CommandQueue;

// Staging buffer pushed from the ESP-NOW receive callback (runs on the WiFi
// task) and drained from loop(). Guarded by a portMUX so the std::queue is
// never concurrently mutated.
static portMUX_TYPE espnowMux = portMUX_INITIALIZER_UNLOCKED;
static std::queue<std::vector<int>> espnowInbox;

// Number of ESP-NOW frames accepted by onEspNowRecv since boot. Used by the
// link-health logger to tell "silent peer" apart from "link is fine".
static volatile uint32_t espnowRecvCount = 0;

// Arduino-ESP32 2.x signature (espressif32 6.x). If you ever bump to
// Arduino-ESP32 3.x change `const uint8_t* mac` to
// `const esp_now_recv_info_t* info`.
static void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < (int)sizeof(BridgePacket)) return;
    BridgePacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.tag != BRIDGE_TAG_DATA) return;

    // Match the shape processCommand() expects: {cmd, arg1, arg2}.
    std::vector<int> cmd = { (int)pkt.d1, (int)pkt.d2, (int)pkt.d3 };
    portENTER_CRITICAL(&espnowMux);
    espnowInbox.push(std::move(cmd));
    espnowRecvCount++;
    portEXIT_CRITICAL(&espnowMux);
}

static void printMac(const char* label, const uint8_t mac[6]) {
    Serial.print(label);
    for (int i = 0; i < 6; ++i) {
        if (mac[i] < 0x10) Serial.print('0');
        Serial.print(mac[i], HEX);
        if (i < 5) Serial.print(':');
    }
    Serial.println();
}

static bool initEspNowReceiver() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Arduino-ESP32 turns on STA modem-sleep by default. A sleeping,
    // un-associated STA misses ESP-NOW frames and never ACKs them, so the
    // Xiao's connectFixed() probe times out and it never enters loop().
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Lock to the same channel as the Xiao bridge before esp_now_init().
    // Without this, un-associated STAs can default to different channels.
    esp_wifi_set_channel(XIAO_BRIDGE_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Read back what the driver actually committed to.
    uint8_t ch = 0; wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    Serial.print("Internal radio channel: "); Serial.println(ch);

    uint8_t self[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, self);
    printMac("Internal STA MAC : ", self);

    if (ESPNow.init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        return false;
    }

    // Needed so connectFixed() below can observe MAC-layer ACKs from the Xiao.
    ESPNow.beginTracking();

    // Add the Xiao as a peer so unicast frames from it are MAC-ACKed and so
    // we can send replies back if we ever need to.
    ESPNow.setFixedPeer(XIAO_BRIDGE_MAC, XIAO_BRIDGE_CHANNEL);
    ESPNow.add_peer(XIAO_BRIDGE_MAC, XIAO_BRIDGE_CHANNEL);
    printMac("Xiao bridge peer : ", XIAO_BRIDGE_MAC);

    if (ESPNow.reg_recv_cb(onEspNowRecv) != ESP_OK) {
        Serial.println("ESP-NOW recv callback registration FAILED");
        return false;
    }
    Serial.println("ESP-NOW receiver ready.");
    return true;
}

// Actively probe the Xiao to confirm we can reach it. Returns true on
// link-layer ACK. Useful at boot and as a periodic health check.
static bool checkEspNowConnection() {
    bool ok = ESPNow.connectFixed(/*timeoutMs=*/500);
    Serial.print("ESP-NOW link to Xiao: ");
    Serial.println(ok ? "OK (ACK received)"
                      : "NO ACK (peer silent or wrong channel/MAC)");
    return ok;
}

std::vector<Fan*> circulationFan = {new Fan("CirculationFanLeft", FANLEFTPWM1, FANLEFTPWM2), new Fan("CirculationFanRight", FANRIGHTPWM1, FANRIGHTPWM2)};

O2Sensor sensor;
//============

Axis* arm = new Axis(-200000);

Axis* loadingMechanism = new Axis(-300000);

// ---- Limit-switch callback cooldown ---------------------------------------
// After a limit switch fires, ignore re-triggers for this many ms so the
// callback can't keep stomping on a freshly-issued command (e.g. after a
// home cycle the switch may bounce / stay asserted while the axis backs
// off, or a new MOVEBAR can arrive while the home callback is still
// settling).
static constexpr uint32_t LIMIT_CB_COOLDOWN_MS = 750;
static uint32_t lastResetLimitMs       = 0;
static uint32_t lastLoadLimitMs        = 0;
static uint32_t lastBuildPlateLimitMs  = 1000;
static uint32_t lastLoadPlateLimitMs   = 1000;

static inline bool limitCbCooldownActive(uint32_t &lastMs) {
  uint32_t now = millis();
  if ((now - lastMs) < LIMIT_CB_COOLDOWN_MS) {
    return true; // still cooling down; suppress this callback
  }
  lastMs = now;
  return false;
}

void onResetLimitTriggered(){
  if (limitCbCooldownActive(lastResetLimitMs)) return;
  #if DEBUGMODESTART
    Serial.println("ResetLimit callback from main.cpp fired");
  #endif
  arm->stop();
  arm->moveAxis(armHome - 20000);
  arm->setHome(true);
}

void onLoadLimitTriggered(){

  //if(arm->getHome()){
  //  return; // If we're already at home, ignore further triggers (e.g. from bouncing)
  //}

  if (limitCbCooldownActive(lastLoadLimitMs)) return;
  #if DEBUGMODESTART
    Serial.println("LoadLimit callback from main.cpp fired");
  #endif
  arm->stop();
  arm->resetPosition();
  arm->setHome(false);
}


void onBuildPlateLimitTriggered(){

  //if(loadingMechanism->getHome("BuildPlateStepper")){
  //  return; }// If we're already at home, ignore further triggers (e.g. from bouncing)

  if (limitCbCooldownActive(lastBuildPlateLimitMs)) return;
  #if DEBUGMODESTART
  //  Serial.println("BuildPlateLimit callback from main.cpp fired");
  #endif
  loadingMechanism->stop("BuildPlateStepper");
  loadingMechanism->resetPosition("BuildPlateStepper");
  loadingMechanism->setHome(false, "BuildPlateStepper");
}


void onLoadPlateLimitTriggered(){

  //if(loadingMechanism->getHome("LoadPlateStepper")){
  //  return; // If we're already at home, ignore further triggers (e.g. from bouncing)
  //}


  if (limitCbCooldownActive(lastLoadPlateLimitMs)) return;
  #if DEBUGMODESTART
  //  Serial.println("LoadPlateLimit callback from main.cpp fired");
  #endif
  loadingMechanism->stop("LoadPlateStepper");
  loadingMechanism->resetPosition("LoadPlateStepper");
  loadingMechanism->setHome(false, "LoadPlateStepper");
}


void processCommand(std::vector<int> commandData){
  Serial.print("Command ID:"); 
  Serial.println(commandData[0]);

  switch(commandData[0]){
    case INITALL:
      loadingMechanism->init();
      arm->init();
    break;
    case HOMEALL:
      loadingMechanism->home();
      arm->home();
    break; 
    case HOMEBAR:
      arm->home();
    break;
    case HOMEPLATES:
      loadingMechanism->home();
    break;
    case LIFTONEPLATE:
      if(commandData[1] == PlateID::BUILDPLATE){
        loadingMechanism->moveSingleStepper("BuildPlateStepper", commandData[2]);
      }
      else if(commandData[1] == PlateID::LOADPLATE){
        loadingMechanism->moveSingleStepper("LoadPlateStepper", commandData[2]);
      }
    break;
    case LOWERONEPLATE:
      if(commandData[1] == PlateID::BUILDPLATE){
        loadingMechanism->moveSingleStepper("BuildPlateStepper", -commandData[2]);
      }
      else if(commandData[1] == PlateID::LOADPLATE){
        loadingMechanism->moveSingleStepper("LoadPlateStepper", -commandData[2]);

      }
    break;
    case STARTFAN:
      for(Fan* fan : circulationFan){
        fan->setSpeed(Fan::Speeds::SPEED_MAX, Fan::Direction::FORWARD);
      }
    break;

    case STOPFAN:
      for(Fan* fan : circulationFan){
        fan->stop();
      }
    break;

    case SMALLSTEP:
        loadingMechanism->moveSingleStepper("BuildPlateStepper", -commandData[2]);
        loadingMechanism->moveSingleStepper("LoadPlateStepper", commandData[2]);

    break;
    case RAISEBOTHPLATES:
        loadingMechanism->moveSingleStepper("BuildPlateStepper", commandData[2]);
        loadingMechanism->moveSingleStepper("LoadPlateStepper", commandData[2]);

    break;
    case PREPPRINT:
    //Serial.println("Prepping for Print");
        //loadingMechanism->home();
        //arm->home();

        // Give the home() calls a moment to actually kick the steppers into
        // motion so isRunning() can report true on the next poll. Without this
        // the wait loop may see "not running yet" and fall through immediately.
        // delay(20);

        // // Block here until BOTH axes have come to rest at their home limits.
        // // We must keep pumping the motor controllers (update()) AND the limit
        // // switches (which live inside update()) so the homing callbacks can
        // // fire and stop each axis. Do NOT recursively call loop() — that
        // // re-enters the ESP-NOW drain and link-health logger.
        // while (loadingMechanism->isRunning() || arm->isRunning()) {
        //   loadingMechanism->update();
        //   arm->update();
        //   delay(1);
        // }

        // Both axes are now homed and zeroed by their limit-switch callbacks.
        // It is safe to issue the lift moves; no callback will stomp them.
        loadingMechanism->moveSingleStepper("BuildPlateStepper", totalPlateDistance);
        loadingMechanism->moveSingleStepper("LoadPlateStepper", commandData[2]);

    break;
    case MOVEBAR:
      arm->moveAxis(oneSwipedistance);
    break; 
    default:

    break;
  };

}



void setup() {
  Serial.begin(115200);

  delay(4000);
  // Commands now arrive over ESP-NOW from the Xiao transmitter bridge
  // (which is itself talking UART to the Teensy 4.1). No direct UART link
  // to the External Board from this board anymore.
  if (!initEspNowReceiver()) {
    Serial.println("WARNING: ESP-NOW unavailable, commands will not arrive.");
  } else {
    // Active boot-time check: ping the Xiao and report whether it ACKs.
    checkEspNowConnection();
  }

  #if DEBUGMODESTART
  

  Serial.println("Homing All...");
  
  #endif

  for (Fan* fan : circulationFan){
    fan->setup();
    fan->setBreakType(Fan::BreakType::BRAKE);
   // fan->setSpeed(Speeds::MEDIUM, Direction::FORWARD);
  }

  //sensor.setup();

  
  Serial.println("Initializing Loader Bar...");

  arm->addLimitSwitch("ResetLimit", resetLimit);
  arm->addLimitSwitch("LoadLimit", loadLimit);

  arm->setLimitSwitchCallback("ResetLimit", onResetLimitTriggered);
  arm->setLimitSwitchCallback("LoadLimit", onLoadLimitTriggered);

  arm->addStepper("LeftStepper", std::vector<int>{leftSTEP, leftDIR, leftEN}, Axis::StepperMotor::Speed::FAST, Axis::StepperMotor::Direction::DIR_NORMAL);
  arm->addStepper("RightStepper", std::vector<int>{rightSTEP, rightDIR, rightEN}, Axis::StepperMotor::Speed::FAST, Axis::StepperMotor::Direction::DIR_REVERSE);

  arm->setSpeed(Axis::Speed::SLOW);
  #if DEBUGMODESTART
    Serial.println("Initializing Axis...");
  #endif

  arm->init();

  loadingMechanism->addLimitSwitch("BuildPlateLimit", buildPlateLimit, "BuildPlateStepper");
  loadingMechanism->addLimitSwitch("LoadPlateLimit", loadPlateLimit, "LoadPlateStepper");

  loadingMechanism->setLimitSwitchCallback("BuildPlateLimit", onBuildPlateLimitTriggered);
  loadingMechanism->setLimitSwitchCallback("LoadPlateLimit", onLoadPlateLimitTriggered);

  
  loadingMechanism->addStepper("BuildPlateStepper", std::vector<int>{BuildSTEP, BuildDIR, BuildEN}, Axis::StepperMotor::Speed::MEDIUM, Axis::StepperMotor::Direction::DIR_NORMAL); // This hardware may be a problem
  loadingMechanism->addStepper("LoadPlateStepper", std::vector<int>{LoadSTEP, LoadDIR, LoadEN}, Axis::StepperMotor::Speed::MEDIUM, Axis::StepperMotor::Direction::DIR_REVERSE); // DIR_REVERSE for Load , DIR_NORMAL for Build?

  // ---- Bring up the TMC2209 UART bus and switch BuildPlate to UART control. ----
  // Done BEFORE loadingMechanism->init() so the FastAccelStepper backend skips
  // the (broken) BuildPlate STEP pin entirely.
  TMCSerial.begin(TMC_BAUD, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
  configureTMC(drvLoad);
  configureTMC(drvBuild);
  loadingMechanism->setStepperUart("BuildPlateStepper", &drvBuild, BUILD_VACTUAL);

  loadingMechanism->init();

  #if DEBUGMODESTART
    delay(2000); 
    Serial.println("Homing Loader Bar...");
  #endif


  //arm->home();
  //loadingMechanism->home();

 // for(Fan* fan : circulationFan){
 //   fan->setSpeed(Fan::Speeds::SPEED_MAX, Fan::Direction::FORWARD);
 // }
 //CommandQueue.push(std::vector<int>{HOMEALL});

 //CommandQueue.push(std::vector<int>{STARTFAN});
 //CommandQueue.push(std::vector<int>{MOVEBAR}) ;
 // RAISEBOTHPLATES handler reads commandData[2] as the distance, so this
 // vector must have at least 3 elements. Slot [1] is unused for this command.
 //CommandQueue.push(std::vector<int>{RAISEBOTHPLATES, 0, (int)totalPlateDistance});
 //CommandQueue.push(std::vector<int>{HOMEALL});

  //CommandQueue.push(std::vector<int>{LISTONEPLATE, PlateID::BUILDPLATE, 100000});



 
}

void loop() {

  // ---- ESP-NOW link health check ----------------------------------------
  // Every 5 s, actively probe the Xiao bridge and log how many frames we've
  // received since boot. Lets us tell "link down" from "link up but peer
  // hasn't sent anything yet".
  {
    static uint32_t lastLinkCheck = 0;
    if (millis() - lastLinkCheck > 5000) {
      lastLinkCheck = millis();
      bool linkUp = checkEspNowConnection();
      portENTER_CRITICAL(&espnowMux);
      uint32_t rx = espnowRecvCount;
      portEXIT_CRITICAL(&espnowMux);
      Serial.print("  frames received from Xiao: "); Serial.println(rx);
      (void)linkUp;
    }
  }

  


  #if DEBUGMODELOOP 
    //Serial.println("-------------------------------------");
    //arm->printLimitSwitchStates();
    //loadingMechanism->printLimitSwitchStates();

    static uint32_t t = 0;
    if (millis() - t > 500) {
        t = millis();
        loadingMechanism->printLimitSwitchStates();
        arm->printLimitSwitchStates();
        loadingMechanism->printStepperStates();
        arm->printStepperStates();

    }

  #else
  
  
  #endif
  delay(1);


  // update() returns true while a stepper is still moving. Dispatch the next
  // queued command only when BOTH axes are idle. Note: update() must run every
  // loop unconditionally so limit switches are polled.
  bool armBusy  = arm->update();
  bool loadBusy = loadingMechanism->update();
  if(!armBusy && !loadBusy){
    if(!CommandQueue.empty()){
      processCommand(CommandQueue.front());
      CommandQueue.pop();
    }
  }


 // Drain anything the ESP-NOW recv callback queued up. Done under the same
 // portMUX the callback uses so we never see a half-written std::queue.
 {
   portENTER_CRITICAL(&espnowMux);
   while (!espnowInbox.empty()) {
     CommandQueue.push(std::move(espnowInbox.front()));
     espnowInbox.pop();
   }
   portEXIT_CRITICAL(&espnowMux);
 }

  // Busy -> idle transition: notify the Xiao bridge so the External Board
  // can unblock any GCode line that is waiting on the Internal Board.
  {
    static bool wasBusy = false;
    bool nowBusy = !CommandQueue.empty() || arm->isRunning() || loadingMechanism->isRunning();
    if (wasBusy && !nowBusy) {
      BridgePacket status;
      status.tag = 0xD2;
      status.d1  = 2;   // 2 = idle/done sentinel (0=fail ACK, 1=ok ACK are taken)
      status.d2  = 0;
      status.d3  = 0;
      esp_now_send(XIAO_BRIDGE_MAC, reinterpret_cast<const uint8_t*>(&status), sizeof(status));
    }
    wasBusy = nowBusy;
  }

  //delay(2000000);
  //Serial.printf("Queue %d ,Arm: %d,Bar: %d", i2c.isNewData(), arm->update(), loadingSet->update());
  //Serial.println();
  //arm->update();
  //loadingSet->update();


  
}




//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================

/*
        // data to be sent and received
struct I2cTxStruct {
    char textA[16];         // 16 bytes
    int valA;               //  2
    unsigned long valB;     //  4
    byte padding[10];       // 10
                            //------
                            // 32
};

struct I2cRxStruct {
    char textB[16];         // 16 bytes
    int valC;               //  2
    unsigned long valD;     //  4
    byte padding[10];       // 10
                            //------
                            // 32
};

I2cTxStruct txData = {"xxx", 236, 0};
I2cRxStruct rxData;

bool newTxData = false;
bool newRxData = false;
bool rqSent = false;


        // I2C control stuff
#include <Wire.h>

const byte thisAddress = 9; // these need to be swapped for the other Arduino
const byte otherAddress = 8;

void updateDataToSend() {

        // update the data after the previous message has been
        //    sent in response to the request
        // this ensures the new data will ready when the next request arrives
    if (rqSent == true) {
        rqSent = false;

        char sText[] = "SendB";
        strcpy(txData.textA, sText);
        txData.valA += 10;
        if (txData.valA > 300) {
            txData.valA = 236;
        }
        txData.valB = millis();

    }
}

//=========

void showTxData() {

            // for demo show the data that as been sent
        Serial.print("Sent ");
        Serial.print(txData.textA);
        Serial.print(' ');
        Serial.print(txData.valA);
        Serial.print(' ');
        Serial.println(txData.valB);

}

//=============

void showNewData() {

    Serial.print("This just in    ");
    Serial.print(rxData.textB);
    Serial.print(' ');
    Serial.print(rxData.valC);
    Serial.print(' ');
    Serial.println(rxData.valD);
}

//============

        // this function is called by the Wire library when a message is received
void receiveEvent(int numBytesReceived) {

    if (newRxData == false) {
            // copy the data to rxData
        Wire.readBytes( (byte*) &rxData, numBytesReceived);
        newRxData = true;
    }
    else {
            // dump the data
        while(Wire.available() > 0) {
            byte c = Wire.read();
        }
    }
}

//===========

void requestEvent() {
    Wire.write((byte*) &txData, sizeof(txData));
    rqSent = true;
}

//=================================

void setup() {
  delay(6000);
    Serial.begin(115200);
    Serial.println("\nStarting I2C SlaveRespond demo\n");

        // set up I2C
        Wire.setPins(1, 0);
    Wire.begin(thisAddress); // join i2c bus
    Wire.onReceive(receiveEvent); // register function to be called when a message arrives
    Wire.onRequest(requestEvent); // register function to be called when a request arrives

}

//============

void loop() {

        // this bit checks if a message has been received
    if (newRxData == true) {
        showNewData();
        newRxData = false;
    }


        // this function updates the data in txData
    updateDataToSend();
        // this function sends the data if one is ready to be sent

}
*/
//============



//*----------------------------------------------------------------

// Wire Scanner - scans for I2C devices on all Wire ports
//
// This Wire library adapted is for Teensy boards
//
//   https://github.com/PaulStoffregen/Wire/
//
// Adapted from I2C Scanner originally published on Arduino Playground
// see comments below for link and credits for original authors.
/*
#include <Wire.h>



void printKnownChips(byte address)
{
  // Is this list missing part numbers for chips you use?
  // Please suggest additions here:
  // https://github.com/PaulStoffregen/Wire/issues/new
  switch (address) {
    case 0x00: Serial.print(F("AS3935")); break;
    case 0x01: Serial.print(F("AS3935")); break;
    case 0x02: Serial.print(F("AS3935")); break;
    case 0x03: Serial.print(F("AS3935")); break;
    case 0x04: Serial.print(F("ADAU1966")); break;
    case 0x0A: Serial.print(F("SGTL5000")); break; // MCLK required
    case 0x0B: Serial.print(F("SMBusBattery?")); break;
    case 0x0C: Serial.print(F("AK8963")); break;
    case 0x10: Serial.print(F("CS4272")); break;
    case 0x11: Serial.print(F("Si4713")); break;
    case 0x13: Serial.print(F("VCNL4000,AK4558")); break;
    case 0x18: Serial.print(F("LIS331DLH")); break;
    case 0x19: Serial.print(F("LSM303,LIS331DLH")); break;
    case 0x1A: Serial.print(F("WM8731")); break;
    case 0x1C: Serial.print(F("LIS3MDL")); break;
    case 0x1D: Serial.print(F("LSM303D,LSM9DS0,ADXL345,MMA7455L,LSM9DS1,LIS3DSH")); break;
    case 0x1E: Serial.print(F("LSM303D,HMC5883L,FXOS8700,LIS3DSH")); break;
    case 0x20: Serial.print(F("MCP23017,MCP23008,PCF8574,FXAS21002,SoilMoisture")); break;
    case 0x21: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x22: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x23: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x24: Serial.print(F("MCP23017,MCP23008,PCF8574,ADAU1966,HM01B0")); break;
    case 0x25: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x26: Serial.print(F("MCP23017,MCP23008,PCF8574")); break;
    case 0x27: Serial.print(F("MCP23017,MCP23008,PCF8574,LCD16x2,DigoleDisplay")); break;
    case 0x28: Serial.print(F("BNO055,EM7180,CAP1188")); break;
    case 0x29: Serial.print(F("TSL2561,VL6180,TSL2561,TSL2591,BNO055,CAP1188")); break;
    case 0x2A: Serial.print(F("SGTL5000,CAP1188")); break;
    case 0x2B: Serial.print(F("CAP1188")); break;
    case 0x2C: Serial.print(F("MCP44XX ePot")); break;
    case 0x2D: Serial.print(F("MCP44XX ePot")); break;
    case 0x2E: Serial.print(F("MCP44XX ePot")); break;
    case 0x2F: Serial.print(F("MCP44XX ePot")); break;
    case 0x30: Serial.print(F("Si7210")); break;
    case 0x31: Serial.print(F("Si7210")); break;
    case 0x32: Serial.print(F("Si7210")); break;
    case 0x33: Serial.print(F("MAX11614,MAX11615,Si7210,MLX90640,MLX90641")); break;
    case 0x34: Serial.print(F("MAX11612,MAX11613")); break;
    case 0x35: Serial.print(F("MAX11616,MAX11617")); break;
    case 0x38: Serial.print(F("RA8875,FT6206,MAX98390")); break;
    case 0x39: Serial.print(F("TSL2561, APDS9960")); break;
    case 0x3A: Serial.print(F("MLX90632")); break;
    case 0x3C: Serial.print(F("SSD1306,DigisparkOLED")); break;
    case 0x3D: Serial.print(F("SSD1306")); break;
    case 0x40: Serial.print(F("PCA9685,Si7021,MS8607")); break;
    case 0x41: Serial.print(F("STMPE610,PCA9685")); break;
    case 0x42: Serial.print(F("PCA9685")); break;
    case 0x43: Serial.print(F("PCA9685")); break;
    case 0x44: Serial.print(F("PCA9685, SHT3X, ADAU1966")); break;
    case 0x45: Serial.print(F("PCA9685, SHT3X")); break;
    case 0x46: Serial.print(F("PCA9685")); break;
    case 0x47: Serial.print(F("PCA9685")); break;
    case 0x48: Serial.print(F("ADS1115,PN532,TMP102,LM75,PCF8591,CS42448")); break;
    case 0x49: Serial.print(F("ADS1115,TSL2561,PCF8591,CS42448,TC74A1")); break;
    case 0x4A: Serial.print(F("ADS1115,Qwiic Keypad,CS42448")); break;
    case 0x4B: Serial.print(F("ADS1115,TMP102,BNO080,Qwiic Keypad,CS42448")); break;
    case 0x50: Serial.print(F("EEPROM,FRAM")); break;
    case 0x51: Serial.print(F("EEPROM")); break;
    case 0x52: Serial.print(F("Nunchuk,EEPROM")); break;
    case 0x53: Serial.print(F("ADXL345,EEPROM")); break;
    case 0x54: Serial.print(F("EEPROM")); break;
    case 0x55: Serial.print(F("EEPROM")); break;
    case 0x56: Serial.print(F("EEPROM")); break;
    case 0x57: Serial.print(F("EEPROM")); break;
    case 0x58: Serial.print(F("TPA2016,MAX21100")); break;
    case 0x5A: Serial.print(F("MPR121,MLX90614")); break;
    case 0x60: Serial.print(F("MPL3115,MCP4725,MCP4728,TEA5767,Si5351")); break;
    case 0x61: Serial.print(F("MCP4725,AtlasEzoDO")); break;
    case 0x62: Serial.print(F("LidarLite,MCP4725,AtlasEzoORP")); break;
    case 0x63: Serial.print(F("MCP4725,AtlasEzoPH")); break;
    case 0x64: Serial.print(F("AtlasEzoEC, ADAU1966")); break;
    case 0x66: Serial.print(F("AtlasEzoRTD")); break;
    case 0x68: Serial.print(F("DS1307,DS3231,MPU6050,MPU9050,MPU9250,ITG3200,ITG3701,LSM9DS0,L3G4200D")); break;
    case 0x69: Serial.print(F("MPU6050,MPU9050,MPU9250,ITG3701,L3G4200D")); break;
    case 0x6A: Serial.print(F("LSM9DS1")); break;
    case 0x6B: Serial.print(F("LSM9DS0")); break;
    case 0x6F: Serial.print(F("Qwiic Button")); break;
    case 0x70: Serial.print(F("HT16K33,TCA9548A")); break;
    case 0x71: Serial.print(F("SFE7SEG,HT16K33")); break;
    case 0x72: Serial.print(F("HT16K33")); break;
    case 0x73: Serial.print(F("HT16K33")); break;
    case 0x76: Serial.print(F("MS5607,MS5611,MS5637,BMP280")); break;
    case 0x77: Serial.print(F("BMP085,BMA180,BMP280,MS5611")); break;
    case 0x7C: Serial.print(F("FRAM_ID")); break;
    default: Serial.print(F("unknown chip"));
  }
}

void scan(TwoWire &myport) {
  int nDevices = 0;
  for (int address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    myport.beginTransmission(address);
    int error = myport.endTransmission();

    if (error == 0) {
      Serial.print(F("Device found at address 0x"));
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address,HEX);
      Serial.print("  (");
      printKnownChips(address);
      Serial.println(")");
      nDevices++;
    } else if (error==4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println(F("No I2C devices found"));
  } else {
    Serial.println(F("done"));
  }
  Serial.println();
}



void setup() {

  Serial.begin(115200);
  delay(5000);

  
  Serial.println("Starting Code");
  
  Wire.begin(0x51);

  Wire.setPins(1, 0);

}


void loop() {
  Serial.println();

  Serial.println(F("Scanning Wire..."));
  scan(Wire);
  delay(500);

  delay(5000);           // wait 5 seconds for next scan
}

*/









// --------------------------------------
// i2c_scanner
// https://playground.arduino.cc/Main/I2cScanner/
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
/*
#include <Wire.h>

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void onRequest() {
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}

void onReceive(int len) {
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Starting...");
  Serial.setDebugOutput(true);
  Wire.setPins(1,0); 

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  //Wire.setPins();
  Wire.begin((uint8_t)I2C_DEV_ADDR);

#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
#endif
}

void loop() {
 // Serial.println("Loop");
//  delay(200);

}

*/