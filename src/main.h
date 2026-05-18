
/*
Rx commands with get a integer with the folllowing enum

we will then report an id as necessaryh

0 will be an id for the item
1 will be the value itself

*/

#define DEBUGMODESTART true
#define DEBUGMODELOOP false

enum IncomingCommands{

    HOMEALL = 0,
    HOMEBAR = 1,
    HOMEPLATES = 2,
    LIFTONEPLATE = 3,
    LOWERONEPLATE = 4,
    SMALLSTEP = 5,
    RAISEBOTHPLATES = 6,
    PREPPRINT = 7,
    MOVEBAR = 8,
    INITALL = 9,
    STARTFAN = 10,
    STOPFAN = 11

    



} incommingCommands;


int const LoadEN = 9, LoadSTEP = 8, LoadDIR = 7;
int const BuildEN = 6, BuildSTEP = 5, BuildDIR = 4;

int const leftEN = 38, leftDIR = 36, leftSTEP = 37;
int const rightEN = 35, rightDIR = 33, rightSTEP = 34;

int const FANLEFTPWM1 = 10, FANLEFTPWM2 = 11;
int const FANRIGHTPWM1 = 12, FANRIGHTPWM2 = 13;

// Parking position the arm drives to after the ResetLimit switch fires.
// MUST be a position the arm can actually travel TO from the reset limit
// (i.e. on the opposite side of the homing direction). homingDistance is
// -100000 (reset side), so armHome lives on the positive / load side.
long  const armHome = 0;
long const oneSwipedistance = 10000;

//long const buildingDistance = 110000; // Total plate distance for the load and build plates.
long const totalPlateDistance = 120000; // Total plate distance for the load and build plates. 

const int resetLimit = 3, loadLimit = 40;
const int buildPlateLimit = 2, loadPlateLimit = 42;

const int SDAPIN = 1, SCLPIN = 0;

// ---- TMC2209 UART (BuildPlateStepper has broken STEP/DIR; driven via UART VACTUAL) ----
int   const TMC_RX_PIN  = 18;     // MCU RX <- PDN_UART (direct)
int   const TMC_TX_PIN  = 17;     // MCU TX -> PDN_UART (through 1k pull-up)
long  const TMC_BAUD    = 115200;
float const TMC_RSENSE  = 0.11f;  // BTT/FYSETC carrier default
// MS1/MS2 jumpers must select these slave addresses on the carriers:
//   BuildPlate driver: MS1=GND, MS2=GND -> address 0
//   LoadPlate  driver: MS1=VCC, MS2=VCC -> address 3
uint8_t const BUILD_TMC_ADDR = 0;
uint8_t const LOAD_TMC_ADDR  = 3;
// ~2800 LSB * 0.715 Hz/LSB ~= 2000 microsteps/s, matching Axis::Speed::MEDIUM.
int32_t const BUILD_VACTUAL  = 2800;

    namespace PlateID{
        int LOADPLATE = 1;
        int BUILDPLATE = 0;
    }
