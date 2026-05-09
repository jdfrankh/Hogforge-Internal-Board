
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

long  const armHome = -100000;
long const oneSwipedistance = 100000;

long const totalPlateDistance = 300000;

const int resetLimit = 3, loadLimit = 40;
const int buildPlateLimit = 2, loadPlateLimit = 42;

const int SDAPIN = 0, SCLPIN = 1;

    namespace PlateID{
        int LOADPLATE = 1;
        int BUILDPLATE = 0;
    }
