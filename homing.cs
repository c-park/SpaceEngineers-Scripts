//----- Refer To Steam Workshop Discussion Section For Variables Definition ----- 
 
//Configures the missile launch mode to use when script timer loop is activated: 
// 
//  0 = Launch missile as Active Radar Homing Mode using launching ship's R_REMOTE for initial lock-on. Switches over to on-board Radar when lock-on is stable. 
//  1 = Launch missile as Active Radar Homing Mode using coordinates from launching ship's R_TARGET for initial lock-on. Switches over to on-board Radar when lock-on is stable. 
//  2 = Launch missile as Semi-Active Radar Homing Mode using launching ship's R_REMOTE as the lock-on Radar permanently. 
//  3 = Launch missile as Camera Guided Mode using launching ship's R_CAMERA for direction tracking. 
//  4 = Launch missile as Cruise Mode using launching ship's R_TARGET as GPS coordinates. 
//  99 = Launch missile in Dummy Mode. This is used for testing if the missile detaches properly at the correct connection and display missile detected configurations. 
// 
int missileLaunchType = 0; 
 
//Type of block to disconnect missile from launching ship: 0 = Merge Block, 1 = Rotor, 2 = Connector, 3 = Merge Block And Any Locked Connectors, 4 = Rotor And Any Locked Connectors, 99 = No detach required 
int missileDetachPortType = 0; 
 
//Spin Missile By This RPM After Launch Clearance 
int spinAmount = 0; 
 
//Whether to perform a vertical takeoff for the launching procedure 
bool verticalTakeoff = false; 
 
//Whether to reduce the GFD beam width by the radius of your missile 
bool excludeGFDMissileRadius = false; 
 
//------------------------------ Reference Block Name Configuration ------------------------------ 
 
string strShipRefRemoteControl = "R_REMOTE";        //Mode 0 and 2 only 
string strShipRefTargetPanel = "R_TARGET";          //Mode 1 and 4 only 
string strShipRefCamera = "R_CAMERA";               //Mode 3 only 
 
//By default all gyroscopes, thrusters and merge blocks will be considered for use. Setting a value here limits the script to use specific set of blocks 
const string strGyroscopesPrefix = ""; 
const string strThrustersPrefix = ""; 
const string strDetachPort = ""; 
const string strDirectionRefBlock = ""; 
const string strLockOnSensor = "R_SENSOR"; 
 
//For debugging purposes 
const string strStatusDisplayPrefix = "<D>"; 
 
//------------------------------ Missile Handling Configuration ------------------------------ 
 
double driftVectorReduction = 1.5; 
double launchSeconds = 1; 
 
//By default, script will determine whether to use these parameters. Setting a value forces script to use it 
bool? boolDrift = null; 
bool? boolLeadTarget = null; 
bool? boolNaturalDampener = null; 
 
//------------------------------ GetFreeDestination Lock On Configuration ------------------------------ 
 
const float GFD_MIN_LOCK_DISTANCE = 100;                //Minimum distance allowed for Remote Control to lock-on 
const float GFD_MAX_LOCK_DISTANCE = 20000;              //Maximum distance allowed for Remote Control to lock-on 
const float GFD_MIN_INITIAL_DISTANCE = 100;             //Minimum distance allowed for initial lock on 
float GFD_BEAM_WIDTH = 0f;                              //Width of the locking beam. You can set less than zero to shrink all detected target radius (e.g. Setting -10f means all targets with radius 10m will be invisible. However, targets less than 10m will be inverse expanded; 2m targets will be 8m) 
 
const int GFD_STABILIZATION_TICKS = 30;                 //Number of ticks the on-board lock-on has to maintain to evaluate lock-on is stabilized 
const float GFD_STABILIZATION_DISTANCE = 1;             //Gap distance in metres the on-board and launching ship's lock-on must keep within to evaluate lock-on is stabilizing 
 
const double GFD_INTERRUPT_THRESHOLD = 50;              //Amount of distance change in one tick to declare GFD lock interrupted 
 
const int GFD_FULL_LOCK_INTERVAL = 60;                  //How many ticks to wait before using full 2 beam GFD lock-on algorithm. In between those, the shortcut 1 beam GFD lock-on will be used 
 
//------------------------------ Above Is User Configuration Section. This Section Is For PID Tuning ------------------------------ 
 
const double DEF_SMALL_GRID_P = 240;                    //The default proportional gain of small grid gyroscopes 
const double DEF_SMALL_GRID_I = 0;                      //The default integral gain of small grid gyroscopes 
const double DEF_SMALL_GRID_D = 200;                    //The default derivative gain of small grid gyroscopes 
 
const double DEF_BIG_GRID_P = 50;                       //The default proportional gain of large grid gyroscopes 
const double DEF_BIG_GRID_I = 0.5;                      //The default integral gain of large grid gyroscopes 
const double DEF_BIG_GRID_D = 4;                        //The default derivative gain of large grid gyroscopes 
 
bool useDefaultPIDValues = true;                        //Whether to use predefined PID values based on detected grid size 
 
double AIM_P = 0;                                       //The proportional gain of the gyroscope turning (Set useDefaultPIDValues to true to use default values) 
double AIM_I = 0;                                       //The integral gain of the gyroscope turning (Set useDefaultPIDValues to true to use default values) 
double AIM_D = 0;                                       //The derivative gain of the gyroscope turning (Set useDefaultPIDValues to true to use default values) 
double AIM_LIMIT = 60;                                  //Limit value of both yaw and pitch combined 
 
double INTEGRAL_WINDUP_LIMIT = 30;                      //Integral value limit to minimize integral windup. Zero means no limit 
 
//------------------------------ Script Parameters Configuration ------------------------------ 
 
const int MERGE_SEPARATE_WAIT_THRESHOLD = 60; 
 
//------------------------------ Below Is Main Script Body ------------------------------ 
 
IMyRemoteControl shipRefRemoteControl = null; 
IMyTextPanel shipRefTargetPanel = null; 
IMyTerminalBlock shipRefCamera = null; 
 
IMyRemoteControl remoteControl = null; 
IMyTerminalBlock refForwardBlock = null; 
IMyTerminalBlock refDownwardBlock = null; 
IMySensorBlock lockOnSensor = null; 
IMyTerminalBlock statusDisplay = null; 
 
List<IMyTerminalBlock> gyroscopes = null; 
string[] gyroYawField = null; 
string[] gyroPitchField = null; 
string[] gyroRollField = null; 
float[] gyroYawFactor = null; 
float[] gyroPitchFactor = null; 
float[] gyroRollFactor = null; 
 
List<IMyTerminalBlock> thrusters = null; 
float[] thrustValues = null; 
 
List<IMyTerminalBlock> launchThrusters = null; 
 
MatrixD refWorldMatrix = default(MatrixD); 
MatrixD refLookAtMatrix = default(MatrixD); 
bool refForwardReverse = false; 
 
Vector3D naturalGravity = new Vector3D(); 
double naturalGravityLength = 0; 
 
Vector3D driftVector = new Vector3D(); 
double speed = 0; 
double rpm = 0; 
 
IMyCubeGrid lockOnGrid = null; 
IMyCubeGrid lastDetectedGrid = null; 
 
Vector3D remoteTargetLocation = new Vector3D(); 
Vector3D remoteTargetPosition = new Vector3D(); 
double remoteTargetRadius = 0; 
bool remoteTargetFound = false; 
 
Vector3D targetPosition = new Vector3D(); 
Vector3D lastTargetPosition = new Vector3D(); 
 
bool targetPositionSet = false; 
int lastTargetPositionClock = 0; 
 
Vector3D targetVector = new Vector3D(); 
double distToTarget = 0; 
 
Vector3D targetDirection = new Vector3D(); 
double targetSpeed = 0; 
 
double targetYawAngle = 0; 
double targetPitchAngle = 0; 
double targetRollAngle = 0; 
 
double lastYawError = 0; 
double lastYawIntegral = 0; 
double lastPitchError = 0; 
double lastPitchIntegral = 0; 
double lastRollError = 0; 
double lastRollIntegral = 0; 
 
int subCounter = 0; 
int subMode = 0; 
int mode = 0; 
int clock = 0; 
bool init = false; 
 
IMyTerminalBlock detachBlockLocal = null; 
IMyTerminalBlock detachBlockRemote = null; 
 
string nameMatcher = ""; 
 
Random rnd = new Random(); 
 
const double RPM_FACTOR = 1800 / Math.PI; 
const double ACOS_FACTOR = 180 / Math.PI; 
const float GYRO_FACTOR = (float)(Math.PI / 30);  
 
Vector3D Y_VECTOR = new Vector3D(0, -1, 0); 
Vector3D Z_VECTOR = new Vector3D(0, 0, -1); 
Vector3D POINT_ZERO = new Vector3D(0, 0, 0); 
 
void Main(string arguments) 
{ 
    //---------- Initialization And General Controls ---------- 
 
    if (!init) 
    { 
        if (subMode == 0)       //Check for configuration command 
        { 
            subMode = 1; 
 
            if (arguments != null && arguments.Length > 0) 
            { 
                ProcessConfigurationCommand(arguments); 
                return; 
            } 
        } 
         
        if (subMode == 1)       //Missile still on launching ship's grid 
        { 
            if (!InitLaunchingShipRefBlocks()) 
            { 
                throw new Exception("--- Initialization Failed ---"); 
            } 
 
            if (!DetachFromGrid()) 
            { 
                throw new Exception("--- Initialization Failed ---"); 
            } 
 
            subCounter = 0; 
            subMode = (missileDetachPortType == 99 ? 3 : 2); 
            return; 
        } 
        else if (subMode == 2)  //Missile waiting for successful detachment from launching ship 
        { 
            if (detachBlockLocal == null || detachBlockRemote == null) 
            { 
                subMode = 3; 
                return; 
            } 
            else if (detachBlockLocal.CubeGrid != detachBlockRemote.CubeGrid) 
            { 
                subMode = 3; 
                return; 
            } 
            else 
            { 
                subCounter++; 
 
                if (subCounter >= MERGE_SEPARATE_WAIT_THRESHOLD) 
                { 
                    Echo("Error: Missile detach failed."); 
                    throw new Exception("--- Initialization Failed ---"); 
                } 
                 
                return; 
            } 
        } 
        else if (subMode == 3)  //Missile successfully detached and currently initializing 
        { 
            if (missileDetachPortType == 3 || missileDetachPortType == 4) 
            { 
                DetachLockedConnectors(); 
            } 
             
            if (!InitMissileBlocks()) 
            { 
                throw new Exception("--- Initialization Failed ---"); 
            } 
        } 
         
        if (missileLaunchType == 99) 
        { 
            subMode = 0; 
            mode = 99; 
            clock = 0; 
        } 
        else 
        { 
            if (excludeGFDMissileRadius) 
            { 
                GFD_BEAM_WIDTH = (float)(ComputeMissileDiagonalVector().Length() / -2); 
            } 
             
            subCounter = (int)(launchSeconds * 60); 
            FireThrusters(verticalTakeoff ? launchThrusters : thrusters, true); 
 
            subMode = (missileLaunchType == 0 || missileLaunchType == 2 ? 1 : 0); 
            mode = 1; 
            clock = 0; 
        } 
         
        init = true; 
        return; 
    } 
 
    //---------- Modes And Controls ---------- 
 
    clock += 1; 
 
    CalculateParameters(); 
 
    if (mode == 1)          //Launching 
    { 
        if (subMode == 1) 
        { 
            if (!targetPositionSet) 
            { 
                remoteTargetLocation = shipRefRemoteControl.WorldMatrix.Forward + shipRefRemoteControl.WorldMatrix.Translation; 
            } 
             
            if (FindGFDTarget(shipRefRemoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation)) 
            { 
                targetPositionSet = true; 
            } 
            else 
            { 
                targetPositionSet = false; 
            } 
 
            if (targetPositionSet) 
            { 
                if ((remoteTargetPosition - shipRefRemoteControl.WorldMatrix.Translation).Length() < GFD_MIN_INITIAL_DISTANCE || (GetMissileMidPoint() - remoteTargetPosition).Length() < 1) 
                { 
                    targetPositionSet = false; 
                } 
            } 
 
            if (targetPositionSet) 
            { 
                targetPosition = remoteTargetPosition; 
            } 
 
            if (subCounter > 0) 
            { 
                subCounter--; 
            } 
            else if (targetPositionSet) 
            { 
                if (verticalTakeoff) 
                { 
                    FireThrusters(launchThrusters, false); 
                    FireThrusters(thrusters, true); 
                } 
 
                SetGyroOverride(true); 
 
                if (spinAmount > 0) 
                { 
                    SetGyroRoll(spinAmount); 
                } 
 
                subCounter = 0; 
                subMode = 0; 
                mode = 10 + missileLaunchType; 
            } 
            else 
            { 
                if (verticalTakeoff && subCounter == 0) 
                { 
                    FireThrusters(launchThrusters, false); 
                    FireThrusters(thrusters, true); 
 
                    subCounter = -1; 
                } 
 
                if (boolNaturalDampener == true) 
                { 
                    AimAtNaturalGravity(); 
                } 
            } 
        } 
        else 
        { 
            if (subCounter > 0) 
            { 
                subCounter--; 
            } 
            else 
            { 
                if (verticalTakeoff) 
                { 
                    FireThrusters(launchThrusters, false); 
                    FireThrusters(thrusters, true); 
                } 
 
                SetGyroOverride(true); 
 
                if (spinAmount > 0) 
                { 
                    SetGyroRoll(spinAmount); 
                } 
 
                lastTargetPosition = targetPosition = GetFlyStraightVector(); 
 
                subCounter = 0; 
                subMode = 0; 
                mode = 10 + missileLaunchType; 
            } 
        } 
    } 
    else if (mode == 10)    //Active Radar Homing With Initial Shipborne GFD Lock-On 
    { 
        if (lockOnSensor != null && CheckLockOnSensor()) 
        { 
            mode = 20; 
        } 
 
        if (subMode == 0)       //Initial Lock-On 
        { 
            if (!targetPositionSet) 
            { 
                remoteTargetLocation = shipRefRemoteControl.WorldMatrix.Forward + shipRefRemoteControl.WorldMatrix.Translation; 
            } 
             
            if (FindGFDTarget(shipRefRemoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation)) 
            { 
                targetPositionSet = true; 
            } 
            else 
            { 
                targetPositionSet = false; 
            } 
 
            if (targetPositionSet) 
            { 
                if ((remoteTargetPosition - shipRefRemoteControl.WorldMatrix.Translation).Length() < GFD_MIN_INITIAL_DISTANCE || (GetMissileMidPoint() - remoteTargetPosition).Length() < 1) 
                { 
                    targetPositionSet = false; 
                } 
            } 
 
            bool missileLocked = false; 
 
            if (targetPositionSet) 
            { 
                targetPosition = remoteTargetPosition; 
 
                if (FindGFDTarget(remoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, remoteTargetRadius, out remoteTargetPosition, out remoteTargetLocation)) 
                { 
                    if ((remoteTargetPosition - targetPosition).Length() < GFD_STABILIZATION_DISTANCE) 
                    { 
                        missileLocked = true; 
                    } 
                } 
            } 
 
            if (missileLocked) 
            { 
                subCounter += 1; 
 
                if (subCounter >= GFD_STABILIZATION_TICKS) 
                { 
                    lastTargetPosition = targetPosition; 
                    lastTargetPositionClock = clock; 
                     
                    subCounter = 0; 
                    subMode = 1; 
                } 
            } 
            else 
            { 
                subCounter = 0; 
            } 
 
            if (targetPositionSet) 
            { 
                CalculateTargetParameters(); 
                targetPositionSet = true; 
            } 
            else 
            { 
                CalculateTargetParameters(); 
            } 
        } 
        else if (subMode == 1)  //Active Radar Homing 
        { 
            if (subCounter <= 0) 
            { 
                remoteTargetFound = FindGFDTarget(remoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation); 
                subCounter = GFD_FULL_LOCK_INTERVAL - 1; 
            } 
            else 
            { 
                remoteTargetFound = FindGFDTarget(remoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation); 
                subCounter--; 
            } 
 
            if (remoteTargetFound) 
            { 
                targetPosition = remoteTargetPosition; 
                targetPositionSet = true; 
            } 
            else 
            { 
                targetPositionSet = false; 
            } 
 
            CalculateTargetParameters(); 
        } 
         
        AimAtTarget(); 
 
        if (boolNaturalDampener == true) 
        { 
            AimAtNaturalGravity(); 
        } 
    } 
    else if (mode == 11)    //Active Radar Homing With Initial Shipborne GPS Coordinates Lock-On 
    { 
        if (lockOnSensor != null && CheckLockOnSensor()) 
        { 
            mode = 20; 
        } 
 
        if (subMode == 0)       //Initial Lock-On 
        { 
            Vector3D parsedVector; 
            if (ParseCoordinates(shipRefTargetPanel.GetPublicTitle(), out parsedVector)) 
            { 
                targetPosition = parsedVector; 
                targetPositionSet = true; 
            } 
            else 
            { 
                lastTargetPosition = targetPosition = GetFlyStraightVector(); 
                targetPositionSet = false; 
            } 
 
            if (targetPositionSet && (GetMissileMidPoint() - targetPosition).Length() < 1) 
            { 
                lastTargetPosition = targetPosition = GetFlyStraightVector(); 
                targetPositionSet = false; 
            } 
             
            bool missileLocked = false; 
 
            if (targetPositionSet) 
            { 
                if (FindGFDTarget(remoteControl, targetPosition, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation)) 
                { 
                    if ((remoteTargetPosition - targetPosition).Length() < remoteTargetRadius) 
                    { 
                        missileLocked = true; 
                    } 
                } 
            } 
 
            if (missileLocked) 
            { 
                subCounter += 1; 
 
                if (subCounter >= GFD_STABILIZATION_TICKS) 
                { 
                    lastTargetPosition = targetPosition; 
                    lastTargetPositionClock = clock; 
                     
                    subCounter = 0; 
                    subMode = 1; 
                } 
            } 
            else 
            { 
                subCounter = 0; 
            } 
        } 
        else if (subMode == 1)  //Active Radar Homing 
        { 
            if (subCounter <= 0) 
            { 
                remoteTargetFound = FindGFDTarget(remoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation); 
                subCounter = GFD_FULL_LOCK_INTERVAL - 1; 
            } 
            else 
            { 
                remoteTargetFound = FindGFDTarget(remoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation); 
                subCounter--; 
            } 
 
            if (remoteTargetFound) 
            { 
                targetPosition = remoteTargetPosition; 
                targetPositionSet = true; 
            } 
            else 
            { 
                targetPositionSet = false; 
            } 
        } 
         
        CalculateTargetParameters(); 
        AimAtTarget(); 
 
        if (boolNaturalDampener == true) 
        { 
            AimAtNaturalGravity(); 
        } 
    } 
    else if (mode == 12)    //Semi-Active Radar Homing Using Shipborne GFD Lock-On 
    { 
        if (lockOnSensor != null && CheckLockOnSensor()) 
        { 
            mode = 20; 
        } 
         
        if (subCounter <= 0) 
        { 
            remoteTargetFound = FindGFDTarget(shipRefRemoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation); 
            subCounter = GFD_FULL_LOCK_INTERVAL - 1; 
        } 
        else 
        { 
            remoteTargetFound = FindGFDTarget(shipRefRemoteControl, remoteTargetLocation, GFD_MAX_LOCK_DISTANCE, out remoteTargetPosition, out remoteTargetRadius, out remoteTargetLocation); 
            subCounter--; 
        } 
 
        if ((GetMissileMidPoint() - remoteTargetPosition).Length() < 1) 
        { 
            remoteTargetFound = false; 
        } 
 
        if (remoteTargetFound) 
        { 
            targetPosition = remoteTargetPosition; 
            targetPositionSet = true; 
        } 
        else 
        { 
            targetPositionSet = false; 
            subCounter = 0; 
        } 
         
        CalculateTargetParameters(); 
        AimAtTarget(); 
 
        if (boolNaturalDampener == true) 
        { 
            AimAtNaturalGravity(); 
        } 
    } 
    else if (mode == 13)    //Camera Guided Mode 
    { 
        if (lockOnSensor != null && CheckLockOnSensor()) 
        { 
            mode = 20; 
        } 
         
        MatrixD shipWorldMatrix = shipRefCamera.WorldMatrix; 
        Vector3D shipToMissileVector = remoteControl.GetPosition() - shipWorldMatrix.Translation; 
        Vector3D missileToViewLineVector = Vector3D.Reject(shipToMissileVector, shipWorldMatrix.Forward); 
         
        double extraDistanceExtend = Math.Max(5.6713 * missileToViewLineVector.Length(), speed * 2); 
        extraDistanceExtend += (shipToMissileVector - missileToViewLineVector).Length(); 
 
        targetPosition = shipWorldMatrix.Translation + (shipWorldMatrix.Forward * extraDistanceExtend); 
        targetPositionSet = true; 
         
        CalculateTargetParameters(); 
        AimAtTarget(); 
 
        if (boolNaturalDampener == true) 
        { 
            AimAtNaturalGravity(); 
        } 
    } 
    else if (mode == 14)    //Cruise Mode 
    { 
        if (lockOnSensor != null && CheckLockOnSensor()) 
        { 
            mode = 20; 
        } 
 
        Vector3D parsedVector; 
        if (ParseCoordinates(shipRefTargetPanel.GetPublicTitle(), out parsedVector)) 
        { 
            targetPosition = parsedVector; 
            targetPositionSet = true; 
        } 
         
        CalculateTargetParameters(); 
        AimAtTarget(); 
 
        if (boolNaturalDampener == true) 
        { 
            AimAtNaturalGravity(); 
        } 
    } 
    else if (mode == 20)    //Sensor Chase 
    { 
        targetPosition = GetLockOnGridPosition(); 
        targetPositionSet = true; 
 
        CalculateTargetParameters(); 
        AimAtTarget(); 
 
        if (boolNaturalDampener == true) 
        { 
            AimAtNaturalGravity(); 
        } 
    } 
 
    if (statusDisplay != null) 
    { 
        if (mode == 0) 
        { 
            DisplayStatus("Idle"); 
        } 
        else if (mode == 1) 
        { 
            DisplayStatus("Launching"); 
        } 
        else if (mode == 10 || mode == 11) 
        { 
            if (subMode == 0) 
            { 
                DisplayStatus("Initial Lock - Stable Ticks: " + subCounter); 
            } 
            else if (subMode == 1) 
            { 
                DisplayStatus((remoteTargetFound ? "Lock" : "Trace") + ": [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]"); 
            } 
            else 
            { 
                DisplayStatus("-"); 
            } 
        } 
        else if (mode == 12) 
        { 
            DisplayStatus((remoteTargetFound ? "Lock" : "Trace") + ": [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]"); 
        } 
        else if (mode == 13) 
        { 
            DisplayStatus("Camera"); 
        } 
        else if (mode == 14) 
        { 
            DisplayStatus("Aim: [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]"); 
        } 
        else if (mode == 20) 
        { 
            DisplayStatus("Sensor: [" + Math.Round(targetPosition.GetDim(0), 2) + "," + Math.Round(targetPosition.GetDim(1), 2) + "," + Math.Round(targetPosition.GetDim(2), 2) + "]"); 
        } 
        else 
        { 
            DisplayStatus("-"); 
        } 
    } 
} 
 
//------------------------------ Miscellaneous Methods ------------------------------ 
 
void DisplayStatus(string statusMsg) 
{ 
    if (statusDisplay != null) 
    { 
        statusDisplay.SetCustomName(strStatusDisplayPrefix + " Mode: " + mode + ", " + statusMsg); 
    } 
} 
 
bool CheckLockOnSensor() 
{ 
    bool switchToGrid = false; 
    IMyCubeGrid detectedGrid = lockOnSensor.LastDetectedEntity as IMyCubeGrid; 
 
    if (lastDetectedGrid != detectedGrid) 
    { 
        if (detectedGrid != null) 
        { 
            if (lockOnGrid == null) 
            { 
                switchToGrid = true; 
            } 
            else 
            { 
                double detectedGridLength = (detectedGrid.GridIntegerToWorld(detectedGrid.Min) - detectedGrid.GridIntegerToWorld(detectedGrid.Max)).Length(); 
                double lockOnGridLength = (lockOnGrid.GridIntegerToWorld(lockOnGrid.Min) - lockOnGrid.GridIntegerToWorld(lockOnGrid.Max)).Length(); 
 
                switchToGrid = (detectedGridLength > lockOnGridLength); 
            } 
 
            if (switchToGrid) 
            { 
                lockOnGrid = detectedGrid; 
                return true; 
            } 
        } 
        lastDetectedGrid = detectedGrid; 
    } 
 
    return false; 
} 
 
Vector3D GetLockOnGridPosition() 
{ 
    return lockOnGrid.GridIntegerToWorld((lockOnGrid.Min + lockOnGrid.Max) / 2); 
} 
 
Vector3D GetMissileMidPoint() 
{ 
    return (Me.CubeGrid.GridIntegerToWorld(Me.CubeGrid.Min) + Me.CubeGrid.GridIntegerToWorld(Me.CubeGrid.Max)) / 2; 
} 
 
Vector3D GetFlyStraightVector() 
{ 
    return (driftVector * 1000) + remoteControl.GetPosition(); 
} 
 
//------------------------------ Missile And Target Information Methods ------------------------------ 
 
void CalculateParameters() 
{ 
    //---------- Calculate Missile Related Variables ---------- 
     
    refWorldMatrix = refForwardBlock.WorldMatrix; 
    refLookAtMatrix = MatrixD.CreateLookAt(POINT_ZERO, (refForwardReverse ? refWorldMatrix.Backward : refWorldMatrix.Forward), refWorldMatrix.Up); 
 
    driftVector = remoteControl.GetShipVelocities().LinearVelocity; 
    speed = driftVector.Length(); 
     
    naturalGravity = remoteControl.GetNaturalGravity(); 
    naturalGravityLength = naturalGravity.Length(); 
    naturalGravity = (naturalGravityLength > 0 ? naturalGravity / naturalGravityLength : POINT_ZERO); 
} 
 
void CalculateTargetParameters() 
{ 
    //---------- Calculate Target Parameters ---------- 
 
    if (boolLeadTarget == true) 
    { 
        if (targetPositionSet) 
        { 
            targetDirection = targetPosition - lastTargetPosition; 
            targetSpeed = targetDirection.Length(); 
 
            if (targetSpeed > 0) 
            { 
                targetDirection = targetDirection / targetSpeed; 
                targetSpeed = targetSpeed * 60 / (clock - lastTargetPositionClock); 
            } 
 
            lastTargetPosition = targetPosition; 
            lastTargetPositionClock = clock; 
 
            targetPositionSet = false; 
        } 
        else 
        { 
            targetPosition = lastTargetPosition + ((targetDirection * targetSpeed) / 60 * (clock - lastTargetPositionClock)); 
        } 
 
        if (targetSpeed > 0) 
        { 
            Vector3D aimPosition = ComputeIntersectionPoint(targetDirection, targetPosition, targetSpeed, remoteControl.GetPosition(), speed); 
            if (!Double.IsNaN(aimPosition.Sum)) 
            { 
                targetPosition = aimPosition; 
            } 
        } 
    } 
 
    targetVector = targetPosition - remoteControl.GetPosition(); 
    distToTarget = targetVector.Length(); 
    targetVector = targetVector / distToTarget; 
 
    if (boolDrift == true && speed >= 5) 
    { 
        targetVector = (targetVector * speed) - (driftVector / driftVectorReduction); 
        targetVector.Normalize(); 
    } 
     
    targetVector = Vector3D.TransformNormal(targetVector, refLookAtMatrix); 
    targetVector.Normalize(); 
 
    if (Double.IsNaN(targetVector.Sum)) 
    { 
        targetVector = new Vector3D(Z_VECTOR); 
    } 
} 
 
//------------------------------ Missile Lock-On And Leading Methods ------------------------------ 
 
//Two beam GFD lock-on method. This is the optimized GFD lock-on algorithm. 
bool FindGFDTarget(IMyRemoteControl remoteControl, Vector3D targetLocation, float targetDistance, out Vector3D targetPosition, out double targetRadius, out Vector3D nextTargetLocation) 
{ 
    Vector3D remotePosition = remoteControl.WorldMatrix.Translation; 
    Vector3D targetVector1 = targetLocation - remotePosition; 
    targetVector1.Normalize(); 
 
    Vector3D targetPoint1 = (targetVector1 * targetDistance) + remotePosition; 
    Vector3D offsetTarget1 = remoteControl.GetFreeDestination(targetPoint1, targetDistance, GFD_BEAM_WIDTH); 
    double remoteToOffsetDist1 = (offsetTarget1 - remotePosition).Length(); 
 
    if ((targetPoint1 - offsetTarget1).Length() < 1) 
    { 
        targetPosition = targetLocation; 
        targetRadius = 0; 
 
        nextTargetLocation = targetLocation; 
 
        return false; 
    } 
 
    Vector3D estimatedPoint1 = (targetVector1 * remoteToOffsetDist1) + remotePosition; 
    Vector3D targetNormal1 = targetVector1.Cross(offsetTarget1 - estimatedPoint1); 
    targetNormal1.Normalize(); 
 
    Vector3D targetVector2 = offsetTarget1 - estimatedPoint1; 
    targetVector2 = targetVector2 / targetVector2.Length(); 
    targetVector2 = (estimatedPoint1 - targetVector2 + targetNormal1) - remotePosition; 
    targetVector2.Normalize(); 
 
    Vector3D targetPoint2 = (targetVector2 * targetDistance) + remotePosition; 
    Vector3D offsetTarget2 = remoteControl.GetFreeDestination(targetPoint2, targetDistance, GFD_BEAM_WIDTH); 
    double remoteToOffsetDist2 = (offsetTarget2 - remotePosition).Length(); 
 
    if ((targetPoint2 - offsetTarget2).Length() < 1) 
    { 
        targetPosition = targetLocation; 
        targetRadius = 0; 
 
        nextTargetLocation = targetLocation; 
 
        return false; 
    } 
 
    Vector3D estimatedPoint2 = (targetVector2 * remoteToOffsetDist2) + remotePosition; 
    Vector3D targetNormal2 = targetVector2.Cross(offsetTarget2 - estimatedPoint2); 
    targetNormal2.Normalize(); 
 
    Vector3D intersectVector = targetNormal1.Cross(targetNormal2); 
 
    double zTopValue = (remotePosition - offsetTarget2).LengthSquared() - (remotePosition - offsetTarget1).LengthSquared(); 
    double zBottomValue = 2 * intersectVector.Dot(offsetTarget2 - offsetTarget1); 
 
    if (zBottomValue != 0) 
    { 
        targetPoint1 = remotePosition + (intersectVector * (zTopValue / zBottomValue)); 
        if (Double.IsNaN(targetPoint1.Sum)) 
        { 
            targetPosition = targetLocation; 
            targetRadius = 0; 
 
            nextTargetLocation = targetLocation; 
 
            return false; 
        } 
        else 
        { 
            double b = targetVector1.Dot(2 * remotePosition - targetPoint1 - offsetTarget1); 
            double c = (remotePosition - targetPoint1).Dot(remotePosition - offsetTarget1); 
            double u = Math.Sqrt((b * b) - (4 * c)); 
             
            targetPosition = targetPoint1; 
            targetRadius = ((Math.Min(-b + u, -b - u) * 0.5 * targetVector1) + remotePosition - targetPoint1).Length() - GFD_BEAM_WIDTH; 
             
            nextTargetLocation = targetPoint1 + targetNormal1; 
 
            return true; 
        } 
    } 
    else 
    { 
        targetPosition = targetLocation; 
        targetRadius = 0; 
 
        nextTargetLocation = targetLocation; 
 
        return false; 
    } 
} 
 
//One beam GFD lock-on shortcut method. This is the shortcut GFD lock-on where it uses a pre-calculated radius. 
bool FindGFDTarget(IMyRemoteControl remoteControl, Vector3D targetLocation, float targetDistance, double targetRadius, out Vector3D targetPosition, out Vector3D nextTargetLocation) 
{ 
    Vector3D remotePosition = remoteControl.WorldMatrix.Translation; 
    Vector3D targetVector1 = targetLocation - remotePosition; 
    targetVector1.Normalize(); 
 
    Vector3D targetPoint1 = (targetVector1 * targetDistance) + remotePosition; 
    Vector3D offsetTarget1 = remoteControl.GetFreeDestination(targetPoint1, targetDistance, GFD_BEAM_WIDTH); 
 
    if ((targetPoint1 - offsetTarget1).Length() < 1) 
    { 
        targetPosition = targetLocation; 
 
        nextTargetLocation = targetLocation; 
 
        return false; 
    } 
 
    double b = 2 * targetVector1.Dot(remotePosition - offsetTarget1); 
    double c = (remotePosition - offsetTarget1).LengthSquared() - Math.Pow(Math.Max((targetRadius + GFD_BEAM_WIDTH) * 0.5, 20), 2); 
    double u = Math.Sqrt(Math.Round((b * b) - (4 * c), 4)); 
    Vector3D intersectPoint = (Math.Min(-b + u, -b - u) * 0.5 * targetVector1) + remotePosition; 
 
    Vector3D offsetVector1 = offsetTarget1 - intersectPoint; 
    offsetVector1.Normalize(); 
 
    Vector3D coordVector = Vector3D.Reject(targetVector1, offsetVector1); 
    coordVector.Normalize(); 
     
    targetPosition = intersectPoint + (coordVector * (targetRadius + GFD_BEAM_WIDTH)); 
 
    if (Double.IsNaN(targetPosition.Sum)) 
    { 
        targetPosition = targetLocation; 
        nextTargetLocation = targetLocation; 
 
        return false; 
    } 
    else 
    { 
        nextTargetLocation = targetPosition + coordVector; 
     
        return true; 
    } 
} 
 
Vector3D ComputeIntersectionPoint(Vector3D targetDirection, Vector3D targetLocation, double targetSpeed, Vector3D currentLocation, double currentSpeed) 
{ 
    //---------- Calculate Impact Point ---------- 
     
    //targetDirection Must Be Normalized 
    double a = (targetSpeed * targetSpeed) - (currentSpeed * currentSpeed); 
    double b = (2 * targetDirection.Dot(targetLocation - currentLocation) * targetSpeed); 
    double c = (targetLocation - currentLocation).LengthSquared(); 
 
    double t; 
 
    if (a == 0) 
    { 
        t = -c / a; 
    } 
    else 
    { 
        //Use Formula To Find Root: t = ( -b +- sqrt(b^2 - 4ac) ) / 2a 
        double u = (b * b) - (4 * a * c); 
        if (u <= 0) 
        { 
            //Root Cannot Be Found. Target Unreachable 
            return new Vector3D(Double.NaN, Double.NaN, Double.NaN); 
        } 
        u = Math.Sqrt(u); 
 
        double t1 = (-b + u) / (2 * a); 
        double t2 = (-b - u) / (2 * a); 
 
        t = (t1 > 0 ? (t2 > 0 ? (t1 < t2 ? t1 : t2) : t1) : t2); 
    } 
 
    if (t < 0) 
    { 
        return new Vector3D(Double.NaN, Double.NaN, Double.NaN); 
    } 
    else 
    { 
        return targetLocation + (targetDirection * targetSpeed * t); 
    } 
} 
 
//------------------------------ Missile Aiming Methods ------------------------------ 
 
int GetMultiplierSign(double value) 
{ 
    return (value < 0 ? -1 : 1); 
} 
 
void AimAtTarget() 
{ 
    //---------- Activate Gyroscopes To Turn Towards Target ---------- 
 
    Vector3D yawVector = new Vector3D(targetVector.GetDim(0), 0, targetVector.GetDim(2)); 
    Vector3D pitchVector = new Vector3D(0, targetVector.GetDim(1), targetVector.GetDim(2)); 
    yawVector.Normalize(); 
    pitchVector.Normalize(); 
 
    targetYawAngle = Math.Acos(yawVector.Dot(Z_VECTOR)) * GetMultiplierSign(targetVector.GetDim(0)); 
    targetPitchAngle = Math.Acos(pitchVector.Dot(Z_VECTOR)) * GetMultiplierSign(targetVector.GetDim(1)); 
 
    //---------- PID Controller Adjustment ---------- 
 
    lastYawIntegral = lastYawIntegral + (targetYawAngle / 60); 
    double yawDerivative = (targetYawAngle - lastYawError) * 60; 
    lastYawError = targetYawAngle; 
    targetYawAngle = (AIM_P * targetYawAngle) + (AIM_I * lastYawIntegral) + (AIM_D * yawDerivative); 
 
    lastPitchIntegral = lastPitchIntegral + (targetPitchAngle / 60); 
    double pitchDerivative = (targetPitchAngle - lastPitchError) * 60; 
    lastPitchError = targetPitchAngle; 
    targetPitchAngle = (AIM_P * targetPitchAngle) + (AIM_I * lastPitchIntegral) + (AIM_D * pitchDerivative); 
 
    if (Math.Abs(targetYawAngle) + Math.Abs(targetPitchAngle) > AIM_LIMIT) 
    { 
        double adjust = AIM_LIMIT / (Math.Abs(targetYawAngle) + Math.Abs(targetPitchAngle)); 
        targetYawAngle *= adjust; 
        targetPitchAngle *= adjust; 
    } 
 
    //---------- Set Gyroscope Parameters ---------- 
 
    SetGyroYaw(targetYawAngle); 
    SetGyroPitch(targetPitchAngle); 
} 
 
void AimAtNaturalGravity() 
{ 
    //---------- Activate Gyroscopes To Aim Dampener At Natural Gravity ---------- 
     
    if (refDownwardBlock == null || naturalGravityLength < 0.01) 
    { 
        return; 
    } 
     
    MatrixD dampenerLookAtMatrix = MatrixD.CreateLookAt(POINT_ZERO, refDownwardBlock.WorldMatrix.Forward, (refForwardReverse ? refWorldMatrix.Backward : refWorldMatrix.Forward)); 
 
    Vector3D gravityVector = Vector3D.TransformNormal(naturalGravity, dampenerLookAtMatrix); 
    gravityVector.SetDim(1, 0); 
    gravityVector.Normalize(); 
 
    if (Double.IsNaN(gravityVector.Sum)) 
    { 
        gravityVector = new Vector3D(Z_VECTOR); 
    } 
 
    targetRollAngle = Math.Acos(gravityVector.Dot(Z_VECTOR)) * GetMultiplierSign(gravityVector.GetDim(0)); 
 
    //---------- PID Controller Adjustment ---------- 
 
    lastRollIntegral = lastRollIntegral + (targetRollAngle / 60); 
    lastRollIntegral = (INTEGRAL_WINDUP_LIMIT > 0 ? Math.Max(Math.Min(lastRollIntegral, INTEGRAL_WINDUP_LIMIT), -INTEGRAL_WINDUP_LIMIT) : lastRollIntegral); 
    double rollDerivative = (targetRollAngle - lastRollError) * 60; 
    lastRollError = targetRollAngle; 
    targetRollAngle = (AIM_P * targetRollAngle) + (AIM_I * lastRollIntegral) + (AIM_D * rollDerivative); 
     
    //---------- Set Gyroscope Parameters ---------- 
 
    SetGyroRoll(targetRollAngle); 
} 
 
//------------------------------ Missile Separation Methods ------------------------------ 
 
bool DetachFromGrid() 
{ 
    List<IMyTerminalBlock> blocks; 
    IMyTerminalBlock detachBlock; 
 
    switch (missileDetachPortType) 
    { 
        case 0: 
        case 3: 
            blocks = (strDetachPort != null && strDetachPort.Length > 0 ? GetBlocksWithName<IMyShipMergeBlock>(strDetachPort) : GetBlocksOfType<IMyShipMergeBlock>()); 
            detachBlock = GetClosestBlockFromReference(blocks, Me); 
 
            if (detachBlock == null) 
            { 
                Echo("Error: Cannot find any Merge Block " + (strDetachPort != null && strDetachPort.Length > 0 ? "with name " + strDetachPort + " to detach" : "to detach.")); 
                return false; 
            } 
 
            detachBlockLocal = detachBlock; 
            detachBlockRemote = GetConnectedMergeBlock(Me.CubeGrid, detachBlock) as IMyShipMergeBlock; 
 
            if (detachBlockRemote == null) 
            { 
                Echo("Error: Merge Block to be detached is not properly connected to another Merge Block."); 
                return false; 
            } 
 
            detachBlock.ApplyAction("OnOff_Off"); 
            return true; 
        case 1: 
        case 4: 
            blocks = (strDetachPort != null && strDetachPort.Length > 0 ? GetBlocksWithName<IMyMotorStator>(strDetachPort) : GetBlocksOfType<IMyMotorStator>()); 
            detachBlock = GetClosestBlockFromReference(blocks, Me); 
 
            if (detachBlock == null) 
            { 
                Echo("Error: Cannot find any Rotor " + (strDetachPort != null && strDetachPort.Length > 0 ? "with name " + strDetachPort + " to detach" : "to detach.")); 
                return false; 
            } 
             
            detachBlockLocal = Me; 
            detachBlockRemote = detachBlock; 
 
            detachBlock.ApplyAction("Detach"); 
            return true; 
        case 2: 
            blocks = (strDetachPort != null && strDetachPort.Length > 0 ? GetBlocksWithName<IMyShipConnector>(strDetachPort) : GetBlocksOfType<IMyShipConnector>()); 
            detachBlock = GetClosestBlockFromReference(blocks, Me); 
 
            if (detachBlock == null) 
            { 
                Echo("Error: Cannot find any Connector " + (strDetachPort != null && strDetachPort.Length > 0 ? "with name " + strDetachPort + " to detach" : "to detach.")); 
                return false; 
            } 
 
            detachBlockLocal = detachBlock; 
            detachBlockRemote = ((IMyShipConnector)detachBlock).OtherConnector; 
 
            if (detachBlockRemote == null) 
            { 
                Echo("Error: Connector to be detached is not properly connected to another Connector."); 
                return false; 
            } 
 
            detachBlock.ApplyAction("Unlock"); 
            return true; 
        case 99: 
            return true; 
        default: 
            Echo("Error: Unknown missileDetachPortType - " + missileDetachPortType + "."); 
            return false; 
    } 
} 
 
IMyTerminalBlock GetClosestBlockFromReference(List<IMyTerminalBlock> checkBlocks, IMyTerminalBlock referenceBlock) 
{ 
    IMyTerminalBlock checkBlock = null; 
    double prevCheckDistance = Double.MaxValue; 
 
    for (int i = 0; i < checkBlocks.Count; i++) 
    { 
        double currCheckDistance = (checkBlocks[i].GetPosition() - referenceBlock.GetPosition()).Length(); 
        if (currCheckDistance < prevCheckDistance) 
        { 
            prevCheckDistance = currCheckDistance; 
            checkBlock = checkBlocks[i]; 
        } 
    } 
 
    return checkBlock; 
} 
 
IMyTerminalBlock GetConnectedMergeBlock(IMyCubeGrid grid, IMyTerminalBlock mergeBlock) 
{ 
    IMySlimBlock slimBlock = grid.GetCubeBlock(mergeBlock.Position - new Vector3I(Base6Directions.GetVector(mergeBlock.Orientation.Left))); 
    return (slimBlock == null ? null : slimBlock.FatBlock as IMyTerminalBlock); 
} 
 
void DetachLockedConnectors() 
{ 
    List<IMyTerminalBlock> blocks = GetBlocksOfType<IMyShipConnector>(); 
    for (int i = 0; i < blocks.Count; i++) 
    { 
        if (blocks[i].CubeGrid == Me.CubeGrid) 
        { 
            IMyShipConnector otherConnector = ((IMyShipConnector)blocks[i]).OtherConnector; 
            if (otherConnector == null || blocks[i].CubeGrid != otherConnector.CubeGrid) 
            { 
                blocks[i].ApplyAction("Unlock"); 
            } 
        } 
    } 
} 
 
//------------------------------ String Parsing Methods ------------------------------ 
 
bool ParseCoordinates(string coordinates, out Vector3D parsedVector) 
{ 
    parsedVector = new Vector3D(); 
    coordinates = coordinates.Trim(); 
 
    double result; 
    string[] tokens = coordinates.Split(':'); 
 
    if (coordinates.StartsWith("GPS") && tokens.Length >= 5) 
    { 
        if (Double.TryParse(tokens[2], out result)) 
        { 
            parsedVector.SetDim(0, result); 
        } 
        else 
        { 
            return false; 
        } 
 
        if (Double.TryParse(tokens[3], out result)) 
        { 
            parsedVector.SetDim(1, result); 
        } 
        else 
        { 
            return false; 
        } 
 
        if (Double.TryParse(tokens[4], out result)) 
        { 
            parsedVector.SetDim(2, result); 
        } 
        else 
        { 
            return false; 
        } 
 
        return true; 
    } 
    else if (coordinates.StartsWith("[T:") && tokens.Length >= 4) 
    { 
        if (Double.TryParse(tokens[1], out result)) 
        { 
            parsedVector.SetDim(0, result); 
        } 
        else 
        { 
            return false; 
        } 
 
        if (Double.TryParse(tokens[2], out result)) 
        { 
            parsedVector.SetDim(1, result); 
        } 
        else 
        { 
            return false; 
        } 
         
        if (Double.TryParse(tokens[3].Substring(0, tokens[3].Length - 1), out result)) 
        { 
            parsedVector.SetDim(2, result); 
        } 
        else 
        { 
            return false; 
        } 
 
        return true; 
    } 
    else 
    { 
        return false; 
    } 
} 
 
//------------------------------ Command Processing Methods ------------------------------ 
 
void ProcessConfigurationCommand(string commandLine) 
{ 
    string[] keyValues = commandLine.Split(','); 
 
    for (int i = 0; i < keyValues.Length; i++) 
    { 
        string[] tokens = keyValues[i].Trim().Split(':'); 
        if (tokens.Length > 0) 
        { 
            ProcessSingleConfigCommand(tokens); 
        } 
    } 
} 
 
void ProcessSingleConfigCommand(string[] tokens) 
{ 
    string cmdToken = tokens[0].Trim(); 
    if (cmdToken.Equals("MODE") && tokens.Length >= 2) 
    { 
        int modeValue; 
        if (Int32.TryParse(tokens[1], out modeValue)) 
        { 
            missileLaunchType = modeValue; 
        } 
    } 
    else if (cmdToken.Equals("R_REM") && tokens.Length >= 2) 
    { 
        strShipRefRemoteControl = tokens[1]; 
    } 
    else if (cmdToken.Equals("R_TAR") && tokens.Length >= 2) 
    { 
        strShipRefTargetPanel = tokens[1]; 
    } 
    else if (cmdToken.Equals("R_CAM") && tokens.Length >= 2) 
    { 
        strShipRefCamera = tokens[1]; 
    } 
    else if (cmdToken.Equals("V_DVR") && tokens.Length >= 2) 
    { 
        double dvrValue; 
        if (Double.TryParse(tokens[1], out dvrValue)) 
        { 
            driftVectorReduction = dvrValue; 
        } 
    } 
    else if (cmdToken.Equals("V_LS") && tokens.Length >= 2) 
    { 
        double lsValue; 
        if (Double.TryParse(tokens[1], out lsValue)) 
        { 
            launchSeconds = lsValue; 
        } 
    } 
    else if (cmdToken.Equals("V_DRIFT") && tokens.Length >= 2) 
    { 
        bool driftValue; 
        if (bool.TryParse(tokens[1], out driftValue)) 
        { 
            boolDrift = driftValue; 
        } 
    } 
    else if (cmdToken.Equals("V_LEAD") && tokens.Length >= 2) 
    { 
        bool leadValue; 
        if (bool.TryParse(tokens[1], out leadValue)) 
        { 
            boolLeadTarget = leadValue; 
        } 
    } 
    else if (cmdToken.Equals("V_DAMP") && tokens.Length >= 2) 
    { 
        bool dampenerValue; 
        if (bool.TryParse(tokens[1], out dampenerValue)) 
        { 
            boolNaturalDampener = dampenerValue; 
        } 
    } 
    else if (cmdToken.Equals("P_VT") && tokens.Length >= 2) 
    { 
        bool vtValue; 
        if (bool.TryParse(tokens[1], out vtValue)) 
        { 
            verticalTakeoff = vtValue; 
        } 
    } 
    else if (cmdToken.Equals("P_EMR") && tokens.Length >= 2) 
    { 
        bool emrValue; 
        if (bool.TryParse(tokens[1], out emrValue)) 
        { 
            excludeGFDMissileRadius = emrValue; 
        } 
    } 
    else if (cmdToken.Equals("SPIN") && tokens.Length >= 2) 
    { 
        double spinValue; 
        if (Double.TryParse(tokens[1], out spinValue)) 
        { 
            spinAmount = (int)spinValue; 
        } 
    } 
} 
 
//------------------------------ Initialization Methods ------------------------------ 
 
bool InitLaunchingShipRefBlocks() 
{ 
    List<IMyTerminalBlock> blocks; 
    switch (missileLaunchType) 
    { 
        case 0: 
        case 2: 
            blocks = GetBlocksWithName<IMyRemoteControl>(strShipRefRemoteControl); 
 
            if (blocks.Count == 0) 
            { 
                Echo("Error: Required Remote Control with name " + strShipRefRemoteControl + " not found."); 
                return false; 
            } 
            else 
            { 
                if (blocks.Count > 1) 
                { 
                    Echo("Warning: More than one Remote Control with name " + strShipRefRemoteControl + " found. Using first one detected."); 
                } 
 
                shipRefRemoteControl = blocks[0] as IMyRemoteControl; 
                return true; 
            } 
        case 3: 
            blocks = GetBlocksWithName<IMyTerminalBlock>(strShipRefCamera); 
 
            if (blocks.Count == 0) 
            { 
                Echo("Error: Required Camera Or Aiming Block with name " + strShipRefCamera + " not found."); 
                return false; 
            } 
            else 
            { 
                if (blocks.Count > 1) 
                { 
                    Echo("Warning: More than one Camera Or Aiming Block with name " + strShipRefCamera + " found. Using first one detected."); 
                } 
 
                shipRefCamera = blocks[0]; 
                return true; 
            } 
        case 1: 
        case 4: 
            blocks = GetBlocksWithName<IMyTextPanel>(strShipRefTargetPanel); 
 
            if (blocks.Count == 0) 
            { 
                Echo("Error: Required Text Panel with name " + strShipRefTargetPanel + " not found."); 
                return false; 
            } 
            else 
            { 
                if (blocks.Count > 1) 
                { 
                    Echo("Warning: More than one Text Panel with name " + strShipRefTargetPanel + " found. Using first one detected."); 
                } 
 
                shipRefTargetPanel = blocks[0] as IMyTextPanel; 
                return true; 
            } 
        case 99: 
            return true; 
        default: 
            Echo("Error: Unknown missileLaunchType - " + missileLaunchType + "."); 
            return false; 
    } 
} 
 
bool InitMissileBlocks() 
{ 
    gyroscopes = GetGyroscopes(); 
    if (gyroscopes == null) return false; 
 
    thrusters = GetThrusters(); 
    if (thrusters == null) return false; 
     
    remoteControl = GetRemoteControl(); 
    if (remoteControl == null) return false; 
 
    if (strDirectionRefBlock != null && strDirectionRefBlock.Length > 0) 
    { 
        refForwardBlock = GridTerminalSystem.GetBlockWithName(strDirectionRefBlock); 
    } 
 
    if (spinAmount > 0) 
    { 
        boolNaturalDampener = false; 
    } 
     
    if (refForwardBlock == null || boolNaturalDampener == null || boolDrift == null || verticalTakeoff) 
    { 
        thrustValues = ComputeMaxThrustValues(thrusters); 
    } 
 
    if (refForwardBlock == null) 
    { 
        refForwardBlock = ComputeHighestThrustReference(thrusters, thrustValues); 
        refForwardReverse = true; 
    } 
     
    refWorldMatrix = refForwardBlock.WorldMatrix; 
    if (refForwardReverse) 
    { 
        refWorldMatrix = MatrixD.CreateWorld(refWorldMatrix.Translation, refWorldMatrix.Backward, refWorldMatrix.Up); 
    } 
 
    InitGyrosAndThrusters(); 
    thrustValues = null; 
 
    if (boolLeadTarget == null) 
    { 
        boolLeadTarget = true; 
    } 
 
    if (strStatusDisplayPrefix != null && strStatusDisplayPrefix.Length > 0) 
    { 
        List<IMyTerminalBlock> blocks = GetBlocksWithPrefix<IMyTerminalBlock>(strStatusDisplayPrefix); 
        if (blocks.Count > 0) 
        { 
            statusDisplay = blocks[0]; 
        } 
    } 
 
    return true; 
} 
 
void TurnOnBlocks(List<IMyTerminalBlock> blocks) 
{ 
    for (int i = 0; i < blocks.Count; i++) 
    { 
        blocks[i].ApplyAction("OnOff_On"); 
    } 
} 
 
List<IMyTerminalBlock> GetGyroscopes() 
{ 
    List<IMyTerminalBlock> blocks = GetBlocksWithPrefix<IMyGyro>(strGyroscopesPrefix); 
    if (blocks.Count > 0) 
    { 
        return blocks; 
    } 
     
    GridTerminalSystem.GetBlocksOfType<IMyGyro>(blocks); 
    if (blocks.Count > 0) 
    { 
        return blocks; 
    } 
    else 
    { 
        Echo("Error: No Gyroscopes found."); 
        return null; 
    } 
} 
 
List<IMyTerminalBlock> GetThrusters() 
{ 
    List<IMyTerminalBlock> blocks = GetBlocksWithPrefix<IMyThrust>(strThrustersPrefix); 
    if (blocks.Count > 0) 
    { 
        return blocks; 
    } 
     
    GridTerminalSystem.GetBlocksOfType<IMyThrust>(blocks); 
    if (blocks.Count > 0) 
    { 
        return blocks; 
    } 
    else 
    { 
        Echo("Error: No Thrusters found."); 
        return null; 
    } 
} 
 
IMyRemoteControl GetRemoteControl() 
{ 
    List<IMyTerminalBlock> blocks = GetBlocksOfType<IMyRemoteControl>(); 
    IMyRemoteControl remoteBlock = (blocks.Count > 0 ? blocks[0] as IMyRemoteControl : null); 
    if (remoteBlock == null) 
    { 
        Echo("Error: No Remote Control found."); 
    } 
    return remoteBlock; 
} 
 
void InitGyrosAndThrusters() 
{ 
    //---------- Find Gyroscope Orientation With Respect To Ship ---------- 
 
    gyroYawField = new string[gyroscopes.Count]; 
    gyroPitchField = new string[gyroscopes.Count]; 
    gyroYawFactor = new float[gyroscopes.Count]; 
    gyroPitchFactor = new float[gyroscopes.Count]; 
    gyroRollField = new string[gyroscopes.Count]; 
    gyroRollFactor = new float[gyroscopes.Count]; 
 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        Base6Directions.Direction gyroUp = gyroscopes[i].WorldMatrix.GetClosestDirection(refWorldMatrix.Up); 
        Base6Directions.Direction gyroLeft = gyroscopes[i].WorldMatrix.GetClosestDirection(refWorldMatrix.Left); 
        Base6Directions.Direction gyroForward = gyroscopes[i].WorldMatrix.GetClosestDirection(refWorldMatrix.Forward); 
 
        switch (gyroUp) 
        { 
            case Base6Directions.Direction.Up: 
                gyroYawField[i] = "Yaw"; 
                gyroYawFactor[i] = GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Down: 
                gyroYawField[i] = "Yaw"; 
                gyroYawFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Left: 
                gyroYawField[i] = "Pitch"; 
                gyroYawFactor[i] = GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Right: 
                gyroYawField[i] = "Pitch"; 
                gyroYawFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Forward: 
                gyroYawField[i] = "Roll"; 
                gyroYawFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Backward: 
                gyroYawField[i] = "Roll"; 
                gyroYawFactor[i] = GYRO_FACTOR; 
                break; 
        } 
 
        switch (gyroLeft) 
        { 
            case Base6Directions.Direction.Up: 
                gyroPitchField[i] = "Yaw"; 
                gyroPitchFactor[i] = GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Down: 
                gyroPitchField[i] = "Yaw"; 
                gyroPitchFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Left: 
                gyroPitchField[i] = "Pitch"; 
                gyroPitchFactor[i] = GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Right: 
                gyroPitchField[i] = "Pitch"; 
                gyroPitchFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Forward: 
                gyroPitchField[i] = "Roll"; 
                gyroPitchFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Backward: 
                gyroPitchField[i] = "Roll"; 
                gyroPitchFactor[i] = GYRO_FACTOR; 
                break; 
        } 
 
        switch (gyroForward) 
        { 
            case Base6Directions.Direction.Up: 
                gyroRollField[i] = "Yaw"; 
                gyroRollFactor[i] = GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Down: 
                gyroRollField[i] = "Yaw"; 
                gyroRollFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Left: 
                gyroRollField[i] = "Pitch"; 
                gyroRollFactor[i] = GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Right: 
                gyroRollField[i] = "Pitch"; 
                gyroRollFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Forward: 
                gyroRollField[i] = "Roll"; 
                gyroRollFactor[i] = -GYRO_FACTOR; 
                break; 
            case Base6Directions.Direction.Backward: 
                gyroRollField[i] = "Roll"; 
                gyroRollFactor[i] = GYRO_FACTOR; 
                break; 
        } 
 
        gyroscopes[i].ApplyAction("OnOff_On"); 
    } 
 
    //---------- Check Whether To Use Default PID Values ---------- 
 
    if (useDefaultPIDValues) 
    { 
        if (Me.CubeGrid.ToString().Contains("Large")) 
        { 
            AIM_P = DEF_BIG_GRID_P; 
            AIM_I = DEF_BIG_GRID_I; 
            AIM_D = DEF_BIG_GRID_D; 
        } 
        else 
        { 
            AIM_P = DEF_SMALL_GRID_P; 
            AIM_I = DEF_SMALL_GRID_I; 
            AIM_D = DEF_SMALL_GRID_D; 
            AIM_LIMIT *= 2; 
        } 
    } 
 
    //---------- Find Forward Thrusters ---------- 
     
    List<IMyTerminalBlock> checkThrusters = thrusters; 
    thrusters = new List<IMyTerminalBlock>(); 
 
    if (boolNaturalDampener == null || boolDrift == null || verticalTakeoff) 
    { 
        IMyTerminalBlock leftThruster = null; 
        IMyTerminalBlock rightThruster = null; 
        IMyTerminalBlock upThruster = null; 
        IMyTerminalBlock downThruster = null; 
         
        float leftThrustTotal = 0; 
        float rightThrustTotal = 0; 
        float upThrustTotal = 0; 
        float downThrustTotal = 0; 
 
        for (int i = 0; i < checkThrusters.Count; i++) 
        { 
            Base6Directions.Direction thrusterDirection = refWorldMatrix.GetClosestDirection(checkThrusters[i].WorldMatrix.Backward); 
            switch (thrusterDirection) 
            { 
                case Base6Directions.Direction.Forward: 
                    thrusters.Add(checkThrusters[i]); 
                    break; 
                case Base6Directions.Direction.Left: 
                    leftThruster = checkThrusters[i]; 
                    leftThrustTotal += thrustValues[i]; 
                    break; 
                case Base6Directions.Direction.Right: 
                    rightThruster = checkThrusters[i]; 
                    rightThrustTotal += thrustValues[i]; 
                    break; 
                case Base6Directions.Direction.Up: 
                    upThruster = checkThrusters[i]; 
                    upThrustTotal += thrustValues[i]; 
                    break; 
                case Base6Directions.Direction.Down: 
                    downThruster = checkThrusters[i]; 
                    downThrustTotal += thrustValues[i]; 
                    break; 
            } 
 
            checkThrusters[i].ApplyAction("OnOff_On"); 
        } 
 
        float highestThrust = Math.Max(Math.Max(Math.Max(leftThrustTotal, rightThrustTotal), upThrustTotal), downThrustTotal); 
        if (highestThrust == 0) 
        { 
            if (boolNaturalDampener == true) 
            { 
                Echo("Warning: Natural Gravity Dampener feature not possible as there are no Downward Thrusters found."); 
            } 
            boolNaturalDampener = false; 
 
            if (boolDrift == null) 
            { 
                boolDrift = true; 
            } 
        } 
        else 
        { 
            if (leftThrustTotal == highestThrust) 
            { 
                refDownwardBlock = leftThruster; 
            } 
            else if (rightThrustTotal == highestThrust) 
            { 
                refDownwardBlock = rightThruster; 
            } 
            else if (upThrustTotal == highestThrust) 
            { 
                refDownwardBlock = upThruster; 
            } 
            else 
            { 
                refDownwardBlock = downThruster; 
            } 
            boolNaturalDampener = (refDownwardBlock != null); 
 
            if (boolDrift == null) 
            { 
                float lowestThrust = Math.Min(Math.Min(Math.Min(leftThrustTotal, rightThrustTotal), upThrustTotal), downThrustTotal); 
                boolDrift = (highestThrust > lowestThrust * 2); 
            } 
        } 
 
        if (verticalTakeoff && refDownwardBlock != null) 
        { 
            launchThrusters = new List<IMyTerminalBlock>(); 
 
            for (int i = 0; i < checkThrusters.Count; i++) 
            { 
                if (refDownwardBlock.WorldMatrix.Forward.Dot(checkThrusters[i].WorldMatrix.Forward) >= 0.9) 
                { 
                    launchThrusters.Add(checkThrusters[i]); 
                } 
            } 
        } 
    } 
    else 
    { 
        for (int i = 0; i < checkThrusters.Count; i++) 
        { 
            Base6Directions.Direction thrusterDirection = refWorldMatrix.GetClosestDirection(checkThrusters[i].WorldMatrix.Backward); 
            if (thrusterDirection == Base6Directions.Direction.Forward) 
            { 
                thrusters.Add(checkThrusters[i]); 
            } 
            else if (boolNaturalDampener == true && thrusterDirection == Base6Directions.Direction.Up) 
            { 
                refDownwardBlock = checkThrusters[i]; 
            } 
 
            checkThrusters[i].ApplyAction("OnOff_On"); 
        } 
 
        if (boolNaturalDampener == true && refDownwardBlock == null) 
        { 
            Echo("Warning: Natural Gravity Dampener feature not possible as there are no Downward Thrusters found."); 
            boolNaturalDampener = false; 
        } 
    } 
} 
 
float[] ComputeMaxThrustValues(List<IMyTerminalBlock> checkThrusters) 
{ 
    float[] thrustValues = new float[checkThrusters.Count]; 
     
    for (int i = 0; i < checkThrusters.Count; i++) 
    { 
        float prevThrustOverride = checkThrusters[i].GetValueFloat("Override"); 
        checkThrusters[i].SetValueFloat("Override", checkThrusters[i].GetMaximum<float>("Override")); 
        thrustValues[i] = ((IMyThrust)checkThrusters[i]).ThrustOverride; 
        checkThrusters[i].SetValueFloat("Override", prevThrustOverride); 
    } 
 
    return thrustValues; 
} 
 
IMyTerminalBlock ComputeHighestThrustReference(List<IMyTerminalBlock> checkThrusters, float[] thrustValues) 
{ 
    IMyTerminalBlock fwdThruster = null; 
    IMyTerminalBlock bwdThruster = null; 
    IMyTerminalBlock leftThruster = null; 
    IMyTerminalBlock rightThruster = null; 
    IMyTerminalBlock upThruster = null; 
    IMyTerminalBlock downThruster = null; 
     
    float fwdThrustTotal = 0; 
    float bwdThrustTotal = 0; 
    float leftThrustTotal = 0; 
    float rightThrustTotal = 0; 
    float upThrustTotal = 0; 
    float downThrustTotal = 0; 
     
    for (int i = 0; i < checkThrusters.Count; i++) 
    { 
        Base6Directions.Direction thrusterDirection = Me.WorldMatrix.GetClosestDirection(checkThrusters[i].WorldMatrix.Backward); 
        switch (thrusterDirection) 
        { 
            case Base6Directions.Direction.Forward: 
                fwdThruster = checkThrusters[i]; 
                fwdThrustTotal += thrustValues[i]; 
                break; 
            case Base6Directions.Direction.Backward: 
                bwdThruster = checkThrusters[i]; 
                bwdThrustTotal += thrustValues[i]; 
                break; 
            case Base6Directions.Direction.Left: 
                leftThruster = checkThrusters[i]; 
                leftThrustTotal += thrustValues[i]; 
                break; 
            case Base6Directions.Direction.Right: 
                rightThruster = checkThrusters[i]; 
                rightThrustTotal += thrustValues[i]; 
                break; 
            case Base6Directions.Direction.Up: 
                upThruster = checkThrusters[i]; 
                upThrustTotal += thrustValues[i]; 
                break; 
            case Base6Directions.Direction.Down: 
                downThruster = checkThrusters[i]; 
                downThrustTotal += thrustValues[i]; 
                break; 
        } 
    } 
 
    List<IMyTerminalBlock> highestThrustReferences = new List<IMyTerminalBlock>(2); 
 
    float highestThrust = Math.Max(Math.Max(Math.Max(Math.Max(Math.Max(fwdThrustTotal, bwdThrustTotal), leftThrustTotal), rightThrustTotal), upThrustTotal), downThrustTotal); 
    if (fwdThrustTotal == highestThrust) 
    { 
        highestThrustReferences.Add(fwdThruster); 
    } 
    if (bwdThrustTotal == highestThrust) 
    { 
        highestThrustReferences.Add(bwdThruster); 
    } 
    if (leftThrustTotal == highestThrust) 
    { 
        highestThrustReferences.Add(leftThruster); 
    } 
    if (rightThrustTotal == highestThrust) 
    { 
        highestThrustReferences.Add(rightThruster); 
    } 
    if (upThrustTotal == highestThrust) 
    { 
        highestThrustReferences.Add(upThruster); 
    } 
    if (downThrustTotal == highestThrust) 
    { 
        highestThrustReferences.Add(downThruster); 
    } 
 
    if (highestThrustReferences.Count == 1) 
    { 
        return highestThrustReferences[0]; 
    } 
    else 
    { 
        Vector3D diagonalVector = ComputeMissileDiagonalVector(); 
 
        IMyTerminalBlock closestToLengthRef = highestThrustReferences[0]; 
        double closestToLengthValue = 0; 
 
        for (int i = 0; i < highestThrustReferences.Count; i++) 
        { 
            double dotLength = Math.Abs(diagonalVector.Dot(highestThrustReferences[i].WorldMatrix.Forward)); 
            if (dotLength > closestToLengthValue) 
            { 
                closestToLengthValue = dotLength; 
                closestToLengthRef = highestThrustReferences[i]; 
            } 
        } 
 
        return closestToLengthRef; 
    } 
} 
 
Vector3D ComputeMissileDiagonalVector() 
{ 
    IMyCubeGrid cubeGrid = Me.CubeGrid; 
 
    Vector3D minVector = cubeGrid.GridIntegerToWorld(cubeGrid.Min); 
    Vector3D maxVector = cubeGrid.GridIntegerToWorld(cubeGrid.Max); 
 
    return (minVector - maxVector); 
} 
 
//------------------------------ Thruster Control Methods ------------------------------ 
 
void FireThrusters(List<IMyTerminalBlock> listThrusters, bool overrideMode) 
{ 
    if (listThrusters != null) 
    { 
        for (int i = 0; i < listThrusters.Count; i++) 
        { 
            listThrusters[i].SetValue<float>("Override", (overrideMode ? listThrusters[i].GetMaximum<float>("Override") : 0f)); 
        } 
    } 
} 
 
//------------------------------ Gyroscope Control Methods ------------------------------ 
 
void SetGyroOverride(bool bOverride) 
{ 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        if (((IMyGyro)gyroscopes[i]).GyroOverride != bOverride) 
        { 
            gyroscopes[i].ApplyAction("Override"); 
        } 
    } 
} 
 
void SetGyroYaw(double yawRate) 
{ 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        gyroscopes[i].SetValueFloat(gyroYawField[i], (float)yawRate * gyroYawFactor[i]); 
    } 
} 
 
void SetGyroPitch(double pitchRate) 
{ 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        gyroscopes[i].SetValueFloat(gyroPitchField[i], (float)pitchRate * gyroPitchFactor[i]); 
    } 
} 
 
void SetGyroRoll(double rollRate) 
{ 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        gyroscopes[i].SetValueFloat(gyroRollField[i], (float)rollRate * gyroRollFactor[i]); 
    } 
} 
 
void ZeroTurnGyro() 
{ 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        gyroscopes[i].SetValueFloat(gyroYawField[i], 0); 
        gyroscopes[i].SetValueFloat(gyroPitchField[i], 0); 
    } 
} 
 
void ResetGyro() 
{ 
    for (int i = 0; i < gyroscopes.Count; i++) 
    { 
        gyroscopes[i].SetValueFloat("Yaw", 0); 
        gyroscopes[i].SetValueFloat("Pitch", 0); 
        gyroscopes[i].SetValueFloat("Roll", 0); 
    } 
} 
 
//------------------------------ Name Finder API ------------------------------ 
 
List<IMyTerminalBlock> GetBlocksOfType<T>() where T: IMyTerminalBlock 
{ 
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(); 
    GridTerminalSystem.GetBlocksOfType<T>(blocks); 
 
    return blocks; 
} 
 
List<IMyTerminalBlock> GetBlocksWithName(string name) 
{ 
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(); 
    GridTerminalSystem.SearchBlocksOfName(name, blocks); 
 
    return blocks; 
} 
 
List<IMyTerminalBlock> GetBlocksWithName<T>(string name) where T: IMyTerminalBlock 
{ 
    nameMatcher = name; 
 
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(); 
    GridTerminalSystem.GetBlocksOfType<T>(blocks, MatchNameExact); 
 
    return blocks; 
} 
 
List<IMyTerminalBlock> GetBlocksWithPrefix<T>(string name) where T: IMyTerminalBlock 
{ 
    nameMatcher = name; 
 
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(); 
    GridTerminalSystem.GetBlocksOfType<T>(blocks, MatchNamePrefix); 
 
    return blocks; 
} 
 
List<IMyTerminalBlock> GetBlocksWithSuffix<T>(string name) where T: IMyTerminalBlock 
{ 
    nameMatcher = name; 
 
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(); 
    GridTerminalSystem.GetBlocksOfType<T>(blocks, MatchNameSuffix); 
 
    return blocks; 
} 
 
List<IMyTerminalBlock> GetBlocksContaining<T>(string name) where T: IMyTerminalBlock 
{ 
    nameMatcher = name; 
 
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>(); 
    GridTerminalSystem.GetBlocksOfType<T>(blocks, MatchNameContaining); 
 
    return blocks; 
} 
 
bool MatchNameExact(IMyTerminalBlock block) 
{ 
    return block.CustomName.Equals(nameMatcher); 
} 
 
bool MatchNamePrefix(IMyTerminalBlock block) 
{ 
    return block.CustomName.StartsWith(nameMatcher); 
} 
 
bool MatchNameSuffix(IMyTerminalBlock block) 
{ 
    return block.CustomName.EndsWith(nameMatcher); 
} 
 
bool MatchNameContaining(IMyTerminalBlock block) 
{ 
    return block.CustomName.Contains(nameMatcher); 
}
