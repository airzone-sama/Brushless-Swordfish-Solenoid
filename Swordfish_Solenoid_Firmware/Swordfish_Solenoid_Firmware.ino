#include <Bounce2.h>
#include <EEPROM.h>

// Pin Definitions
#define PIN_SELECT_FIRE_A 14
#define PIN_SELECT_FIRE_B 16
#define PIN_BATTERY_MONITOR A3
#define PIN_TRIGGER_HALF 4
#define PIN_TRIGGER_FULL 5
#define PIN_TRIGGER_REV 6
#define PIN_CONFIG_MODE 7
#define PIN_PUSHER_FET 15
#define PIN_MOTOR_A 9
#define PIN_MOTOR_B 10
#define PIN_LED_RX 17


// Configuration Options
byte BatteryS = 4;
byte BurstSize = 2;
byte TargetDPSBurst = 10;
byte TargetDPSAuto = 10;
byte MotorSpeedFull = 50; // For full-pull
byte MotorSpeedHalf = 30; // For half-pull

// Modes
#define MODE_LOW_BATT 0
#define MODE_NORMAL 1
#define MODE_CONFIG 2
#define MODE_CONFIG_ROF_AUTO 3
#define MODE_CONFIG_ROF_BURST 4
#define MODE_CONFIG_HALF_POWER 5
#define MODE_CONFIG_FULL_POWER 6
byte SystemMode = MODE_NORMAL;


// Pusher Controls
// Pusher 3S
#define PULSE_ON_TIME_HI_3S 35
#define PULSE_ON_TIME_LO_3S 45
#define PULSE_RETRACT_TIME_3S 25
// Pusher 4S
#define PULSE_ON_TIME_HI_4S 25   //25
#define PULSE_ON_TIME_LO_4S 35   //25
#define PULSE_RETRACT_TIME_4S 25   //85
int PulseOnTime;
int PulseOnTimeHigh;
int PulseOnTimeLow;
int PulseRetractTime;
#define SOLENOID_CYCLE_IDLE 0
#define SOLENOID_CYCLE_PULSE 1
#define SOLENOID_CYCLE_RETRACT 2
#define SOLENOID_CYCLE_COOLDOWN 3
byte CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
unsigned long LastSolenoidCycleStarted = 0;


// Firing Controls
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_AUTO_LASTSHOT 3
#define FIRE_MODE_IDLE 4
byte CurrentFireMode = FIRE_MODE_SINGLE; // This is the user request based on the button state
byte ProcessingFireMode = FIRE_MODE_IDLE; // This is what will actually be fired.
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
long ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
byte TargetDPS = 10; // This is what the solenoid will operate at. 
bool RequestShot = false; // Set to true to request the firing sequence to commence
bool RequestAutoStop = false; // Set to true to stop Full Auto


// Motor Controls
#define MOTOR_SPINUP_LAG 100 // How long we give the motors before we know that have spun up.
#define MOTOR_SPINDOWN_3S 4000
#define MOTOR_SPINDOWN_4S 6000
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
#define MOTOR_MAX_SPEED 2000
int MaxMotorSpeed = MOTOR_MAX_SPEED;
int DecelerateTime = 0;
int AccelerateTime = 0;
long MotorRampUpPerMS = 0;
long MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;
#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce FireHalfTriggerBounce = Bounce();
Bounce FireFullTriggerBounce = Bounce();
Bounce RevTriggerBounce = Bounce();
Bounce ConfigModeBounce = Bounce();
Bounce ModeSelectABounce = Bounce();
Bounce ModeSelectBBounce = Bounce();


// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireFullTriggerPressed = false; // Fire Trigger is Depressed
bool FireHalfTriggerPressed = false; // Fire Trigger is Depressed
bool ConfigModePressed = false; // Config button is Depressed


// Battery Controls
#define BATTERY_3S_MIN 9.6
#define BATTERY_3S_MAX 12.6
#define BATTERY_4S_MIN 12.8
#define BATTERY_4S_MAX 16.8
#define BATTERY_CALFACTOR 0.0 // Adjustment for calibration.
float BatteryMaxVoltage;
float BatteryMinVoltage;
float BatteryCurrentVoltage = 99.0;
bool BatteryFlat = false;
int BatteryPercent = 100;


// Serial Comms
#define SERIAL_INPUT_BUFFER_MAX 25
char SerialInputBuffer[SERIAL_INPUT_BUFFER_MAX];
byte SavedMode = FIRE_MODE_SINGLE;
byte SavedBurstSize = 0;
bool HasSavedMode = false;
bool AutoFire = false;
int AutoFireMotorSpeed = 0;


// EEPROM Addresses
#define ADDR_BURST_SIZE 0
#define ADDR_DPS_BURST 1
#define ADDR_DPS_AUTO 2
#define ADDR_MOTOR_FULL 3
#define ADDR_MOTOR_HALF 4

void setup() {
  // Setial startup
  Serial.begin( 57600 ); // Debugging
  Serial1.begin( 57600 ); // Bluetooth
  Serial.println( F("Booting") );

  // Set up debouncing
  Serial.println( F("Configuring Debouncing") );  
  
  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_TRIGGER_REV );
  RevTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL );
  FireFullTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_TRIGGER_HALF, INPUT_PULLUP);
  FireHalfTriggerBounce.attach( PIN_TRIGGER_HALF );
  FireHalfTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_SELECT_FIRE_A, INPUT_PULLUP);
  ModeSelectABounce.attach( PIN_SELECT_FIRE_A );
  ModeSelectABounce.interval( DebounceWindow );
  
  pinMode(PIN_SELECT_FIRE_B, INPUT_PULLUP);
  ModeSelectBBounce.attach( PIN_SELECT_FIRE_B );
  ModeSelectBBounce.interval( DebounceWindow ); 

  pinMode(PIN_CONFIG_MODE, INPUT_PULLUP);
  ConfigModeBounce.attach( PIN_CONFIG_MODE );
  ConfigModeBounce.interval( DebounceWindow );  

  Serial.println( F("Debouncing Configured") );


  // Setup Motor Outputs
  Serial.println( F("Configuring PWM Ports") );
  pinMode( PIN_MOTOR_A, OUTPUT );
  pinMode( PIN_MOTOR_B, OUTPUT );
  digitalWrite( PIN_MOTOR_A, LOW );
  digitalWrite( PIN_MOTOR_B, LOW );
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM( 1000 );

  // Setup Pusher Outputs
  Serial.println( F("Configuring Pusher FET") );
  pinMode( PIN_PUSHER_FET, OUTPUT );
  digitalWrite( PIN_PUSHER_FET, LOW );  


  // LED
  Serial.println( F("Configuring LED") );
  pinMode( PIN_LED_RX, OUTPUT );
  digitalWrite( PIN_LED_RX, LOW );  

  Serial.println( F("Loading EEPROM") );
  LoadEEPROM();

  Serial.println( F("Detecting Battery") );
  // Setup Battery
  SetupSelectBattery();
  if( BatteryS == 3 )
  {
    Serial.println( F("Configuring for 3S Battery") );
    //PulseOnTime = PULSE_ON_TIME_3S;
    PulseRetractTime = PULSE_RETRACT_TIME_3S;
    PulseOnTimeHigh = PULSE_ON_TIME_HI_3S;
    PulseOnTimeLow = PULSE_ON_TIME_LO_3S;

    BatteryMaxVoltage = BATTERY_3S_MAX;
    BatteryMinVoltage = BATTERY_3S_MIN;

    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;
  }
  else
  {
    Serial.println( F("Configuring for 4S Battery") );
    //PulseOnTime = PULSE_ON_TIME_4S;
    PulseOnTimeHigh = PULSE_ON_TIME_HI_4S;
    PulseOnTimeLow = PULSE_ON_TIME_LO_4S;
    PulseRetractTime = PULSE_RETRACT_TIME_4S;

    BatteryMaxVoltage = BATTERY_4S_MAX;
    BatteryMinVoltage = BATTERY_4S_MIN;

    DecelerateTime = MOTOR_SPINDOWN_4S;
    AccelerateTime = MOTOR_SPINUP_4S;
  }

  SystemMode = MODE_NORMAL;
  CalculateRampRates(); 

  // Now wait until the trigger is high
  Serial.println( F("Waiting for trigger safety") );
  FireHalfTriggerBounce.update();
  while( FireHalfTriggerBounce.read() == HIGH )
  {
    delay(10);
    FireHalfTriggerBounce.update();
  }
  delay(10);

  Serial.println( F("Booted") );
}


// Detect the battery type based on averaging 1 second worth of samples.
void SetupSelectBattery()
{
  #define NUM_SAMPLES 100
  #define SAMPLE_DELAY 10

  byte CollectedSamples = 0;
  float SampleAverage = 0;
  
  for( byte c = 0; c < NUM_SAMPLES; c++)
  {
    float SensorValue = analogRead( PIN_BATTERY_MONITOR );
    CollectedSamples ++;
    SampleAverage += SensorValue;
    delay( SAMPLE_DELAY );
  }

  BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
  if( BatteryCurrentVoltage < BATTERY_4S_MIN )
  {
    BatteryS = 3;
  }
  else
  {
    BatteryS = 4;
  }
}


void LoadEEPROM()
{
  bool CorruptData = false;

  BurstSize = EEPROM.read( ADDR_BURST_SIZE );
  TargetDPSBurst = EEPROM.read( ADDR_DPS_BURST );
  TargetDPSAuto = EEPROM.read( ADDR_DPS_AUTO );
  MotorSpeedFull = EEPROM.read( ADDR_MOTOR_FULL );
  MotorSpeedHalf = EEPROM.read( ADDR_MOTOR_HALF );

  Serial.println( F("Read from EEPROM") );
  Serial.println( BurstSize );
  Serial.println( TargetDPSBurst );
  Serial.println( TargetDPSAuto );
  Serial.println( MotorSpeedFull );
  Serial.println( MotorSpeedHalf );

  if( (BurstSize < 1) || (BurstSize > 99) ) CorruptData = true;
  if( (TargetDPSBurst < 1) || (TargetDPSBurst > 99) ) CorruptData = true;
  if( (TargetDPSAuto < 1) || (TargetDPSAuto > 99) ) CorruptData = true;
  if( (MotorSpeedFull < 30) || (MotorSpeedFull > 100) ) CorruptData = true;
  if( (MotorSpeedHalf < 30) || (MotorSpeedHalf > 100) ) CorruptData = true;
  
  FireHalfTriggerBounce.update();
  int TriggerStatus = FireHalfTriggerBounce.read();
  if( (TriggerStatus == HIGH) || CorruptData )
  {
    Serial.println( F("Something wrong with EEPROM or held trigger while booting") );
    Serial.println( CorruptData );
    Serial.println( TriggerStatus == LOW );
    Serial.println( (TriggerStatus == LOW) || CorruptData );    
    BurstSize = 2;
    TargetDPSBurst = 10;
    TargetDPSAuto = 10;
    MotorSpeedFull = 50;
    MotorSpeedHalf = 30;

    EEPROM.write( ADDR_BURST_SIZE, BurstSize );
    EEPROM.write( ADDR_DPS_BURST, TargetDPSBurst );
    EEPROM.write( ADDR_DPS_AUTO, TargetDPSAuto );
    EEPROM.write( ADDR_MOTOR_FULL, MotorSpeedFull );
    EEPROM.write( ADDR_MOTOR_HALF, MotorSpeedHalf );
  }

  Serial.println( F("Initialised") );
  Serial.println( BurstSize );
  Serial.println( TargetDPSBurst );
  Serial.println( TargetDPSAuto );
  Serial.println( MotorSpeedFull );
  Serial.println( MotorSpeedHalf );
}


/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 */
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }  


  Serial.print( F("Ramp Up per MS = ") );
  Serial.println( MotorRampUpPerMS );

  Serial.print( F("Ramp Down per MS = ") );
  Serial.println( MotorRampDownPerMS );
}

// Updates the PWM Timers
void UpdatePWM( int NewSpeed )
{
  NewSpeed = (NewSpeed * 2) + 2; // Adjust for the prescalar
  OCR1A = NewSpeed;
  OCR1B = NewSpeed;
}


void loop() {
  ProcessButtons(); // Get User and Sensor input
  ProcessBatteryMonitor(); // Check battery voltage
  ProcessSystemMode(); // Find out what the system should be doing

  // Process Serial input
  if( ProcessSerialInput() )
  {
    ProcessSerialCommand();
  }
  
  ProcessRevCommand(); // Handle motor intentions
  
  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }
    
  // Process speed control  
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();

  // Process Firing Controls
  ProcessFiring();
  ProcessSolenoid();
}

bool ProcessSerialInput()
{
  bool SerialDataAvailable = false;
  if( Serial.available() != 0 )
    SerialDataAvailable = true;

  if( Serial1.available() != 0 )
    SerialDataAvailable = true;
    
  if( !SerialDataAvailable ) return false; // Ignore when there is no serial input
  
  static byte CurrentBufferPosition = 0;

  while( (Serial.available() > 0) || (Serial1.available() > 0) )
  {
    char NextByte = 0;
    if( Serial.available() != 0 )
      NextByte = Serial.read();
    else if( Serial1.available() != 0 )
      NextByte = Serial1.read();
    else
      NextByte = 0; //WTF is this happening??

    switch( NextByte )
    {
      case '#': // Starting new command
        CurrentBufferPosition = 0;
        break;
      case '$': // Ending command
        return true; // Jump out.. There's more data in the buffer, but we can read that next time around.
        break;
      case '?': // Presume help - Simulate DS
        SerialInputBuffer[0] = 'D';
        SerialInputBuffer[1] = 'S';
        return true;
        break;
      default: // Just some stuff coming through
        SerialInputBuffer[ CurrentBufferPosition ] = NextByte; // Insert into the buffer
        CurrentBufferPosition ++; // Move the place to the right
        if( CurrentBufferPosition >= SERIAL_INPUT_BUFFER_MAX ) CurrentBufferPosition = (SERIAL_INPUT_BUFFER_MAX - 1);  // Capture Overflows.
    }
  }

  return false;
}

void ProcessSerialCommand()
{
  char CommandHeader[3]; // Place the header into this buffer
  // Copy it using a lazy way
  CommandHeader[0] = SerialInputBuffer[0];
  CommandHeader[1] = SerialInputBuffer[1];
  CommandHeader[2] = 0;

  // Run Motor Full Command - FM
  if( (strcmp( CommandHeader, "FM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_FULL;
  }

  // Run Motor Half Command - HM
  if( (strcmp( CommandHeader, "HM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_HALF;
  }

  // Stop Motor Command - SM
  if( (strcmp( CommandHeader, "SM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_NONE;
  }

  // Single Fire Full Command - SF
  if( (strcmp( CommandHeader, "SF" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    CurrentFireMode = FIRE_MODE_SINGLE;
    AutoFireMotorSpeed = COMMAND_REV_FULL;
  }

  // Single Fire Half Command - SH
  if( (strcmp( CommandHeader, "SH" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    CurrentFireMode = FIRE_MODE_SINGLE;
    AutoFireMotorSpeed = COMMAND_REV_HALF;
  }

  // Burst Fire Full Command - BF
  if( (strcmp( CommandHeader, "BF" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    CurrentFireMode = FIRE_MODE_BURST;
    AutoFireMotorSpeed = COMMAND_REV_FULL;
  }  

  // Burst Fire Full Command - BH
  if( (strcmp( CommandHeader, "BH" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    CurrentFireMode = FIRE_MODE_BURST;
    AutoFireMotorSpeed = COMMAND_REV_HALF;
  }  

  // Burst Size Command - BS
  if( (strcmp( CommandHeader, "BS" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    EEPROM.write( ADDR_BURST_SIZE, BurstSize );
  }

  // Full Auto Rate Command - FR
  if( (strcmp( CommandHeader, "FR" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    TargetDPSAuto = constrain( atoi( IntValue ), 1, 99 );
    EEPROM.write( ADDR_DPS_AUTO, TargetDPSAuto );
  }
  
  // Burst Rate Command - BR
  if( (strcmp( CommandHeader, "BR" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    TargetDPSBurst = constrain( atoi( IntValue ), 1, 99 );
    EEPROM.write( ADDR_DPS_BURST, TargetDPSBurst );
  }
  
  // Full Power Command - FP
  if( (strcmp( CommandHeader, "FP" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorSpeedFull = constrain( atoi( IntValue ), 30, 100 );
    TimeLastMotorSpeedChanged = millis();
    EEPROM.write( ADDR_MOTOR_FULL, MotorSpeedFull );
  }

  // Half Power Command - HP
  if( (strcmp( CommandHeader, "HP" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorSpeedHalf = constrain( atoi( IntValue ), 30, 100 );
    TimeLastMotorSpeedChanged = millis();
    EEPROM.write( ADDR_MOTOR_HALF, MotorSpeedHalf );
  }

  // Query Device Command - QD
  if( strcmp( CommandHeader, "QD" ) == 0 )
  {
    Serial.println( F("#SF-OK$") );
    Serial1.println( F("#SF-OK$") );
  } 

  // Query Voltage Command - QV
  if( strcmp( CommandHeader, "QV" ) == 0 )
  {
    char VoltBuffer[5];
    sprintf( VoltBuffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    VoltBuffer[4] = 0;
    VoltBuffer[3] = VoltBuffer[2];
    VoltBuffer[2] = '.';
    Serial.println( VoltBuffer );
    Serial1.println( VoltBuffer );
  }   

  // Display Settings - DS
  if( strcmp( CommandHeader, "DS" ) == 0 )
  {
    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") );
    Serial.println( F("Blaster Settings:") );
    Serial1.println( F("Blaster Settings:") );
    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") );    

    Serial.print( F("Full Trigger Power = ") );
    Serial1.print( F("Full Trigger Power = ") );
    Serial.println( MotorSpeedFull );
    Serial1.println( MotorSpeedFull );    
    Serial.println( F("Change with #FP-xxx$  (xxx = 030 - 100)\n") );
    Serial1.println( F("Change with #FP-xxx$  (xxx = 030 - 100)\n") );

    Serial.print( F("Half Trigger Power = ") );
    Serial1.print( F("Half Trigger Power = ") );
    Serial.println( MotorSpeedHalf );
    Serial1.println( MotorSpeedHalf );     
    Serial.println( F("Change with #HP-xxx$  (xxx = 030 - 100)\n") );
    Serial1.println( F("Change with #HP-xxx$  (xxx = 030 - 100)\n") );

    Serial.print( F("Burst Size = ") );
    Serial1.print( F("Burst Size = ") );
    Serial.println( BurstSize );
    Serial1.println( BurstSize );    
    Serial.println( F("Change with #BS-xx$  (xx = 01 - 99)\n") );
    Serial1.println( F("Change with #BS-xx$  (xx = 01 - 99)\n") );

    Serial.print( F("ROF Burst = ") );
    Serial1.print( F("ROF Burst = ") );    
    Serial.println( TargetDPSBurst );
    Serial1.println( TargetDPSBurst );    
    Serial.println( F("Change with #BR-xx$  (xx = 01 - 99; Pusher Physical Limit Applies)\n") );
    Serial1.println( F("Change with #BR-xx$  (xx = 01 - 99; Pusher Physical Limit Applies)\n") );

    Serial.print( F("ROF Auto = ") );
    Serial1.print( F("ROF Auto = ") );    
    Serial.println( TargetDPSAuto );
    Serial1.println( TargetDPSAuto );        
    Serial.println( F("Change with #FR-xx$  (xx = 01 - 99; Pusher Physical Limit Applies)") );
    Serial1.println( F("Change with #FR-xx$  (xx = 01 - 99; Pusher Physical Limit Applies)") );
    Serial.println( F("--------------------\n") );
    Serial1.println( F("--------------------\n") );

    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") ); 
    Serial.println( F("Blaster Status:") );
    Serial1.println( F("Blaster Status:") );
    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") );

    Serial.print( F("Battery Voltage = ") );
    Serial1.print( F("Battery Voltage = ") );
    char VoltBuffer[5];
    sprintf( VoltBuffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    VoltBuffer[4] = 0;
    VoltBuffer[3] = VoltBuffer[2];
    VoltBuffer[2] = '.';
    Serial.println( VoltBuffer );
    Serial1.println( VoltBuffer );   

    Serial.print( F("Motor Ramp Up Rate = ") );
    Serial1.print( F("Motor Ramp Up Rate = ") );
    Serial.println( MotorRampUpPerMS );
    Serial1.println( MotorRampUpPerMS );  
    
    Serial.print( F("Motor Ramp Down Rate = ") );
    Serial1.print( F("Motor Ramp Down Rate = ") );
    Serial.println( MotorRampDownPerMS );
    Serial1.println( MotorRampDownPerMS );  

    Serial.print( F("Battery S = ") );
    Serial1.print( F("Battery S = ") );
    Serial.println( BatteryS );
    Serial1.println( BatteryS );

    Serial.print( F("System Mode = ") );
    Serial1.print( F("System Mode = ") );
    Serial.println( SystemMode );
    Serial1.println( SystemMode );
    
    Serial.print( F("Full Trigger State = ") );
    Serial1.print( F("Full Trigger State = ") );
    Serial.println( FireFullTriggerPressed );
    Serial1.println( FireFullTriggerPressed );

    Serial.print( F("Half Trigger State = ") );
    Serial1.print( F("Half Trigger State = ") );
    Serial.println( FireHalfTriggerPressed );
    Serial1.println( FireHalfTriggerPressed );

    Serial.print( F("Rev Trigger State = ") );
    Serial1.print( F("Rev Trigger State = ") );
    Serial.println( RevTriggerPressed );
    Serial1.println( RevTriggerPressed );

    Serial.println( F("--------------------\n") );
    Serial1.println( F("--------------------\n") );               
  } 
      
}

/*
 * Process the manual commands leading to motor reving
 * 
 * Logic:
 * If AutoRev is being performed, disconnect it when the half trigger is pulled.
 * We are looking for the following events: 
 * If the Half Trigger is pressed, Rev to Speed A
 * If the Rev Trigger is pressed, and the Half Trigger is also pressed, Rev to Speed B
 * If the Rev Trigger is pressed, but the Half Trigger is not, then ignore the command.
 * 
 */
void ProcessRevCommand()
{
  
  static bool PreviousFireHalfTriggerPressed = false; // Keep track of the human input

  if( !((SystemMode == MODE_NORMAL) || (SystemMode == MODE_CONFIG_FULL_POWER)|| (SystemMode == MODE_CONFIG_HALF_POWER)) ) // Spin the motors down when something out of the ordinary happens.
  {
     CommandRev = COMMAND_REV_NONE;
     AutoRev = false;
     return;
  }

  if( (PreviousFireHalfTriggerPressed != FireHalfTriggerPressed) && (SystemMode == MODE_NORMAL) )
  {
    // Human has taken control - disengage autopilot but only when not in config mode
    PreviousFireHalfTriggerPressed = FireHalfTriggerPressed;
    AutoRev = false;
  }

  if( !AutoRev )
  {
    if( FireHalfTriggerPressed )
    {
      if( RevTriggerPressed )
      {
          CommandRev = COMMAND_REV_FULL;
      }
      else
      {
        CommandRev = COMMAND_REV_HALF;
      }
    }
    else
    {
      CommandRev = COMMAND_REV_NONE;
    }
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

// Update the motors with the new speed
void ProcessMainMotors()
{
  static int PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    // Debugging output
    Serial.println(CurrentMotorSpeed);

    // Use this for Servo Library
    if( CurrentMotorSpeed > MOTOR_MAX_SPEED )
      UpdatePWM( MOTOR_MAX_SPEED );
    else
      UpdatePWM( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Calculate the desired motor speed
 */
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..  

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    //Serial.println( MSElapsed );
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}

// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

  Serial.print( F("New target speed = ") );
  Serial.println( TargetMotorSpeed );
}

// Process the firing request and queue up some darts to fire.
void ProcessFiring()
{
  if( !((SystemMode == MODE_NORMAL) || (SystemMode == MODE_CONFIG_ROF_BURST)|| (SystemMode == MODE_CONFIG_ROF_AUTO)) ) // Finish off the stroke unless in running or in ROF config mode
  {
    ShotsToFire = 0;
    if( ProcessingFireMode == FIRE_MODE_AUTO_LASTSHOT )
      ProcessingFireMode = FIRE_MODE_IDLE;
    return;
  }

  static unsigned long InitiatedAutoFire = 0;
  if( AutoFire )
  {
    if( InitiatedAutoFire == 0 ) // Started auto fire process. Start spinning the motors
    {
      InitiatedAutoFire = millis();
      AutoRev = true;
      CommandRev = AutoFireMotorSpeed;
      return;
    }
    if( (millis() - InitiatedAutoFire) < MOTOR_SPINUP_LAG ) // Wait for the lag
    {
      return;
    }
    RequestShot = true;
  }
  else
  {
    InitiatedAutoFire = 0;
  }
    

  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == MODE_NORMAL) ) // Don't try and push a dart into stationary flywheels..
  {
    if( ProcessingFireMode == FIRE_MODE_AUTO )
    {
      ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
      LastSolenoidCycleStarted = millis();
      ShotsToFire = 0;
      ExecuteFiring = true;
    }
    return;
  }

  // Requesting Shot while we were doing nothing special
  if( RequestShot && (ProcessingFireMode == FIRE_MODE_IDLE) )
  {
    PulseOnTime = map( BatteryPercent, 1, 100, PulseOnTimeLow, PulseOnTimeHigh );
    Serial.print( F( "PulseOnTime = " ) );
    Serial.println( PulseOnTime );
    Serial.print( F( "PulseRetractTime = " ) );
    Serial.println( PulseRetractTime );
    ProcessingFireMode = CurrentFireMode;
    switch( ProcessingFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        TargetDPS = 99; // Single fire mode is always flat out
        break;
      case FIRE_MODE_BURST:
        ShotsToFire = BurstSize; // Set the burst size
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        TargetDPS = TargetDPSBurst;
        break;        
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        TargetDPS = TargetDPSAuto;
        break;        
    }
  }
  else if( RequestAutoStop && (ProcessingFireMode == FIRE_MODE_AUTO) ) // Requesting Stop while firing in Full Auto 
  {
    ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
    LastSolenoidCycleStarted = millis();
    ExecuteFiring = true;

    if( CurrentSolenoidCyclePosition == SOLENOID_CYCLE_PULSE )
    {
      ShotsToFire = 1;
    }
    else
    {
      ShotsToFire = 0;
    }
  }
}

// Make the solenoid do the things it's supposed to do
void ProcessSolenoid()
{
  if( !ExecuteFiring ) // Just skip if there is no firing to execute
  {
    return;
  }

  // Calculate duty cycle whenever the target changes.
  static byte PrevTargetDPS = 0;
  if( PrevTargetDPS != TargetDPS )
  {
    PrevTargetDPS = TargetDPS;
    if( TargetDPS == 99 ) // Full rate
    {
      TimeBetweenShots = 0;
    }
    else
    {
      int PulseOverhead = PulseOnTime + PulseRetractTime;
      int TotalPulseOverhead = PulseOverhead * TargetDPS;
      int FreeMS = 1000 - TotalPulseOverhead;
      if( FreeMS <= 0 )
      {
        TimeBetweenShots = 0; // Pusher won't achieve this rate
      }
      else
      {
        TimeBetweenShots = FreeMS / TargetDPS;
      }
    }
  }

  // We actually have nothing to do
  if( ProcessingFireMode == FIRE_MODE_IDLE )
  {
    return; // Solenoid is idling.
  }

  // We are apparently supposed to fire 0 darts... Typically for end-of-firing scenarios
  if( (ShotsToFire == 0) && (ProcessingFireMode != FIRE_MODE_IDLE) )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    Serial.println( F("Finished shooting") );
    ExecuteFiring = false;
    if( AutoFire )
    {
      AutoFire = false;
      if( HasSavedMode )
      {
        BurstSize = SavedBurstSize;
        CurrentFireMode = SavedMode;
        RequestShot = false;
        AutoRev = false;
        CommandRev = COMMAND_REV_NONE;
        HasSavedMode = false;
      }
    }
    return;    
  }

  // Last check to ensure the motors are running before we send a dart into them
  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == MODE_NORMAL) )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    Serial.println( F("Shooting Aborted - Motors not running") );
    ExecuteFiring = false;
    return;        
  }

  // Pulse solenoid on high
  if( (millis() - LastSolenoidCycleStarted) < PulseOnTime )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_PULSE )
    {
      //Serial.println( F("Start Pulse") );
      /*
      if( (SystemMode != MODE_NORMAL) ) // Don't fire unless the system m ode is normal
      {
        ShotsToFire = 0;
        Serial.println( F("Mag Out!!") );
        return;
      }
      */
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_PULSE;
    digitalWrite( PIN_PUSHER_FET, HIGH );
    return;
  }

  // Release solenoid for retraction
  if( (millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime) )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_RETRACT )
    {
      //Serial.println( F("End Pulse") );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_RETRACT;
    digitalWrite( PIN_PUSHER_FET, LOW );
    return;      
  }  

  // Wait for the Global Cool Down... i.e. ROF adjustment
  if((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime + TimeBetweenShots))
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_COOLDOWN )
    {
      //Serial.println( F("Cooling Down") );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_COOLDOWN;
    digitalWrite( PIN_PUSHER_FET, LOW );
    return;      
  }

  // We have completed a single solenoid cycle. Return to idle, ready for the next shot.
  CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
  ShotsToFire -= 1;
  LastShot = millis();
  LastSolenoidCycleStarted = millis();
  //Serial.println( F("Bang!!") );  
}

// Keep tabs on the battery.
void ProcessBatteryMonitor()
{
  
  // Only count one in 10 run-through cycles
  static byte RunNumber = 0;
  RunNumber++;
  if( RunNumber <= 200 ) // Only read once every 300 cycles.. For performance reasons.
    return;
  RunNumber = 0;
  
  #define NUM_SAMPLES 100
  static byte CollectedSamples = 0;
  static float SampleAverage = 0;
  float SensorValue = analogRead( PIN_BATTERY_MONITOR );
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += SensorValue;
  }
  else
  {
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.6 ) // If the current voltage is 0, we are probably debugging
      {
        BatteryFlat = true;
      }
      else
      {
        BatteryFlat = false;
      }
    }
    else
    {
      BatteryFlat = false;
    } 
    BatteryPercent = map( (int)(BatteryCurrentVoltage * 10), (int)(BatteryMinVoltage * 10), (int)(BatteryMaxVoltage * 10), 1, 100 );
    //Serial.print( ("BatteryVoltage = ") );
    //Serial.println( BatteryCurrentVoltage );
    CollectedSamples = 0;
    SampleAverage = 0;
  }
}

/*
 * Process input from Buttons and Sensors.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  ConfigModeBounce.update(); // Update the pin bounce state
  ConfigModePressed = !(ConfigModeBounce.read());

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read()); 
  RequestShot = FireFullTriggerBounce.fell(); // Programatically keep track of the request for a shot
  RequestAutoStop = FireFullTriggerBounce.rose();

  FireHalfTriggerBounce.update(); // Update the pin bounce state
  FireHalfTriggerPressed = (FireHalfTriggerBounce.read()); // n.b. This switch is NC.

  // Determine the current firing mode
  ModeSelectABounce.update();
  ModeSelectBBounce.update();
  if( !AutoFire )
  {
    if( ModeSelectABounce.read() == LOW && ModeSelectBBounce.read() == HIGH && CurrentFireMode != FIRE_MODE_AUTO_LASTSHOT )
      CurrentFireMode = FIRE_MODE_AUTO;
    else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == HIGH )
      CurrentFireMode = FIRE_MODE_SINGLE;
    else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == LOW )
      CurrentFireMode = FIRE_MODE_BURST;
  }
}

// We are toggline between different system states here..
// Also handle the blasted configuration controls here... Because there's no special configuration screen
void ProcessSystemMode()
{
  static byte LastSystemMode = MODE_NORMAL;

  static unsigned long ConfigHeldTime = 0;  
  static byte LEDStatus = LOW;
  static unsigned long LEDStateChange = 0;

  
  if( BatteryFlat ) // Battery low
  {
    SystemMode = MODE_LOW_BATT;
  }
  else
  {
    // Configuration goes like this:
    // 1) Hold in Config Button for 3 sec min
    // 2) Hold in the button you want to configure
    // 3) Release the Config Button 
    // 4) Use Trigger to increase value, Rev to decrease value
    // 5) Hit Config button to go back to normal mode
    //
    // *** n.b. Don't have a magazine loaded when doing this.
    if( ConfigModeBounce.fell() && (SystemMode == MODE_NORMAL) ) // User has hit the config button in normal mode.
    {
      SystemMode = MODE_CONFIG;
      ConfigHeldTime = millis();
      LEDStateChange = ConfigHeldTime;
      LEDStatus = HIGH;
    }
    if( ConfigModeBounce.rose() && (SystemMode == MODE_CONFIG) && ((millis() - ConfigHeldTime) <= 3000) ) // Released the config button before we are in a specific config mode
    {
      SystemMode = MODE_NORMAL;
      ConfigHeldTime = 0;
      LEDStateChange = 0;
      LEDStatus = LOW;
      digitalWrite( PIN_LED_RX, LEDStatus );
    }
    if( SystemMode == MODE_CONFIG ) // Flash the Config LED in interim config mode
    {
      if( (millis() - LEDStateChange) > 500 )
      {
        LEDStateChange = millis();
        LEDStatus = !LEDStatus;
        digitalWrite( PIN_LED_RX, LEDStatus );
      }
    }
    if( (SystemMode == MODE_CONFIG) && ((millis() - ConfigHeldTime) > 3000) && ConfigModeBounce.rose() ) // User been holding the config button for more than 3 seconds, held in a button, and released the config button
    {
      if( FireFullTriggerPressed && !RevTriggerPressed ) // Full trigger is pressed - Configure Full Power
      {
        CommandRev = COMMAND_REV_FULL;
        AutoRev = true;
        TimeLastMotorSpeedChanged = millis();
        SystemMode = MODE_CONFIG_FULL_POWER;
      }
      else if( FireHalfTriggerPressed && !RevTriggerPressed ) // Half Trigger only is pressed - Configure Half Power
      {
        CommandRev = COMMAND_REV_HALF;
        AutoRev = true;
        TimeLastMotorSpeedChanged = millis();
        SystemMode = MODE_CONFIG_HALF_POWER;
      }
      else if( RevTriggerPressed && !FireHalfTriggerPressed ) // Rev trigger is pressed, but the full trigger is not - Configure ROF for Full Auto
      {
        SystemMode = MODE_CONFIG_ROF_AUTO;
        ProcessingFireMode = FIRE_MODE_AUTO;
        TargetDPS = TargetDPSAuto;
        ShotsToFire = 99999; // Set to something unreasonably high
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;      
      }
      else if( RevTriggerPressed && FireHalfTriggerPressed ) // Rev trigger and full trigger are pressed - Configure ROF for Burst
      {
        SystemMode = MODE_CONFIG_ROF_BURST;
        ProcessingFireMode = FIRE_MODE_AUTO;
        TargetDPS = TargetDPSBurst;
        ShotsToFire = 99999; // Set to something unreasonably high
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;      
      }
      else // Nothing was actually pressed, so cancel
      {
        SystemMode = MODE_NORMAL;
        ConfigHeldTime = 0;
        LEDStateChange = 0;
        LEDStatus = LOW;
        digitalWrite( PIN_LED_RX, LEDStatus );        
      }
    }
    if( SystemMode > MODE_CONFIG ) // Flash the Config LED 1 x per second in specific config mode
    {
      if( (millis() - LEDStateChange) > 1000 )
      {
        LEDStateChange = millis();
        LEDStatus = !LEDStatus;
        digitalWrite( PIN_LED_RX, LEDStatus );
      }
    }    
    if( ConfigModeBounce.fell() && (SystemMode > MODE_CONFIG) ) // User has hit the config button in specific config mode. Go back to config mode now.
    {
      SystemMode = MODE_NORMAL;
      ConfigHeldTime = 0;
      LEDStateChange = 0;
      LEDStatus = LOW;
      digitalWrite( PIN_LED_RX, LEDStatus );
      CommandRev = COMMAND_REV_NONE;
      ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
      LastSolenoidCycleStarted = millis();
      ExecuteFiring = true;
      ShotsToFire = 0;
    }
    if( (SystemMode > MODE_CONFIG) ) // We are in a specific configuration now, handle the button presses
    {
      if( FireHalfTriggerBounce.rose() ) // User hit the trigger button. Value should go up
      {
        if( SystemMode == MODE_CONFIG_FULL_POWER )
        {
          MotorSpeedFull += 5;
          if( MotorSpeedFull > 100)
            MotorSpeedFull = 30;
          TimeLastMotorSpeedChanged = millis();
          EEPROM.write( ADDR_MOTOR_FULL, MotorSpeedFull );
        }
        if( SystemMode == MODE_CONFIG_HALF_POWER )
        {
          MotorSpeedHalf += 5;
          if( MotorSpeedHalf > 100)
            MotorSpeedHalf = 30;
          TimeLastMotorSpeedChanged = millis();
          EEPROM.write( ADDR_MOTOR_HALF, MotorSpeedHalf );          
        }        
        if( SystemMode == MODE_CONFIG_ROF_AUTO )
        {
          long MaxWholeDPS = 1000 / (PulseOnTime + PulseRetractTime);
          if( TargetDPSAuto == 99 )
            TargetDPSAuto = 1;
          else if( TargetDPSAuto >= MaxWholeDPS )
            TargetDPSAuto = 99;
          else
            TargetDPSAuto ++;
          TargetDPS = TargetDPSAuto;
          EEPROM.write( ADDR_DPS_AUTO, TargetDPSAuto );
        }
        if( SystemMode == MODE_CONFIG_ROF_BURST )
        {
          long MaxWholeDPS = 1000 / (PulseOnTime + PulseRetractTime);
          if( TargetDPSBurst == 99 )
            TargetDPSBurst = 1;
          else if( TargetDPSBurst >= MaxWholeDPS )
            TargetDPSBurst = 99;
          else
            TargetDPSBurst ++;
          TargetDPS = TargetDPSBurst;
          EEPROM.write( ADDR_DPS_BURST, TargetDPSBurst );
        }        
      }
      if( RevTriggerBounce.rose() ) // User has hit the rev button. Value should go down
      {
        if( SystemMode == MODE_CONFIG_FULL_POWER )
        {
          MotorSpeedFull -= 5;
          if( MotorSpeedFull < 30)
            MotorSpeedFull = 100;          
          EEPROM.write( ADDR_MOTOR_FULL, MotorSpeedFull );
          TimeLastMotorSpeedChanged = millis();
        }
        if( SystemMode == MODE_CONFIG_HALF_POWER )
        {
          MotorSpeedHalf -= 5;
          if( MotorSpeedHalf < 30)
            MotorSpeedHalf = 100;   
          EEPROM.write( ADDR_MOTOR_HALF, MotorSpeedHalf );       
          TimeLastMotorSpeedChanged = millis(); 
        }        
        if( SystemMode == MODE_CONFIG_ROF_AUTO )
        {
          long MaxWholeDPS = 1000 / (PulseOnTime + PulseRetractTime);
          if( TargetDPSAuto == 99 )
            TargetDPSAuto = MaxWholeDPS;
          else if( TargetDPS <= 1 )
            TargetDPSAuto = 99;
          else
            TargetDPSAuto --;          
          TargetDPS = TargetDPSAuto;
          EEPROM.write( ADDR_DPS_AUTO, TargetDPSAuto );
        }
        if( SystemMode == MODE_CONFIG_ROF_BURST )
        {
          long MaxWholeDPS = 1000 / (PulseOnTime + PulseRetractTime);
          if( TargetDPSBurst == 99 )
            TargetDPSBurst = MaxWholeDPS;
          else if( TargetDPSBurst <= 1 )
            TargetDPSBurst = 99;
          else
            TargetDPSBurst --;          
          TargetDPS = TargetDPSBurst;
          EEPROM.write( ADDR_DPS_BURST, TargetDPSBurst );
        }
      }
    }
  }

  
  if( LastSystemMode != SystemMode )
  {
    Serial.print( F("New System Mode = ") );
    Serial.println( SystemMode );
    LastSystemMode = SystemMode;
  }
}
