/*
 * Project DecentralizedLV
 * Description:
 * Author:
 * Date:
 */
#define SPEEDTHR 10
#define MTR_TEMP_THR    60
#define MTR_LPM_OFFSET  10
#define BATT_TEMP_THR   35
#define BATT_LPM_OFFSET 10
#define IGNITION_UNDERVOLT 3500 //ADC value where the ignition rail is cut off due to low voltage
#define IGNITION_LOW 2900        //ADC value where the ignition rail reads as low voltage
#define IGNITION_ULTRA_LOW  2500    //ADC value where ignition remains off until voltage goes above LOW voltage (waiting for buck conv.)

#define PUMP_INIT_MODE 0
#define DRV_FAN_INIT_MODE 1

#define CAN_SNS 0x100

//#define USING_VW_DASH
//#define USING_CAMRY_DASH

//Macros to map hardware pins to the switches
#define HEADLIGHT   D0
#define HIGHBEAM    D3
#define LTURN       D4
#define RTURN       D5
#define BRAKE       D6
#define TRUNKRL     A5
#define SOLARCHG    A4
#define REV         A3
#define PUMPSEL     A2
#define IGN         A1
#define MULPURBTN   A0

SYSTEM_MODE(SEMI_AUTOMATIC);

Timer aTimer(10000, startupAnimTimer);
Timer pTimer(5000, pumpStartTimer);
Timer rTimer(3000, relayReInitTimer);

//Global Variables
uint8_t rTurn, lTurn, brake, reverse, fanPWM;   //PWM controllable lights
bool rPress, lPress, brkPress, revPress, pumpPress, headlight, last_headlight, highbeam, last_highbeam, highPress, trunkRel, solPress, pumpSelect, ignition, mButton;
bool radFan, radPump, brakeBoost, battFan, mpptOn, solChg;
bool startAnim = true;
bool startupHdl = true;

bool LPMode;    //Low-power enable flag - setting true disables the power steering pump, vacuum pump, reduces other system power
bool pumpInit;  //Flag to manually send the CAN packets for the power steering pump so it turns on from being completely cut off
bool pumpRelay;    //Flag set if pump should be on. Controls high-power relay for steering pump
bool pumpLastPress;
uint8_t pumpState = 0;

bool lastIgn = false;
bool LPMDebounce = false;

uint16_t ignitionADC;
uint8_t pumpMode, newPumpMode;   //Changes user-set pump power-saving modes
uint16_t speed;     //Vehicle speed over CAN - used to determine state of steering pump and fans
uint16_t rmsTemp;   //Temperature of RMS over CAN - used to determine states of pumps/fans
uint16_t rmsRPM;
uint16_t motorTemp;
uint16_t battTemp;
uint32_t pSwitchDebounce;
uint32_t pwmAnimationTick;

////////////////////////////
// CAN Message Formatting //
////////////////////////////

CANMessage pinStatus; //Main transmit message for switch data
// byte 0: Right Turn PWM 0-255
// byte 1: Left Turn PWM 0-255
// byte 2: Brake PWM 0-255
// byte 3: Reverse PWM 0-255
// byte 4: b0: Headlight, b1: High-Beam, b2: Trunk-Release, b3: Driver-Fan, b4: Power-Steer-Relay, b5: reverse-sense-camera
// byte 5: Power Mode: 0-Acc, 1-Ign, 2-LPAcc, 3-LPIgn
// byte 6: Pump Mode: 0-Off, 1-LPRun, 2-Run, 4-Boost
// byte 7: b0: Radiator Fan, b1: Radiator pump, b2: Brake Boost, b3: Battery Fan, b4: MPPT-On

CANMessage pumpCTL;

CANMessage inputMessage;

CANChannel can(CAN_D1_D2);

//Function Prototypes
void configurePins();
void sendPumpCAN();
void motorControl(bool initialize);
void readPins();
void parseSwitches();
void dashSpoof();
void CANReceive();
void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7);

uint8_t spoofNum = 0;
uint8_t errcode = 0;

uint8_t battPct;
uint8_t rmsFault = 1;

uint32_t loop_time = 0;
bool bmsFault = false;
bool switchFault = false;
bool dashAnim = true;

bool USING_CAMRY_DASH = true;
uint8_t trn_gear = 0;

// setup() runs once, when the device is first turned on.
void setup() {
    Serial.begin(115200);

    USING_CAMRY_DASH = EEPROM.read(0);

    can.begin(500000);      //Start CAN at 500kbps
    can.addFilter(0x101, 0x7FF);        //Allow incoming messages from ID 0x101
    can.addFilter(0x103, 0x7FF);        //Allow incoming messages from ID 0x103
    can.addFilter(0x110, 0x7FF);        //Allow incoming messages from ID 0x110
    can.addFilter(0x116, 0x7FF);        //Allow incoming messages from ID 0x110

    LPMode = 0;     //Low power mode is false by default

    speed = 0;      //Vehicle speed that comes from the dashboard
    rmsTemp = MTR_TEMP_THR; //Manually run fan unless CAN is received

    pumpMode = 0;       //LPM by default
    newPumpMode = pumpMode; //Set switch-select variable to initial value
    pumpRelay = 1;      //Turn on relay so pump can be instantly started

    pumpInit = 0;       //Flags for the power steering pump, off by default
    pumpLastPress = 0;

    configurePins();    //Set up the GPIO pins for reading the state of the switches
    readPins();     //Get the initial reading of the switches

    if(!digitalRead(A0)){   //Dash Selector
        USING_CAMRY_DASH = !USING_CAMRY_DASH;       //Switch to other dashboard
        EEPROM.write(0, USING_CAMRY_DASH);          //Save to EEPROM for read on next boot
        for(int i = 1; i <= 10; i++){                //Loop and flash headlights to indicate success
            CANSend(CAN_SNS,0,0,0,0,(i%2),0,0,0);   //CANSend with a flash every other iteration
            delay(500);
        }
    }

    last_headlight = !headlight;    //Set to inverse so the main loop does an update upon starting the car
    last_highbeam = !highbeam;      //Set to inverse so the main loop does an update upon starting the car

    //Set the lights to be off initially
    rTurn = 0;
    lTurn = 0;
    brake = 0;
    reverse = 0;

    //Turn off the pumps and fans until we get the ignition signal
    radFan = 0;
    radPump = 0;
    brakeBoost = 0;
    battFan = 0;

    mpptOn = 1;

    battPct = 10;

    fanPWM = 75;

    pwmAnimationTick = 0;
    aTimer.start();

    battTemp = 40;
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
    readPins();     //Read the state of each of the pins on the Microcontroller
    parseSwitches();    //Update flags and variables based on other factors in the system, like temperature, speed, current
    //Main calculation for the CAN message sent to the corner boards
    byte tx4 = headlight + (highbeam << 1) + (trunkRel << 2) + (solChg << 3) + (pumpRelay << 4) + (revPress << 5) + (brkPress << 6);
    byte tx5 = ignition + (LPMode << 1) + (startAnim << 2) + (startupHdl << 3);
    byte tx7 = radFan + (radPump << 1) + (brakeBoost << 2) + (battFan << 3) + (mpptOn << 4);
    CANSend(CAN_SNS, rTurn,lTurn,brake,fanPWM,tx4,tx5,pumpMode,tx7);   //Send out the main message to the corner boards
    if(pumpInit){   //pumpInit is set true for 3 seconds when the switch is turned on to initialize the power steering pump (which requires CAN)
        motorControl(pumpInit,0);   //Function that determines what to send the steering pump based on speed
        pumpState = 1;  //Pump is running
    }
    CANReceive();   //Receive any incoming messages and parse what they mean

    if(!USING_CAMRY_DASH){
        CANSend(0x280, 0x49, 0x0E, rmsRPM&255, rmsRPM>>8, 0x0E, 0x00, 0x1B, 0x0E);  //Send the code to emulate the RPM dial
        dashSpoof();    //Spoof the dash CAN messages so the error lights turn off
        analogWrite(DAC1,1850+(battPct*18.5));
        if(ignitionADC > IGNITION_UNDERVOLT){
            digitalWrite(A7, LOW);
        }
        else if(ignitionADC > IGNITION_LOW /*&& !LPMDebounce*/){
            if((millis()/500)%2 == 0){
                digitalWrite(A7, HIGH);
            }
            else{
                digitalWrite(A7, LOW);
            }
        }
        else{
            digitalWrite(A7, LOW);
        }
    }
    else{
        uint8_t lowACC = 0;
        if(ignitionADC > IGNITION_LOW && ignitionADC < IGNITION_UNDERVOLT) lowACC = 0x04;
        uint8_t powerSteerState = 0x0;
        if(pumpState == 0) powerSteerState = 0x38;
        else if(pumpState == 2){
            if((millis()/250)%2 == 0) powerSteerState = 0x38;
            else powerSteerState = 0x0;
        }
        uint8_t sysState = 0x0;
        if(bmsFault || switchFault) sysState = 0x04;
        uint8_t engineFault = 0x40;
        if(ignition && rmsFault) engineFault = 0x0;
        uint8_t otherGear = 0;
        if(revPress) otherGear = 0x10;
        else if(trn_gear == 0) otherGear = 0x08;
        CANSend(0x3B7,0,0,0,0,0,0,0,0);
        CANSend(0x3B1,0,0,0,0,0,0,0,0);
        CANSend(0x3BB,engineFault,lowACC,(motorTemp*1.59)+65,0,0,0,0,0);
        CANSend(0x394,0,powerSteerState,0,0,0,0,0,0);
        CANSend(0x32C,0,0,0,0,0,0,0,0);
        CANSend(0x378,0,0,0,0,0,0,0,0);
        CANSend(0x412,0,0,0,0,0,0,0,0);
        CANSend(0x411,0,0,0,0,0,0,0,0);
        CANSend(0x43a,1,1,1,1,0,0,0,0);
        CANSend(0x633,0x81,0,0,0,0,0,sysState,ignition?0x0D:0);
        CANSend(0x1EA,0,0,0,0,0,0,rmsRPM/200,rmsRPM%200);
        CANSend(0x3BC,0,otherGear,0,0,reverse?0:(trn_gear*16),trn_gear?0xA0:0,0,0);
        CANSend(0x620,0x10,0,0,0,headlight ? 0xF0:0xB0,dashAnim ? 0x0: 0x40,0x08,0);
        if(ignition) CANSend(0x1C4,0,0,0,0,0,1,0,0);
        
        if(headlight != last_headlight || highbeam != last_highbeam){       //This CAN id only needs to get sent if the value has changed
            CANSend(0x622,0x12,00,0xE8,(headlight << 5) + (highbeam << 6),00,00,00,00);
            last_headlight = headlight;
            last_highbeam = highbeam;
        }
    }

    //Serial.printlnf("L: %d, T: %d",loop_time,millis());
    while(millis()-loop_time < 10) delayMicroseconds(1);
    loop_time = millis();
}

//Code that sends out spoof CAN Messages for the airbag and ABS so the lights turn off
void dashSpoof(){
    //Spoof num selects the ID which is sent each time around the loop so it's not sent too frequently
    if(spoofNum == 0){  //Spoof for ABS
        //Errcode is a flag to set different error lights on the dashboard
        CANSend(0x1A0, 0x18, errcode, 0x00, 0x00, 0xfe, 0xfe, 0x00, 0xff);
        //delay(2);
        spoofNum = 1;
    }
    else if(spoofNum == 1){ //Spoof for airbag
        CANSend(0x050, 0x00, 0x80, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00);
        //delay(2);
        spoofNum = 0;
    }
}

void CANReceive(){
    while(can.receive(inputMessage)){
        if(inputMessage.id == 0x110){   //Message from Joe's computer on the RPM of the motor, gets sent to dash
            if(!USING_CAMRY_DASH) rmsRPM = (inputMessage.data[2] + (inputMessage.data[3] << 8)) << 2;
            else rmsRPM = inputMessage.data[2] + (inputMessage.data[3] << 8);
        }
        else if(inputMessage.id == 0x103){   //Message from back-left corner board if the fault flasher is on, turns on/off the dash indicator
            if(inputMessage.data[2] == 1 || inputMessage.data[3] == 1){
                switchFault = inputMessage.data[2];
                bmsFault = inputMessage.data[3];
                errcode = 0x82;     //Turn on the dash warning label
            }
            else{
                bmsFault = false;
                switchFault = false;
                errcode = 0x88;     //Turn off the dash warning label
            }
        }
        else if(inputMessage.id == 0x101){
            
            if(inputMessage.data[0] > 0 && inputMessage.data[0] <= 131 && inputMessage.data[1] > 0 && inputMessage.data[1] <= 131) trn_gear = 0;
            else if(inputMessage.data[1] > 132 && inputMessage.data[1] <= 150) trn_gear = 1;
            else if(inputMessage.data[0] > 132 && inputMessage.data[0] <= 145) trn_gear = 2;
            else if(inputMessage.data[1] > 150 && inputMessage.data[1] <= 175) trn_gear = 3;
            else if(inputMessage.data[0] > 145 && inputMessage.data[0] <= 162) trn_gear = 4;
            else if(inputMessage.data[1] > 175 && inputMessage.data[1] <= 210) trn_gear = 5;
            else trn_gear = 6;
            Serial.printlnf("S1: %d, S2: %d, G: %d",inputMessage.data[0],inputMessage.data[1],trn_gear);
        }
        else if(inputMessage.id == 0x116){
            rmsTemp = inputMessage.data[0];
            motorTemp = inputMessage.data[1];
            battTemp = inputMessage.data[5];
            battPct = inputMessage.data[6];
        }
        else if(inputMessage.id == 0x69A){
            rmsFault = inputMessage.data[2];
        }
    }
}

void CANSend(uint16_t Can_addr, byte data0, byte data1, byte data2, byte data3, byte data4, byte data5, byte data6, byte data7){
    pinStatus.id = Can_addr;
    pinStatus.len = 8;
    pinStatus.data[0] = data0;
    pinStatus.data[1] = data1;
    pinStatus.data[2] = data2;
    pinStatus.data[3] = data3;
    pinStatus.data[4] = data4;
    pinStatus.data[5] = data5;
    pinStatus.data[6] = data6;
    pinStatus.data[7] = data7;
    
    can.transmit(pinStatus);
}

//Read all of the pins on the device
void readPins(){
    headlight = digitalRead(HEADLIGHT);
    highPress = digitalRead(HIGHBEAM);
    lPress = digitalRead(LTURN);
    rPress = digitalRead(RTURN);
    brkPress = digitalRead(BRAKE);
    trunkRel = digitalRead(TRUNKRL);
    solPress = digitalRead(SOLARCHG);
    revPress = digitalRead(REV);
    pumpPress = digitalRead(PUMPSEL);
    ignitionADC = analogRead(IGN);
    mButton = digitalRead(MULPURBTN);
}

//Take the state of the switches and do stuff with it
void parseSwitches(){
    if(bmsFault){
        RGB.control(true);
        RGB.color(255,0,0);
    }
    else if(switchFault){
        RGB.control(true);
        RGB.color(255,100,0);
    }
    else{
        RGB.control(false);
    }
    if(ignitionADC > IGNITION_UNDERVOLT){
        ignition = true;   //Run a check to make sure LV battery isn't totally drained before turning on high-power items
        LPMode = false;
        //digitalWrite(A7, LOW);
    }
    else if(ignitionADC > IGNITION_LOW /*&& !LPMDebounce*/){
        ignition = true;
        //LPMode = true;
        //if((millis()/500)%2 == 0){
        //    digitalWrite(A7, HIGH);
        //}
        //else{
        //    digitalWrite(A7, LOW);
        //}
    }
    /*else if(ignitionADC > IGNITION_ULTRA_LOW){      //BIG PROBLEM - Battery voltage is below 10V - attempt to disable everything so BMS has power
        static uint32_t dbTimer;
        ignition = false;
        LPMode = true;
        digitalWrite(A7, HIGH);
        if(!LPMDebounce){
            LPMDebounce = true;
            dbTimer = millis();
        }
        if(dbTimer+10000 < millis()){
            LPMDebounce = false;
        }
    }*/
    else{
        LPMode = false;
        ignition = false;
    } 

    lTurn = 255*lPress;     //Max brightness whenever switch is pressd
    rTurn = 255*rPress;
    reverse = 255*revPress;

    if(brkPress) brake = 255;       //100% duty cycle - max brightness
    else if(headlight) brake = 80;  //~30% duty cycle - dim rear driving lights when headlight is on
    else brake = 0;

    if(lTurn || rTurn || reverse || brkPress){
        startAnim = false;
    }

    if(ignition) solChg = 0;
    else solChg = solPress;

    if(ignition){
        fanPWM =  (battTemp << 2) + 25;
        if(fanPWM < 50) fanPWM = 0;
        else if(fanPWM > 255) fanPWM = 255;
    }
    else{
        if(battTemp < 35){
            fanPWM = 0;
        }
        else fanPWM = 70;
    }

    if(LPMode == 0){
        if(ignition != lastIgn){
            lastIgn = ignition;
            startupHdl = ignition;
            aTimer.start();
        }

        if(startAnim) pwmAnimationHelper();
        
        highbeam = highPress;
        if(ignition){
            if(rmsTemp < MTR_TEMP_THR) radFan = 0;
            else radFan = 1;
            radPump = 1;
            brakeBoost = 1;
            if(battTemp < BATT_TEMP_THR) battFan = 0;
            else battFan = 1;
        }
        else{
            radFan = 0;
            radPump = 0;
            brakeBoost = 0;
        }

        if(pumpPress != pumpLastPress){
            RGB.control(true);
            if(!bmsFault && !switchFault) RGB.color(0,255,0);
            pumpLastPress = pumpPress;
            if(pumpPress){
                pumpRelay = 1;
                pumpInit = 1;   //Set init 
                pumpMode = 1;
                pTimer.start();                
            }
            else{
                newPumpMode = 0;
                pumpInit = 0;
                pTimer.start();
                pumpState =2;  //Pump is about to turn off
            }
        }
    }
    else if(LPMode == 1){
        newPumpMode = 0; //Turn off pump after 10s
        pumpInit = 0;   //Disable pump initialize flag
        highbeam = 0;   //Disable high beam, technically unnecesary
        pTimer.start();

        if(ignition){
            if(rmsTemp < MTR_TEMP_THR + MTR_LPM_OFFSET) radFan = 0;
            else radFan = 1;
            if(rmsTemp < MTR_TEMP_THR) radPump = 0;
            else radPump = 1;
            brakeBoost = 1;
            if(battTemp < BATT_TEMP_THR + BATT_LPM_OFFSET) battFan = 0;
            else battFan = 1;
        }
        else{
            radFan = 0;
            radPump = 0;
            brakeBoost = 0;
        }
    }
}

//Configure all of the pins on the microcontroller
void configurePins(){
    pinMode(D0, INPUT);
    pinMode(D3, INPUT);
    pinMode(D4, INPUT);
    pinMode(D5, INPUT);
    pinMode(D6, INPUT);
    pinMode(D7, OUTPUT);
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    pinMode(A6, OUTPUT);
    pinMode(A7, OUTPUT);
}
//Control Power Steering Motor Based on speed/mode. initialize = 1 will send CAN command regardless to get the pump running
void motorControl(bool initialize, bool stop){
    if(stop) return;
    if(!initialize){
        // Mode 0: Pump always off
        // Mode 1: Pump slow
        // Mode 2: Pump boost
        if(pumpMode != 2) return;
    }
    sendPumpCAN(initialize);  //Send CAN code if we haven't returned
}

//CAN message to send to the power steering pump, update value flag is there to prevent updating the CAN data every time (since it's constant)
void sendPumpCAN(bool updateValue){
    if(updateValue){
        if(!bmsFault && !switchFault) RGB.color(255,0,255);
        pumpCTL.id = 0x201;
        pumpCTL.len = 8;
        
        pumpCTL.data[0] = 0x49;
        pumpCTL.data[1] = 0x04;
        pumpCTL.data[2] = 0x20;
        pumpCTL.data[3] = 0x20;
        pumpCTL.data[4] = 0x02;
        pumpCTL.data[5] = 0x02;
        pumpCTL.data[6] = 0x05;
        pumpCTL.data[7] = 0x0E;
    }
    can.transmit(pumpCTL);
}

//Timer to control startup of the pump, stops sending the CAN message after 3 seconds so the pump slows down
void pumpStartTimer(){
    RGB.control(false);
    pumpInit = 0;   //Set the initialization flag so the main loop stops sending the 
    if(newPumpMode == 0 && pumpPress){  //If switch was recently turned off but back on again within 3s, stop the timer and exit to cancel
        pTimer.stopFromISR();
        pumpState = 1;
        return;
    }
    pumpMode = newPumpMode;     //If the switch ahs not been turned back on again, apply the newly selected power mode
    if(pumpMode == 0){          //If the pump is getting turned off turn off the relay
        pumpRelay = 0;
        rTimer.startFromISR();      //Start the 1 second timer so the pump relay turns back on (though motor does not start, must send CAN again)
        pumpState = 0;  //Pump is stopped
    }
    pTimer.stopFromISR();   //Stop the 3s timer so the pump mode isn't getting constantly updated
}

void pwmAnimationHelper(){
    pwmAnimationTick += 5;
    if(pwmAnimationTick < 128){
        reverse = pwmAnimationTick;
        brake = 0;
        lTurn = 0;
        rTurn = 0;
    }
    else if(pwmAnimationTick < 255){
        reverse = pwmAnimationTick;
        brake = pwmAnimationTick-128;
        lTurn = 0;
        rTurn = 0;
    }
    else if(pwmAnimationTick < 382){
        reverse = 255-(pwmAnimationTick-255);
        brake = pwmAnimationTick-128;
        lTurn = 0;
        rTurn = 0;
    }
    else if(pwmAnimationTick < 510){
        reverse = 0;
        brake = 255-(pwmAnimationTick-510);
        lTurn = pwmAnimationTick-382;
        rTurn = pwmAnimationTick-382;
    }
    else if(pwmAnimationTick < 637){
        reverse = 0;
        brake = 0;
        lTurn = pwmAnimationTick-382;
        rTurn = pwmAnimationTick-382;
    }
}

//Timer to turn back on steering pump relay after 1 second so pump is ready to go
void relayReInitTimer(){
    pumpRelay = 1;
    rTimer.stopFromISR();
}

void startupAnimTimer(){
    startAnim = 0;
    startupHdl = 0;
    dashAnim = false;
    aTimer.stopFromISR();
}