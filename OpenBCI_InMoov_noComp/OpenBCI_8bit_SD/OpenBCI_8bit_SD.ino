/*
 * 
 *  >>>> THIS CODE DESIGNED FOR OBCI_8bit board <<<<
 *
 * This code is written to target an ATmega328P with UNO bootloader. 
 * Adjust as needed if you are testing on different hardware.
 *
 *
 * Made by Joel Murphy, Luke Travis, Conor Russomanno Summer, 2014. 
 * SDcard code is based on RawWrite example in SDFat library 
 * ASCII commands are received on the serial port to configure and control
 * Serial protocol uses '+' immediately before and after the command character
 * We call this the 'burger' protocol. the '+' re the buns. Example:
 * To begin streaming data, this code needs to see '+b+' on the serial port.
 * The included Dongle with OpenBCI_8bit_Host code is designed to insert the '+' characters.
 * Any PC or mobile device should send the command characters at 200Hz max. 
 * OpenBCI_8bit_Host will do the rest. You're welcome.
 *
 * This software is provided as-is with no promise of work ability
 * Use at your own risk.
 *
 */ 
 
#include <EEPROM.h>
#include <SPI.h>
//#include <SdFat.h>   // not using SD. could be an option later
//#include <SdFatUtil.h>
#include "OpenBCI_8.h"  
#include <Filters.h>
#include <StandardCplusplus.h>  //for vector & deques
#include <deque>
#include <Servo.h>
 
 
boolean isVerbose;  // if true, you'll get a more verbose Serial Monitor print out...
 
//------------------------------------------------------------------------------
//  << SD CARD BUSINESS >> has been taken out. See OBCI_SD_LOG_CMRR 
//  SD_SS on pin 7 defined in OpenBCI library
boolean use_SD = false;
char fileSize = '0';  // SD file size indicator
//------------------------------------------------------------------------------
//  << OpenBCI BUSINESS >>
boolean is_running = false;    // this flag is set in serialEvent on reciept of ascii prompt
OpenBCI OBCI; //Uses SPI bus and pins to say data is ready. 
byte sampleCounter = 0;
// these are used to change individual channel settings from PC
char currentChannel;    // keep track of what channel we're loading settings for
boolean getChannelSettings = false; // used to receive channel settings command
int channelSettingsCounter; // used to retrieve channel settings from serial port
int leadOffSettingsCounter;
boolean getLeadOffSettings = false;
 
// these are all subject to the radio requirements: 31byte max packet length (maxPacketLength - 1 for packet checkSum)
#define OUTPUT_NOTHING (0)  // quiet
#define OUTPUT_BINARY (1)  // normal transfer mode
#define OUTPUT_BINARY_SYNTHETIC (2)  // needs portage
int outputType;
 
RunningStatistics stats;
 
//------------------------------------------------------------------------------
//  << LIS3DH Accelerometer Business >>
//  LIS3DH_SS on pin 5 defined in OpenBCI library
volatile boolean auxAvailable = false;
boolean useAccelOnly = false;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//Scaling Factor Code
float fs_Hz = 250.0f;  //sample rate used by OpenBCI board...set by its Arduino code
float ADS1299_Vref = 4.5f;  //reference voltage for ADC in ADS1299.  set by its hardware
float ADS1299_gain = 24.0;  //assumed gain setting for ADS1299.  set by its Arduino code
float scale_fac_uVolts_per_count = ADS1299_Vref / ((float)(pow(2,23)-1)) / ADS1299_gain  * 1000000.f; //ADS1299 datasheet Table 7, confirmed through experiment


float tempValue;
float value1;
float value1F; //filtered
float value1F2;
FilterTwoPole highPassFilter;
FilterTwoPole lowPassFilter;

//
const int channelCount = 8;
std::deque<float> filterData[channelCount];
int ourChan = 0;

float upperThreshold = 0;
float lowerThreshold = 0;
int averagePeriod = 25;
int thresholdPeriod = 1250;
float myAverage = 0.0;  
//float acceptableLimitUV = 255;
float InMoov_output_normalized = 0.0; 
int InMoov_output_normalized_int;

Servo servos[5];
int numFingers = 5;

int openThreshold[] = {
  30, //thumb
  30, //pointer
  5, //middle
  10, //ring
  30 //pinky
};
int closedThreshold[] = {
  150, //thumb
  150, //pointer
  150, //middle
  150, //ring
  150 //pinky
};

int pos[5]; //array of current finger positions ... this gets written to the hand
float normalizedPos;


//------------------------------------------------------------------------------
 
void setup(void) {
  
  isVerbose = true;
  
  Serial.begin(115200);
  
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  delay(1000);
  Serial.print(F("OpenBCI V3 8bit Board\nSetting ADS1299 Channel Values\n"));
  OBCI.useAccel = true;
  OBCI.initialize();  // configures channel settings on the ADS and idles the Accel
//setup the electrode impedance detection parameters. math on the PC side calculates Z of electrode/skin
  OBCI.configure_Zdetect(LOFF_MAG_6NA, LOFF_FREQ_31p2HZ);   
  Serial.print(F("ADS1299 Device ID: 0x")); Serial.println(OBCI.getADS_ID(),HEX);
  Serial.print(F("LIS3DH Device ID: 0x")); Serial.println(OBCI.getAccelID(),HEX);
  //Serial.print(F("Free RAM: ")); Serial.println(FreeRam()); // how much RAM?
  sendEOT();
  
  
  //filters for cleaning up the raw data
  highPassFilter = FilterTwoPole(20.0, 1, 0);  //float frequency0, float qualityFactor, float xInit
  lowPassFilter = FilterTwoPole();
  
  //assign the servo finger outputs to A1-A5 of OpenBCI board GPIO pins
  servos[0].attach(A1);
  servos[1].attach(A2);
  servos[2].attach(A3);
  servos[3].attach(A4);
  servos[4].attach(A5);
  
  Serial.println();
  Serial.print("Scale Factor: ");
  Serial.println(scale_fac_uVolts_per_count, 9);
  
  
}
 
 
 
void loop() {
    
  if(is_running){
    
      while(!(OBCI.isDataAvailable())){}   // watch the DRDY pin
 
      OBCI.updateChannelData(); // retrieve the ADS channel data 8x3 bytes
      if(OBCI.LIS3DH_DataAvailable()){
        OBCI.updateAccelData();    // fresh axis data goes into the X Y Z 
        auxAvailable = true;    // pass the dataReady to SDCard, if present
      }
      if(use_SD){  
      //writeDataToSDcard(sampleCounter);   // send the new data to SD card
      }
      //OBCI.sendChannelData(sampleCounter);  // send the new data over radio


      value1 = OBCI.ads.channelData[ourChan] * scale_fac_uVolts_per_count; //apply uV scale factor
      value1F = highPassFilter.input(value1);
      
      tempValue = abs(value1); //RMS
      value1 = tempValue;
//      Serial.print("Channel Data * Scale Factor: "); 
//      if(value1 <= 25000){
//        Serial.println(int(value1));
//      }
      
      
//      value1F = highPassFilter.input(value1);
//      
//      value1F = sqrt((value1F)*(value1F)); //RMS      
//      
      if(filterData[ourChan].size() >= 50){ //make sure filterData[ourChan] doesn't grow bigger than 50 packets
        filterData[ourChan].pop_front();
      }
      filterData[ourChan].push_back(value1F); // add value to deque 
//       
//      //HAND SHIT
      if(filterData[ourChan].size() >= averagePeriod){ //make sure there are enough packets to average
        for(int i = filterData[ourChan].size() - averagePeriod; i < filterData[ourChan].size(); i++){
          myAverage += abs(filterData[ourChan][i]);  // abs(data_forDisplay_uV[ourChan][i]);
        }   
        myAverage = myAverage / float(averagePeriod); //finishing the average
      }
      
//      Serial.print("myAverage: "); 
      Serial.println(int(myAverage));
//      
//      if(myAverage >= upperThreshold){
//        upperThreshold = myAverage; 
//      }
//      
//      if(myAverage <= lowerThreshold){
//         lowerThreshold = myAverage; 
//      }
//      
//      if(millis()%100 == 0){
//        upperThreshold -= 25;
//        lowerThreshold += 25;
//      }
//      
//      InMoov_output_normalized = map(myAverage, lowerThreshold, upperThreshold, 0, 1);
//      InMoov_output_normalized_int = int(InMoov_output_normalized * 100);
//      Serial.println(InMoov_output_normalized_int);
      
      
      
      
//      setAllFingers(InMoov_output_normalized);
      
      
//      for(int i = data_forDisplay_uV[ourChan].length - averagePeriod; i < data_forDisplay_uV[ourChan].length; i++){
//         if(data_forDisplay_uV[ourChan][i] <= acceptableLimitUV){ //prevent BIG spikes from effecting the average
//           myAverage += abs(data_forDisplay_uV[ourChan][i]);
//         }
//      }
//  
//      myAverage = myAverage / float(averagePeriod); //finishing the average
//      
//      if(myAverage >= upperThreshold && myAverage <= acceptableLimitUV){ // 
//         upperThreshold = myAverage; 
//      }
//      
//      if(myAverage <= lowerThreshold){
//         lowerThreshold = myAverage; 
//      }
//      
//      if(upperThreshold >= 25){
//        upperThreshold -= (upperThreshold - 25)/(frameRate * 5); //have upper threshold creep downwards to keep range tight
//      }
//      
//      if(lowerThreshold <= 15){
//        lowerThreshold += (15 - lowerThreshold)/(frameRate * 5); //have lower threshold creep upwards to keep range tight
//      }
//      
//      println("inMoov_output: | " + inMoov_output + " |");
//      inMoov_serial.write((char)inMoov_output);    
      
      
    sampleCounter++;    // get ready for next time
  }
 
} // end of loop
 
 
 
// some variables to help find 'burger protocol' commands. don't laugh.
int plusCounter = 0;
char testChar;
unsigned long commandTimer;
 
void serialEvent(){
  while(Serial.available()){      
    char inChar = (char)Serial.read();  // take a gander at that!
    
    if(plusCounter == 1){  // if we have received the first 'bun'
      testChar = inChar;   // this might be the 'patty'
      plusCounter++;       // get ready to look for another 'bun'
      commandTimer = millis();  // don't wait too long! and don't laugh!
    }
  
    if(inChar == '+'){  // if we see a 'bun' on the serial
      plusCounter++;    // make a note of it
      if(plusCounter == 3){  // looks like we got a command character
        if(millis() - commandTimer < 5){  // if it's not too late,
          if(getChannelSettings){ // if we just got an 'x', expect channel setting parameters
            loadChannelSettings(testChar);  // go get channel settings prameters
          }else if(getLeadOffSettings){  // if we just got a 'z', expect impedeance detect parameters
            loadLeadOffSettings(testChar); // go get lead off settings parameters
          }else{
            getCommand(testChar);    // decode the command
          }
        }
        plusCounter = 0;  // get ready for the next one, whatever it is
      }
    }
  }
}
    
    
void getCommand(char token){
    switch (token){
// TURN CHANNELS ON/OFF COMMANDS
      case '1':
        changeChannelState_maintainRunningState(1,DEACTIVATE); break;
      case '2':
        changeChannelState_maintainRunningState(2,DEACTIVATE); break;
      case '3':
        changeChannelState_maintainRunningState(3,DEACTIVATE); break;
      case '4':
        changeChannelState_maintainRunningState(4,DEACTIVATE); break;
      case '5':
        changeChannelState_maintainRunningState(5,DEACTIVATE); break;
      case '6':
        changeChannelState_maintainRunningState(6,DEACTIVATE); break;
      case '7':
        changeChannelState_maintainRunningState(7,DEACTIVATE); break;
      case '8':
        changeChannelState_maintainRunningState(8,DEACTIVATE); break;
      case '!':
        changeChannelState_maintainRunningState(1,ACTIVATE); break;
      case '@':
        changeChannelState_maintainRunningState(2,ACTIVATE); break;
      case '#':
        changeChannelState_maintainRunningState(3,ACTIVATE); break;
      case '$':
        changeChannelState_maintainRunningState(4,ACTIVATE); break;
      case '%':
        changeChannelState_maintainRunningState(5,ACTIVATE); break;
      case '^':
        changeChannelState_maintainRunningState(6,ACTIVATE); break;
      case '&':
        changeChannelState_maintainRunningState(7,ACTIVATE); break;
      case '*':
        changeChannelState_maintainRunningState(8,ACTIVATE); break;
             
// TEST SIGNAL CONTROL COMMANDS
      case '0':
        activateAllChannelsToTestCondition(ADSINPUT_SHORTED,ADSTESTSIG_NOCHANGE,ADSTESTSIG_NOCHANGE); break;
      case '-':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW); break;
      case '=':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
      case 'p':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_DCSIG); break;
      case '[':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_SLOW); break;
      case ']':
        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST); break;
 
// SD CARD COMMANDS
      case 'A': case'S': case'F': case'G': case'H': case'J': case'K': case'L': case 'a':
        //use_SD = true; fileSize = token; setupSDcard(fileSize); 
        break;
      case 'j':
        if(use_SD){  // what are the consequenses of closing the file....?
          //closeSDfile(); use_SD = false;
        }
        break;
 
// CHANNEL SETTING COMMANDS
      case 'x':  // get ready to receive new settins for a channel
        if(!is_running) {Serial.println(F("ready to accept new channel settings"));}
        channelSettingsCounter = 0;  // initialize the channelSettingsCounter
        getChannelSettings = true; break;  // tell the serialEvent that we're doing this now
      case 'X':  // update the ADS with all new channel settings
        if(!is_running) {Serial.println(F("updating channel settings"));}
        writeChannelSettingsToADS(); break;  // push the new settings to the ADS chip
      case 'd':  // reset all channel settings to default
        if(!is_running) {Serial.println(F("updating channel settings do default"));}
        setChannelsToDefaultSetting(); break;
      case 'D':  // send the coded default channel settings to the controlling program
        sendDefaultChannelSettings(); break;  
      case 'c':
        // use 8 channel mode
        break;
      case 'C':
        // use 16 channel mode
        break;
        
// LEAD OFF IMPEDANCE DETECTION COMMANDS
      case 'z':
        if(!is_running) {Serial.println(F("ready to accept new impedance detect settings"));}
        leadOffSettingsCounter = 0;  // reset counter
        getLeadOffSettings = true;  // tell the serialEvent that we're doing this now 
        break;
      case 'Z':
        if(!is_running) {Serial.println(F("updating impedance detect settings"));}
        changeChannelLeadOffDetect_maintainRunningState();
        break;
 
// STREAM DATA COMMANDS
      case 'n':  
        startRunning(OUTPUT_BINARY);
        break;
      case 'b':
        //if(use_SD) stampSD(ACTIVATE);
        OBCI.enable_accel(RATE_25HZ);      // fire up the accelerometer
        startRunning(OUTPUT_BINARY);       // turn on the fire hose
        break;
     case 's':
        stopRunning();
        //if(use_SD) stampSD(DEACTIVATE);  // mark the SD log with millis() if it's logging
        break;
     case 'v':
       // something cool here
       break;
// QUERY THE ADS REGISTERS
     case '?':
       printRegisters();
       break;
       
// OTHER COMMANDS
     case '/':
//       something cool here
        stats = RunningStatistics();
        stats.setWindowSecs(1.0/100);
        //testRunningStatistics();
       break;
     default:
       break;
     }
  }// end of getCommand
  
void sendEOT(){
  Serial.print(F("$$$"));
}
 
void loadChannelSettings(char c){
//  char SRB_1;
  if(channelSettingsCounter == 0){  // if it's the first byte in this channel's array, this byte is the channel number to set
    currentChannel = c - '1'; // we just got the channel to load settings into (shift number down for array usage)
    if(!is_running) Serial.print(F("loading settings for channel ")); Serial.println(currentChannel+1,DEC);
    channelSettingsCounter++;
    return;
  }
//  setting bytes are in order: POWER_DOWN, GAIN_SET, INPUT_TYPE_SET, BIAS_SET, SRB2_SET, SRB1_SET
  if(!is_running) {
    Serial.print(F("load setting ")); Serial.print(channelSettingsCounter-1);
    Serial.print(F(" with ")); Serial.println(c);
  }
  c -= '0';
  if(channelSettingsCounter-1 == GAIN_SET){ c <<= 4; }  // shift the gain value to it's bit position
  OBCI.ADSchannelSettings[currentChannel][channelSettingsCounter-1] = c;  // assign the new value to currentChannel array
  if(channelSettingsCounter-1 == SRB1_SET){
    for(int i=0; i<8; i++){
      OBCI.ADSchannelSettings[i][SRB1_SET] = c;
    }
  }
  channelSettingsCounter++;
  if(channelSettingsCounter == 7){  // 1 currentChannel, plus 6 channelSetting parameters
    if(!is_running) Serial.print(F("done receiving settings for channel "));Serial.println(currentChannel+1,DEC);
    getChannelSettings = false;
  }
}
 
void writeChannelSettingsToADS(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType; 
  stopRunning();                   //must stop running to change channel settings
  
  OBCI.updateADSchannelSettings();    // change the channel settings
  
  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}
 
void setChannelsToDefaultSetting(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType; 
  stopRunning();  //must stop running to change channel settings
  OBCI.setChannelsToDefault();   // default channel settings
  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}
 
void loadLeadOffSettings(char c){
   if(leadOffSettingsCounter == 0){  // if it's the first byte in this channel's array, this byte is the channel number to set
    currentChannel = c - '1'; // we just got the channel to load settings into (shift number down for array usage)
    if(!is_running) Serial.print(F("changing LeadOff settings for channel ")); Serial.println(currentChannel+1,DEC);
    leadOffSettingsCounter++;
    return;
  }
//  setting bytes are in order: PCHAN, NCHAN
  if(!is_running) {
    Serial.print(F("load setting ")); Serial.print(leadOffSettingsCounter-1);
    Serial.print(F(" with ")); Serial.println(c);
  }
  c -= '0';
  OBCI.ADSleadOffSettings[currentChannel][leadOffSettingsCounter-1] = c;
  leadOffSettingsCounter++;
  if(leadOffSettingsCounter == 3){  // 1 currentChannel, plus 2 leadOff setting parameters
    if(!is_running) Serial.print(F("done receiving leadOff settings for channel "));Serial.println(currentChannel+1,DEC);
    getLeadOffSettings = false;
  }
}
 
boolean stopRunning(void) {
  if(is_running == true){
    OBCI.stopStreaming();                    // stop the data acquisition  //
    is_running = false;
    }
    return is_running;
  }
 
boolean startRunning(int OUT_TYPE) {
  if(is_running == false){
    outputType = OUT_TYPE;
    OBCI.startStreaming();    
    is_running = true;
  }
    return is_running;
}
 
int changeChannelState_maintainRunningState(int chan, int start)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    Serial.print(F("Activating channel "));
    Serial.println(chan);
    OBCI.activateChannel(chan);
  } else {
    Serial.print(F("Deactivating channel "));
    Serial.println(chan);
    OBCI.deactivateChannel(chan);
  }
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}
 
// CALLED WHEN COMMAND CHARACTER IS SEEN ON THE SERIAL PORT
int activateAllChannelsToTestCondition(int testInputCode, byte amplitudeCode, byte freqCode)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  //set the test signal to the desired state
  OBCI.configureInternalTestSignal(amplitudeCode,freqCode);    
  //loop over all channels to change their state
  for (int Ichan=1; Ichan <= 8; Ichan++) {
    OBCI.ADSchannelSettings[Ichan-1][INPUT_TYPE_SET] = testInputCode;
//    OBCI.activateChannel(Ichan,gainCode,testInputCode,false);  //Ichan must be [1 8]...it does not start counting from zero
  }
  OBCI.updateADSchannelSettings();
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}
 
int changeChannelLeadOffDetect_maintainRunningState()
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
 
  OBCI.changeChannelLeadOffDetect();
  
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}
 
void sendDefaultChannelSettings(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  OBCI.reportDefaultChannelSettings();  // reads CH1SET 
  sendEOT();
  delay(10);
  
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}
 
void printRegisters(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  
  //must stop running to change channel settings
  stopRunning();
  
  if(is_running == false){
    // print the ADS and LIS3DH registers
    OBCI.printAllRegisters();
  }
  sendEOT();
  delay(20);
    //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

//HAND STUFF===============
// Motion to set the servo into "virtual" 0 position: alltovirtual
void allMiddle() {   
  for(int i = 0; i < numFingers; i++){
    servos[i].write(90);
  }
}
// Motion to set the servo into "rest" position: alltorest
void allOpen() {    
  for(int i = 0; i < numFingers; i++){
    servos[i].write(openThreshold[i]);
  }  
}
// Motion to set the servo into "max" position: alltomax
void allClosed() {
  for(int i = 0; i < numFingers; i++){
    servos[i].write(closedThreshold[i]);
  }  
}

void setAllFingers(float _normalizedPos){
  for(int i = 0; i < numFingers; i++){
    pos[i] = (int)((_normalizedPos * (closedThreshold[i] - openThreshold[i])) + openThreshold[i]);  //map the normalized value onto the full range or each finger
    servos[i].write(pos[i]);
  }
}

void verbosePrint(String _string){
  if(isVerbose){
    Serial.println(_string);
  }
}
//===========================

