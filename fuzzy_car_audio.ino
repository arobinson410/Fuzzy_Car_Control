#include <Fuzzy.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Objects Required to Run GPS
#define mySerial Serial1
Adafruit_GPS GPS(&Serial1);

//Input for tuning potentiometer
const int analogInPin = A0;

//Input for microphone
const int micPin = A1;

//Euler's Constant
const float euler = 2.718;

//Fuzzy System
Fuzzy *fuzzy = new Fuzzy();

//Function Declarations
float GetAnalogValue();
float dBfromAnalogMic();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Car Radio Volume Tuner");
  randomSeed(analogRead(1));


  //Declare Inputs and Output
  FuzzyInput *speedometer = new FuzzyInput(1);
  FuzzyInput *microphone = new FuzzyInput(2);

  FuzzyOutput *volume = new FuzzyOutput(1);

  //Create and delegate fuzzy sets to the speedometer
  //Values in these sets are in MPH
  FuzzySet *slowest = new FuzzySet(0, 0, 35, 45);
  FuzzySet *slow = new FuzzySet(35, 45, 45, 65);
  FuzzySet *average = new FuzzySet(45, 65, 65, 70);
  FuzzySet *fast = new FuzzySet(65, 70, 70, 80);
  FuzzySet *fastest = new FuzzySet(70, 80, 100, 100);
  
  speedometer->addFuzzySet(slowest);
  speedometer->addFuzzySet(slow);
  speedometer->addFuzzySet(average);
  speedometer->addFuzzySet(fast);
  speedometer->addFuzzySet(fastest); //4% Program Storage, 0% Dynamic Memory

  //Create and delegate fuzzy sets to the microphone
  //Microphone values are from the ADC and have a DC bias of VCC/2
  //So the mic range is from 2.5V to 5V, which means the ADC values will
  //be from 512 to 1023
  //Values in these sets are in dB
  FuzzySet *silent = new FuzzySet(0, 0, 30, 40);
  FuzzySet *quiet = new FuzzySet(30, 60, 60, 80);
  FuzzySet *normal = new FuzzySet(60, 80, 80, 90);
  FuzzySet *loud = new FuzzySet(80, 90, 90, 95);
  FuzzySet *loudest = new FuzzySet(90, 95, 105, 105);

  microphone->addFuzzySet(silent);
  microphone->addFuzzySet(quiet);
  microphone->addFuzzySet(normal);
  microphone->addFuzzySet(loud);
  microphone->addFuzzySet(loudest); //6% Program Storage, 0% Dynamic Memory

  //Since the volume sets are in dB, it can be used for both the input and output.
  //There will need to be defuzzification for the output, as the output range on the radio 
  //is between 0 and 34
  volume->addFuzzySet(silent);
  volume->addFuzzySet(quiet);
  volume->addFuzzySet(normal);
  volume->addFuzzySet(loud);
  volume->addFuzzySet(loudest); //6% Program Storage, 0% Dyanmic Memory

  //Add the inputs and outputs to the fuzzy system
  fuzzy->addFuzzyInput(speedometer);
  fuzzy->addFuzzyInput(microphone);
  fuzzy->addFuzzyOutput(volume);

  //Create Fuzzy Consequence for all states of the output
  FuzzyRuleConsequent *thenVolumeSilent = new FuzzyRuleConsequent();
  thenVolumeSilent->addOutput(silent);
  FuzzyRuleConsequent *thenVolumeQuiet = new FuzzyRuleConsequent();
  thenVolumeQuiet->addOutput(quiet);
  FuzzyRuleConsequent *thenVolumeNormal = new FuzzyRuleConsequent();
  thenVolumeNormal->addOutput(normal);
  FuzzyRuleConsequent *thenVolumeLoud = new FuzzyRuleConsequent();
  thenVolumeNormal->addOutput(loud);
  FuzzyRuleConsequent *thenVolumeLoudest = new FuzzyRuleConsequent();
  thenVolumeLoudest->addOutput(loudest); //6% Program Storage, 0% Dynamic Memory

  //Declaration of Fuzzy Rules that the system will follow
  FuzzyRuleAntecedent *ifSpeedSlowestAndNoiseSilent = new FuzzyRuleAntecedent();
  ifSpeedSlowestAndNoiseSilent->joinWithAND(slowest, silent);
  FuzzyRule *rule01 = new FuzzyRule(1, ifSpeedSlowestAndNoiseSilent, thenVolumeQuiet);//
  
  FuzzyRuleAntecedent *ifSpeedSlowestAndNoiseQueit = new FuzzyRuleAntecedent();
  ifSpeedSlowestAndNoiseQueit->joinWithAND(slowest, quiet);
  FuzzyRule *rule02 = new FuzzyRule(2, ifSpeedSlowestAndNoiseQueit, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedSlowestAndNoiseNormal = new FuzzyRuleAntecedent();
  ifSpeedSlowestAndNoiseNormal->joinWithAND(slowest, normal);
  FuzzyRule *rule03 = new FuzzyRule(3, ifSpeedSlowestAndNoiseNormal, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedSlowestAndNoiseLoud = new FuzzyRuleAntecedent();
  ifSpeedSlowestAndNoiseLoud->joinWithAND(slowest, loud);
  FuzzyRule *rule04 = new FuzzyRule(4, ifSpeedSlowestAndNoiseLoud, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedSlowestAndNoiseLoudest = new FuzzyRuleAntecedent();
  ifSpeedSlowestAndNoiseLoudest->joinWithAND(slowest, loudest);
  FuzzyRule *rule05 = new FuzzyRule(5, ifSpeedSlowestAndNoiseLoudest, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedSlowAndNoiseSilent = new FuzzyRuleAntecedent();
  ifSpeedSlowAndNoiseSilent->joinWithAND(slow, silent);
  FuzzyRule *rule06 = new FuzzyRule(6, ifSpeedSlowAndNoiseSilent, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedSlowAndNoiseQuiet = new FuzzyRuleAntecedent();
  ifSpeedSlowAndNoiseQuiet->joinWithAND(slow, quiet);
  FuzzyRule *rule07 = new FuzzyRule(7, ifSpeedSlowAndNoiseQuiet, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedSlowAndNoiseNormal = new FuzzyRuleAntecedent();
  ifSpeedSlowAndNoiseNormal->joinWithAND(slow, normal);
  FuzzyRule *rule08 = new FuzzyRule(8, ifSpeedSlowAndNoiseNormal, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedSlowAndNoiseLoud = new FuzzyRuleAntecedent();
  ifSpeedSlowAndNoiseLoud->joinWithAND(slow, loud);
  FuzzyRule *rule09 = new FuzzyRule(9, ifSpeedSlowAndNoiseLoud, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedSlowAndNoiseLoudest = new FuzzyRuleAntecedent();
  ifSpeedSlowAndNoiseLoudest->joinWithAND(slow, loudest);
  FuzzyRule *rule10 = new FuzzyRule(10, ifSpeedSlowAndNoiseLoudest, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedAverageAndNoiseSilent = new FuzzyRuleAntecedent();
  ifSpeedAverageAndNoiseSilent->joinWithAND(average, silent);
  FuzzyRule *rule11 = new FuzzyRule(11, ifSpeedAverageAndNoiseSilent, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedAverageAndNoiseQuiet = new FuzzyRuleAntecedent();
  ifSpeedAverageAndNoiseQuiet->joinWithAND(average, quiet);
  FuzzyRule *rule12 = new FuzzyRule(12, ifSpeedAverageAndNoiseQuiet, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedAverageAndNoiseNormal = new FuzzyRuleAntecedent();
  ifSpeedAverageAndNoiseNormal->joinWithAND(average, normal);
  FuzzyRule *rule13 = new FuzzyRule(13, ifSpeedAverageAndNoiseNormal, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedAverageAndNoiseLoud = new FuzzyRuleAntecedent();
  ifSpeedAverageAndNoiseLoud->joinWithAND(average, loud);
  FuzzyRule *rule14 = new FuzzyRule(14, ifSpeedAverageAndNoiseLoud, thenVolumeNormal);

  FuzzyRuleAntecedent *ifSpeedAverageAndNoiseLoudest = new FuzzyRuleAntecedent();
  ifSpeedAverageAndNoiseLoudest->joinWithAND(average, loudest);
  FuzzyRule *rule15 = new FuzzyRule(15, ifSpeedAverageAndNoiseLoudest, thenVolumeNormal);

  FuzzyRuleAntecedent *ifSpeedFastAndNoiseSilent = new FuzzyRuleAntecedent();
  ifSpeedFastAndNoiseSilent->joinWithAND(fast, silent);
  FuzzyRule *rule16 = new FuzzyRule(16, ifSpeedFastAndNoiseSilent, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedFastAndNoiseQuiet = new FuzzyRuleAntecedent();
  ifSpeedFastAndNoiseQuiet->joinWithAND(fast, quiet);
  FuzzyRule *rule17 = new FuzzyRule(17, ifSpeedFastAndNoiseQuiet, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedFastAndNoiseNormal = new FuzzyRuleAntecedent();
  ifSpeedFastAndNoiseNormal->joinWithAND(fast, normal);
  FuzzyRule *rule18 = new FuzzyRule(18, ifSpeedFastAndNoiseNormal, thenVolumeNormal);

  FuzzyRuleAntecedent *ifSpeedFastAndNoiseLoud = new FuzzyRuleAntecedent();
  ifSpeedFastAndNoiseLoud->joinWithAND(fast, loud);
  FuzzyRule *rule19 = new FuzzyRule(19, ifSpeedFastAndNoiseLoud, thenVolumeLoud);

  FuzzyRuleAntecedent *ifSpeedFastAndNoiseLoudest = new FuzzyRuleAntecedent();
  ifSpeedFastAndNoiseLoudest->joinWithAND(fast, loudest);
  FuzzyRule *rule20 = new FuzzyRule(20, ifSpeedFastAndNoiseLoudest, thenVolumeLoud);

  FuzzyRuleAntecedent *ifSpeedFastestAndNoiseSilent = new FuzzyRuleAntecedent();
  ifSpeedFastestAndNoiseSilent->joinWithAND(fastest, silent);
  FuzzyRule *rule21 = new FuzzyRule(21, ifSpeedFastestAndNoiseSilent, thenVolumeQuiet);//

  FuzzyRuleAntecedent *ifSpeedFastestAndNoiseQuiet = new FuzzyRuleAntecedent();
  ifSpeedFastestAndNoiseQuiet->joinWithAND(fastest, quiet);
  FuzzyRule *rule22 = new FuzzyRule(22, ifSpeedFastestAndNoiseQuiet, thenVolumeQuiet);

  FuzzyRuleAntecedent *ifSpeedFastestAndNoiseNormal = new FuzzyRuleAntecedent();
  ifSpeedFastestAndNoiseNormal->joinWithAND(fastest, normal);
  FuzzyRule *rule23 = new FuzzyRule(23, ifSpeedFastestAndNoiseQuiet, thenVolumeNormal);

  FuzzyRuleAntecedent *ifSpeedFastestAndNoiseLoud = new FuzzyRuleAntecedent();
  ifSpeedFastestAndNoiseLoud->joinWithAND(fastest, loud);
  FuzzyRule *rule24 = new FuzzyRule(24, ifSpeedFastestAndNoiseLoud, thenVolumeLoud);

  FuzzyRuleAntecedent *ifSpeedFastestAndNoiseLoudest = new FuzzyRuleAntecedent();
  ifSpeedFastestAndNoiseLoudest->joinWithAND(fastest, loudest);
  FuzzyRule *rule25 = new FuzzyRule(25, ifSpeedFastestAndNoiseLoudest, thenVolumeLoudest); //9% Program Memory, 0% Dynamic Memory

  //Add Rules to the Fuzzy System
  fuzzy->addFuzzyRule(rule01); //12% Program Memory, 1% Dynamic Memory
  fuzzy->addFuzzyRule(rule02);
  fuzzy->addFuzzyRule(rule03);
  fuzzy->addFuzzyRule(rule04);
  fuzzy->addFuzzyRule(rule05);
  fuzzy->addFuzzyRule(rule06);
  fuzzy->addFuzzyRule(rule07);
  fuzzy->addFuzzyRule(rule08);
  fuzzy->addFuzzyRule(rule09);
  fuzzy->addFuzzyRule(rule10);
  fuzzy->addFuzzyRule(rule11);
  fuzzy->addFuzzyRule(rule12);
  fuzzy->addFuzzyRule(rule13);
  fuzzy->addFuzzyRule(rule14);
  fuzzy->addFuzzyRule(rule15);
  fuzzy->addFuzzyRule(rule16);
  fuzzy->addFuzzyRule(rule17);
  fuzzy->addFuzzyRule(rule18);
  fuzzy->addFuzzyRule(rule19);
  fuzzy->addFuzzyRule(rule20);
  fuzzy->addFuzzyRule(rule21);
  fuzzy->addFuzzyRule(rule22);
  fuzzy->addFuzzyRule(rule23);
  fuzzy->addFuzzyRule(rule24);
  fuzzy->addFuzzyRule(rule25); //13% Program Memory, 1% Dynamic Memory
  
  //GPS Setup
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  Serial1.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();
void loop() {

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - timer > 5000) {
    timer = millis();

    int inputSpeed = GPS.speed * 1.151;
    int inputNoise = dBfromAnalogMic();   

    fuzzy->setInput(1, inputSpeed);
    fuzzy->setInput(2, inputNoise);
    fuzzy->fuzzify();
  
    float output = fuzzy->defuzzify(1);
    float tunePrecent = GetAnalogValue();
    float finalOutputdB = tunePrecent * output;
  
    int carLevel = dBtoCarLevel(finalOutputdB);
  
    //Serial Debug Output
    Serial.print("GPS Speed (MPH): ");
    Serial.print(inputSpeed);
    Serial.println(" MPH");
    
    Serial.print("Noise: ");
    Serial.print(inputNoise);
    Serial.println(" dB");
    
    Serial.print("Output: ");
    Serial.println(output);
    Serial.print("TunePrecent: ");
    Serial.println(tunePrecent);
    Serial.print("Adjusted Output: ");
    Serial.println(finalOutputdB);
    Serial.print("Car Level: ");
    Serial.println(carLevel);
  
    Serial.println('\n');

  }

}

float GetAnalogValue(){

  float potVal = analogRead(analogInPin);
  return potVal / 256.0;
}


float dBfromAnalogMic(){

  float micVal = analogRead(micPin);
  return 21.9 * log(micVal) - 68.7; //Equation based on meter and phone producing 440Hz sine wave
}

int dBtoCarLevel(float dB){

  if(dB > 81)
    dB = 81;
  
  float carLevel = 0.0361 * pow(euler, 0.0812 * dB); //Equation based on phone sound meter with 440Hz sine wave

  if(carLevel < 1)
    carLevel = 1;
  
  return (int)carLevel;
}  
