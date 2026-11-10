/*
* MIT License
*
* Copyright (c) 2026 thieu-b55
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include <Arduino.h>
#include <FlashStorage_STM32.h>
#include <cstring>
#include <cmath>
#include <sstream>
#include <iostream>

using namespace std;

HardwareSerial A7670_SERIAL(PA3, PA2);                                                                //RX, TX  Serial2
HardwareSerial SERIAL_1(PA10, PA9);                                                                   //RX, TX  Serial1

#define   INPUT_0     PB5
#define   INPUT_1     PB6
#define   INPUT_2     PB7
#define   INPUT_3     PB8
#define   INPUT_4     PB9
#define   OUTPUT_0    PA0
#define   OUTPUT_1    PA1
#define   OUTPUT_2    PA4
#define   OUTPUT_3    PA5
#define   OUTPUT_4    PA6

#define   CONTROLE    0
#define   TLF_NUM     20
#define   COMMANDO    40
#define   START       60
#define   ON_OFF      75
#define   MINUTEN     90
#define   METER       105
#define   IN_0        120
#define   IN_1        135
#define   IN_2        150
#define   IN_3        165
#define   IN_4        180
#define   OUT_0       195
#define   OUT_1       210
#define   OUT_2       225
#define   OUT_3       240
#define   OUT_4       255
#define   MEM_0       270
#define   MEM_1       285
#define   MEM_2       300
#define   MEM_3       315
#define   MEM_4       330

void eeprom();
void instellen();
void wacht_op_antwoord();
void lees_antwoord();
void start_positie();
void input_opvragen();
void input_positie();
void input_weergeven();
void A7670_gnss_opvragen();
void sms_ontvangen();
void list();
void zoek_nummer();
void zoek_naam();
void gps_controle();
void stuur_sms();

// van google AI
void checkGpsVerplaatsing(float , float);
float berekenAfstand(float, float, float, float);
bool eersteMeting = true;
const float AARD_STRAAL_METERS = 6371000.0f;   // Straal van de aarde in meters (als float)
const float CONFIG_M_PI = 3.1415926535f;
float vorigeLat = 0.0f;
float vorigeLon = 0.0f;
float afstand_float;
// einde google AI

bool start_sms_bool = false;
bool input_0_vorig_bool = false;
bool input_1_vorig_bool = false;
bool input_2_vorig_bool = false;
bool input_3_vorig_bool = false;
bool input_4_vorig_bool = false;
bool input_0_nu_bool;
bool input_1_nu_bool;
bool input_2_nu_bool;
bool input_3_nu_bool;
bool input_4_nu_bool;
bool positie_bool = true;
bool positie_verzonden_bool = false;
bool gps_fix_bool = false;
bool error_bool;
bool list_bool = false;
bool geparkeerd_bool = false;

char data_char[275];
char buffer_char[100];
char controle_char[20];
char tlf_nummer_char[20];
char ontvangen_nummer_char[20];
char naam_char[15];
char print_char[50];

float lat_float;
float long_float;

int aantal_minuten_stil = 0;
int aantal_minuten_ingesteld;

unsigned int  input_int;

String ontvangen_string;
String data_string;
String GNSS_string;
String A7670_lat_string;
String A7670_long_string;
String A7670_sms_string;

unsigned long start_millis_long;
unsigned long int_start_millis_long;
unsigned long verplaatsing_interval_long;
unsigned long verplaatsing_millis_long;

void setup(){
  delay(2500);
  ontvangen_string.reserve(275);
  data_string.reserve(50);
  GNSS_string.reserve(100);
  A7670_lat_string.reserve(20);
  A7670_long_string.reserve(20);
  A7670_sms_string.reserve(100);
  SERIAL_1.begin(115200);
  eeprom();
  A7670_SERIAL.begin(9600);
  pinMode(INPUT_0, INPUT);
  pinMode(INPUT_1, INPUT);
  pinMode(INPUT_2, INPUT);
  pinMode(INPUT_3, INPUT);
  pinMode(INPUT_4, INPUT);
  pinMode(OUTPUT_0, OUTPUT);
  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OUTPUT_2, OUTPUT);
  pinMode(OUTPUT_3, OUTPUT);
  pinMode(OUTPUT_4, OUTPUT);
  if(EEPROM.read(START) == 0){
    digitalWrite(OUTPUT_0, false);
    digitalWrite(OUTPUT_1, false);
    digitalWrite(OUTPUT_2, false);
    digitalWrite(OUTPUT_3, false);
    digitalWrite(OUTPUT_4, false);
    EEPROM.write(MEM_0, 0);
    EEPROM.write(MEM_1, 0);
    EEPROM.write(MEM_2, 0);
    EEPROM.write(MEM_3, 0);
    EEPROM.write(MEM_4, 0);
  }
  if(EEPROM.read(START) == 1){
    if(EEPROM.read(MEM_0) == 0){
      digitalWrite(OUTPUT_0, false);
    }
    if(EEPROM.read(MEM_0) == true){
      digitalWrite(OUTPUT_0, true);
    }
    if(EEPROM.read(MEM_1) == 0){
      digitalWrite(OUTPUT_1, false);
    }
    if(EEPROM.read(MEM_1) == 1){
      digitalWrite(OUTPUT_1, true);
    }
    if(EEPROM.read(MEM_2) == 0){
      digitalWrite(OUTPUT_2, false);
    }
    if(EEPROM.read(MEM_2) == 1){
      digitalWrite(OUTPUT_2, true);
    }
    if(EEPROM.read(MEM_3) == 0){
      digitalWrite(OUTPUT_3, false);
    }
    if(EEPROM.read(MEM_3) == 1){
      digitalWrite(OUTPUT_3, true);
    }
    if(EEPROM.read(MEM_4) == 0){
      digitalWrite(OUTPUT_4, false);
    }
    if(EEPROM.read(MEM_4) == 1){
      digitalWrite(OUTPUT_4, true);
    }
  }
  if(EEPROM.read(ON_OFF)){
    start_sms_bool = true;
  }
  positie_bool = true;
  instellen();
  EEPROM.get(TLF_NUM, tlf_nummer_char);
  start_millis_long = millis();
  verplaatsing_millis_long = millis();
  verplaatsing_interval_long = 60000;
  aantal_minuten_ingesteld = 15;
}

void loop(){
  /*
  geef positie door bij opstart
  */
  if(start_sms_bool || (!(positie_bool))){
    start_positie();
  }
  /*
  geef positie door bij gps fix
  */
  if(((millis() - int_start_millis_long) > 60000) && (!positie_bool)){
    int_start_millis_long = millis();
    gps_controle();
  }
  /*
  inputs checken op verandering als on/off == 1
  */
  if(EEPROM.read(ON_OFF)){
    input_opvragen();
  }

  if((millis() - verplaatsing_millis_long) > 60000){
    verplaatsing_millis_long = millis();
    A7670_gnss_opvragen();
    if(gps_fix_bool){
      checkGpsVerplaatsing(lat_float, long_float);
    }
  }
  /*
  opvragen seriële poort A7670E 
  */
  if(A7670_SERIAL.available() > 0){
    memset(data_char, 0, sizeof data_char);
    A7670_SERIAL.readBytesUntil(char(0), data_char, 250);
    int x = 0;
    ontvangen_string = "";
    while(data_char[x] != char(0) && (x < 250)){
      ontvangen_string += data_char[x];
      x++;
    }
    if((strstr(data_char, "CMT") != NULL)){
      sms_ontvangen();
    }
  }
}

/*FUNCTIONS*/

void eeprom(){
  EEPROM.get(CONTROLE, controle_char);
  if((strstr(controle_char, "thieu-b55") == NULL)){
    SERIAL_1.println("eeprom instellen");
    EEPROM.put(CONTROLE, "thieu-b55");
    EEPROM.put(TLF_NUM, "+32000000000");
    EEPROM.put(COMMANDO, "positie");
    EEPROM.write(START, 0);                       // 1 >> output waarde bij afschakelen bij opstart    0 >> output 0 bij opstart
    EEPROM.write(ON_OFF, 1);                      // 0 >> automatische meldingen uitgeschakeld
    EEPROM.put(MINUTEN, "10");
    EEPROM.put(METER, "250");
    EEPROM.put(IN_0, "in_0");
    EEPROM.put(IN_1, "in_1");
    EEPROM.put(IN_2, "in_2");
    EEPROM.put(IN_3, "in_3");
    EEPROM.put(IN_4, "in_4");
    EEPROM.put(OUT_0, "out_0");
    EEPROM.put(OUT_1, "out_1");
    EEPROM.put(OUT_2, "out_2");
    EEPROM.put(OUT_3, "out_3");
    EEPROM.put(OUT_4, "out_4");
    EEPROM.write(MEM_0, 0);
    EEPROM.write(MEM_1, 0);
    EEPROM.write(MEM_2, 0);
    EEPROM.write(MEM_3, 0);
    EEPROM.write(MEM_4, 0);
    EEPROM.commit();
    SERIAL_1.println("eeprom ingesteld");
  }
}

void instellen(){
   //OK?
  A7670_SERIAL.println("AT");
  wacht_op_antwoord();
  lees_antwoord();

  //GSM character set
  A7670_SERIAL.println("AT+CSCS=\"GSM\""); 
  wacht_op_antwoord();
  lees_antwoord();
  A7670_SERIAL.println("AT+CSCS?"); 
  wacht_op_antwoord();
  lees_antwoord();
  
  //SMS Text mode
  A7670_SERIAL.println("AT+CMGF=1");
  wacht_op_antwoord();
  lees_antwoord();
  A7670_SERIAL.println("AT+CMGF?");
  wacht_op_antwoord();
  lees_antwoord();
  
  // Send SMS to Serial not to memory
  A7670_SERIAL.println("AT+CNMI=2,2,0,0,0"); 
  wacht_op_antwoord();
  lees_antwoord();
  A7670_SERIAL.println("AT+CNMI?");
  wacht_op_antwoord();
  lees_antwoord();

  //Turn on GNSS 
  A7670_SERIAL.println("AT+CGNSSPWR=1,0");
  delay(15000);
  lees_antwoord();
  A7670_SERIAL.println("AT+CGNSSPWR?");
  wacht_op_antwoord();
  lees_antwoord();
  
  //GPS + GLONASS + GALILEO + SBAS + QZSS
  A7670_SERIAL.println("AT+CGNSSMODE=3");
  wacht_op_antwoord();
  lees_antwoord();
  delay(2500);
}

void wacht_op_antwoord(){
  while(!(A7670_SERIAL.available())){
    delay(1);
  }
}

void lees_antwoord(){
  while(A7670_SERIAL.available()){
    //SERIAL_1.write(char(A7670_SERIAL.read()));
    A7670_SERIAL.read();
    delay(10);
  }
  delay(10);
}

void start_positie(){
  if(start_sms_bool){
    start_sms_bool = false;
    A7670_sms_string = "module start \n";
    positie_bool = true;
    positie_verzonden_bool = false;
    gps_controle();
  }
}

void input_opvragen(){
  /*
  geef positie door bij veranderen INPUT_0
  */
  input_0_nu_bool = digitalRead(INPUT_0);
  if(input_0_nu_bool != input_0_vorig_bool){
    input_int = 0;
    input_positie();
  }
  /*
  geef positie en status door bij veranderen INPUT_1
  */
  input_1_nu_bool = digitalRead(INPUT_1);
  if(input_1_nu_bool != input_1_vorig_bool){
    input_int = 1;
    input_positie();
  }
  /*
  geef positie en status door bij veranderen INPUT_2
  */
  input_2_nu_bool = digitalRead(INPUT_2);
  if(input_2_nu_bool != input_2_vorig_bool){
    input_int = 2;
    input_positie();
  }
  /*
  geef positie en status door bij veranderen INPUT_3
  */
  input_3_nu_bool = digitalRead(INPUT_3);
  if(input_3_nu_bool != input_3_vorig_bool){
    input_int = 3;
    input_positie();
  }
  /*
  geef positie en status door bij veranderen INPUT_4
  */
  input_4_nu_bool = digitalRead(INPUT_4);
  if(input_4_nu_bool != input_4_vorig_bool){
    input_int = 4;
    input_positie();
  }
}

void input_positie(){
  input_weergeven();
  positie_bool = true;
  positie_verzonden_bool = false;
  gps_controle();   
}

void input_weergeven(){
  char naam_char[20];
  A7670_sms_string = "";
  if(input_int == 0){
    sprintf(print_char, "%s    %d  >>  %d \n", EEPROM.get(IN_0, naam_char), input_0_vorig_bool, input_0_nu_bool);
    A7670_sms_string = A7670_sms_string + print_char;
    input_0_vorig_bool = input_0_nu_bool;
  }
  if(input_int == 1){
    sprintf(print_char, "%s    %d  >>  %d \n", EEPROM.get(IN_1, naam_char), input_1_vorig_bool, input_1_nu_bool);
    A7670_sms_string = A7670_sms_string + print_char;
    input_1_vorig_bool = input_1_nu_bool;
  }
  if(input_int == 2){
    sprintf(print_char, "%s    %d  >>  %d \n", EEPROM.get(IN_2, naam_char), input_2_vorig_bool, input_2_nu_bool);
    A7670_sms_string = A7670_sms_string + print_char;
    input_2_vorig_bool = input_2_nu_bool;
  }
  if(input_int == 3){
    sprintf(print_char, "%s    %d  >>  %d \n", EEPROM.get(IN_3, naam_char), input_3_vorig_bool, input_3_nu_bool);
    A7670_sms_string = A7670_sms_string + print_char;
    input_3_vorig_bool = input_3_nu_bool;
  }
  if(input_int == 4){
    sprintf(print_char, "%s    %d  >>  %d \n", EEPROM.get(IN_4, naam_char), input_4_vorig_bool, input_4_nu_bool);
    A7670_sms_string = A7670_sms_string + print_char;
    input_4_vorig_bool = input_4_nu_bool;
  }
}

void A7670_gnss_opvragen(){
  int dummy_int;
  int lat_1_int;
  int lat_2_int;
  int long_1_int;
  int long_2_int;
  gps_fix_bool = false;
  GNSS_string = "";
  A7670_SERIAL.println("AT+CGNSSINFO"); 
  wacht_op_antwoord();
  while(A7670_SERIAL.available()){
    GNSS_string += char(A7670_SERIAL.read());
    delay(10);
  }
  // gps fixed?
  if(((GNSS_string.indexOf('N') != -1) || (GNSS_string.indexOf('S') != -1)) && ((GNSS_string.indexOf('E') != -1) || (GNSS_string.indexOf('W') != -1))){
    gps_fix_bool = true;
    dummy_int = GNSS_string.indexOf(',');
    dummy_int = GNSS_string.indexOf(',', (dummy_int + 1));
    dummy_int = GNSS_string.indexOf(',', (dummy_int + 1));
    dummy_int = GNSS_string.indexOf(',', (dummy_int + 1));
    lat_1_int = GNSS_string.indexOf(',', (dummy_int + 1));
    lat_2_int = GNSS_string.indexOf(',', (lat_1_int + 1));
    long_1_int = GNSS_string.indexOf(',', (lat_2_int + 1));
    long_2_int = GNSS_string.indexOf(',', (long_1_int + 1));
    A7670_lat_string = GNSS_string.substring((lat_1_int + 1), lat_2_int);
    A7670_long_string = GNSS_string.substring((long_1_int + 1), long_2_int);
    char A7670_lat_char[20];
    char A7670_long_char[20];
    strcpy(A7670_lat_char, A7670_lat_string.c_str());
    strcpy(A7670_long_char, A7670_long_string.c_str());
    lat_float = atof(A7670_lat_char);
    long_float = atof(A7670_long_char);
  }
}

void sms_ontvangen(){
  /*
  zoek nummer van afzender
  */
  int eerste_int;
  int tweede_int;
  unsigned int x_int = 0;
  int pos_int = 0;
  eerste_int = ontvangen_string.indexOf('"');                                                                                                   
  tweede_int = ontvangen_string.indexOf('"', (eerste_int + 1));
  ontvangen_string.substring((eerste_int + 1), tweede_int).toCharArray(tlf_nummer_char, ((tweede_int - eerste_int) + 1));
  /*
  eerste oproep nummer bewaren in EEPROM
  */
  if(strstr(data_char, "#") != NULL){ 
    A7670_sms_string = "";
    // data_string >>> string vanaf # (inbegrepen) tot einde ontvangen_string 
    pos_int = ontvangen_string.indexOf("#");
    data_string = ontvangen_string.substring(pos_int, ontvangen_string.length() );
    strcpy(data_char, data_string.c_str());
    // tlf nummer ingeven in flash
    if(strstr(data_char, "+") != NULL){
      memset(ontvangen_nummer_char, 0, sizeof ontvangen_nummer_char);
      zoek_nummer();
      EEPROM.put(TLF_NUM, ontvangen_nummer_char);
      EEPROM.commit();
      EEPROM.get(TLF_NUM, ontvangen_nummer_char);
      A7670_sms_string = "nummer in flash \n";
      A7670_sms_string = A7670_sms_string + ontvangen_nummer_char;
      stuur_sms();
    }
  /*
  uitsturen uitgangen
  */
    EEPROM.get(OUT_0, naam_char);
    // naam_char moet op data_char[2] beginnen anders vergissing met ingeven nieuwe naam
    if((string(data_char).find(string(naam_char))) == 2){
      char* positie = strstr(data_char, naam_char);
      int index = positie - data_char;
      if(data_char[index + strlen(naam_char) +1] == char(48)){
        digitalWrite(OUTPUT_0, false);
        EEPROM.write(MEM_0, 0);
        EEPROM.commit();
      }
      if((data_char[index + strlen(naam_char) +1]) == char(49)){
        digitalWrite(OUTPUT_0, true);
        EEPROM.write(MEM_0, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string + naam_char + "   " + digitalRead(OUTPUT_0);
      stuur_sms();
    }
    EEPROM.get(OUT_1, naam_char);
    if((string(data_char).find(string(naam_char))) == 2){
      char* positie = strstr(data_char, naam_char);
      int index = positie - data_char;
      if(data_char[index + strlen(naam_char) +1] == char(48)){
        digitalWrite(OUTPUT_1, false);
        EEPROM.write(MEM_1, 0);
        EEPROM.commit();
      }
      if(int(data_char[index + strlen(naam_char) +1]) == char(49)){
        digitalWrite(OUTPUT_1, true);
        EEPROM.write(MEM_1, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string + naam_char + "   " + digitalRead(OUTPUT_1);
      stuur_sms();
    }
    EEPROM.get(OUT_2, naam_char);
    if((string(data_char).find(string(naam_char))) == 2){
      char* positie = strstr(data_char, naam_char);
      int index = positie - data_char;
      if(data_char[index + strlen(naam_char) +1] == char(48)){
        digitalWrite(OUTPUT_2, false);
        EEPROM.write(MEM_2, 0);
        EEPROM.commit();
      }
      if(int(data_char[index + strlen(naam_char) +1]) == char(49)){
        digitalWrite(OUTPUT_2, true);
        EEPROM.write(MEM_2, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string + naam_char + "   " + digitalRead(OUTPUT_2);
      stuur_sms();
    }
    EEPROM.get(OUT_3, naam_char);
    if((string(data_char).find(string(naam_char))) == 2){
      char* positie = strstr(data_char, naam_char);
      int index = positie - data_char;
      if(data_char[index + strlen(naam_char) +1] == char(48)){
        digitalWrite(OUTPUT_3, false);
        EEPROM.write(MEM_3, 0);
        EEPROM.commit();
      }
      if(int(data_char[index + strlen(naam_char) +1]) == char(49)){
        digitalWrite(OUTPUT_3, true);
        EEPROM.write(MEM_3, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string + naam_char + "   " + digitalRead(OUTPUT_3);
      stuur_sms();
    }
    EEPROM.get(OUT_4, naam_char);
    if((string(data_char).find(string(naam_char))) == 2){
      char* positie = strstr(data_char, naam_char);
      int index = positie - data_char;
      if(data_char[index + strlen(naam_char) +1] == char(48)){
        digitalWrite(OUTPUT_4, false);
        EEPROM.write(MEM_4, 0);
        EEPROM.commit();
      }
      if(int(data_char[index + strlen(naam_char) +1]) == char(49)){
        digitalWrite(OUTPUT_4, true);
        EEPROM.write(MEM_4, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string + naam_char + "   " + digitalRead(OUTPUT_4);
      stuur_sms();
    }
   /*
  input/output naam bewaren in EEPROM
  */
    if((strstr(data_char, "i0") != NULL)){
      zoek_naam();
      EEPROM.put(IN_0, naam_char);
      EEPROM.commit();
      sprintf(print_char, "i0 >> %s", EEPROM.get(IN_0, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "i1") != NULL){
      zoek_naam();
      EEPROM.put(IN_1, naam_char);
      EEPROM.commit();
      sprintf(print_char, "i1 >> %s", EEPROM.get(IN_1, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "i2") != NULL){
      zoek_naam();
      EEPROM.put(IN_2, naam_char);
      EEPROM.commit();
      sprintf(print_char, "i2 >> %s", EEPROM.get(IN_2, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "i3") != NULL){
      zoek_naam();
      EEPROM.put(IN_3, naam_char);
      EEPROM.commit();
      sprintf(print_char, "i3 >> %s", EEPROM.get(IN_3, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "i4") != NULL){
      zoek_naam();
      EEPROM.put(IN_4, naam_char);
      EEPROM.commit();
      sprintf(print_char, "i4 >> %s", EEPROM.get(IN_4, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "o0") != NULL){
      zoek_naam();
      EEPROM.put(OUT_0, naam_char);
      EEPROM.commit();
      sprintf(print_char, "o0 >> %s", EEPROM.get(OUT_0, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "o1") != NULL){
      zoek_naam();
      EEPROM.put(OUT_1, naam_char);
      EEPROM.commit();
      sprintf(print_char, "o1 >> %s", EEPROM.get(OUT_1, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "o2") != NULL){
      zoek_naam();
      EEPROM.put(OUT_2, naam_char);
      EEPROM.commit();
      sprintf(print_char, "o2 >> %s", EEPROM.get(OUT_2, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "o3") != NULL){
      zoek_naam();
      EEPROM.put(OUT_3, naam_char);
      EEPROM.commit();
      sprintf(print_char, "o3 >> %s", EEPROM.get(OUT_3, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
    if(strstr(data_char, "o4") != NULL){
      zoek_naam();
      EEPROM.put(OUT_4, naam_char);
      EEPROM.commit();
      sprintf(print_char, "o4 >> %s", EEPROM.get(OUT_4, naam_char));
      A7670_sms_string = print_char;
      stuur_sms();
    }
  /*
  positie weergeven op aanvraag
  */
    EEPROM.get(COMMANDO, naam_char);
    if((strstr(data_char, naam_char)) != NULL){
      // inien commando "com" geen positie opvragen
      if((string(data_char).find(string("com"))) == -1){
        positie_bool = true;
        positie_verzonden_bool = false;
        gps_controle();
      }
    }
  /*
  geparkeerd ??
  */
    if(strstr(data_char, "park") != NULL){
      if(geparkeerd_bool){
        A7670_sms_string = "geparkeerd \n";
      }
      else{
        A7670_sms_string = "niet geparkeerd \n";
      }
      positie_bool = true;
      positie_verzonden_bool = false;
      gps_controle();
    }
  /*
  positie opvraag commando instellen
  */
    if(strstr(data_char, "com") != NULL){
      memset(naam_char, 0,  sizeof naam_char);
      data_string.substring(6, (data_string.length() - 2)).toCharArray(naam_char, data_string.length());
      EEPROM.put(COMMANDO, naam_char);
      EEPROM.commit();
      EEPROM.get(COMMANDO, naam_char);
      sprintf(print_char, "positie opvragen met : %s",  naam_char);
      A7670_sms_string = print_char;
      stuur_sms();
    }
  /*
  instellen minuten zonder verplaatsing alvorens geparkeerd
  */
    if(strstr(data_char, "min") != NULL){
      memset(naam_char, 0,  sizeof naam_char);
      data_string.substring(6, (data_string.length() - 2)).toCharArray(naam_char, data_string.length());
      EEPROM.put(MINUTEN, naam_char);
      EEPROM.commit();
      EEPROM.get(MINUTEN, naam_char);
      sprintf(print_char, "stilstand alvorens geparkeerd : %s minuten",  naam_char);
      A7670_sms_string = print_char;
      stuur_sms();
    }
  /*
  instellen verplaatsing vanaf geparkeerd alvorens alarm in meters
  */
    if(strstr(data_char, "mtr") != NULL){
      memset(naam_char, 0,  sizeof naam_char);
      data_string.substring(6, (data_string.length() - 2)).toCharArray(naam_char, data_string.length());
      EEPROM.put(METER, naam_char);
      EEPROM.commit();
      EEPROM.get(METER, naam_char);
      sprintf(print_char, "verplaatsing alvorens alarm : %s meter",  naam_char);
      A7670_sms_string = print_char;
      stuur_sms();
   }
  /*
  reset alle uitgangen
  */
    if(strstr(data_char, "rst") != NULL){
      digitalWrite(OUTPUT_0, false);
      EEPROM.write(MEM_0, 0);
      digitalWrite(OUTPUT_1, false);
      EEPROM.write(MEM_1, 0);
      digitalWrite(OUTPUT_2, false);
      EEPROM.write(MEM_2, 0);
      digitalWrite(OUTPUT_3, false);
      EEPROM.write(MEM_3, 0);
      digitalWrite(OUTPUT_4, false);
      EEPROM.write(MEM_4, 0);
      EEPROM.commit();
      A7670_sms_string = "";
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_0, naam_char), digitalRead(OUTPUT_0));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_1, naam_char), digitalRead(OUTPUT_1));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_2, naam_char), digitalRead(OUTPUT_2));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_3, naam_char), digitalRead(OUTPUT_3));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d", EEPROM.get(OUT_4, naam_char), digitalRead(OUTPUT_4));
      A7670_sms_string = A7670_sms_string + print_char;
      stuur_sms();
    }
  /*
  set alle uitgangen
  */
    if(strstr(data_char, "set") != NULL){
      digitalWrite(OUTPUT_0, true);
      EEPROM.write(MEM_0, 1);
      digitalWrite(OUTPUT_1, true);
      EEPROM.write(MEM_1, 1);
      digitalWrite(OUTPUT_2, true);
      EEPROM.write(MEM_2, 1);
      digitalWrite(OUTPUT_3, true);
      EEPROM.write(MEM_3, 1);
      digitalWrite(OUTPUT_4, true);
      EEPROM.write(MEM_4, 1);
      EEPROM.commit();
      A7670_sms_string = "";
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_0, naam_char), digitalRead(OUTPUT_0));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_1, naam_char), digitalRead(OUTPUT_1));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_2, naam_char), digitalRead(OUTPUT_2));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_3, naam_char), digitalRead(OUTPUT_3));
      A7670_sms_string = A7670_sms_string + print_char;
      sprintf(print_char, "%s    %d", EEPROM.get(OUT_4, naam_char), digitalRead(OUTPUT_4));
      A7670_sms_string = A7670_sms_string + print_char;
      stuur_sms();
    }
  /*
  list
  */
    if(strstr(data_char, "list") != NULL){
      list();
    }
  /*
  moeten uitgangen na opstart waarde hebben van bij shdn?
  */
    if(strstr(data_char, "start") != NULL){
      if(data_char[8] == char(48)){
        EEPROM.write(START, 0);
        EEPROM.commit();
      }
      if(int(data_char[8]) == char(49)){
        EEPROM.write(START, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string +  "start   " + EEPROM.read(START);
      stuur_sms();
    }
  /*
  automatische sms uitschakelen(0) of inschakelen(1)
  */
    if(strstr(data_char, "on/off") != NULL){
      if(data_char[9] == char(48)){
        EEPROM.write(ON_OFF, 0);
        EEPROM.commit();
      }
      if(int(data_char[9]) == char(49)){
        EEPROM.write(ON_OFF, 1);
        EEPROM.commit();
      }
      A7670_sms_string = A7670_sms_string +  "on/off   " + EEPROM.read(ON_OFF);
      stuur_sms();
    }
  }
}

void list(){
  char tmp_char[50];
  A7670_sms_string = "";
  A7670_sms_string = A7670_sms_string + tlf_nummer_char + "\n";
  sprintf(print_char, "%s    %s", "positie opvragen", EEPROM.get(COMMANDO, tmp_char));
  A7670_sms_string = A7670_sms_string + print_char + "\n";
  sprintf(print_char, "%s    %d", "start", EEPROM.read(START));
  A7670_sms_string = A7670_sms_string + print_char + "\n";
  sprintf(print_char, "%s    %d", "on/off", EEPROM.read(ON_OFF));
  A7670_sms_string = A7670_sms_string + print_char;
  stuur_sms();
  A7670_sms_string = "";
  sprintf(print_char, "%s    %s", "minuten", EEPROM.get(MINUTEN, tmp_char));
  A7670_sms_string = A7670_sms_string + print_char + "\n";
  sprintf(print_char, "%s    %s", "verplaatsing in meter", EEPROM.get(METER, tmp_char));
  A7670_sms_string = A7670_sms_string + print_char;
  stuur_sms();
  A7670_sms_string = "";
  sprintf(print_char, "%s    %d \n", EEPROM.get(IN_0, naam_char), digitalRead(INPUT_0));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(IN_1, naam_char), digitalRead(INPUT_1));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(IN_2, naam_char), digitalRead(INPUT_2));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(IN_3, naam_char), digitalRead(INPUT_3));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(IN_4, naam_char), digitalRead(INPUT_4));
  A7670_sms_string = A7670_sms_string + print_char;
  stuur_sms();
  A7670_sms_string = "";
  sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_0, naam_char), digitalRead(OUTPUT_0));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_1, naam_char), digitalRead(OUTPUT_1));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_2, naam_char), digitalRead(OUTPUT_2));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_3, naam_char), digitalRead(OUTPUT_3));
  A7670_sms_string = A7670_sms_string + print_char;
  sprintf(print_char, "%s    %d \n", EEPROM.get(OUT_4, naam_char), digitalRead(OUTPUT_4));
  A7670_sms_string = A7670_sms_string + print_char;
  stuur_sms();
}
/*
zoek nummer
*/
void zoek_nummer(){ 
  int positie_int = 2;
  int char_positie_int = 0;
  memset(ontvangen_nummer_char, 0, sizeof(ontvangen_nummer_char));
  while((int(data_string[positie_int]) > 32) && (int(data_string[positie_int]) < 127)){
    ontvangen_nummer_char[char_positie_int] = data_string[positie_int];
    char_positie_int ++;
    positie_int ++;
  }
}
/*
zoek naam input/output
*/
void zoek_naam(){
  int positie_int = 5;
  int char_positie_int = 0;
  memset(naam_char, 0, sizeof(naam_char));
  while((int(data_string[positie_int]) > 32) && (int(data_string[positie_int]) < 127)){
    naam_char[char_positie_int] = data_string[positie_int];
    char_positie_int ++;
    positie_int ++;
  }
}
/*
controle gps fix alvorens versturen sms
*/
void gps_controle(){
  A7670_gnss_opvragen();
  if((gps_fix_bool) && (!(positie_verzonden_bool))){
    A7670_sms_string = A7670_sms_string + "positie : \n";
    A7670_sms_string = A7670_sms_string + "www.google.com/maps/place/" + A7670_lat_string + "," + A7670_long_string;
    stuur_sms();
    A7670_sms_string = "";
    positie_verzonden_bool = true;
  }
  else{
    if((!gps_fix_bool) && (positie_bool)){
      A7670_sms_string = A7670_sms_string + "positie nog niet beschikbaar";
      stuur_sms();
      A7670_sms_string = "";
      positie_bool = false;
    }
  }
}
/*
sms sturen
*/
void stuur_sms(){
  memset(buffer_char, 0, sizeof buffer_char);
  sprintf(buffer_char, "AT+CMGS=\"%s\"", tlf_nummer_char);
  A7670_SERIAL.println(buffer_char);
  wacht_op_antwoord();
  lees_antwoord();
  A7670_SERIAL.println(A7670_sms_string);
  wacht_op_antwoord();
  lees_antwoord();  
  A7670_SERIAL.write(26);
  wacht_op_antwoord();
  lees_antwoord();
}
/*
Volgende functies voor het berekenen van verplaatsing komen van google AI
*/
float naarRadialen(float graden) {
    return graden * CONFIG_M_PI / 180.0f;
}
// Geoptimaliseerde Haversine voor STM32 Hardware FPU
float berekenAfstand(float lat1, float lon1, float lat2, float lon2) {
    float dLat = naarRadialen(lat2 - lat1);
    float dLon = naarRadialen(lon2 - lon1);

    float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
              cos(naarRadialen(lat1)) * cos(naarRadialen(lat2)) *
              sin(dLon / 2.0f) * sin(dLon / 2.0f);
               
    float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
    
    return AARD_STRAAL_METERS * c;
}
void checkGpsVerplaatsing(float actueleLat, float actueleLon) {
  if (eersteMeting){
    vorigeLat = actueleLat;
    vorigeLon = actueleLon;
    eersteMeting = false;
    return;
  }
  afstand_float= berekenAfstand(vorigeLat, vorigeLon, actueleLat, actueleLon);
  EEPROM.get(METER, naam_char);
  // char_array naar int
  stringstream str(naam_char);
  int max_afstand_int;
  str >> max_afstand_int;
  /**/
  if((int(afstand_float)) > max_afstand_int){                                               // indien elke minuut meer dan ... meter afgelegd
    aantal_minuten_stil = 0;
    vorigeLat = actueleLat;
    vorigeLon = actueleLon;
  }
  else{
    aantal_minuten_stil += 1;
    EEPROM.get(MINUTEN, naam_char);
    // char_array naar int
    stringstream str(naam_char);
    int minuten_int;
    str >> minuten_int;
    /**/
      if(EEPROM.read(ON_OFF)){
        if((aantal_minuten_stil >= minuten_int) && (!geparkeerd_bool)){
          geparkeerd_bool = true;
          A7670_sms_string = "Geparkeerd \n";
          positie_bool = true;
          positie_verzonden_bool = false;
          gps_controle();
        }
      }
  }
  if(geparkeerd_bool && ((int(afstand_float)) > max_afstand_int)){
      geparkeerd_bool = false;
      A7670_sms_string = "";
      memset(buffer_char, 0, sizeof buffer_char);
      sprintf(buffer_char, "Verplaatsing meer dan %s meter \n", EEPROM.get(METER, naam_char));
      A7670_sms_string = A7670_sms_string + buffer_char;
      positie_bool = true;
      positie_verzonden_bool = false;
      gps_controle();
  }
}
  