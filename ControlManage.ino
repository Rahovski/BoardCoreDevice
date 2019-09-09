#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DirectIO.h>
#include <limits.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

/*------------------------------0 9 9 2 for arduino_MEGA2560 with EthernetShield----------------------------*/

/*------------------------------------------------------DATA--------------------------------------------*/
short int Number_of_all_Device = 0;
short int Number_Button = 0;
short int Number_Motor = 0;
short int Number_Output = 0;
short int Number_Pot = 0;
short int Number_Servo = 0;
short int Number_Mpb = 0;
short int Number_Encoder = 0;
/*-------------------------------ButtonData-------------------*/
char ButtonNameDataBase[64][16];
byte ButtonPinDataBase[64];
byte ButtonValueDataBase[64];
/*------------------------------MotorData---------------------*/
char MotorNameDataBase[14][16];
char MotorNoSortBase[14][16];
byte MotorPinDataBase[14];
short int MotorPhaseDataBase[14];
int32_t MotorNeedValue[14];
int32_t MotorCurrentValue[14];
/*----------------------------MODIFIED MOTOR DATA-------------*/
bool MotorSensorState[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0};                   //Data from Holls sensors (logic 1 or 0)
bool MotorSensorPolarity[14] = {1, 1, 1, 1, 1, 1, 1, 1, 1 , 1, 1, 1, 1, 1};                //Zero sensor polarity
int32_t MotorNullCorrection[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};           //Angle for motor,which one need to make after find zero sensor
bool MotorType[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //VID or STH
uint32_t MotorRange[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //for VID is a correction angle
/*-------------------------------OutputData-------------------*/
char OutputNameDataBase[64][16];
byte OutputPinDataBase[64];
short int OutputValueDataBase[64];
/*-------------------------------ServoData-------------------*/
//Not used for avia simulators
char ServoNameDataBase[8][16];
byte ServoPinDataBase[8];
short int ServoValueDataBase[8];
/*---------------------------------PotData--------------------*/
char PotNameDataBase[16][16];
byte PotPinDataBase[16];
short int PotValueDataBase[16];
/*------------------------MultiPositionButton---------------*/
char MpbNameDataBase[9][16];
short int MpbButtonCountBase[9];
short int **MpbPinDataBase = new short int* [9];
/*-----------------------EncoderData------------------------*/
char EncoderNameDataBase[9][16];
byte EncoderPinDataBase[9];
short int EncoderValueDataBase[9];
/*----------------------------------------------------------*/
bool motorPhases[8][4] = { //[phase][pin]
  //Winding   A  B  A  B
  {1, 1, 0 , 0},
  {0, 1 , 0 , 0},
  {0, 1, 1 , 0},
  {0, 0, 1 , 0},
  {0, 0, 1 , 1},
  {0, 0, 0 , 1},
  {1, 0, 0 , 1},
  {1, 0, 0 , 0}
};
short int motor_delay = 0;
uint16_t soft_delay = 0;
short int _step = 0; // For step motor control
short int ReservePin[70];
byte mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEF, 0x00};
int DontTouchPins[8] = {0, 1, 4, 10, 50, 51, 52, 53};
int used_EEPROM = 0;
bool ch_system_info = 0;
byte ip_data[4] = {192, 168, 2, 11};
//Basic setting on board
IPAddress myDNS(192, 168, 2, 1);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 128);
unsigned int localPort = 8888;
EthernetUDP Udp_Core;
/*------------------------------------------------END_DATA----------------------------------------------*/

bool UpperFront(bool last_stay, bool stay, bool polarity = true) {
  return (polarity == last_stay) && ((!stay) == polarity);
}

void StartCalibration() {
  uint8_t IsCalibrated[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0};
  uint16_t CalibrUngle[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0 , 0 , 0, 0, 0, 0};
  for (int i = 0; i < 14; i++) {
    MotorNeedValue[i] = 0;
    MotorCurrentValue[i] = 0;
  }
  for (uint8_t i = 0; i < Number_Motor; i++) {
    short int SensorPin = MotorPinDataBase[i] + 4;
    InputPin mi_8(SensorPin, false);
    MotorSensorState[i] = mi_8.read() == LOW;
  }
  long int max_range = 0;
  //Because MotorRange always above zero
  for (uint8_t r = 0; r < Number_Motor; r++) {
    if (max_range < MotorRange[r]) {
      max_range = MotorRange[r];
    }
  }
  byte Calibrated = 0;
  int CalibrateCounter = 0;
  soft_delay = 0;
  while ((Calibrated < Number_Motor) && (CalibrateCounter <= max_range)) {
    delay(5);
    CalibrateCounter++;
    int8_t dir = -1;
    for (int i = 0; i < Number_Motor; i++) {
      bool CurSensorState = false;
      short int SensorPin = MotorPinDataBase[i] + 4;
      if (!IsCalibrated[i]) {
        dir = 1 - MotorSensorPolarity[i] * 2;
        MotorAction(CalibrUngle[i]*dir, i);
        CalibrUngle[i]++;
        //This works fine
        if (MotorType[i] == 1) {
          InputPin mi_8(SensorPin, false);
          CurSensorState = mi_8.read() == LOW;
          IsCalibrated[i] = UpperFront(MotorSensorState[i], CurSensorState, MotorSensorPolarity[i]);
          MotorSensorState[i] = CurSensorState;
        }
        else {
          IsCalibrated[i] = (MotorCurrentValue[i] == MotorRange[i]);
        }
        if (IsCalibrated[i]) {
          MotorCurrentValue[i] = MotorNullCorrection[i];
          MotorNeedValue[i] = 0;
          Calibrated++;
        }
      } /*else if((IsCalibrated[i])&&(CalibrUngle[i]!=0)) {
        dir = 1 - MotorSensorPolarity[i] * 2;
        MotorAction(-CalibrUngle[i]*dir, i);
        CalibrUngle[i]--;
      }*/
      //MotorSensorState[i] = MotorNullCorrection[i];
    }
  }
  //Change soft_delay , if you add more motors
  soft_delay = 32;
}

void setup() {
  // put your setup code here, to run once:;
  Serial.begin(9600);
  used_EEPROM = EEPROM.read(0);
  /*Считываем из ПЗУ данные о количестве приборах*/
  if (used_EEPROM) {
    for (int i = 0; i < 4096; i++) {
      EEPROM.write(i, 0);
    }
  }
  //pinMode(2, OUTPUT);
  //digitalWrite(2, LOW);
  Number_of_all_Device = (short int)EEPROM.read(1);
  Number_Button = (short int)EEPROM.read(2);
  //Number_Encoder = (short int)EEPROM.read(8);
  for (int i = 0; i < 70; i++) {
    ReservePin[i] = -1;
  }
  for (int i = 0; i < 64; i++) {
    ButtonNameDataBase[i][0] = '\0';
    ButtonPinDataBase[i] = 128;
    ButtonValueDataBase[i] = -1;
  }
  int pos = 1;
  for (int i = 0; i < Number_Button; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if ( tmp != 0) {
        ButtonNameDataBase[i][j] = (char)tmp;
        j++;
      } else {
        ButtonNameDataBase[i][j] = '\0';
        break;
      }
    }
    if (Number_Button == 70) {
      Number_Button = EEPROM.read(2);
    }
    ButtonPinDataBase[i] = (short int)EEPROM.read(pos * 16 + strlen(ButtonNameDataBase[i]) + 1);
    ReservePin[ButtonPinDataBase[i]] = ButtonPinDataBase[i];
    pos++;
  }

  for (int i = 0; i < 14; i++) {
    MotorNameDataBase[i][0] = '\0';
    MotorNoSortBase[i][0] = '\0';
    MotorPinDataBase[i] = 128;
    MotorPhaseDataBase[i] = 0;
    MotorNeedValue[i] = 0;
    MotorCurrentValue[i] = 0;
  }
  pos = 65;
  Number_Motor = (short int)EEPROM.read(3);
  for (int i = 0; i < Number_Motor; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if ( tmp != 0) {
        MotorNameDataBase[i][j] = (char)tmp;
        MotorNoSortBase[i][j] = (char)tmp;
        j++;
      } else {
        MotorNameDataBase[i][j] = '\0';
        MotorNoSortBase[i][j] = '\0';
        break;
      }
    }
    MotorPinDataBase[i] = (short int)EEPROM.read(pos * 16 + strlen(MotorNameDataBase[i]) + 1);
    MotorNeedValue[i] = 0;
    MotorCurrentValue[i] = 0;
    for (int j = MotorPinDataBase[i]; j < MotorPinDataBase[i] + 4; j++) {
      ReservePin[j] = j;
    }
    pos++;
  }
  pos = 79;
  for (int i = 0; i < 64; i++) {
    OutputNameDataBase[i][0] = '\0';
    OutputPinDataBase[i] = 128;
    OutputValueDataBase[i] = 0;
  }
  Number_Output = (short int)EEPROM.read(4);
  for (int i = 0; i < Number_Output; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if ( tmp != 0) {
        OutputNameDataBase[i][j] = (char)tmp;
        j++;
      } else {
        OutputNameDataBase[i][j] = '\0';
        break;
      }
    }
    OutputPinDataBase[i] = (short int)EEPROM.read(pos * 16 + strlen(OutputNameDataBase[i]) + 1);
    OutputValueDataBase[i] = 0;
    //pinMode(OutputDataBase[i].pin, OUTPUT);
    //OutputPin pin(OutputDataBase[i].pin);
    ReservePin[OutputPinDataBase[i]] = OutputPinDataBase[i];
    pos++;
  }

  pos = 143;
  for (int i = 0; i < 16; i++) {
    PotNameDataBase[i][0] = '\0';
    PotPinDataBase[i] = 128;
    PotValueDataBase[i] = -1;
  }
  Number_Pot = (short int)EEPROM.read(5);
  for (int i = 0; i < Number_Pot; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if ( tmp != 0) {
        PotNameDataBase[i][j] = (char)tmp;
        j++;
      } else {
        PotNameDataBase[i][j] = '\0';
        break;
      }
    }
    PotPinDataBase[i] = (short int)EEPROM.read(pos * 16 + strlen(PotNameDataBase[i]) + 1);
    ReservePin[PotPinDataBase[i]] = PotPinDataBase[i];
    pos++;
  }

  pos = 159;
  for (int i = 0; i < 8; i++) {
    ServoNameDataBase[i][0] = '\0';
    ServoPinDataBase[i] = 128;
    ServoValueDataBase[i] = -1;
  }
  Number_Servo = (short int)EEPROM.read(6);
  for (int i = 0; i < Number_Servo; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if ( tmp != 0) {
        ServoNameDataBase[i][j] = (char)tmp;
        j++;
      } else {
        ServoNameDataBase[i][j] = '\0';
        break;
      }
    }
    ServoPinDataBase[i] = (short int)EEPROM.read(pos * 16 + strlen(ServoNameDataBase[i]) + 1);
    ServoValueDataBase[i] = 0;
    ReservePin[ServoPinDataBase[i]] = ServoPinDataBase[i];
    pos++;
  }

  pos = 167;
  for (int i = 0; i < 9; i++) {
    MpbNameDataBase[i][0] = '\0';
    MpbButtonCountBase[i] = 0;
  }
  Number_Mpb = (short int)EEPROM.read(7);
  for (int i = 0; i < Number_Mpb; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if ( tmp != 0) {
        MpbNameDataBase[i][j] = (char)tmp;
        j++;
      } else {
        MpbNameDataBase[i][j] = '\0';
        break;
      }
    }
    short int count_of_Buttons = 0;
    for (int k = 0; k < Number_Button; k++) {
      if (strcmp(MpbNameDataBase[i], ButtonNameDataBase[k]) == 0) {
        count_of_Buttons++;
      }
    }
    MpbButtonCountBase[i] = count_of_Buttons;
    MpbPinDataBase[i] = new short int[MpbButtonCountBase[i]];
    int z = 0;
    for (int k = 0; k < Number_Button; k++) {
      if (strcmp(MpbNameDataBase[i], ButtonNameDataBase[k]) == 0) {
        MpbPinDataBase[i][z] = ButtonPinDataBase[k];
        z++;
        /*Маркер, который обозначает, что данный объект это часть
          галетного переключателя. При составлении пакета при -g пропускается по этому
          маркеру*/
        ButtonPinDataBase[k] = 250;
      }
    }
    pos++;
  }
  pos = 176;
  for (int i = 0; i < Number_Motor; i++) {
    MotorSensorPolarity[i] = EEPROM.read(pos * 16 + i);
  }
  for (int i = 0; i < Number_Motor; i++) {
    MotorType[i] = EEPROM.read(pos * 16 + 14 + i);
    if (MotorType[i]) {
      ReservePin[MotorPinDataBase[i] + 4] = MotorPinDataBase[i] + 4;//ResrvePin for zero sensor from EEPROM
    }
  }
  int k = 0;
  short int byte_long_dec_temp;
  byte four_byte_number_bin[32];
  for (int i = 0; i < Number_Motor; i++) {
    for (int h = 0 ; h < 32; h++) {
      four_byte_number_bin[h] = 0;
    }
    k = 7;
    for (int h = 0; h < 4; h++) {
      k = (h + 1) * 8 - 1;
      byte_long_dec_temp = EEPROM.read(pos * 16 + 28 + (i * 4) + h);
      while (byte_long_dec_temp > 1) {
        four_byte_number_bin[k] = byte_long_dec_temp  % 2;
        byte_long_dec_temp /= 2;
        k--;
      }
      four_byte_number_bin[k] = byte_long_dec_temp;
    }
    for (int j = 0; j < 32; j++ ) {
      MotorRange[i] = MotorRange[i] + four_byte_number_bin[j] * pow_2(31 - j);
    }
  }
  for (int i = 0; i < Number_Motor; i++) {
    for (int h = 0 ; h < 32; h++) {
      four_byte_number_bin[h] = 0;
    }
    k = 7;
    for (int h = 0; h < 4; h++) {
      k = (h + 1) * 8 - 1;
      byte_long_dec_temp = EEPROM.read(pos * 16 + 84 + (i * 4) + h);
      while (byte_long_dec_temp > 1) {
        four_byte_number_bin[k] = byte_long_dec_temp  % 2;
        byte_long_dec_temp /= 2;
        k--;
      }
      //k = ((3 - h) * 8) - 1;
      four_byte_number_bin[k] = byte_long_dec_temp;
    }
    for ( int j = 1; j < 32; j++ ) {
      MotorNullCorrection[i] = MotorNullCorrection[i] + four_byte_number_bin[j] * pow_2(31 - j);
    }
    if (four_byte_number_bin[0]) {
      MotorNullCorrection[i] = 0 - MotorNullCorrection[i];
    }

  }
  pos = 185;
  /*pos = 167;
    for (int i = 0; i < 9; i++) {
    EncoderNameDataBase[i][0] = '\0';
    EncoderPinDataBase[i] = 128;
    EncoderValueDataBase[i] = 0;
    }*/
  /*for (int i = 0; i < Number_Encoder; i++) {
    int j = 0;
    for (;;) {
      int tmp = EEPROM.read(pos * 16 + j);
      if (tmp != 0) {
        EncoderNameDataBase[i][j] = (char)tmp;
        j++;
      } else {
        EncoderNameDataBase[i][j] = '\0';
        break;
      }
    }
    EncoderPinDataBase[i] = (short int)EEPROM.read(pos * 16 + strlen(EncoderNameDataBase[i]) + 1);
    ReservePin[EncoderPinDataBase[i]] = EncoderPinDataBase[i];
    ReservePin[EncoderPinDataBase[i] + 1] = EncoderPinDataBase[i] + 1;
    pos++;
    }*/
  //Check DataBase CalibRation
  //TODO: check task ХУЙ
  //WARNING! Стартовая калибровка приборов
  DataBaseNameSort(OutputNameDataBase, OutputPinDataBase, 0, Number_Output - 1);
  DataBaseMotorSort(MotorNameDataBase, MotorPinDataBase, 0, Number_Motor - 1);
  DataBaseNameSort(ButtonNameDataBase, ButtonPinDataBase, 0, Number_Button - 1);
  StartCalibration();
  //Check and recive soft delay and IP
  //If they changed in last work
  //EEPROM.read(9) - change system info?
  soft_delay = 64;
  ch_system_info = EEPROM.read(9);
  if (ch_system_info) {
    for (int i = 0 ; i < 4; i++) {
      ip_data[i] = EEPROM.read(pos * 16 + i);
    }
    soft_delay = EEPROM.read(pos * 16 + 4);
  }
  IPAddress ip(ip_data[0], ip_data[1], ip_data[2], ip_data[3]);
  Ethernet.begin(mac, ip);
  Udp_Core.begin(localPort);
  wdt_enable(WDTO_2S);
}

/*Realoaded str function for merge*/
String MakeStr(String main, String tmp) {
  main = main + tmp;
  return main;
}

String MakeStr(String main, int tmp) {
  main = main + String(tmp, DEC);
  return main;
}

String MakeStr(String main, char tmp[]) {
  main = main + String(tmp);
  return main;
}


/*Work Done
  Need_to_check_Ram*/
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
void Exam_connect_device(EthernetUDP Udp) {
  if (Number_of_all_Device == 0) {
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write("-c No saved device");
    Udp.endPacket();
    return;
  }
  String System_data = ""; // переминовать в system data
  short int Summ_position = 0;
  short int All_Summ_device = 0;
  for (int i = 0; i < Number_Mpb; i++) {
    Summ_position += MpbButtonCountBase[i];
  }
  All_Summ_device = (Number_of_all_Device + Number_Mpb);
  All_Summ_device = All_Summ_device - Summ_position;
  System_data = System_data + "-c ";
  System_data = System_data + String(All_Summ_device, DEC);
  System_data = System_data + ", ";
  short int length_buff_data = 0;
  length_buff_data += System_data.length();
  for (int i = 0; i < Number_Button; i++) {
    length_buff_data += strlen(ButtonNameDataBase[i]);
    length_buff_data += strlen("=button,");
    length_buff_data += strlen(ButtonPinDataBase[i]);
    length_buff_data += strlen(",");
  }
  for (int i = 0; i < Number_Mpb; i++) {
    length_buff_data += strlen(MpbNameDataBase[i]);
    length_buff_data += strlen("=MultiPositionSwitch,");
  }
  /*for (int i = 0; i < Number_Encoder; i++) {
    length_buff_data += strlen(EncoderNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("Encoder");
    length_buff_data += strlen(",");
    }*/
  for (int i = 0; i < Number_Motor; i++) {
    length_buff_data += strlen(MotorNameDataBase[i]);
    length_buff_data += strlen("=Motor,");
    length_buff_data += strlen("START_PIN=");
    length_buff_data += strlen(MotorPinDataBase[i]);
    length_buff_data += strlen(",");
  }
  for (int i = 0; i < Number_Output; i++) {
    length_buff_data += strlen(OutputNameDataBase[i]);
    length_buff_data += strlen("=Output,");
    length_buff_data += strlen(OutputPinDataBase[i]);
    length_buff_data += strlen(",");
  }
  for (int i = 0; i < Number_Pot; i++) {
    length_buff_data += strlen(PotNameDataBase[i]);
    length_buff_data += strlen("=Pot,");
    length_buff_data += strlen(PotPinDataBase[i]);
    length_buff_data += strlen(",");
  }
  char *Udp_buffer_packet_for_data_list;
  Udp_buffer_packet_for_data_list = new char[length_buff_data];
  for (int i = 0; i < length_buff_data; i++) {
    Udp_buffer_packet_for_data_list[i] = '\0';
  }
  System_data.toCharArray(Udp_buffer_packet_for_data_list, System_data.length());
  for (int i = 0; i < Number_Button; i++) {
    if (ButtonPinDataBase[i] == 250  ) {
      continue;
    }
    strcat(Udp_buffer_packet_for_data_list, ButtonNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=Button,");
    char val[3] = {((int)ButtonPinDataBase[i]) / 10 + 48, ((int)ButtonPinDataBase[i]) % 10 + 48, 0};
    strcat(Udp_buffer_packet_for_data_list, val);
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  for (int i = 0; i < Number_Mpb; i++) {
    strcat(Udp_buffer_packet_for_data_list, MpbNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=MultiPositionSwitch,");
  }
  /*for (int i = 0; i < Number_Encoder; i++) {
    strcat(Udp_buffer_packet_for_data_list, EncoderNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "Encoder");
    strcat(Udp_buffer_packet_for_data_list, ",");
    }*/
  for (int i = 0; i < Number_Motor; i++) {
    strcat(Udp_buffer_packet_for_data_list, MotorNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=Motor,");
    strcat(Udp_buffer_packet_for_data_list, "START_PIN=");
    char val[3] = {((int)MotorPinDataBase[i]) / 10 + 48, ((int)MotorPinDataBase[i]) % 10 + 48, 0};
    strcat(Udp_buffer_packet_for_data_list, val);
    strcat(Udp_buffer_packet_for_data_list, ",");

  }
  for (int i = 0; i < Number_Output; i++) {
    strcat(Udp_buffer_packet_for_data_list, OutputNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=Output,");
    char val[3] = {((int)OutputPinDataBase[i]) / 10 + 48, ((int)OutputPinDataBase[i]) % 10 + 48, 0};
    strcat(Udp_buffer_packet_for_data_list, val);
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  for (int i = 0; i < Number_Pot; i++) {
    strcat(Udp_buffer_packet_for_data_list, PotNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=Pot,");
    char val[3] = {((int)PotPinDataBase[i]) / 10 + 48, ((int)PotPinDataBase[i]) % 10 + 48, 0};
    strcat(Udp_buffer_packet_for_data_list, val);
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write(Udp_buffer_packet_for_data_list);
  Udp.endPacket();
  delete[] Udp_buffer_packet_for_data_list;
}
/*Work Done*/
void Ping_status(EthernetUDP Udp) {
  Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
  Udp.write("-p Done");
  Udp.endPacket();
}

//UDP:-g\\n
void Show_device(EthernetUDP Udp) {
  if (Number_of_all_Device == 0) {
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write("-c No saved device");
    Udp.endPacket();
    return;
  }
  String System_data = ""; // переминовать в system data
  short int Summ_position = 0;
  short int All_Summ_device = 0;
  for (int i = 0; i < Number_Mpb; i++) {
    Summ_position += MpbButtonCountBase[i];
  }
  All_Summ_device = (Number_of_all_Device + Number_Mpb);
  All_Summ_device = All_Summ_device - Summ_position;
  System_data = System_data + "-c ";
  System_data = System_data + String(All_Summ_device, DEC);
  System_data = System_data + ", ";
  short int length_buff_data = 0;
  length_buff_data += System_data.length();
  for (int i = 0; i < Number_Button; i++) {
    length_buff_data += strlen(ButtonNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("button");
    length_buff_data += strlen(",");
  }
  for (int i = 0; i < Number_Mpb; i++) {
    length_buff_data += strlen(MpbNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("MultiPositionSwitch");
    length_buff_data += strlen(",");
  }
  /*for (int i = 0; i < Number_Encoder; i++) {
    length_buff_data += strlen(EncoderNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("Encoder");
    length_buff_data += strlen(",");
    }*/
  for (int i = 0; i < Number_Motor; i++) {
    length_buff_data += strlen(MotorNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("Motor");
    length_buff_data += strlen(",");
  }
  for (int i = 0; i < Number_Output; i++) {
    length_buff_data += strlen(OutputNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("Output");
    length_buff_data += strlen(",");
  }
  for (int i = 0; i < Number_Pot; i++) {
    length_buff_data += strlen(PotNameDataBase[i]);
    length_buff_data += strlen("=");
    length_buff_data += strlen("Pot");
    length_buff_data += strlen(",");
  }
  char *Udp_buffer_packet_for_data_list;
  Udp_buffer_packet_for_data_list = new char[length_buff_data];
  for (int i = 0; i < length_buff_data; i++) {
    Udp_buffer_packet_for_data_list[i] = '\0';
  }
  System_data.toCharArray(Udp_buffer_packet_for_data_list, System_data.length());
  for (int i = 0; i < Number_Button; i++) {
    if (ButtonPinDataBase[i] == 250  ) {
      continue;
    }
    strcat(Udp_buffer_packet_for_data_list, ButtonNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "Button");
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  for (int i = 0; i < Number_Mpb; i++) {
    strcat(Udp_buffer_packet_for_data_list, MpbNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "MultiPositionSwitch");
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  /*for (int i = 0; i < Number_Encoder; i++) {
    strcat(Udp_buffer_packet_for_data_list, EncoderNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "Encoder");
    strcat(Udp_buffer_packet_for_data_list, ",");
    }*/
  for (int i = 0; i < Number_Motor; i++) {
    strcat(Udp_buffer_packet_for_data_list, MotorNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "Motor");
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  for (int i = 0; i < Number_Output; i++) {
    strcat(Udp_buffer_packet_for_data_list, OutputNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "Output");
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  for (int i = 0; i < Number_Pot; i++) {
    strcat(Udp_buffer_packet_for_data_list, PotNameDataBase[i]);
    strcat(Udp_buffer_packet_for_data_list, "=");
    strcat(Udp_buffer_packet_for_data_list, "Pot");
    strcat(Udp_buffer_packet_for_data_list, ",");
  }
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write(Udp_buffer_packet_for_data_list);
  Udp.endPacket();
  delete[] Udp_buffer_packet_for_data_list;
}

/*Add device to DataBase
  -a type,name,pin; Complete*/

//Поставить защиту от большего имени(больше 16 символов) и правильности ввода пина.
//Сделать таблицу ошибок
void add_device(String CommandString, EthernetUDP Udp) {
  /*Check Type of enter Device*/
  String _buffer = "";
  short int i = 2;
  short int k = 0;
  while (CommandString[i] != ',') {
    _buffer += CommandString[i++];
    k++;
  }
  /*Button add*/
  if (_buffer == "Button") {
    Number_of_all_Device++;
    Number_Button++;
    i++;
    k = 0;
    _buffer = "";
    while (CommandString[i] != ',') {
      _buffer += CommandString[i++];
      k++;
    }
    for (int j = 0; j < k; j++) {
      ButtonNameDataBase[Number_Button - 1][j] =  char(_buffer[j]);
    }
    i++;
    int tmp_val = 0;
    while (CommandString[i] != '\0') {
      short int buff = CommandString[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      }
      i++;
    }
    bool status_of_check =  CheckDontTouchPins(tmp_val, Udp);;
    bool status_of_write_pin = CheckAndEnterReservePins(tmp_val, Udp);
    if ((!status_of_check) && (!status_of_write_pin)) {
      ButtonPinDataBase[Number_Button - 1] = tmp_val;
      ReservePin[tmp_val] = tmp_val;
      EEPROM.write(1, Number_of_all_Device);
      EEPROM.write(2, Number_Button);
      Write_Memory_to_EEPROM(ButtonNameDataBase[Number_Button - 1], ButtonPinDataBase[Number_Button - 1], Number_Button);
      DataBaseNameSort(ButtonNameDataBase, ButtonPinDataBase, 0, Number_Button - 1);
      //pinMode(tmp_val, INPUT_PULLUP);
      //MyPinMode(tmp_val, 2);
    } else {
      Number_of_all_Device--;
      ButtonNameDataBase[Number_Button - 1][0] = '\0';
      Number_Button--;
    }
    /*Motor add*/
  } else if (_buffer == "Motor") {
    Number_of_all_Device++;
    Number_Motor++;
    i++;
    k = 0;
    _buffer = "";
    while (CommandString[i] != ',') {
      _buffer += CommandString[i++];
      k++;
    }
    for (int j = 0; j < k; j++) {
      MotorNameDataBase[Number_Motor - 1][j] =  (char)_buffer[j];
      MotorNoSortBase[Number_Motor - 1][j] = (char)_buffer[j];
    }
    i++;
    int tmp_val = 0;
    while (CommandString[i] != '\0') {
      short int buff = CommandString[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      }
      i++;
    }
    bool status_of_check = CheckDontTouchPinsForMotor(tmp_val, Udp);
    bool status_of_write_pin = CheckAndEnterReservePinsForMotor(tmp_val, Udp);
    if ((!status_of_check) && (!status_of_write_pin)) {
      MotorPinDataBase[Number_Motor - 1] = tmp_val;
      for (int i = tmp_val; i < tmp_val + 4; i++) {
        ReservePin[i] = i; // Reserve pins for stepper motor with 4 wires connection.
      }
      EEPROM.write(1, Number_of_all_Device);
      EEPROM.write(3, Number_Motor);
      Write_Memory_to_EEPROM(MotorNameDataBase[Number_Motor - 1], MotorPinDataBase[Number_Motor - 1], Number_Motor + 64);
      DataBaseMotorSort(MotorNameDataBase, MotorPinDataBase, 0, Number_Motor - 1);
    } else {
      Number_of_all_Device--;
      MotorNameDataBase[Number_Motor - 1][0] = '\0';
      MotorNoSortBase[Number_Motor - 1][0] = '\0';
      Number_Motor--;
    }
    /*Output add*/
  } else if (_buffer == "Output") {
    Number_of_all_Device++;
    Number_Output++;
    i++;
    k = 0;
    _buffer = "";
    while (CommandString[i] != ',') {
      _buffer += CommandString[i++];
      k++;
    }
    for (int j = 0; j < k; j++) {
      OutputNameDataBase[Number_Output - 1][j] = (char)_buffer[j];
    }
    i++;
    int tmp_val = 0;
    while (CommandString[i] != '\0') {
      short int buff = CommandString[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      }
      i++;
    }
    bool status_of_check = CheckDontTouchPins(tmp_val, Udp);
    bool status_of_write_pin = CheckAndEnterReservePins(tmp_val, Udp);
    if ((!status_of_check) && (!status_of_write_pin)) {
      OutputPinDataBase[Number_Output - 1] = tmp_val;
      ReservePin[tmp_val] = tmp_val;
      //pinMode(tmp_val, OUTPUT);
      //MyPinMode(tmp_val, 1);
      EEPROM.write(1, Number_of_all_Device);
      EEPROM.write(4, Number_Output);
      Write_Memory_to_EEPROM(OutputNameDataBase[Number_Output - 1], OutputPinDataBase[Number_Output - 1], Number_Output + 78);
      DataBaseNameSort(OutputNameDataBase, OutputPinDataBase, 0, Number_Output - 1);
    } else {
      Number_of_all_Device--;
      OutputNameDataBase[Number_Output - 1][0] = '\0';
      Number_Output--;
    }
  } else if (_buffer == "Pot" ) {
    Number_of_all_Device++;
    Number_Pot++;
    i++;
    k = 0;
    _buffer = "";
    while (CommandString[i] != ',') {
      _buffer += CommandString[i++];
      k++;
    }
    for (int j = 0; j < k; j++) {
      PotNameDataBase[Number_Pot - 1][j] = (char)_buffer[j];
    }
    i++;
    int tmp_val = 0;
    while (CommandString[i] != '\0') {
      short int buff = CommandString[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      }
      i++;
    }
    bool status_of_write_pin = CheckAnalogPinsForWrite(tmp_val, Udp);
    if (!status_of_write_pin) {
      PotPinDataBase[Number_Pot - 1] = tmp_val + 54;
      ReservePin[tmp_val + 54] = tmp_val + 54;
      EEPROM.write(1, Number_of_all_Device);
      EEPROM.write(5, Number_Pot);
      Write_Memory_to_EEPROM(PotNameDataBase[Number_Pot - 1], PotPinDataBase[Number_Pot - 1], Number_Pot + 142);
    } else {
      Number_of_all_Device--;
      PotNameDataBase[Number_Pot - 1][0] = '\0';
      Number_Pot--;
    }
  } else if (_buffer == "MultiPositionButton") {
    //++++++++
    Number_Mpb++;
    i++;
    k = 0;
    _buffer = "";
    while (CommandString[i] != '\0') {
      _buffer += CommandString[i++];
      k++;
    }
    for (int j = 0; j < k; j++) {
      MpbNameDataBase[Number_Mpb - 1][j] = (char)_buffer[j];
    }
    i++;
    short int Button_count = 0;
    for (int z = 0; z < Number_Button; z++) {
      if (strcmp(MpbNameDataBase[Number_Mpb - 1], ButtonNameDataBase[z]) == 0) {
        Button_count++;
      }
    }
    if (Button_count) {
      EEPROM.write(7, Number_Mpb);//++++

      Write_Name_Memory_to_EEPROM(MpbNameDataBase[Number_Mpb - 1], Number_Mpb + 166);

      MpbButtonCountBase[Number_Mpb - 1] = Button_count;
      MpbPinDataBase[Number_Mpb - 1] = new short int [MpbButtonCountBase[Number_Mpb - 1]];
      int g = 0;
      for (int z = 0; z < Number_Button; z++) {
        if (strcmp(MpbNameDataBase[Number_Mpb - 1], ButtonNameDataBase[z]) == 0) {
          MpbPinDataBase[Number_Mpb - 1][g] = ButtonPinDataBase[z];
          ButtonPinDataBase[z] = 250;
          g++;
        }
      }
    } else {
      MpbNameDataBase[Number_Mpb - 1][0] = '\0';
      Number_Mpb--;
    }
  } else if (_buffer == "Encoder") {
    return; //Encoder_not_added
    /*Number_of_all_Device++;
      Number_Encoder++;
      i++;
      k = 0;
      _buffer = "";
      /*Programm Block take a name from CommandString*/
    while (CommandString[i] != ',') {
      _buffer += CommandString[i++];
      k++;
    }
    /*One send name to Name_Encoder_Array*/
    for (int j = 0; j < k; j++) {
      EncoderNameDataBase[Number_Encoder - 1][j] =  char(_buffer[j]);
    }
    i++;
    int tmp_val = 0;
    /*Take number of pin from CommandString*/
    while (CommandString[i] != '\0') {
      short int buff = CommandString[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      }
      i++;
    }
    /*One check entering pin for entering in DontTouchPins and reserving interval*/
    bool status_of_check = CheckDontTouchPinsEncoder(tmp_val, Udp);
    bool status_of_write_pin = CheckAndEnterReservePinsEncoder(tmp_val, Udp);
    if ((!status_of_check) && (!status_of_write_pin)) {
      EncoderPinDataBase[Number_Encoder - 1] = tmp_val;
      //Faster than use cycle two times
      ReservePin[tmp_val] = tmp_val;
      ReservePin[tmp_val + 1] = tmp_val + 1;
      EEPROM.write(1, Number_of_all_Device);
      EEPROM.write(8, Number_Encoder);
      Write_Memory_to_EEPROM(EncoderNameDataBase[Number_Encoder - 1], EncoderPinDataBase[Number_Encoder - 1], Number_Encoder + 166);
    } else {
      Number_of_all_Device--;
      EncoderNameDataBase[Number_Encoder - 1][0] = '\0';
      Number_Encoder --;
    }
  } else {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакета
    Udp.write("-a Error 10"); //Error_enter_type
    Udp.endPacket();
  }
}

void Write_Memory_to_EEPROM(char enter_name[], int enter_pin, int enter_pos) {
  if (enter_pos == 0) return;
  int len = strlen(enter_name);
  for (int i = 0; i < len; i++) {
    EEPROM.write(enter_pos * 16 + i, (byte)enter_name[i]);
  }
  EEPROM.write(enter_pos * 16 + len, byte(0));//Записываем конец строки
  EEPROM.write(enter_pos * 16 + len + 1, (byte)enter_pin); //Записываем пин на 17 бит нашей структуры
}
/*Переписать только для имени!!!*/
void Write_Name_Memory_to_EEPROM(char enter_name[], int enter_pos) {
  if (enter_pos == 0) return;
  int len = strlen(enter_name);
  for (int i = 0; i < len; i++) {
    EEPROM.write(enter_pos * 16 + i, (byte)enter_name[i]);
  }
  EEPROM.write(enter_pos * 16 + len, byte(0));//Записываем конец строки
}

/*Pins 0-53 Only Digital. Pins 54-69 It's analog(A0-A15)/Mega*/
/*--------------------------CHECK_DONT_TOUCH_PINS_ANY_TYPE + MOTOR-------------------*/
bool CheckDontTouchPins(int enter_pin, EthernetUDP Udp) {
  for (int i = 0; i < 8; i++) {
    if (DontTouchPins[i] == enter_pin) {
      Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакета
      Udp.write("-a Error 02");
      Udp.endPacket();
      return true;
    }
  }
  return false;
}
bool CheckDontTouchPinsForMotor(int enter_pin, EthernetUDP Udp) {
  for (int u = 0; u < 5; u++) {
    for (int i = 0; i < 8; i++) {
      if (DontTouchPins[i] == enter_pin) {
        Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
        Udp.write("-a Error 02");
        Udp.endPacket();
        return true;
      }
    }
    enter_pin++;
  }
  return false;
}
/*--------------------CHECK_AND_ENTER_PINS_ENCODER-----------------------------*/
bool CheckDontTouchPinsEncoder(int enter_pin, EthernetUDP Udp) {
  for (int u = 0; u < 2; u++) {
    for (int i = 0; i < 8; i++) {
      if (DontTouchPins[i] == enter_pin) {
        Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
        Udp.write("-a Error 02");
        Udp.endPacket();
        return true;
      }
    }
    enter_pin++;
  }
  return false;
}
bool CheckAndEnterReservePinsEncoder(int enter_pin, EthernetUDP Udp) {
  if (enter_pin > 69) {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
    Udp.write("-a Error 06");
    Udp.endPacket();
    return true;
  }
  if ((ReservePin[enter_pin] != -1) || (ReservePin[enter_pin + 1] != -1)) {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
    Udp.write("-a Error 04");
    Udp.endPacket();
    return true;
  }
  return false;
}
/*--------------------CHECK_AND_ENTER_RESERVE_PINS-----------------------------*/
bool CheckAndEnterReservePins(int enter_pin, EthernetUDP Udp) {
  if (enter_pin > 69) {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
    Udp.write("-a Error 06");
    Udp.endPacket();
    return true;
  }
  if (ReservePin[enter_pin] != -1) {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
    Udp.write("-a Error 04");
    Udp.endPacket();
    return true;
  }
  return false;
}

/*DONE*/
bool CheckAndEnterReservePinsForMotor(int enter_pin, EthernetUDP Udp) {
  if (enter_pin > 65) {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакета
    Udp.write("-a Error 05");
    Udp.endPacket();
    return true;
  }
  for (int i = enter_pin; i < enter_pin + 4; i++) {
    if (ReservePin[i] != -1) {
      Udp.beginPacket(Udp.remoteIP(), 10002);
      Udp.write("-a Error 03");
      Udp.endPacket();
      return true;
    }
  }
  return false;
}
/*---------------------CheckAnalogPinsForWrite---------------------*/
bool CheckAnalogPinsForWrite(int enter_pin, EthernetUDP Udp) {
  if ((enter_pin < 0) || (enter_pin > 15)) {
    //TelCli.println("Error 07"); // This pin isn't in board analog pins
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
    Udp.write("-a Error 07");
    Udp.endPacket();
    return true;
  }
  //Check second part of massive with analog Device
  if (ReservePin[enter_pin + 54] == enter_pin + 54) {
    // TelCli.println("Error 04"); //This pin reserve from another Pin
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакет
    Udp.write("-a Error 04");
    Udp.endPacket();
    return true;
  }
  return false;
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*-d _name. DONE*/
void delete_device(String CommandString, EthernetUDP Udp) {
  if (Number_of_all_Device == 0) {
    Udp.beginPacket(Udp.remoteIP(), 10002);//Начало пакете
    Udp.write("-d No data");
    Udp.endPacket();
    return;
  }
  short int m = 2;
  char buffer_name[16] = "\0";
  while (CommandString[m] != '\0') {
    buffer_name[m - 2] = (char)CommandString[m];
    m++;
  }
  buffer_name[m] = '\0';
  for (int i = 0; i < Number_Button; i++) {
    if (strcmp(ButtonNameDataBase[i], buffer_name) == 0) {
      //pinMode(ButtonDataBase[i].pin, INPUT);
      //MyPinMode(ButtonDataBase[i].pin, 0);
      ReservePin[ButtonPinDataBase[i]] = -1;
      for (int j = i; j < Number_Button; j++) {
        //ButtonNameDataBase[j] = ButtonNameDataBase[j + 1];
        strcpy(ButtonNameDataBase[j], ButtonNameDataBase[j + 1]);
        ButtonPinDataBase[j] = ButtonPinDataBase[j + 1];
      }
      ButtonNameDataBase[Number_Button][0] = '\0';
      ButtonPinDataBase[Number_Button] = 128;
      Number_Button--;
      Number_of_all_Device--;
      return;
    }
  }
  for (int i = 0; i < Number_Pot; i++) {
    if (strcmp(PotNameDataBase[i], buffer_name) == 0) {
      //pinMode(PotDataBase[i].pin, INPUT);
      //MyPinMode(PotDataBase[i].pin, 0);
      ReservePin[PotPinDataBase[i]] = -1;
      for (int j = i; j < Number_Pot; j++) {
        //PotNameDataBase[j] = PotNameDataBase[j + 1];
        strcpy(PotNameDataBase[j], PotNameDataBase[j + 1]);
        PotPinDataBase[j] = PotPinDataBase[j + 1];
      }
      PotNameDataBase[Number_Pot][0] = '\0';
      PotPinDataBase[Number_Pot] = 128;
      Number_Pot--;
      Number_of_all_Device--;
      return;
    }
  }
  for (int i = 0; i < Number_Output; i++) {
    if (strcmp(OutputNameDataBase[i], buffer_name) == 0) {
      //pinMode(OutputDataBase[i].pin, INPUT);
      //MyPinMode(OutputDataBase[i].pin, 0);
      ReservePin[OutputPinDataBase[i]] = -1;
      for (int j = i; j < Number_Output; j++) {
        //OutputNameDataBase[j] = OutputNameDataBase[j + 1];
        strcpy(OutputNameDataBase[j], OutputNameDataBase[j + 1]);
        OutputPinDataBase[j] = OutputPinDataBase[j + 1];
      }
      OutputNameDataBase[Number_Output][0] = '\0';
      OutputPinDataBase[Number_Output] = 128;
      Number_Output--;
      Number_of_all_Device--;
      return;
    }
  }
  for (int i = 0; i < Number_Motor; i++) {
    if (strcmp(MotorNameDataBase[i], buffer_name) == 0) {
      for (int j = MotorPinDataBase[i]; j < MotorPinDataBase[i] + 5; j++) {
        //pinMode(ReservePin[j], INPUT);
        //MyPinMode(ReservePin[j], 0);
        ReservePin[j] = -1;
      }
      for (int j = i; j < Number_Motor; j++) {
        //MotorNameDataBase[j] = MotorNameDataBase[j + 1];
        strcpy(MotorNameDataBase[j], MotorNameDataBase[j + 1]);
        MotorPinDataBase[j] = MotorPinDataBase[j + 1];
      }
      MotorNameDataBase[Number_Motor][0] = '\0';
      MotorPinDataBase[Number_Motor] = 128;
      Number_Motor--;
      Number_of_all_Device--;
      return;
    }
  }
}
/*Work DONE*/
void Give_signal(EthernetUDP Udp) {
  if (!(Number_of_all_Device == 0)) {
    String data = "";
    char *Udp_buffer_packet_for_data_list = 0;
    data = MakeStr(data, "-g ");
    for (int i = 0; i < Number_Button; i++) {
      if (ButtonPinDataBase[i] == 250) {
        continue;
      }
      InputPin mi_8(ButtonPinDataBase[i], true);
      char tmp[21] = "";
      strcpy(tmp, ButtonNameDataBase[i]);
      strcat(tmp, "=");
      char val[2] = {(int)inverse_signal(mi_8.read()) + 48, 0};//have 0 or 1
      strcat(tmp, val);
      strcat(tmp, ",");
      data += tmp;
    }
    //If we not have high voltage on Mpb, return 1 (new add)
    for (int i = 0; i < Number_Mpb; i++) {
      bool low_voltage = true;
      for (int j = 0; j < MpbButtonCountBase[i]; j++) {
        InputPin mi_8(MpbPinDataBase[i][j], true);
        if (inverse_signal(mi_8.read()) == 1) {
          char tmp[21] = "";
          strcpy(tmp, MpbNameDataBase[i]);
          strcat(tmp, "=");
          //char val[2] = {(int)j * 2 + 48, 0};
          char val[3] = {((int)j * 2) / 10 + 48, ((int)j * 2) % 10 + 48, 0};
          strcat(tmp, val);
          strcat(tmp, ",");
          data += tmp;
          low_voltage = false;
          break;
        }
      }
      if (low_voltage) {
        char tmp[21] = "";
        strcpy(tmp, MpbNameDataBase[i]);
        strcat(tmp, "=");
        strcat(tmp, "1");
        strcat(tmp, ",");
        data += tmp;
      }
    }
    /*for (int i = 0; i < Number_Encoder; i++) {
      char tmp[21] = "";
      Serial.println(EncoderNameDataBase[i]);
      Serial.println(EncoderValueDataBase[i]);
      strcpy(tmp, EncoderNameDataBase[i]);
      strcat(tmp, "=");
      char val[4] = {EncoderValueDataBase[i] / 100 + 48, (EncoderValueDataBase[i] % 100) / 10 + 48, EncoderValueDataBase[i] % 10 + 48, 0};
      strcat(tmp, val);
      strcat(tmp, ",");
      data += tmp;
      EncoderValueDataBase[i] = 0;
      }*/
    for (int i = 0; i < Number_Pot; i++) {
      char tmp[21] = "";
      strcpy(tmp, PotNameDataBase[i]);
      strcat(tmp, "=");
      short int avalue = analogRead(PotPinDataBase[i]);
      char val[5] = {avalue / 1000 + 48, (avalue % 1000) / 100 + 48, (avalue % 100) / 10 + 48, avalue % 10 + 48, 0};
      strcat(tmp, val);
      strcat(tmp, ",");
      data += tmp;
    }
    for (int i = 0; i < Number_Output; i++) {
      char tmp[21] = "";
      strcpy(tmp, OutputNameDataBase[i]);
      strcat(tmp, "=");
      char val[4] = {((int)OutputValueDataBase[i] % 1000) / 100 + 48, ((int)OutputValueDataBase[i] % 100) / 10 + 48, (int)OutputValueDataBase[i] % 10 + 48, 0};
      strcat(tmp, val);
      strcat(tmp, ",");
      data += tmp;
    }
    for (int i = 0; i < Number_Motor; i++) {
      char tmp[21] = "";
      strcpy(tmp, MotorNameDataBase[i]);
      strcat(tmp, "=");
      //char val[5] = {MotorCurrentValue[i] / 1000 + 48, (MotorCurrentValue[i] % 1000) / 100 + 48, (MotorCurrentValue[i] % 100) / 10 + 48, MotorCurrentValue[i] % 10 + 48, 0};
      char val[12];
      for (int i = 0; i < 11; i++) {
        val[i] = '0';
      }
      val[11] = '\0';
      int32_t temp_val = MotorCurrentValue[i];
      bool minus = false;
      byte m = 10;
      if (temp_val < 0) {
        minus = true;
        temp_val = 0 - temp_val;
      }
      while (temp_val >= 1) {
        val[m] = temp_val % 10 + 48;
        temp_val = temp_val / 10;
        m--;
      }
      if (minus) {
        val[0] = '-';
      }
      strcat(tmp, val);
      strcat(tmp, ",");
      data += tmp;
    }
    data = MakeStr(data, ",");
    Udp_buffer_packet_for_data_list = new char[data.length()];
    data.toCharArray(Udp_buffer_packet_for_data_list, data.length());
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write(Udp_buffer_packet_for_data_list);
    Udp.endPacket();
    delete[] Udp_buffer_packet_for_data_list;
  } else {
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write("-g No data for give");
    Udp.endPacket();
    return;
  }
}

uint8_t inverse_signal(int _signal) {
  if (_signal) return 0;
  return 1;
}

void Check_Device() {
  /*for (int i = 0; i < Number_Encoder; i++) {
    }*/
  wdt_disable();
  if (motor_delay != soft_delay) {
    motor_delay ++;
    wdt_enable(WDTO_1S);
    return;
  }
  motor_delay = 0;
  for (int i = 0; i < Number_Motor; i++) {
    MotorAction(MotorNeedValue[i], i);
  }
  wdt_enable(WDTO_1S);
}

void Locating_pins(EthernetUDP Udp) {
  bool check_Command = false;
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("Enter to debug and Locating pins. Enter -1\n to find buttons, -2\n to find output, -3\n Check_lamps, -4\n for test Motor");
  Udp.endPacket();
  while (!check_Command) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      String Udp_Locating_Command = "";
      char tmp = '0';
      for (int i = 0; i < packetSize; i++) {
        tmp = (char)Udp.read();
        switch (tmp) {
          case '-':
            continue;
            break;
          case '\n':
            CommandLocating_execution(Udp_Locating_Command, Udp);
            Udp_Locating_Command = "";
            Udp.flush();
            check_Command = true;
            break;
          default:
            Udp_Locating_Command += tmp;
        }
      }
    }
  }
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("End Locating.");
  Udp.endPacket();
}

void CommandLocating_execution(String Enter_Command_For_Locating, EthernetUDP Udp_Locating) {
  switch (Enter_Command_For_Locating[0]) {
    case '1':
      Locating_Buttons(Udp_Locating);
      break;
    case '2':
      Locating_Output(Udp_Locating);
      break;
    case '4':
      Locating_Motors(Udp_Locating);
      break;
    default:
      Udp_Locating.beginPacket(Udp_Locating.remoteIP(), 10002);
      Udp_Locating.write("Wrong locating command");
      Udp_Locating.endPacket();
      break;
  }
}

void Locating_Buttons(EthernetUDP Udp_Loc) {
  Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
  Udp_Loc.write("Enter start and finish pin. Format: -Start pin,End pin\n");
  Udp_Loc.endPacket();
  //+++++
  bool enter_interval = false;
  short int start_pin = -1;
  short int end_pin = -1;
  String Data_Pin = "";
  while (!enter_interval) {
    int packetSize = Udp_Loc.parsePacket();
    char tmp = '0';
    for (int i = 0 ; i < packetSize; i++) {
      tmp = (char)Udp_Loc.read();
      switch (tmp) {
        case '-':
          continue;
          break;
        case '\n':
          enter_interval = true;
          break;
        default:
          Data_Pin += tmp;
      }
    }
  }
  //++++
  int i = 0;
  short int tmp_val = 0;
  while (Data_Pin[i] != '\0') {
    if (Data_Pin[i] != ',') {
      short int buff = Data_Pin[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
        i++;
      }
    } else {
      start_pin = tmp_val;
      tmp_val = 0;
      i++;
      continue;
    }
  }
  end_pin = tmp_val;
  //Check correcting Interval
  if ((start_pin < 0 ) || (start_pin > 69) || (end_pin < 0) || (end_pin > 69) || (end_pin < start_pin)) {
    Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
    Udp_Loc.write("Incorrect interval. Returning to main menu");
    Udp_Loc.endPacket();
    return;
  }
  Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
  Udp_Loc.write("Enter success.Locating on.");
  Udp_Loc.endPacket();
  bool Exit_from_locating_buttons = false;
  short int data[70];
  bool check_DTP = false;

  for (int i = start_pin; i <= end_pin; i++) {
    check_DTP = false;
    for (int j = 0; j < 8; j++) {
      if (i == DontTouchPins[j]) {
        check_DTP = true;
        break;
      }
    }
    if (!check_DTP) {
      InputPin mi_8(i, true);
      data[i] = mi_8.read();
    }
  }
  while (!Exit_from_locating_buttons) {
    int packetSize = Udp_Loc.parsePacket();
    if (packetSize) {
      char tmp = '0';
      String Data_exit = "";
      for (int i = 0; i < packetSize; i++) {
        tmp = (char)Udp_Loc.read();
        switch (tmp) {
          case '-':
            continue;
            break;
          case '\n':
            Exit_from_locating_buttons = true;
            break;
          default:
            Data_exit += tmp;
        }
      }
      switch (Data_exit[0]) {
        case 'e':
          Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
          Udp_Loc.write("-l Returning to main menu.");
          Udp_Loc.endPacket();
          return;
          break;
        default:
          Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
          Udp_Loc.write("-l haha");
          Udp_Loc.endPacket();
          Exit_from_locating_buttons = false;
          break;
      }
    }
    for (int i = start_pin; i <= end_pin; i++) {
      check_DTP = false;
      for (int j = 0; j < 8; j++) {
        if (i == DontTouchPins[j]) {
          check_DTP = true;
          break;
        }
      }
      if (!check_DTP) {
        InputPin mi_8(i, true);
        if (mi_8.read() > data[i]) {
          data[i] = mi_8.read();
          String Loc_data = "";
          Loc_data = MakeStr(Loc_data, "It is: ");
          Loc_data = MakeStr(Loc_data, i);
          Loc_data = MakeStr(Loc_data, ",");
          char *Packet_Loc_Data = new char[Loc_data.length()];
          Loc_data.toCharArray(Packet_Loc_Data, Loc_data.length());
          Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
          Udp_Loc.write(Packet_Loc_Data);
          Udp_Loc.endPacket();
          delete[] Packet_Loc_Data;
        } else if (mi_8.read() < data[i]) {
          data[i] = mi_8.read();
          String Loc_data = "";
          Loc_data = MakeStr(Loc_data, "It is: ");
          Loc_data = MakeStr(Loc_data, i);
          Loc_data = MakeStr(Loc_data, ",");
          char *Packet_Loc_Data = new char[Loc_data.length()];
          Loc_data.toCharArray(Packet_Loc_Data, Loc_data.length());
          Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
          Udp_Loc.write(Packet_Loc_Data);
          Udp_Loc.endPacket();
          delete[] Packet_Loc_Data;
        }
      }
    }
  }
}

void Locating_Output(EthernetUDP Udp) {
  bool check_Command = false;
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("Output Locating. Enter -start pin,end pin/n");
  Udp.endPacket();
  bool enter_interval = false;
  short int start_pin = -1;
  short int end_pin = -1;
  String Data_Pin = "";
  while (!enter_interval) {
    int packetSize = Udp.parsePacket();
    char tmp = '0';
    for (int i = 0 ; i < packetSize; i++) {
      tmp = (char)Udp.read();
      switch (tmp) {
        case '-':
          continue;
          break;
        case '\n':
          enter_interval = true;
          break;
        default:
          Data_Pin += tmp;
      }
    }
  }
  //++++
  int i = 0;
  short int tmp_val = 0;
  while (Data_Pin[i] != '\0') {
    if (Data_Pin[i] != ',') {
      short int buff = Data_Pin[i] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
        i++;
      }
    } else {
      start_pin = tmp_val;
      tmp_val = 0;
      i++;
      continue;
    }
  }
  end_pin = tmp_val;
  //Check correcting Interval
  if ((start_pin < 0 ) || (start_pin > 69) || (end_pin < 0) || (end_pin > 69) || (end_pin < start_pin)) {
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write("Incorrect interval. Returning to main menu");
    Udp.endPacket();
    return;
  }
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("Enter success.Locating Output ON.");
  Udp.endPacket();
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("After answering packet enter -n\n for next pin or -e\n for exit");
  Udp.endPacket();
  bool Exit_from_locating_buttons = false;
  bool check_DTP = false;
  for (int i = start_pin; i <= end_pin; i++) {
    check_DTP = false;
    for (int j = 0; j < 8; j++) {
      if (i == DontTouchPins[j]) {
        check_DTP = true;
        break;
      }
    }
    if (!check_DTP) {
      bool Entering_packet = false;
      OutputPin mi_8(i, HIGH);
      String Loc_data = "";
      Loc_data = MakeStr(Loc_data, "It is: ");
      Loc_data = MakeStr(Loc_data, i);
      Loc_data = MakeStr(Loc_data, ",");
      char *Packet_Loc_Data = new char[Loc_data.length()];
      Loc_data.toCharArray(Packet_Loc_Data, Loc_data.length());
      Udp.beginPacket(Udp.remoteIP(), 10002);
      Udp.write(Packet_Loc_Data);
      Udp.endPacket();
      delete[] Packet_Loc_Data;
      while (!Entering_packet) {
        int packetSize = Udp.parsePacket();
        if (packetSize) {
          char tmp = '0';
          String Next_action = "";
          for (int i = 0 ; i < packetSize; i++) {
            tmp = (char)Udp.read();
            if (tmp != '-') {
              Next_action += tmp;
            }
          }
          switch (Next_action[0]) {
            case 'n':
              Entering_packet = true;
              mi_8 = LOW;
              break;
            case 'e':
              mi_8 = LOW;
              Udp.beginPacket(Udp.remoteIP(), 10002);
              Udp.write("-l Returning to main menu");
              Udp.endPacket();
              return;
              break;
            default:
              Udp.beginPacket(Udp.remoteIP(), 10002);
              Udp.write("Error_enter_packet");
              Udp.endPacket();
              break;
          }
        }
      }
    }
  }
}
int LinearSearch(char SearchingName[]) {
  for (uint8_t i = 0; i < Number_Motor; i++) {
    if (strcmp(MotorNoSortBase[i], SearchingName) == 0) {
      return i;
    }
  }
  return -1;
}
void Locating_Motors(EthernetUDP Udp_Loc) {
  Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
  Udp_Loc.write("Enter name of motor. Format: -Name\n");
  Udp_Loc.endPacket();
  // -m 400\n - move rotor
  // -a Range,Angle,
  bool Finish_read_name = false;
  char Enter_Name[16];
  Enter_Name[0] = '\0';
  int char_index = 0;
  while (!Finish_read_name) {
    int packetSize = Udp_Loc.parsePacket();
    char tmp = '0';
    for (int i = 0; i < packetSize; i++) {
      tmp = (char)Udp_Loc.read();
      switch (tmp) {
        case '-':
          continue;
          break;
        case '\n':
          Finish_read_name = true;
          break;
        default:
          Enter_Name[char_index] = tmp;
          char_index++;
      }
    }
  }
  Enter_Name[char_index] = '\0';
  short int sort_founding_index = BinaryNameSearch(MotorNameDataBase, Enter_Name, Number_Motor);
  short int need_founding_index = LinearSearch(Enter_Name);
  if (need_founding_index != -1) {
    Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
    Udp_Loc.write("Motor Found.Ch zero Mode: On or Off");
    Udp_Loc.endPacket();
    bool select_mode = false;
    char Mode_Name[10]; //
    Mode_Name[0] = '\0';
    bool zero_mode = false;
    /*Select zero polarity mode: with or not*/
    while (!select_mode) {
      int packetSize = Udp_Loc.parsePacket();

      //Exp. for refull buffer
      if (packetSize > 10) {
        Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
        Udp_Loc.write("Error 8"); //Refull enter buffer
        Udp_Loc.endPacket();
        return;
      }

      char_index = 0;
      char tmp = '0';
      for (int i = 0; i < packetSize; i++) {
        tmp = (char)Udp_Loc.read();
        switch (tmp) {
          case '-':
            continue;
            break;
          case '\n':
            for (int k = 0; k < strlen(Mode_Name); k++) {
              Mode_Name[k] = toupper(Mode_Name[k]);
            }
            Mode_Name[char_index] = '\0';
            if (strcmp(Mode_Name, "ON") == 0) {
              zero_mode = true;
            } else if (strcmp(Mode_Name, "OFF") == 0) {
              zero_mode = false;
            } else {
              Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
              Udp_Loc.write("Error 11"); //Wrong turn zero mode
              Udp_Loc.endPacket();
              return;
            }
            select_mode = true;
            break;
          default:
            Mode_Name[char_index] = tmp;
            char_index++;
        }
      }
    }
    Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
    Udp_Loc.write("Enter_Command_to_control"); //Wrong turn zero mode
    Udp_Loc.endPacket();
    String enter_command = "";
    bool exit_command = false;
    bool check_sumb = false;
    while (!exit_command) {
      int packetSize = Udp_Loc.parsePacket();
      char tmp = '0';
      for (int i = 0; i < packetSize; i++) {
        tmp = (char)Udp_Loc.read();
        switch (tmp) {
          case '-':
            if (!check_sumb) {
              check_sumb = true;
              continue;
            } else {
              enter_command += tmp;
            }
            break;
          case '\n':
            exit_command = Locating_Motor_control(enter_command, zero_mode , need_founding_index, sort_founding_index, Udp_Loc);
            enter_command = "";
            check_sumb = false;
            break;
          default:
            enter_command += tmp;
            break;
        }
      }
    }
  } else {
    Udp_Loc.beginPacket(Udp_Loc.remoteIP(), 10002);
    Udp_Loc.write("Motor not Found"); // ERROR_TEST_FOUND_NAME!
    Udp_Loc.endPacket();
    return;
  }
}

bool Locating_Motor_control(String Command, bool zero_select, short int index_of_motor, short int sort_index_of_motor, EthernetUDP Loc_Udp) {
  /*Command avaible:
    Before control we need to select mode:
    with zero_sensor or not
    -a Type(Vid,STH),MotorRange(+),MotorNullCorrection(+ -),Polarity/n
    (0,1)(number)(+-number)(0 - reserve pol, 1 - unreserve pol)
    -m Enter_angle(Move Motor on enter angle + - )/n
    -z Find(for STH and Some VID) polarity. Read start hall.
    -c User change motor, which one he works*/
  bool _exit = false;
  if (Command[0] == '-') {
    Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
    Loc_Udp.write("Wrong command form.Return to main menu");
    Loc_Udp.endPacket();
    return true;
  }

  int i = 2;
  bool minus = false;
  long tmp_val = 0; // for command -a and -m
  String _temp_string = ""; // for command -a
  byte k = 0; // for command -a
  byte g = 0; //marker for byte_data
  long  int enter_angle = 0;
  uint16_t steps = 0;
  short int phase = MotorPhaseDataBase[sort_index_of_motor];
  short int _step = 0;
  byte pins[4] = { -1, -1, -1, -1};
  byte zero_pin = -1;
  String Loc_data = "";
  uint8_t zero_data = 0;
  //Data for convert dec to bin and one write to EEPROM
  short int byte_index_dec[4] = {0, 0, 0, 0};
  short int four_byte_number_bin[32]; // 32 bit for long (32 bytes)
  short int bi = 31;
  //For local Calib
  uint8_t IsCalibrated = 0;
  long int CalibrUngle = 0;
  short int SensorPin = MotorPinDataBase[sort_index_of_motor] + 4;
  byte Calibrated = 0;
  int CalibrateCounter = 0;
  short temp_delay = soft_delay;
  InputPin mi_8(SensorPin, false);
  for (int j = 0; j < 4; j++) {
    pins[j] = MotorPinDataBase[sort_index_of_motor] + j;
  }
  if (zero_select) {
    zero_pin = MotorPinDataBase[sort_index_of_motor] + 4;
  }
  switch (Command[0]) {
    case 'a':
      while (Command[i] != ',') {
        _temp_string += Command[i];
        i++;
      }
      if (_temp_string == "VID") {
        MotorType[sort_index_of_motor] = 0;
        EEPROM.write(176 * 16 + 14 + index_of_motor, (byte)MotorType[sort_index_of_motor]);
      } else if (_temp_string == "STH") {
        MotorType[sort_index_of_motor] = 1;
        EEPROM.write(176 * 16 + 14 + index_of_motor, (byte)MotorType[sort_index_of_motor]);
        ReservePin[MotorPinDataBase[sort_index_of_motor] + 4] = MotorPinDataBase[sort_index_of_motor] + 4;//Reserve Pin for zero sensor
      } else {
        Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
        Loc_Udp.write("Error 16");//!!
        Loc_Udp.endPacket();
        return true;
      }
      i++;
      while (Command[i] != ',') {
        short int buff = Command[i] - 48;
        if ((buff >= 0) && (buff < 10)) {
          tmp_val *= 10;
          tmp_val += buff;
        } else if (buff == -3) {
          Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
          Loc_Udp.write("Error 16");//!!
          Loc_Udp.endPacket();
          return true;
        }
        i++;
      }
      //Check value of MotorRange
      if (tmp_val > LONG_MAX) {
        Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
        Loc_Udp.write("Error 16");//!!
        Loc_Udp.endPacket();
        return true;
      }
      MotorRange[sort_index_of_motor] = tmp_val;

      /*Convert to bin,after write in EEPROM!*/
      for (int h = 0; h < 32; h++) {
        four_byte_number_bin[h] = 0;
      }
      while (tmp_val > 1) {
        four_byte_number_bin[bi] = tmp_val % 2;
        tmp_val = tmp_val / 2;
        bi--;
      }
      four_byte_number_bin[bi] = tmp_val;
      while (g < 4) {
        byte b = 0;
        for (int h = 0; h < 32; h++) {
          byte_index_dec[g] = byte_index_dec[g] + four_byte_number_bin[h] * pow_2(7 - b);
          b++;
          if ((h + 1) % 8 == 0) {
            g++;
            b = 0;
          }
        }
      }
      for (int h = 0; h < 4; h++) {
        EEPROM.write(176 * 16 + 28 + (index_of_motor * 4) + h, byte(byte_index_dec[h]));
      }
      tmp_val = 0;
      i++;
      while (Command[i] != ',') {
        short int buff = Command[i] - 48;
        if ((buff >= 0) && (buff < 10)) {
          tmp_val *= 10;
          tmp_val += buff;
        } else if (buff == -3) {
          minus = true;
        }
        i++;
      }
      if (tmp_val > LONG_MAX) {
        Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
        Loc_Udp.write("Error 16");//!!
        Loc_Udp.endPacket();
        return true;
      }
      /*Write data to MotorNullCorrection*/
      if (minus) {
        MotorNullCorrection[sort_index_of_motor] = 0 - tmp_val;
      } else {
        MotorNullCorrection[sort_index_of_motor] = tmp_val;
      }

      /*Convert to bin,after write in EEPROM MotorNullCorrection! Range: -2147483647 to 2147483647 */
      //Init Data
      bi = 31;
      g = 0;
      for (int h = 0; h < 4; h++) {
        byte_index_dec[h] = 0;
      }
      for (int h = 0; h < 31; h++) {
        four_byte_number_bin[h] = 0;
      }
      //Check number's sign
      if (minus) four_byte_number_bin[0] = 1;

      while (tmp_val > 1) {
        four_byte_number_bin[bi] = tmp_val % 2;
        tmp_val = tmp_val / 2;
        bi--;
      }
      four_byte_number_bin[bi] = tmp_val;
      while (g < 4) {
        byte b = 0;
        for (int h = 0; h < 32; h++) {
          byte_index_dec[g] = byte_index_dec[g] + four_byte_number_bin[h] * pow_2(7 - b);
          b++;
          if ((h + 1) % 8 == 0) {
            g++;
            b = 0;
          }
        }
      }
      for (int h = 0; h < 4; h++) {
        EEPROM.write(176 * 16 + 84 + (index_of_motor * 4) + h, byte(byte_index_dec[h]));
      }
      /*End of convert and write*/
      tmp_val = 0;
      i++;
      while (Command[i] != '\0') {
        short int buff = Command[i] - 48;
        if ((buff >= 0) && (buff < 10)) {
          tmp_val *= 10;
          tmp_val += buff;
        }
        i++;
      }
      //Check value of MotorSensorPolarity
      if ((tmp_val > 1) || (tmp_val < 0)) {
        Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
        Loc_Udp.write("Error 16");//!!
        Loc_Udp.endPacket();
        return true;
      }
      MotorSensorPolarity[sort_index_of_motor] = tmp_val;
      EEPROM.write(176 * 16 + index_of_motor, MotorSensorPolarity[index_of_motor]);
      Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
      Loc_Udp.write("Add Calib info done.");
      Loc_Udp.endPacket();
      _exit = false;
      break;
    case 'm':
      /*Read enter_angle*/
      while (Command[i] != '\0') {
        short int buff = Command[i] - 48;
        if ((buff >= 0) && (buff < 10)) {
          enter_angle *= 10;
          enter_angle += buff;
        } else if (buff == -3) {
          minus = true;
        }
        i++;
      }
      if (minus) {
        enter_angle = 0 - enter_angle;
        _step = 1;
      } else {
        _step = -1;
      }
      steps = 0;
      if (zero_select) {
        InputPin mi_8(zero_pin, false);
        while (enter_angle != 0) {
          phase += _step;
          enter_angle += _step;
          steps++;
          if (phase > 7) phase = 0;
          if (phase < 0) phase = 7;
          for (int f = 0; f < 4; f++) {
            OutputPin(pins[f], ((motorPhases[phase][f] == 1) ? HIGH : LOW));
          }
          delay(8);
          if ((mi_8.read() > zero_data) || (mi_8.read() < zero_data)) {
            zero_data = mi_8.read();
            Loc_data = MakeStr(Loc_data, "It is zero pin: ");
            Loc_data = MakeStr(Loc_data, steps);
            Loc_data = MakeStr(Loc_data, " #");
            char *Packet_Loc_Data = new char[Loc_data.length()];
            Loc_data.toCharArray(Packet_Loc_Data, Loc_data.length());
            Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
            Loc_Udp.write(Packet_Loc_Data);
            Loc_Udp.endPacket();
            delete[] Packet_Loc_Data;
            steps = 0;
          }
        }
      } else {
        while (enter_angle != 0) {
          phase += _step;
          enter_angle += _step;
          if (phase > 7) phase = 0;
          if (phase < 0) phase = 7;
          for (int f = 0; f < 4; f++) {
            OutputPin(pins[f], ((motorPhases[phase][f] == 1) ? HIGH : LOW));
          }
          delay(5);
        }
      }
      MotorPhaseDataBase[sort_index_of_motor] = phase;
      _exit = false;
      break;
    case 's':
      Locating_Motors(Loc_Udp);//User choice new target
      break;
    case 'e':
      Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
      Loc_Udp.write("Return to main menu.Reset Func");
      Loc_Udp.endPacket();
      _exit = true;
      break;
    default:
      Loc_Udp.beginPacket(Loc_Udp.remoteIP(), 10002);
      Loc_Udp.write("Wrong type of command.Return to main menu");
      Loc_Udp.endPacket();
      _exit = true;
      break;
  }
  return _exit;
}

uint32_t pow_2(int _rize) {
  uint32_t result = 2;
  if (_rize == 0) return 1;
  if (_rize == 1) return 2;
  for ( int  i = 0; i < _rize - 1; i++) {
    result *= 2;
  }
  return result;
}

void send_signal(String CommandString, EthernetUDP Udp) {
  short int z_len = 2;
  while (CommandString[z_len] != '\0') {
    bool DevNotFound = false;
    char buffer_name[16] = "\0";
    short int m = 2;
    while (CommandString[z_len] != '=') {
      buffer_name[m - 2] += CommandString[z_len];
      m++;
      z_len++;
    }
    z_len++;
    int32_t tmp_val = 0;
    bool minus_mark = false;
    if (CommandString[z_len] == '-') {
      minus_mark = true;
      z_len++;
    }
    while (CommandString[z_len] != ',') {
      short int buff = CommandString[z_len] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      }
      z_len++;
    }
    if (minus_mark) {
      tmp_val = 0 - tmp_val;
    }
    buffer_name[m] = '\0';
    short int Search_index = BinaryNameSearch(MotorNameDataBase, buffer_name, Number_Motor);
    if (Search_index != -1) {
      MotorNeedValue[Search_index] = tmp_val;
      DevNotFound = false;
    } else {
      DevNotFound = true;
    }
    Search_index = BinaryNameSearch(OutputNameDataBase, buffer_name, Number_Output);
    if (Search_index != -1) {
      if ((tmp_val > 256) || (tmp_val < 0)) {
        Udp.beginPacket(Udp.remoteIP(), 10002);
        Udp.write("-s Error enter data to Output Device");
        Udp.endPacket();
        return;
      } else {
        z_len++;//Ползунок на след. имя
        DevNotFound = false;
        OutputValueDataBase[Search_index] = tmp_val;
        OutputPin mi_8(OutputPinDataBase[Search_index]);//Изначально LOW
        analogWrite(OutputPinDataBase[Search_index], OutputValueDataBase[Search_index]);
      }
    } else {
      DevNotFound = true;
    }
    if (DevNotFound) z_len++;
  }
  if (CommandString[0] == 's') {
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write("-s Done");
    Udp.endPacket();
  }
}

void DataBaseNameSort(char (*DataBase)[16], byte PinDataBase[], int start, int _end) {
  if ((_end == 0) || (_end == 1)) {
    return;
  }
  int i = start, j = _end;
  char Pivot[16];
  for (int k = 0; k < 16; k++) {
    Pivot[k] = DataBase[(start + _end) / 2][k];
  }
  do {
    while (strcmp(DataBase[i], Pivot) < 0) {
      i++;
    }
    while (strcmp(DataBase[j], Pivot) > 0) {
      j--;
    }
    if (i <= j) {
      if (strcmp(DataBase[i], DataBase[j]) > 0) { //Sort A-Z
        char _temp[16];
        short int temp_pin;
        for (int k = 0; k < 16; k++) {
          _temp[k] = DataBase[i][k];
        }
        temp_pin = PinDataBase[i];
        for (int k = 0; k < 16; k++) {
          DataBase[i][k] = DataBase[j][k];
        }
        PinDataBase[i] = PinDataBase[j];
        for (int k = 0; k < 16; k++) {
          DataBase[j][k] = _temp[k];
        }
        PinDataBase[j] = temp_pin;
      }
      i++;
      j--;
    }
  } while (i <= j);
  if (i < _end) DataBaseNameSort(DataBase, PinDataBase, i, _end);
  if (start < j) DataBaseNameSort(DataBase, PinDataBase, start, j);
}

void DataBaseMotorSort(char (*DataBase)[16], byte PinDataBase[], int start, int _end) {
  if (_end == 0) {
    return;
  }
  int i = start, j = _end;
  char Pivot[16];
  for (int k = 0; k < 16; k++) {
    Pivot[k] = DataBase[(start + _end) / 2][k];
  }
  do {
    while (strcmp(DataBase[i], Pivot) < 0) {
      i++;
    }
    while (strcmp(DataBase[j], Pivot) > 0) {
      j--;
    }
    if (i <= j) {
      if (strcmp(DataBase[i], DataBase[j]) > 0) { //Sort A-Z
        char _temp[16];
        short int temp_pin;
        //Temp value for calib data
        bool temp_type = 0;
        int32_t temp_nullcor = 0;
        uint32_t temp_range = 0;
        bool temp_senspol = 0;
        for (int k = 0; k < 16; k++) {
          _temp[k] = DataBase[i][k];
        }
        temp_pin = PinDataBase[i];
        temp_type = MotorType[i];
        temp_nullcor = MotorNullCorrection[i];
        temp_range = MotorRange[i];
        temp_senspol = MotorSensorPolarity[i];
        for (int k = 0; k < 16; k++) {
          DataBase[i][k] = DataBase[j][k];
        }
        PinDataBase[i] = PinDataBase[j];
        MotorType[i] = MotorType[j];
        MotorNullCorrection[i] = MotorNullCorrection[j];
        MotorRange[i] = MotorRange[j];
        MotorSensorPolarity[i] = MotorSensorPolarity[j];
        for (int k = 0; k < 16; k++) {
          DataBase[j][k] = _temp[k];
        }
        PinDataBase[j] = temp_pin;
        MotorType[j] = temp_type;
        MotorNullCorrection[j] = temp_nullcor;
        MotorRange[j] = temp_range;
        MotorSensorPolarity[j] = temp_senspol;
      }
      i++;
      j--;
    }
  } while (i <= j);
  if (i < _end) DataBaseMotorSort(DataBase, PinDataBase, i, _end);
  if (start < j) DataBaseMotorSort(DataBase, PinDataBase, start, j);
}

int BinaryNameSearch(char (*DataBase)[16], char Found_Name[], int _size) {
  short int first = 0, last = _size;
  while (1) {
    short int mid = (last + first) / 2;
    if (strcmp(Found_Name, DataBase[mid]) == 0) {
      return mid;
    } else if (strcmp(Found_Name, DataBase[mid]) > 0) {
      first = mid + 1;
    } else {
      last = mid - 1;
    }
    /*Обработка случая, при иинтервале 0 - 1*/
    if (mid == 0) {
      if (strcmp(Found_Name, DataBase[mid + 1]) == 0) {
        return mid + 1;
      }
    }
    if (first > last) return -1;
  }
}


void MotorAction(int32_t enter_angle, int need_motor) {
  //Russian delay! Without stop main programm
  short int pins[4];
  short int SensorPin;
  //MotorNeedValue[need_motor] = enter_angle - MotorCurrentValue[need_motor];
  //need_angle+=20;
  for (int i = 0; i < 4; i++) {
    pins[i] = MotorPinDataBase[need_motor] + i;
  }
  SensorPin = MotorPinDataBase[need_motor] + 4;
  if (enter_angle - MotorCurrentValue[need_motor] != 0) {
    if (enter_angle - MotorCurrentValue[need_motor] > 0) {
      _step = 1;
    }
    else
    {
      _step = -1;
    }
    MotorPhaseDataBase[need_motor] += _step;
    //MotorNeedValue[need_motor] -= _step;
    MotorCurrentValue[need_motor] += _step;
    if (MotorPhaseDataBase[need_motor] > 7) MotorPhaseDataBase[need_motor] = 0;
    if (MotorPhaseDataBase[need_motor] < 0)
    {
      MotorPhaseDataBase[need_motor] = 7;
    }
    for (int i = 0; i < 4; i++) {
      OutputPin(pins[i], ((motorPhases[MotorPhaseDataBase[need_motor]][i] == 1) ? HIGH : LOW));
    }
  }
}

void Terminate_EEPROM(EthernetUDP Udp) {
  for (int i = 0; i < 3072; i++) {
    EEPROM.write(i, 255);
  }
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("-t Done");
  Udp.endPacket();
}

/*Change system info on board: IP,(MAC),Motor_Delay
  Format: -i 192.168.2.1,Delay_Motor(10-255):*/
void Shift_System_info(String Command, EthernetUDP Udp) {
  short int z_len = 2;
  short int cont_summ = 0;
  short int tmp_val = 0;
  for (int i = 0; i < 4; i++) {
    cont_summ = 0;
    tmp_val = 0;
    while ((Command[z_len] != '.') && (Command[z_len] != ',')) {
      if (cont_summ < 4) {
        short int buff = Command[z_len] - 48;
        if ((buff >= 0) && (buff < 10)) {
          tmp_val *= 10;
          tmp_val += buff;
        } else {
          Udp.beginPacket(Udp.remoteIP(), 10002);
          Udp.write("Error 12");//Wrong sumbol in ip command format
          return;
        }
        z_len++;
        cont_summ++;
      } else {
        Udp.beginPacket(Udp.remoteIP(), 10002);
        Udp.write("Error 13");//Wrong command format.Check string, entering in command -i ...\n
        Udp.endPacket();
        return;
      }
    }
    ip_data[i] = tmp_val;
    EEPROM.write(185 * 16 + i, ip_data[i]);
    z_len++;// Going to next byte of IP_address
  }
  cont_summ = 0;
  tmp_val = 0;
  while (Command[z_len] != '\0') {
    if (cont_summ < 5) {
      short int buff = Command[z_len] - 48;
      if ((buff >= 0) && (buff < 10)) {
        tmp_val *= 10;
        tmp_val += buff;
      } else {
        Udp.beginPacket(Udp.remoteIP(), 10002);
        Udp.write("Error 14");//Wrong sumbol in Motor_delay format
        Udp.endPacket();
        return;
      }
      z_len++;
      cont_summ++;
    } else {
      Udp.beginPacket(Udp.remoteIP(), 10002);
      Udp.write("Error 13");//Wrong command format.Check string, entering in command -i ...\n
      Udp.endPacket();
      return;
    }
  }
  if ((tmp_val < 10) || (tmp_val > 250)) {
    Udp.beginPacket(Udp.remoteIP(), 10002);
    Udp.write("Error 15");//Wrong value of motor delay
    Udp.endPacket();
    return;
  }
  soft_delay = tmp_val;
  EEPROM.write(185 * 16 + 4, soft_delay);
  Udp.beginPacket(Udp.remoteIP(), 10002);
  Udp.write("i Done");//Wrong value of motor delay
  Udp.endPacket();

  Udp.stop();//Close UDP session
  IPAddress new_ip(ip_data[0], ip_data[1], ip_data[2], ip_data[3]);
  Ethernet.begin(mac, new_ip);
  Udp.begin(localPort);
  EEPROM.write(9, 1); //We changed system info
}

void loop() {
  // put your main code here, to run repeatedly:
  while (1) {
    Udp_Control(); //Управление с помощью Udp
    Check_Device();//Проверка значений и отработка значений
    wdt_reset();
    //Check_Server_Connection();
  }
}

void (* resetFunc) (void) = 0;

/*void Check_Server_Connection() {
  Ping_Delay++;
  if (Ping_Delay == 30000) {
    bool ping_success = false;
    while (!ping_success) {
      ICMPEchoReply Server_Reply = ping(mi8_server, 1);
      if (Server_Reply.status == SUCCESS) {
        ICMPEcho();
        ping_success = true;
        Ping_Delay = 0;
      } else {
        Udp_Core.stop();
        Ethernet.begin(mac, ip);
        Udp_Core.begin(localPort);
      }
    }
  }
  }*/

void Command_execution(String UdpCommand) {
  switch (UdpCommand[0]) {
    case 'c':
      Show_device(Udp_Core);//+
      break;
    case 's':
      send_signal(UdpCommand, Udp_Core); //+-
      break;
    case 'g':
      Give_signal(Udp_Core);
      break;
    case 'm':
      wdt_disable();
      send_signal(UdpCommand, Udp_Core);
      Give_signal(Udp_Core);
      wdt_enable(WDTO_1S);
      break;
    case 'r':
      wdt_disable();
      resetFunc();
      break;
    case 'a':
      wdt_disable();
      add_device(UdpCommand, Udp_Core); //+
      wdt_enable(WDTO_1S);
      break;
    case 'd':
      delete_device(UdpCommand, Udp_Core);//+
      break;
    case 'p':
      Ping_status(Udp_Core);//+
      break;
    case 'i':
      wdt_disable();
      Shift_System_info(UdpCommand, Udp_Core);
      wdt_enable(WDTO_1S);
      break;
    case 'e':
      Exam_connect_device(Udp_Core);//+
      break;
    case 't':
      wdt_disable();
      Terminate_EEPROM(Udp_Core);
      wdt_enable(WDTO_1S);
      break;
    case 'l':
      wdt_disable();
      Locating_pins(Udp_Core);
      wdt_enable(WDTO_1S);
      break;
    case 'z':
      wdt_disable();
      StartCalibration();
      soft_delay = EEPROM.read(185 * 16 + 4);
      wdt_enable(WDTO_1S);
      break;
    case 'v':
      Udp_Core.beginPacket(Udp_Core.remoteIP(), 10002);
      Udp_Core.write("Version 0.9.9.1");
      Udp_Core.endPacket();
      break;
    default:
      Udp_Core.beginPacket(Udp_Core.remoteIP(), 10002);
      Udp_Core.write("Error enter type of command");
      Udp_Core.endPacket();
  }
}

void Udp_Control() {
  int packetSize = Udp_Core.parsePacket();
  if (packetSize) {
    bool command_flag = true;
    String UdpCommand = "";
    char tmp = (char)Udp_Core.read();
    if (tmp != '-') {
      Udp_Core.beginPacket(Udp_Core.remoteIP(), 10002);
      Udp_Core.write("Error: wrong command format");
      Udp_Core.endPacket();
      return;
    }
    for (int i = 0; i < packetSize; i++) {
      tmp = (char)Udp_Core.read();
      switch (tmp) {
        case '\n':
          Command_execution(UdpCommand);
          command_flag = false;
          UdpCommand = "";
          Udp_Core.flush();
          break;
        case '-':
          if (!command_flag) {
            command_flag = true;
          } else {
            UdpCommand += "-";
          }
          break;
        default:
          UdpCommand += tmp;
      }
    }
  }
  Ethernet.maintain();
}

