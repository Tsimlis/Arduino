
#define pin_nega 32
#define pin_posi 33
#define Fuel_pin 12  // สัญญาณออกคุมหัวฉีด // Mosfet 12V
#define Ingi_pin 13  // สัญญาณออกคุมหัวเทียน // Mosfet 12V
#define TPS_pin 14 // รับโวลต์จาก map sensor // 0-3V 

int freq, sub, delay_time = 200000;

uint16_t positive, negative;
long time_N, time_P, old_time, time_fuel, time_ingi;
int time_cicle, degree_value, degree;
bool trik_N = true, trik_P = true, fuel = false, ingi = false; 

volatile unsigned int RPM = 0, TPS = 0;
volatile uint8_t point_rpm = 0, point_tps = 0;

extern volatile unsigned int tps_table[10] = { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90 };                      // เปอร์เซ็นคันเร่ง
extern volatile unsigned int rpm_table[10] = { 800, 1200, 1800, 3500, 4000, 5000, 6000, 7000, 8000, 9000 };  // จำนวนรอบเครื่องยนต์

extern volatile int Fuel_table[10][10] = {
  // หน่วนเป็น ไมโครวินาที  us
  /*tps -->   0|   10|   20|   30|   40|   50|   60|   70|   80|   90 */
  /*800 */ 3200, 3200, 3500, 3700, 4000, 4500, 6000, 7000, 8000, 10000,  //0
  /*1200*/ 3200, 3200, 3500, 3800, 4100, 4600, 5100, 6500, 7500, 9000,   //1
  /*1800*/ 2100, 2100, 3500, 3800, 4300, 4600, 5200, 6700, 7500, 8500,   //2
  /*3500*/ 1800, 2000, 2500, 2800, 3500, 4000, 4500, 5000, 6000, 7000,   //3
  /*4000*/ 1500, 2000, 2300, 2500, 3500, 4000, 4300, 4000, 5000, 6500,   //4
  /*5000*/ 1500, 1800, 2000, 2300, 2500, 3000, 3500, 3800, 4000, 6800,   //5
  /*6000*/ 1200, 1300, 1500, 1500, 1500, 2500, 3000, 3500, 4500, 6000,   //6
  /*7000*/ 1000, 1000, 1000, 1000, 1300, 1500, 2500, 3000, 5000, 5800,   //7
  /*8000*/ 1000, 1000, 1000, 1000, 1000, 1000, 1500, 2500, 4500, 5500,   //8
  /*9000*/ 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1300, 2500, 5000    //9
  // rpm
  // |
  // V
};
extern volatile int Inj_table[10][10] = {
  // หน่วนเป็น องศา
  /*tps-->  0| 10| 20| 30| 40| 50| 60| 70| 80| 90 */
  /*800 */ 10, 10, 10, 10, 10, 10, 11, 13, 15, 15,  //0
  /*1200*/ 11, 10, 10, 12, 12, 12, 13, 13, 15, 15,  //1
  /*1800*/ 13, 13, 13, 13, 14, 14, 14, 15, 15, 17,  //2
  /*3500*/ 15, 15, 15, 15, 15, 16, 16, 16, 16, 17,  //3
  /*4000*/ 17, 17, 17, 17, 17, 17, 19, 19, 19, 19,  //4
  /*5000*/ 19, 19, 19, 19, 19, 20, 20, 20, 20, 20,  //5
  /*6000*/ 20, 20, 20, 20, 20, 20, 20, 21, 21, 23,  //6
  /*7000*/ 20, 20, 20, 20, 21, 21, 22, 22, 23, 25,  //7
  /*8000*/ 23, 23, 23, 23, 23, 23, 25, 25, 25, 25,  //8
  /*9000*/ 23, 23, 23, 25, 25, 25, 25, 27, 27, 27   //9
  // rpm
  // |
  // V
};

void location_point(int rpm, int tps) {
  // check point table from RPM
  if (rpm >= rpm_table[0] && rpm < rpm_table[1]) point_rpm = 0;
  else if (rpm >= rpm_table[1] && rpm < rpm_table[2]) point_rpm = 1;
  else if (rpm >= rpm_table[2] && rpm < rpm_table[3]) point_rpm = 2;
  else if (rpm >= rpm_table[3] && rpm < rpm_table[4]) point_rpm = 3;
  else if (rpm >= rpm_table[4] && rpm < rpm_table[5]) point_rpm = 4;
  else if (rpm >= rpm_table[5] && rpm < rpm_table[6]) point_rpm = 5;
  else if (rpm >= rpm_table[6] && rpm < rpm_table[7]) point_rpm = 6;
  else if (rpm >= rpm_table[7] && rpm < rpm_table[8]) point_rpm = 7;
  else if (rpm >= rpm_table[8] && rpm < rpm_table[9]) point_rpm = 8;
  else if (rpm >= rpm_table[9]) point_rpm = 9;

  // check point table from TPs
  if (tps >= tps_table[0] && tps < tps_table[1]) point_tps = 0;
  else if (tps >= tps_table[1] && tps < tps_table[2]) point_tps = 1;
  else if (tps >= tps_table[2] && tps < tps_table[3]) point_tps = 2;
  else if (tps >= tps_table[3] && tps < tps_table[4]) point_tps = 3;
  else if (tps >= tps_table[4] && tps < tps_table[5]) point_tps = 4;
  else if (tps >= tps_table[5] && tps < tps_table[6]) point_tps = 5;
  else if (tps >= tps_table[6] && tps < tps_table[7]) point_tps = 6;
  else if (tps >= tps_table[7] && tps < tps_table[8]) point_tps = 7;
  else if (tps >= tps_table[8] && tps < tps_table[9]) point_tps = 8;
  else if (tps >= tps_table[9]) point_tps = 9;
}


// 600 rpm = 1s:10r = 100ms:1r
// 1200 rpm = 1s:20r = 50ms:1r
// 6000 rpm = 1s:100r = 10ms:1r
// 12 000 rpm = 1s:200r = 5ms:1r



void setup() {
  Serial.begin(115200);

  pinMode(pin_nega, INPUT);
  pinMode(pin_posi, INPUT);
  pinMode(TPS_pin, INPUT);
  pinMode(Fuel_pin, OUTPUT);
  pinMode(Ingi_pin, OUTPUT);

}


void loop() {

  positive = analogRead(pin_posi);
  negative = analogRead(pin_nega);

  // Serial.println(positive);

  if (negative > 3500) {
    if (trik_N) { // เก็บครั้งเดียวแล้วก็ออก
      trik_N = false;
      time_N = micros();
      fuel = true;
    }
  } else if (negative < 3500) trik_N = true;

  if (positive > 3500) {
    if (trik_P) {
      trik_P = false;
      ingi = true;
      time_P = micros();
      time_cicle = time_P - old_time;
      old_time = time_P;

      degree = (time_cicle / 360) * (30 -  Inj_table[point_rpm][point_tps]);

      freq++;
    }
  } else if (positive < 3500) trik_P = true;

  //  วัดรอบต่อนาที
  if (micros() >= sub + delay_time) {
    sub = micros();
    RPM = (freq * 5) * 60;
    freq = 0;
  }

  

  // ///////////////////////////////////////
  TPS = map(digitalRead(TPS_pin), 0, 4096, 0, 100);  // TPS persent 3.3v
  
  /// ตำแหน่งตาราง
  location_point(RPM, TPS); // คำนวนตำแหน่งจ่ายน้ำมันและองศาไฟ



  // จ่ายน้ำมัน
  if (fuel) {
    time_fuel = micros() - time_N;
    if (time_fuel <= Fuel_table[point_rpm][point_tps]) {
      digitalWrite(Fuel_pin, HIGH); // จ่ายหัวฉีดตาม ms ที่กำหนด
    } else {
      digitalWrite(Fuel_pin, LOW);
      fuel = false;
    }
  }

  // จ่ายไฟหัวเทียน
  if (ingi) {
    time_ingi = micros() - time_P;

    if (time_ingi >= degree) {  // จ่ายหัวเทียนตามองศาที่กำหนด
      if (time_ingi <= degree + 5000) {
        digitalWrite(Ingi_pin, HIGH); 
      } else {
        digitalWrite(Ingi_pin, LOW);
        ingi = false;
      }
    }
  }
  
}
