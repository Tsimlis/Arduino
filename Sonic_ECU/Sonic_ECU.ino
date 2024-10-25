#include "time.h"
#include "esp_timer.h"

// ไฟเปิดกุฎแจเข้า ECU
#define Start_pin 13  // 12V -> output 3-5 V
// รับสัญญาณจากพัช บวก (CKP)
#define Positive_pin 12  // 15V -> Diod to DC -> C -> devider output 3-5 V
// สัญญาณออกคุมหัวฉีด
#define Fuel_pin 14  // Mosfet 12V
// สัญญาณออกคุมหัวเทียน
#define Inj_pin 27  // Mosfet 12V
// รับโวลต์จาก map sensor
#define TPS_pin 33  // 0-5V -> 0-3V Cdevider)
// ส่งสัญญาณควบคุมปั้มน้ำมัน
#define Pump_pin 32  // Mosfet 12V

volatile unsigned int RPM = 0, TPS = 0;
volatile uint8_t point_rpm = 0, point_tps = 0;

bool step_fuel = true, err_late = false;
unsigned int time_boom, time_late;
volatile unsigned int CKP_time_po = 1, CKP_old = 0, fuel_degree, inj_degree;
volatile unsigned long N_po = 0, K_po = 0, micro_fuel = 0;

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
  /*800 */ 10, 10, 10, 10, 10, 10, 11, 13, 15, 15,   //0
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


// รับสัญญาณพัช บวก
void IRAM_ATTR Positive_ISR() {  // IRAM_ATTR เก็บไว้ที่หน่วยความจำ ram เพื่อไม่รบกวน Loop
  K_po = micros();
  CKP_time_po = K_po - N_po;
  N_po = K_po;

  if (CKP_time_po > 500000) CKP_time_po = 150000; // กรณีรอบแรกที่ N_po = 0 , อิงให้เป็น 400rpm
  RPM = 60 * (1000000 / CKP_time_po);    //  calculator RPM
  
  // สลับระหว่างจ่ายน้ำมัน กับจ่ายหัวเทียน
  step_fuel = !step_fuel;
  err_late = true;

  if (RPM < 800) { // กรณีเผื่อจังหวะไม่ตรง
    if (CKP_time_po > CKP_old) step_fuel = false; // เช็คจัวหวะอัดจากช่วงเวลาที่มากกว่าของรอบก่อนหน้า เวลารอบอัดจะมากกว่าจากแรงอัด
  }
  CKP_old = CKP_time_po;

}


void Tiggle(){

  location_point(RPM, TPS);
  inj_degree = (CKP_time_po / 360) * (30 -  Inj_table[point_rpm][point_tps]);  //หาเวลาแต่ละองศา และกลับค่าองศาเพื่อจุดระเบิด
  inj_limit = (CKP_time_po / 360) * 270; //องศาไฟที่รอบตัด จะจุดตอนวาว์ลไอเสียเปิด เพื่อสร้างประกายไฟออกท่อ
  fuel_degree = (CKP_time_po / 360) * 10;  //หาเวลาแต่ละองศา จ่ายน้ำมัน 10 องศาก่อนศูนย์ตายบน

  time_boom = micros() - K_po;  // 1 000 000 us:1s 

  // ค่าความคาดเคลื่อนตั้งแต่ทริกจนถึงคำสั่งนี้ก่อนจ่ายน้ำมันและหัวเที่ยน
  if (err_late) { 
    time_late = micros() - K_po;  //ตำแหน่งแสดงความต่างของการทำงานมาถึงนี้ว่าผ่านไปแล้วกี่ us ****
    err_late = false;
  }

  if (step_fuel) {
    // digitalWrite(Fuel_pin, LOW); // ในจังหวะหัวเทียนระเบิดหัวฉีดจะต้องปิด
    // if (RPM < 10000) {
      // สั่งจ่ายหัวเทียน
      if (time_boom >= inj_degree - time_late) { // ลบค่าความคาดเคลื่อน
        if (time_boom <= (inj_degree - time_late) + 3000) digitalWrite(Inj_pin, HIGH); // จุดหัวเทียน 3ms ตามเวลาที่คำนวนองศามาแล้ว
        else digitalWrite(Inj_pin, LOW);
      }
    // } else {
    //   if (time_boom >= inj_limit - time_late) { // ลบค่าความคาดเคลื่อน
    //     if (time_boom <= (inj_limit - time_late) + 3000) digitalWrite(Inj_pin, HIGH); // จุดหัวเทียน 3ms ตามเวลาที่คำนวนองศามาแล้ว
    //     else digitalWrite(Inj_pin, LOW);
    //   }
    
  } else {
    // digitalWrite(Inj_pin, LOW); // ในจังหวะจ่ายน้ำมันหัวเทียนที่เกินจะต้องถูกปิด
    // if (RPM < 10000) {
      // สั่งจ่ายหัวฉีด
      if (time_boom >= fuel_degree - time_late) {
        if (time_boom <= (fuel_degree - time_late) + Fuel_table[point_rpm][point_tps]) digitalWrite(Fuel_pin, HIGH); // จ่ายหัวฉีดตาม ms ที่กำหนด
        else digitalWrite(Fuel_pin, LOW);
      }
    // } else {
    //   if (time_boom >= fuel_degree - time_late) {
    //     if (time_boom <= (fuel_degree - time_late) + 1000) digitalWrite(Fuel_pin, HIGH); // จ่ายหัวฉีดตาม ms ที่กำหนด
    //     else digitalWrite(Fuel_pin, LOW);
    //   }
    // }
    
  }
}


// PUMP FUEL
unsigned int Q = 0, P = 0, time_pump;

void Pump() {
  // ปั้มทำงาน 3 วิเริ่มแรก
  Q = millis();
  time_pump = Q - P;
  P = Q;

  if (time_pump < 3000) {
    digitalWrite(Pump_pin, HIGH);
    if(time_pump < 1000) digitalWrite(Fuel_pin, HIGH); // จ่ายน้ำมันก่อนสตาร์ท 1s 
    else digitalWrite(Fuel_pin, LOW);
  } else digitalWrite(Pump_pin, LOW);
}



////////////////////////////////กรณีเกิดเหตุการณ์/////////////////////////////

//กรณีคันเร่งค้างหรือแมพเซนเซอร์เสีย ไฟเข้า 3.3v เต็มเครื่องเร่งเต็มที่//
//กรณีหัวฉีดเสีย//
//วัดโวลต์ของเซนเซอร์หม้อน้ำ//
//สร้างจุดใส่ O2 เซนเซอร์ไว้หาการเผาไหม้//
//รอบตัด//



void setup() {
  Serial.begin(115200);

  // PIN
  pinMode(Start_pin, INPUT);
  pinMode(Fuel_pin, OUTPUT);
  pinMode(Pump_pin, OUTPUT);
  pinMode(Inj_pin, INPUT);
  pinMode(Positive_pin, INPUT);
  pinMode(TPS_pin, INPUT);

  attachInterrupt(Positive_pin, Positive_ISR, RISING);  // rising ต่ำไปสูง // เมื่อทริกคำสั่งจ่ายไฟที่ขาออกจะช้าไป 1.8us
}


void loop() {
  
  TPS = map(digitalRead(TPS_pin), 0, 4096, 0, 100);  // TPS persent 3.3v
  
  /// ทดสอบวิ่งสุ่ม
  RPM = random(600, 10000);
  TPS = random(0, 100);

  // ฟังกชันจ่ายน้ำมันและหัวเทียน
  Tiggle();
  // เปิดปั้มติดเมื่อ ECU ทำงาน
  Pump();  
  // เครื่องยนต์ทำงานให้ปั้มติดทำงาน
  if (RPM != 0) digitalWrite(Pump_pin, HIGH); 
  else digitalWrite(Pump_pin, LOW);


  Serial.print("RPM : ");
  Serial.print(RPM);
  Serial.print("  TPS : ");
  Serial.print(TPS);
  Serial.print("  Timelast : ");
  Serial.println(time_late);

  
 // test
  Serial.print("  Fuel_point : ");
  Serial.print(Fuel_table[point_rpm][point_tps]);
  Serial.print("  Inj_point : ");
  Serial.println(Inj_table[point_rpm][point_tps]);
}
