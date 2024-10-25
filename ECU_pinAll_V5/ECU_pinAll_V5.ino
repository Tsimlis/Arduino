#include "driver/adc.h"

// กำหนดหมายเลขขา GPIO ที่ใช้
#define PIN_Pulse 4
#define ADC_TPS ADC1_CHANNEL_4  // GPIO32
#define Fuel_pin 5
#define Igni_pin 12

// ตัวรับค่าการจาก input
volatile uint8_t positive;
volatile uint16_t TPS = 0;  

hw_timer_t *timer = NULL;  // Timer handler
float time_use;

unsigned int cicle, tm_old;
unsigned int sub, delay_time = 200000;

unsigned long time_P, old_time, time_fuel, time_igni;
volatile unsigned int time_cicle, time_cicle_old, degree;
volatile bool trik_P = true, step = true; 

volatile unsigned int RPM = 0;
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
  /*4000*/ 15, 17, 17, 17, 17, 17, 19, 19, 19, 19,  //4
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



// Timer ISR (runs every 1 ms)
void IRAM_ATTR onTimer() {
  // uint32_t startCycles = ESP.getCycleCount(); // start
  
  //  อ่านค่าอนาล็อก tps
  TPS = (int)(adc1_get_raw(ADC_TPS) / 10.24); // 135 us
  
  // uint32_t endCycles = ESP.getCycleCount(); // end
  // uint32_t cycles = endCycles - startCycles;
  // time_use = cycles / 80.0; 

}


void setup() {
  Serial.begin(115200);

  // ตั้งค่า timer ให้ทำงานที่ความเร็วสูงสุด
  timer = timerBegin(0, 80, true);  // Prescaler = 1, 1 tick = 12.5 ns
  timerAttachInterrupt(timer, &onTimer, true);  // Attach ISR
  timerAlarmWrite(timer, 300, true);  // Trigger every 1 tick (12.5 ns)
  timerAlarmEnable(timer);  // Enable timer
  
  // Configure ADC1 channel
  adc1_config_width(ADC_WIDTH_BIT_10);
  adc1_config_channel_atten(ADC_TPS, ADC_ATTEN_DB_11);

  // set up Output
  gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << Fuel_pin) | (1 << Igni_pin);
    gpio_config(&io_conf);
}

void loop() {
  
  // อ่านสถานะของ GPIO16 โดยใช้ register โดยตรง
  positive = (GPIO.in >> PIN_Pulse) & 0x1; // 26 ns

  if (positive) {
    if (trik_P) {
      trik_P = false;

      time_P = micros();
      time_cicle = time_P - old_time; // จับเวลาแต่ละรอบ
      old_time = time_P;

      step = !step;
      // if (time_cicle > time_cicle_old) step = false; // หาจังหวะระเบิด, ฉีด
      // else step = true;
      time_cicle_old = time_cicle;

      RPM = (int)(1000000 / time_cicle) * 60; //  วัดรอบต่อนาที

      location_point(RPM, TPS);

      degree = (int)(time_cicle / 360) * (30 - Inj_table[point_rpm][point_tps]); // จัดองศาหัวเทียน

      
      // ฟังกชันการทำงาน
      if (step) {  // จ่ายน้ำมัน

        GPIO.out_w1ts = (1 << Fuel_pin); // 140 ns
        delay_us(Fuel_table[point_rpm][point_tps]);
        GPIO.out_w1tc = (1 << Fuel_pin); // 70 ns
// 
      } else {  // จ่ายไฟหัวเทียน

        delay_us(degree);
        GPIO.out_w1ts = (1 << Igni_pin); // 140 ns
        delay_us(1000);
        GPIO.out_w1tc = (1 << Igni_pin); // 70 ns

      }

      // Serial.println(String(point_rpm) + " => " + String(point_tps));

      // //reset point
      // point_tps = 0;
      // point_rpm = 0;

    }
  } else trik_P = true;

  // Serial.println(time_use);
  // Serial.println(positive);
  // Serial.println(TPS);
  

  int tm = micros();
  cicle = tm - tm_old;
  tm_old = tm;
  Serial.println(cicle);

}

// หน่วงเวลา หยุดการทำงาน
void delay_us(uint32_t us) {
    uint64_t m = (uint64_t)esp_timer_get_time();
    if(us){
        uint64_t e = (m + us);
        if(m > e){ //overflow
            while((uint64_t)esp_timer_get_time() > e){
                asm volatile ("nop");
            }
        }
        while((uint64_t)esp_timer_get_time() < e){
            asm volatile ("nop");
        }
    }
}