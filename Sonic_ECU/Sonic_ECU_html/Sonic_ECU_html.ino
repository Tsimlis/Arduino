#include "time.h"
#include "esp_timer.h"
////////////////////////
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h> // ใช้สำหรับการจัดเก็บข้อมูลในหน่วยความจำถาวร

// ตั้งค่า SSID และรหัสผ่านสำหรับโหมด AP
const char* ssid = "ESP32-AP-WebSocket";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool stop = true, check = true;
int rpm = 0, tps = 0, num = 1, rssi;

Preferences preferences; // สร้าง Preferences object
///////////////////////

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
  initWifi(); //ส่วนของไวไฟ

  // PIN
  pinMode(Start_pin, INPUT);
  pinMode(Fuel_pin, OUTPUT);
  pinMode(Pump_pin, OUTPUT);
  pinMode(Inj_pin, INPUT);
  pinMode(Positive_pin, INPUT);
  pinMode(TPS_pin, INPUT);

  attachInterrupt(Positive_pin, Positive_ISR, RISING);  // rising ต่ำไปสูง
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

  wifiLoop(); // ส่วนของเว็บ
}









///////////////////////////////////////////////////////////////


void initWifi(){

   // เริ่มต้น ESP32 ในโหมด AP
  WiFi.softAP(ssid, password);  // ปกติสามารถเชื่อมได้ 4 เครื่อง สูงสุด 10 เครื่อง
  WiFi.softAP((WiFi.softAPIP().toString() + "_" + (String)ssid).c_str(), password);

  Serial.println();
  Serial.print("Access Point \"");
  Serial.print(ssid);
  Serial.println("\" เริ่มทำงาน");

  // แสดง IP Address ของ ESP32
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // กำหนด WebSocket event
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // กำหนดหน้าเว็บ HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<!DOCTYPE html><html>"
      "<head><title>ESP32 WebSocket Server</title>"
      "<style>"
      "body {"
      "  font-family: Arial, sans-serif;"
      "  display: flex;"
      "  justify-content: center;"
      "  align-items: center;"
      "  height: 100vh;"
      "  margin: 0;"
      "  background-color: #f4f4f4;"
      "}"
      ".container {"
      "  text-align: center;"
      "  background: white;"
      "  border-radius: 10px;"
      "  box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);"
      "  padding: 20px;"
      "}"
      "p {"
      "  font-size: 1.2em;"
      "  margin: 10px 0;"
      "}"
      "table {"
      "  width: 100%;"
      "  border-collapse: collapse;"
      "  margin-top: 20px;"
      "}"
      "table, th, td {"
      "  border: 1px solid #ddd;"
      "}"
      "th, td {"
      "  font-size: 14px;"
      "  padding: 5px;"
      "  text-align: center;"
      "}"
      "th {"
      "  background-color: #f2f2f2;"
      "}"
      ".highlight {"
      "  background-color: #c6f2c6; /* สีเขียว */"
      "}"
      ".grey {"
      "  background-color: #f2f2f2;"
      "}"
      "button {"
      "  background-color: #008cba;"
      "  color: white;"
      "  border: none;"
      "  padding: 10px 20px;"
      "  text-align: center;"
      "  font-size: 16px;"
      "  margin: 10px 5px;"
      "  cursor: pointer;"
      "  border-radius: 5px;"
      "}"
      "button:hover {"
      "  background-color: #005f5f;"
      "}"
      "</style>"
      "</head>"
      "<body>"
      "<div class='container'>"
      "<h2>ESP32 WebSocket Server</h2>"
      "<p>RPM: <span id='rpm'>data.rpm</span></p>"
      "<p>TPS: <span id='tps'>data.tps</span></p>"
      "<table id='fuel-table'></table>"

      "<button id='toggle-button'style='background-color: teal;'>Fuel Table</button>"
      "<button id='stop-button' style='background-color: steelblue;'>Stop</button>"
      "<button id='play-button' style='background-color: brown;'>Play</button>"
      "<button id='clear-button' >Clear</button>"

      "<script>"
      "var ws = new WebSocket('ws://' + window.location.hostname + '/ws');"
      "var showFuelTable = true;"
      "ws.onopen = function() {"
      "  document.getElementById('status').textContent = 'Connected';"
      "};"
      "ws.onmessage = function(event) {"
      "  var data = JSON.parse(event.data);"
      "  document.getElementById('rpm').innerHTML = data.rpm;"
      "  document.getElementById('tps').innerHTML = data.tps;"
      "  updateTable(data.tpsTable, data.rpmTable, showFuelTable ? data.fuelTable : data.injTable, data.highlightI, data.highlightJ);"
      "};"

      "function updateTable(tpsTable, rpmTable, table, highlightI, highlightJ) {"
      "  var tableElem = document.getElementById('fuel-table');"
      "  var rows = '<tr><th>RPM/TPS</th>';"
      "  for (var i = 0; i < tpsTable.length; i++) {"
      "    var cellClass = i === highlightJ ? 'highlight' : '';"
      "    rows += '<th class=\"' + cellClass + '\">' + tpsTable[i] + '%' + '</th>';"
      "  }"
      "  rows += '</tr>';"

      "  for (var i = 0; i < table.length; i++) {"
      "    var row = '<tr>';"
      "    var green = i === highlightI ? 'highlight' : 'grey';"
      "    row += '<td class=\"' + green + '\">' + rpmTable[i] + '</td>';"

      "    for (var j = 0; j < table[i].length; j++) {"
      "      var cellClass = (i === highlightI && j === highlightJ) ? 'highlight' : '';"
      "      row += '<td class=\"' + cellClass + '\" contenteditable=\"true\" onblur=\"sendCellValue(' + i + ',' + j + ',this.innerHTML)\">' + table[i][j] + '</td>';"
      "    }"
      "    row += '</tr>';"
      "    rows += row;"
      "  }"
      "  tableElem.innerHTML = rows;"
      "}"

      "function sendCellValue(row, col, value) {"
      "  var data = {"
      "    type: showFuelTable ? 'fuel' : 'inj',"
      "    row: row,"
      "    col: col,"
      "    value: parseInt(value)"
      "  };"
      "  ws.send(JSON.stringify(data));"
      "}"

      "document.getElementById('toggle-button').onclick = function() {"
      "  showFuelTable = !showFuelTable;"
      "  this.textContent = showFuelTable ? 'Fuel Table' : 'Injection Table';"
      "};"

      "document.getElementById('stop-button').onclick = function() {"
      "  ws.send(JSON.stringify({ type: 'stop' }));"
      "};"

      "document.getElementById('play-button').onclick = function() {"
      "  ws.send(JSON.stringify({ type: 'play' }));"
      "};"

      "document.getElementById('clear-button').onclick = function() {"
      "  ws.send(JSON.stringify({ type: 'clear' }));"
      "};"

      "</script>"
      "</div>"
      "</body>"
      "</html>");
  });

  // เริ่มต้น server
  server.begin();
  // เปิด Preferences
  preferences.begin("tables", false); // false = R/W mode
  // โหลดตารางที่บันทึกไว้
  loadTableData();

}

/////////////ภายใน loop ////////////
void wifiLoop(){

  stop = false;
  if (millis() > 500 * num) {  //ความถี่ 500ms ในการส่งข้อมูลไปหน้าเว็บแต่ละครั้ง
    num++;
    stop = true;
  }
  

  if (stop && check ) {
    // อ่านค่า RSSI ของ AP ที่เชื่อมต่อ
    int rssi = WiFi.RSSI();

    // สร้าง JSON object
    StaticJsonDocument<1024> jsonDoc;
    jsonDoc["rpm"] = RPM;
    jsonDoc["tps"] = TPS;
    jsonDoc["rssi"] = rssi;

    JsonArray tpsTable = jsonDoc.createNestedArray("tpsTable");
    JsonArray rpmTable = jsonDoc.createNestedArray("rpmTable");
    JsonArray fuelTable = jsonDoc.createNestedArray("fuelTable");
    JsonArray injTable = jsonDoc.createNestedArray("injTable");

    for (int i = 0; i < 10; i++) {
      tpsTable.add(tps_table[i]);
      rpmTable.add(rpm_table[i]);
      JsonArray rowFuel = fuelTable.createNestedArray();
      JsonArray rowInj = injTable.createNestedArray();
      for (int j = 0; j < 10; j++) {
        rowFuel.add(Fuel_table[i][j]);
        rowInj.add(Inj_table[i][j]);
      }
    }

    jsonDoc["highlightI"] = point_rpm;
    jsonDoc["highlightJ"] = point_tps;

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    ws.textAll(jsonString); 
  }

}

//////////////////ส่วนของฟังกชันภายนอกของเว็บ///////////////

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("ลูกค้าเชื่อมต่อ");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("ลูกค้ายกเลิกการเชื่อมต่อ");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      data[len] = 0;
      StaticJsonDocument<200> jsonDoc;
      deserializeJson(jsonDoc, (char*)data);

      const char* type = jsonDoc["type"];
      if (strcmp(type, "fuel") == 0 || strcmp(type, "inj") == 0) {
        int row = jsonDoc["row"];
        int col = jsonDoc["col"];
        int value = jsonDoc["value"];

        if (strcmp(type, "fuel") == 0) {
          Fuel_table[row][col] = value;
          editTableData("fuel", row, col);
        } else if (strcmp(type, "inj") == 0) {
          Inj_table[row][col] = value;
          editTableData("inj", row, col);
        }

        Serial.print("Updated ");
        Serial.print(type);
        Serial.print(" table at row ");
        Serial.print(row);
        Serial.print(", col ");
        Serial.print(col);
        Serial.print(" to ");
        Serial.println(value);

      } else if (strcmp(type, "stop") == 0) {
        check = false;
        Serial.println("Data stoped");
      } else if (strcmp(type, "play") == 0) {
        check = true;
        Serial.println("Data played");
      } else if (strcmp(type, "clear") == 0) {
        saveTableData();
        clearTableData();
        Serial.println("Data cleared");
      }
    }
  }
}


// ฟังก์ชันสำหรับการโหลดตารางจาก Preferences
void loadTableData() {
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      Fuel_table[i][j] = preferences.getInt(String("fuel_" + String(i) + "_" + String(j)).c_str(), Fuel_table[i][j]);
      Inj_table[i][j] = preferences.getInt(String("inj_" + String(i) + "_" + String(j)).c_str(), Inj_table[i][j]);
    }
  }
}

void editTableData(String type, int i, int j) {
  if (type == "fuel") preferences.putInt(String("fuel_" + String(i) + "_" + String(j)).c_str(), Fuel_table[i][j]);
  if (type == "inj") preferences.putInt(String("inj_" + String(i) + "_" + String(j)).c_str(), Inj_table[i][j]);
}

// ฟังก์ชันสำหรับการบันทึกตารางลง Preferences
void saveTableData() {
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      preferences.putInt(String("fuel_" + String(i) + "_" + String(j)).c_str(), Fuel_table[i][j]);
      preferences.putInt(String("inj_" + String(i) + "_" + String(j)).c_str(), Inj_table[i][j]);
    }
  }
}

// ฟังก์ชันสำหรับการลบข้อมูลใน Preferences
void clearTableData() {
  // ลบคีย์ทั้งหมดที่เกี่ยวข้องกับ Fuel_table และ Inj_table
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      preferences.remove(String("fuel_" + String(i) + "_" + String(j)).c_str());
      preferences.remove(String("inj_" + String(i) + "_" + String(j)).c_str());
    }
  }
}
