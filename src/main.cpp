#include <Arduino.h>
#include <ledc.h>
#include <pcnt.h>

#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#define U_PART U_SPIFFS
#define Ticker_Time   2000   //定时周期 ms

hw_timer_t * timer = NULL;
/* Prepare configuration for the PCNT unit */
pcnt_config_t pcnt_config;

const char* host = "esp32";
const char* ssid = "ninja400";
const char* password = "12345678";
const char* PARAM_STRING = "inputString";
const char* PARAM_INT = "inputInt";
const char* PARAM_FLOAT = "inputFloat";
const char* PARAM_FREQ = "FreqSet";

AsyncWebServer server(80);
size_t content_len;
volatile float duty,duty_before ,freq, FreqSet;
int16_t *pluse_number;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR Handle_PCNT(void){
  portENTER_CRITICAL_ISR(&timerMux);
  pcnt_get_counter_value(PCNT_UNIT_0,pluse_number);
  portEXIT_CRITICAL_ISR(&timerMux);
  //pcnt_counter_pause(PCNT_UNIT_0);
  //pcnt_counter_clear(PCNT_UNIT_0);
    /* Register ISR handler and enable interrupts for PCNT unit */
    //pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    //pcnt_intr_enable(PCNT_UNIT);

    /* Everything is set up, now go to counting */
    //pcnt_counter_resume(PCNT_UNIT_0);
    //freq = *pluse_number * 5; 
    //duty = 1 / freq;
}
// HTML web page to handle 3 input fields (inputString, inputInt, inputFloat)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP32 Input & OTA Upgrade</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  </form><br>
  <form action="/get" target="hidden-form">
    Freq(%Freq% Hz) : Duty(%Duty% uS) : <input type="number " name="FreqSet">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    inputString (%inputString%): <input type="text" name="inputString">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    inputInt (%inputInt%): <input type="number " name="inputInt">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    inputFloat (%inputFloat%): <input type="number " name="inputFloat">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  </form><br>
  <form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='OTA Upgrade'></form>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  char outstr[15];
  if(var == "inputString"){
    return readFile(SPIFFS, "/inputString.txt");
  }
  else if(var == "inputInt"){
    return readFile(SPIFFS, "/inputInt.txt");
  }
  else if(var == "inputFloat"){
    return readFile(SPIFFS, "/inputFloat.txt");
  }else if (var == "Freq")
  {
    dtostrf(freq,10,0,outstr);
    return outstr;
  }else if (var == "Duty")
  {
    dtostrf(duty/10,11,1,outstr);
    return outstr;
  }
  
  return String();
}

void handleRoot(AsyncWebServerRequest *request) {
  request->redirect("/update");
}

void handleUpdate(AsyncWebServerRequest *request) {
  //char* html = "<form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  request->send_P(200, "text/html", index_html,processor);
}

void handleDoUpdate(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index){
    Serial.println("Update");
    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_PART : U_FLASH;
#ifdef ESP8266
    Update.runAsync(true);
    if (!Update.begin(content_len, cmd)) {
#else
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
#endif
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
#ifdef ESP8266
  } else {
    Serial.printf("Progress: %d%%\n", (Update.progress()*100)/Update.size());
#endif
  }

  if (final) {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
    response->addHeader("Refresh", "20");  
    response->addHeader("Location", "/");
    request->send(response);
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      Serial.println("Update complete");
      Serial.flush();
      ESP.restart();
    }
  }
}

void printProgress(size_t prg, size_t sz) {
  Serial.printf("Progress: %d%%\n", (prg*100)/content_len);
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void webInit() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {request->redirect("/update");});
  
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){handleUpdate(request);});
  server.on("/doUpdate", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    [](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data,
                  size_t len, bool final) {handleDoUpdate(request, filename, index, data, len, final);}
  );
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputString value on <ESP_IP>/get?inputString=<inputMessage>
    if (request->hasParam(PARAM_STRING)) {
      inputMessage = request->getParam(PARAM_STRING)->value();
      writeFile(SPIFFS, "/inputString.txt", inputMessage.c_str());
    }
    // GET inputInt value on <ESP_IP>/get?inputInt=<inputMessage>
    else if (request->hasParam(PARAM_INT)) {
      inputMessage = request->getParam(PARAM_INT)->value();
      writeFile(SPIFFS, "/inputInt.txt", inputMessage.c_str());
    }
    // GET inputFloat value on <ESP_IP>/get?inputFloat=<inputMessage>
    else if (request->hasParam(PARAM_FLOAT)) {
      inputMessage = request->getParam(PARAM_FLOAT)->value();
      writeFile(SPIFFS, "/inputFloat.txt", inputMessage.c_str());
    }
    else if (request->hasParam(PARAM_FREQ)) {
      inputMessage = request->getParam(PARAM_FREQ)->value();
      FreqSet = inputMessage.toInt();
      ledcWriteTone(1,FreqSet);
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});
  server.begin();
#ifdef ESP32
  Update.onProgress(printProgress);
#endif
}



// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            26
#define PLUSE_PIN          18

/* Initialize PCNT functions for one channel:
 *  - configure and initialize PCNT with pos-edge counting 
 *  - set up the input filter
 *  - set up the counter events to watch
 * Variables:
 * UNIT - Pulse Counter #, INPUT_SIG - Signal Input Pin, INPUT_CTRL - Control Input Pin,
 * Channel - Unit input channel, H_LIM - High Limit, L_LIM - Low Limit,
 * THRESH1 - configurable limit 1, THRESH0 - configurable limit 2, 
 */
void pcnt_init_channel(pcnt_unit_t PCNT_UNIT,int PCNT_INPUT_SIG_IO , int PCNT_INPUT_CTRL_IO = PCNT_PIN_NOT_USED,pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0, int PCNT_H_LIM_VAL = 32767, int PCNT_L_LIM_VAL = 0, int PCNT_THRESH1_VAL = 50, int PCNT_THRESH0_VAL = -50 ) {
         // Set PCNT input signal and control GPIOs
        pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
        pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
        pcnt_config.channel = PCNT_CHANNEL;
        pcnt_config.unit = PCNT_UNIT;
        // What to do on the positive / negative edge of pulse input?
        pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
        pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        pcnt_config.lctrl_mode = PCNT_MODE_KEEP; // Reverse counting direction if low
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
        pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;
    
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT, 20);
    pcnt_filter_enable(PCNT_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    /* Register ISR handler and enable interrupts for PCNT unit */
    //pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    //pcnt_intr_enable(PCNT_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT);
}

#define FILTER_N 4  // 递推平均滤波法（又称滑动平均滤波法）计算次数

uint64_t filter_buf_freg[FILTER_N + 1];

uint64_t Read_Freq_IN(uint64_t in,uint64_t filter_buf[FILTER_N]){    // 递推平均值AD转换算法 返回电压值
  int i;
  uint64_t filter_sum = 0;
  filter_buf[FILTER_N] = in;
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
    filter_sum += filter_buf[i];
  }
  return ((filter_sum / FILTER_N));
}


void setup() {
  Serial.begin(115200);

  pinMode(PLUSE_PIN, INPUT_PULLUP);

  // Setup timer and attach timer to a led pin
  /*
  ledcSetup(LEDC_CHANNEL_0, 1, 12);
  ledcWriteTone(LEDC_CHANNEL_0,1);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  ledcSetup(1,1,12);
  ledcWriteTone(1,1);
  ledcAttachPin(LED_BUILTIN, 1);


  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  */
  // Remove the password parameter, if you want the AP (Access Point) to be open
  //WiFi.softAP(ssid);
  Serial.println("");

  //IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  //Serial.println(IP);

  char host[16];
#ifdef ESP8266
  snprintf(host, 12, "ESP%08X", ESP.getChipId());
#else
  snprintf(host, 16, "ESP%012llX", ESP.getEfuseMac());
#endif

  //MDNS.begin(host);
  //webInit();
  //MDNS.addService("http", "tcp", 80);
  //Serial.printf("Ready! Open http://%s.local in your browser\n", host);

  //pcnt_init_channel(PCNT_UNIT_0,PLUSE_PIN); // Initialize Unit 0 to pin PLUSE_PIN
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &Handle_PCNT, true);
  timerAlarmWrite(timer, 5000000, true);
  timerAlarmEnable(timer);
}

void loop() {
  /*
  // To access your stored values on inputString, inputInt, inputFloat
  String yourInputString = readFile(SPIFFS, "/inputString.txt");
  Serial.print("*** Your inputString: ");
  Serial.println(yourInputString);
  
  int yourInputInt = readFile(SPIFFS, "/inputInt.txt").toInt();
  Serial.print("*** Your inputInt: ");
  Serial.println(yourInputInt);
  
  float yourInputFloat = readFile(SPIFFS, "/inputFloat.txt").toFloat();
  Serial.print("*** Your inputFloat: ");
  Serial.println(yourInputFloat);
  delay(5000);
*/
  if (Serial.available())
  {
    FreqSet = Serial.readStringUntil('\n').toDouble();
    ledcWriteTone(LEDC_CHANNEL_0,FreqSet);
    ledcWriteTone(1,FreqSet);

  }

  if (duty_before != duty)
  {
    duty_before = duty;
    //freq = 10000000 / Read_Freq_IN(duty,filter_buf_freg);
    //Serial.printf("freq & duty %f  : %f \n", freq,duty);
    //delay(300);
  }


}