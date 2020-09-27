#include <Arduino.h>
#include <ledc.h>
#include <pcnt.h>

#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#define U_PART U_SPIFFS
#define Ticker_Time   1000   //定时周期 ms
#define Default_Freq  0 
#define LED_PIN            26
#define PLUSE_PIN          18

#define PCNT_TEST_UNIT      PCNT_UNIT_0   //使用计数器0通道
#define PCNT_H_LIM_VAL      32767
#define PCNT_L_LIM_VAL     -32767
#define PCNT_THRESH1_VAL    0
#define PCNT_THRESH0_VAL   -0
#define PCNT_INPUT_SIG_IO   18  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  18  // Control GPIO HIGH=count up, LOW=count down

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
float duty,duty_before ,freq, freq_before, FreqSet;
int16_t pluse_number;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
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
    dtostrf(duty,11,1,outstr);
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
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
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
      ledcWriteTone(LEDC_CHANNEL_0,FreqSet);
      ledcWriteTone(LEDC_CHANNEL_1,FreqSet);
      writeFile(SPIFFS, "/FreqSet.txt", inputMessage.c_str());
    }
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404);});
  server.begin();
  Update.onProgress(printProgress);
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_init(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED ,
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = PCNT_TEST_UNIT,
        .channel = PCNT_CHANNEL_0,        
        // What to do when control input is low or high?       
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 20);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    //pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    //pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    //pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    //pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}


#define FILTER_N 4  // 递推平均滤波法（又称滑动平均滤波法）计算次数

float filter_buf_freg[FILTER_N + 1];

float Read_Freq_IN(float in,float filter_buf[FILTER_N]){    // 递推平均值AD转换算法 返回电压值
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

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  FreqSet = readFile(SPIFFS, "/FreqSet.txt").toFloat();
// Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, Default_Freq, 12);
  ledcWriteTone(LEDC_CHANNEL_0,FreqSet);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  ledcSetup(LEDC_CHANNEL_1,Default_Freq,12);
  ledcWriteTone(LEDC_CHANNEL_1,FreqSet);
  ledcAttachPin(LED_BUILTIN, LEDC_CHANNEL_1);

  //Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  char host[16];
  snprintf(host, 16, "ESP%012llX", ESP.getEfuseMac());

  MDNS.begin(host);
  webInit();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("Ready! Open http://%s.local in your browser\n", host);

  pcnt_init(); // Initialize Unit 0 to pin PLUSE_PIN
  
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, Ticker_Time * 1000, true);
  // Start an alarm
  timerAlarmEnable(timer);
  Serial.println("I'm Started... ");
}

void loop() {
  if (Serial.available())
  {
    FreqSet = Serial.readStringUntil('\n').toFloat();
    ledcWriteTone(LEDC_CHANNEL_0,FreqSet);
    ledcWriteTone(LEDC_CHANNEL_1,FreqSet);
  }

if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);

    if (pcnt_get_counter_value(PCNT_TEST_UNIT,&pluse_number) == ESP_OK)
    {
      pcnt_counter_pause(PCNT_TEST_UNIT);
      pcnt_counter_clear(PCNT_TEST_UNIT);
      pcnt_counter_resume(PCNT_TEST_UNIT);
      freq = pluse_number * (1000/Ticker_Time);
      duty = 1000000 / freq;
      while (duty_before != duty)
      {
        duty_before = duty;
        Serial.printf("freq & duty %f - %f \n", freq,duty);
      }
    }
  }

}