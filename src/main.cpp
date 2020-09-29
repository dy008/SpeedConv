#include <Arduino.h>
#include <ledc.h>
#include <pcnt.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ESPmDNS.h>

#define U_PART U_SPIFFS
#define Ticker_Time   250   //定时周期 ms
#define Default_Freq  0 
#define Pluse_TO_ECU    19
#define Pluse_TO_SpeedMeter   26
#define Pluse_UPLIM   2000    // The Speed maybe over 270Km/h???

#define PCNT_TEST_UNIT      PCNT_UNIT_0   //使用计数器0通道
#define PCNT_H_LIM_VAL      10
#define PCNT_L_LIM_VAL     -32767
#define PCNT_THRESH1_VAL    0
#define PCNT_THRESH0_VAL   -0
#define PCNT_INPUT_SIG_IO   18  // Pulse Input GPIO

#define ECU_LEDC_CHANNEL  LEDC_CHANNEL_0
#define SPM_LEDC_CHANNEL  LEDC_CHANNEL_2

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

hw_timer_t * timer = NULL;
/* Prepare configuration for the PCNT unit */
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

const char* host = "esp32";
const char* ssid = "ninja400";
const char* password = "12345678";
const char* PARAM_ECU = "ECU_CV";   // 发往ECU的轮速脉冲系数
const char* PARAM_SPM = "SPM_CV";   // 发往码表的轮速脉冲系数
/* 445Hz = 60KM/h  即1km/h = 7.42个脉冲 */
const char* PARAM_ECU_HILIM = "ECU_HILIM";  // 发往ECU的轮速频率高限即国版ECU限速值 132km/h = 979Hz

AsyncWebServer server(80);
size_t content_len;
float freq, freq_before, ECU_Speed, SPM_Speed, ECU_CV, SPM_CV,ECU_HILIM;
int16_t pluse_number;   // Pluse Counter value
volatile bool Moto_Runing = false;    // Motorcycle is Running

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

void IRAM_ATTR onTimer(){
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i = 0;    
    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            PCNT.int_clr.val = BIT(i);
            }
        }
  portENTER_CRITICAL_ISR(&timerMux);
  Moto_Runing = true;
  portEXIT_CRITICAL_ISR(&timerMux);
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
    ECU Speed Coefficient( %ECU_CV% ): <input type="number " name="ECU_CV">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    SpeedMeter Speed Coefficient( %SPM_CV% ): <input type="number " name="SPM_CV">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    ECU Speed Limited( %ECU_HILIM% km/h): <input type="number " name="ECU_HILIM">
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
  if(var == "ECU_CV"){
    return readFile(SPIFFS, "/ECU_CV.txt");
  }
  else if(var == "SPM_CV"){
    return readFile(SPIFFS, "/SPM_CV.txt");
  }
  else if(var == "ECU_HILIM"){
    return readFile(SPIFFS, "/ECU_HILIM.txt");
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
    if (request->hasParam(PARAM_ECU)) {
      inputMessage = request->getParam(PARAM_ECU)->value();
      writeFile(SPIFFS, "/ECU_CV.txt", inputMessage.c_str());
      ECU_CV = inputMessage.toFloat();
    }
    // GET inputInt value on <ESP_IP>/get?inputInt=<inputMessage>
    else if (request->hasParam(PARAM_SPM)) {
      inputMessage = request->getParam(PARAM_SPM)->value();
      writeFile(SPIFFS, "/SPM_CV.txt", inputMessage.c_str());
      SPM_CV = inputMessage.toFloat();
    }
    // GET inputFloat value on <ESP_IP>/get?inputFloat=<inputMessage>
    else if (request->hasParam(PARAM_ECU_HILIM)) {
      inputMessage = request->getParam(PARAM_ECU_HILIM)->value();
      writeFile(SPIFFS, "/ECU_HILIM.txt", inputMessage.c_str());
      ECU_HILIM = inputMessage.toFloat() * 7.42;
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
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    //pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(3000);

  pinMode(PCNT_INPUT_SIG_IO, INPUT_PULLUP);

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  ECU_CV = readFile(SPIFFS, "/ECU_CV.txt").toFloat();
  SPM_CV = readFile(SPIFFS, "/SPM_CV.txt").toFloat();
  ECU_HILIM = readFile(SPIFFS, "/ECU_HILIM.txt").toFloat()  * 7.42;

// Setup timer and attach timer to a led pin
  ledcSetup(ECU_LEDC_CHANNEL, Default_Freq, 12);    // Initial OUT TO Engine Pin
  ledcWriteTone(ECU_LEDC_CHANNEL,Default_Freq);
  ledcAttachPin(Pluse_TO_ECU , ECU_LEDC_CHANNEL);
  ledcSetup(SPM_LEDC_CHANNEL,Default_Freq,12);    // Initial OUT TO SpeedMeter Pin
  ledcWriteTone(SPM_LEDC_CHANNEL,Default_Freq);
  ledcAttachPin(Pluse_TO_SpeedMeter, SPM_LEDC_CHANNEL);

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
  // timerAlarmEnable(timer);
  Serial.println("I'm Started... ");
}

void loop() {
  if (Moto_Runing)
  {
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);
    pcnt_intr_disable(PCNT_TEST_UNIT);
    pcnt_event_disable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_config.counter_h_lim = 32767;
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_resume(PCNT_TEST_UNIT);
           
    timerAlarmEnable(timer);        // Start Calc Motorcycle Speed
    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.printf("ESP32 Radio Shutdown... \n");
    portENTER_CRITICAL_ISR(&timerMux);
    Moto_Runing = false;
    portEXIT_CRITICAL_ISR(&timerMux);
  }

if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    if (pcnt_get_counter_value(PCNT_TEST_UNIT,&pluse_number) == ESP_OK)
    {
      pcnt_counter_pause(PCNT_TEST_UNIT);
      pcnt_counter_clear(PCNT_TEST_UNIT);
      pcnt_counter_resume(PCNT_TEST_UNIT);
      freq = pluse_number * (1000/Ticker_Time);
      if (freq >= Pluse_UPLIM)
      {         // Over Speed Limited
        freq = 2000;
      }      
      ECU_Speed = freq * ECU_CV;
      SPM_Speed = freq * SPM_CV;
      if (ECU_Speed > ECU_HILIM )
      {         // Over ECU Speed Limited ToDO
        ECU_Speed = ECU_HILIM;
      }
      // Output Speed Pluse...
      ledcWriteTone(ECU_LEDC_CHANNEL,ECU_Speed);
      ledcWriteTone(SPM_LEDC_CHANNEL,SPM_Speed);
      
      if (freq_before != freq)
      {
        freq_before = freq;
        Serial.printf("freq & ECU & SPM & HiLim %.1f %.1f1 %.1f %.1f \n", freq,ECU_Speed,SPM_Speed,ECU_HILIM);
      }
    }
  }
}