#include <Arduino.h>
#include <ledc.h>

#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#define U_PART U_SPIFFS

const char* host = "esp32";
const char* ssid = "ninja400";
const char* password = "12345678";
const char* PARAM_STRING = "inputString";
const char* PARAM_INT = "inputInt";
const char* PARAM_FLOAT = "inputFloat";

AsyncWebServer server(80);
size_t content_len;

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
  if(var == "inputString"){
    return readFile(SPIFFS, "/inputString.txt");
  }
  else if(var == "inputInt"){
    return readFile(SPIFFS, "/inputInt.txt");
  }
  else if(var == "inputFloat"){
    return readFile(SPIFFS, "/inputFloat.txt");
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

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button = {18, 0, false};
hw_timer_t * timer = NULL;
volatile bool FirstFall;
volatile uint64_t before_time = 0 , duty,duty_before ,freq;

void IRAM_ATTR Pin_isr() {
  if (!FirstFall)
  {
    timer = timerBegin(1, 8, true);
    FirstFall = true;
  }
  else
  {
    duty = timerRead(timer);
    timerEnd(timer);
    FirstFall = false;
    button.pressed = true;
  }
  
    
}

void setup() {
  Serial.begin(115200);

  pinMode(button.PIN, INPUT_PULLUP);
  attachInterrupt(button.PIN, Pin_isr, FALLING);

  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, 1, 12);
  ledcWriteTone(LEDC_CHANNEL_0,1);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  ledcSetup(1,1,12);
  ledcWriteTone(1,1);
  ledcAttachPin(LED_BUILTIN, 1);

  timer = timerBegin(1, 8, true);

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid);
  Serial.println("");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  char host[16];
#ifdef ESP8266
  snprintf(host, 12, "ESP%08X", ESP.getChipId());
#else
  snprintf(host, 16, "ESP%012llX", ESP.getEfuseMac());
#endif

  MDNS.begin(host);
  webInit();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("Ready! Open http://%s.local in your browser\n", host);
}

void loop() {
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

  if (Serial.available())
  {
    double_t setfreq = Serial.readStringUntil('\n').toDouble();
    ledcWriteTone(LEDC_CHANNEL_0,setfreq);
    ledcWriteTone(1,setfreq);
  }
/*
  if (duty_before != duty)
  {
    duty_before = duty;
    freq = 10000000 / Read_Freq_IN(duty,filter_buf_freg);
    Serial.printf("freq & duty %lld  : %lld \n", freq,duty);
    //delay(300);
  }
 */ 

}