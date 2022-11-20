#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>



#define WIFI_TIMEOUT_MS 10000
const char* host = "esp32";
#define WIFI_NETWORK "Redmi Note 10"
#define WIFI_PASSWPRD "12345678"


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


WebServer server(80);



#define SENSOR_PIN  33 // ESP32 pin GIOP33 connected to DS18B20 sensor's DQ pin
#define analogPin 34  // ----Mcr Sensor Data pin 
#define power 27    //----- Mcr sencer powered by ESP G27 pin
#define heater 23    // -----Heating Element Control by ESP G14 Pin ( ON - OFF)
#define fan 19   // -----Heat circulation Fan Control by ESP G12 Pin ( ON - OFF)
#define bip 18  //  ---- bazar Alarm sound pin 
#define button 5 // ---- Machine on & off also a long press enebal OTA update


int radc = 0 ;
long int buf[20000], temp, timPS = 18000000, timR = 150000;
float secondFilter[500], m = 0;
int adc = 4095, interval = 45000;
const char* QLT;
int h = 0, s = 0, mint = 0, ms = 0;


float tempC; // temperature in Celsius
float tempF; // temperature in Fahrenheit


unsigned long previousMillis = 0, previousMillis1 = 0;
unsigned long currentMillis, currentMillis1 ;

// button section
// byte btn =0;
// byte ledState = LOW;


// button section


const unsigned char myBitmap [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xc0, 0x0f, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x01, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xc0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0x80, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xfc, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0f, 0xff, 0xc0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0x80, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);



const char* loginIndex =
  "<!DOCTYPE HTML>"
  "<html>"
  "<head>"
  " <meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='stylesheet' href='https://use.fontawesome.com/releases/v5.7.2/css/all.css' integrity='sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr' crossorigin='anonymous'>"

  "<style>"
  "html {"
  "font-family: Arial;"
  "display: inline-block;"
  "margin: 0px auto;"
  "text-align: center;"
  "}"
  " h3 { font-size: 2.2rem; }"


  "input{"
  "width:180px;"
  "padding:9px 25px;"
  "background-color:#34A56F;"
  "color:white;"
  "border-radius:5px;"
  "border:none;"
  "}"
  "input:hover{"
  "background-color:#87CEEB;"
  "color:black;"
  "}"



  "</style>"


  "</head>"
  "<body>"
  "<form name='loginForm'>"
  "<br><br><br>"
  "<h3>Welcome to Quick Curd Maker Project </h3>"
  "<br>"
  "<br>"
  "<br>"
  "<font size=4><b>Admin Login Page</b></font>"

  "<br>"
  "<br>"

  //  "<h5> Username:</h5>"
  "<spam id ='user'>Username:</spam>"
  "<input type='text' size=25 name='userid'>"
  "<br>"
  "<br>"
  "<spam id ='user'>Password:</spam>"
  // "<h5> Password:</h5>"
  "<input type='Password' size=25 name='pwd'><br>"
  "<br>"
  "<br>"

  "<input type='submit' onclick='check(this.form)' value='OTA Update'>"

  "</form>"

  // <!--this is javascript part-->

  "<script>"
  "function check(form)"
  "{"
  "if(form.userid.value=='amit' && form.pwd.value=='amit')"
  "{"
  "window.open('/serverIndex')"
  "}"
  "else"
  "{"
  " alert('Error Password or Username')/*displays error message*/"
  "}"
  "}"
  "</script>"


  "</body>"
  "</html>";



const char* serverIndex =
  "<style>"

  "h3 {font-size: 2.5rem;}"
  "dev {font-size: 2.5rem;}"

  "input{"
  "width:300px;"
  "padding:9px 25px;"
  "background-color:#34A56F;"
  "color:white;"
  "border-radius:5px;"
  "border:none;"
  "}"
  "input:hover{"
  "background-color:#87CEEB;"
  "color:black;"
  "}"
  "</style>"

  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
  "<center>"
  "<br>"
  "<br>"
  "<br>"
  "<br><br>"
  "<h3>ESP32 Curd Project New Firmware update</h3><br><br>"
  "<input type='file' name='update'><br>"
  "<br>"
  "<br>"
  "<input type='submit' id='ota' value='Update'>"
  "</form>"
  "<br>"
  "<br>"
  "<div id='prg'>progress: 0%</div>"
  "</center>"
  "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
  "},"
  "error: function (a, b, c) {"
  "}"
  "});"
  "});"
  "</script>";




void KeepWiFiAlive(void * parameters) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi is connected_-_-_-_-_-_-_-----_-");
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }
    Serial.print("Connecting WiFi....");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWPRD);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
      Serial.print(".");
      delay(100);
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(" WiFi Connect Failed ..");
      vTaskDelay(20000 / portTICK_PERIOD_MS);
      continue;
      // ESP.restart();
    }

    Serial.println( WiFi.localIP());
  }
}




void task2(void * parameters) {

  for (;;) {
    float m1 = 0, m2 = 0, large = 0, small = 0, ms = 0, ms1 = 0, allSum = 0, get = 0, sum1 = 0, mx = 0;
    long int sum = 0;


    digitalWrite(power, HIGH);

    // radc = analogRead(analogPin);
    for (int i = 0; i < 20000; i++) { //Get 10 sample value from the sensor for smooth the value
      buf[i] = analogRead(analogPin);
      sum += buf[i];
    }
    digitalWrite(power, LOW);
    mx = (float)sum / 20000;

    radc = mx;

    m1 = (adc / 100);
    m2 = radc / m1;
    for (int i = 0; i < 1000; i++) {
      secondFilter[i] = m2;
      sum1 = sum1 + secondFilter[i];

    }
    m = (float)sum1 / 1000;

    Serial.println(radc);
    Serial.print("MCR = %");// MILK  conductivity rate
    Serial.println(m);
    vTaskDelay(750 / portTICK_PERIOD_MS);
  }

}



void timeOver(void * parameters) {
  for (;;) {

    if (currentMillis >= timPS) {

      digitalWrite(heater, LOW);
      digitalWrite(fan, LOW);
      digitalWrite(bip, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(bip, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(bip, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(bip, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);

      Serial.println("off the total machine");

    }
    else {

      if (tempC >= 45.00) {
        currentMillis1 = millis();
        digitalWrite(heater, LOW);
        if (currentMillis1 - previousMillis1 >= 70000)
        {
          digitalWrite(fan, LOW);
          previousMillis1 = currentMillis1;
        }

      }
      else {
        digitalWrite(heater, HIGH);
        digitalWrite(fan, HIGH);
      }

      Serial.println("Runing the total machine");

    }

    ms = currentMillis;
    s = ms / 1000;
    mint = s / 60;
    h = mint / 60;

    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }

}



// void buttonPress(void * parameters){
//   for(;;){
//       if (millis() - lastTimeButtonStateChanged > debounceDuration) {
//       byte buttonState = digitalRead(button);
//       if (buttonState != lastButtonState) {
//         lastTimeButtonStateChanged = millis();
//         lastButtonState = buttonState;
//         if (buttonState == LOW) {
//           ledState = (ledState == HIGH) ? LOW: HIGH;
//           digitalWrite(heater, ledState);
//           digitalWrite(fan, ledState);
//           digitalWrite(bip, ledState);
//           digitalWrite(LED_BUILTIN, ledState);
//         }
//       }
//     }

//   }
// }





WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "in.pool.ntp.org", 19800, 60000);
// NTPClient(UDP& udp, const char* poolServerName, long timeOffset, unsigned long updateInterval);
char dayWeek [7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};






void setup() {
  Serial.begin(115200);
  Serial.println("Ready....");
  DS18B20.begin();    // initialize the DS18B20 sensor
  previousMillis = millis();
  previousMillis1 = millis();


  xTaskCreatePinnedToCore(
    KeepWiFiAlive,
    "Wifi Connecte",
    6000,
    NULL,
    2,
    NULL,
    0
  );


  xTaskCreatePinnedToCore(
    task2,
    " Taking MCR Value ",         // you can assigen RTOS for DBuging purpuse
    2550,                // tell us , how much byts it can use, for effecient esp Ram (Stske size)
    NULL,                // task paramitors assigen
    3,                         // task Priority
    NULL,                    //   task handels
    CONFIG_ARDUINO_RUNNING_CORE  // Define Which CPU Core You want to Use (if Dual core then ethor 0 Or 1)

  );



  xTaskCreatePinnedToCore(
    timeOver,
    "Time Over",
    1000,
    NULL,
    1,
    NULL,
    CONFIG_ARDUINO_RUNNING_CORE
  );



  // xTaskCreatePinnedToCore(
  //   buttonPress,
  //   " Curd Machine ON & OFF ",         // you can assigen RTOS for DBuging purpuse
  //   1000,                // tell us , how much byts it can use, for effecient esp Ram (Stske size)
  //   NULL,                // task paramitors assigen
  //   1,                         // task Priority
  //   NULL,                    //   task handels
  //   CONFIG_ARDUINO_RUNNING_CORE  // Define Which CPU Core You want to Use (if Dual core then ethor 0 Or 1)

  // );



  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  // server.on("/", HTTP_GET, []() {
  //   server.sendHeader("Connection", "close");
  //   server.send(200, "text/html", firstPageIndex);
  //   });
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();



  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(50);
  display.clearDisplay();
  delay(10);

  pinMode(power, OUTPUT);
  pinMode(heater, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(bip, OUTPUT);
  pinMode(button, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  display.drawBitmap(0, 0, myBitmap, 128, 64, WHITE);
  display.display() ;
  delay(1000);
  display.clearDisplay();


}

void loop() {

  // put your main code here, to run repeatedly:
  DS18B20.requestTemperatures();       // send the command to get temperatures
  tempC = DS18B20.getTempCByIndex(0);  // read temperature in °C
  tempF = tempC * 9 / 5 + 32; // convert °C to °F



  if ( m >= 3.5 && m <= 6.5) {

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("non recognise value");
    Serial.println("non recognise value");
  }
  else if ( m >= 6.6 && m <= 11.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("Pure+ mineral water");
    Serial.println("Pure+ mineral water");
  }
  else if ( m >= 11.6 && m <= 24.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("Pure mineral water");
    Serial.println("Pure mineral water");
  }
  else if ( m >= 24.6 && m <= 33.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("Mineral water");
    Serial.println("Mineral water");
  }
  else if ( m >= 33.6 && m <= 39.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's normal water");
    Serial.println("It's normal water");
  }
  else if ( m >= 39.5 && m <= 46.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade D Milk");
    Serial.println("It's grade D Milk");
  }
  else if ( m >= 46.6 && m <= 50.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade C Milk");
    Serial.println("It's grade C Milk");
  }
  else if ( m >= 50.6 && m <= 56.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade B Milk");
    Serial.println("It's grade B Milk");
  }
  else if ( m >= 56.6 && m <= 62.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade B+ Milk");
    Serial.println("It's grade B+ Milk");
  }
  else if ( m >= 62.6 && m <= 67.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade A Milk");
    Serial.println("It's grade A Milk");
  }
  else if ( m >= 67.6 && m <= 72.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade A+ Milk");
    Serial.println("It's grade A+ Milk");
  }
  else if ( m >= 72.6 && m <= 80.5) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's grade A++ Milk");
    Serial.println("It's grade A++ Milk");
  }
  else if ( m >= 80.6 ) {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("It's solt water");
    Serial.println("It's solt water");
  }
  else {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println("Dip the sensor");
    Serial.println("Dip the sensor");

  }


  Serial.println(WiFi.localIP());

  currentMillis = millis();

  Serial.print("Hour: ");
  Serial.println(h);
  Serial.print("Minits: ");
  Serial.println(mint);
  Serial.print(dayWeek[timeClient.getDay()]);
  Serial.print(" ");
  Serial.println(timeClient.getFormattedTime());
  digitalWrite(LED_BUILTIN, HIGH);

  server.handleClient();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("MCR..=");// MILK  conductivity rate
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(44, 0);
  display.print(m);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(82, 0);
  display.println("%");
  // display tempreture ....... section.
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Tempe. =");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(50, 10);
  display.print(tempC);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(95, 10);
  display.println("C");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println("IP=");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(25, 20);
  display.println(WiFi.localIP());
  // QLT -- milk QUALATY
  // -------  < NTP Server Time and Days name > -------
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 50);
  display.println(dayWeek[timeClient.getDay()]);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(60, 50);
  display.println(timeClient.getFormattedTime());
  // -----  <Time stopWatch Display section > --------
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 30);
  display.println("Time: ");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(34, 30);
  display.println("H:");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(47, 30);
  display.println(h);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(62, 30);
  display.println("M:");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(75, 30);
  display.println(mint);


  display.display() ;

  display.clearDisplay();

  delay(50);
  digitalWrite(LED_BUILTIN, LOW);



}
