/*------------------------------------------------------------------------------
  16/02/2020
  Author: DKJONAS website https://github.com/dkjonas/Wavin-AHC-9000-mqtt
  Platforms: ESP8266
  Language: C++/Arduino
  ------------------------------------------------------------------------------
  Description: 

  This code provides an gateway to use with a Wavin AHC-9000 Floor Heating System. 
  The code sends data over MQTT, so you are able to use it with most Smart-Home platforms.
  This code / system runs very good on Home Assistant https://www.home-assistant.io

  ------------------------------------------------------------------------------
  Usage:

  The first time you flash this code, you will see an access point named "WavinAP".
  Connect to this network and follow the config-portal and fill out the forms.
  
  For more info from here look at the docs on the mentioned repositories.
  ------------------------------------------------------------------------------
  Thanks to these awesome people who helped with this code:

  https://github.com/heinekmadsen     
  https://github.com/dkjonas      
  https://github.com/paller

  ------------------------------------------------------------------------------
  License:
  Please see attached LICENSE.txt file for details.
  ------------------------------------------------------------------------------*/

#include <FS.h>
#include <DNSServer.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <PubSubClientTools.h>
#include <ESP32WebServer.h>
#include <ESP_WiFiManager.h>
#include <ESP_DoubleResetDetector.h>
#include "WavinController.h"
#include <Ticker.h>
#include "SPIFFS.h"





/*------------------------------------------------------------------------------ 
TICKER SETUP - USED FOR SHOWING IF ESP IS IN CONFIG MODE OR NOT
------------------------------------------------------------------------------*/
String Router_SSID;
String Router_Pass; 
String ssid = "ESP_" + String(ESP_getChipId(), HEX);
Ticker ticker;
 
void tick()
{
  //toggle state
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}
 
//gets called when WiFiManager enters configuration mode
void configModeCallback (ESP_WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

/*------------------------------------------------------------------------------ 
DOUBLE RESET DETECTOR
------------------------------------------------------------------------------*/

// Number of seconds after reset during which a 
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

DoubleResetDetector* drd;

bool initialConfig = false;
/*------------------------------------------------------------------------------ 
// MQTT / WIFI / HOSTNAME DEFINES
------------------------------------------------------------------------------*/

// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/3/target', where 3 is the output id and XXXXXXXXXXXX is the mac
const String   MQTT_PREFIX              = "heat/";       // include tailing '/' in prefix
const String   MQTT_DEVICE_NAME         = "floor";       // only alfanumeric and no '/'
const String   MQTT_ONLINE              = "/online";      
const String   MQTT_SUFFIX_CURRENT      = "/current";    // include heading '/' in all suffixes
const String   MQTT_SUFFIX_SETPOINT_GET = "/target";
const String   MQTT_SUFFIX_SETPOINT_SET = "/target_set";
const String   MQTT_SUFFIX_MODE_GET     = "/mode";
const String   MQTT_SUFFIX_MODE_SET     = "/mode_set";
const String   MQTT_SUFFIX_BATTERY      = "/battery";
const String   MQTT_SUFFIX_OUTPUT       = "/output";

const String   MQTT_VALUE_MODE_STANDBY  = "off";
const String   MQTT_VALUE_MODE_MANUAL   = "heat";

String   MQTT_CLIENT        = "Wavin-AHC-9000-mqtt"; // mqtt client_id prefix. Will be suffixed with Esp8266 mac to make it unique

char     MQTT_PORT[6]       = "1883";                // mqtt port = 1883 (Most used port)
String   WIFI_SSID          = "";                    // wifi ssid
String   WIFI_PASS          = "";                    // wifi password

String   MQTT_SERVER        = "";                    // mqtt server address without port number
String   MQTT_USER          = "";                    // mqtt user. Use "" for no username
String   MQTT_PASS          = "";                    // mqtt password. Use "" for no password

/*------------------------------------------------------------------------------ 
FLAGS AND CALLBACK FOR WIFIMANAGER
------------------------------------------------------------------------------*/

//flag for saving data
bool shouldSaveConfig = true;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/*------------------------------------------------------------------------------ 
CONTROL OF WAVIN
------------------------------------------------------------------------------*/

String mqttDeviceNameWithMac;
String mqttClientWithMac;

// Operating mode is controlled by the MQTT_SUFFIX_MODE_ topic.
// When mode is set to MQTT_VALUE_MODE_MANUAL, temperature is set to the value of MQTT_SUFFIX_SETPOINT_
// When mode is set to MQTT_VALUE_MODE_STANDBY, the following temperature will be used
const float STANDBY_TEMPERATURE_DEG = 5.0;

const uint8_t TX_ENABLE_PIN = 13;
const uint16_t RECIEVE_TIMEOUT_MS = 1000;
WavinController wavinController(TX_ENABLE_PIN, RECIEVE_TIMEOUT_MS);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastUpdateTime = 0;

const uint16_t POLL_TIME_MS = 5000;

struct lastKnownValue_t {
  uint16_t temperature;
  uint16_t setpoint;
  uint16_t battery;
  uint16_t status;
  uint16_t mode;
} lastSentValues[WavinController::NUMBER_OF_CHANNELS];

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;

bool configurationPublished[WavinController::NUMBER_OF_CHANNELS];


// Read a float value from a non zero terminated array of bytes and
// return 10 times the value as an integer
uint16_t temperatureFromString(String payload)
{
  float targetf = payload.toFloat();
  return (unsigned short)(targetf * 10);
}


// Returns temperature in degrees with one decimal
String temperatureAsFloatString(uint16_t temperature)
{
  float temperatureAsFloat = ((float)temperature) / 10;
  return String(temperatureAsFloat, 1);
}


uint8_t getIdFromTopic(char* topic)
{
  unsigned int startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  int i = 0;
  uint8_t result = 0;

  while(topic[startIndex+i] != '/' && i<3)
  {
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }

  return result;
}


void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String topicString = String(topic);
  
  char terminatedPayload[length+1];
  for(unsigned int i=0; i<length; i++)
  {
    terminatedPayload[i] = payload[i];
  }
  terminatedPayload[length] = 0;
  String payloadString = String(terminatedPayload);

  uint8_t id = getIdFromTopic(topic);

  if(topicString.endsWith(MQTT_SUFFIX_SETPOINT_SET))
  {
    uint16_t target = temperatureFromString(payloadString);
    wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, id, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, target);
  }
  else if(topicString.endsWith(MQTT_SUFFIX_MODE_SET))
  {
    if(payloadString == MQTT_VALUE_MODE_MANUAL) 
    {
      wavinController.writeMaskedRegister(
        WavinController::CATEGORY_PACKED_DATA,
        id,
        WavinController::PACKED_DATA_CONFIGURATION,
        WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL,
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
    else if (payloadString == MQTT_VALUE_MODE_STANDBY)
    {
      wavinController.writeMaskedRegister(
        WavinController::CATEGORY_PACKED_DATA, 
        id, 
        WavinController::PACKED_DATA_CONFIGURATION, 
        WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY, 
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
  }

  // Force re-read of registers from controller now
  lastUpdateTime = 0;

}


void resetLastSentValues()
{
  for(int8_t i=0; i<WavinController::NUMBER_OF_CHANNELS; i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;
    lastSentValues[i].mode = LAST_VALUE_UNKNOWN;

    configurationPublished[i] = false;
  }
}


void publishIfNewValue(String topic, String payload, uint16_t newValue, uint16_t *lastSentValue)
{
  if (newValue != *lastSentValue)
  {
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true))
    {
        *lastSentValue = newValue;
    }
    else
    {
      *lastSentValue = LAST_VALUE_UNKNOWN;
    }
  }
}


// Publish discovery messages for HomeAssistant
// See https://www.home-assistant.io/docs/mqtt/discovery/
void publishConfiguration(uint8_t channel)
{
  String climateTopic = String("homeassistant/climate/" + mqttDeviceNameWithMac + "/" + channel + "/config");
  String climateMessage = String(
    "{\"name\": \"" +mqttDeviceNameWithMac + "_" + channel +  "_climate\", "
    "\"current_temperature_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_CURRENT + "\", " 
    "\"temperature_command_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_SET + "\", " 
    "\"temperature_state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_GET + "\", " 
    "\"mode_command_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_SET + "\", " 
    "\"mode_state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_GET + "\", " 
    "\"modes\": [\"" + MQTT_VALUE_MODE_MANUAL + "\", \"" + MQTT_VALUE_MODE_STANDBY + "\"], " 
    "\"availability_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE +"\", "
    "\"payload_available\": \"True\", "
    "\"payload_not_available\": \"False\", "
    "\"qos\": \"0\"}"
  );
  
  String batteryTopic = String("homeassistant/sensor/" + mqttDeviceNameWithMac + "/" + channel + "/config");
  String batteryMessage = String(
    "{\"name\": \"" +mqttDeviceNameWithMac + "_" + channel +  "_battery\", "
    "\"state_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/battery\", " 
    "\"availability_topic\": \"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE +"\", "
    "\"payload_available\": \"True\", "
    "\"payload_not_available\": \"False\", "
    "\"device_class\": \"battery\", "
    "\"unit_of_measurement\": \"%\", "
    "\"qos\": \"0\"}"
  );

  mqttClient.publish(climateTopic.c_str(), climateMessage.c_str(), true);  
  mqttClient.publish(batteryTopic.c_str(), batteryMessage.c_str(), true);
  
  configurationPublished[channel] = true;
}

/*------------------------------------------------------------------------------ 
SETUP
------------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  //set led pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //clean FS, for testing
 //read configuration from FS json
  Serial.println("mounting FS...");
  char c_MQTT_CLIENT[40] = "";
  char c_MQTT_SERVER[40] = "";
  char c_MQTT_USER[40] = "";
  char c_MQTT_PASS[40] = "";
  
  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        //DynamicJsonBuffer jsonBuffer;
        //JsonObject& json = jsonBuffer.parseObject(buf.get());
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        //json.printTo(Serial);
        serializeJson(json, Serial);
        //if (json.success()) {
        if (!deserializeError)
        {
          Serial.println("\nparsed json");

          strcpy(c_MQTT_CLIENT, json["mqtt_client"]);
          strcpy(c_MQTT_SERVER, json["mqtt_server"]);
          strcpy(MQTT_PORT, json["mqtt_port"]);
          strcpy(c_MQTT_USER, json["mqtt_user"]);
          strcpy(c_MQTT_PASS, json["mqtt_pass"]);
          

          MQTT_CLIENT = String(c_MQTT_CLIENT);
          MQTT_SERVER = String(c_MQTT_SERVER);
          MQTT_USER = String(c_MQTT_USER);
          MQTT_PASS = String(c_MQTT_PASS);
          

        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // Custom Parameters
  ESP_WMParameter custom_mqtt_client("client", "SET MQTT CLIENT NAME",c_MQTT_CLIENT, 40);
  ESP_WMParameter custom_mqtt_server("server", "MQTT SERVER",c_MQTT_SERVER, 40);
  ESP_WMParameter custom_mqtt_port("port", "MQTT PORT", MQTT_PORT, 6);
  ESP_WMParameter custom_mqtt_user("user", "MQTT USER - Leave blank if not present", c_MQTT_USER, 40);
  ESP_WMParameter custom_mqtt_pass("pass", "MQTT PASS - Leave Blank if not present", c_MQTT_PASS, 40);
  

  ESP_WiFiManager ESP_wifiManager;

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  ESP_wifiManager.setAPCallback(configModeCallback);

  //set config save notify callback
  ESP_wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  //add all your parameters here
  ESP_wifiManager.addParameter(&custom_mqtt_client);
  ESP_wifiManager.addParameter(&custom_mqtt_server);
  ESP_wifiManager.addParameter(&custom_mqtt_port);
  ESP_wifiManager.addParameter(&custom_mqtt_user);
  ESP_wifiManager.addParameter(&custom_mqtt_pass);

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  // SSID to uppercase
  ssid.toUpperCase();

 if (Router_SSID != "")
  {
    ESP_wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.
    Serial.println("Got stored Credentials. Timeout 60s");
  }
  else
  {
    Serial.println("No stored Credentials. No timeout");
    initialConfig = true;
  }
  // Detect Double reset to initiate the Captive portal for configuration
  if (drd->detectDoubleReset())  
    {
      Serial.println("Double Reset Detected");
      ESP_wifiManager.startConfigPortal("WavinAP");
    } 
  else 
  {
    Serial.println("No Double Reset Detected");
    if (!ESP_wifiManager.autoConnect("WavinAP")) {
      Serial.println("failed to connect and hit timeout");
      delay(5000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
      //keep LED on
     digitalWrite(LED_BUILTIN, HIGH); 
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  ticker.detach();
  
  //read updated parameters
  strcpy(c_MQTT_CLIENT, custom_mqtt_client.getValue());
  strcpy(c_MQTT_SERVER, custom_mqtt_server.getValue());
  strcpy(MQTT_PORT, custom_mqtt_port.getValue());
  strcpy(c_MQTT_USER, custom_mqtt_user.getValue());
  strcpy(c_MQTT_PASS, custom_mqtt_pass.getValue());
  
  MQTT_CLIENT = String(c_MQTT_CLIENT);
  MQTT_SERVER = String(c_MQTT_SERVER);
  MQTT_USER = String(c_MQTT_USER);
  MQTT_PASS = String(c_MQTT_PASS);

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    Serial.println("saving config");
    //DynamicJsonBuffer jsonBuffer;
    //JsonObject& json = jsonBuffer.createObject();
    DynamicJsonDocument json(1024);

    json["mqtt_client"] = c_MQTT_CLIENT;
    json["mqtt_server"] = c_MQTT_SERVER;
    json["mqtt_port"] = MQTT_PORT;
    json["mqtt_user"] = c_MQTT_USER;
    json["mqtt_pass"] = c_MQTT_PASS;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }

    //json.printTo(Serial);
    serializeJson(json, Serial);
    //json.printTo(configFile);
    serializeJson(json, configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());  //SPIFFS.format();

  //ESP_WiFiManager with custom parameters

 


  //Default
  uint8_t mac[6];
  WiFi.macAddress(mac);

  char macStr[13] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  mqttDeviceNameWithMac = String(MQTT_DEVICE_NAME + macStr);
  mqttClientWithMac = String(MQTT_CLIENT + macStr);

  std::string s_mqtt_port = MQTT_PORT;
  uint16_t i_mqtt_port = atoi(s_mqtt_port.c_str());

  mqttClient.setServer(MQTT_SERVER.c_str(), i_mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
ArduinoOTA.setHostname("WAVIN-GATEWAY");
ArduinoOTA.setPort(3232);

   ArduinoOTA.onStart([]() {
     String type;
     if (ArduinoOTA.getCommand() == U_FLASH) {
       type = "sketch";
     } else { // SPIFFS
       type = "filesystem";
     }

     // Print to USB instead of Wavin
     //Serial.swap();

     // NOTE: if updating FS this would be the place to unmount FS using FS.end()
     Serial.println("Start updating " + type);
   });

     ArduinoOTA.onEnd([]() {
     Serial.println("\nEnd");
   });

   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
   });

   ArduinoOTA.onError([](ota_error_t error) {
     Serial.printf("Error[%u]: ", error);
     if (error == OTA_AUTH_ERROR) {
       Serial.println("Auth Failed");
     } else if (error == OTA_BEGIN_ERROR) {
       Serial.println("Begin Failed");
     } else if (error == OTA_CONNECT_ERROR) {
       Serial.println("Connect Failed");
     } else if (error == OTA_RECEIVE_ERROR) {
       Serial.println("Receive Failed");
     } else if (error == OTA_END_ERROR) {
       Serial.println("End Failed");
     }
   });

   ArduinoOTA.begin();
  
 }

/*------------------------------------------------------------------------------ 
LOOP
------------------------------------------------------------------------------*/

void loop()
{
  ArduinoOTA.handle();
  drd->stop();
  //MySerial.begin(38400);
  if (WiFi.status() != WL_CONNECTED)
  {

    if (WiFi.waitForConnectResult() != WL_CONNECTED) return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {

     // OTA
     ArduinoOTA.handle();

    //Serial.println("Wifi OK, checking mqtt...");
    if (!mqttClient.connected())
    {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
      if (mqttClient.connect(mqttClientWithMac.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str(), will.c_str(), 1, true, "False") )
      {
          Serial.println("Connected to mqtt");
          String setpointSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_SETPOINT_SET);
          mqttClient.subscribe(setpointSetTopic.c_str(), 1);
          
          String modeSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_MODE_SET);
          mqttClient.subscribe(modeSetTopic.c_str(), 1);
          
          mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, true);

          // Forces resending of all parameters to server
          resetLastSentValues();
      }
      else
      {
          return;
      }
    }
  
    // Process incomming messages and maintain connection to the server
    if(!mqttClient.loop())
    {
        return;
    }

    if (lastUpdateTime + POLL_TIME_MS < millis())
    {
      lastUpdateTime = millis();

      uint16_t registers[11];

      for(uint8_t channel = 0; channel < WavinController::NUMBER_OF_CHANNELS; channel++)
      {
        if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
        {
          uint16_t primaryElement = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK;
          bool allThermostatsLost = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK;

          if(primaryElement==0)
          {
              // Channel not used
              continue;
          }

          if(!configurationPublished[channel])
          {
            uint16_t standbyTemperature = STANDBY_TEMPERATURE_DEG * 10;
            wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_STANDBY_TEMPERATURE, standbyTemperature);
            publishConfiguration(channel);
          }

          // Read the current setpoint programmed for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
          {
            uint16_t setpoint = registers[0];

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_GET);
            String payload = temperatureAsFloatString(setpoint);

            publishIfNewValue(topic, payload, setpoint, &(lastSentValues[channel].setpoint));
          }

          // Read the current mode for the channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_CONFIGURATION, 1, registers))
          {
            uint16_t mode = registers[0] & WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK; 

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_GET);
            if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY)
            {
              publishIfNewValue(topic, MQTT_VALUE_MODE_STANDBY, mode, &(lastSentValues[channel].mode));
            }
            else if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL)
            {
              publishIfNewValue(topic, MQTT_VALUE_MODE_MANUAL, mode, &(lastSentValues[channel].mode));
            }            
          }

          // Read the current status of the output for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
          {
            uint16_t status = registers[0] & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK;

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_OUTPUT);
            String payload;
            if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
              payload = "on";
            else
              payload = "off";

            publishIfNewValue(topic, payload, status, &(lastSentValues[channel].status));
          }

          // If a thermostat for the channel is connected to the controller
          if(!allThermostatsLost)
          {
            // Read values from the primary thermostat connected to this channel 
            // Primary element from controller is returned as index+1, so 1 i subtracted here to read the correct element
            if (wavinController.readRegisters(WavinController::CATEGORY_ELEMENTS, primaryElement-1, 0, 11, registers))
            {
              uint16_t temperature = registers[WavinController::ELEMENTS_AIR_TEMPERATURE];
              uint16_t battery = registers[WavinController::ELEMENTS_BATTERY_STATUS]; // In 10% steps

              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_CURRENT);
              String payload = temperatureAsFloatString(temperature);

              publishIfNewValue(topic, payload, temperature, &(lastSentValues[channel].temperature));

              topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_BATTERY);
              payload = String(battery*10);

              publishIfNewValue(topic, payload, battery, &(lastSentValues[channel].battery));
            }
          }         
        }

        // Process incomming messages and maintain connection to the server
        if(!mqttClient.loop())
       
        {
        return;
        }
      }
    }
  }
}
