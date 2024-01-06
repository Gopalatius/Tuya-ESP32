#include <WiFi.h>
#include <AccelStepper.h>
#include <WiFiManager.h>
#include "TuyaESP32.h"
#include "nvs_flash.h"

// Constants
#define STEPS 400
#define SWITCH_DP "48"

// Pin assignments
#define ENABLE_PIN 25
#define BUTTON_PIN 32
#define STEP_PIN 26
#define DIR_PIN 27
#define VIO_PIN 15

// Tuya device client and WiFi manager
tuya_iot_client_t client;
WiFiManager wifiManager;

// Stepper motor setup
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Function declarations
void setupStepper();
void connectToWiFi();
void handleTuyaEvent(tuya_iot_client_t *client, tuya_event_msg_t *event);
void hardwareSwitchSet(bool value);
void tuyaDataPointDownload(const char *json_dps);
void tuyaLinkAppTask(void *pvParameters);
void checkIfResetTask(void *pvParameters);
void resetWiFi();
void exampleQrcodePrint(const char *productkey, const char *uuid);
void setOneShotBlinkPeriod(int period);
void setBlinkPeriod(int period);
void toggleLED(bool on);
void triggerOneShotBlink();
void ledControlTask(void *pvParameter);
//global variables
volatile bool resetFlag = false;

// Global variables
volatile int blinkPeriod = 1000; // Default blink period in milliseconds
volatile bool ledState = false; // LED state
volatile bool oneShotBlink = false; // One-shot blink flag
volatile int oneShotBlinkPeriod = 100; // Default one-shot blink period in milliseconds


void setup()
{
	Serial.begin(115200);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), resetWiFi, FALLING);
	pinMode(LED_BUILTIN, OUTPUT);
	setupStepper();
	connectToWiFi();

	xTaskCreate(tuyaLinkAppTask, "tuya_link", 1024 * 6, NULL, 4, NULL);
	xTaskCreate(checkIfResetTask, "reset_wifi", 1024, NULL, 4, NULL);
  xTaskCreate(ledControlTask, "LED Control Task", 1024, NULL, 1, NULL);
}

void loop()
{
	// Main loop does nothing in this application
}

void setupStepper()
{
	pinMode(VIO_PIN, OUTPUT);
	digitalWrite(VIO_PIN, HIGH);

	pinMode(ENABLE_PIN, OUTPUT);
	stepper.setEnablePin(ENABLE_PIN);

	stepper.setMaxSpeed(1000);
	stepper.setAcceleration(500);
	stepper.setPinsInverted(false, false, true);
	stepper.disableOutputs();
}

void connectToWiFi()
{
	WiFi.mode(WIFI_STA);

  setBlinkPeriod(1000);
  toggleLED(true);
	if (!wifiManager.autoConnect("SMOOR-credentials", "chewy-jackal-recipient"))
	{
		Serial.println("Failed to connect");
		ESP.restart();
	}
  toggleLED(false);

	Serial.print(F("Connected. My IP address is: "));
	Serial.println(WiFi.localIP());
}

void hardwareSwitchSet(bool value)
{
	Serial.println(value ? "Switch ON" : "Switch OFF");
	digitalWrite(LED_BUILTIN, value ? HIGH : LOW);
	spinTheLock(value ? -STEPS : 0);
}

void tuyaDataPointDownload(const char *json_dps)
{
	Serial.printf("Data point download value:%s\n", json_dps);
	cJSON *dps = cJSON_Parse(json_dps);
	if (!dps)
	{
		Serial.println("JSON parsing error, exit!");
		return;
	}

	cJSON *data = dps->child;
	if (strcmp(data->string, SWITCH_DP) == 0)
	{
		hardwareSwitchSet(cJSON_IsTrue(data));
	}

	cJSON_Delete(dps);
	tuya_iot_dp_report_json(&client, json_dps);
}

void exampleQrcodePrint(const char *productkey, const char *uuid)
{
	char urlbuf[255];
	sprintf(urlbuf, "https://smartapp.tuya.com/s/p?p=%s&uuid=%s&v=2.0", productkey, uuid);
	Serial.println(urlbuf);
	Serial.println("(Use this URL to generate a static QR code for the Tuya APP scan code binding)");
}

void handleTuyaEvent(tuya_iot_client_t *client, tuya_event_msg_t *event)
{
	switch (event->id)
	{
		case TUYA_EVENT_BIND_START:
      setBlinkPeriod(500);
      toggleLED(true);
			exampleQrcodePrint(client->config.productkey, client->config.uuid);
      
			break;
		case TUYA_EVENT_MQTT_CONNECTED:
      toggleLED(false);
      setOneShotBlinkPeriod(3000);
      triggerOneShotBlink();
			Serial.println("Device MQTT Connected!");
      // tuya_iot_dp_report_json(&client, json_dps);
			break;
		case TUYA_EVENT_DP_RECEIVE:
			tuyaDataPointDownload((const char *) event->value.asString);
			break;
		default:
			break;
	}
}

void tuyaLinkAppTask(void *pvParameters)
{
	tuya_iot_config_t config;;
	config.software_ver = "1.0.0";
	config.productkey = TUYA_PRODUCT_KEY;
	config.uuid = TUYA_DEVICE_UUID;
	config.authkey = TUYA_DEVICE_AUTHKEY;
	config.event_handler = handleTuyaEvent;
	int ret = tuya_iot_init(&client, &config);
	assert(ret == OPRT_OK);

	tuya_iot_start(&client);

	for (;;)
	{
		tuya_iot_yield(&client);
	}
}

void spinTheLock(int steps)
{
	stepper.enableOutputs();
	stepper.moveTo(steps);
	stepper.runToPosition();
	stepper.disableOutputs();
}

void resetWiFi()
{
	resetFlag = true;
}

void checkIfResetTask(void *pvParameters)
{
	while (true)
	{
		if (resetFlag)
		{
			Serial.println("Resetting WiFi settings");
			wifiManager.resetSettings();
      nvs_flash_erase();
			ESP.restart();
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);	// Optional: To prevent this task from running too frequently
	}
}
void ledControlTask(void *pvParameter) {
    while (true) {
        if (ledState) {
            digitalWrite(LED_BUILTIN, HIGH);
            vTaskDelay(blinkPeriod / portTICK_PERIOD_MS);
            digitalWrite(LED_BUILTIN, LOW);
            vTaskDelay(blinkPeriod / portTICK_PERIOD_MS);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
            vTaskDelay(100 / portTICK_PERIOD_MS); // Short delay for responsive state changes
        }

        if (oneShotBlink) {
            digitalWrite(LED_BUILTIN, HIGH);
            vTaskDelay(oneShotBlinkPeriod / portTICK_PERIOD_MS); // Use one-shot blink period
            digitalWrite(LED_BUILTIN, LOW);
            oneShotBlink = false; // Reset one-shot flag
        }
    }
}
void setOneShotBlinkPeriod(int period) {
    oneShotBlinkPeriod = period;
}
void setBlinkPeriod(int period) {
    blinkPeriod = period;
}

void toggleLED(bool on) {
    ledState = on;
}

void triggerOneShotBlink() {
    oneShotBlink = true;
}


