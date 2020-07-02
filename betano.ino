// Load Wi-Fi library
#include <esp_timer.h>
#include "BluetoothSerial.h"

#define NBALFALOOPS 2
/* Communication with the outside world */
BluetoothSerial ESP_BT;

/* Chrono struct */
struct Karen {
    const uint8_t PIN; // Pin of the interrupt
    uint32_t nRebounds; // number of time the interrupt was detected
    uint32_t nTriggered; // number of sectors elapsed
    bool triggered; // Set as true everytime the interrupt is detected, cleared when read as a sector
    int64_t time;
};
Karen alfano = {12, 0, 0, false, 0};
int64_t sectors[2048] = {0};

/* Led indicator */
String outputState = "off";
const int output = 13;

/* Timer */
static void gaeta_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "One-shot timer called, time since boot: %lld us", time_since_boot);
}


void IRAM_ATTR isr() {
  alfano.nRebounds++;
  if (alfano.triggered == false)
  {
    alfano.triggered = true;
    alfano.nTriggered++;
    alfano.time = esp_timer_get_time();
  }
}

void setup() {
  // Start serual and bluetooth
  Serial.begin(115200);
  ESP_BT.begin("SV-650_BTelemetry-dev");

  // Power the LED on
  pinMode(output, OUTPUT);
  digitalWrite(output, HIGH);

  // Attach the interrupt for the magnetic sensor
  pinMode(alfano.PIN, INPUT_PULLUP);
  attachInterrupt(alfano.PIN, isr, HIGH);

  // Start the timer
  esp_timer_create_args_t timer_args;
  timer_args.callback = &gaeta_timer_callback;
  timer_args.name = "gaeta";
  esp_timer_handle_t gaeta;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &gaeta));
  ESP_ERROR_CHECK(esp_timer_start_once(gaeta, 5000000));
  sectors[0] = esp_timer_get_time();
  for (int i = 1; i < NBALFALOOPS; i++) {
    sectors[i] = sectors[0];
    alfano.nTriggered++;
  }
}

void loop(){
  int64_t current = esp_timer_get_time();
  if (alfano.triggered && (current - alfano.time) > 1000000) {
    sectors[alfano.nTriggered] = alfano.time;
    float hotlap =  (float)(sectors[alfano.nTriggered] - sectors[alfano.nTriggered - NBALFALOOPS]) / 1000000;
    int64_t lap_minutes = hotlap / 60;
    float lap_seconds = fmod(hotlap, 60);

    Serial.printf("Alfano has been pressed for the %u time. %u rebounds this time. "
                  "Time of trigger %u; Sector %f seconds\nTime: %u:%09.6f [%u]\n",
                  alfano.nTriggered, alfano.nRebounds,
                  alfano.time, hotlap, lap_minutes, lap_seconds, NBALFALOOPS);
    ESP_BT.printf("Alfano has been pressed for the %u time. %u rebounds this time. "
                  "Time of trigger %u; Hotlap %f seconds\nTime: %u:%09.6f [%u]\n",
                  alfano.nTriggered, alfano.nRebounds,
                  alfano.time, hotlap, lap_minutes, lap_seconds, NBALFALOOPS);
    alfano.triggered = false;
    alfano.nRebounds = 0;
  }
}
