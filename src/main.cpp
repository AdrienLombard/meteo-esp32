#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>
#include "constantes.h"

bool wifiOK;

bool webserviceOK;

Adafruit_BME280 bme; // Le capteur

float getTemperature()
{

  float temperature = 0;

  // Récupération de la température
  // On prend un certain nombre de mesures, et on en fait la moyenne
  for (int i = 0; i < NB_MESURES; i++)
  {
    temperature += bme.readTemperature();
  }

  temperature /= NB_MESURES;

  return temperature;
}

float getPression()
{

  float pression = 0;

  // Récupération de la pression
  // On prend un certain nombre de mesures, et on en fait la moyenne
  for (int i = 0; i < NB_MESURES; i++)
  {
    pression += bme.readPressure();
  }

  pression /= (float)NB_MESURES;

  // La mesure étant faite en pascal, on divise par 100 pour obtenir des hectopascal.
  pression /= 100.0F;

  // On calcule la pression relative au niveau de la mer (La pression diminue d'environ de 1 hPa tous les 8,3 m d'altitude)
  pression += ALTITUDE / 8.3F;

  return pression;
}

float getHumidite()
{

  float humidite = 0;

  // Récupération de l'humidité
  // On prend un certain nombre de mesures, et on en fait la moyenne
  for (int i = 0; i < NB_MESURES; i++)
  {
    humidite += bme.readHumidity();
  }
  humidite /= NB_MESURES;

  return humidite;
}

float getAltitude()
{

  float altitude = 0;

  // Récupération de l'altitude
  // On prend un certain nombre de mesures, et on en fait la moyenne
  for (int i = 0; i < NB_MESURES; i++)
  {
    altitude += bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
  altitude /= NB_MESURES;

  return altitude;
}

float *getMesures()
{

  // Les données sont stockées dans un tableau de 4 float
  float *mesures = new float[4];

  float temperature = getTemperature();
  float pression = getPression();
  float humidite = getHumidite();
  float altitude = getAltitude();

  Serial.println("Température : " + String(temperature) + "°C");
  Serial.println("Pression : " + String(pression) + " hPa");
  Serial.println("Humidite : " + String(humidite) + " %");
  Serial.println("Altitude : " + String(altitude) + " m");
  Serial.println("\n");

  mesures[0] = temperature;
  mesures[1] = pression;
  mesures[2] = humidite;
  mesures[3] = altitude;

  return mesures;
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void initBME280()
{
  bool status;

  // Début de la lecture sur l'adresse du capteur
  status = bme.begin(0x76);

  if (!status) {
    Serial.println("Impossible de trouver le capteur BME280, vérifier le câblage !");
  } else {
    Serial.println("BME280 connecté");
  }

  Serial.println("\n");
}

void goToSleep()
{

  // On paramètre la durée de veille
  // Attention ! En MICROsecondes !
  esp_sleep_enable_timer_wakeup(SLEEPING_DURATION);

  // On déconnecte et désactive Wifi & bluetooth
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  adc_power_off();
  esp_wifi_stop();
  esp_bt_controller_disable();

  Serial.println("Dodo ! Réveil dans " + String(SLEEPING_DURATION / 1000000) + " secondes.");
  Serial.flush();
  esp_deep_sleep_start();
}

void init()
{
  Serial.begin(115200);
  delay(1000);

  print_wakeup_reason();
  Serial.println("Debout !");

  // Au lieu d'être alimenté par la pin 3V3, le capteur utilise le GPIO 18.
  // Il est donc nécessaire de le passer en HIGH pour qu'il produise 3,3V.
  pinMode(GPIO_NUM_18, OUTPUT);
  digitalWrite(GPIO_NUM_18, HIGH);
}

void initWifi()
{
  // Démarrage de la connexion
  WiFi.begin(ssid, password);

  Serial.print("Tentative de connexion...");

  // On laisse quelques secondes à la connexion pour se faire
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT)
  {
    Serial.print(".");
    delay(100);
  }

  Serial.println("\n");

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Impossible de se connecter au wifi.");
    wifiOK = false;
  }
  else
  {
    Serial.println("Connexion etablie!");
    Serial.print("Adresse IP: ");
    Serial.println(WiFi.localIP());
    wifiOK = true;
  }
}

void envoiDonnees(float *mesures)
{
  HTTPClient http;

  // On ouvre la connexion vers l'URL du webservice
  http.begin(wsURL);

  // On formate les mesures en JSON
  String json = "{\"donnees\":[{\"temperature\":" + String(mesures[0]) + ",\"pression\":" + String(mesures[1]) + ",\"humidite\":" + String(mesures[2]) + ",\"altitude\":" + String(mesures[3]) + ",\"idCapteur\":" + ID_CAPTEUR + "}]}";

  // On rajoute quelques headers à la requête
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Content-Length", String(json.length()));
  http.addHeader("Host", hostRequete);
  http.addHeader("Connection", "close");
  int httpResponseCode = http.POST(json);

  Serial.println("Code réponse : " + String(httpResponseCode));

  // On considère que tout s'est bien passé si on reçoit un code réponse 201 CREATED
  webserviceOK = (httpResponseCode == 201);
}

void setup()
{

  init();

  initWifi();

  if (wifiOK) {

    initBME280();

    float *mesures = getMesures();

    envoiDonnees(mesures);

  }

  goToSleep();
}

void loop()
{
}