#include <Arduino.h>

RTC_DATA_ATTR const char *ssid = "SSID";
RTC_DATA_ATTR const char *password = "PASSWORD";

const char *wsURL = "URL WEBSERVICE";

// Host du webservice destiné à la requête HTTP
const char *hostRequete = "HOST WEBSERVICE POUR REQUETE HTTP";

const int ID_CAPTEUR = 1;

const float SEALEVELPRESSURE_HPA = 1013.25;
const unsigned int NB_MESURES = 100;
const unsigned int SLEEPING_DURATION = 900000000; // 15 minutes en MICROsecondes
const unsigned int WIFI_TIMEOUT = 5000; // 5 secondes en millisecondes
const unsigned int ALTITUDE = 544; // L'altitude en mètres où se trouve le capteur, pour mesure de la pression relative

