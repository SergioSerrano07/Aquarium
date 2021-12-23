/*
 
 Se presenta la programación de un proyecto basado en el control de un acuario 
 mediante una APP móvil.

 La programación del proyecto se presenta incialmente con las librerias usadas a 
 lo largo del proyecto, continuado de las diferentes variables y funciones que 
 presenta y la programación que necesita cada uno de los sensores, teniendo en 
 cuenta el envío de los valores mediante una APP móvil desde cualquier parte del
 mundo.

 Proyecto y programación realizada por Guillermo Sierra y Sergio Serrano.

 Junio 2021
 
 */

// Se incluyen las librerías necesarias para el proyecto
#define BLYNK_PRINT Serial
#include <WiFi.h>              
#include <WiFiClient.h> //Libreria Blynk
#include <BlynkSimpleEsp32.h> //Librerias ESP32
#include <DHT.h> // // Sensor DHT22
#include <RBDdimmer.h> // Controlador dimmer
#include <OneWire.h>
#include <DallasTemperature.h> // Sensor DS18b20
#include "DFRobot_ESP_PH.h" // Sensor de pH
#include <EEPROM.h>

// Permiso de conexión Wifi e interatuar con la APP móvil

char auth[] = "1Qy3kHxIrAM1RewdHIj9QPzqWhZDUtEy"; //Token de la APP de Blynk
char ssid[] = "MOVISTAR_E3B0"; //Nombre de nuestra WIFI a la que queremos acceder
char pass[] = "7MnLAwyCugtoxpub6rxA"; //Contrasela de la WIFI

// Controlador de programación del pH
DFRobot_ESP_PH ph;
#define ESPADC 4096.0   // Valor Analógico-Digital conversión
#define ESPVOLTAGE 3300 // Valor de voltaje ESP32
#define PH_PIN 35  //PIN en placa ESP32
float voltage, phValue, temperature = 25;

#define oneWireBus 15 //PIN sensor de temperatura interna del acuario

#define boya 5 //PIN Sensor nivel de agua que activa la bomba de agua
#define rele_bomba 17 //PIN rele de la bomba de agua

//Parámetros boya-relé (controlador rellenado automático)
int tiempo_actual_1 = 0;
int tiempo_actual_2 = 0;
int tiempo_resultante = 0;
int estado_boya = 0;

#define DHTTYPE DHT22 //Sensor de Humedad y Temperatura ambiente
#define DHTPIN 2 //PIN Sensor de Humedad y Temperatura ambiente

#define outputPin 12 //PIN(PWM) para el dimmer (Iluminación)
#define zerocross 34 //PIN C-Z placa ESP32

//Sensor de temperatura interna del acuario
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

//Sensor de humedad y temperatura ambiente
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

//Parámetros control de iluminación acuario
dimmerLamp dimmer(outputPin, zerocross); //Puerto para ESP32
int outVal = 0; // Inicia valor dimmer: pantalla acuario
int dim_val; // Valor dimmer para Blynk

float temp = 0;

void sendSensor(){
   
  float h = dht.readHumidity(); //Humedad ambiente
  float t = dht.readTemperature(); //Temperatura ambiente
 
  if (isnan(h) || isnan(t))
    {
    Serial.println("Error lectura sensor");
    return;
    }
   
 sensors.requestTemperatures();
 temp = sensors.getTempCByIndex(0);
 Serial.println(String("Sıcaklik=")+temp+ String(" C"));

  Blynk.virtualWrite(V1, t); // Pin virtual para la temperatura externa del acuario
  Blynk.virtualWrite(V2, h); // Pin virtual para la humedad
  Blynk.virtualWrite(V3, temp); //Pin virtual para la temperatura interna del acuario
  Blynk.virtualWrite(V6, phValue); //Pin virtual para el medidor de pH
}

BLYNK_WRITE(V4)  {  //Pin virtual Blynk Iluminacion
  outVal = param.asInt();
  dim_val = map(outVal, 0, 1023, 0, 100);
  dimmer.setPower(dim_val);            
  Blynk.virtualWrite(V5, dim_val); //Pin virtual Blynk ValorIluminacion
}

void setup(){
  // Empieza el monitor serie
  Serial.begin(115200);
  ph.begin();
  pinMode(boya, INPUT);
  pinMode(rele_bomba, OUTPUT);
  dht.begin();
  timer.setInterval(1000L, sendSensor);
  dimmer.begin(NORMAL_MODE, ON);
  Blynk.begin(auth, ssid, pass, "sergioacuario.ddns.net", 8080);
  sensors.begin();
  
}

  void loop(){
{
 static unsigned long timepoint = millis();
 if (millis() - timepoint > 1000U) 
 {
  timepoint = millis();
  voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // Lectura voltaje
  Serial.print("voltage:");
  Serial.println(voltage, 4);
  
  Serial.print("temperature:");
  Serial.print(temperature, 1);
  Serial.println("^C");

  phValue = ph.readPH(voltage, temperature); // Convierte el voltaje a pH a partir de la temperatura
  Serial.print("pH:");
  Serial.println(phValue, 4);
 }
 ph.calibration(voltage, temperature); // Proceso de calibracion pH CMD

  {
   estado_boya = digitalRead(boya);

  if (estado_boya == LOW) {
    tiempo_actual_1 = millis();
    digitalWrite(rele_bomba, HIGH);
  }

  else {
    tiempo_actual_2 = millis();
    tiempo_resultante = tiempo_actual_2 - tiempo_actual_1;

      if (tiempo_resultante >= 1000) {
        digitalWrite(rele_bomba, LOW);
        delay(5000);   // Unico parametro a modificar segun necesidades (cantidad agua acuario)
      }
  }
  }
  Blynk.run();
  timer.run();
}
