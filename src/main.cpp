#include "Wire.h"
#include <WiFi.h>
#include "I2Cdev.h"       // Librerias I2C para controlar el mpu6050
#include "MPU6050.h"      // Librerias I2C para controlar el mpu6050 - Libreria modificada en el archivo MPU6050.h - https://www.youtube.com/watch?v=FMZO4UTVZBk (observar comentarios)
#include <SSD1306Wire.h>  // legacy: #include "SSD1306.h"
#include <NTPClient.h>    // Actualizar la hora por medio del servidor NTPClient - Esta librería ha sido modificada https://github.com/arduino-libraries/NTPClient/issues/36
#include <WiFiUdp.h>      // Udp.cpp: Biblioteca para enviar / recibir paquetes UDP


////////////////////////// Inicializar NTPCliente //////////////////////////


// Network credentials
const char* ssid     = "RED_CANGO";
const char* password = "*/Anita2020_CNN";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String formattedTime;
String dayStamp;


////////////////////////// Inicializar MPU6050 //////////////////////////


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

// Instancia objeto de la clase MPU6050
MPU6050 sensor;

// Valores RAW (sin procesar) del giroscopio en los ejes x,y,z
const int timeData = 3; // Tiempo en recolectar datos en segundos
int16_t accelX = 0, accelY = 0, accelZ = 0;
//int16_t gyroX = 0, gyroY =0, gyroZ = 0;


////////////////////////// Inicializar SSD1306 //////////////////////////


// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, 21, 22);  // ADDRESS, SDA, SCL  -  If not, they can be specified manually.


////////////////////////// CARACTERISTICAS CNN //////////////////////////


// Caracteristicas
double mavXAxis = 0.0,mavYAxis = 0.0,mavZAxis = 0.0;
double rmsXAxis = 0.0,rmsYAxis = 0.0,rmsZAxis = 0.0;
double wlXAxis = 0.0, wlYAxis = 0.0, wlZAxis = 0.0;
double wlaXAxis = 0.0, wlaYAxis = 0.0, wlaZAxis = 0.0;

const int pinStart = 0;
//int buttonState = 0;
//const int SDAPin = 21;
//const int SCLPin = 22;
boolean flag = false;
unsigned long debounce = 0; // Tiempo del rebote.


void ttime() {

  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }

  formattedDate = timeClient.getFormattedDate();
  formattedTime = timeClient.getFormattedTime();

  display.clear();
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(64, 0, "FECHA: ");
  display.drawString(64, 10, String(formattedDate));
  
  display.drawString(64, 30, "HORA: ");
  display.drawString(64, 40, String(formattedTime));
  
  display.display();

}


void featuresExtraction() {
  
  int sizeSample = timeData*10;
  
  mavXAxis = 0.0;
  mavYAxis = 0.0;
  mavZAxis = 0.0;

  rmsXAxis = 0.0;
  rmsYAxis = 0.0;
  rmsZAxis = 0.0;

  wlXAxis = 0.0;
  wlYAxis = 0.0;
  wlZAxis = 0.0;

  for (int k = 0; k<sizeSample;k++)
  { 
    sensor.getAcceleration(&accelX, &accelY, &accelZ);
   
    double ax_m_s2 = accelX * (9.81/16384.0);
    double ay_m_s2 = accelY * (9.81/16384.0);
    double az_m_s2 = accelZ * (9.81/16384.0);

    //Valor absoluto medio (MAV)
    mavXAxis = mavXAxis + abs(ax_m_s2);
    mavYAxis = mavYAxis + abs(ay_m_s2);
    mavZAxis = mavZAxis + abs(az_m_s2);

    ///Valor eficaz (RMS)
    rmsXAxis = rmsXAxis + ax_m_s2*ax_m_s2;
    rmsYAxis = rmsYAxis + ay_m_s2*ay_m_s2;
    rmsZAxis = rmsZAxis + az_m_s2*az_m_s2;

    //Longitud de forma de onda (WL)
    wlXAxis = wlXAxis + abs(ax_m_s2 - wlaXAxis);
    wlYAxis = wlYAxis + abs(ay_m_s2 - wlaYAxis);
    wlZAxis = wlZAxis + abs(az_m_s2 - wlaZAxis);

    wlaXAxis = ax_m_s2;
    wlaYAxis = ay_m_s2;
    wlaZAxis = az_m_s2;
    
    delay(100);
  }
  
  mavXAxis = mavXAxis/(double)sizeSample;
  mavYAxis = mavYAxis/(double)sizeSample;
  mavZAxis = mavZAxis/(double)sizeSample;
  
  rmsXAxis = sqrt(rmsXAxis/(double)sizeSample);
  rmsYAxis = sqrt(rmsYAxis/(double)sizeSample);
  rmsZAxis = sqrt(rmsZAxis/(double)sizeSample);
}


void pulse() {
  if(!digitalRead(pinStart) && (millis()-debounce > 500))
  {
    debounce = millis();
    flag = true;
   } 
} 


void setup() {

    Serial.begin(115200);    //Iniciando puerto serial

    display.init();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    
    //Serial.print("Connecting to ");
    //Serial.println(ssid);
    WiFi.begin(ssid, password);
    delay(1000);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //Serial.print(".");
    }

    // Print local IP address and start web server
    //Serial.println("");
    //Serial.println("WiFi connected.");

    // Initialize a NTPClient to get time
    timeClient.begin();
    timeClient.setTimeOffset(-18000);   //GMT-5

    pinMode(pinStart,INPUT_PULLUP);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();           //Iniciando I2C  
    sensor.initialize();    //Iniciando el sensor

    //Serial.println("Iniciando MPU6050");
    if (sensor.testConnection()) delay(500); //Serial.println("Sensor iniciado correctamente"); 
    else delay(500); //Serial.println("Error al iniciar el sensor");

}

void loop() {

    display.clear();
    ttime();    //Inicio del reloj
    display.clear();
    
    //Condición si se presionó el botón, este retorna un true, en la bandera
    if(!digitalRead(pinStart)) {
        pulse();
    }
    
    //Si la bandera es true, sucede el ingreso de datos
    if(flag) {
        //Mensaje de inicio de ingreso de datos
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(64, 22, "¡Empiece Ahora!");
        display.display();
        delay(100);
        
        //Extracción de datos
        featuresExtraction();

        //Imprime los datos recogidos del sensor MPU6050
        Serial.println(mavXAxis);
        Serial.println(mavYAxis);
        Serial.println(mavZAxis);
        
        Serial.println(rmsXAxis);
        Serial.println(rmsYAxis);
        Serial.println(rmsZAxis);

        Serial.println(wlXAxis);
        Serial.println(wlYAxis);
        Serial.println(wlZAxis);
        
        //La bandera regresa a false
        flag = false;

        //Mensaje de fin de ingreso de datos
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(64, 22, "¡Muy bien!");
        display.display();            
        delay(2000);
        display.clear();    //Finaliza el proceso y reinicia el reloj
    }
}


