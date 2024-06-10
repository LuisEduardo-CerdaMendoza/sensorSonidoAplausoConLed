#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
  #define WIRE Wire
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
  #define WIRE Wire
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
  #define WIRE Wire
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
  #define WIRE Wire
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
  #define WIRE Wire
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
  #define BUTTON_A  9
  #define BUTTON_B  8
  #define BUTTON_C  7
  #define WIRE Wire1
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
  #define WIRE Wire
#endif

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);

const int sensorPin = 2; // Pin digital donde está conectado el sensor de sonido
const int ledPin = 7;    // Pin digital donde está conectado el LED

int clapDetected = 0;     // Variable para rastrear si se ha detectado un aplauso
int ledState = LOW;       // Estado inicial del LED (apagado)

void setup() {
  Serial.begin(9600); // Iniciar la comunicación serial
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Inicializar el display OLED
  display.display(); // Mostrar pantalla en blanco inicia l
  delay(1); // Esperar 2 segundos
  display.clearDisplay(); // Borrar la pantalla
  pinMode(sensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int sensorValue = digitalRead(sensorPin);

  if (sensorValue == HIGH && !clapDetected) {
    clapDetected = 1;       // Marcar que se ha detectado un aplauso
    ledState = !ledState;   // Cambiar el estado del LED
    digitalWrite(ledPin, ledState); // Encender o apagar el LED

    display.clearDisplay(); // Borrar la pantalla antes de mostrar un nuevo mensaje
    display.setTextSize(1); // Tamaño del texto
    display.setTextColor(WHITE); // Color del texto (blanco)
    display.setCursor(0, 0); // Posición del cursor
    display.print("LED: ");
    display.println(ledState == HIGH ? "Encendido" : "Apagado"); // Mostrar estado del LED
    display.display(); // Mostrar en pantalla

    delay(500); // Espera un momento para evitar múltiples detecciones por un solo aplauso
  } else if (sensorValue == LOW && clapDetected) {
    clapDetected = 0; // Restablecer la detección de aplauso
  }
}
