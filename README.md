 #include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Paramètres de l'écran LCD
#define I2C_ADDR    0x27 // Adresse I2C (à vérifier avec un scanner)
#define LCD_COLUMNS 16
#define LCD_ROWS    2
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

// Capteur DHT22
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Broches
const int sensorPin = 7;;
const int ledPin = 8;
const int relayPin = 13;

// Variables d'état
bool etatLED = false;
bool lastStateSensor = HIGH;

void setup() {
  Serial.begin(9600);
  Serial.println("Temperature, Humidity, LED & Relay Control");

  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.print("Temp: ");
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
}

void loop() {
  // 1. Gestion du capteur et de la LED
  int detect = digitalRead(sensorPin);
  if (detect != lastStateSensor) {
    delay(50);
    detect = digitalRead(sensorPin);
    if (detect == LOW) {
      etatLED = !etatLED;
      digitalWrite(ledPin, etatLED);
      Serial.print("LED: ");
      Serial.println(etatLED ? "ON" : "OFF");
    }
    lastStateSensor = detect;
  }

  // 2. Lecture du DHT22, affichage et contrôle du relais
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.setCursor(0,0);
    lcd.print("DHT Error    ");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %, Temp: ");
  Serial.print(t);
  Serial.println(" °C");

  lcd.setCursor(6, 0);
  lcd.print(t);
  lcd.print(" C   "); //Ajout d'espaces pour nettoyer l'affichage
  lcd.setCursor(6, 1);
  lcd.print(h);
  lcd.print(" %   "); //Ajout d'espaces pour nettoyer l'affichage


  if (t >= 26.0) {
    digitalWrite(relayPin, HIGH);
    Serial.println("Motor ON (Relay)");
  } else if (t <= 23.0) {
    digitalWrite(relayPin, LOW);
    Serial.println("Motor OFF (Relay)");
  }

  delay(1000); // Délai pour la lecture du DHT22 (peut être ajusté)
}
