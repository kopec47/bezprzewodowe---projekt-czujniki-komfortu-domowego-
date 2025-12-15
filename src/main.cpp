#define CUSTOM_BLE_SPI
#define BLE_SPI_MISO D12
#define BLE_SPI_MOSI D11
#define BLE_SPI_CLK  D3
#define BLE_SPI_CS   A1
#define BLE_SPI_IRQ  A0
#define BLE_SPI_FREQ 8000000
#define BLE_SPI_MODE SPI_MODE0
#define BLE_CHIP_TYPE BLUENRG_M0
#define BLE_RESET    D7

#include <STM32duinoBLE.h>
#include <Arduino.h>
#include <Wire.h>
#include <TMP117.h>
#include <Adafruit_HDC302x.h>
#include <ILPS22QSSensor.h>
#include <LIS2DUXS12Sensor.h>

// TMP117
uint8_t TMP_ADDR = 0x48;
TMP117 tmp(TMP_ADDR);

// HDC302x
uint8_t HDC_ADDR = 0x44;
Adafruit_HDC302x hdc = Adafruit_HDC302x();

// ILPS22QS
ILPS22QSSensor pressureSensor(&Wire);

// LIS2DUXS12 Akcelerometr
#define SENSOR_ODR 25.0f
#define ACC_FS 2
#define FIFO_SAMPLE_THRESHOLD 10
LIS2DUXS12Sensor accel(&Wire);
bool useAccelerometer = false;
bool motionDetected   = false;
int32_t lastAccValue[3] = {0, 0, 0};
const int32_t MOTION_THRESHOLD = 500; // mg

// BLE – UUID
BLEService sensorsService("a61c8642-2e46-4d1d-2137-f77d8adb5e41");
BLEFloatCharacteristic charTemp   ("a61c8642-2e46-4d1d-2137-f77d8adb5e51", BLERead | BLENotify);
BLEFloatCharacteristic charHumid  ("a61c8642-2e46-4d1d-2137-f77d8adb5e52", BLERead | BLENotify);
BLEFloatCharacteristic charPressure("a61c8642-2e46-4d1d-2137-f77d8adb5e53", BLERead | BLENotify);

// Flagi i timery
bool useThermometer   = false;
bool useHumidity      = false;
bool usePressure      = false;
bool envSensorsInited = false;   // <-- NOWA flaga: czy środowiskowe są już zainicjalizowane
bool bleConnected     = false;
unsigned long lastCheck     = 0; // ostatni pomiar / ruch
unsigned long accelLastCheck = 0;

void newTempEmpty() {}

// --- INICJALIZACJA ŚRODOWISKOWYCH (WOŁAMY DOPIERO PO RUCHU) ---
void initEnvSensorsOnce() {
  if (envSensorsInited) return;

  Serial.println("=== INICJALIZACJA CZUJNIKÓW ŚRODOWISKOWYCH (po ruchu) ===");

  // TMP117
  tmp.init(newTempEmpty);
  if (tmp.readConfig() != 65535) {
    useThermometer = true;
    Serial.println("  Termometr OK");
  } else {
    Serial.println("  Termometr ERROR");
  }

  // HDC302x
  if (hdc.begin(HDC_ADDR, &Wire)) {
    useHumidity = true;
    Serial.println(" Wilgotność OK");
  } else {
    Serial.println(" Wilgotność ERROR");
  }

  // ILPS22QS
  if (pressureSensor.begin() == ILPS22QS_OK &&
      pressureSensor.Enable() == ILPS22QS_OK) {
    usePressure = true;
    Serial.println("  Ciśnienie OK");
  } else {
    Serial.println("  Ciśnienie ERROR");
  }

  envSensorsInited = true;
  Serial.println("=== KONIEC INICJALIZACJI ŚRODOWISKOWYCH ===");
}

// --- AKCELEROMETR ---
bool accelerometerOk() {
  if (accel.begin() != 0) return false;

  uint8_t status = 0;
  status |= accel.Enable_X();
  status |= accel.Set_X_ODR(SENSOR_ODR);
  status |= accel.Set_X_FS(ACC_FS);
  status |= accel.Set_FIFO_X_BDR(LIS2DUXS12_BDR_XL_ODR);
  status |= accel.Set_FIFO_Watermark_Level(FIFO_SAMPLE_THRESHOLD);
  status |= accel.Set_FIFO_Stop_On_Fth(1);
  status |= accel.Set_FIFO_Mode(LIS2DUXS12_STREAM_MODE);

  return (status == 0);
}

bool checkMotion() {
  uint8_t fullStatus = 0;
  if (accel.Get_FIFO_Watermark_Status(&fullStatus) != 0) {
    return false;
  }

  if (fullStatus) {
    uint16_t samples_to_read;
    if (accel.Get_FIFO_Num_Samples(&samples_to_read) != 0) {
      return false;
    }

    // Bierzemy ostatnią próbkę
    uint8_t tag;
    if (accel.Get_FIFO_Tag(&tag) == 0 && tag == 2) {
      int32_t acc_value[3];
      if (accel.Get_FIFO_X_Axes(acc_value) == 0) {
        int32_t dx = abs(acc_value[0] - lastAccValue[0]);
        int32_t dy = abs(acc_value[1] - lastAccValue[1]);
        int32_t dz = abs(acc_value[2] - lastAccValue[2]);

        memcpy(lastAccValue, acc_value, sizeof(acc_value));

        if (dx > MOTION_THRESHOLD || dy > MOTION_THRESHOLD || dz > MOTION_THRESHOLD) {
          Serial.print(" RUCH WYKRYTY! X:");
          Serial.print(acc_value[0]);
          Serial.print(" Y:");
          Serial.print(acc_value[1]);
          Serial.print(" Z:");
          Serial.println(acc_value[2]);
          return true;
        }
      }
    }
  }
  return false;
}

// --- ODCZYTY ---
double getTemperature() {
  return tmp.getTemperature();
}

double getHumidityVal() {
  double t = 0.0, RH = 0.0;
  hdc.readTemperatureHumidityOnDemand(t, RH, TRIGGERMODE_LP0);
  return RH;
}

float getPressureVal() {
  float pres = 0.0;
  if (pressureSensor.GetPressure(&pres) == ILPS22QS_OK)
    return pres;
  else
    return -1;
}

void readAndSendAllSensors() {
  if (!envSensorsInited) return; // bezpieczeństwo: nie rób nic jeśli nie zainicjowane

  lastCheck = millis();
  Serial.println("=== ODCZYT CZUJNIKÓW (po ruchu) ===");

  if (useThermometer) {
    double temp = getTemperature();
    Serial.print("Temperatura: ");
    Serial.print(temp, 2);
    Serial.println(" °C");
    charTemp.writeValue((float)temp);
  }

  if (useHumidity) {
    double humid = getHumidityVal();
    Serial.print("Wilgotność: ");
    Serial.print(humid, 2);
    Serial.println(" %");
    charHumid.writeValue((float)humid);
  }

  if (usePressure) {
    float pres = getPressureVal();
    if (pres >= 0) {
      Serial.print("Ciśnienie: ");
      Serial.print(pres, 2);
      Serial.println(" hPa");
      charPressure.writeValue(pres);
    }
  }

  Serial.print("Połączony: ");
  Serial.println(bleConnected ? "TAK" : "NIE");
  Serial.println("========================");
}

// --- BLE ZDARZENIA ---
void onBLEConnected(BLEDevice central) {
  bleConnected = true;
  Serial.println("POŁĄCZONY – WYBUDZONY, ALE CZEKA NA RUCH");
}

void onBLEDisconnected(BLEDevice central) {
  bleConnected = false;
  Serial.println("ODŁĄCZONY – TRYB RUCH + SLEEP");
}

// --- SLEEP ---
void enterBatterySleep() {
  Serial.println("SLEEP (tylko akcelerometr aktywny)...");

  BLE.stopAdvertise();

  // Deep sleep – wybudzi reset / zewnętrzny sygnał
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();

  Serial.println("WYBUDZONY!");

  if (!BLE.begin()) {
    NVIC_SystemReset();
  }

  sensorsService.addCharacteristic(charTemp);
  sensorsService.addCharacteristic(charHumid);
  sensorsService.addCharacteristic(charPressure);

  BLE.addService(sensorsService);
  BLE.setAdvertisedService(sensorsService);
  BLE.setLocalName("ComfortApp");
  BLE.advertise();

  Serial.println("BLE OK ");

  // Po wybudzeniu środowiskowe znowu nie są zainicjalizowane
  envSensorsInited = false;
  useThermometer   = false;
  useHumidity      = false;
  usePressure      = false;
}

// --- SETUP ---
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  analogReadResolution(10);
  analogRead(A0);

  Wire.begin();
  delay(500);

  if (!BLE.begin()) {
    Serial.println(" BLE failed!");
    while (1);
  }

  BLE.setEventHandler(BLEConnected, onBLEConnected);
  BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

  sensorsService.addCharacteristic(charTemp);
  sensorsService.addCharacteristic(charHumid);
  sensorsService.addCharacteristic(charPressure);

  BLE.addService(sensorsService);
  BLE.setAdvertisedService(sensorsService);
  BLE.setLocalName("ComfortApp");
  BLE.advertise();

  Serial.println("=== INICJALIZACJA – tylko akcelerometr ===");

  if (accelerometerOk()) {
    useAccelerometer = true;
    Serial.println("Akcelerometr OK");
  } else {
    Serial.println("Akcelerometr ERROR");
  }

  
  Serial.println("Czujniki środowiskowe zostaną włączone dopiero po pierwszym ruchu.");
  Serial.println("==================");
}

// Dodaj flagę i timer
unsigned long lastMotionTime = 0;
const unsigned long MOTION_COOLDOWN = 5000; // 5 sekund

// W loop() - zmień na:
void loop() {
  BLE.poll();

  // Sprawdzanie akcelerometru co 100ms
  if (useAccelerometer && millis() - accelLastCheck > 100) {
    accelLastCheck = millis();

    // Sprawdź czy minęło 5s od ostatniego ruchu
    if (millis() - lastMotionTime > MOTION_COOLDOWN) {
      if (checkMotion()) {
        lastMotionTime = millis();  // ← NOWE: timestamp ruchu
        motionDetected = true;
        lastCheck = millis();

        if (!envSensorsInited) {
          initEnvSensorsOnce();
        }
        readAndSendAllSensors();
      }
    }
  }

  // Timer 1h bezczynności
  if (!bleConnected && (millis() - lastCheck > 3600000UL)) {
    enterBatterySleep();
  }
}
