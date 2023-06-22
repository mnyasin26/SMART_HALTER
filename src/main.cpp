// config untuk kode mana yang akan diupload
#include "dev_config.h"
#ifdef V2

#include <Arduino.h>

// Nama Program    : IoT Node Halter
// Versi Revisi    : V.2.1a
// Nama Pembuat    : Ali Akbar, Muhamad Nur Yasin Amadudin
// Tanggal Di Buat : 02/06/2023
// Tanggal Pembaharuan Terakhir: 13/06/2023
// Keterangan : simbol (*) kode yang ditambahkan

// Task
TaskHandle_t Task1;

//*
#include <Wire.h>
#include "heartRate.h"

// Inisialisasi library JY901 (Sensor IMU 10)
#include <JY901.h>
// Inisialisasi varibel untuk menampung nilai - nilai pembacaan dari sensor
String AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, roll, pitch, yaw;

// Inisialisasi library axp20x (Power Management)
#include <axp20x.h>
AXP20X_Class axp;
// Inisialisasi variabel untuk menyimpan addres axp192
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;
String SOG, COG, VBAT, STAT, ABAT = "0.0";

// Inisialisasi library GPS Neo 6
#include <TinyGPS++.h>
TinyGPSPlus gps;
// Menggunakan pin serial 1 pada esp32
HardwareSerial GPS2(1);
// Inisialisasi variabel untuk menampung nilai koordinat dari GPS2
String Latitude, Longitude, Altitude;

// Inisialisasi library untuk OLED 128x64
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64, &Wire);
// Inisialisasi pin SDA dan SCL untuk koneksi OLED
const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;

// Inisialisasi library AMG88xx (Thermal Camera/Sensor Suhu)
#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
// Inisialisasi variabel untuk menyimpan hasil pembacaan suhu
float Suhu = 0;
float bSuhu = 0;

// Inisialisasi library Max30102 *
#include "MAX30105.h" //sparkfun MAX3010X library
// #include <DFRobot_MAX30102.h>
// DFRobot_MAX30102 particleSensor;
MAX30105 particleSensor;

// Inisialisasi variabel untuk menampung nilai pembacaan sensor
// int32_t SPO2;
// int32_t heartRate;
// int8_t SPO2Valid;
// int8_t heartRateValid;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Inisialisasi variabel untuk menyimpan data NAN ketika nilai HR dan SPO2 dari sensor tidak terbaca
String HR = "NAN", SPO = "NAN";

// Inisialisasi library SPI
#include <SPI.h>
// Inisialisasi library LoRa
#include <LoRa.h>
// Inisialisasi pin yang digunakan LoRa => ESP32
#define SS 18
#define RST 14
#define DIO0 26
#define SCK 5
#define MISO 19
#define MOSI 27
// Insialisasi variabel untuk kirim dan terima pesan pada komunikasi LoRa
String RSI = "NAN";        // Variabel untuk menampung nilai RSI
String outgoing;           // Pesan keluar
byte msgCount = 0;         // Menghitung jumlah pesan keluar
byte MasterNode = 0xFF;    // Addres Node Gateway
byte Node_Halter = 0xBB;   // Addres Node Kandang
String NoDevice = "02";    // Nomor Device
String ID = "SHMKR100601"; // ID  Device
String pesan;              // Variabel untuk menampung pesan yang akan dikirim ke Gateway

// Inisialisasi variabel untuk menampung nilai millis
unsigned long lastSendTime = 0;
unsigned long lastSendTime2 = 0;
// Interval antar pengiriman data LoRa
int interval = 50;
// Variabel counter
int count;

// Inisialisasi pin Button
#define BUTTONYELLOW 14
#define BUTTONGREEN 25
// Variabel untuk menyimpan kondisi button
int BTNG, BTNY;
// Variabel untuk fungsi flagging
bool state, state2;

// deklrasi fungsi (definisi ada di bawah loop) *
void sendMessage(String outgoing, byte MasterNode, byte otherNode);
void onReceive(int packetSize);
void Task1code(void *parameter);

#define USEFIFO
void setup()
{
  // Memulai komunikasi serial
  Serial.begin(9600);

  // Memulai komunikasi SPI
  SPI.begin(SCK, MISO, MOSI, SS);
  // Setting pin LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6))
  {
    Serial.println("Gagal menjalankan LoRa. Periksan wiring rangkaian.");
  }

  // while (!particleSensor.begin())
  // {
  //   Serial.println("MAX30102 was not found");
  //   delay(1000);
  // }

  // Initialize sensor *
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1)
      ;
  }

  // Setup to sense a nice looking saw tooth on the plotter *
  byte ledBrightness = 170; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;    // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  // Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; // Options: 69, 118, 215, 411
  int adcRange = 16384; // Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings

  // Mulai menjalankan GPS
  GPS2.begin(9600, SERIAL_8N1, 34, 12);

  // Mulai menjalankan komunikasi I2C
  Wire.begin(i2c_sda, i2c_scl);

  // Mulai menjalankan sensor IMU 10
  JY901.startIIC();

  // Mulai menjalankan library AXP (Power Management)
  int ret = axp.begin(Wire, slave_address);
  if (ret)
  {
    Serial.println("Ooops, AXP202/AXP192 power chip detected ... Check your wiring!");
  }
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                     AXP202_VBUS_CUR_ADC1 |
                     AXP202_BATT_CUR_ADC1 |
                     AXP202_BATT_VOL_ADC1,
                 true);

  // // Konfigurasi sensor MAX30102
  // particleSensor.sensorConfiguration(250, SAMPLEAVG_1,
  //                                    MODE_MULTILED, SAMPLERATE_100,
  //                                    PULSEWIDTH_411, ADCRANGE_16384);

  // Mulai menjalankan OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed 1"));
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
    {
      Serial.println(F("SSD1306 allocation failed 2"));
    }
  }

  // Mulai menjalankan sensor Thermal Camera / Suhu
  bool STATER = false;
  if (!amg.begin(0x68))
  {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    STATER = true;
  }
  if (STATER == true)
  {
    if (!amg.begin(0x69))
    {
      Serial.println("Could not find a valid AMG88xx sensor, check wiring2");
    }
  }

  // Menampilkan teks pada OLED
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int i = 0;
  while (i < 3)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("\n   START");
    display.println(String() + "     " + (i + 1));
    Serial.println(i);
    display.display();
    delay(1000);
    i++;
  }

  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      1);        /* pin task to core 0 */

  // Konfigurasi pin BUTTON sebagai input
  pinMode(BUTTONYELLOW, INPUT_PULLUP);
  pinMode(BUTTONGREEN, INPUT_PULLUP);
}

long irValue;
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100; // calculate SpO2 by this sampling interval

double ESpO2 = 95.0;    // initial value of estimated SpO2
double FSpO2 = 0.7;     // filter factor for estimated SpO2
double frate = 0.95;    // low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0      // adjust to display heart beat and SpO2 in the same scale
#define MAX_SPO2 100.0
#define MIN_SPO2 80.0
#define SAMPLING 5      // if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0

void loop()
{
  BTNY = digitalRead(BUTTONYELLOW);
  BTNG = digitalRead(BUTTONGREEN);

  if (BTNY == 0)
  {
    ESP.restart();
  }

  if (millis() - lastSendTime > interval)
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    if (axp.isBatteryConnect())
    {
      ABAT = axp.getBattChargeCurrent();
      VBAT = axp.getBattVoltage();
      state2 = false;
    }
    else
    {
      VBAT = "NAN";
      state2 = true;
    }

    if (gps.location.isValid())
    {
      Latitude = String(gps.location.lat(), 9);
      Longitude = String(gps.location.lng(), 9);
      SOG = String(gps.speed.kmph());
      COG = String(gps.course.deg());
    }
    else
    {
      Latitude = "NAN";
      Longitude = "NAN";
      SOG = "NAN";
      COG = "NAN";
    }

    Altitude = JY901.getAltitude() / 100.0;
    AccX = JY901.getAccX();
    AccY = JY901.getAccY();
    AccZ = JY901.getAccZ();
    GyroX = JY901.getGyroX();
    GyroY = JY901.getGyroY();
    GyroZ = JY901.getGyroZ();
    MagX = JY901.getMagX();
    MagY = JY901.getMagY();
    MagZ = JY901.getMagZ();
    roll = JY901.getRoll();
    pitch = JY901.getPitch();
    yaw = JY901.getYaw();

    // Data dummy vbat, heartrate, spo2
    float dummy_vbat = random(4, 5);
    int dummy_heartrate = random(70, 95);
    int dummy_spo2 = random(90, 100);
    Suhu = Suhu - 1;
    if (Suhu < 130)
    {
      bSuhu = Suhu;
    }
    pesan = String() + "," + ID + "," + Latitude + "," + Longitude + "," + Altitude + "," + SOG + "," + COG + "," + AccX + "," + AccY + "," + AccZ + "," + GyroX + "," + GyroY + "," + GyroZ + "," + MagX + "," + MagY + "," + MagZ + "," + roll + "," + pitch + "," + yaw + "," + ABAT + "," + /*VBAT*/ VBAT + "," + /*HR*/ HR + "," + /*SPO*/ SPO + "," + bSuhu + ",*";
    display.println(String() + "   " + ID);
    display.println(String() + "LAT : " + Latitude);
    display.println(String() + "LON : " + Longitude);
    if (state == false)
    {
      display.println(String() + "STAT: NOT CONNECTED");
      display.println(String() + "RSSI: NAN");
    }
    else
    {
      display.println(String() + "STAT: " + STAT);
      display.println(String() + "RSSI: " + RSI);
    }

    if (state2 == false)
    {
      display.println(String() + "VBAT: " + VBAT + "mV");
    }
    else
    {
      display.println(String() + "VBAT: NO BATTERY");
    }

    //    Serial.println(String()+ "FREE RAM : " + ESP.getFreeHeap() + "\tRAM SIZE : " +  ESP.getHeapSize() + "\t USED RAM : " + (ESP.getMaxAllocHeap()/1000));

    display.println(String() + "   USED       USED");
    display.println(String() + "RAM:" + (ESP.getMaxAllocHeap() / 1000) + "kb  ROM: NAN kb");
    display.display();

    Serial.println("02" + pesan);

    Suhu = 0;
    lastSendTime = millis(); // timestamp the message
    interval = 1000;
  }

  else
  {
    while (GPS2.available())
    {
      gps.encode(GPS2.read());
    }
  }

  if (millis() - lastSendTime2 > 1000)
  {
    // particleSensor.heartrateAndOxygenSaturation(&SPO2, &SPO2Valid, &heartRate, &heartRateValid);
    count++;
    if (irValue > 50000)
    {
      HR = String(beatAvg);
      SPO = String(ESpO2, 2);
    }
    else
    {
      HR = "NAN";
      SPO = "NAN";
    }

    amg.readPixels(pixels);
    int x, y;

    for (y = 0; y <= AMG88xx_PIXEL_ARRAY_SIZE; y++)
    {
      // for (x = 35; x <= 37; x++){
      float tmp = pixels[y - 1];
      if (Suhu < tmp)
      {
        Suhu = tmp;
        // tT -= 0.4;
      }
    }

    if (count >= 5)
    {
      state = false;
    }
    lastSendTime2 = millis(); // timestamp the message
  }

  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(MasterNode);        // add destination address
  LoRa.write(Node_Halter);       // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();       // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  {         // check length for error
            // Serial.println("error: message length does not match length");
    return; // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != Node_Halter && recipient != MasterNode)
  {
    // Serial.println("This message is not for me.");
    state = false;
    return; // skip rest of function
  }

  else if (recipient == Node_Halter)
  {
    STAT = String() + "CONNECTED";
    state = true;
    RSI = String(LoRa.packetRssi());

    String Val = incoming;
    if (Val == "02,SHMKR100601")
    {
      String message = String(pesan);
      sendMessage(message, MasterNode, Node_Halter);
      delay(100);
    }
  }
}

void Task1code(void *parameter)
{
  for (;;)
  {
    uint32_t ir, red, green;
    double fred, fir;
    double SpO2 = 0; // raw SpO2 before low pass filtered

#ifdef USEFIFO
    particleSensor.check(); // Check the sensor, read up to 3 samples

    while (particleSensor.available())
    { // do we have new data
#ifdef MAX30105
      red = particleSensor.getFIFORed(); // Sparkfun's MAX30105
      ir = particleSensor.getFIFOIR();   // Sparkfun's MAX30105
#else
      red = particleSensor.getFIFOIR(); // why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
      ir = particleSensor.getFIFORed(); // why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board

      irValue = particleSensor.getIR();

      if (checkForBeat(irValue) == true && irValue > 50000)
      {
        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
          rateSpot %= RATE_SIZE;                    // Wrap variable

          // Take average of readings
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }

#endif
      i++;
      fred = (double)red;
      fir = (double)ir;
      avered = avered * frate + (double)red * (1.0 - frate); // average red level by low pass filter
      aveir = aveir * frate + (double)ir * (1.0 - frate);    // average IR level by low pass filter
      sumredrms += (fred - avered) * (fred - avered);        // square sum of alternate component of red level
      sumirrms += (fir - aveir) * (fir - aveir);             // square sum of alternate component of IR level
      if ((i % SAMPLING) == 0)
      { // slow down graph plotting speed for arduino Serial plotter by thin out
        if (millis() > TIMETOBOOT)
        {
          //        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
          //        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
          float ir_forGraph = 2.0 * (fir - aveir) / aveir * SCALE + (MIN_SPO2 + MAX_SPO2) / 2.0;
          float red_forGraph = 2.0 * (fred - avered) / avered * SCALE + (MIN_SPO2 + MAX_SPO2) / 2.0;
          // trancation for Serial plotter's autoscaling
          if (ir_forGraph > 100.0)
            ir_forGraph = 100.0;
          if (ir_forGraph < 80.0)
            ir_forGraph = 80.0;
          if (red_forGraph > 100.0)
            red_forGraph = 100.0;
          if (red_forGraph < 80.0)
            red_forGraph = 80.0;
          //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
          if (ir < FINGER_ON)
            ESpO2 = MINIMUM_SPO2; // indicator for finger detached
          // Serial.print(ir_forGraph); // to display pulse wave at the same time with SpO2 data
          // Serial.print(",");
          // Serial.print(red_forGraph); // to display pulse wave at the same time with SpO2 data
          // Serial.print(",");
          // if (ESpO2 <= 81)
          // {
          //     ESpO2 = 0;
          // }

          // Serial.print(beatAvg);
          // Serial.print(",");
          // Serial.print(ESpO2); // low pass filtered SpO2
          // Serial.print(85.0); // reference SpO2 line
          // Serial.print(",");
          // Serial.print(90.0); // warning SpO2 line
          // Serial.print(",");
          // Serial.print(95.0); // safe SpO2 line
          // Serial.print(",");
          // Serial.print(100.0); // max SpO2 line
          // Serial.print("IR=");
          // Serial.print(irValue);
          // Serial.print(", BPM=");
          // Serial.print(beatsPerMinute);
          // Serial.print(", Avg BPM=");

          // if (irValue < 50000)
          //   Serial.print(" No finger?");

          // Serial.println();
        }
      }
      if ((i % Num) == 0)
      {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        // Serial.println(R);
        SpO2 = -23.3 * (R - 0.4) + 100;               // http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // low pass filter
        //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
        sumredrms = 0.0;
        sumirrms = 0.0;
        i = 0;
        break;
      }
      particleSensor.nextSample(); // We're finished with this sample so move to next sample
                                   // Serial.println(SpO2);
    }
#else

    while (1)
    {                                // do we have new data
#ifdef MAX30105
      red = particleSensor.getRed(); // Sparkfun's MAX30105
      ir = particleSensor.getIR();   // Sparkfun's MAX30105
#else
      red = particleSensor.getIR(); // why getFOFOIR outputs Red data by MAX30102 on MH-ET LIVE breakout board
      ir = particleSensor.getRed(); // why getFIFORed outputs IR data by MAX30102 on MH-ET LIVE breakout board
#endif
      i++;
      fred = (double)red;
      fir = (double)ir;
      avered = avered * frate + (double)red * (1.0 - frate); // average red level by low pass filter
      aveir = aveir * frate + (double)ir * (1.0 - frate);    // average IR level by low pass filter
      sumredrms += (fred - avered) * (fred - avered);        // square sum of alternate component of red level
      sumirrms += (fir - aveir) * (fir - aveir);             // square sum of alternate component of IR level
      if ((i % SAMPLING) == 0)
      { // slow down graph plotting speed for arduino IDE toos menu by thin out
        // #if 0
        if (millis() > TIMETOBOOT)
        {
          float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
          float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
          // trancation for Serial plotter's autoscaling
          if (ir_forGraph > 100.0)
            ir_forGraph = 100.0;
          if (ir_forGraph < 80.0)
            ir_forGraph = 80.0;
          if (red_forGraph > 100.0)
            red_forGraph = 100.0;
          if (red_forGraph < 80.0)
            red_forGraph = 80.0;
          //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
          if (ir < FINGER_ON)
            ESpO2 = MINIMUM_SPO2;                            // indicator for finger detached
          Serial.print((2.0 * fir - aveir) / aveir * SCALE); // to display pulse wave at the same time with SpO2 data
          Serial.print(",");
          Serial.print((2.0 * fred - avered) / avered * SCALE); // to display pulse wave at the same time with SpO2 data
          Serial.print(",");
          Serial.print(ESpO2); // low pass filtered SpO2
          Serial.print(",");
          Serial.print(85.0); //
          Serial.print(",");
          Serial.print(90.0); // warning SpO2 line
          Serial.print(",");
          Serial.print(95.0); // safe SpO2 line
          Serial.print(",");
          Serial.println(100.0); // max SpO2 line
                                 // #endif
        }
      }
      if ((i % Num) == 0)
      {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        // Serial.println(R);
        SpO2 = -23.3 * (R - 0.4) + 100; // http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
        //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
        sumredrms = 0.0;
        sumirrms = 0.0;
        i = 0;
        break;
      }
      particleSensor.nextSample(); // We're finished with this sample so move to next sample
                                   // Serial.println(SpO2);
    }
#endif
  }
}

#endif