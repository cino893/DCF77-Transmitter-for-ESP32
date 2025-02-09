/*
  based on this sketch: https://github.com/aknik/ESP32/blob/master/DFC77/DFC77_esp32_Solo.ino

  Some functions are inspired by work of G6EJD ( https://www.youtube.com/channel/UCgtlqH_lkMdIa4jZLItcsTg )

  Refactor by DeltaZero, converts to syncronous, added "cron" that you can bypass, see line 29
                                                    The cron does not start until 10 minutes from reset (see constant onTimeAfterReset)
  Every clock I know starts to listen to the radio at aproximatelly the hour o'clock, so cron takes this into account

  Alarm clocks from Junghans: Every hour (innecesery)
  Weather Station from Brigmton: 2 and 3 AM
  Chinesse movements and derivatives: 1 o'clock AM
*/

//WARNING: DOESN'T WORK ON ESP32C3 PROPABLY DUE TO NO LEDC_HIGH_SPEED_MODE

#include <WiFi.h>

#include <Ticker.h>

#include <Time.h>

#include <driver/i2s.h>


#include "credentials.h"  // If you put this file in the same forlder that the rest of the tabs, then use "" to delimiter,
 // otherwise use <> or comment it and write your credentials directly on code
// const char* ssid = "YourOwnSSID";
// const char* password = "YourSoSecretPassword";

// #define LEDBUILTIN 5      // LED pin, LED flashes when antenna is transmitting
// C3 has no controllable build IN LED - use serial to debug
#define ANTENNAPIN D6 // Antenna pin. Connect antenna from here to ground, use a 1k resistor to limit transmitting power. A slightly tuned ferrite antenna gets around 3 meters and a wire loop may work if close enough.

#define I2S_NUM         I2S_NUM_0
#define SAMPLE_RATE     77500      // 77.5 kHz
#define CHANNELS        1         // Mono
#define BIT_DEPTH        I2S_BITS_PER_SAMPLE_8BIT
#define I2S_PIN_SDA     D4         // SDA pin
#define I2S_PIN_SCL     D5         // SCL pin

// Frequencies for 15% and 100% duty cycles
#define DUTY_CYCLE_15   0.15f
#define DUTY_CYCLE_100  1.0f
#define BUFFER_SIZE     256        // Liczba próbek w buforze
// Buffer for I2S data
int8_t *i2s_buffer; // 8-bit samples
int8_t i2s_buffer_sine_high[BUFFER_SIZE]; // 8-bit samples
int8_t i2s_buffer_sine_low[BUFFER_SIZE]; // 8-bit samples

// #define CONTINUOUSMODE // Uncomment this line to bypass de cron and have the transmitter on all the time

// cron (if you choose the correct values you can even run on batteries)
// If you choose really bad this minutes, everything goes wrong, so minuteToWakeUp must be greater than minuteToSleep
#define minuteToWakeUp 58 // Every hoursToWakeUp at this minute the ESP32 wakes up get time and star to transmit
#define minuteToSleep 15 + 2 // If it is running at this minute then goes to sleep and waits until minuteToWakeUp

byte hoursToWakeUp[] = {
  0,
  3
}; // you can add more hours to adapt to your needs
// When the ESP32 wakes up, check if the actual hour is in the list and
// runs or goes to sleep until next minuteToWakeUp

Ticker tickerDecisec; // TBD at 100ms

//complete array of pulses for a minute
//0 = no pulse, 1=100ms, 2=200ms
int impulseArray[60];
int impulseCount = 0;
int actualHours, actualMinutes, actualSecond, actualDay, actualMonth, actualYear, DayOfW;
long dontGoToSleep = 0;
const long onTimeAfterReset = 60000 * 15; // Fifteen minutes (typical clock max fetch time)
int timeRunningContinuous = 0;

const char * ntpServer = "es.pool.ntp.org"; // enter your closer pool or pool.ntp.org
const char * TZ_INFO = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"; // enter your time zone (https://remotemonitoringsystems.ca/time-zone-abbreviations.php)

struct tm timeinfo;

String signalStr = "";
char signalE = '?';

void setup() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  Serial.begin(115200);
  Serial.println();
  Serial.println("DCF77 transmitter");

  // can be added to save energy when battery-operated

  if (setCpuFrequencyMhz(80)) {
    Serial.print("CPU frequency set @");
    Serial.print(getCpuFrequencyMhz());
    Serial.println("Mhz");
  } else
    Serial.println("Fail to set cpu frequency");

  if (esp_sleep_get_wakeup_cause() == 0) dontGoToSleep = millis();

//   ledcAttach(ANTENNAPIN, 77500, 8); // Set pin PWM, 77500hz DCF freq, resolution of 8bit
  initI2S();
//   ledcWrite(ANTENNAPIN, 0);
  signalE = '0';

  //  pinMode (LEDBUILTIN, OUTPUT);
  //  digitalWrite (LEDBUILTIN, LOW); // LOW if LEDBUILTIN is inverted like in Wemos boards

  WiFi_on();
  getNTP();
  WiFi_off();
  show_time();

  CodeTime(); // first conversion just for cronCheck
  #ifndef CONTINUOUSMODE
  if ((dontGoToSleep == 0) or((dontGoToSleep + onTimeAfterReset) < millis())) cronCheck(); // first check before start anything
  #else
  Serial.println("CONTINUOUS MODE NO CRON!!!");
  #endif

  // sync to the start of a second
  Serial.print("Syncing... ");
  int startSecond = timeinfo.tm_sec;
  long count = 0;
  do {
    count++;
    if (!getLocalTime( & timeinfo)) {
      Serial.println("Error obtaining time...");
      delay(3000);
      ESP.restart();
    }
  } while (startSecond == timeinfo.tm_sec);

  tickerDecisec.attach_ms(1000, DcfOut); // from now on calls DcfOut every 100ms
  Serial.print("Ok ");
  Serial.println(count);
}

void loop() {
  // There is no code inside the loop. This is a syncronous program driven by the Ticker
}

// Inicjalizacja I2S
void initI2S() {
    i2s_config_t i2s_config = {
        mode: (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        sample_rate: SAMPLE_RATE,
        bits_per_sample: I2S_BITS_PER_SAMPLE_8BIT,
        channel_format: I2S_CHANNEL_FMT_ONLY_RIGHT,
        communication_format: I2S_COMM_FORMAT_I2S,
        dma_buf_count: 8,
        dma_buf_len: BUFFER_SIZE,
    };

    i2s_pin_config_t pin_config = {
        bck_io_num: D5,
        ws_io_num: D4,
        data_out_num: 18,
        data_in_num: -1
    };

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

// Generowanie 77,5 kHz fali sinusoidalnej w buforze
void generateSineWave() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float angle = (2.0f * M_PI * i) / BUFFER_SIZE;
        i2s_buffer_sine_high[i] = 127 * sin(angle);  // Przeskalowane do 8-bitów (-128 do 127)
    }
    for (int i = 0; i < BUFFER_SIZE; i++) {
        i2s_buffer_sine_low[i] =  i2s_buffer_sine_high[i] * 0.20;  // obniżenie do 20%
    }
}

// Wysłanie bitu DCF77
void sendDCF77Bit(int bit) {
    size_t bytes_written;

    if (bit == 2) {
        signalE = '1';
        // Bit "1" - obniżenie amplitudy przez 200 ms
        i2s_buffer=i2s_buffer_sine_high;
        i2s_write(I2S_NUM, i2s_buffer, BUFFER_SIZE, &bytes_written, 200);

        i2s_buffer=i2s_buffer_sine_low;
        i2s_write(I2S_NUM, i2s_buffer, BUFFER_SIZE, &bytes_written, 800);
    } else if(bit == 1){
        signalE = '0';
        // Bit "0" - obniżenie amplitudy przez 100 ms
        i2s_buffer=i2s_buffer_sine_high;
        i2s_write(I2S_NUM, i2s_buffer, BUFFER_SIZE, &bytes_written, 100);

        i2s_buffer=i2s_buffer_sine_low;
        i2s_write(I2S_NUM, i2s_buffer, BUFFER_SIZE, &bytes_written, 900);
    } else {
        i2s_buffer=i2s_buffer_sine_high;
        i2s_write(I2S_NUM, i2s_buffer, BUFFER_SIZE, &bytes_written, 1000);
    }
}

void CodeTime() {
  DayOfW = timeinfo.tm_wday;
  if (DayOfW == 0) DayOfW = 7;
  actualDay = timeinfo.tm_mday;
  actualMonth = timeinfo.tm_mon + 1;
  actualYear = timeinfo.tm_year - 100;
  actualHours = timeinfo.tm_hour;
  actualMinutes = timeinfo.tm_min + 1; // DCF77 transmitts the next minute
  if (actualMinutes >= 60) {
    actualMinutes = 0;
    actualHours++;
  }
  actualSecond = timeinfo.tm_sec;
  if (actualSecond == 60) actualSecond = 0;

  int n, Tmp, TmpIn;
  int ParityCount = 0;

  //we put the first 20 bits of each minute at a logical zero value
  for (n = 0; n < 20; n++) impulseArray[n] = 1;

  // set DST bit
  if (timeinfo.tm_isdst == 0) {
    impulseArray[18] = 2; // CET or DST OFF
  } else {
    impulseArray[17] = 2; // CEST or DST ON
  }

  //bit 20 must be 1 to indicate active time
  impulseArray[20] = 2;

  //calculates the bits for the minutes
  TmpIn = Bin2Bcd(actualMinutes);
  for (n = 21; n < 28; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  if ((ParityCount & 1) == 0)
    impulseArray[28] = 1;
  else
    impulseArray[28] = 2;

  //calculates bits for the hours
  ParityCount = 0;
  TmpIn = Bin2Bcd(actualHours);
  for (n = 29; n < 35; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  if ((ParityCount & 1) == 0)
    impulseArray[35] = 1;
  else
    impulseArray[35] = 2;
  ParityCount = 0;

  //calculate the bits for the actual Day of Month
  TmpIn = Bin2Bcd(actualDay);
  for (n = 36; n < 42; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = Bin2Bcd(DayOfW);
  for (n = 42; n < 45; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //calculates the bits for the actualMonth
  TmpIn = Bin2Bcd(actualMonth);
  for (n = 45; n < 50; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //calculates the bits for actual year
  TmpIn = Bin2Bcd(actualYear); // 2 digit year
  for (n = 50; n < 58; n++) {
    Tmp = TmpIn & 1;
    impulseArray[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  //equal date
  if ((ParityCount & 1) == 0)
    impulseArray[58] = 1;
  else
    impulseArray[58] = 2;

  //last missing pulse
  impulseArray[59] = 0; // No pulse
}

int Bin2Bcd(int dato) {
  int msb, lsb;
  if (dato < 10)
    return dato;
  msb = (dato / 10) << 4;
  lsb = dato % 10;
  return msb + lsb;
}

void DcfOut() {

    Serial.println("co sie dzieje");
    Serial.println("" + signalStr);
    signalStr = "";


  sendDCF77Bit(impulseArray[actualSecond]);

    if (actualSecond == 1 || actualSecond == 15 || actualSecond == 21 || actualSecond == 29) Serial.print("-");
    if (actualSecond == 36 || actualSecond == 42 || actualSecond == 45 || actualSecond == 50) Serial.print("-");
    if (actualSecond == 28 || actualSecond == 35 || actualSecond == 58) Serial.print("P");

    if (impulseArray[actualSecond] == 1) Serial.println("0");
    if (impulseArray[actualSecond] == 2) Serial.println("1");
    if (impulseArray[actualSecond] == 0) Serial.println("x");

    if (actualSecond == 59) {
      Serial.println("");
      show_time();
      #ifndef CONTINUOUSMODE
      if ((dontGoToSleep == 0) or((dontGoToSleep + onTimeAfterReset) < millis())) cronCheck();
      #else
      Serial.println("CONTINUOUS MODE NO CRON!!!");
      timeRunningContinuous++;
      if (timeRunningContinuous > 360) ESP.restart(); // 6 hours running, then restart all over
      #endif
    }

  signalStr += signalE;

  if (!getLocalTime( & timeinfo)) {
    Serial.println("Error obtaining time...");
    delay(3000);
    ESP.restart();
  }
  CodeTime();
}
