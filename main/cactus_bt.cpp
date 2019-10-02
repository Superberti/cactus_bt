/*
 * LED-Kaktus Ansteuerungssoftware zur Demonstration von
 * Industrieautomatisationsprotokollen. Diesmal mit Bluetooth
 *
 * 30.09.2019 O. Rutsch, Sympatec GmbH
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp32_digital_led_lib.h"
#include "tools.h"
#include <driver/gpio.h>
#include <math.h>
#include <string.h>
#include <vector>
#include "time.h"
#include "sys/time.h"
#include "CactusCommands.h"
#include "CactusErrors.h"
#include "fireworks_effects.h"
#include "WS2812FX.h"

using namespace std;

#define SPP_SERVER_NAME "CACTUS_SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "Sympatec Cactus"
#define NUM_LEDS 144
#define MAX_LINE_SIZE 2048
#define HIGH 1
#define LOW 0
#define OUTPUT GPIO_MODE_OUTPUT
#define INPUT GPIO_MODE_INPUT

static bool smConnected = false;
static char *pCurrentCommandLine;
int gActiveEffect = 0;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const char *TAG = "BT Cactus";

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

// Prototypes
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void app_cpp_main(void *);
void LedEffect(int aEffectNum, int aDuration_ms);
vector<uint8_t> GetLEDsPercent(double aPercent);
int CheckRGB(std::string &, std::string &, std::string &, int &, int &, int &);
void SetCactusPercent(int aPercentage, pixelColor_t aColor);
uint32_t ProcessCommandLine(char *aCmdLine);
void DelayMilliseconds(uint32_t ms);

strand_t STRANDS[] = {
    // Avoid using any of the strapping pins on the ESP32
    {.rmtChannel = 0, .gpioNum = 17, .ledType = LED_WS2813_V2, .brightLimit = 255, .numPixels = NUM_LEDS, .pixels = NULL, ._stateVars = NULL},
};

int STRANDCNT = sizeof(STRANDS) / sizeof(STRANDS[0]);

extern "C" void app_main(void)
{
  pCurrentCommandLine = (char *)malloc(MAX_LINE_SIZE);
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  // Kein WiFi, wir machen Bluetooth

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_init()) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK)
  {
    ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

#if (CONFIG_BT_SSP_ENABLED == true)
  /* Set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

  /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
  esp_bt_pin_code_t pin_code;
  esp_bt_gap_set_pin(pin_type, 0, pin_code);

  xTaskCreatePinnedToCore(app_cpp_main,   /* Function to implement the task */
                          "app_cpp_main", /* Name of the task */
                          10000,          /* Stack size in words */
                          NULL,           /* Task input parameter */
                          0,              /* Priority of the task */
                          NULL,           /* Task handle. */
                          1);             /* Core where the task should run */
  while (true)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  free(pCurrentCommandLine);
  return;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  switch (event)
  {
  case ESP_SPP_INIT_EVT:
    ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
    esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
    break;
  case ESP_SPP_DISCOVERY_COMP_EVT:
    ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
    break;
  case ESP_SPP_OPEN_EVT:
    ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
    break;
  case ESP_SPP_CLOSE_EVT:
    ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
    smConnected = false;
    break;
  case ESP_SPP_START_EVT:
    ESP_LOGI(TAG, "ESP_SPP_START_EVT");
    break;
  case ESP_SPP_CL_INIT_EVT:
    ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
    break;
  case ESP_SPP_DATA_IND_EVT:
  {
    ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
             param->data_ind.len, param->data_ind.handle);
    esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
    // Wir gehen immer von einem Textprotokoll aus, deshalb wird die Zeile kopiert und null-terminiert
    int length = min(MAX_LINE_SIZE, param->data_ind.len + 1);
    memcpy(pCurrentCommandLine, param->data_ind.data, length);
    pCurrentCommandLine[length - 1] = 0;
    ProcessCommandLine(pCurrentCommandLine);
  }
  break;
  case ESP_SPP_CONG_EVT:
    ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
    break;
  case ESP_SPP_WRITE_EVT:
    ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
    break;
  case ESP_SPP_SRV_OPEN_EVT:
    smConnected = true;
    ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
    break;
  default:
    break;
  }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  switch (event)
  {
  case ESP_BT_GAP_AUTH_CMPL_EVT:
  {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
      esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
    }
    else
    {
      ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
    }
    break;
  }
  case ESP_BT_GAP_PIN_REQ_EVT:
  {
    ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
    if (param->pin_req.min_16_digit)
    {
      ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
      esp_bt_pin_code_t pin_code = {0};
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
    }
    else
    {
      ESP_LOGI(TAG, "Input pin code: 1234");
      esp_bt_pin_code_t pin_code;
      pin_code[0] = '1';
      pin_code[1] = '2';
      pin_code[2] = '3';
      pin_code[3] = '4';
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
    break;
  }

#if (CONFIG_BT_SSP_ENABLED == true)
  case ESP_BT_GAP_CFM_REQ_EVT:
    ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
    esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
    break;
  case ESP_BT_GAP_KEY_NOTIF_EVT:
    ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
    break;
  case ESP_BT_GAP_KEY_REQ_EVT:
    ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
    break;
#endif

  default:
  {
    ESP_LOGI(TAG, "event: %d", event);
    break;
  }
  }
  return;
}

//uint32_t IRAM_ATTR millis() { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

void DelayMilliseconds(uint32_t ms)
{
  uint32_t m = millis();
  if (ms)
  {
    uint32_t e = (m + ms);
    if (m > e)
    { //overflow
      while (millis() > e)
      {
        __asm__ __volatile__("nop;");
      }
    }
    while (millis() < e)
    {
      __asm__ __volatile__("nop;");
    }
  }
}

class Rainbower
{
private:
  const uint8_t color_div = 4;
  const uint8_t anim_step = 5;
  uint8_t anim_max;
  uint8_t stepVal1;
  uint8_t stepVal2;
  uint8_t StepCounter = 0;
  pixelColor_t color1;
  pixelColor_t color2;

public:
  Rainbower();
  pixelColor_t drawNext();
};

Rainbower::Rainbower()
{
  anim_max = 255 - anim_step;
  stepVal1 = 0;
  stepVal2 = 0;
  color1 = pixelFromRGB(anim_max, 0, 0);
  color2 = pixelFromRGB(anim_max, 0, 0);
}

pixelColor_t Rainbower::drawNext()
{
  pixelColor_t CurrentColor = pixelFromRGB(color1.r / color_div, color1.g / color_div, color1.b / color_div);
  switch (stepVal1)
  {
  case 0:
    color1.g += anim_step;
    if (color1.g >= anim_max)
      stepVal1++;
    break;
  case 1:
    color1.r -= anim_step;
    if (color1.r == 0)
      stepVal1++;
    break;
  case 2:
    color1.b += anim_step;
    if (color1.b >= anim_max)
      stepVal1++;
    break;
  case 3:
    color1.g -= anim_step;
    if (color1.g == 0)
      stepVal1++;
    break;
  case 4:
    color1.r += anim_step;
    if (color1.r >= anim_max)
      stepVal1++;
    break;
  case 5:
    color1.b -= anim_step;
    if (color1.b == 0)
      stepVal1 = 0;
    break;
  }
  StepCounter++;
  return CurrentColor;
}

class Scannerer
{
private:
  strand_t *pStrand;
  pixelColor_t BlackColor;
  unsigned int currIdx;

public:
  Scannerer(strand_t *);
  void drawNext(pixelColor_t);
};

Scannerer::Scannerer(strand_t *pStrandIn)
{
  pStrand = pStrandIn;
  BlackColor = pixelFromRGB(0, 0, 0);
  currIdx = 0;
}

void Scannerer::drawNext(pixelColor_t aColor)
{
  const int GlowTail = 30;
  float r, g, b;
  for (int i = 0; i < GlowTail; i++)
  {
    pixelColor_t CurrentColor = aColor;
    r = CurrentColor.r;
    g = CurrentColor.g;
    b = CurrentColor.b;
    r /= ((20.0 / GlowTail) * i + 1);
    g /= ((20.0 / GlowTail) * i + 1);
    b /= ((20.0 / GlowTail) * i + 1);
    CurrentColor.r = r;
    CurrentColor.g = g;
    CurrentColor.b = b;
    pStrand->pixels[(currIdx - i) % pStrand->numPixels] = CurrentColor;
  }
  pStrand->pixels[(currIdx - GlowTail) % pStrand->numPixels] = BlackColor;
  strand_t *MyStrand[] = {pStrand};
  digitalLeds_drawPixels(MyStrand, STRANDCNT);
  currIdx++;
}

Rainbower MyRainbow;

void scanner(strand_t *pStrand, unsigned long delay_ms, unsigned long timeout_ms)
{
  Scannerer MyScanner(pStrand);

  for (int i = 0; i < pStrand->numPixels; i++)
  {
    pixelColor_t c = MyRainbow.drawNext();
    MyScanner.drawNext(c);
    delay(delay_ms);
  }
}

// digitales Abbild des Kaktus, jeden cm abgerastert von unten nach oben
const vector<vector<uint8_t>> cactus{
    {0, 143},
    {1, 2, 142},
    {3, 4, 140, 141},
    {5, 139},
    {6, 138},
    {7, 8, 136, 137},
    {9, 135},
    {10, 133, 134},
    {11, 131, 132},
    {12, 13, 127, 128, 129, 130},
    {14, 15, 16, 125, 126, 96, 97, 98, 99},
    {17, 18, 19, 94, 95, 100, 101, 124, 123},
    {20, 48, 49, 92, 93, 102, 103, 121, 122},
    {21, 22, 46, 47, 50, 51, 90, 91, 104, 120},
    {23, 44, 45, 52, 53, 88, 89, 105, 119},
    {24, 43, 54, 87, 106, 107, 117, 118},
    {25, 26, 41, 42, 55, 86, 108, 116},
    {27, 40, 56, 57, 85, 109, 115},
    {28, 29, 38, 39, 58, 59, 84, 83, 110, 111, 113, 114},
    {30, 31, 37, 60, 81, 82, 112},
    {32, 36, 61, 80},
    {33, 34, 35, 62, 79, 78},
    {63, 64, 77},
    {65, 75, 76},
    {66, 74, 67},
    {68, 73},
    {69, 72},
    {70, 71},
};

vector<uint8_t> GetLEDsPercent(double aPercent)
{
  const int Columns = cactus.size();
  int MaxCol = int(Columns * (aPercent / 100.0) + 0.5);
  vector<uint8_t> Leds;
  for (int i = 0; i < MaxCol; i++)
  {
    Leds.insert(Leds.end(), cactus.at(i).begin(), cactus.at(i).end());
  }
  return Leds;
}

void app_cpp_main(void *pvParameters)
{
  bool toggle = false;
  gpioSetup(17, OUTPUT, LOW);
  gpioSetup(2, OUTPUT, HIGH);

  digitalLeds_initDriver();

  for (int i = 0; i < STRANDCNT; i++)
  {
    gpioSetup(STRANDS[i].gpioNum, OUTPUT, LOW);
  }

  strand_t *MyStrand[] = {&STRANDS[0]};
  int rc = digitalLeds_addStrands(MyStrand, STRANDCNT);
  if (rc)
  {
    ets_printf("digitalLeds_addStrands error code: %d. Halting\n", rc);
    while (true)
    {
      toggle = !toggle;
      gpio_set_level(GPIO_NUM_2, (uint32_t)toggle);
      delay(100);
    };
  }
  /*
  if (digitalLeds_initStrands(STRANDS, STRANDCNT))
  {
    ets_printf("Init FAILURE: halting\n");
    while (true)
    {
      toggle = !toggle;
      gpio_set_level(GPIO_NUM_2, (uint32_t)toggle);
      delay(100);
    };
  }
*/

  uint32_t CurrentTime = 0;
  uint32_t LastTime = millis();
  Rainbower MyRainbow;
  bool ConnectMsg = true;
  for (int i = 0; i <= 100; i += 5)
  {
    MyRainbow.drawNext();
    MyRainbow.drawNext();
    pixelColor_t Color = MyRainbow.drawNext();
    SetCactusPercent(i, Color);
    delay(50);
  }

  delay(500);
  for (int i = 100; i >= 0; i -= 5)
  {
    MyRainbow.drawNext();
    MyRainbow.drawNext();
    pixelColor_t Color = MyRainbow.drawNext();
    SetCactusPercent(i, Color);
    delay(50);
  }

  for (;;)
  {
    if (gActiveEffect > 0)
      LedEffect(-1, 0); // Effektschleife aktivieren
    // Status der LEDs jede Sekunde veröffentlichen
    CurrentTime = millis();
    if ((CurrentTime - LastTime) > 1000)
    {
      LastTime = CurrentTime;
      if (smConnected)
      {
        if (ConnectMsg)
        {
          ConnectMsg = false;
          pixelColor_t Color = pixelFromRGB(0, 100, 0);
          SetCactusPercent(100, Color);
          delay(50);
          SetCactusPercent(0, Color);
          delay(50);
          SetCactusPercent(100, Color);
          delay(50);
          SetCactusPercent(0, Color);
        }

        toggle = !toggle;
        gpio_set_level(GPIO_NUM_2, (uint32_t)toggle);
      }
      else
      {
        ConnectMsg = true;
        pixelColor_t Color = pixelFromRGB(30, 0, 0);
        SetCactusPercent(100, Color);
        delay(50);
        SetCactusPercent(0, Color);
      }
    }
    delay(10);
  }
  vTaskDelete(NULL);
}

uint32_t ProcessCommandLine(char *aCmdLine)
{
  std::vector<std::string> iTokens;
  KillReturnAndEndl(aCmdLine);
  string InputLine(aCmdLine);
  ParseLine(InputLine, iTokens);

  TCactusCommand CurrentCommand = eCMD_UNKNOWN;

  if (iTokens.size() == 0)
  {
    ESP_LOGI(TAG, "Empty command detected...");
    return ERR_EMPTYCMD;
  }
  // Nachsehen, welches Kommando gesendet wurde
  for (unsigned int i = 0; i < sizeof(CactusCommands) / sizeof(CactusCommands[0]); i++)
  {
    if (iTokens.at(0) == std::string(CactusCommands[i].mCmdStr))
    {
      CurrentCommand = CactusCommands[i].mCmd;
      break;
    }
  }
  if (CurrentCommand == eCMD_UNKNOWN)
  {
    ESP_LOGI(TAG, "Unknown cactus command: %s", iTokens.at(0).c_str());
    return ERR_UNKNOWNCOMMAND;
  }
  if ((int(iTokens.size()) - 1) < CactusCommands[CurrentCommand].mNumParams)
  {
    ESP_LOGI(TAG, "Number of parameters incorrect: %d %d %s", int(iTokens.size()) - 1, CactusCommands[CurrentCommand].mNumParams,
             InputLine.c_str());
    return ERR_NUMPARAMS;
  }

  // ESP_LOGI(TAG, "Executing command: %d", CurrentCommand);

  strand_t *pStrand = &STRANDS[0];

  if (gActiveEffect > 0 && CurrentCommand != eCMD_LED_EFFECT)
  {
    ESP_LOGI(TAG, "Command not allowed while effect is active: %s", InputLine.c_str());
    return ERR_EFFECT_RUNNING;
  }

  // Jetzt können die Kommandos ausgewertet und verarbeitet werden.
  switch (CurrentCommand)
  {

  case eCMD_SET_LED:
  {
    int iLedNumber = -1;
    int r, g, b;
    iLedNumber = strtol(iTokens.at(1).c_str(), NULL, 10);
    if (iLedNumber < 0 || iLedNumber >= NUM_LEDS)
    {
      ESP_LOGI(TAG, "LED number out of bounds (0-%d): %s", NUM_LEDS, iTokens.at(1).c_str());
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    int Err = CheckRGB(iTokens.at(2), iTokens.at(3), iTokens.at(4), r, g, b);
    if (Err != ERR_OK)
    {
      return Err;
    }

    pixelColor_t Color = pixelFromRGB(r, g, b);

    pStrand->pixels[iLedNumber] = Color;
    strand_t *MyStrand[] = {pStrand};
    digitalLeds_drawPixels(MyStrand, STRANDCNT);
    ESP_LOGI(TAG, "SETLED %d %d %d %d executed.", iLedNumber, r, g, b);
    // delay(50);
    break;
  }
  case eCMD_SET_LED_RANGE:
  {
    int iStartLedNumber = -1;
    int iEndLedNumber = -1;
    int r, g, b;
    iStartLedNumber = strtol(iTokens.at(1).c_str(), NULL, 10);
    iEndLedNumber = strtol(iTokens.at(2).c_str(), NULL, 10);
    if (iStartLedNumber < 0 || iStartLedNumber > NUM_LEDS)
    {
      ESP_LOGI(TAG, "Start LED number out of bounds (0-%d): %s", NUM_LEDS, iTokens.at(1).c_str());
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    if (iEndLedNumber < 0 || iEndLedNumber > NUM_LEDS)
    {
      ESP_LOGI(TAG, "End LED number out of bounds (0-%d): %s", NUM_LEDS, iTokens.at(2).c_str());
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    if (iStartLedNumber > iEndLedNumber)
    {
      ESP_LOGI(TAG, "Start LED number > end LED number: Start:%s End:%s", iTokens.at(1).c_str(), iTokens.at(2).c_str());
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    int NumLedsToSet = iEndLedNumber - iStartLedNumber + 1;
    int ExpectedParams = NumLedsToSet * 3;
    if (ExpectedParams > iTokens.size() - 3)
    {
      ESP_LOGI(TAG, "Too few RGB parameters. Expected %d, got %d", ExpectedParams, iTokens.size() - 3);
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    // ESP_LOGI(TAG, "Start:%d End:%d Num LEDs to set:%d", iStartLedNumber, iEndLedNumber, NumLedsToSet);
    for (int i = iStartLedNumber; i <= iEndLedNumber; i++)
    {
      int Err = CheckRGB(iTokens.at(3 + (i - iStartLedNumber) * 3), iTokens.at(4 + (i - iStartLedNumber) * 3),
                         iTokens.at(5 + (i - iStartLedNumber) * 3), r, g, b);
      // printf("r:%d g%d b%d",r,g,b);
      if (Err != ERR_OK)
      {
        return Err;
      }

      pixelColor_t Color = pixelFromRGB(r, g, b);

      pStrand->pixels[i] = Color;
    }
    strand_t *MyStrand[] = {pStrand};
    digitalLeds_drawPixels(MyStrand, STRANDCNT);
    break;
  }
  case eCMD_LED_EFFECT:
  {
    int EffectNumber = strtol(iTokens.at(1).c_str(), NULL, 10);
    if (EffectNumber < 0 || EffectNumber > 59)
    {
      ESP_LOGI(TAG, "Effect number out of bounds (0-59): %s", iTokens.at(1).c_str());
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    gActiveEffect = EffectNumber;
    break;
  }
  case eCMD_FILL_CACTUS:
  {
    int r, g, b;
    int FillPercent = strtol(iTokens.at(1).c_str(), NULL, 10);
    if (FillPercent < 0 || FillPercent > 100)
    {
      ESP_LOGI(TAG, "Percentage out of bounds (0-100): %s", iTokens.at(1).c_str());
      return ERR_PARAM_OUT_OF_BOUNDS;
    }
    int Err = CheckRGB(iTokens.at(2), iTokens.at(3), iTokens.at(4), r, g, b);
    if (Err != ERR_OK)
    {
      return Err;
    }
    pixelColor_t Color = pixelFromRGB(r, g, b);
    SetCactusPercent(FillPercent, Color);
    break;
  }
  default:
  {
    ESP_LOGI(TAG, "Command not (yet) implemented: %s", InputLine.c_str());
    return ERR_CMD_NOTSUPPORTED;
  }
  }
  return ERR_OK;
}

int CheckRGB(std::string &rs, std::string &gs, std::string &bs, int &r, int &g, int &b)
{
  r = -1;
  g = -1;
  b = -1;
  r = strtol(rs.c_str(), NULL, 10);
  g = strtol(gs.c_str(), NULL, 10);
  b = strtol(bs.c_str(), NULL, 10);

  if (r < 0 || r > 255)
  {
    ESP_LOGI(TAG, "Parameter red out of bounds (0-255): %s", rs.c_str());
    return ERR_PARAM_OUT_OF_BOUNDS;
  }
  if (g < 0 || g > 255)
  {
    ESP_LOGI(TAG, "Parameter green out of bounds (0-255): %s", gs.c_str());
    return ERR_PARAM_OUT_OF_BOUNDS;
  }
  if (b < 0 || b > 255)
  {
    ESP_LOGI(TAG, "Parameter blue out of bounds (0-255): %s", bs.c_str());
    return ERR_PARAM_OUT_OF_BOUNDS;
  }
  return ERR_OK;
}

void SetCactusPercent(int aPercentage, pixelColor_t aColor)
{
  strand_t *pStrand = &STRANDS[0];
  pixelColor_t BlackColor = pixelFromRGB(0, 0, 0);
  vector<uint8_t> Leds = GetLEDsPercent(aPercentage);
  for (int t = 0; t < pStrand->numPixels; t++)
    pStrand->pixels[t] = BlackColor;
  for (int t = 0; t < Leds.size(); t++)
    pStrand->pixels[Leds.at(t)] = aColor;
  strand_t *MyStrand[] = {pStrand};
  digitalLeds_drawPixels(MyStrand, STRANDCNT);
}

void LedEffect(int aEffectNum, int aDuration_ms)
{
  bool toggle = true;
  strand_t *pStrand = &STRANDS[0];

  Rainbower MyRainbow;
  Scannerer MyScanner(pStrand);
  FireworksEffects Firework(pStrand);
  strand_t *MyStrand[] = {pStrand};
  WS2812FX ws2812fx(pStrand);
  ws2812fx.setSpeed(100);
  ws2812fx.setColor(0x007BFF);

  int StartTime = millis();
  int i = 0;
  // Bei aEffectNum=-1 wird auf den gActiveEffect gelauscht
  int LastEffect = 0;
  while (((millis() - StartTime) < aDuration_ms) || aEffectNum == -1)
  {
    int CurrentEffect = aEffectNum;
    if (aEffectNum == -1)
    {
      if (gActiveEffect == 0)
      {
        // Kaktus ausschalten
        pixelColor_t BlackColor = pixelFromRGB(0, 0, 0);
        for (int t = 0; t < pStrand->numPixels; t++)
        {
          pStrand->pixels[t] = BlackColor;
        }
        digitalLeds_drawPixels(MyStrand, STRANDCNT);
        break; // Effektschleife beenden
      }
      else
      {
        CurrentEffect = gActiveEffect;
      }
      // Effekt hat sich geändert?
      if (LastEffect != CurrentEffect)
      {
        LastEffect = CurrentEffect;
        if (CurrentEffect > 7 && CurrentEffect < 60)
        {
          ws2812fx.stop();
          ws2812fx.setMode(CurrentEffect);
          ws2812fx.start();
        }
      }
    }

    switch (CurrentEffect)
    {
    case 1:
    {
      pixelColor_t c = MyRainbow.drawNext();
      MyScanner.drawNext(c);
      delay(15);

      break;
    }
    case 2:
    {
      for (int t = 0; t < pStrand->numPixels / 2; t++)
      {
        pixelColor_t c = MyRainbow.drawNext();
        pStrand->pixels[t] = c;
        pStrand->pixels[pStrand->numPixels - t - 1] = c;
        digitalLeds_drawPixels(MyStrand, STRANDCNT);
        delay(15);
      }
      break;
    }
    case 3:
    {
      pixelColor_t BlackColor = pixelFromRGB(0, 0, 0);

      for (int t = 0; t < pStrand->numPixels; t++)
      {
        pixelColor_t BlinkColor = MyRainbow.drawNext();
        pixelColor_t c = toggle ? BlackColor : BlinkColor;
        pStrand->pixels[t] = c;
      }
      digitalLeds_drawPixels(MyStrand, STRANDCNT);
      delay(50);

      break;
    }

    case 4:
    {
      pixelColor_t BlackColor = pixelFromRGB(0, 128, 0);
      pixelColor_t WhiteColor = pixelFromRGB(255, 255, 255);

      for (int t = 0; t < pStrand->numPixels; t++)
      {
        pixelColor_t BlinkColor = ((t + i) % 5) ? BlackColor : WhiteColor;
        pStrand->pixels[t] = BlinkColor;
      }
      digitalLeds_drawPixels(MyStrand, STRANDCNT);
      delay(50);

      break;
    }

    case 5:
    {
      pixelColor_t BlackColor = pixelFromRGB(128, 0, 0);
      pixelColor_t WhiteColor = pixelFromRGB(255, 255, 255);

      for (int t = 0; t < pStrand->numPixels; t++)
      {
        pixelColor_t BlinkColor = ((t + i) % 5) ? BlackColor : WhiteColor;
        pStrand->pixels[t] = BlinkColor;
      }
      digitalLeds_drawPixels(MyStrand, STRANDCNT);
      delay(50);

      break;
    }

    case 6:
    {
      pixelColor_t GreenColor = pixelFromRGB(rand() % 256, rand() % 256, rand() % 256);
      pixelColor_t BlackColor = pixelFromRGB(0, 0, 0);
      for (int i = 1; i <= 100; i++)
      {
        vector<uint8_t> Leds = GetLEDsPercent(i);
        for (int t = 0; t < pStrand->numPixels; t++)
          pStrand->pixels[t] = BlackColor;
        for (int t = 0; t < Leds.size(); t++)
          pStrand->pixels[Leds.at(t)] = GreenColor;
        digitalLeds_drawPixels(MyStrand, STRANDCNT);
        delay(10);
      }
      break;
    }

    case 7:
    {
      Firework.Render();
      delay(10);
      break;
    }

    } // end switch
    if (CurrentEffect > 7 && CurrentEffect < 60)
    {
      ws2812fx.service();
    }
    toggle = !toggle;
    i++;
    gpio_set_level(GPIO_NUM_2, (uint32_t)toggle);
  }
}