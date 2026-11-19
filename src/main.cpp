#include <Arduino.h>
#include <time.h>
#include <NimBLEDevice.h>
void displaySetup(); void botoesSetup(); void relogioSetup(); void botoesUpdate(); void displayUpdate(); void displayForcarRedesenho(); void relogioAgora(struct tm*); void ecuSendWakeup(); void ecuPoll();
void handleRoot(); void handleData(); void handleTestLog(); void handleLogs(); void handleAllDataPage(); void handleClearCel();
void notifyCallback(NimBLERemoteCharacteristic*,uint8_t*,size_t,bool);

#include "original_main.h"
#include "botoes.h"
#include "ecu_civic98.h"
#include "display.h"
#include "web_dashboard.h"