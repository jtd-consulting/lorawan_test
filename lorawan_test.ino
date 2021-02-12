/******************************************************************************
 *
 *  Filename    : lorawan_test.ino
 *  Author      : Joe Deu-Ngoc
 *  Date        : 2021/02/11
 *  Description : Arduino sketch to demonstrate LoRaWAN OTAA
 *
 ******************************************************************************
 *
 *  Copyright (c) 2021 JTD Consulting
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the
 *  "Software"), to deal in the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************/
#include <basicmac.h>
#include <hal/hal.h>
#include <SPI.h>


/******************************************************************************
 * Macros
 ******************************************************************************/
#define SERIAL_WAIT_MS                    (5000)
#define MIN_TX_SPACING_MS                 (60000)


/******************************************************************************
 * setup (Arduino entrypoint)
 * --------------------------
 * Setup Serial console
 * Setup BasicMAC stack
 * Apply channel mask
 * Start join procedure
 * 
 ******************************************************************************/
void setup(void)
{
  uint32_t boot_time = millis();
  
  Serial.begin(115200);
  while(millis() - boot_time < SERIAL_WAIT_MS && !Serial);

  os_init(nullptr);
  LMIC_reset();

  LMIC_startJoining();
}

/******************************************************************************
 * loop (Arduino loop)
 * -------------------
 * Handle BasicMAC events
 * Send a packet, if possible
 ******************************************************************************/
void loop(void)
{
  static uint32_t pkt_tx_ms = millis();

  os_runstep();

  if(!(LMIC.opmode & (OP_JOINING | OP_TXRXPEND)) &&
    millis() - pkt_tx_ms > MIN_TX_SPACING_MS)
  {
    uint8_t buf[] = "0123456789!";  // Make sure we fit in DR0 packet
    LMIC_setTxData2(1, buf, sizeof(buf) - 1, 0);
    pkt_tx_ms = millis();
  }
}

/******************************************************************************
 * BasicMAC required symbols
 ******************************************************************************
 * channel_is_allowed
 * ------------------
 * Checks if channel is in allowed channel list defined by the application
 ******************************************************************************/
extern "C" int channel_is_allowed(u1_t channel)
{
  static const uint8_t allowed_channels[] =
  {
    0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40
  };

  if(channel / 8 >= sizeof(allowed_channels))
  {
    return 0;
  }

  return (allowed_channels[channel / 8] & (0x80 >> (channel % 8))) != 0;
}

/****************************************************************************** 
 * lmic_pins
 * ---------
 * Configure hardware pins for LoRa radio
 ******************************************************************************/
const lmic_pinmap lmic_pins =
{
    .nss = 36,
    .tx = LMIC_UNUSED_PIN,
    .rx = LMIC_UNUSED_PIN,
    .rst = 44,
    .dio = { LMIC_UNUSED_PIN, 40, LMIC_UNUSED_PIN},
    .busy = 39,
    .tcxo = LMIC_UNUSED_PIN,
};

/******************************************************************************
 * os_getDevEui
 * ------------
 * Gives the stack the Device EUI in little-endian order as required by
 * BasicMAC
 * Setting is big-endian (as-is from TTN Console)
 ******************************************************************************/
void os_getDevEui(u1_t* buf)
{
  static const u1_t PROGMEM v[] =
  {
    0xCC, 0x5D, 0x78, 0xFF, 0xFE, 0x77, 0x07, 0x08
  };

  for(int i = 0; i < sizeof(v); i++)
  {
    buf[7 - i] = v[i];
  }
}

/******************************************************************************
 * os_getJoinEui
 * -------------
 * Gives the stack the Join EUI, p.k.a. Application EUI, in little-endian
 * order as required by BasicMAC
 * Setting is big-endian (as-is from TTN Console)
 ******************************************************************************/
void os_getJoinEui(u1_t* buf)
{
  static const u1_t PROGMEM v[] =
  {
    0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0xCE, 0x92
  };

  for(int i = 0; i < 8; i++)
  {
    buf[7 - i] = v[i];
  }
}

/******************************************************************************
 * os_getNwkKey
 * Gives the stack the Network Key in big-endian order as required by BasicMAC
 * Setting is big-endian (as-is from TTN Console)
 ******************************************************************************/
void os_getNwkKey(u1_t* buf)
{
  static const u1_t PROGMEM v[] =
  {
    0x12, 0x59, 0xE6, 0x47, 0x93, 0x4D, 0xAC, 0xDE,
    0x6B, 0x26, 0xD6, 0x3A, 0x9F, 0xD8, 0x6C, 0x7D
  };

  for(int i = 0; i < sizeof(v); i++)
  {
    buf[i] = v[i];
  }
}

/******************************************************************************
 * os_getRegion
 * ------------
 * Always return the US region
 ******************************************************************************/
u1_t os_getRegion(void)
{
  return REGCODE_US915;
}

/******************************************************************************
 * onLmicEvent
 * -----------
 * BasicMAC event callback
 ******************************************************************************/
void onLmicEvent(ev_t event)
{
  static const char * const name[] =
  {
    "",
    "EV_SCAN_TIMEOUT",
    "EV_BEACON_FOUND",
    "EV_BEACON_MISSED",
    "EV_BEACON_TRACKED",
    "EV_JOINING",
    "EV_JOINED",
    "EV_RFU1",
    "EV_JOIN_FAILED",
    "EV_REJOIN_FAILED",
    "EV_TXCOMPLETE",
    "EV_LOST_TSYNC",
    "EV_RESET",
    "EV_RXCOMPLETE",
    "EV_LINK_DEAD",
    "EV_LINK_ALIVE",
    "EV_SCAN_FOUND",
    "EV_TXSTART",
    "EV_TXDONE",
    "EV_DATARATE",
    "EV_START_SCAN",
    "EV_ADR_BACKOFF"
  };

  switch(event)
  {
    case EV_JOINED:
      Serial.println(name[event]);
      LMIC_setLinkCheckMode(0);
      break;

    case EV_TXCOMPLETE:
      Serial.println(name[event]);
      if(LMIC.txrxFlags & TXRX_ACK)
      {
        // Do something with the acknowledgement
      }
      if(LMIC.dataLen)
      {
        // Do something with the payload
      }
      break;

    case EV_SCAN_TIMEOUT:
    case EV_BEACON_FOUND:
    case EV_BEACON_MISSED:
    case EV_BEACON_TRACKED:
    case EV_JOINING:
    case EV_RFU1:
    case EV_JOIN_FAILED:
    case EV_REJOIN_FAILED:
    case EV_LOST_TSYNC:
    case EV_RESET:
    case EV_RXCOMPLETE:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
    case EV_SCAN_FOUND:
    case EV_TXSTART:
    case EV_TXDONE:
    case EV_DATARATE:
    case EV_START_SCAN:
    case EV_ADR_BACKOFF:
      Serial.println(name[event]);
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println(event);
      break;
  }
}
