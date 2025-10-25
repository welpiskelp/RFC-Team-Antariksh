/*************************************************************************
   COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: (Your Name / Gemini)
   All rights reserved.
*************************************************************************/

#include "lora_logger.h"
#include <LoRa.h>
#include <Arduino.h>
#include <SPI.h>

// ********************************************************************
// !! CRITICAL CONFIGURATION !!
// You MUST update these pins to match your LoRa module's wiring.
// The SPI bus (SPI1) is shared with your other sensors.
// ********************************************************************

#define LORA_CS_PIN   10  // SPI Chip Select
#define LORA_RST_PIN  9   // Reset
#define LORA_IRQ_PIN  2   // DIO0 (Interrupt Request)
#define LORA_FREQ     915E6 // LoRa Frequency (433 MHz)

static bool lora_ready = false;

/**
 * @brief Initialize the LoRa module on the SPI1 bus.
 */
static void lora_init(void) {
    // Set SPI1 bus for LoRa library
    LoRa.setSPI(SPI1);
    
    // Set LoRa pins
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_IRQ_PIN);

    Serial.println("Attempting LoRa init...");

    // Start LoRa
    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("LoRa: begin failed!");
        lora_ready = false;
        return;
    }
    
    // LoRa.setTxPower(20); // Set TX power (default is 17)
    Serial.println("LoRa: begun OK");
    lora_ready = true;
}

/**
 * @brief Write data (telemetry string) over LoRa.
 */
static void lora_write(const char *msg, size_t len) {
    if (!lora_ready || len == 0) return;

    // Go to Idle mode (stops RX if active)
    LoRa.idle(); 

    // Send packet
    if (LoRa.beginPacket()) {
        LoRa.write((const uint8_t*)msg, len);
        LoRa.endPacket(); // endPacket() is blocking (waits for TX to finish)
    }
}

/**
 * @brief Helper to send an ACK packet back to ground station.
 * This function returns to RX mode after sending.
 */
void lora_send_ack(const char* ack_msg) {
    if (!lora_ready) return;

    LoRa.idle(); // Stop receiving
    
    LoRa.beginPacket();
    LoRa.print(ack_msg);
    LoRa.endPacket();

    // Go back to listening for more commands
    LoRa.receive();
}


/* Exported logger_interface_t object */
logger_interface_t lora_logger_interface = {
    .init = lora_init,
    .write = lora_write,
    .is_immediate_flush = true /* Send packets immediately */
};