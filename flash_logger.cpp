/*************************************************************************
   COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: Aarush Jaiswal & Rik Seth

   Flash (EEPROM) logger backend using ring buffer.
   - Compatible with logger_interface_t from logger.h
   - Writes logs to EEPROM as fixed-size entries.
   - Allows later SD dump for permanent storage.
*************************************************************************/

#include "flash_logger.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>

/* Persistent ring buffer metadata stored in EEPROM (first few bytes after 64) */
typedef struct {
    uint16_t head;   // next write index
    uint16_t tail;   // next read index
    bool full;
} flash_ring_meta_t;

/* Local cached metadata */
static flash_ring_meta_t meta;
static bool flash_ready = false;

/* EEPROM address layout */
#define META_ADDR_START   FLASH_LOG_START_ADDR
#define META_SIZE         sizeof(flash_ring_meta_t)
#define DATA_START_ADDR   (META_ADDR_START + META_SIZE)
#define DATA_REGION_SIZE  (FLASH_LOG_TOTAL_BYTES - META_SIZE)
#define ENTRY_SIZE        FLASH_LOG_ENTRY_SIZE
#define ENTRY_CAPACITY    (DATA_REGION_SIZE / ENTRY_SIZE)

/* Helper macros */
static inline uint16_t entry_to_addr(uint16_t index) {
    return DATA_START_ADDR + (index * ENTRY_SIZE);
}

/* ---------- internal helpers ---------- */

static void load_meta() {
    EEPROM.get(META_ADDR_START, meta);
    if (meta.head >= ENTRY_CAPACITY || meta.tail >= ENTRY_CAPACITY) {
        meta.head = 0;
        meta.tail = 0;
        meta.full = false;
    }
}

static void save_meta() {
    EEPROM.put(META_ADDR_START, meta);
    EEPROM.commit();
}

static bool ring_is_empty() {
    return (!meta.full && (meta.head == meta.tail));
}

static void ring_advance() {
    if (meta.full) {
        meta.tail = (meta.tail + 1) % ENTRY_CAPACITY;
    }
    meta.head = (meta.head + 1) % ENTRY_CAPACITY;
    meta.full = (meta.head == meta.tail);
}

/* ---------- flash logger interface ---------- */

static void flashlogger_init(void) {
    EEPROM.begin(FLASH_LOG_TOTAL_BYTES);
    load_meta();
    flash_ready = true;
}

static void flashlogger_write(const char* msg, size_t len) {
    if (!flash_ready || len == 0) return;

    char entry[ENTRY_SIZE];
    memset(entry, 0, ENTRY_SIZE);
    size_t copy_len = (len > ENTRY_SIZE - 1) ? (ENTRY_SIZE - 1) : len;
    memcpy(entry, msg, copy_len);
    entry[ENTRY_SIZE - 1] = '\0';

    /* Compute EEPROM address for this entry */
    uint16_t addr = entry_to_addr(meta.head);

    /* Write entire entry */
    for (size_t i = 0; i < ENTRY_SIZE; i++) {
        EEPROM.write(addr + i, (uint8_t)entry[i]);
    }

    ring_advance();
    save_meta();
}

/* ---------- EEPROM → SD dump ---------- */
void flashlogger_flush_to_sd(const char* filename) {
    if (!flash_ready) return;
    File f = SD.open(filename, FILE_WRITE);
    if (!f) {
        Serial.println("FlashLogger: Failed to open SD file!");
        return;
    }

    Serial.println("FlashLogger: Dumping EEPROM log buffer...");

    uint16_t count = meta.full ? ENTRY_CAPACITY : (meta.head >= meta.tail ?
                    (meta.head - meta.tail) : (ENTRY_CAPACITY + meta.head - meta.tail));

    for (uint16_t i = 0; i < count; i++) {
        uint16_t index = (meta.tail + i) % ENTRY_CAPACITY;
        uint16_t addr = entry_to_addr(index);

        char buf[ENTRY_SIZE + 2];
        for (size_t j = 0; j < ENTRY_SIZE; j++) {
            buf[j] = (char)EEPROM.read(addr + j);
        }
        buf[ENTRY_SIZE] = '\n';
        buf[ENTRY_SIZE + 1] = '\0';

        f.write((uint8_t*)buf, ENTRY_SIZE + 1);
    }

    f.flush();
    f.close();

    Serial.print("FlashLogger: Dump complete, entries written = ");
    Serial.println(count);
}

/* ---------- exported interface ---------- */
logger_interface_t flash_logger_interface = {
    .init = flashlogger_init,
    .write = flashlogger_write,
    .is_immediate_flush = false
};
