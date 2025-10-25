/*************************************************************************
   COPYRIGHT NOTICE
   (c) 2025 Team Antariksh
   Author: Rik Seth (refined & telemetry-format integrated)
*************************************************************************/

#include "logger.h"
#include <string.h>
#include <stdio.h>   /* snprintf, vsnprintf */
#include <stdlib.h>  /* size_t */
#include <EEPROM.h>           // <-- ADD THIS
#include "sdcard_logger.h"

/* Static storage for ring buffers (entries * bytes per entry) */
static char crit_buff[CRIT_BUFFER_ENTRIES * LOG_ENTRY_SIZE];
static char log_buff[ LOG_BUFFER_ENTRIES  * LOG_ENTRY_SIZE ];

/* Ring buffer for fixed-size entries */
typedef struct {
    char   *storage;     /* pointer to contiguous storage */
    size_t entry_size;   /* LOG_ENTRY_SIZE */
    size_t capacity;     /* number of entries */
    size_t head;         /* next write index (0..capacity-1) */
    size_t tail;         /* next read index */
    bool   full;
} ring_entries_t;

/* Logger registry */
static logger_interface_t *registered_loggers[MAX_LOGGER_INTERFACES];
static size_t              num_registered = 0;

/* immediate flush logger list (subset of registered) */
static logger_interface_t *immediate_loggers[MAX_LOGGER_INTERFACES];
static size_t              num_immediate = 0;

/* NEW: EEPROM logging state */
static logger_interface_t* sd_logger = NULL;
static uint16_t eeprom_base_addr = 0;
static uint16_t eeprom_next_addr = 0;


/* ring buffers */
static ring_entries_t crit_ring;
static ring_entries_t log_ring;

/* ---------- ring helpers ---------- */

static void ring_init(ring_entries_t *r, char *storage, size_t entry_size, size_t capacity) {
    r->storage = storage;
    r->entry_size = entry_size;
    r->capacity = capacity;
    r->head = 0;
    r->tail = 0;
    r->full = false;
}

static bool ring_is_empty(const ring_entries_t *r) {
    return (!r->full && (r->head == r->tail));
}

static bool ring_is_full(const ring_entries_t *r) {
    return r->full;
}

static size_t ring_count(const ring_entries_t *r) {
    if (r->full) return r->capacity;
    if (r->head >= r->tail) return r->head - r->tail;
    return r->capacity + r->head - r->tail;
}

static void ring_advance_pointer(ring_entries_t *r) {
    if (r->full) {
        r->tail = (r->tail + 1) % r->capacity;
    }
    r->head = (r->head + 1) % r->capacity;
    r->full = (r->head == r->tail);
}

static void ring_retreat_pointer(ring_entries_t *r) {
    r->full = false;
    r->tail = (r->tail + 1) % r->capacity;
}

static bool ring_enqueue_entry(ring_entries_t *r, const char *entry) {
    if (ring_is_full(r)) {
        return false;
    }
    char *dest = r->storage + (r->head * r->entry_size);
    memcpy(dest, entry, r->entry_size);
    ring_advance_pointer(r);
    return true;
}

static bool ring_dequeue_entry(ring_entries_t *r, char *out_entry) {
    if (ring_is_empty(r)) return false;
    char *src = r->storage + (r->tail * r->entry_size);
    memcpy(out_entry, src, r->entry_size);
    ring_retreat_pointer(r);
    return true;
}

/* ---------- logger internals ---------- */

static void write_to_non_immediate(const char *msg, size_t len) {
    for (size_t i = 0; i < num_registered; ++i) {
        logger_interface_t *li = registered_loggers[i];
        if (li && !li->is_immediate_flush && li->write) {
            li->write(msg, len);
        }
    }
}

void logger_init(void) {
    //ring_init(&crit_ring, crit_buff, LOG_ENTRY_SIZE, CRIT_BUFFER_ENTRIES);
    //ring_init(&log_ring,  log_buff,  LOG_ENTRY_SIZE, LOG_BUFFER_ENTRIES);
    num_registered = 0;
    num_immediate = 0;
    memset(crit_buff, 0, sizeof(crit_buff));
    memset(log_buff,  0, sizeof(log_buff));
    sd_logger = NULL;
    eeprom_base_addr = 0;
    eeprom_next_addr = 0;
}


bool logger_register_interface(logger_interface_t *interface) {
    if (!interface) return false;
    for (size_t i = 0; i < num_registered; ++i) {
        if (registered_loggers[i] == interface) return false;
    }
    if (num_registered >= MAX_LOGGER_INTERFACES) return false;
    registered_loggers[num_registered++] = interface;
    if (interface->is_immediate_flush) {
        if (num_immediate < MAX_LOGGER_INTERFACES) {
            immediate_loggers[num_immediate++] = interface;
        }
        else {
            // NEW: Check if this is the sdcard logger (which is non-immediate)
            // This is a bit of a hack, but effective.
            if (interface == &sdcard_logger_interface) {
                sd_logger = interface;
        }
    }
    if (interface->init) interface->init();
    return true;
}
}

void logger_flush(void) {
    /*
    char entry[LOG_ENTRY_SIZE];
    while (ring_dequeue_entry(&crit_ring, entry)) {
        entry[LOG_ENTRY_SIZE - 1] = '\0';
        write_to_non_immediate(entry, strnlen(entry, LOG_ENTRY_SIZE));
    }
    while (ring_dequeue_entry(&log_ring, entry)) {
        entry[LOG_ENTRY_SIZE - 1] = '\0';
        write_to_non_immediate(entry, strnlen(entry, LOG_ENTRY_SIZE));
    }
    */
}

    /* ---------- NEW EEPROM Functions ---------- */

    void logger_init_eeprom(uint16_t base_addr, uint16_t next_addr) {
        eeprom_base_addr = base_addr;
        eeprom_next_addr = next_addr;

        // Sanity check
        if (eeprom_next_addr < eeprom_base_addr) {
            eeprom_next_addr = eeprom_base_addr;
        }
    }

    uint16_t logger_get_next_eeprom_addr(void) {
        return eeprom_next_addr;
    }

    void flush_eeprom_to_sd(void) {
        // Check if SD logger was found and EEPROM was initialized
        if (!sd_logger || !sd_logger->write || eeprom_base_addr == 0) {
            return;
        }

        char read_buf[LOG_ENTRY_SIZE];

        for (uint16_t addr = eeprom_base_addr; addr < eeprom_next_addr; addr += LOG_ENTRY_SIZE) {
            // Stop if we somehow read past the end
            if (addr + LOG_ENTRY_SIZE > EEPROM.length()) {
                break;
            }

            EEPROM.get(addr, read_buf);
            read_buf[LOG_ENTRY_SIZE - 1] = '\0'; // Ensure null termination

            size_t len = strnlen(read_buf, LOG_ENTRY_SIZE);

            if (len > 0) {
                // Write the log entry string to the SD card
                sd_logger->write(read_buf, len);
            }
        }

        // After writing all data, force sync the SD card file
        sdcard_force_sync();
    }

/* ---------- TELEMETRY DATA FORMAT LOGGING ---------- */
/* The telemetry log will follow the format specified in the Data Budget table */
/*
typedef struct {
    //const char* team_id;       // <TEAM ID>
    //double time_stamp;         // <TIME STAMPING> (Seconds)
    unsigned int packet_count; // <PACKET COUNT>
    double altitude;           // <ALTITUDE> (m)
    double pressure;           // <PRESSURE> (Pa)
    double temp;               // <TEMP> (°C)
    double voltage;            // <VOLTAGE> (V)
    double gnss_time;          // <GNSS TIME> (Seconds)
    double gnss_lat;           // <GNSS LATITUDE> (deg)
    double gnss_lon;           // <GNSS LONGITUDE> (deg)
    double gnss_alt;           // <GNSS ALTITUDE> (m)
    int gnss_sats;             // <GNSS SATS> (count)
    double accel_data;         // <ACCELEROMETER DATA> (m/s^2)
   // const char* flight_state;  // <FLIGHT SOFTWARE STATE>
} telemetry_data_t;*/

/* Internal function to format telemetry data */
static void format_telemetry(char *buf, size_t buf_size, const telemetry_data_t *t) {
    snprintf(buf, buf_size,
        "%s, %.2f, %u, %.2f, %.2f, %.2f, %.2f, %.2f, %.4f, %.4f, %.2f, %d, %.2f,%.2f,%.2f,%.2f,%.2f,%.2f, %s",
        t->team_id, t->time_stamp, t->packet_count, t->altitude, t->pressure,
        t->temp, t->voltage, t->gnss_time, t->gnss_lat, t->gnss_lon,
        t->gnss_alt, t->gnss_sats, t->accel_data_x,t->accel_data_y,t->accel_data_z,t->gyro_data_x,t->gyro_data_y,t->gyro_data_z, t->flight_state);
}

/* ---------- logger_emit modified for telemetry ---------- */

static void logger_emit(bool critical, const telemetry_data_t *data) {
    char buf[LOG_ENTRY_SIZE];
    format_telemetry(buf, LOG_ENTRY_SIZE, data);
    size_t msg_len = strnlen(buf, LOG_ENTRY_SIZE);

    for (size_t i = 0; i < num_immediate; ++i) {
        logger_interface_t *li = immediate_loggers[i];
        if (li && li->write) {
            li->write(buf, msg_len);
        }
    }

   /* if (critical) {
        if (!ring_enqueue_entry(&crit_ring, buf)) {
            char tmp[LOG_ENTRY_SIZE];
            if (ring_dequeue_entry(&crit_ring, tmp)) {
                (void)ring_enqueue_entry(&crit_ring, buf);
            }
        }
    }

    if (!ring_enqueue_entry(&log_ring, buf)) {
        char tmp[LOG_ENTRY_SIZE];
        if (ring_dequeue_entry(&log_ring, tmp)) {
            (void)ring_enqueue_entry(&log_ring, buf);
        }
    }
*/
    /*
     * 3. NEW: Write to EEPROM
     */
    if (eeprom_base_addr != 0 && (eeprom_next_addr + LOG_ENTRY_SIZE) <= EEPROM.length()) {
        EEPROM.put(eeprom_next_addr, buf);
        eeprom_next_addr += LOG_ENTRY_SIZE;
    }
}

/* ---------- Public function to log telemetry ---------- */
void log_telemetry(const telemetry_data_t *data, bool critical) {
    if (!data) return;
    logger_emit(critical, data);
}

/* ---------- Legacy severity-based logs (kept intact) ---------- */
void log_fatal(const char* file, const char *func, int line, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char msg[LOG_ENTRY_SIZE];
    vsnprintf(msg, LOG_ENTRY_SIZE, fmt, args);
    va_end(args);

    telemetry_data_t t = {
        .team_id = "2024-ASI-ROCKETRY-027",
        .time_stamp = 0, .packet_count = 0, .altitude = 0,
        .pressure = 0, .temp = 0, .voltage = 0,
        .gnss_time = 0, .gnss_lat = 0, .gnss_lon = 0,
        .gnss_alt = 0, .gnss_sats = 0, .accel_data_x = 0, .accel_data_y=0,.accel_data_z=0,
        .gyro_data_x = 0, .gyro_data_y = 0, .gyro_data_z = 0,
        .flight_state = msg
    };
    logger_emit(true, &t);
}

void log_error(const char* file, const char *func, int line, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char msg[LOG_ENTRY_SIZE];
    vsnprintf(msg, LOG_ENTRY_SIZE, fmt, args);
    va_end(args);

    telemetry_data_t t = {
        .team_id = "2024-ASI-ROCKETRY-027",
        .time_stamp = 0, .packet_count = 0, .altitude = 0,
        .pressure = 0, .temp = 0, .voltage = 0,
        .gnss_time = 0, .gnss_lat = 0, .gnss_lon = 0,
        .gnss_alt = 0, .gnss_sats = 0, .accel_data_x = 0, .accel_data_y=0,.accel_data_z=0,
        .gyro_data_x = 0, .gyro_data_y = 0, .gyro_data_z = 0,
        .flight_state = msg
    };
    logger_emit(true, &t);
}

void log_warn(const char* file, const char *func, int line, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char msg[LOG_ENTRY_SIZE];
    vsnprintf(msg, LOG_ENTRY_SIZE, fmt, args);
    va_end(args);

    telemetry_data_t t = {
        .team_id = "2024-ASI-ROCKETRY-027",
        .time_stamp = 0, .packet_count = 0, .altitude = 0,
        .pressure = 0, .temp = 0, .voltage = 0,
        .gnss_time = 0, .gnss_lat = 0, .gnss_lon = 0,
        .gnss_alt = 0, .gnss_sats = 0, .accel_data_x = 0, .accel_data_y=0,.accel_data_z=0,
        .gyro_data_x = 0, .gyro_data_y = 0, .gyro_data_z = 0,
        .flight_state = msg
    };
    logger_emit(false, &t);
}

void log_info(const char* file, const char *func, int line, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char msg[LOG_ENTRY_SIZE];
    vsnprintf(msg, LOG_ENTRY_SIZE, fmt, args);
    va_end(args);

    telemetry_data_t t = {
        .team_id = "2024-ASI-ROCKETRY-027",
        .time_stamp = 0, .packet_count = 0, .altitude = 0,
        .pressure = 0, .temp = 0, .voltage = 0,
        .gnss_time = 0, .gnss_lat = 0, .gnss_lon = 0,
        .gnss_alt = 0, .gnss_sats = 0, .accel_data_x = 0, .accel_data_y=0,.accel_data_z=0,
        .gyro_data_x = 0, .gyro_data_y = 0, .gyro_data_z = 0,
        .flight_state = msg
    };
    logger_emit(false, &t);
}

void log_debug(const char* file, const char *func, int line, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char msg[LOG_ENTRY_SIZE];
    vsnprintf(msg, LOG_ENTRY_SIZE, fmt, args);
    va_end(args);

    telemetry_data_t t = {
        .team_id = "2024-ASI-ROCKETRY-027",
        .time_stamp = 0, .packet_count = 0, .altitude = 0,
        .pressure = 0, .temp = 0, .voltage = 0,
        .gnss_time = 0, .gnss_lat = 0, .gnss_lon = 0,
        .gnss_alt = 0, .gnss_sats = 0, .accel_data_x = 0, .accel_data_y=0,.accel_data_z=0,
        .gyro_data_x = 0, .gyro_data_y = 0, .gyro_data_z = 0,
        .flight_state = msg
    };
    logger_emit(false, &t);
}
