#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration: adjust to target system */
#define LOG_ENTRY_SIZE        128   /* bytes per log entry */
#define CRIT_BUFFER_ENTRIES   32    /* number of critical entries */
#define LOG_BUFFER_ENTRIES    16    /* number of regular entries */
#define MAX_LOGGER_INTERFACES 4

typedef struct logger_interface {
    void (*init)(void);
    void (*write)(const char *msg, size_t len);
    bool is_immediate_flush;
} logger_interface_t;

/* Helper macro to register a static logger instance */
#define REGISTER_LOGGER(_name, _init, _write, _immediate) \
    static logger_interface_t _name##_logger = {          \
        .init = (_init),                                 \
        .write = (_write),                               \
        .is_immediate_flush = (_immediate)               \
    }

/* Initialization / registration */
void logger_init(void);
/* returns true if registration succeeded */
bool logger_register_interface(logger_interface_t *interface);
/* flush buffered logs to non-immediate interfaces */
void logger_flush(void);

/**
 * @brief Initializes the logger's EEPROM addresses.
 * @param base_addr The starting address for EEPROM data (e.g., EEPROM_DATA_ADDR).
 * @param next_addr The address to write the *next* log entry
 * (e.g., EEPROM_DATA_ADDR or a recovered address).
 */
//void logger_init_eeprom(uint16_t base_addr, uint16_t next_addr);

/**
 * @brief Gets the next available EEPROM write address.
 * @return The 16-bit EEPROM address.
 */
//uint16_t logger_get_next_eeprom_addr(void);

/**
 * @brief Reads all log data from EEPROM and writes it to the SD card.
 * This function will call the registered SD card logger's write function.
 */
//void flush_eeprom_to_sd(void);

/* logging APIs (source-location aware) */
void log_fatal(const char* file, const char *func, int line, const char *fmt, ...);
void log_error(const char* file, const char *func, int line, const char *fmt, ...);
void log_warn (const char* file, const char *func, int line, const char *fmt, ...);
void log_info (const char* file, const char *func, int line, const char *fmt, ...);
void log_debug(const char* file, const char *func, int line, const char *fmt, ...);

/* --------------------------------------------------------------
 * Added for telemetry support (matches logger.cpp)
 * -------------------------------------------------------------- */

/**
 * @brief Telemetry data structure (matches data budget format).
 */
typedef struct {
    const char* team_id;       // <TEAM ID>
    double time_stamp;         // <TIME STAMPING> (Seconds)
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
    double accel_data_x;       // <ACCELEROMETER DATA> (m/s^2)
    double accel_data_y;
    double accel_data_z;
    double gyro_data_x;
    double gyro_data_y;
    double gyro_data_z;
    const char* flight_state;  // <FLIGHT SOFTWARE STATE>
} telemetry_data_t;

/**
 * @brief Log a telemetry packet (structured data format).
 * @param data Pointer to telemetry data structure.
 * @param critical True if critical data (goes to critical buffer).
 */
void log_telemetry(const telemetry_data_t *data, bool critical);

/* -------------------------------------------------------------- */

/* convenience macros to pass file/func/line */
#define BASE_FILE_NAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_FATAL(...) log_fatal(BASE_FILE_NAME, __func__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) log_error(BASE_FILE_NAME, __func__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)  log_warn(BASE_FILE_NAME, __func__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...)  log_info(BASE_FILE_NAME, __func__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...) log_debug(BASE_FILE_NAME, __func__, __LINE__, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H */
