#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Sensor positions and their corresponding bit-shifts.
 */
typedef enum {
    US_FRONT = 0,
    US_LEFT  = 1,
    US_RIGHT = 2,
    US_MAX_SENSORS = 3
} us_pos_t;

/**
 * @brief Bitmask definitions for checking specific sensors.
 */
#define US_MASK_FRONT  (1 << US_FRONT)
#define US_MASK_LEFT   (1 << US_LEFT)
#define US_MASK_RIGHT  (1 << US_RIGHT)
#define US_MASK_ALL    (0x07)

/**
 * @brief Direction decision returned by the driver.
 */
typedef enum {
    DIR_FORWARD  = 0,
    DIR_LEFT     = 1,
    DIR_RIGHT    = 2,
    DIR_BACKWARD = 3,
    DIR_NO_CHANGE = 4
} us_direction_t;

/**
 * @brief Individual ultrasonic sensor hardware mapping.
 */
typedef struct {
    bool en;
    int trig_port;
    int trig_pin;
    int echo_port;
    int echo_pin;
} us_t;

/**
 * @brief Portable Ultrasonic Driver Structure.
 */
typedef struct us_driver_s {
    us_t sensors[US_MAX_SENSORS];

    float wall_threshold_cm; /* user sets this */

    /**
     * @brief Platform-specific GPIO + timer init for one sensor.
     */
    void  (*init_hal)(int trig_port, int trig_pin,
                      int echo_port, int echo_pin);

    /**
     * @brief Platform-specific distance read in cm.
     * @return Distance in cm as float.
     */
    float (*read_hal)(int trig_port, int trig_pin,
                      int echo_port, int echo_pin);

} us_driver_t;

/* --- Driver API --- */

/**
 * @brief Initializes all enabled sensors.
 */
void us_init(us_driver_t *driver);

/**
 * @brief Reads all enabled sensors and returns raw distances.
 * @param driver  Pointer to driver instance.
 * @param out_cm  Array of size US_MAX_SENSORS, filled with distances in cm.
 */
void us_read_all(us_driver_t *driver, float out_cm[US_MAX_SENSORS]);

/**
 * @brief Packs wall detections into a single byte using wall_threshold_cm.
 * Bit 0: Front
 * Bit 1: Left
 * Bit 2: Right
 * @return uint8_t bitmask of walls detected.
 */
uint8_t us_get_wall_mask(us_driver_t *driver);


void us_init_hal(int trig_port, int trig_pin, int echo_port, int echo_pin);
float us_read_hal(int trig_port, int trig_pin, int echo_port, int echo_pin);
void us_delay_us(uint32_t us);



/**
 * @brief Applies the navigation logic and returns a direction.
 *
 * Logic:
 *  - No wall anywhere         -> DIR_NO_CHANGE
 *  - Wall front only          -> check left/right:
 *      both free              -> randomly pick DIR_LEFT or DIR_RIGHT
 *      one blocked            -> go to the free side
 *  - Wall front + left + right -> DIR_BACKWARD
 */
us_direction_t us_get_direction(us_driver_t *driver);

#endif // ULTRASONIC_SENSOR_H