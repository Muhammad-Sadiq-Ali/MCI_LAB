#include "ultrasonic_sensor.h"
#include <stdlib.h> /* for rand() */

void us_init(us_driver_t *driver)
{
    for (int i = 0; i < US_MAX_SENSORS; i++) {
        if (driver->sensors[i].en) {
            driver->init_hal(
                driver->sensors[i].trig_port,
                driver->sensors[i].trig_pin,
                driver->sensors[i].echo_port,
                driver->sensors[i].echo_pin
            );
        }
    }
}

void us_read_all(us_driver_t *driver, float out_cm[US_MAX_SENSORS])
{
    for (int i = 0; i < US_MAX_SENSORS; i++) {
        if (driver->sensors[i].en) {
            out_cm[i] = driver->read_hal(
                driver->sensors[i].trig_port,
                driver->sensors[i].trig_pin,
                driver->sensors[i].echo_port,
                driver->sensors[i].echo_pin
            );
        } else {
            out_cm[i] = -1.0f; /* sensor not enabled */
        }
    }
}

uint8_t us_get_wall_mask(us_driver_t *driver)
{
    float distances[US_MAX_SENSORS];
    us_read_all(driver, distances);

    uint8_t mask = 0;
    for (int i = 0; i < US_MAX_SENSORS; i++) {
        if (driver->sensors[i].en &&
            distances[i] >= 0.0f &&
            distances[i] <= driver->wall_threshold_cm) {
            mask |= (1 << i);
        }
    }
    return mask;
}

us_direction_t us_get_direction(us_driver_t *driver)
{
    uint8_t mask = us_get_wall_mask(driver);

    bool wall_front = (mask & US_MASK_FRONT) != 0;
    bool wall_left  = (mask & US_MASK_LEFT)  != 0;
    bool wall_right = (mask & US_MASK_RIGHT) != 0;

    /* no wall anywhere */
    if (!wall_front && !wall_left && !wall_right)
        return DIR_NO_CHANGE;

    /* wall only on sides, front is clear */
    if (!wall_front)
        return DIR_FORWARD;

    /* wall in front, check sides */
    if (!wall_left && !wall_right)
        return (rand() % 2) ? DIR_LEFT : DIR_RIGHT;

    if (!wall_left)
        return DIR_LEFT;

    if (!wall_right)
        return DIR_RIGHT;

    /* all 3 blocked */
    return DIR_BACKWARD;
}