#ifndef _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_
#define _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_

/*
 * Only fetch the 2 strongest targets per zone across the SPI bus.
 * (Reduces DMA/SPI read payload size significantly compared to default 4).
 */
#define VL53L8CX_NB_TARGET_PER_ZONE 2U

/*
 * Disable unneeded sensor metrics from the sensor MCU output stream.
 * Keeping:
 *  - Distance (distance_mm)
 *  - Target status (target_status)
 *  - Number of targets detected (nb_target_detected, prevents stale zone/target latching)
 *  - Active SPAD count (nb_spads_enabled, needed for 1024 SPAD contact fallback)
 */
#define VL53L8CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L8CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L8CX_DISABLE_RANGE_SIGMA_MM
#define VL53L8CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L8CX_DISABLE_MOTION_INDICATOR

#endif // _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_

