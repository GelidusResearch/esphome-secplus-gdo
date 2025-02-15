#ifdef TOF_SENSOR

#ifndef VEHICLE_TRACKER_H
#define VEHICLE_TRACKER_H

#pragma once
#include "esp_log.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "include/gdo.h"
#include <algorithm>
#include <cmath>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <numeric>
#include <vector>


extern "C" {
#include "VL53L1X_api.h"
}

#define MEASUREMENT_CYCLE_MS 100 // Measurement cycle in ms for ToF sensor
#define VEHICLE_SENCE_TIME 15    // Vehicle state sencing transition time in seconds
#define DISTANCE_ARRAY_SIZE 8    // Used for averaging distance measurements

typedef enum {
  UNKNOWN,
  PARKED,
  ARRIVING,
  LEAVING,
  AWAY,
  DETECTED,
} vehicle_state_t;

typedef struct {
  bool presence_window;
  uint16_t measurement_time_window;
  vehicle_state_t vehicle_state;
  gdo_door_state_t current_door_state;
  gdo_door_state_t previous_door_state;
  uint16_t current_distance_measurement;
  uint16_t parked_distance_threshold;
  uint32_t last_state_change_time;
} vehicle_status_t;

namespace esphome {
namespace secplus_gdo {

class GDOComponent; // Forward declaration

class VehicleTracker {
private:
  std::vector<int16_t> &distance_measurements; // Reference to circular buffer for distance readings
  size_t current_index;      // Current index for inserting a new reading
  int64_t last_state_change; // Timestamp in microseconds
  void update_state(vehicle_state_t new_state);
  float vehicle_parked_threshold_{100}; // Provide a default value

public:
  VehicleTracker(std::vector<int16_t> &measurements, uint16_t parked_threshold, uint16_t time_window);
  vehicle_status_t vp_status; // Vehicle presence status
  void process_vehicle_state(uint16_t parked_threshold, uint16_t time_window, GDOComponent *gdo);
  int setup_tof_sensor(gpio_num_t sda, gpio_num_t scl, int frequency);
  uint16_t get_tof_distance();
  vehicle_state_t get_state() const;
  int16_t get_average_distance() const;
  uint64_t time_since_last_state_change() const;
  void update_measurements(uint16_t new_reading);



};

} // namespace secplus_gdo
} // namespace esphome

#endif // VEHICLE_TRACKER_H
#endif // TOF_SENSOR
