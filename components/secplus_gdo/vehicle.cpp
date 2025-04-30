#ifdef TOF_SENSOR
#include "esp_timer.h"
#include "vehicle.h"
#include "secplus_gdo.h"

namespace esphome {
namespace secplus_gdo {

static const char *const TAG = "vehicle_tracker";

static vehicle_status_t vp_status = {
    .presence_window = false,
    .measurement_time_window = 2,
    .vehicle_state = UNKNOWN,
    .current_door_state = GDO_DOOR_STATE_UNKNOWN,
    .previous_door_state = GDO_DOOR_STATE_UNKNOWN,
    .current_distance_measurement = 0,
    .parked_distance_threshold = 0,
    .parked_distance_threshold_variance = 5,
    .last_state_change_time = 0,
};


VehicleTracker::VehicleTracker(std::vector<int16_t> &measurements, uint16_t parked_threshold, uint16_t time_window)
    : distance_measurements(measurements) {}

// Update the vehicle state
void VehicleTracker::update_state(vehicle_state_t new_state) {
  if (vp_status.vehicle_state != new_state) {
    vp_status.vehicle_state = new_state;
    vp_status.last_state_change_time = esp_timer_get_time();
  }
}

// Get the current state
vehicle_state_t VehicleTracker::get_state() const {
  return vp_status.vehicle_state;
}

// Get time since last state change
uint64_t VehicleTracker::time_since_last_state_change() const {
  int64_t now = esp_timer_get_time();
  return (now - vp_status.last_state_change_time) / 1000000.0; // Convert microseconds to seconds
}

void VehicleTracker::update_measurements(uint16_t new_reading) {
    distance_measurements.push_back(new_reading);
    if (distance_measurements.size() > DISTANCE_ARRAY_SIZE) {
        distance_measurements.erase(distance_measurements.begin());
    }
}

/**
 * @brief Process the Vehicle state.
 * @param measurements Vector of distance measurements.
 * @param parked_threshold Distance threshold for parked vehicle.
 * @param time_window Time window for state change.
 */
void VehicleTracker::process_vehicle_state(uint16_t parked_threshold, uint16_t variance, uint16_t time_window, GDOComponent *gdo) {
  int64_t now = esp_timer_get_time();
  if ((now - time_window * 1000000 ) > vp_status.last_state_change_time) {
    vp_status.last_state_change_time = now;
  } else {
    return;
  }
  bool within_threshold = false;
  for (const auto &distance : distance_measurements) {
    if (std::abs(distance - parked_threshold) <= variance) {
      within_threshold = true;
      break;
    }
  }

  switch (vp_status.vehicle_state) {
    case UNKNOWN:
      if (within_threshold) {
        update_state(PARKED);
        gdo->set_vehicle_parked(true);
        ESP_LOGI(TAG, "PARKED");
      } else {
        update_state(AWAY);
        gdo->set_vehicle_parked(false);
        ESP_LOGI(TAG, "AWAY");
      }
      break;
    case PARKED:
      if (!within_threshold) {
        update_state(LEAVING);
        ESP_LOGI(TAG, "LEAVING");
        gdo->set_vehicle_parked(false);
        gdo->set_vehicle_leaving(true);
      }
      break;
    case AWAY:
      if (within_threshold) {
        update_state(ARRIVING);
        ESP_LOGI(TAG, "ARRIVING");
        gdo->set_vehicle_arriving(true);
        gdo->set_vehicle_leaving(false);
      }
      break;
    case ARRIVING:
      if (within_threshold) {
        update_state(PARKED);
        gdo->set_vehicle_arriving(false);
        gdo->set_vehicle_parked(true);
        ESP_LOGI(TAG, "PARKED");
      } else if (!within_threshold) {
        update_state(LEAVING);
        gdo->set_vehicle_arriving(false);
        gdo->set_vehicle_leaving(true);
        ESP_LOGI(TAG, "LEAVING");
      }
      break;
    case LEAVING:
      if (!within_threshold) {
        update_state(AWAY);
        gdo->set_vehicle_leaving(false);
        gdo->set_vehicle_parked(false);
      }
      break;
    default:
      update_state(UNKNOWN);
      break;
  }
}

int16_t VehicleTracker::get_average_distance() const {
  if (distance_measurements.empty()) {
    return -1;
  }
  int32_t sum = std::accumulate(distance_measurements.begin(), distance_measurements.end(), 0);
  return sum / distance_measurements.size();
}

/**
 * @brief Inits the Vehicle ToF hardware sensor.
 * @return 0 on success, error code otherwise.
 */

int VehicleTracker::setup_tof_sensor(gpio_num_t sda, gpio_num_t scl,
                                     int frequency) {
  uint8_t model_id, module_type, sensorState = 0;
  VL53L1X_ERROR status = 0;
  uint16_t TOF = VL53L1_I2C_ADDRESS;
  i2c_init_config(I2C_NUM_0, sda, scl, frequency);
  ESP_LOGI(TAG, "I2C Init Started ...");

  // startup the I2C interface and check for the ToF sensor
  i2c_init();

  uint8_t countdown = 10;
  while (sensorState == 0) {
    status = VL53L1X_BootState(TOF, &sensorState);
    vTaskDelay(25 / portTICK_PERIOD_MS);
    if (--countdown == 0) {
      ESP_LOGE(TAG, "VL53L1X Boot State Error: %d\n", status);
      return (int)status;
    }
  }

  // check the VL53L1 device and wait for it to boot
  status = VL53L1_RdByte(TOF, 0x010F, &model_id);
  ESP_LOGI(TAG, "VL53L1X Model_ID: %X, status = %d", model_id, status);
  status = VL53L1_RdByte(TOF, 0x0110, &module_type);
  ESP_LOGI(TAG, "VL53L1X Model_Type: %X, status = %d", module_type, status);
  VL53L1X_SensorInit(TOF); // initialize the ToF sensor
  VL53L1X_SetDistanceMode(
      TOF, DISTANCE_MODE_LONG); // 1=short (up to 1 M), 2=long (up to 4 M)
  VL53L1X_SetTimingBudgetInMs(
      TOF,
      MEASUREMENT_CYCLE_MS); // in ms possible values [20, 50, 100, 200, 500]
  VL53L1X_SetInterMeasurementInMs(
      TOF, 5 + MEASUREMENT_CYCLE_MS); // in ms, IM must be > = TB
  // VL53L1X_SetROI(TOF, 16, 16);
  // VL53L1X_SetSignalThreshold(TOF, 256);
  VL53L1X_SetDistanceThreshold(TOF, 100, 4000, 3, 1);
  VL53L1X_StartRanging(
      TOF); // need to start the VL53L1 with a first request for measurement
  ESP_LOGI(TAG, "I2C ToF Device Ranging started ...");
  return 0;
}

/**
 * @brief Reads the current ToF measurement.
 * @return Distance in cm.
 */

uint16_t VehicleTracker::get_tof_distance() {
  uint16_t distance = 0;
  uint8_t status = 0;
  VL53L1X_ERROR err = VL53L1X_CheckForDataReady(VL53L1_I2C_ADDRESS, &status);
  if (err == 0 && status == 1) {
    err = VL53L1X_GetDistance(VL53L1_I2C_ADDRESS, &distance);
    if (err == 0) {
      return abs(distance / 10);
    } else {
      ESP_LOGE(TAG, "VL53L1X Error: %d\n", err);
    }
  }
  return 0;
}

} // namespace secplus_gdo
} // namespace esphome
#endif