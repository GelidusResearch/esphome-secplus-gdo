/*
 * Copyright (C) 2024  Konnected Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include "include/gdo.h"
#include "cover/gdo_door.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef TOF_SENSOR
extern "C" {
#include "VL53L1X_api.h"
}
#include "vehicle.h"
#endif
#include "light/gdo_light.h"
#include "lock/gdo_lock.h"
#include "number/gdo_number.h"
#include "esphome/core/defines.h"
#include "select/gdo_select.h"
#include "switch/gdo_switch.h"

namespace esphome {
namespace secplus_gdo {

class GDOComponent : public Component {
public:

  void setup() override;
  void loop() override{};
  void dump_config() override;
  void on_shutdown() override { gdo_deinit(); }
  void start_gdo() { start_gdo_ = true; }
  // Use Late priority so we do not start the GDO lib until all saved
  // preferences are loaded
  float get_setup_priority() const override { return setup_priority::LATE; }

  void register_protocol_select(GDOSelect *select) {
    this->protocol_select_ = select;
  }

  void set_protocol_state(gdo_protocol_type_t protocol) {
    if (this->protocol_select_) {
      this->protocol_select_->update_state(protocol);
    }
  }

  void register_motion(std::function<void(bool)> f) { f_motion = f; }
  void set_motion_state(gdo_motion_state_t state) {
    if (f_motion) {
      f_motion(state == GDO_MOTION_STATE_DETECTED);
    }
  }

  void register_obstruction(std::function<void(bool)> f) { f_obstruction = f; }
  void set_obstruction(gdo_obstruction_state_t state) {
    if (f_obstruction) {
      f_obstruction(state == GDO_OBSTRUCTION_STATE_OBSTRUCTED);
    }
  }

  void register_button(std::function<void(bool)> f) { f_button = f; }
  void set_button_state(gdo_button_state_t state) {
    if (f_button) {
      f_button(state == GDO_BUTTON_STATE_PRESSED);
    }
  }

  void register_motor(std::function<void(bool)> f) { f_motor = f; }
  void set_motor_state(gdo_motor_state_t state) {
    if (f_motor) {
      f_motor(state == GDO_MOTOR_STATE_ON);
    }
  }

  void register_sync(std::function<void(bool)> f) { f_sync = f; }

  void register_openings(std::function<void(uint16_t)> f) { f_openings = f; }
  void set_openings(uint16_t openings) {
    if (f_openings) {
      f_openings(openings);
    }
  }
#ifdef TOF_SENSOR
  uint16_t last_distance = 0;
  void register_tof_distance(std::function<void(uint16_t)> f) { f_tof_distance = f; }
  void set_tof_distance(uint16_t tof_distance) {
    if (f_tof_distance) {
      f_tof_distance(tof_distance);
    }
  }
  void register_vehicle_parked(std::function<void(bool)> f) { f_vehicle_parked = f; }
  void register_vehicle_arriving(std::function<void(bool)> f) { f_vehicle_arriving = f; }
  void register_vehicle_leaving(std::function<void(bool)> f) { f_vehicle_leaving = f; }

  void set_vehicle_parked(bool state) {
      if (f_vehicle_parked) {
          f_vehicle_parked(state);
      }
  }
  void set_vehicle_arriving(bool state) {
      if (f_vehicle_arriving) {
          f_vehicle_arriving(state);
      }
  }
  void set_vehicle_leaving(bool state) {
      if (f_vehicle_leaving) {
          f_vehicle_leaving(state);
      }
  }

  void register_vehicle_parked_threshold(GDONumber *num) { this->vehicle_parked_threshold_ = num; }

  uint16_t get_vehicle_parked_threshold() {
    return this->vehicle_parked_threshold_->state;
  }

  void register_vehicle_parked_threshold_variance(GDONumber *num) { this->vehicle_parked_threshold_variance_ = num; }

  uint16_t get_vehicle_parked_threshold_variance() {
    if (this->vehicle_parked_threshold_variance_ == nullptr) {
      return 5;
    } else {
      return this->vehicle_parked_threshold_variance_->state;
    }
  }
#endif


  void register_door(GDODoor *door) { this->door_ = door; }
  void set_door_state(gdo_door_state_t state, float position) {
    if (this->door_) {
      this->door_->set_state(state, position);
    }
  }

  void register_light(GDOLight *light) { this->light_ = light; }
  void set_light_state(gdo_light_state_t state) {
    if (this->light_) {
      this->light_->set_state(state);
    }
  }

  void register_lock(GDOLock *lock) { this->lock_ = lock; }
  void set_lock_state(gdo_lock_state_t state) {
    if (this->lock_) {
      this->lock_->set_state(state);
    }
  }

  void register_learn(GDOSwitch *sw) { this->learn_switch_ = sw; }
  void set_learn_state(gdo_learn_state_t state) {
    if (this->learn_switch_) {
      this->learn_switch_->write_state(state == GDO_LEARN_STATE_ACTIVE);
    }
  }

  void register_open_duration(GDONumber *num) { open_duration_ = num; }
  void set_open_duration(uint16_t ms) {
    if (open_duration_) {
      // Convert milliseconds to seconds for display with 0.1s precision
      float seconds = ms / 1000.0f;
      open_duration_->update_state(seconds);
    }
  }

  void register_close_duration(GDONumber *num) { close_duration_ = num; }
  void set_close_duration(uint16_t ms) {
    if (close_duration_) {
      // Convert milliseconds to seconds for display with 0.1s precision
      float seconds = ms / 1000.0f;
      close_duration_->update_state(seconds);
    }
  }

  void register_client_id(GDONumber *num) { client_id_ = num; }
  void set_client_id(uint32_t num) {
    if (client_id_) {
      client_id_->update_state(num);
    }
  }

  void register_rolling_code(GDONumber *num) { rolling_code_ = num; }
  void set_rolling_code(uint32_t num) {
    if (rolling_code_) {
      rolling_code_->update_state(num);
    }
  }

  uint32_t get_rolling_code() const {
    if (rolling_code_ && rolling_code_->has_state()) {
      return (uint32_t)rolling_code_->state;
    }
    return 0;
  }

  void register_min_command_interval(GDONumber *num) { min_command_interval_ = num; }
  void set_min_command_interval(uint32_t num) {
    if (min_command_interval_) {
      min_command_interval_->update_state(num);
    }
  }

  void register_time_to_close(GDONumber *num) { time_to_close_ = num; }
  void set_time_to_close(uint16_t num) {
    if (time_to_close_) {
      time_to_close_->update_state(num);
    }
  }

  void register_toggle_only(GDOSwitch *sw) { this->toggle_only_switch_ = sw; }
  void register_obst_override(GDOSwitch *sw) { this->obst_override_switch_ = sw; }
  void set_sync_state(bool synced);

  // Public method to defer operations to avoid blocking in event handlers
  void defer_operation(const std::string &name, uint32_t delay, std::function<void()> &&f) {
    this->set_timeout(name, delay, std::move(f));
  }

  // Public method to cancel timeouts (wraps protected cancel_timeout)
  void cancel_operation(const std::string &name) {
    this->cancel_timeout(name);
  }

  // Public methods for sync retry management (needed by static event handler)
  bool should_retry_sync() {
    return sync_retry_count_ < MAX_SYNC_RETRIES;
  }

  void increment_sync_retry() {
    sync_retry_count_++;
  }

  void reset_sync_retry() {
    sync_retry_count_ = 0;
  }

  uint8_t get_sync_retry_count() const {
    return sync_retry_count_;
  }

  uint8_t get_max_sync_retries() const {
    return MAX_SYNC_RETRIES;
  }

  // Public methods for restart-after-sync-failure management
  bool has_restarted_for_sync() const {
    return has_restarted_for_sync_;
  }

  void set_has_restarted_for_sync(bool value) {
    has_restarted_for_sync_ = value;
  }

  void save_restart_flag() {
    restart_attempt_pref_.save(&has_restarted_for_sync_);
  }

protected:
  gdo_status_t status_{};
  std::function<void(gdo_lock_state_t)> f_lock{nullptr};
  std::function<void(gdo_light_state_t)> f_light{nullptr};
  std::function<void(uint16_t)> f_openings{nullptr};
  std::function<void(bool)> f_motion{nullptr};
  std::function<void(bool)> f_obstruction{nullptr};
  std::function<void(bool)> f_button{nullptr};
  std::function<void(bool)> f_motor{nullptr};
  std::function<void(bool)> f_sync{nullptr};

  GDODoor *door_{nullptr};
  GDOLight *light_{nullptr};
  GDOLock *lock_{nullptr};
  GDONumber *open_duration_{nullptr};
  GDONumber *close_duration_{nullptr};
  GDONumber *client_id_{nullptr};
  GDONumber *rolling_code_{nullptr};
  GDONumber *min_command_interval_{nullptr};
  GDONumber *time_to_close_{nullptr};

#ifdef TOF_SENSOR
  GDONumber *target_tof_distance_{nullptr};
  GDONumber *vehicle_parked_threshold_{nullptr};
  GDONumber *vehicle_parked_threshold_variance_{nullptr};
  std::function<void(uint16_t)> f_tof_distance{nullptr};
  std::function<void(bool)> f_vehicle_parked{nullptr};
  std::function<void(bool)> f_vehicle_leaving{nullptr};
  std::function<void(bool)> f_vehicle_arriving{nullptr};
#endif
  GDOSelect *protocol_select_{nullptr};
  GDOSwitch *learn_switch_{nullptr};
  GDOSwitch *toggle_only_switch_{nullptr};
  GDOSwitch *obst_override_switch_{nullptr};
  bool start_gdo_{false};
  bool gdo_started_{false};

  // Sync retry tracking
  uint8_t sync_retry_count_{0};
  static const uint8_t MAX_SYNC_RETRIES = 5;

  // Track if we've already restarted once after sync failure
  // Stored in RTC memory so it persists across warm restarts but not power cycles
  ESPPreferenceObject restart_attempt_pref_;
  bool has_restarted_for_sync_{false};

}; // GDOComponent
} // namespace secplus_gdo
} // namespace esphome
