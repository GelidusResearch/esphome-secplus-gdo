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

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include <cmath>


namespace esphome {
namespace secplus_gdo {

class GDONumber : public number::Number, public Component {
public:
  void dump_config() override {}

  float get_setup_priority() const override {
    // Run before main component (which uses LATE = -100.0) so NVS values are loaded first
    return setup_priority::DATA - 1.0f;  // DATA is -200.0, so this is -201.0
  }

  // Normalize client_id to Security+ protocol format: 0xXXX539
  // The last 12 bits must be 0x539 (1337 in decimal)
  float normalize_client_id(float client_id) {
    uint32_t int_value = static_cast<uint32_t>(client_id);
    if ((int_value & 0xFFF) != 0x539) {
      client_id = ceil((client_id - 0x539) / 0x1000) * 0x1000 + 0x539;
    }
    return client_id;
  }

  void setup() override {
    float value;
    std::string obj_id = this->get_object_id();
    bool is_duration = (obj_id.find("open_duration") != std::string::npos ||
                       obj_id.find("close_duration") != std::string::npos);

    // Load value from preferences
    this->pref_ =
        global_preferences->make_preference<float>(this->get_object_id_hash());
    bool loaded_from_nvs = this->pref_.load(&value);

    if (!loaded_from_nvs) {
      // Set appropriate default values based on the component type
      if (obj_id.find("min_command_interval") != std::string::npos) {
        value = 250;  // Default 250ms for min command interval
        ESP_LOGI("GDONumber", "No stored min_command_interval, using default: %.1f", value);
      } else if (obj_id.find("rolling_code") != std::string::npos) {
        value = 0;  // Default rolling code for Security+ V2 protocol compliance
        ESP_LOGW("GDONumber", "NVS EMPTY: No stored rolling_code, using default: %.0f", value);
      } else if (obj_id.find("client_id") != std::string::npos) {
        value = 1638;  // Default client ID for Security+ V2 (0x666)
        value = normalize_client_id(value);  // Normalize to 0xXXX539 format
        ESP_LOGW("GDONumber", "NVS EMPTY: No stored client_id, using normalized default: %.0f (0x%X)", value, (uint32_t)value);
      } else if (is_duration) {
        // For duration measurements, don't set an initial state if no measurement exists
        ESP_LOGI("GDONumber", "No stored duration for %s, will remain unknown until measured", obj_id.c_str());
        this->last_saved_value_ = 0.0f;  // Initialize with 0 for tracking
        return;  // Don't set state or publish - let it remain unknown
      } else {
        value = this->traits.get_min_value();
        ESP_LOGI("GDONumber", "No stored value for %s, using min_value: %.1f", obj_id.c_str(), value);
      }
    } else {
      // Successfully loaded from NVS - use INFO for credentials, DEBUG for others
      if (obj_id.find("client_id") != std::string::npos) {
        // Normalize client_id if loaded from NVS
        float original_value = value;
        value = normalize_client_id(value);
        if (value != original_value) {
          ESP_LOGW("GDONumber", "NVS LOADED client_id %.0f (0x%X) normalized to %.0f (0x%X)",
                   original_value, (uint32_t)original_value, value, (uint32_t)value);
        } else {
          ESP_LOGI("GDONumber", "NVS LOADED: client_id = %.0f (0x%X)", value, (uint32_t)value);
        }
      } else if (obj_id.find("rolling_code") != std::string::npos) {
        ESP_LOGI("GDONumber", "NVS LOADED: rolling_code = %.0f", value);
      } else {
        ESP_LOGI("GDONumber", "Loaded stored value for %s: %.1f", obj_id.c_str(), value);
      }
    }

    // Track whether value was loaded from NVS or is a default
    // If loaded from NVS, set last_saved_value to match
    // If default, set to NAN to force first save
    if (loaded_from_nvs) {
      this->last_saved_value_ = value;  // Was loaded from NVS
    } else {
      this->last_saved_value_ = NAN;  // Was default, needs first save
    }

    // Initialize state properly for all number types
    // For duration measurements, only set state if we have a valid stored value
    this->state = value;
    this->publish_state(value);
  }

  void update_state(float value) {
    bool value_changed = (value != this->state);

    if (value_changed) {
      this->state = value;
      this->publish_state(value);
    }

    // Only save to persistent storage if the value has changed significantly
    // OR if it hasn't been saved yet (last_saved_value_ is NAN means first save needed)
    // This reduces flash writes and improves responsiveness
    std::string obj_id = this->get_object_id();
    bool needs_save = value_changed || std::isnan(last_saved_value_) || (fabs(value - last_saved_value_) > 0.01f);

    if (!needs_save) {
      return;  // Value unchanged and already saved, skip
    }

    if (obj_id.find("open_duration") != std::string::npos ||
        obj_id.find("close_duration") != std::string::npos) {
      // For duration measurements, only save if changed by more than 0.1s
      // and debounce saves to reduce flash writes during door movement
      if (fabs(value - last_saved_value_) > 0.1f) {
        this->cancel_timeout("save_duration");
        this->set_timeout("save_duration", 2000, [this, value]() {
          if (!this->pref_.save(&value)) {
            ESP_LOGW("GDONumber", "Failed to save duration value (hash: %u): %.1f", this->get_object_id_hash(), value);
          } else {
            this->last_saved_value_ = value;
            ESP_LOGD("GDONumber", "Successfully saved duration value: %.1f", value);
          }
        });
      }
    } else {
      // For other number types, save immediately with error handling
      if (!this->pref_.save(&value)) {
        ESP_LOGW("GDONumber", "Failed to save value for %s (hash: %u): %.1f", obj_id.c_str(), this->get_object_id_hash(), value);
      } else {
        // Use INFO level for critical credentials (client_id, rolling_code) for visibility
        if (obj_id.find("client_id") != std::string::npos || obj_id.find("rolling_code") != std::string::npos) {
          ESP_LOGI("GDONumber", "NVS SAVED: %s = %.0f", obj_id.c_str(), value);
        } else {
          ESP_LOGD("GDONumber", "Successfully saved value for %s: %.1f", obj_id.c_str(), value);
        }
      }
    }
  }

  void control(float value) override {
    std::string obj_id = this->get_object_id();

    // Normalize client_id if being set by user
    if (obj_id.find("client_id") != std::string::npos) {
      float original_value = value;
      value = normalize_client_id(value);
      if (value != original_value) {
        ESP_LOGW("GDONumber", "User input client_id %.0f (0x%X) normalized to %.0f (0x%X)",
                 original_value, (uint32_t)original_value, value, (uint32_t)value);
      }
    }

    if (value == this->state) {
      return;
    }

    if (this->f_control) {
      this->f_control(value);
      this->update_state(value);
      // For user-controlled values, save immediately to ensure persistence
      this->cancel_timeout("save_duration");
      if (!this->pref_.save(&value)) {
        ESP_LOGW("GDONumber", "Failed to save user-controlled value: %.1f", value);
      } else {
        this->last_saved_value_ = value;
      }
    }
  }

  void set_control_function(std::function<void(float)> f) { f_control = f; }

 protected:
  ESPPreferenceObject pref_;
  std::function<void(float)> f_control{nullptr};
  float last_saved_value_{0.0f};  // Track last saved value to reduce flash writes
};
} // namespace secplus_gdo
} // namespace esphome
