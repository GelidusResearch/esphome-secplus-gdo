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

  void setup() override {
    float value;
    this->pref_ =
        global_preferences->make_preference<float>(this->get_object_id_hash());
    if (!this->pref_.load(&value)) {
      // Set appropriate default values based on the component type
      std::string obj_id = this->get_object_id();
      if (obj_id.find("min_command_interval") != std::string::npos) {
        value = 250;  // Default 250ms for min command interval
      } else if (obj_id.find("open_duration") != std::string::npos || 
                 obj_id.find("close_duration") != std::string::npos) {
        value = 0.0f;  // Default 0.0s for duration measurements
      } else {
        value = this->traits.get_min_value();
      }
    }

    this->last_saved_value_ = value;  // Initialize last saved value
    this->control(value);
  }

  void update_state(float value) {
    if (value == this->state) {
      return;
    }

    this->state = value;
    this->publish_state(value);
    
    // Only save to persistent storage if the value has changed significantly
    // This reduces flash writes and improves responsiveness
    std::string obj_id = this->get_object_id();
    if (obj_id.find("open_duration") != std::string::npos || 
        obj_id.find("close_duration") != std::string::npos) {
      // For duration measurements, only save if changed by more than 0.1s
      // and debounce saves to reduce flash writes during door movement
      if (fabs(value - last_saved_value_) > 0.1f) {
        this->cancel_timeout("save_duration");
        this->set_timeout("save_duration", 2000, [this, value]() {
          this->pref_.save(&value);
          this->last_saved_value_ = value;
        });
      }
    } else {
      // For other number types, save immediately
      this->pref_.save(&value);
    }
  }

  void control(float value) override {
    if (value == this->state) {
      return;
    }

    if (this->f_control) {
      this->f_control(value);
      this->update_state(value);
      // For user-controlled values, save immediately to ensure persistence
      this->cancel_timeout("save_duration");
      this->pref_.save(&value);
      this->last_saved_value_ = value;
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