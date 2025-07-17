/*
 * Copyright (C) 2024  Konnected Inc.
 * Copyright (C) 2025  Gelidus Research
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

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "inttypes.h"
#include "secplus_gdo.h"
#ifdef TOF_SENSOR
#include "vehicle.h"
#endif

namespace esphome
{
  namespace secplus_gdo
  {

#ifdef TOF_SENSOR
    uint16_t interval_count = 0;
    uint16_t last_distance = 0;
    std::vector<int16_t> distance_measurements(DISTANCE_ARRAY_SIZE, 0);
    VehicleTracker vt(distance_measurements, 1000, 2);
#endif

    static const char *const TAG = "secplus_gdo";

    // Add diagnostic counters to measure event frequency
    static uint32_t door_position_events = 0;
    static uint32_t duration_events = 0;
    static uint32_t last_diagnostic_time = 0;

    static void gdo_event_handler(const gdo_status_t *status, gdo_cb_event_t event, void *arg)
    {
      GDOComponent *gdo = static_cast<GDOComponent *>(arg);

      // Print diagnostic info every 10 seconds
      uint32_t now = millis();
      if (now - last_diagnostic_time > 10000) {
        ESP_LOGI(TAG, "Event frequency: %d door_position, %d duration events in last 10s",
                 door_position_events, duration_events);
        door_position_events = 0;
        duration_events = 0;
        last_diagnostic_time = now;
      }

      switch (event)
      {
      case GDO_CB_EVENT_SYNCED:
        ESP_LOGI(TAG, "Synced: %s, protocol: %s", status->synced ? "true" : "false",
                 gdo_protocol_type_to_string(status->protocol));
        if (status->protocol == GDO_PROTOCOL_SEC_PLUS_V2)
        {
          ESP_LOGI(TAG, "Client ID: %" PRIu32 ", Rolling code: %" PRIu32,
                   status->client_id, status->rolling_code);
          if (status->synced)
          {
            // Save the last successful ClientID rolling code value to NVS for use
            // on reboot
            gdo->set_client_id(status->client_id);
            gdo->set_rolling_code(status->rolling_code);
          }
        }

        if (!status->synced)
        {
          // Limit sync retries to prevent infinite loops
          if (gdo->should_retry_sync())
          {
            gdo->increment_sync_retry();

            // Only increment rolling code if we have a valid starting value
            uint32_t new_rolling_code = status->rolling_code;
            if (new_rolling_code > 0) {
              new_rolling_code += 100;
            } else {
              // If rolling code is 0, try a reasonable starting value
              new_rolling_code = 100;
              ESP_LOGW(TAG, "Rolling code was 0, using initial value: %" PRIu32, new_rolling_code);
            }

            // Use public defer method to avoid blocking the event handler
            // Capture the rolling code and current retry count for the timeout callback
            uint8_t current_retry = gdo->get_sync_retry_count();
            uint8_t max_retries = gdo->get_max_sync_retries();
            gdo->defer_operation("sync_retry", 10, [new_rolling_code, current_retry, max_retries]() {
              if (gdo_set_rolling_code(new_rolling_code) != ESP_OK)
              {
                ESP_LOGE(TAG, "Failed to set rolling code");
              }
              else
              {
                ESP_LOGI(TAG, "Rolling code set to %" PRIu32 ", retrying sync (attempt %d/%d)",
                         new_rolling_code, current_retry, max_retries);

                // Retry sync without blocking delay
                // The GDO library will handle its own timing internally
                gdo_sync();
              }
            });
          }
          else
          {
            ESP_LOGW(TAG, "Max sync retries (%d) reached, stopping sync attempts", gdo->get_max_sync_retries());
            ESP_LOGW(TAG, "Try setting client ID and rolling code manually in Home Assistant");
            gdo->reset_sync_retry(); // Reset for next time
          }
        }
        else
        {
          // Reset retry counter on successful sync
          gdo->reset_sync_retry();
          gdo->set_protocol_state(status->protocol);
        }
        gdo->set_sync_state(status->synced);
        break;
      case GDO_CB_EVENT_LIGHT:
        gdo->set_light_state(status->light);
        break;
      case GDO_CB_EVENT_LOCK:
        gdo->set_lock_state(status->lock);
        break;
      case GDO_CB_EVENT_DOOR_POSITION:
        door_position_events++;  // Count door position events
        ESP_LOGV(TAG, "Door state: %d, position: %d", status->door, status->door_position);  // Changed to LOGV to reduce spam
        // Defer the door position update to avoid blocking the event handler
        gdo->defer_operation("door_position_update", 1, [gdo, status]() {
          float position = (float)(10000 - status->door_position) / 10000.0f;

          if (status->door > GDO_DOOR_STATE_CLOSING &&
              status->door < GDO_DOOR_STATE_OPENING)
          {
            // If the door is not in a known state, we assume it is open
            ESP_LOGW(TAG, "Door state unknown, assuming open");
            gdo->set_door_state(GDO_DOOR_STATE_OPEN, position);
          }
          else if (status->door > GDO_DOOR_STATE_MAX)
          {
            // Invalid door state, default to open
            ESP_LOGW(TAG, "Invalid door state %d, assuming open", status->door);
            gdo->set_door_state(GDO_DOOR_STATE_OPEN, position);
          }
          else
          {
            // Normal case - valid door state
            gdo->set_door_state((gdo_door_state_t)status->door, position);
          }

          if (status->door != GDO_DOOR_STATE_OPENING && status->door != GDO_DOOR_STATE_CLOSING)
          {
            gdo->set_motor_state(GDO_MOTOR_STATE_OFF);
          }
        });
        break;
      case GDO_CB_EVENT_LEARN:
        ESP_LOGI(TAG, "Learn: %s", gdo_learn_state_to_string(status->learn));
        gdo->set_learn_state(status->learn);

        // Auto-off learn mode after 1 minute (60 seconds) to prevent learning undesirable devices
        if (status->learn == GDO_LEARN_STATE_ACTIVE) {
          ESP_LOGI(TAG, "Learn mode will auto-off after 60 seconds");
          gdo->defer_operation("learn_auto_off", 60000, []() {
            ESP_LOGI(TAG, "Learn mode auto-off timeout reached");
            gdo_deactivate_learn();
          });
        } else {
          // Cancel auto-off if learn mode is manually disabled
          gdo->cancel_operation("learn_auto_off");
        }
        break;
      case GDO_CB_EVENT_OBSTRUCTION:
        ESP_LOGI(TAG, "Obstruction: %s",
                 gdo_obstruction_state_to_string(status->obstruction));
        gdo->set_obstruction(status->obstruction);
        break;
      case GDO_CB_EVENT_MOTION:
        ESP_LOGI(TAG, "Motion: %s", gdo_motion_state_to_string(status->motion));
        gdo->set_motion_state(status->motion);
        break;
      case GDO_CB_EVENT_BATTERY:
        ESP_LOGI(TAG, "Battery: %s", gdo_battery_state_to_string(status->battery));
        break;
      case GDO_CB_EVENT_BUTTON:
        ESP_LOGI(TAG, "Button: %s", gdo_button_state_to_string(status->button));
        gdo->set_button_state(status->button);
        break;
      case GDO_CB_EVENT_MOTOR:
        ESP_LOGI(TAG, "Motor: %s", gdo_motor_state_to_string(status->motor));
        gdo->set_motor_state(status->motor);
        break;
      case GDO_CB_EVENT_OPENINGS:
        ESP_LOGI(TAG, "Openings: %d", status->openings);
        gdo->set_openings(status->openings);

        break;
      case GDO_CB_EVENT_SET_TTC:
        ESP_LOGI(TAG, "Set Time to close: %d", status->ttc_seconds);
        break;
      case GDO_CB_EVENT_CANCEL_TTC:
        ESP_LOGI(TAG, "Cancel Time to close: %d", status->ttc_seconds);
        break;
      case GDO_CB_EVENT_UPDATE_TTC:
        ESP_LOGI(TAG, "Update Time to close: %d", status->ttc_seconds);
        if ((status->ttc_seconds == 0) && (status->ttc_enabled)) {
          // Use public defer method to avoid blocking in event handler
          gdo->defer_operation("ttc_close", 10, []() {
            gdo_door_close();
          });
        }
        break;
      case GDO_CB_EVENT_PAIRED_DEVICES:
        ESP_LOGI(TAG,
                 "Paired devices: %d remotes, %d keypads, %d wall controls, %d "
                 "accessories, %d total",
                 status->paired_devices.total_remotes,
                 status->paired_devices.total_keypads,
                 status->paired_devices.total_wall_controls,
                 status->paired_devices.total_accessories,
                 status->paired_devices.total_all);
        break;
      case GDO_CB_EVENT_OPEN_DURATION_MEASUREMENT:
        duration_events++;  // Count duration events
        ESP_LOGV(TAG, "Open duration: %d", status->open_ms);  // Changed to LOGV to reduce spam even more
        // Defer the duration update to avoid blocking the event handler
        gdo->defer_operation("open_duration_update", 5, [gdo, ms = status->open_ms]() {
          gdo->set_open_duration(ms);
        });
        break;
      case GDO_CB_EVENT_CLOSE_DURATION_MEASUREMENT:
        duration_events++;  // Count duration events
        ESP_LOGV(TAG, "Close duration: %d", status->close_ms);  // Changed to LOGV to reduce spam even more
        // Defer the duration update to avoid blocking the event handler
        gdo->defer_operation("close_duration_update", 5, [gdo, ms = status->close_ms]() {
          gdo->set_close_duration(ms);
        });
        break;
#ifdef TOF_SENSOR
      case GDO_CB_EVENT_TOF_TIMER:
      {
        uint16_t distance = vt.get_tof_distance();
        uint16_t threshold = gdo->get_vehicle_parked_threshold();
        uint16_t variance =  gdo->get_vehicle_parked_threshold_variance();
        vt.update_measurements(distance);
        vt.process_vehicle_state(threshold, variance, VEHICLE_SENCE_TIME, gdo);
        if (distance < gdo->last_distance - 1 || distance > gdo->last_distance + 1)
        {
          gdo->set_tof_distance(distance);
        }
        gdo->last_distance = distance;
      }
      break;
#endif
      default:
        ESP_LOGI(TAG, "Unknown event: %d", event);
        break;
      }
    }

    void GDOComponent::setup()
    {
#ifdef GDO_TOGGLE_ONLY
      // Set the toggle only state and control here because we cannot guarantee the
      // cover instance was created before the switch
      this->door_->set_toggle_only(this->toggle_only_switch_->state);
      this->toggle_only_switch_->set_control_function(
          std::bind(&esphome::secplus_gdo::GDODoor::set_toggle_only, this->door_,
                    std::placeholders::_1));
#endif
#ifdef GDO_OBST_OVERRIDE
      // Set the obstruction override state and control
      this->obst_override_switch_->set_control_function(
          [this](bool state) {
            ESP_LOGI("GDOComponent", "Obstruction override %s", state ? "enabled" : "disabled");
            gdo_set_obst_override(state);
          });
#endif
      gdo_config_t gdo_conf = {
          .uart_num = UART_NUM_1,
          .obst_from_status = GDO_OBST_FROM_STATE,
          .invert_uart = true,
          .uart_tx_pin = (gpio_num_t)GDO_UART_TX_PIN,
          .uart_rx_pin = (gpio_num_t)GDO_UART_RX_PIN,
#ifdef GDO_OBST_INPUT_PIN
          .obst_in_pin = (gpio_num_t)GDO_OBST_INPUT_PIN,
#else
          .obst_in_pin = (gpio_num_t)-1,
#endif
#ifdef GDO_RF_TX_PIN
          .rf_tx_pin = (gpio_num_t)GDO_RF_TX_PIN,
#else
          .rf_tx_pin = (gpio_num_t)-1,
#endif
#ifdef GDO_RF_RX_PIN
          .rf_rx_pin = (gpio_num_t)GDO_RF_RX_PIN,
#else
          .rf_rx_pin = (gpio_num_t)-1,
#endif
      };
#ifdef TOF_SENSOR
      gdo_set_tof_timer(100000, true);
#endif
      gdo_init(&gdo_conf);
      gdo_get_status(&this->status_);

      // Initialize client ID and rolling code from stored values (if available)
      // This is essential for first-run and after reboot functionality
      if (this->client_id_) {
        uint32_t client_id_to_use = 0;
        if (this->client_id_->has_state()) {
          client_id_to_use = (uint32_t)this->client_id_->state;
          ESP_LOGI(TAG, "Loading stored client ID: %" PRIu32, client_id_to_use);
        } else {
          // No stored state, use the component's initial value
          client_id_to_use = (uint32_t)this->client_id_->traits.get_min_value();
          ESP_LOGI(TAG, "No stored client ID, using initial value: %" PRIu32, client_id_to_use);
        }

        // If client ID is still 0, it's invalid - use a reasonable default
        if (client_id_to_use == 0) {
          client_id_to_use = 1000;  // Default client ID
          ESP_LOGW(TAG, "Client ID was 0, using default: %" PRIu32, client_id_to_use);
        }

        gdo_set_client_id(client_id_to_use);
      }

      if (this->rolling_code_) {
        uint32_t rolling_code_to_use = 0;
        if (this->rolling_code_->has_state()) {
          rolling_code_to_use = (uint32_t)this->rolling_code_->state;
          ESP_LOGI(TAG, "Loading stored rolling code: %" PRIu32, rolling_code_to_use);
        } else {
          // No stored state, use the component's initial value
          rolling_code_to_use = (uint32_t)this->rolling_code_->traits.get_min_value();
          ESP_LOGI(TAG, "No stored rolling code, using initial value: %" PRIu32, rolling_code_to_use);
        }

        // If rolling code is still 0, use a reasonable default
        if (rolling_code_to_use == 0) {
          rolling_code_to_use = 1000;  // Default rolling code
          ESP_LOGW(TAG, "Rolling code was 0, using default: %" PRIu32, rolling_code_to_use);
        }

        gdo_set_rolling_code(rolling_code_to_use);
      }

#ifdef TOF_SENSOR
      vt.setup_tof_sensor(gpio_num_t(GDO_TOF_SDA_PIN), gpio_num_t(GDO_TOF_SCL_PIN), 400000);
      delay(100);
#endif

      if (this->start_gdo_)
      {
        gdo_start(gdo_event_handler, this);
        ESP_LOGI(TAG, "secplus GDO started!");
      }
      else
      {
        // check every 500ms for readiness before starting GDO
        this->set_interval("gdo_start", 500, [this]()
                           {
      if (this->start_gdo_) {
        gdo_start(gdo_event_handler, this);
        ESP_LOGI(TAG, "secplus GDO started!");
        this->cancel_interval("gdo_start");
      } });
      }
    }

    void GDOComponent::set_sync_state(bool synced)
    {
      if (this->door_)
      {
        this->door_->set_sync_state(synced);
      }

      if (this->light_)
      {
        this->light_->set_sync_state(synced);
      }

      if (this->lock_)
      {
        this->lock_->set_sync_state(synced);
      }

      if (this->f_sync)
      {
        this->f_sync(synced);
      }
    }

     void GDOComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Setting up secplus GDO ...");
    }

  } // namespace secplus_gdo
} // namespace esphome

// Need to wrap the panic handler to disable the GDO TX pin and pull the output
// high to prevent spuriously triggering the GDO to open when the ESP32 panics.
extern "C"
{
#include "esp_rom_gpio.h"
#include "soc/gpio_reg.h"

  void __real_esp_panic_handler(void *);
  void __wrap_esp_panic_handler(void *info)
  {
    esp_rom_printf("PANIC: DISABLING GDO UART TX PIN!\n");

    // Disable the UART TX pin and set it as input
    // This prevents spurious signals during panic that could trigger the garage door
    esp_rom_gpio_pad_select_gpio(GDO_UART_TX_PIN);
    esp_rom_gpio_pad_set_drv(GDO_UART_TX_PIN, 0);

    // Disable output (set as input)
    REG_WRITE(GPIO_ENABLE_W1TC_REG, (1 << GDO_UART_TX_PIN));

    // Set pin low using output register (even though it's input, this sets the internal state)
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << GDO_UART_TX_PIN));

    // Call the original panic handler
    __real_esp_panic_handler(info);
  }
} // extern "C"
