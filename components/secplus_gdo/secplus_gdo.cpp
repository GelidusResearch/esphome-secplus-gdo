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
#include "driver/gpio.h"
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
      // Add safety checks to prevent panics
      if (!status || !arg) {
        ESP_LOGE(TAG, "Invalid parameters in event handler: status=%p, arg=%p", status, arg);
        return;
      }

      GDOComponent *gdo = static_cast<GDOComponent *>(arg);

      // Additional safety check for gdo pointer validity
      if (!gdo) {
        ESP_LOGE(TAG, "Invalid GDO component pointer");
        return;
      }

      if (status->protocol == GDO_PROTOCOL_SEC_PLUS_V2) {
        static uint32_t last_rc = 0;
        uint32_t current_rc = gdo->get_rolling_code();

        if (status->rolling_code != last_rc && status->rolling_code > current_rc) {
          ESP_LOGV(TAG, "Rolling code: %" PRIu32 " → %" PRIu32, current_rc, status->rolling_code);
          gdo->set_rolling_code(status->rolling_code);
          last_rc = status->rolling_code;
        }
      }

      // Wrap everything in a try-catch to prevent crashes
      // Note: Exception handling disabled in ESP-IDF, using safety checks instead

      uint32_t now = millis();
      if (now - last_diagnostic_time > 10000) {
        door_position_events = 0;
        duration_events = 0;
        last_diagnostic_time = now;
      }

      switch (event)
      {
      case GDO_CB_EVENT_SYNCED:
        ESP_LOGI(TAG, "Synced: %s, protocol: %s", status->synced ? "true" : "false",
                 gdo_protocol_type_to_string(status->protocol));

        // Add debug info for protocol detection
        if (!status->synced) {
          ESP_LOGI(TAG, "Sync failed - Protocol: %s, Client ID: %" PRIu32 ", Rolling code: %" PRIu32,
                   gdo_protocol_type_to_string(status->protocol),
                   status->client_id, status->rolling_code);
        }

        // Don't attempt sync retries for unknown protocols to prevent panics
        if (status->protocol == GDO_PROTOCOL_UNKNOWN && !status->synced) {
          ESP_LOGW(TAG, "Unknown protocol detected, skipping sync retries. Please set protocol manually.");
          gdo->reset_sync_retry();
          gdo->set_sync_state(status->synced);
          break;
        }

        // Also skip sync retries for Security+ v1 if not synced to prevent issues
        if (status->protocol == GDO_PROTOCOL_SEC_PLUS_V1 && !status->synced) {
          ESP_LOGW(TAG, "Security+ v1 protocol detected but not synced. This may indicate incorrect protocol detection.");
          ESP_LOGW(TAG, "Please verify your garage door opener supports Security+ v1 or set protocol manually to 'security+2.0'.");
          gdo->reset_sync_retry();
          gdo->set_sync_state(status->synced);
          break;
        }

        // Add safety check for any other non-v2 protocols that fail to sync
        if (status->protocol != GDO_PROTOCOL_SEC_PLUS_V2 && !status->synced) {
          ESP_LOGW(TAG, "Non-Security+ v2 protocol (%s) detected but not synced. Limiting sync retries.",
                   gdo_protocol_type_to_string(status->protocol));
          // Still allow some retries but with extra caution
          if (gdo->get_sync_retry_count() >= 3) {
            ESP_LOGW(TAG, "Too many sync failures with protocol %s. Please check protocol setting.",
                     gdo_protocol_type_to_string(status->protocol));
            gdo->reset_sync_retry();
            gdo->set_sync_state(status->synced);
            break;
          }
        }

        if (status->protocol == GDO_PROTOCOL_SEC_PLUS_V2)
        {
          ESP_LOGI(TAG, "Client ID: %" PRIu32 ", Rolling code: %" PRIu32,
                   status->client_id, status->rolling_code);

          // Check current stored rolling code to prevent backward movement
          uint32_t current_rolling_code = gdo->get_rolling_code();

          if (status->synced) {
            // Valid active sync - always update credentials
            ESP_LOGI(TAG, "VALID SYNC - Saving Rolling Code: %" PRIu32 " → now: %" PRIu32 ")",
                     current_rolling_code, status->rolling_code);
            gdo->set_client_id(status->client_id);
            gdo->set_rolling_code(status->rolling_code);
          } else {
            // Sync failed - only update if rolling code moved forward
            if (status->rolling_code > current_rolling_code) {
              ESP_LOGW(TAG, "SYNC FAILED (%" PRIu32 " → %" PRIu32 ") - Saving Current Rolling Code",
                       current_rolling_code, status->rolling_code);
              gdo->set_client_id(status->client_id);
              gdo->set_rolling_code(status->rolling_code);
            }
          }
        }

        if (!status->synced)
        {
          // Additional safety: If we've already detected a problematic protocol,
          // don't attempt any sync retries to prevent crashes
          if (status->protocol == GDO_PROTOCOL_UNKNOWN ||
              status->protocol == GDO_PROTOCOL_SEC_PLUS_V1) {
            ESP_LOGW(TAG, "Skipping sync retries for problematic protocol: %s",
                     gdo_protocol_type_to_string(status->protocol));
            gdo->reset_sync_retry();
            gdo->set_sync_state(status->synced);

            // Stop the GDO communication to prevent TX pin staying active
            ESP_LOGW(TAG, "Stopping GDO communication due to protocol issues");
            ESP_LOGW(TAG, "Manually setting TX pin low to stop continuous transmission");

            // Manually set TX pin low to stop transmission
            gpio_set_level((gpio_num_t)GDO_UART_TX_PIN, 0);
            gpio_set_direction((gpio_num_t)GDO_UART_TX_PIN, GPIO_MODE_OUTPUT);

            // Note: We don't call gdo_stop() here as it might cause issues in the event handler
            return;  // Exit early to prevent any further processing
          }

          // Check if we should attempt a one-time restart after sync failure
          if (!gdo->has_restarted_for_sync()) {
            ESP_LOGW(TAG, "Sync failed - will restart in 5 seconds to recover...");
            gdo->set_has_restarted_for_sync(true);
            gdo->save_restart_flag();

            // Schedule restart after 5 seconds to allow logs to be sent
            gdo->defer_operation("restart_after_sync_failure", 5000, []() {
              ESP_LOGW(TAG, "Restarting now after sync failure...");
              App.safe_reboot();
            });

            gdo->reset_sync_retry();
            gdo->set_sync_state(status->synced);
            return;  // Exit early
          } else {
            // Already attempted restart once, don't try again to prevent boot loop
            ESP_LOGW(TAG, "Sync failed but restart already attempted - manual intervention required");
            ESP_LOGW(TAG, "Please set Client ID and Rolling Code manually in Home Assistant.");
            ESP_LOGW(TAG, "TX pin may remain active until manual sync is successful.");
            gdo->reset_sync_retry();
            gdo->set_sync_state(status->synced);
            return;  // Exit early
          }
        }
        else
        {
          // Reset retry counter on successful sync
          gdo->reset_sync_retry();
          gdo->set_protocol_state(status->protocol);

          // Clear restart-after-sync-failure flag on successful sync
          if (gdo->has_restarted_for_sync()) {
            gdo->set_has_restarted_for_sync(false);
            gdo->save_restart_flag();
            ESP_LOGI(TAG, "Sync successful - restart flag cleared");
          }

          // Note: Cannot cancel timeout from static event handler due to access restrictions
          // The timeout will naturally expire, which is fine since sync is now successful
          ESP_LOGI(TAG, "Sync successful - TX pin should now be in normal operation mode");
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
        // Process door position directly to avoid defer operation crashes
        {
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
        }
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
        // Process duration directly to avoid defer operation crashes
        gdo->set_open_duration(status->open_ms);
        break;
      case GDO_CB_EVENT_CLOSE_DURATION_MEASUREMENT:
        duration_events++;  // Count duration events
        ESP_LOGV(TAG, "Close duration: %d", status->close_ms);  // Changed to LOGV to reduce spam even more
        // Process duration directly to avoid defer operation crashes
        gdo->set_close_duration(status->close_ms);
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

      // Log boot reason to help diagnose cold boot vs warm restart differences
      esp_reset_reason_t reset_reason = esp_reset_reason();
      const char* reset_reason_str = "UNKNOWN";
      switch(reset_reason) {
        case ESP_RST_POWERON: reset_reason_str = "POWER_ON (cold boot)"; break;
        case ESP_RST_SW: reset_reason_str = "SOFTWARE"; break;
        case ESP_RST_PANIC: reset_reason_str = "PANIC"; break;
        case ESP_RST_INT_WDT: reset_reason_str = "INTERRUPT_WATCHDOG"; break;
        case ESP_RST_TASK_WDT: reset_reason_str = "TASK_WATCHDOG"; break;
        case ESP_RST_WDT: reset_reason_str = "OTHER_WATCHDOG"; break;
        case ESP_RST_DEEPSLEEP: reset_reason_str = "DEEP_SLEEP"; break;
        case ESP_RST_BROWNOUT: reset_reason_str = "BROWNOUT"; break;
        case ESP_RST_SDIO: reset_reason_str = "SDIO"; break;
        default: break;
      }
      ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      ESP_LOGI(TAG, "Reset reason: %s", reset_reason_str);
      ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

      // Initialize restart-after-sync-failure tracking
      // This allows one restart attempt after sync failure, preventing boot loops
      this->restart_attempt_pref_ = global_preferences->make_preference<bool>(fnv1_hash("restart_sync"));
      if (reset_reason == ESP_RST_POWERON) {
        // Cold boot (power cycle) - clear the restart flag for fresh start
        this->has_restarted_for_sync_ = false;
        this->restart_attempt_pref_.save(&this->has_restarted_for_sync_);
        ESP_LOGI(TAG, "Cold boot detected - restart flag cleared");
      } else {
        // Warm restart - load previous restart flag state
        bool loaded = this->restart_attempt_pref_.load(&this->has_restarted_for_sync_);
        if (loaded) {
          ESP_LOGI(TAG, "Loaded restart flag from NVS: %s", this->has_restarted_for_sync_ ? "true (restart already attempted)" : "false");
        } else {
          // No previous state, default to false
          this->has_restarted_for_sync_ = false;
          ESP_LOGI(TAG, "No restart flag in NVS, defaulting to false");
        }
      }

#ifdef TOF_SENSOR
      gdo_set_tof_timer(100000, true);
#endif
      ESP_LOGI(TAG, "Calling gdo_init() with UART%d TX:%d RX:%d",
               gdo_conf.uart_num, gdo_conf.uart_tx_pin, gdo_conf.uart_rx_pin);
      gdo_init(&gdo_conf);
      ESP_LOGI(TAG, "gdo_init() completed successfully");
      gdo_get_status(&this->status_);

      // NOTE: Protocol is NOT loaded/set from NVS on startup
      // GDOLIB will auto-detect the protocol during sync
      // The protocol select is only for manual override by the user
      // The detected protocol will be saved to NVS when sync succeeds

      // Initialize client ID and rolling code from stored values (if available)
      // This is essential for first-run and after reboot functionality
      ESP_LOGI(TAG, "Loading credentials from NVS...");
      if (this->client_id_) {
        uint32_t client_id_to_use = 0;
        if (this->client_id_->has_state()) {
          client_id_to_use = (uint32_t)this->client_id_->state;
          ESP_LOGI(TAG, "✓ Client ID has state: %" PRIu32, client_id_to_use);
        } else {
          // No stored state, use the component's initial value
          client_id_to_use = (uint32_t)this->client_id_->traits.get_min_value();
          ESP_LOGI(TAG, "⚠ No stored client ID state, using initial value: %" PRIu32, client_id_to_use);
        }

        // If client ID is still 0, it's invalid - use a reasonable default
        if (client_id_to_use == 0) {
          client_id_to_use = 1000;  // Default client ID
          ESP_LOGW(TAG, "Client ID was 0, using default: %" PRIu32, client_id_to_use);
        }

        ESP_LOGI(TAG, "Calling gdo_set_client_id(%" PRIu32 ")", client_id_to_use);
        gdo_set_client_id(client_id_to_use);
      } else {
        ESP_LOGW(TAG, "⚠ client_id_ entity is null!");
      }

      if (this->rolling_code_) {
        uint32_t rolling_code_to_use = 0;
        if (this->rolling_code_->has_state()) {
          rolling_code_to_use = (uint32_t)this->rolling_code_->state;
          ESP_LOGI(TAG, "✓ Rolling code has state: %" PRIu32, rolling_code_to_use);
        } else {
          rolling_code_to_use = (uint32_t)this->rolling_code_->traits.get_min_value();
          ESP_LOGI(TAG, "⚠ No stored rolling code state, using initial value: %" PRIu32, rolling_code_to_use);
        }

        // If rolling code is zero, set to default value
        if (rolling_code_to_use == 0) {
          rolling_code_to_use = 1000;  // Default rolling code
          ESP_LOGW(TAG, "Rolling code was 0, using default: %" PRIu32, rolling_code_to_use);
        }

        ESP_LOGI(TAG, "Calling gdo_set_rolling_code(%" PRIu32 ")", rolling_code_to_use);
        gdo_set_rolling_code(rolling_code_to_use);
      } else {
        ESP_LOGW(TAG, "⚠ rolling_code_ entity is null!");
      }

      // Load open duration from NVS and set it in GDOLIB (if previously learned)
      if (this->open_duration_ && this->open_duration_->has_state()) {
        uint16_t open_ms = (uint16_t)(this->open_duration_->state * 1000.0f);  // Convert seconds to ms
        ESP_LOGI(TAG, "Loading stored open duration: %.1f seconds (%" PRIu16 " ms)",
                 this->open_duration_->state, open_ms);
        gdo_set_open_duration(open_ms);
      }

      // Load close duration from NVS and set it in GDOLIB (if previously learned)
      if (this->close_duration_ && this->close_duration_->has_state()) {
        uint16_t close_ms = (uint16_t)(this->close_duration_->state * 1000.0f);  // Convert seconds to ms
        ESP_LOGI(TAG, "Loading stored close duration: %.1f seconds (%" PRIu16 " ms)",
                 this->close_duration_->state, close_ms);
        gdo_set_close_duration(close_ms);
      }

#ifdef TOF_SENSOR
      vt.setup_tof_sensor(gpio_num_t(GDO_TOF_SDA_PIN), gpio_num_t(GDO_TOF_SCL_PIN), 400000);
      delay(100);
#endif

      if (this->start_gdo_)
      {
        gdo_start(gdo_event_handler, this);
        ESP_LOGI(TAG, "Secplus GDO started!");

        // Add a timeout to stop transmission if no sync after 30 seconds
        this->set_timeout("sync_timeout", 30000, [this]() {
          ESP_LOGW(TAG, "Sync timeout reached - manually stopping TX to prevent continuous transmission");
          gpio_set_level((gpio_num_t)GDO_UART_TX_PIN, 0);
          gpio_set_direction((gpio_num_t)GDO_UART_TX_PIN, GPIO_MODE_OUTPUT);
        });
      }
      else
      {
        // check every 500ms for readiness before starting GDO
        this->set_interval("gdo_start", 500, [this]()
                           {
      if (this->start_gdo_) {
        gdo_start(gdo_event_handler, this);
        ESP_LOGI(TAG, "Secplus GDO started!");
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
