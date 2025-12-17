#include "esphome/core/log.h"
#include "gdo_door.h"
#include "../secplus_gdo.h"

namespace esphome {
namespace secplus_gdo {

const char *TAG = "gdo_cover";

void GDODoor::set_state(gdo_door_state_t state, float position) {
  if (this->pre_close_active_) {
    // If we are in the pre-close state and the door is closing,
    // then it was triggered by something else and we need to cancel the
    // pre-close
    if (state == GDO_DOOR_STATE_CLOSING) {
      this->cancel_timeout("pre_close");
      this->pre_close_active_ = false;
      if (this->pre_close_end_trigger) {
        this->pre_close_end_trigger->trigger();
      }
    } else {
      // If we are in the pre-close state, and the door is not closing, then do
      // not update the state so that the stop button remains active on the
      // front end to cancel the closing operation
      return;
    }
  }
  if (this->state_ == state && this->position == position) {
    return;
  }

  ESP_LOGI(TAG, "Door state: %s, position: %.0f%%",
           gdo_door_state_to_string(state), position * 100.0f);

  switch (state) {
  case GDO_DOOR_STATE_OPEN:
    this->position = COVER_OPEN;
    this->current_operation = COVER_OPERATION_IDLE;
    break;
  case GDO_DOOR_STATE_CLOSED:
    this->position = COVER_CLOSED;
    this->current_operation = COVER_OPERATION_IDLE;
    break;
  case GDO_DOOR_STATE_OPENING:
    this->current_operation = COVER_OPERATION_OPENING;
    this->position = position;
    break;
  case GDO_DOOR_STATE_CLOSING:
    this->current_operation = COVER_OPERATION_CLOSING;
    this->position = position;
    break;
  case GDO_DOOR_STATE_STOPPED:
    this->prev_operation = this->current_operation;
    // falls through
  case GDO_DOOR_STATE_MAX:
  default:
    this->current_operation = COVER_OPERATION_IDLE;
    this->position = position;
    break;
  }

  this->publish_state(false);
  this->state_ = state;
}

void GDODoor::do_action_after_warning(const cover::CoverCall &call) {
  // Save current TTC value
  uint16_t saved_ttc = 0;
  if (this->parent_) {
    saved_ttc = this->parent_->get_time_to_close();
    ESP_LOGI(TAG, "Close notification: Saved TTC=%d, setting to 1 for warning", saved_ttc);
  }

  if (this->pre_close_start_trigger) {
    this->pre_close_start_trigger->trigger();
  }

  // Set TTC to 1 second - this triggers the 10-second warning
  esp_err_t err = gdo_set_time_to_close(1);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set TTC for warning, closing without warning");
    this->do_action(call);
    return;
  }

  // Trigger the close command which will execute after warning completes (~11 seconds)
  this->set_timeout("close_after_warning", 11000, [this, call, saved_ttc]() {
    ESP_LOGI(TAG, "Warning complete, closing door now");
    this->do_action(call);

    // Restore original TTC after door starts closing
    ESP_LOGI(TAG, "Restoring TTC to %d", saved_ttc);
    gdo_set_time_to_close(saved_ttc);

    if (this->pre_close_end_trigger) {
      this->pre_close_end_trigger->trigger();
    }
  });
}

void GDODoor::do_action(const cover::CoverCall &call) {
  if (call.get_stop()) {
    ESP_LOGD(TAG, "Sending STOP action");
    // Use timeout to defer GDOLIB call to avoid blocking web server
    this->set_timeout("stop_action", 1, []() {
      gdo_door_stop();
    });
  }

  if (call.get_toggle()) {
    if (this->position == COVER_CLOSED) {
      this->set_state(GDO_DOOR_STATE_OPENING, this->position);
    } else if (this->position == COVER_OPEN) {
      this->set_state(GDO_DOOR_STATE_CLOSING, this->position);
    }
    ESP_LOGD(TAG, "Sending TOGGLE action");
    // Use timeout to defer GDOLIB call to avoid blocking web server
    this->set_timeout("toggle_action", 1, []() {
      gdo_door_toggle();
    });
  }

  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (pos == COVER_OPEN) {
      if (this->toggle_only_) {
        ESP_LOGD(TAG, "Sending TOGGLE action");
        // Use timeout to defer GDOLIB call to avoid blocking web server
        this->set_timeout("open_toggle_action", 1, []() {
          gdo_door_toggle();
        });
        if (this->state_ == GDO_DOOR_STATE_STOPPED &&
            this->prev_operation == COVER_OPERATION_OPENING) {
          // If the door was stopped while opening, then we need to toggle to
          // stop, then toggle again to open,
          this->set_timeout("stop_door", 1000, [=]() {
            gdo_door_stop();
          });
          this->set_timeout("open_door", 2000, [=]() {
            gdo_door_toggle();
          });
        }
      } else {
        ESP_LOGD(TAG, "Sending OPEN action");
        // Use timeout to defer GDOLIB call to avoid blocking web server
        this->set_timeout("open_action", 1, []() {
          gdo_door_open();
        });
      }

      this->set_state(GDO_DOOR_STATE_OPENING, this->position);
    } else if (pos == COVER_CLOSED) {
      if (this->toggle_only_) {
        ESP_LOGD(TAG, "Sending TOGGLE action");
        // Use timeout to defer GDOLIB call to avoid blocking web server
        this->set_timeout("close_toggle_action", 1, []() {
          gdo_door_toggle();
        });
        if (this->state_ == GDO_DOOR_STATE_STOPPED &&
            this->prev_operation == COVER_OPERATION_CLOSING) {
          // If the door was stopped while closing, then we need to toggle to
          // stop, then toggle again to close,
          this->set_timeout("stop_door", 1000, [=]() {
            gdo_door_stop();
          });
          this->set_timeout("close_door", 2000, [=]() {
            gdo_door_toggle();
          });
        }
      } else {
        ESP_LOGD(TAG, "Sending CLOSE action");
        // Use timeout to defer GDOLIB call to avoid blocking web server
        this->set_timeout("close_action", 1, []() {
          gdo_door_close();
        });
      }

      this->set_state(GDO_DOOR_STATE_CLOSING, this->position);
    } else {
      ESP_LOGD(TAG, "Moving garage door to position %f", pos);
      // Use timeout to defer GDOLIB call to avoid blocking web server
      uint16_t target = 10000 - (pos * 10000);
      this->set_timeout("move_to_position", 1, [target]() {
        gdo_door_move_to_target(target);
      });
    }
  }
}

void GDODoor::control(const cover::CoverCall &call) {
  if (!this->synced_) {
    this->publish_state(false);
    return;
  }

  if (call.get_stop()) {
    ESP_LOGD(TAG, "Stop command received");
    if (this->pre_close_active_) {
      ESP_LOGD(TAG, "Canceling pending action");
      this->cancel_timeout("pre_close");
      this->pre_close_active_ = false;
      if (this->pre_close_end_trigger) {
        this->pre_close_end_trigger->trigger();
      }
    }
    this->target_position_ = this->position;
    this->do_action(call);
  }

  if (call.get_toggle()) {
    ESP_LOGD(TAG, "Toggle command received");
    if (this->position != COVER_CLOSED) {
      this->target_position_ = COVER_CLOSED;
      if (this->close_notification_) {
        this->do_action_after_warning(call);
      } else {
        this->do_action(call);
      }
    } else {
      this->target_position_ = COVER_OPEN;
      this->do_action(call);
    }
  }

  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (this->position == pos) {
      ESP_LOGD(TAG, "Door is already %s",
               pos == COVER_OPEN ? "open" : "closed");
      this->publish_state(false);
      return;
    }

    if ((this->current_operation == COVER_OPERATION_OPENING &&
         pos > this->position) ||
        (this->current_operation == COVER_OPERATION_CLOSING &&
         pos < this->position)) {
      ESP_LOGD(
          TAG,
          "Door is already moving in target direction; target position: %.0f%%",
          *this->target_position_);
      this->publish_state(false);
      return;
    }

    if (this->pre_close_active_) {
      // don't start the pre-close again if the door is already going to close.
      if (pos < this->position) {
        ESP_LOGD(TAG, "Door is already closing");
        this->publish_state(false);
        return;
      }

      ESP_LOGD(TAG, "Canceling pending action");
      this->cancel_timeout("pre_close");
      this->pre_close_active_ = false;
      if (this->pre_close_end_trigger) {
        this->pre_close_end_trigger->trigger();
      }
    }

    if (this->current_operation == COVER_OPERATION_OPENING ||
        this->current_operation == COVER_OPERATION_CLOSING) {
      ESP_LOGD(TAG, "Door is in motion - Sending STOP action");
      gdo_door_stop();
    }

    if (pos == COVER_OPEN) {
      ESP_LOGD(TAG, "Open command received");
      this->do_action(call);
    } else if (pos == COVER_CLOSED) {
      ESP_LOGD(TAG, "Close command received");
      if (this->close_notification_) {
        this->do_action_after_warning(call);
      } else {
        this->do_action(call);
      }
    } else {
      ESP_LOGD(TAG, "Move to position %f command received", pos);
      if (pos < this->position) {
        ESP_LOGV(TAG,
                 "Current position: %f; Intended position: %f; Door will move "
                 "down%s",
                 this->position, pos, this->close_notification_ ? " after warning" : " immediately");
        if (this->close_notification_) {
          this->do_action_after_warning(call);
        } else {
          this->do_action(call);
        }
      } else {
        ESP_LOGV(TAG,
                 "Current position: %f; Intended position: %f; Door will move "
                 "up immediately",
                 this->position, pos);
        this->do_action(call);
      }
    }

    this->target_position_ = pos;
  }
}

} // namespace secplus_gdo
} // namespace esphome
