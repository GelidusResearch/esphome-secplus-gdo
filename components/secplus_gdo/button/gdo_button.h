/************************************
 *
 * Copyright (C) 2026 Gelidus Research
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#pragma once

#include "esphome/components/button/button.h"
#include "include/gdo.h"

namespace esphome {
namespace secplus_gdo {

class GDOButton : public button::Button {
public:
  void press_action() override {
    gdo_enable_ttc();
    gdo_get_ttc_state();
  }
};

} // namespace secplus_gdo
} // namespace esphome
