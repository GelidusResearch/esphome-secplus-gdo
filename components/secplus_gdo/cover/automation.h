#pragma once

#include "esphome/core/automation.h"

namespace esphome {
namespace secplus_gdo {
class GDODoor;

class CoverClosingStartTrigger : public Trigger<> {
public:
  CoverClosingStartTrigger(GDODoor *gdo_cover) {}
};

class CoverClosingEndTrigger : public Trigger<> {
public:
  CoverClosingEndTrigger(GDODoor *gdo_cover) {}
};
} // namespace secplus_gdo
} // namespace esphome