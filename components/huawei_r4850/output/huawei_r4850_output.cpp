#include "huawei_r4850_output.h"
#include "esphome/core/log.h"

namespace esphome {
namespace huawei_r4850 {

static const uint16_t SET_FAN_DUTY_CYCLE = 0x114;

void HuaweiR4850FloatOutput::set_parent(HuaweiR4850Component *parent, uint16_t registerId) {
  this->parent_ = parent;
  this->registerId_ = registerId;
}

void HuaweiR4850FloatOutput::write_state(float state) {
  if (this->registerId_ == SET_FAN_DUTY_CYCLE) {
    uint16_t raw = state * 25600.0f;
    std::vector<uint8_t> data = {(uint8_t)((raw >> 8) & 0xFF), (uint8_t)(raw & 0xFF), 0x00, 0x00, 0x00, 0x00};
    parent_->set_value(this->registerId_, data);
  } else {
    int32_t raw = state * 1024.0f;
    std::vector<uint8_t> data = {0x00, 0x00, (uint8_t)((raw >> 24) & 0xFF), (uint8_t)((raw >> 16) & 0xFF), (uint8_t)((raw >> 8) & 0xFF), (uint8_t)(raw & 0xFF)};
    parent_->set_value(this->registerId_, data);
  }
}

}  // namespace huawei_r4850
}  // namespace esphome
