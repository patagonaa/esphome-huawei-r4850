#include "huawei_r4850_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace huawei_r4850 {

static const int16_t SET_FAN_SPEED_MAX_FUNCTION = 0x134;
static const int16_t SET_STANDBY_FUNCTION = 0x132;

void HuaweiR4850Switch::setup() {
  optional<bool> initial_state = this->get_initial_state_with_restore_mode();

   if (initial_state.has_value())
    this->write_state(initial_state.value());
}

void HuaweiR4850Switch::write_state(bool state) {
  this->last_state_ = state;
  this->send_state_(state);
}

void HuaweiR4850Switch::send_state_(bool state) {
  std::vector<uint8_t> data = {0x00, (uint8_t)(state ? 0x01 : 0x00), 0x00, 0x00, 0x00, 0x00};
  this->parent_->set_value(this->registerId_, data);
}

void HuaweiR4850Switch::handle_resend() {
  if (this->resend_ && this->last_state_.has_value()) {
    this->send_state_(this->last_state_.value());
  }
}

bool HuaweiR4850Switch::assumed_state() {
  // Since we cannot poll the device for the actual state of the switch we
  // have to use assumed state
  return true;
}

void HuaweiR4850Switch::handle_update(uint16_t register_id, std::vector<uint8_t> &data) {
  if (register_id != this->registerId_)
    return;
  this->publish_state(data[1]);
}

void HuaweiR4850Switch::handle_error(uint16_t register_id, std::vector<uint8_t> &data) {

}

void HuaweiR4850Switch::handle_timeout() {

}

}  // namespace huawei_r4850
}  // namespace esphome
