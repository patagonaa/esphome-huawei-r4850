#pragma once
#include "../huawei_r4850.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/switch/switch.h"

#include <optional>

namespace esphome {
namespace huawei_r4850 {

class HuaweiR4850Switch : public switch_::Switch, public Component, public HuaweiR4850Input {
 public:
  void setup() override;

  void set_parent(HuaweiR4850Component *parent, uint16_t registerId) {
    this->parent_ = parent;
    this->registerId_ = registerId;
  };
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

  void handle_update(uint16_t register_id, std::vector<uint8_t> &data) override;
  void handle_error(uint16_t register_id, std::vector<uint8_t> &data) override;
  void handle_timeout() override;

 protected:
  HuaweiR4850Component *parent_;
  uint16_t registerId_;
  std::optional<bool> last_state_;
  bool restore_value_{false};

  bool assumed_state_{true};

  void send_state_(bool value);

  void write_state(bool state) override;
  bool assumed_state() override;

  ESPPreferenceObject pref_;
};

}  // namespace huawei_r4850
}  // namespace esphome
