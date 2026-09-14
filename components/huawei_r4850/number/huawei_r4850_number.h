#pragma once
#include "../huawei_r4850.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/number/number.h"

#include <optional>

namespace esphome {
namespace huawei_r4850 {

class HuaweiR4850Number : public number::Number, public Component, public HuaweiR4850Input {
 public:
  void setup() override;

  void set_parent(HuaweiR4850Component *parent, uint16_t registerId);
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

  void handle_update(uint16_t register_id, std::vector<uint8_t> &data) override;
  void handle_error(uint16_t register_id, std::vector<uint8_t> &data) override;
  void handle_timeout() override;

 protected:
  HuaweiR4850Component *parent_;
  uint16_t registerId_;
  std::optional<float> last_state_;
  bool restore_value_{false};

  void send_state_(float value);
  void control(float value) override;

  ESPPreferenceObject pref_;
};

}  // namespace huawei_r4850
}  // namespace esphome
