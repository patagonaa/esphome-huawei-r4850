#pragma once
#include "../huawei_r4850.h"
#include "esphome/core/component.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace huawei_r4850 {

class HuaweiR4850Number : public number::Number, public Component, public HuaweiR4850Input {
 public:
  void set_parent(HuaweiR4850Component *parent, uint16_t registerId);
  void handle_update(uint16_t register_id, std::vector<uint8_t> &data) override;
  void handle_error(uint16_t register_id, std::vector<uint8_t> &data) override;
  void handle_timeout() override;

 protected:
  HuaweiR4850Component *parent_;
  uint16_t registerId_;

  void control(float value) override;
};

}  // namespace huawei_r4850
}  // namespace esphome
