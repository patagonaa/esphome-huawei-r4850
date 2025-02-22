#pragma once
#include "../huawei_r4850.h"
#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"

namespace esphome {
namespace huawei_r4850 {

class HuaweiR4850FloatOutput : public output::FloatOutput, public Component {
 public:
  void set_parent(HuaweiR4850Component *parent, uint16_t registerId);

 protected:
  HuaweiR4850Component *parent_;
  uint16_t registerId_;

  void write_state(float state) override;
};

}  // namespace huawei_r4850
}  // namespace esphome
