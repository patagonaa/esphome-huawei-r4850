#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/canbus/canbus.h"

namespace esphome {
namespace huawei_r4850 {

class HuaweiR4850Input {
 public:
    HuaweiR4850Input() {}
    virtual void handle_update(uint16_t register_id, std::vector<uint8_t> &data) = 0;
    virtual void handle_error(uint16_t register_id, std::vector<uint8_t> &data) = 0;
    virtual void handle_timeout() = 0;
    virtual void handle_resend() = 0;
};

class HuaweiR4850Component : public PollingComponent {
#ifdef USE_SENSOR
  SUB_SENSOR(operating_hours)
  SUB_SENSOR(input_voltage)
  SUB_SENSOR(input_frequency)
  SUB_SENSOR(input_current)
  SUB_SENSOR(input_power)
  SUB_SENSOR(input_temp)
  SUB_SENSOR(efficiency)
  SUB_SENSOR(output_voltage)
  SUB_SENSOR(output_current)
  SUB_SENSOR(output_current_setpoint)
  SUB_SENSOR(output_power)
  SUB_SENSOR(output_temp)
#endif // USE_SENSOR

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(board_type)
  SUB_TEXT_SENSOR(serial_number)
  SUB_TEXT_SENSOR(item)
  SUB_TEXT_SENSOR(model)
#endif // USE_TEXT_SENSOR

#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(canbus_connectivity)
  SUB_BINARY_SENSOR(ac_present)
  SUB_BINARY_SENSOR(current_limiting)
#endif // USE_BINARY_SENSOR

 public:
  HuaweiR4850Component(canbus::Canbus *canbus);
  void setup() override;
  void update() override;

  void set_value(uint16_t register_id, std::vector<uint8_t> &data);

#ifdef USE_SENSOR
  void set_fan_duty_cycle_min_sensor(sensor::Sensor *fan_duty_cycle_min_sensor) {
    fan_duty_cycle_min_sensor_ = fan_duty_cycle_min_sensor;
    needs_fan_status_ = true;
  }
  void set_fan_duty_cycle_target_sensor(sensor::Sensor *fan_duty_cycle_target_sensor) {
    fan_duty_cycle_target_sensor_ = fan_duty_cycle_target_sensor;
    needs_fan_status_ = true;
  }
  void set_fan_rpm_sensor(sensor::Sensor *fan_rpm_sensor) {
    fan_rpm_sensor_ = fan_rpm_sensor;
    needs_fan_status_ = true;
  }
#endif // USE_SENSOR

  void register_input(HuaweiR4850Input *number) {
    this->registered_inputs_.push_back(number);
  }

  void set_psu_address(uint8_t value) {
    this->psu_addr_ = value;
  }

  void set_psu_max_current(float value) {
    psu_max_current_ = value;
  }

  float get_psu_max_current() {
    return psu_max_current_;
  }

  void set_resend_interval(uint32_t interval);
  void resend_inputs();

 protected:
  canbus::Canbus *canbus;
  float psu_max_current_;
  uint8_t psu_addr_;

  bool has_received_elabel_response_ = false;
  std::string raw_elabel_response_;
  uint32_t last_unsolicited_message_{0};
  bool canbus_connectivity_ = false;

#ifdef USE_SENSOR
  void publish_sensor_state_(sensor::Sensor *sensor, float state) {
    if (sensor) {
      sensor->publish_state(state);
    }
  }

  sensor::Sensor *fan_duty_cycle_min_sensor_{nullptr};
  sensor::Sensor *fan_duty_cycle_target_sensor_{nullptr};
  sensor::Sensor *fan_rpm_sensor_{nullptr};
  bool needs_fan_status_{0};
#endif // USE_SENSOR

#ifdef USE_TEXT_SENSOR
  void publish_sensor_state_(text_sensor::TextSensor *sensor, const char *state) {
    if (sensor) {
      sensor->publish_state(state);
    }
  }
#endif // USE_TEXT_SENSOR

#ifdef USE_BINARY_SENSOR
  void publish_sensor_state_(binary_sensor::BinarySensor *sensor, bool state) {
    if (sensor) {
      sensor->publish_state(state);
    }
  }
#endif // USE_BINARY_SENSOR

  std::vector<HuaweiR4850Input *> registered_inputs_{};

  void on_frame(uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &message);

  uint32_t canid_pack_(uint8_t addr, uint8_t command, bool src_controller, bool incomplete);
  void canid_unpack_(uint32_t canId, uint8_t *addr, uint8_t *command, bool *src_controller, bool *incomplete);
};

}  // namespace huawei_r4850
}  // namespace esphome
