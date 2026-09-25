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
    virtual void handle_connected() = 0;
    virtual void handle_timeout() = 0;
    virtual void handle_resend() = 0;
};

enum class R4850InitStatus {
  NegotiatingAddress,
  Init,
  GetAddressBySlot,
  WaitForUnsolicited,
  GetElabel,
  GetInfo,
  Ready,
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
#endif // USE_BINARY_SENSOR

 public:
  HuaweiR4850Component(canbus::Canbus *canbus);
  void setup() override;
  void loop() override;
  void update() override;

  const char *get_addr_log_str() {
    return addr_log_str_;
  }
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

  void set_psu_slot_id(uint16_t value) {
    this->psu_slot_id_ = value;
  }

  esphome::optional<float> get_psu_nominal_current() {
    return psu_nominal_current_;
  }

  void set_resend_interval(uint32_t interval);
  void resend_inputs();

 protected:
  canbus::Canbus *canbus;
  esphome::optional<uint8_t> psu_addr_{};
  esphome::optional<uint16_t> psu_slot_id_{};
  char addr_log_str_[16]{};

  uint32_t last_renegotiation_message_{0};

  R4850InitStatus init_status_ = R4850InitStatus::Init;
  uint32_t last_init_request_{0};

  bool needs_fan_status_{0};

  uint32_t last_unsolicited_message_{0};

  bool has_received_elabel_response_ = false;
  std::string raw_elabel_response_;

  bool has_received_info_response_ = false;
  esphome::optional<float> psu_nominal_current_{};

#ifdef USE_SENSOR
  void publish_sensor_state_(sensor::Sensor *sensor, float state) {
    if (sensor) {
      sensor->publish_state(state);
    }
  }

  sensor::Sensor *fan_duty_cycle_min_sensor_{nullptr};
  sensor::Sensor *fan_duty_cycle_target_sensor_{nullptr};
  sensor::Sensor *fan_rpm_sensor_{nullptr};
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

  void set_init_status_(R4850InitStatus init_status);
  void on_frame(uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &message);

  void handle_timeout_();
  void handle_status_update_(uint8_t error_type, uint16_t register_id, std::vector<uint8_t> &data);
  void handle_control_update_(uint8_t error_type, uint16_t register_id, std::vector<uint8_t> &data);
  void handle_elabel_(bool incomplete, uint16_t register_id, std::vector<uint8_t> &data);
  void handle_info_(bool incomplete, uint16_t register_id, std::vector<uint8_t> &data);

  uint32_t canid_pack_(uint8_t proto, uint8_t addr, uint8_t command, bool src_controller, bool incomplete);
  void canid_unpack_(uint32_t canId, uint8_t *proto, uint8_t *addr, uint8_t *command, bool *src_controller, bool *incomplete);
};

}  // namespace huawei_r4850
}  // namespace esphome
