#include "huawei_r4850.h"

#include <cassert>
#include <map>
#include <sstream>
#include <string>

#include "esphome/core/application.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace huawei_r4850 {

static const char *const TAG = "huawei_r4850";

static const uint8_t R48xx_CMD_DATA = 0x40;
static const uint8_t R48xx_CMD_ELABEL = 0xD2;
static const uint8_t R48xx_CMD_CONTROL = 0x80;
static const uint8_t R48xx_CMD_REGISTER_GET = 0x82;
static const uint8_t R48xx_CMD_UNSOLICITED = 0x11;

static const uint16_t R48xx_DATA_OPERATING_HOURS = 0x10E;
static const uint16_t R48xx_DATA_INPUT_POWER = 0x170;
static const uint16_t R48xx_DATA_INPUT_FREQ = 0x171;
static const uint16_t R48xx_DATA_INPUT_CURRENT = 0x172;
static const uint16_t R48xx_DATA_OUTPUT_POWER = 0x173;
static const uint16_t R48xx_DATA_EFFICIENCY = 0x174;
static const uint16_t R48xx_DATA_OUTPUT_VOLTAGE = 0x175;
static const uint16_t R48xx_DATA_OUTPUT_CURRENT_MAX = 0x176;
static const uint16_t R48xx_DATA_INPUT_VOLTAGE = 0x178;
static const uint16_t R48xx_DATA_OUTPUT_TEMPERATURE = 0x17F;
static const uint16_t R48xx_DATA_INPUT_TEMPERATURE = 0x180;
static const uint16_t R48xx_DATA_OUTPUT_CURRENT_FAST = 0x181;
static const uint16_t R48xx_DATA_OUTPUT_CURRENT_SLOW = 0x182;
static const uint16_t R48xx_DATA_STATUS_FLAGS = 0x183;
static const uint16_t R48xx_DATA_FAN_STATUS = 0x187;

typedef std::map<std::string, std::string> ELabelResponse;

ELabelResponse parse_elabel_response(const std::string &raw_response) {
  ELabelResponse elabel_response;

  // Split by line
  std::istringstream sstream(raw_response);
  std::string line;

  while (std::getline(sstream, line, '\n')) {
    if (line.length() == 0 || line[0] == '/') {
      continue;
    }

    // Split by "="
    size_t pos = line.find("=");
    if (pos != std::string::npos) {
      std::string key = line.substr(0, pos);
      std::string value = line.substr(pos + 1);

      value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

      elabel_response[key] = value;
    }
  }

  return elabel_response;
}

HuaweiR4850Component::HuaweiR4850Component(canbus::Canbus *canbus) { this->canbus = canbus; }

void HuaweiR4850Component::setup() {
  auto cb = [this](uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &data) -> void {
    this->on_frame(can_id, extended_id, rtr, data);
  };

  this->canbus->add_callback(cb);
}

void HuaweiR4850Component::set_resend_interval(uint32_t interval) {
  this->set_interval("resend", interval, [this]() { this->resend_inputs(); });
}

void HuaweiR4850Component::resend_inputs() {
  if (init_status_ == R4850InitStatus::Ready) {
    for (auto &input : this->registered_inputs_) {
      input->handle_resend();
    }
  }
}

void HuaweiR4850Component::loop() {
  // unsolicited messages should be received every ~377ms. Wait some extra time to make sure one was actually
  // supposed to arrive and no other components delayed CAN receive enough to trigger this.
  bool can_connected = last_unsolicited_message_ != 0 && (millis() - last_unsolicited_message_ < 1000);
  if (!can_connected && init_status_ != R4850InitStatus::Disconnected) {
#ifdef USE_BINARY_SENSOR
    this->publish_sensor_state_(canbus_connectivity_binary_sensor_, false);
#endif // USE_BINARY_SENSOR
    handle_timeout_();
    ESP_LOGW(TAG, "No unsolicited messages received lately, stopping polling");
    init_status_ = R4850InitStatus::Disconnected;
  } else if (can_connected && init_status_ == R4850InitStatus::Disconnected) {
#ifdef USE_BINARY_SENSOR
    this->publish_sensor_state_(canbus_connectivity_binary_sensor_, true);
#endif // USE_BINARY_SENSOR
    ESP_LOGI(TAG, "Got unsolicited messages on CAN bus, starting init");
    init_status_ = R4850InitStatus::Init;
  } else {
    switch (init_status_)
    {
    case R4850InitStatus::Init:
      has_received_elabel_response_ = false;
      last_init_request_ = 0;
      init_status_ = R4850InitStatus::GetElabel;
      break;

    case R4850InitStatus::GetElabel:
    {
      if (has_received_elabel_response_) {
        ESP_LOGD(TAG, "Received E-label response");
        init_status_ = R4850InitStatus::Ready;
        last_init_request_ = 0;
      } else if (last_init_request_ == 0 || millis() - last_init_request_ > 5000) {
        ESP_LOGD(TAG, "Sending E-label request");
        raw_elabel_response_.clear();

        uint32_t canId = this->canid_pack_(this->psu_addr_, R48xx_CMD_ELABEL, true, false);
        std::vector<uint8_t> data = {0, 0, 0, 0, 0, 0, 0, 0};
        this->canbus->send_data(canId, true, data);
        last_init_request_ = millis();
      }
      break;
    }
    
    default:
      break;
    }
  }
}

void HuaweiR4850Component::update() {
  if (init_status_ == R4850InitStatus::Ready) {
    ESP_LOGD(TAG, "Sending data request message");
    {
      uint32_t canId = this->canid_pack_(this->psu_addr_, R48xx_CMD_DATA, true, false);
      std::vector<uint8_t> data = {0, 0, 0, 0, 0, 0, 0, 0};
      this->canbus->send_data(canId, true, data);
    }

    if (this->needs_fan_status_) {
      uint32_t canId = this->canid_pack_(this->psu_addr_, R48xx_CMD_REGISTER_GET, true, false);
      std::vector<uint8_t> data = {
        (uint8_t)((R48xx_DATA_FAN_STATUS & 0xF00) >> 8), (uint8_t)(R48xx_DATA_FAN_STATUS & 0x0FF), 0, 0, 0, 0, 0, 0
      };
      this->canbus->send_data(canId, true, data);
    }
  }
}

void HuaweiR4850Component::set_value(uint16_t register_id, std::vector<uint8_t> &data) {
  if(data.size() != 6)
  {
    ESP_LOGE(TAG, "Invalid data count for register id %03x", register_id);
    return;
  }

  uint32_t canId = this->canid_pack_(this->psu_addr_, R48xx_CMD_CONTROL, true, false);

  std::vector<uint8_t> message = {(uint8_t)((register_id & 0xF00) >> 8), (uint8_t)(register_id & 0x0FF)};
  message.insert(message.end(), data.begin(), data.end());

  this->canbus->send_data(canId, true, message);
}

void HuaweiR4850Component::on_frame(uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &message) {
  if (message.size() < 8 || !extended_id) {
    return;
  }

  uint8_t psu_addr, cmd;
  bool src_controller, incomplete;
  this->canid_unpack_(can_id, &psu_addr, &cmd, &src_controller, &incomplete);

  if (psu_addr != this->psu_addr_ || src_controller) {
    // not from our PSU -> skip
    // this used to be handled by a bit mask, but I'm pretty sure this is fast enough as well
    return;
  }

  uint8_t error_type = (message[0] & 0xF0) >> 4;
  uint16_t register_id = ((message[0] & 0x0F) << 8) | message[1];
  std::vector<uint8_t> data(message.begin() + 2, message.end());

  if (cmd == R48xx_CMD_DATA || cmd == R48xx_CMD_REGISTER_GET) {
    handle_status_update_(error_type, register_id, data);
  } else if (cmd == R48xx_CMD_CONTROL) {
    handle_control_update_(error_type, register_id, data);
  } else if (cmd == R48xx_CMD_ELABEL) {
    handle_elabel_(incomplete, register_id, data);
  } else if (cmd == R48xx_CMD_UNSOLICITED) {
    last_unsolicited_message_ = millis();
  }
}

void HuaweiR4850Component::handle_timeout_()
{
  // canbus disconnected -> set sensors to NAN
#ifdef USE_SENSOR
  this->publish_sensor_state_(this->operating_hours_sensor_, NAN);
  this->publish_sensor_state_(this->input_voltage_sensor_, NAN);
  this->publish_sensor_state_(this->input_frequency_sensor_, NAN);
  this->publish_sensor_state_(this->input_current_sensor_, NAN);
  this->publish_sensor_state_(this->input_power_sensor_, NAN);
  this->publish_sensor_state_(this->input_temp_sensor_, NAN);
  this->publish_sensor_state_(this->efficiency_sensor_, NAN);
  this->publish_sensor_state_(this->output_voltage_sensor_, NAN);
  this->publish_sensor_state_(this->output_current_sensor_, NAN);
  this->publish_sensor_state_(this->output_current_setpoint_sensor_, NAN);
  this->publish_sensor_state_(this->output_power_sensor_, NAN);
  this->publish_sensor_state_(this->output_temp_sensor_, NAN);
  this->publish_sensor_state_(this->fan_duty_cycle_min_sensor_, NAN);
  this->publish_sensor_state_(this->fan_duty_cycle_target_sensor_, NAN);
  this->publish_sensor_state_(this->fan_rpm_sensor_, NAN);
#endif // USE_SENSOR

  for (auto &input : this->registered_inputs_) {
    input->handle_timeout();
  }
}

void HuaweiR4850Component::handle_status_update_(uint8_t error_type, uint16_t register_id, std::vector<uint8_t> &data)
{
  if (error_type != 0) {
    ESP_LOGW(TAG, "Value %03x get error: %d", register_id, error_type);
    return;
  }

  int32_t value = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
  float conv_value = 0;
  switch (register_id) {
#ifdef USE_SENSOR
    case R48xx_DATA_OPERATING_HOURS:
      this->publish_sensor_state_(this->operating_hours_sensor_, value);
      ESP_LOGV(TAG, "Operating Hours: %" PRIi32, value);
      break;

    case R48xx_DATA_INPUT_POWER:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->input_power_sensor_, conv_value);
      ESP_LOGV(TAG, "Input power: %f", conv_value);
      break;

    case R48xx_DATA_INPUT_FREQ:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->input_frequency_sensor_, conv_value);
      ESP_LOGV(TAG, "Input frequency: %f", conv_value);
      break;

    case R48xx_DATA_INPUT_CURRENT:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->input_current_sensor_, conv_value);
      ESP_LOGV(TAG, "Input current: %f", conv_value);
      break;

    case R48xx_DATA_OUTPUT_POWER:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->output_power_sensor_, conv_value);
      ESP_LOGV(TAG, "Output power: %f", conv_value);
      break;

    case R48xx_DATA_EFFICIENCY:
      conv_value = value / 1024.0f * 100.0f;
      this->publish_sensor_state_(this->efficiency_sensor_, conv_value);
      ESP_LOGV(TAG, "Efficiency: %f", conv_value);
      break;

    case R48xx_DATA_OUTPUT_VOLTAGE:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->output_voltage_sensor_, conv_value);
      ESP_LOGV(TAG, "Output voltage: %f", conv_value);
      break;

    case R48xx_DATA_OUTPUT_CURRENT_MAX:
      // this is not equal to the value set via max_output_current
      // as it is also set (according to the current AC input voltage) when AC limit is set
      conv_value = value / 1250.0f * this->psu_max_current_;
      this->publish_sensor_state_(this->output_current_setpoint_sensor_, conv_value);
      ESP_LOGV(TAG, "Max Output current: %f", conv_value);
      break;

    case R48xx_DATA_INPUT_VOLTAGE:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->input_voltage_sensor_, conv_value);
      ESP_LOGV(TAG, "Input voltage: %f", conv_value);
      break;

    case R48xx_DATA_OUTPUT_TEMPERATURE:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->output_temp_sensor_, conv_value);
      ESP_LOGV(TAG, "Output temperature: %f", conv_value);
      break;

    case R48xx_DATA_INPUT_TEMPERATURE:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->input_temp_sensor_, conv_value);
      ESP_LOGV(TAG, "Input temperature: %f", conv_value);
      break;

    case R48xx_DATA_OUTPUT_CURRENT_FAST:
      conv_value = value / 1024.0f;
      this->publish_sensor_state_(this->output_current_sensor_, conv_value);
      ESP_LOGV(TAG, "Output current: %f", conv_value);
      break;

    case R48xx_DATA_OUTPUT_CURRENT_SLOW:
      conv_value = value / 1024.0f;
      ESP_LOGV(TAG, "Output current: %f", conv_value);
      break;

    case R48xx_DATA_FAN_STATUS:
    {
      uint16_t duty_min = ((data[0] << 8) | data[1]) / 256;
      uint16_t duty_target = ((data[2] << 8) | data[3]) / 256;
      uint16_t rpm = duty_target > 0 ? (data[4] << 8) | data[5] : 0; // rpm contains the last value even when fan is off due to no AC
      this->publish_sensor_state_(this->fan_duty_cycle_min_sensor_, duty_min);
      this->publish_sensor_state_(this->fan_duty_cycle_target_sensor_, duty_target);
      this->publish_sensor_state_(this->fan_rpm_sensor_, rpm);
      ESP_LOGV(TAG, "Fan status: min %d, target %d, rpm %d", duty_min, duty_target, rpm);
      break;
    }
#endif // USE_SENSOR

    case R48xx_DATA_STATUS_FLAGS:
    {
      uint16_t status_flags_ext = (data[0] << 8) | data[1];
      uint32_t status_flags = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];

#ifdef USE_BINARY_SENSOR
      bool input_power_failure = status_flags & (1 << 29);
      this->publish_sensor_state_(this->ac_present_binary_sensor_, !input_power_failure);
#endif // USE_BINARY_SENSOR
      break;
    }

    default:
      ESP_LOGV(TAG, "Unknown status value %03x: %02x %02x %02x %02x %02x %02x", register_id, data[0], data[1], data[2], data[3], data[4], data[5]);
      break;
  }
}

void HuaweiR4850Component::handle_control_update_(uint8_t error_type, uint16_t register_id, std::vector<uint8_t> &data)
{
  if (error_type == 0) {
    for (auto &input : this->registered_inputs_) {
      input->handle_update(register_id, data);
    }
    ESP_LOGD(TAG, "Value %03x set OK: %02x %02x %02x %02x %02x %02x", register_id, data[0], data[1], data[2], data[3], data[4], data[5]);
  } else {
    for (auto &input : this->registered_inputs_) {
      input->handle_error(register_id, data);
    }
    ESP_LOGW(TAG, "Value %03x set error: %d", register_id, error_type);
  }
}

void HuaweiR4850Component::handle_elabel_(bool incomplete, uint16_t register_id, std::vector<uint8_t> &data)
{
  // Compose the full response string until complete
  raw_elabel_response_ += std::string(data.cbegin(), data.cend());

  if (!incomplete) {
    ESP_LOGI(TAG, "E-Label response received, populating sensors");
    ELabelResponse elabel_response = parse_elabel_response(raw_elabel_response_);
    raw_elabel_response_.clear();

#ifdef ESPHOME_LOG_HAS_DEBUG
    for (auto const &[key, value] : elabel_response) {
      ESP_LOGD(TAG, "  %s: %s", key.c_str(), value.c_str());
    }
#endif // ESPHOME_LOG_HAS_DEBUG

#ifdef USE_TEXT_SENSOR
    std::map<std::string, text_sensor::TextSensor*> sensor_mappings = {
      {"BoardType", board_type_text_sensor_},
      {"BarCode", serial_number_text_sensor_},
      {"Item", item_text_sensor_},
      {"Model", model_text_sensor_},
    };

    for (auto const &[key, sensor] : sensor_mappings) {
      if (elabel_response.contains(key)) {
        this->publish_sensor_state_(sensor, elabel_response[key].c_str());
      }
    }
#endif // USE_TEXT_SENSOR
    has_received_elabel_response_ = true;
  }
}

uint32_t HuaweiR4850Component::canid_pack_(uint8_t addr, uint8_t command, bool src_controller, bool incomplete) {
  uint32_t id = 0x1080007E; // proto ID, group mask, HW/SW id flag already set
  id |= (uint32_t)(addr & 0x7F) << 16; // 7 bit PSU address (0 = broadcast, 1 = first, ...)
  id |= (uint32_t)command << 8; // command id
  id |= (uint32_t)src_controller << 7; // msg source (0 = PSU, 1 = controller)
  id |= (uint32_t)incomplete; // last message marker (0 = finished, always 0 in requests)
  return id;
}

void HuaweiR4850Component::canid_unpack_(uint32_t canId, uint8_t *addr, uint8_t *command, bool *src_controller, bool *incomplete) {
  *addr =           (canId & 0x007F0000) >> 16;
  *command =        (canId & 0x0000FF00) >> 8;
  *src_controller = (canId & 0x00000080) >> 7;
  *incomplete =     (canId & 0x00000001);
}

}  // namespace huawei_r4850
}  // namespace esphome
