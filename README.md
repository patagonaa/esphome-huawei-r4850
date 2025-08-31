# esphome-huawei-r4850

[![License][license-shield]](LICENSE)

ESPHome component to control and read values from Huawei R48xx power supplies via CAN bus.

Fork of [mb-software/esphome-huawei-r4850](https://github.com/mb-software/esphome-huawei-r4850).

## Requirements
This component is tested and verified to work on ESP32 using the `esp32_can` platform.  
In addition to the ESP32 board, a CAN transceiver like the SN65HVD230 is required. These can be wired directly to the 3.3V GPIO and supply pins of the ESP32 board.  
The component has also been tested with the `mcp2515` platform, but due to ESPHome limits (no interrupts, no TX queue, no RX filtering), it's almost guaranteed sensor updates and control messages will be lost, making it unreliable.

## Configuration

### CAN bus

```yaml
canbus:
  - platform: esp32_can
    id: can
    tx_pin: GPIO14
    rx_pin: GPIO27
    use_extended_id: true
    can_id: 0
    bit_rate: 125kbps
    rx_queue_len: 32 # recommended especially with lots of entities as can messages can be lost otherwise
```

### Main component

```yaml
huawei_r4850:
  - id: huawei_r4850_1
    canbus_id: can
    update_interval: 5s
    resend_interval: 5s # resend number/switch values in case the PSU has been disconnected
    psu_address: 1 # 1 = first PSU, 2 = second, ...
    psu_max_current: 53.5 # ~53.5 for R4850G6, ~42.6 for R4830S1
```

- **id**: ID of this component
- **canbus_id**: ID of the [CAN Bus component](https://esphome.io/components/canbus/) the PSU is attached to
- **update_interval** ([Time](https://esphome.io/guides/configuration-types#config-time), Default `5s`): Update interval for sensors
- **resend_interval** ([Time](https://esphome.io/guides/configuration-types#config-time), Optional): Interval for numbers and switches to resend their state (so state is consistent even with CAN / PSU disconnects)
- **psu_address** (int, Required): Address of the PSU (1 = first PSU, 2 = second, ...)
- **psu_max_current** (float, Default `53.5`): Max current rating of the PSU (~53.5 for R4850G6, ~42.6 for R4830S1).  
  If `output_current_setpoint` != `max_output_current`, Max current vs. actual current has to be calculated / calibrated.

### Sensors

```yaml
sensor:
  - platform: huawei_r4850
    huawei_r4850_id: huawei_r4850_1
    output_voltage:
      name: Output voltage
    output_current:
      name: Output current
    # [...]
```

- **huawei_r4850_id**: ID of the main component (required if there are multiple) 
- **input_voltage**: AC input voltage
- **input_frequency**: AC input frequency
- **input_current**: AC input current
- **input_power**: AC input power
- **input_temp**: Input / ambient temperature
- **efficiency**: Efficiency (%)
- **output_voltage**: DC output voltage
- **output_current**: DC output current
- **output_current_setpoint**: DC output current limit (based on DC and AC current limits and power limit)
- **output_power**: DC output power
- **output_temp**: Output / PCB temperature
- **fan_duty_cycle_min**: Current minimum fan duty cycle (temperature dependent)
- **fan_duty_cycle_target**: Current target fan duty cycle)
- **fan_rpm**: Fan RPM

### Numbers

```yaml
number:
  - platform: huawei_r4850
    huawei_r4850_id: huawei_1
    output_voltage:
      name: Set output voltage
    output_voltage_default:
      name: Set default output voltage
    max_output_current:
      name: Set output current limit
    max_output_current_default:
      name: Set default output current limit
    max_ac_current:
      name: Set AC input current limit
    fan_duty_cycle:
      name: Set fan duty cycle
```

If setting one of these values causes the number input to reset, the value was out of range for the PSU. In that case it makes sense to limit the value range using `min_value` and `max_value` on the number.

- **huawei_r4850_id**: ID of the main component (required if there are multiple) 
- **output_voltage**: Output voltage
- **max_output_current**: Output current limit
- **output_voltage_default**: Default output voltage
- **max_output_current_default**: Default output current limit
- **max_ac_current**: AC input current limit
- **fan_duty_cycle**: PSU set fan duty cycle (0 = auto)

Some of these values are saved persistently to the PSU, some additionally only apply when the PSU loses CAN connection:

| Field                      | Persistent | Used while connected | Used while disconnected |
| -------------------------- | :--------: | :------------------: | :---------------------: |
| output_voltage             |            |          X           |                         |
| max_output_current         |            |          X           |                         |
| output_voltage_default     |     X      |                      |            X            |
| max_output_current_default |     X      |                      |            X            |
| max_ac_current             |     X      |          X           |            X            |
| fan_duty_cycle             |            |          X           |                         |


### Switches

```yaml
switch:
  - platform: huawei_r4850
    huawei_r4850_id: huawei_r4850_1
    standby:
      name: Standby
    fan_speed_max:
      name: Fan speed max
```

- **huawei_r4850_id**: ID of the main component (required if there are multiple) 
- **standby**: PSU standby (disables the DC output)
- **fan_speed_max**: If enabled, forces the fan to full speed (even when the PSU would turn the fan off, which it does when AC input current limit is hit (and set to a low value like 5A) and temperature is <65°C)


### Example config

A full example containing most of these settings can be found in [huawei_r4850.yaml](./huawei_r4850.yaml)


[license-shield]: https://img.shields.io/github/license/patagonaa/esphome-huawei-r4850.svg?style=for-the-badge
