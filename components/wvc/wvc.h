#pragma once

#include "esphome.h"

namespace esphome {
namespace wvc {

class WVCComponent : public uart::UARTDevice, public Component {
public:
  void setup() override;
  void loop() override;

  void set_inverter_sn(const std::string &sn);
  void set_inverter_type(const std::string &sn);
  void set_inverter_model(const std::string &type);
  void set_throttle(uint32_t throttle) { this->throttle_ = throttle; }

  void set_vac_sensor(sensor::Sensor *sensor);
  void set_aac_sensor(sensor::Sensor *sensor);
  void set_vdc_sensor(sensor::Sensor *sensor);
  void set_adc_sensor(sensor::Sensor *sensor);
  void set_eff_sensor(sensor::Sensor *sensor);
  void set_acw_sensor(sensor::Sensor *sensor);
  void set_dcw_sensor(sensor::Sensor *sensor);
  void set_temperature_sensor(sensor::Sensor *sensor);
  void set_serial_number_text_sensor(text_sensor::TextSensor *serial_number_text_sensor) {
    serial_number_text_sensor_ = serial_number_text_sensor;
  }
  void set_model_text_sensor(text_sensor::TextSensor *model_text_sensor) {
    model_text_sensor_ = model_text_sensor;
  }
  void set_hardware_revision_text_sensor(text_sensor::TextSensor *hardware_revision_text_sensor) {
    hardware_revision_text_sensor_ = hardware_revision_text_sensor;
  }

 protected:
  void parse_response(const std::string &response);
  void publish_state_once_(text_sensor::TextSensor *text_sensor, const std::string &state);
  float r2_values(std::string type, uint16_t value);
  
 private:
  sensor::Sensor *vac_sensor_{nullptr};
  sensor::Sensor *aac_sensor_{nullptr};
  sensor::Sensor *vdc_sensor_{nullptr};
  sensor::Sensor *adc_sensor_{nullptr};
  sensor::Sensor *eff_sensor_{nullptr};
  sensor::Sensor *dcw_sensor_{nullptr};
  sensor::Sensor *acw_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  text_sensor::TextSensor *hardware_revision_text_sensor_{nullptr};
  text_sensor::TextSensor *serial_number_text_sensor_{nullptr};
  text_sensor::TextSensor *model_text_sensor_{nullptr};
  uart::UARTComponent *uart_;

  bool waiting_for_response = false;
  bool turnoff = false;
  bool turnon = false;

  std::string inverter_sn_;
  std::string inverter_type_;
  std::string inverter_model_;
  uint32_t throttle_{0};  
  void send_command(uint8_t *command, size_t length, uint8_t expected_start_byte_, size_t expected_length_, const std::string &device_id);
};

}  // namespace wvc
}  // namespace esphome
