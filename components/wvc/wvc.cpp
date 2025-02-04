#include "wvc.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace wvc {

static const char *const TAG = "wvc";
static uint8_t expected_start_byte = 0x00;
static size_t expected_length = 0;
static unsigned long last_device_query_time = millis();
static unsigned long last_byte_time = millis();


void WVCComponent::setup() {
	publish_state_once_(serial_number_text_sensor_, inverter_sn_);
	publish_state_once_(hardware_revision_text_sensor_, inverter_type_);
}

void WVCComponent::loop() {
	static std::string buffer;
	if (millis() - last_device_query_time <= 1000) {
		return;
	}

	if (turnoff && millis() - last_device_query_time <= 10000) {
		return;
	}

	if (turnoff && millis() - last_device_query_time >= 10000) {
        	turnoff = false;
		return;
	}

	if (turnon && millis() - last_device_query_time <= 60000) {
		return;
		}

	if (turnon && millis() - last_device_query_time >= 60000) {
		turnon = false;
		return;
	}

	if (!waiting_for_response && millis() - last_device_query_time >= throttle_) {
		uint8_t raw_data_1[] = {0xF2, 0x00, 0x00, 0x65, 0xFD, 0x08, 0x08, 0xF9};
		uint8_t raw_data_2[] = {0xF5, 0xFD, 0x08, 0x08, 0xF9, 0xE9, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68};
		if (inverter_type_ == "R2") {
			raw_data_1[1] = static_cast<uint8_t>(strtol(inverter_sn_.substr(0, 2).c_str(), nullptr, 16));
			raw_data_1[2] = static_cast<uint8_t>(strtol(inverter_sn_.substr(2, 2).c_str(), nullptr, 16));
			send_command(raw_data_1, sizeof(raw_data_1), 0xF2, 19, inverter_sn_);
		} 


		if (inverter_type_ == "R3") {
			raw_data_2[9] = static_cast<uint8_t>(strtol(inverter_sn_.substr(0, 2).c_str(), nullptr, 16));
			raw_data_2[10] = static_cast<uint8_t>(strtol(inverter_sn_.substr(2, 2).c_str(), nullptr, 16));
			raw_data_2[11] = static_cast<uint8_t>(strtol(inverter_sn_.substr(4, 2).c_str(), nullptr, 16));
			raw_data_2[12] = static_cast<uint8_t>(strtol(inverter_sn_.substr(6, 2).c_str(), nullptr, 16));
			if (expected_start_byte == 0xF4){
				raw_data_2[0] = 0xF4;
				raw_data_2[13] = 0x66;
				//{0xF3, 0xFD, 0x08, 0x08, 0xF9, 0xE9, 0x1B, 0x00, 0x00, 0x60, 0x00, 0x0F, 0x15, 0x66};
				send_command(raw_data_2, sizeof(raw_data_2), 0xF3, sizeof(raw_data_2), inverter_sn_);
				ESP_LOGW(TAG, "Turning on %s", inverter_sn_.c_str());
				turnon = true;
				return;
			}
			send_command(raw_data_2, sizeof(raw_data_2), 0xF5, 28, inverter_sn_);
		}
	}
	
	while (available() > 0) {
		char c = read();
		buffer += c;
		//last_byte_time = millis();
		if (buffer.size() >= expected_length) {
			if (buffer[0] == expected_start_byte) {
				ESP_LOGD(TAG, "Received response from inverter: %s", inverter_sn_.c_str());
				parse_response(buffer);
				buffer.clear();
				waiting_for_response = false;
				return;
			}
		}
	}
	
	if (waiting_for_response && millis() - last_device_query_time >= throttle_) {
		ESP_LOGW(TAG, "Response timeout: %hhx, %s", buffer[0], inverter_sn_.c_str());
		buffer.clear();
		waiting_for_response = false;
	}
}

void WVCComponent::send_command(uint8_t *command, size_t length, uint8_t expected_start_byte_, size_t expected_length_, const std::string &device_id) {
	write_array(command, length);
	expected_start_byte = expected_start_byte_;
	expected_length = expected_length_;
	waiting_for_response = true;
	ESP_LOGD(TAG, "Sent command %hhX, size: %d, expected lenght: %d to inverter %s", expected_start_byte, length, expected_length, device_id.c_str());
	last_device_query_time = millis();
}

void WVCComponent::parse_response(const std::string &response) {
	if (response.length() == 19) {
		float VAC = static_cast<uint16_t>((response[13] << 8) | response[12]) * 0.88;

		float VDC = static_cast<uint16_t>((response[9] << 8) | response[8]) * (108.506 / 1023);

		float AAC = static_cast<uint16_t>((response[11] << 8) | response[10]) * (9.71 / 1023);

		float ADC = static_cast<uint16_t>((response[7] << 8) | response[6]) * (70.5705 / 1023);

		int16_t TEMP = temp_r2_lookup(static_cast<int16_t>(response[14]));

		float ACW = VAC * AAC;

		float DCW = VDC * ADC;

		float EFF = (DCW > 0) ? (ACW / DCW) : 0;

		if (EFF < 1) {
			EFF = round(EFF * 100);
		} else {
			EFF = 0;
		}

		if (ACW > 2000) {
			ESP_LOGW(TAG, "ACW exceeds 2000");//, ignoring response");
			//return;
		}


		if (vac_sensor_) vac_sensor_->publish_state(VAC);
		if (aac_sensor_) aac_sensor_->publish_state(AAC);
		if (vdc_sensor_) vdc_sensor_->publish_state(VDC);
		if (adc_sensor_) adc_sensor_->publish_state(ADC);
		if (eff_sensor_) eff_sensor_->publish_state(EFF);
		if (dcw_sensor_) dcw_sensor_->publish_state(DCW);
		if (acw_sensor_) acw_sensor_->publish_state(ACW);
		if (temperature_sensor_) temperature_sensor_->publish_state(TEMP);
		ESP_LOGD(TAG, "Parsed Response: VAC: %.2f, VDC: %.2f, DCW: %.2f, ADC: %.2f. Temp: %d", VAC, VDC, DCW, ADC, TEMP);
	}

	if (response.length() == 28) {
		//ESP_LOGD(TAG, "R3?");
		float VAC = static_cast<int16_t>((response[16] << 8) | response[17]) / 100.0; 
		float VDC = static_cast<int16_t>((response[14] << 8) | response[15]) / 100.0;
		float ADC = static_cast<int16_t>((response[18] << 8) | response[19]) / 100.0;
		float ACW = static_cast<int16_t>((response[24] << 8) | response[25]) / 10;
		int TEMP = static_cast<int16_t>((response[26] << 8) | response[27]) / 100;
		float AAC = ACW/VAC;
		float DCW = VDC * ADC;
		float EFF = (ACW/DCW)*100.0;
		ESP_LOGD(TAG, "Parsed F5 Response: VAC: %.2f, VDC: %.2f, DCW: %.2f, ADC: %.2f. Temp: %d", VAC, VDC, DCW, ADC, TEMP);
		if (ADC <= 0 || VDC < 15) {
			ESP_LOGW(TAG, "ADC <= 0 or VDC < 15, sending retry command 0xF4");
			uint8_t retry_command[] = {0xF4, 0xFD, 0x08, 0x08, 0xF9, 0xE9, 0x1B, 0x00, 0x00, 0x60, 0x00, 0x0F, 0x15, 0x67};
			send_command(retry_command, sizeof(retry_command), 0xF4, sizeof(retry_command), inverter_sn_);
			ESP_LOGW(TAG, "Turning off %s", inverter_sn_.c_str());
			turnoff = true;
			return;
		}
		if (vac_sensor_) vac_sensor_->publish_state(VAC);
		if (aac_sensor_) aac_sensor_->publish_state(AAC);
		if (vdc_sensor_) vdc_sensor_->publish_state(VDC);
		if (adc_sensor_) adc_sensor_->publish_state(ADC);
		if (eff_sensor_) eff_sensor_->publish_state(EFF);
		if (dcw_sensor_) dcw_sensor_->publish_state(DCW);
		if (acw_sensor_) acw_sensor_->publish_state(ACW);
		if (temperature_sensor_) temperature_sensor_->publish_state(TEMP);
	}
}

void WVCComponent::set_inverter_sn(const std::string &sn) {
	inverter_sn_ = sn;
}

void WVCComponent::set_inverter_type(const std::string &type) {
	inverter_type_ = type;
}

void WVCComponent::publish_state_once_(text_sensor::TextSensor *text_sensor, const std::string &state) {
	if (text_sensor == nullptr)
		return;
	if (text_sensor->has_state())
		return;
	text_sensor->publish_state(state);
}

int16_t WVCComponent::temp_r2_lookup(int16_t value) {
	switch (value) {
		case 0:
			return 86;
		case 1:
			return 85;
		case 2:
			return 84;
		case 3:
			return 83;
		case 4:
			return 82;
		case 5:
			return 81;
		case 6:
			return 80;
		case 7:
			return 79;
		case 8:
			return 78;
		case 9:
			return 77;
		case 10:
			return 76;
		case 11:
			return 75;
		case 12:
			return 74;
		case 13:
			return 73;
		case 14:
			return 72;
		case 15:
			return 71;
		case 16:
			return 70;
		case 17:
			return 69;
		case 18:
			return 68;
		case 19:
			return 66;
		case 20:
			return 65;
		case 21:
			return 64;
		case 22:
			return 63;
		case 23:
			return 62;
		case 24:
			return 61;
		case 25:
			return 60;
		case 26:
			return 59;
		case 27:
			return 58;
		case 28:
			return 57;
		case 29:
			return 56;
		case 30:
			return 55;
		case 31:
			return 54;
		case 32:
			return 54;
		case 33:
			return 53;
		case 34:
			return 52;
		case 35:
			return 51;
		case 36:
			return 50;
		case 37:
			return 50;
		case 38:
			return 49;
		case 39:
			return 48;
		case 40:
			return 48;
		case 41:
			return 47;
		case 42:
			return 47;
		case 43:
			return 47;
		case 44:
			return 46;
		case 45:
			return 46;
		case 46:
			return 45;
		case 47:
			return 44;
		case 48:
			return 43;
		case 49:
			return 43;
		case 50:
			return 43;
		case 51:
			return 42;
		case 52:
			return 41;
		case 53:
			return 41;
		case 54:
			return 41;
		case 55:
			return 40;
		case 56:
			return 40;
		case 57:
			return 39;
		case 58:
			return 39;
		case 59:
			return 39;
		case 60:
			return 38;
		case 61:
			return 38;
		case 62:
			return 37;
		case 63:
			return 37;
		case 64:
			return 36;
		case 65:
			return 36;
		case 66:
			return 36;
		case 67:
			return 35;
		case 68:
			return 35;
		case 69:
			return 34;
		case 70:
			return 34;
		case 71:
			return 34;
		case 72:
			return 33;
		case 73:
			return 33;
		case 74:
			return 33;
		case 75:
			return 33;
		case 76:
			return 32;
		case 77:
			return 32;
		case 78:
			return 32;
		case 79:
			return 31;
		case 80:
			return 31;
		case 81:
			return 31;
		case 82:
			return 30;
		case 83:
			return 30;
		case 84:
			return 30;
		case 85:
			return 30;
		case 86:
			return 29;
		case 87:
			return 29;
		case 88:
			return 29;
		case 89:
			return 28;
		case 90:
			return 28;
		case 91:
			return 28;
		case 92:
			return 28;
		case 93:
			return 27;
		case 94:
			return 27;
		case 95:
			return 27;
		case 96:
			return 27;
		case 97:
			return 26;
		case 98:
			return 26;
		case 99:
			return 26;
		case 100:
			return 26;
		case 101:
			return 26;
		case 102:
			return 26;
		case 103:
			return 25;
		case 104:
			return 25;
		case 105:
			return 25;
		case 106:
			return 25;
		case 107:
			return 25;
		case 108:
			return 25;
		case 109:
			return 24;
		case 110:
			return 24;
		case 111:
			return 24;
		case 112:
			return 24;
		case 113:
			return 24;
		case 114:
			return 24;
		case 115:
			return 23;
		case 116:
			return 23;
		case 117:
			return 23;
		case 118:
			return 23;
		case 119:
			return 23;
		case 120:
			return 23;
		case 121:
			return 22;
		case 122:
			return 22;
		case 123:
			return 22;
		case 124:
			return 22;
		case 125:
			return 22;
		case 126:
			return 21;
		case 127:
			return 21;
		case 128:
			return 21;
		case 129:
			return 21;
		case 130:
			return 21;
		case 131:
			return 20;
		case 132:
			return 20;
		case 133:
			return 20;
		case 134:
			return 20;
		case 135:
			return 19;
		case 136:
			return 19;
		case 137:
			return 19;
		case 138:
			return 19;
		case 139:
			return 18;
		case 140:
			return 18;
		case 141:
			return 18;
		case 142:
			return 18;
		case 143:
			return 17;
		case 144:
			return 17;
		case 145:
			return 17;
		case 146:
			return 17;
		case 147:
			return 16;
		case 148:
			return 16;
		case 149:
			return 16;
		case 150:
			return 16;
		case 151:
			return 15;
		case 152:
			return 15;
		case 153:
			return 15;
		case 154:
			return 14;
		case 155:
			return 14;
		case 156:
			return 14;
		case 157:
			return 13;
		case 158:
			return 13;
		case 159:
			return 13;
		case 160:
			return 12;
		case 161:
			return 12;
		case 162:
			return 12;
		case 163:
			return 11;
		case 164:
			return 11;
		case 165:
			return 11;
		case 166:
			return 10;
		case 167:
			return 10;
		case 168:
			return 10;
		case 169:
			return 9;
		case 170:
			return 9;
		case 171:
			return 9;
		case 172:
			return 8;
		case 173:
			return 8;
		case 174:
			return 8;
		case 175:
			return 7;
		case 176:
			return 7;
		case 177:
			return 7;
		case 178:
			return 6;
		case 179:
			return 6;
		case 180:
			return 6;
		case 181:
			return 5;
		case 182:
			return 5;
		case 183:
			return 5;
		case 184:
			return 4;
		case 185:
			return 4;
		case 186:
			return 4;
		case 187:
			return 3;
		case 188:
			return 3;
		case 189:
			return 2;
		case 190:
			return 2;
		case 191:
			return 1;
		case 192:
			return 1;
		case 193:
			return 0;
		case 194:
			return 0;
		case 195:
			return -1;
		case 196:
			return -1;
		case 197:
			return -2;
		case 198:
			return -2;
		case 199:
			return -3;
		case 200:
			return -3;
		case 201:
			return -4;
		case 202:
			return -4;
		case 203:
			return -5;
		case 204:
			return -5;
		case 205:
			return -6;
		case 206:
			return -6;
		case 207:
			return -7;
		case 208:
			return -7;
		case 209:
			return -8;
		case 210:
			return -8;
		case 211:
			return -9;
		case 212:
			return -9;
		case 213:
			return -10;
		case 214:
			return -10;
		case 215:
			return -11;
		case 216:
			return -11;
		case 217:
			return -12;
		case 218:
			return -12;
		case 219:
			return -13;
		case 220:
			return -13;
		case 221:
			return -14;
		case 222:
			return -14;
		case 223:
			return -15;
		case 224:
			return -15;
		case 225:
			return -16;
		case 226:
			return -16;
		case 227:
			return -17;
		case 228:
			return -17;
		case 229:
			return -18;
		case 230:
			return -18;
		case 231:
			return -19;
		case 232:
			return -19;
		case 233:
			return -20;
		case 234:
			return -20;
		case 235:
			return -21;
		case 236:
			return -21;
		case 237:
			return -22;
		case 238:
			return -22;
		case 239:
			return -23;
		case 240:
			return -24;
		case 241:
			return -25;
		case 242:
			return -26;
		case 243:
			return -27;
		case 244:
			return -28;
		case 245:
			return -29;
		case 246:
			return -30;
		case 247:
			return -31;
		case 248:
			return -32;
		case 249:
			return -33;
		case 250:
			return -34;
		case 251:
			return -35;
		case 252:
			return -36;
		case 253:
			return -37;
		case 254:
			return -38;
		case 255:
			return -39;
		default:
			return 0;
	}
}

void WVCComponent::set_vac_sensor(sensor::Sensor *sensor) { vac_sensor_ = sensor; }
void WVCComponent::set_aac_sensor(sensor::Sensor *sensor) { aac_sensor_ = sensor; }
void WVCComponent::set_vdc_sensor(sensor::Sensor *sensor) { vdc_sensor_ = sensor; }
void WVCComponent::set_adc_sensor(sensor::Sensor *sensor) { adc_sensor_ = sensor; }
void WVCComponent::set_acw_sensor(sensor::Sensor *sensor) { acw_sensor_ = sensor; }
void WVCComponent::set_dcw_sensor(sensor::Sensor *sensor) { dcw_sensor_ = sensor; }
void WVCComponent::set_eff_sensor(sensor::Sensor *sensor) { eff_sensor_ = sensor; }
void WVCComponent::set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }

}  // namespace wvc
}  // namespace esphome
