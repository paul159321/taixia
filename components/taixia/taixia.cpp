#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "taixia.h"

namespace esphome {
namespace taixia {

static const char *const TAG = "taixia";

void TaiXia::dump_config() {
  ESP_LOGCONFIG(TAG, "TaiXIA (All-in-One Version):");
  ESP_LOGCONFIG(TAG, "    SA_ID: %x", this->sa_id_);
}

uint8_t TaiXia::checksum(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x0;
  while (len--) crc ^= *data++;
  return crc;
}

std::string TaiXia::parse_string_(const uint8_t *data, uint8_t len) {
  std::string s;

  for (uint8_t i = 0; i < len; i++) {
    uint8_t c = data[i];

    // 結束條件
    if (c == 0x00) break;

    // 過濾非 ASCII 可印字元
    if (c >= 32 && c <= 126) {
      s += (char)c;
    }
  }

  // 去尾巴空白
  while (!s.empty() && s.back() == ' ') {
    s.pop_back();
  }

  return s;
}

bool TaiXia::write_command_(const uint8_t *command,
                           uint8_t *response,
                           uint8_t len,
                           uint8_t max_rlen,
                           uint32_t timeout) {
  // 1. 清空 RX buffer
  uint32_t clear_start = millis();
  while (millis() - clear_start < 20) {
    while (this->available()) this->read();
    yield();
  }

  // 2. 發送指令
  this->write_array(command, len);

  if (response == nullptr) return true;

  // 3. 等第一個 byte（長度）
  uint32_t start = millis();
  while (this->available() == 0) {
    if (millis() - start > timeout) return false;
    delay(1);
  }

  // 4. 讀取長度
  uint8_t total_len = this->read();
  response[0] = total_len;

  if (total_len < 4 || total_len > max_rlen) {
    return false; // 防呆
  }

  // 5. 讀剩餘資料
  uint8_t index = 1;
  uint32_t byte_start = millis();
  uint32_t last_yield = millis();

  while (index < total_len) {
    if (this->available()) {
      response[index++] = this->read();
      byte_start = millis();
    } else {
      delay(0);  // ESP8266 必備
    }

    if (millis() - last_yield > 10) {
      yield();
      last_yield = millis();
    }

    if (millis() - byte_start > 300) {
      return false;
    }
  }

  // 6. checksum 驗證
  uint8_t cs = this->checksum(response, total_len - 1);
  return (cs == response[total_len - 1]);
}

// overload
bool TaiXia::write_command_(const uint8_t *command, uint8_t *response,
                           uint8_t len, uint8_t rlen) {
  return this->write_command_(command, response, len, rlen, 1000);
}

void TaiXia::get_info_() {
  auto fetch = [&](uint8_t svc, auto func) {
    uint8_t cmd[6] = {0x06, 0x00, svc, 0xFF, 0xFF, 0x00};
    cmd[5] = this->checksum(cmd, 5);

    uint8_t res[64];  // ⚠ 必須夠大（Services 很長）

    bool ok = false;

    // 🔁 retry（ESP8266 很重要）
    for (int i = 0; i < 3; i++) {
      if (this->write_command_(cmd, res, 6, sizeof(res), 1000)) {
        ok = true;
        break;
      }
      delay(200);
    }

    if (!ok) {
      Serial.println("TaiXia fetch failed");
      delay(500);
      return;
    }

    uint8_t total_len = res[0];

    // Debug（可開）
    Serial.print("RX: ");
    for (int i = 0; i < total_len; i++) {
      Serial.printf("%02X ", res[i]);
    }
    Serial.println();

    this->buffer_.assign(res, res + total_len);

    func();

    delay(500);  // ⚠ 裝置需要休息
    yield();
  };

  // ===== Version =====
  fetch(SERVICE_ID_READ_VERSION, [&]() {
    if (this->version_textsensor_) {
      this->version_textsensor_->publish_state(
        format_hex_pretty(this->buffer_[3]) + "." +
        format_hex_pretty(this->buffer_[4]));
    }
  });

  // ===== SA ID =====
  fetch(SERVICE_ID_READ_SA_ID, [&]() {
    if (this->sa_id_textsensor_) {
      this->sa_id_textsensor_->publish_state(
        format_hex_pretty(this->buffer_[3]) +
        format_hex_pretty(this->buffer_[4]));
    }
    this->sa_id_ = this->buffer_[4];
  });

  // ===== Brand（變長字串）=====
  fetch(SERVICE_ID_READ_BRAND, [&]() {
    if (this->brand_textsensor_) {
      uint8_t data_len = this->buffer_[0] - 4;

      std::string brand = this->parse_string_(
        &this->buffer_[3],
        data_len
      );
      this->brand_textsensor_->publish_state(brand);
    }
  });

  // ===== Model（變長字串）=====
  fetch(SERVICE_ID_READ_MODEL, [&]() {
    if (this->model_textsensor_) {
      uint8_t data_len = this->buffer_[0] - 4;

      std::string model = this->parse_string_(
        &this->buffer_[3],
        data_len
      );
      this->model_textsensor_->publish_state(model);
    }
  });

  // ===== Services（raw hex）=====
  fetch(SERVICE_ID_READ_SERVICES, [&]() {
    std::string hex;

    for (size_t i = 3; i < this->buffer_.size() - 1; i++) {
      char buf[4];
      sprintf(buf, "%02X ", this->buffer_[i]);
      hex += buf;
    }

    if (this->services_textsensor_) {
      this->services_textsensor_->publish_state(hex);
    }
  });
}

void TaiXia::setup() {
  uint8_t res[8];
  uint8_t cmd[6] = {0x06, 0x00, SERVICE_ID_REGISTER, 0xFF, 0xFF, 0x00};
  cmd[5] = this->checksum(cmd, 5);
  if (this->write_command_(cmd, res, 6, 8)) {
    if (this->sa_id_ == 0) this->sa_id_ = res[6] << 8 | res[7];
  } else {
    this->get_info_();
  }
}

void TaiXia::switch_command(uint8_t sa_id, uint8_t service_id, bool onoff) {
  uint8_t response[6], cmd[6] = {0x06, sa_id, (uint8_t)(WRITE | service_id), 0x00, (uint8_t)(onoff ? 1 : 0), 0x00};
  cmd[5] = this->checksum(cmd, 5);
  this->write_command_(cmd, response, 6, 6);
}

void TaiXia::set_number(uint8_t sa_id, uint8_t service_id, float value) {
  uint8_t response[6], cmd[6] = {0x06, sa_id, (uint8_t)(WRITE | service_id), (uint8_t)((int(value) >> 8) & 0xFF), (uint8_t)(int(value) & 0xFF), 0x00};
  cmd[5] = this->checksum(cmd, 5);
  this->write_command_(cmd, response, 6, 6);
}

bool TaiXia::read_climate_status_() {
  uint8_t resp[6], combined[255], i = 3;
  uint8_t svcs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x0A, 0x0B, 0x10};
  for (uint8_t s : svcs) {
    uint8_t cmd[6] = {0x06, this->sa_id_, s, 0xFF, 0xFF, 0x00};
    cmd[5] = this->checksum(cmd, 5);
    if (this->write_command_(cmd, resp, 6, 6)) {
      combined[i++] = resp[2]; combined[i++] = resp[3]; combined[i++] = resp[4];
    }
    yield();
  }
  combined[0] = i + 1; combined[i] = this->checksum(combined, i);
  this->buffer_.assign(combined, combined + i + 1);
  for (auto &l : this->listeners_) l->on_response(this->sa_id_, this->buffer_);
  return true;
}

bool TaiXia::read_sa_status() { 
  // 如果是冷氣 (1) 或 除濕機 (4)，都執行讀取
  if (this->sa_id_ == 1 || this->sa_id_ == 4) {
    return this->read_climate_status_(); 
  }
  return false;
}
void TaiXia::button_command(uint8_t sa_id, uint8_t service_id, uint8_t value) {
  if (service_id == 0x00) this->get_info_();
  else {
    uint8_t response[6], cmd[6] = {0x06, sa_id, (uint8_t)(WRITE | service_id), 0x00, value, 0x00};
    cmd[5] = this->checksum(cmd, 5);
    this->write_command_(cmd, response, 6, 6);
  }
}

void TaiXia::readline(bool handle_response) {
  if (!available()) return;
  uint8_t len = peek();
  if (len < 2 || len > 254) { read(); return; }
  std::vector<uint8_t> temp;
  uint32_t start = millis();
  while (temp.size() < len) {
    if (millis() - start > 1000) return;
    if (available()) temp.push_back(read());
    yield();
  }
  if (handle_response && (checksum(temp.data(), len - 1) == temp[len - 1])) {
    this->buffer_ = temp;
    for (auto &l : listeners_) l->on_response(this->sa_id_, this->buffer_);
  } else {
    this->buffer_ = temp;
  }
}

void TaiXia::loop() { if (available() >= 6) readline(true); }

bool TaiXia::send(uint8_t packet_length, uint8_t type, uint8_t sa, uint8_t svc, uint16_t data) {
  uint8_t f[6] = {packet_length, sa, svc, (uint8_t)(data >> 8), (uint8_t)data, 0};
  f[5] = checksum(f, 5);
  write_array(f, 6);
  return true;
}

void TaiXiaListener::on_response(uint16_t sa, std::vector<uint8_t> &res) { this->handle_response(res); }

}  // namespace taixia
}  // namespace esphome