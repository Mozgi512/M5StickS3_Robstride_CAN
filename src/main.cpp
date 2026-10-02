// M5StickS3 + CAN トランシーバ（Grove）で Robstride を 1 台動かす最小サンプル。
//
//   起動 → Robstride を探す（ID 1〜127 に Type0）→ CSP（位置制御）で励磁
//   → 今の位置を中心に ±90° の正弦波で往復させる。
//   ボタン A（表の大きいボタン）: 停止（脱力）/ 再開
//
// 配線: Grove 白 G9 = CAN TX（トランシーバの TXD 入力へ）、黄 G10 = CAN RX（トランシーバの RXD 出力から）
//       CANH / CANL を Robstride へ。バスの両端に 120Ω。ビットレート 1 Mbps。
#include <M5Unified.h>
#include <driver/twai.h>
#include <math.h>

constexpr int CAN_TX = 9;
constexpr int CAN_RX = 10;
constexpr bool GROVE_5V = true;   // 3.3V 専用のトランシーバを別電源で使うなら false

constexpr float AMP_RAD = 1.57f;     // 振幅 ±90°
constexpr float PERIOD_S = 4.0f;     // 1 往復 4 秒
constexpr float LIMIT_SPD = 6.0f;    // rad/s
constexpr float LIMIT_CUR = 5.0f;    // A
constexpr float LIMIT_TORQUE = 4.0f; // Nm
constexpr float LOC_KP = 5.0f;

// ===== Robstride プライベートプロトコル（拡張ID 29bit） =====
//   ID = 通信タイプ[28:24] | データ領域2[23:8] | 宛先モータID[7:0]
//   データ領域2 の上位バイトにホスト ID（0xFD）を入れる。
//   応答では bit15..8 にモータ ID が入る（下位 8bit に入れて返す個体もあるので両方を見る）。
enum : uint8_t { TYPE_PING = 0, TYPE_ENABLE = 3, TYPE_STOP = 4, TYPE_READ = 17, TYPE_WRITE = 18 };
enum : uint16_t {
  RUN_MODE = 0x7005, LIMIT_TORQUE_IDX = 0x700B, LOC_REF = 0x7016, LIMIT_SPD_IDX = 0x7017,
  LIMIT_CUR_IDX = 0x7018, MECH_POS = 0x7019, LOC_KP_IDX = 0x701E, LIMIT_CUR_OLD_IDX = 0x2019,
};
constexpr uint8_t HOST_ID = 0xFD;

static uint8_t motorId = 0;   // 0 = 見つかっていない
static bool running = false;
static float center = 0, posRad = 0;

static void sendFrame(uint8_t type, uint8_t id, const uint8_t data[8] = nullptr) {
  twai_message_t m = {};
  m.extd = 1;
  m.identifier = (uint32_t(type) << 24) | (uint32_t(HOST_ID) << 16) | id;
  m.data_length_code = 8;
  if (data) memcpy(m.data, data, 8);
  twai_transmit(&m, pdMS_TO_TICKS(10));
}

// Type18: パラメータ書込み。byte0-1 = インデックス（LE）、byte4-7 = 値（LE）
static void writeParam(uint16_t index, float value) {
  uint8_t d[8] = {uint8_t(index), uint8_t(index >> 8)};
  memcpy(d + 4, &value, 4);   // ESP32 はリトルエンディアン
  sendFrame(TYPE_WRITE, motorId, d);
}
static void writeParamU8(uint16_t index, uint8_t value) {
  uint8_t d[8] = {uint8_t(index), uint8_t(index >> 8), 0, 0, value};
  sendFrame(TYPE_WRITE, motorId, d);
}
// Type17: パラメータ読出し要求。値は Type17 の応答で返る（handleRx）
static void requestParam(uint16_t index) {
  uint8_t d[8] = {uint8_t(index), uint8_t(index >> 8)};
  sendFrame(TYPE_READ, motorId, d);
}

// 受信を全部処理する。mech_pos の応答が来たら posRad を更新して true を返す。
static bool handleRx() {
  bool gotPos = false;
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    if (!m.extd || (((m.identifier >> 8) & 0xFF) != motorId && (m.identifier & 0xFF) != motorId)) continue;
    const uint8_t type = (m.identifier >> 24) & 0x1F;
    const uint16_t le = m.data[0] | (m.data[1] << 8), be = m.data[1] | (m.data[0] << 8);
    if (type == TYPE_READ && (le == MECH_POS || be == MECH_POS)) {
      memcpy(&posRad, m.data + 4, 4);
      gotPos = true;
    }
  }
  return gotPos;
}

static bool readPosition() {
  requestParam(MECH_POS);
  for (uint32_t t0 = millis(); millis() - t0 < 300; delay(1))
    if (handleRx()) return true;
  return false;
}

// ID 1〜127 に Type0 を送り、最初に返事をしたモータの ID を返す。
static uint8_t findMotor() {
  for (uint8_t id = 1; id <= 127; id++) {
    sendFrame(TYPE_PING, id);
    twai_message_t m;
    for (uint32_t t0 = millis(); millis() - t0 < 5;) {
      if (twai_receive(&m, pdMS_TO_TICKS(1)) != ESP_OK || !m.extd) continue;
      const uint8_t hi = (m.identifier >> 8) & 0xFF, lo = m.identifier & 0xFF;
      const uint8_t from = hi && hi < 0xFD ? hi : lo;   // 応答の送り主
      if (from == id) return id;
    }
  }
  return 0;
}

// CSP（run_mode = 5）で励磁し、今の位置を中心にする。
static bool enableCsp() {
  const uint8_t clearFault[8] = {1};
  sendFrame(TYPE_STOP, motorId, clearFault); delay(100);   // 停止 + 故障クリア
  writeParamU8(RUN_MODE, 5);                delay(30);
  writeParam(LIMIT_SPD_IDX, LIMIT_SPD);     delay(2);
  writeParam(LIMIT_CUR_IDX, LIMIT_CUR);     delay(2);
  writeParam(LIMIT_CUR_OLD_IDX, LIMIT_CUR); delay(2);   // 古いファームの電流上限
  writeParam(LOC_KP_IDX, LOC_KP);           delay(2);
  writeParam(LIMIT_TORQUE_IDX, LIMIT_TORQUE); delay(2);
  if (!readPosition()) return false;
  center = posRad;
  writeParam(LOC_REF, center);              delay(2);   // 励磁した瞬間に飛ばないよう目標 = 今の位置
  sendFrame(TYPE_ENABLE, motorId);          delay(50);
  writeParamU8(RUN_MODE, 5);                delay(30);
  writeParam(LOC_REF, center);              delay(2);
  return true;
}

static void show(const char* state) {
  char line[32];
  snprintf(line, sizeof(line), "Robstride ID %u", motorId);
  M5.Display.drawString(line, 4, 4);
  M5.Display.drawString(state, 4, 40);
  snprintf(line, sizeof(line), "pos %+.1f deg", posRad * 57.2958f);
  M5.Display.drawString(line, 4, 90);
}

void setup() {
  auto cfg = M5.config();
  cfg.external_imu = false;   // Grove を CAN に使うので I2C 機器を探さない
  cfg.external_rtc = false;
  M5.begin(cfg);
  M5.Ex_I2C.release();
  if (GROVE_5V) {
    M5.Power.setExtOutput(true);   // StickS3 の Grove 5V は PMIC で入れる
    delay(300);                    // トランシーバの電源が立ち上がるまで待つ
  }
  M5.Display.setRotation(1);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextPadding(M5.Display.width());   // 前の文字を背景色で消す

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(CAN_TX), gpio_num_t(CAN_RX), TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK || twai_start() != ESP_OK) {
    show("CAN start NG");
    while (true) delay(1000);
  }

  show("searching...");
  while (!(motorId = findMotor())) {
    Serial.println("Robstride not found (power / CANH-CANL / 120ohm / TX-RX?)");
    show("not found");
    delay(1000);
  }
  Serial.printf("found Robstride ID %u\n", motorId);
  running = enableCsp();
  show(running ? "RUN  (A: stop)" : "enable NG");
}

void loop() {
  M5.update();
  if (M5.BtnA.wasPressed()) {
    if (running) {
      sendFrame(TYPE_STOP, motorId);   // 停止（脱力）
      running = false;
    } else {
      running = enableCsp();
    }
  }

  // 10 ms ごとに目標角 loc_ref [rad] を送る。CSP がこの目標に追従する。
  static uint32_t sendAt = 0, startAt = 0;
  if (!running) startAt = millis();
  if (running && millis() - sendAt >= 10) {
    sendAt = millis();
    const float t = (millis() - startAt) / 1000.0f;
    writeParam(LOC_REF, center + AMP_RAD * sinf(2 * PI * t / PERIOD_S));
  }

  // 100 ms ごとに角度を読んで表示（応答は次の handleRx で受け取る）
  static uint32_t readAt = 0;
  handleRx();
  if (millis() - readAt >= 100) {
    readAt = millis();
    requestParam(MECH_POS);
    show(running ? "RUN  (A: stop)" : "STOP (A: start)");
  }
  delay(1);
}
