// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "motor_owner.hpp"
#include <cstring>

namespace example_control {
// Require magic so unrelated MSP requests cannot operate the motor.
constexpr std::uint8_t kCommand = 240;
enum class Operation : std::uint8_t { Status, Arm, Run, Stop };
constexpr unsigned kRequestSize = 7, kReplySize = 24;

struct Context { MotorOwner& motors; esc_passthrough_t *session = nullptr; };

inline void put16(std::uint8_t *out, unsigned value) {
    out[0] = value & 255; out[1] = (value >> 8) & 255;
}
inline void put32(std::uint8_t *out, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) out[i] = (value >> (i * 8)) & 255;
}

inline bool command(void *ctx, std::uint8_t id, const std::uint8_t *payload,
                    std::size_t size, std::uint8_t *reply, std::size_t capacity,
                    std::size_t *reply_size, bool *error_reply) {
    if (id != kCommand) return false;
    *reply_size = 0;
    *error_reply = true;
    if (size != kRequestSize || std::memcmp(payload, "DSH1", 4) || capacity < kReplySize)
        return true;
    auto& control = *static_cast<Context*>(ctx);
    unsigned value = payload[5] | static_cast<unsigned>(payload[6]) << 8;
    esp_err_t error = ESP_ERR_INVALID_ARG;
    if (payload[4] == static_cast<unsigned>(Operation::Run)) error = control.motors.run(value);
    else if (value == 0) {
        switch (static_cast<Operation>(payload[4])) {
        case Operation::Status: control.motors.poll(); error = ESP_OK; break;
        case Operation::Arm: error = control.motors.arm(); break;
        case Operation::Stop: error = control.motors.stop(); break;
        default: break;
        }
    }
    std::memset(reply, 0, kReplySize);
    std::memcpy(reply, "DSH1", 4);
    reply[4] = static_cast<std::uint8_t>(control.motors.state());
    reply[5] = esc_passthrough_is_held(control.session);
    put16(reply + 6, control.motors.throttle());
    put32(reply + 8, control.motors.remaining_ms());
    put32(reply + 12, static_cast<std::uint32_t>(error));
    put32(reply + 16, static_cast<std::uint32_t>(control.motors.last_error()));
    esc_session_end_t end;
    if (esc_passthrough_last_end(control.session, &end)) {
        reply[20] = static_cast<std::uint8_t>(end.reason) + 1;
        reply[21] = end.needs_reboot_mask;
        reply[22] = end.io_released;
    }
    reply[23] = 1; // Wire format version.
    *reply_size = kReplySize;
    *error_reply = error != ESP_OK;
    return true;
}
}
