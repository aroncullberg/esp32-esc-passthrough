// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "esc_passthrough.h"
#include "dshot/config.hpp"
#include "dshot/driver.hpp"
#include "dshot/link.hpp"
#include "dshot/passthrough.hpp"
#include <cstdint>

// All methods run on the parser task.
class MotorOwner {
public:
    enum class State : std::uint8_t { Stopped, Arming, Ready, Running, Configuration, Fault };
    static constexpr unsigned kArmingMs = 5000;

    static dshot::Config config() {
        dshot::Config result;
        result.protocol = dshot::Protocol::DShot300;
        result.signaling = dshot::Signaling::Regular;
        result.frame_hz = 1000;
        return result;
    }

    MotorOwner(dshot::Driver& driver, dshot::Link& link, std::int64_t (*clock_us)())
        : driver_(driver), link_(link), handoff_(driver, {&link}), clock_us_(clock_us) {}

    esc_passthrough_lifecycle_t callbacks() { return {this, acquire_callback, release_callback}; }
    void attach_session(esc_passthrough_t *session) { session_ = session; }

    esp_err_t arm() {
        poll();
        if (state_ == State::Configuration || handoff_.held()) return ESP_ERR_INVALID_STATE;
        if (fault_latched_) {
            esc_session_end_t result;
            if (!session_ || esc_passthrough_is_held(session_)) return ESP_ERR_INVALID_STATE;
            if (esc_passthrough_last_end(session_, &result) &&
                (!result.io_released || result.needs_reboot_mask)) return ESP_ERR_INVALID_STATE;
        }
        close();
        esp_err_t error = link_.begin();
        if (error == ESP_OK) error = link_.set(0);
        if (error == ESP_OK) error = driver_.start();
        if (error != ESP_OK) return fault(error);
        fault_latched_ = false;
        error_ = ESP_OK;
        begin_arming();
        return ESP_OK;
    }

    esp_err_t run(std::uint16_t payload) {
        poll();
        if (state_ != State::Ready && state_ != State::Running) return ESP_ERR_INVALID_STATE;
        if (payload < 48 || payload > 2047) return ESP_ERR_INVALID_ARG; // Commands are excluded.
        esp_err_t error = link_.set(payload);
        if (error != ESP_OK) return fault(error);
        throttle_ = payload;
        state_ = State::Running;
        return ESP_OK;
    }

    esp_err_t stop() {
        poll();
        if (state_ == State::Configuration || state_ == State::Fault) return ESP_ERR_INVALID_STATE;
        if (state_ == State::Stopped) return ESP_OK;
        esp_err_t error = link_.set(0);
        if (error != ESP_OK) return fault(error);
        throttle_ = 0;
        if (state_ == State::Running) state_ = State::Ready;
        return ESP_OK;
    }

    void output_failed() { fault_latched_ = true; error_ = ESP_FAIL; }
    void transport_lost() { fault(ESP_FAIL); }

    void poll() {
        if (state_ == State::Configuration) return;
        if (fault_latched_ && state_ != State::Fault) { fault(error_); return; }
        if (state_ != State::Arming && state_ != State::Ready && state_ != State::Running) return;
        esp_err_t error = driver_.last_error();
        if (error != ESP_OK) { fault(error); return; }
        if (state_ == State::Arming && clock_us_() >= ready_at_us_) state_ = State::Ready;
    }

    State state() const { return state_; }
    std::uint16_t throttle() const { return throttle_; }
    esp_err_t last_error() const { return error_; }
    unsigned remaining_ms() const {
        if (state_ != State::Arming) return 0;
        std::int64_t remaining = ready_at_us_ - clock_us_();
        return remaining > 0 ? static_cast<unsigned>((remaining + 999) / 1000) : 0;
    }

private:
    void close() {
        throttle_ = 0;
        ready_at_us_ = 0;
        driver_.stop();
        link_.end();
        state_ = State::Stopped;
    }

    esp_err_t fault(esp_err_t error) {
        close();
        fault_latched_ = true;
        error_ = error;
        state_ = State::Fault;
        return error;
    }

    void begin_arming() {
        throttle_ = 0;
        ready_at_us_ = clock_us_() + static_cast<std::int64_t>(kArmingMs) * 1000;
        state_ = State::Arming;
    }

    static esp_err_t acquire_callback(void *ctx) {
        auto& owner = *static_cast<MotorOwner*>(ctx);
        owner.throttle_ = 0;
        owner.ready_at_us_ = 0;
        esp_err_t error = owner.handoff_.acquire();
        if (error != ESP_OK) return owner.fault(error);
        owner.state_ = State::Configuration;
        return ESP_OK;
    }

    static esp_err_t release_callback(void *ctx, const esc_session_end_t *result) {
        auto& owner = *static_cast<MotorOwner*>(ctx);
        bool resume = result->reason == ESC_SESSION_NORMAL_EXIT && result->io_released &&
                      !result->needs_reboot_mask && !owner.fault_latched_;
        // Release the helper even when resuming output is unsafe.
        esp_err_t error = owner.handoff_.release(resume);
        if (error != ESP_OK) return owner.fault(error);
        if (resume) owner.begin_arming();
        else owner.fault(ESP_FAIL);
        return ESP_OK;
    }

    dshot::Driver& driver_;
    dshot::Link& link_;
    dshot::Passthrough handoff_;
    std::int64_t (*clock_us_)();
    esc_passthrough_t *session_ = nullptr;
    std::int64_t ready_at_us_ = 0;
    State state_ = State::Stopped;
    bool fault_latched_ = false;
    std::uint16_t throttle_ = 0;
    esp_err_t error_ = ESP_OK;
};
