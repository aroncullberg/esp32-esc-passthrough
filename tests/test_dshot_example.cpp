// SPDX-License-Identifier: GPL-3.0-or-later
#include "control.hpp"
#include "esc_uart.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

static std::string events;
static bool link_open, driver_running, io_active, start_ok, begin_ok, set_ok, stop_io_ok, output_ok;
static esp_err_t async_error;
static std::int64_t now_us;
static unsigned wire_size;
static std::vector<std::uint8_t> reply;

namespace dshot {
Driver::Driver(const Config& config) : config_(config) {}
Driver::~Driver() { stop(); }
void Driver::stop() { events += 'T'; driver_running = false; }
esp_err_t Driver::start() {
    events += 'D';
    assert(link_open && !io_active);
    if (passthrough_owner_) return ESP_ERR_INVALID_STATE;
    if (!start_ok) return ESP_FAIL;
    driver_running = true; async_error = ESP_OK; return ESP_OK;
}
esp_err_t Driver::last_error() const { return async_error; }
esp_err_t Driver::validate_passthrough_links(Link* const* links, std::size_t count) const {
    return links && count == 1 && links[0] && &links[0]->driver_ == this ? ESP_OK : ESP_ERR_INVALID_ARG;
}
Link::Link(Driver& driver, gpio_num_t pin) : driver_(driver), pin_(pin) {}
Link::~Link() { driver_.stop(); end(); }
esp_err_t Link::begin() {
    events += 'B'; assert(!io_active);
    if (driver_.passthrough_owner_) return ESP_ERR_INVALID_STATE;
    if (!begin_ok) return ESP_FAIL;
    link_open = true; return ESP_OK;
}
void Link::end() { events += 'E'; assert(!driver_running); link_open = false; }
esp_err_t Link::set(std::uint16_t payload, bool telemetry) {
    (void)telemetry; events += payload ? 'V' : 'Z';
    if (!link_open) return ESP_ERR_INVALID_STATE;
    return set_ok ? ESP_OK : ESP_FAIL;
}
}

extern "C" {
bool esc_uart_pin_allowed(void *ctx, int pin) { (void)ctx; return pin >= 0 && pin < 12; }
esp_err_t esc_uart_init(esc_uart_t *link, const int *pins, int count, int port)
{ (void)link; (void)pins; (void)count; (void)port; return ESP_OK; }
esp_err_t esc_uart_start(void *ctx) { (void)ctx; assert(false); return ESP_FAIL; }
esp_err_t esc_uart_stop(void *ctx) { (void)ctx; assert(false); return ESP_FAIL; }
esc_wire_t esc_uart_wire(esc_uart_t *link) { (void)link; return {}; }
}

static std::int64_t clock_us() { return now_us; }
static esp_err_t io_start(void*) {
    events += 'I'; assert(!driver_running && !link_open); io_active = true; return ESP_OK;
}
static esp_err_t io_stop(void*) {
    events += 'O'; if (!stop_io_ok) return ESP_FAIL; io_active = false; return ESP_OK;
}
static bool select_wire(void*, std::uint8_t) { assert(io_active); return true; }
static bool send_wire(void*, const std::uint8_t*, std::size_t size) { assert(io_active); wire_size = size; return true; }
static bool receive_wire(void*, std::uint8_t *out, std::size_t size, unsigned) {
    assert(io_active);
    if (wire_size != 17 || size != 9) return false;
    const std::uint8_t identity[]{'4','7','1','d',0xe8,0xb2,6,1,0x30};
    std::memcpy(out, identity, 9);
    return true;
}
static bool pulse_wire(void*, unsigned ms) { events += 'P'; assert(io_active && ms == 300); return true; }
static void release_wire(void*) { assert(io_active); }
static bool output(void *ctx, const std::uint8_t *out, std::size_t size) {
    events += 'R'; reply.assign(out, out + size);
    if (!output_ok) static_cast<MotorOwner*>(ctx)->output_failed();
    return output_ok;
}

struct Fixture {
    dshot::Config config = MotorOwner::config();
    dshot::Driver driver{config};
    dshot::Link link{driver, 0};
    MotorOwner owner{driver, link, clock_us};
    example_control::Context control{owner};
    esc_passthrough_t *session = nullptr;
    Fixture() {
        events.clear(); link_open = driver_running = io_active = false;
        start_ok = begin_ok = set_ok = stop_io_ok = output_ok = true;
        async_error = ESP_OK; now_us = 0;
        assert(config.protocol == dshot::Protocol::DShot300);
        assert(config.signaling == dshot::Signaling::Regular && config.frame_hz == 1000);
        esc_passthrough_io_t io{};
        io.start = io_start; io.stop = io_stop;
        io.wire = {nullptr, select_wire, send_wire, receive_wire, release_wire, pulse_wire, nullptr};
        auto options = esc_passthrough_config_default();
        options.pins[0] = 0; options.io = &io; options.lifecycle = owner.callbacks();
        options.output = output; options.output_ctx = &owner;
        options.application_command = example_control::command; options.application_ctx = &control;
        options.session_timeout_ms = 1000;
        assert(esc_passthrough_create(&options, &session) == ESP_OK);
        control.session = session; owner.attach_session(session);
        assert(events.empty() && !link_open && !driver_running);
    }
    ~Fixture() { stop_io_ok = true; esc_passthrough_destroy(session); }
    void msp(std::uint8_t command, const std::vector<std::uint8_t>& payload = {}) {
        std::vector<std::uint8_t> packet{'$', 'M', '<', static_cast<std::uint8_t>(payload.size()), command};
        packet.insert(packet.end(), payload.begin(), payload.end());
        std::uint8_t crc = 0; for (std::size_t i = 3; i < packet.size(); ++i) crc ^= packet[i];
        packet.push_back(crc);
        esc_passthrough_feed(session, packet.data(), packet.size(), static_cast<std::uint32_t>(now_us/1000));
    }
    void fourway(std::uint8_t command) {
        std::uint8_t packet[]{0x2f,command,0,0,1,0,0,0};
        std::uint16_t crc = pt_crc_xmodem(packet, 6); packet[6] = crc >> 8; packet[7] = crc;
        esc_passthrough_feed(session, packet, sizeof(packet), static_cast<std::uint32_t>(now_us/1000));
    }
    void arm_ready() {
        assert(owner.arm() == ESP_OK);
        now_us += MotorOwner::kArmingMs * 1000;
        owner.poll(); assert(owner.state() == MotorOwner::State::Ready);
    }
};

static void test_motor_policy_and_handoff() {
    Fixture f;
    assert(f.owner.run(48) == ESP_ERR_INVALID_STATE);
    assert(f.owner.arm() == ESP_OK && f.owner.state() == MotorOwner::State::Arming);
    assert(f.owner.run(48) == ESP_ERR_INVALID_STATE);
    assert(f.owner.stop() == ESP_OK && f.owner.state() == MotorOwner::State::Arming);
    now_us = MotorOwner::kArmingMs * 1000; f.owner.poll();
    assert(f.owner.run(47) == ESP_ERR_INVALID_ARG);
    assert(f.owner.run(2048) == ESP_ERR_INVALID_ARG);
    assert(f.owner.run(100) == ESP_OK && f.owner.throttle() == 100);
    events.clear(); f.msp(1);
    assert(events == "TER" && f.owner.state() == MotorOwner::State::Configuration);
    assert(!driver_running && !link_open && !io_active && !f.owner.throttle());
    assert(f.owner.arm() == ESP_ERR_INVALID_STATE && f.owner.run(100) == ESP_ERR_INVALID_STATE);
    f.msp(245); assert(io_active); f.fourway(0x37);
    assert(f.owner.stop() == ESP_ERR_INVALID_STATE);
    events.clear(); f.fourway(0x34);
    assert(events == "RPOBZD"); // Exit reply, reboot, IO stop, then zero-frame restoration.
    assert(f.owner.state() == MotorOwner::State::Arming && !f.owner.throttle());
    assert(f.owner.run(100) == ESP_ERR_INVALID_STATE);
    now_us += MotorOwner::kArmingMs * 1000; f.owner.poll();
    assert(f.owner.state() == MotorOwner::State::Ready && !f.owner.throttle());
    assert(f.owner.run(75) == ESP_OK);
}

static void test_faults_and_recovery() {
    Fixture f; f.arm_ready(); f.owner.run(100); f.msp(1);
    esc_passthrough_transport_lost(f.session);
    assert(f.owner.state() == MotorOwner::State::Fault && !link_open && !driver_running);
    f.msp(1); f.fourway(0x34);
    assert(f.owner.state() == MotorOwner::State::Fault && !driver_running);
    assert(f.owner.arm() == ESP_OK);
    now_us += MotorOwner::kArmingMs * 1000; f.owner.poll();
    async_error = ESP_FAIL; f.owner.poll();
    assert(f.owner.state() == MotorOwner::State::Fault && !driver_running && !link_open);
    assert(f.owner.arm() == ESP_OK);
    f.msp(1); esc_passthrough_poll(f.session, static_cast<std::uint32_t>(now_us/1000) + 1000);
    assert(f.owner.state() == MotorOwner::State::Fault);
}

static void test_failure_rollback() {
    {
        Fixture f; start_ok = false;
        assert(f.owner.arm() == ESP_FAIL && !driver_running && !link_open);
    }
    {
        Fixture f; f.arm_ready(); f.msp(1); f.msp(245); begin_ok = false;
        f.fourway(0x34);
        assert(f.owner.state() == MotorOwner::State::Fault && !driver_running && !link_open);
        assert(esc_passthrough_last_error(f.session) == ESP_FAIL);
    }
    {
        Fixture f; f.arm_ready(); f.msp(1); f.msp(245); stop_io_ok = false;
        f.fourway(0x34);
        assert(io_active && !driver_running && !link_open);
        assert(f.owner.arm() == ESP_ERR_INVALID_STATE);
        stop_io_ok = true;
        assert(esc_passthrough_retry_cleanup(f.session) == ESP_OK);
        assert(f.owner.arm() == ESP_OK);
    }
}

static void test_binary_commands() {
    Fixture f;
    f.msp(example_control::kCommand, {'D','S','H','1',1,0,0});
    assert(reply.size() == 30 && reply[2] == '>' && f.owner.state() == MotorOwner::State::Arming);
    f.msp(example_control::kCommand, {'D','S','H','1',2,48,0});
    assert(reply[2] == '!' && f.owner.state() == MotorOwner::State::Arming);
    now_us += MotorOwner::kArmingMs * 1000;
    f.msp(example_control::kCommand, {'D','S','H','1',2,48,0});
    assert(reply[2] == '>' && f.owner.throttle() == 48);
    f.msp(example_control::kCommand, {'D','S','H','0',1,0,0});
    assert(reply[2] == '!' && f.owner.throttle() == 48);
    f.msp(example_control::kCommand, {});
    assert(reply[2] == '!' && f.owner.throttle() == 48);
    output_ok = false;
    f.msp(example_control::kCommand, {'D','S','H','1',0,0,0});
    f.owner.poll();
    assert(!driver_running && f.owner.state() == MotorOwner::State::Fault);
}

int main() {
    test_motor_policy_and_handoff();
    test_faults_and_recovery();
    test_failure_rollback();
    test_binary_commands();
    std::cout << "PASS: fixed DShot300, arming gates, callback handoff order, abnormal-end latch, rollback and USB controls\n";
}
