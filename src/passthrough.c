// SPDX-License-Identifier: GPL-3.0-or-later AND BSD-3-Clause
// Adapted from Betaflight src/main/io/serial_4way.c
// at commit a25aff92155a9507642304701cc442f915ac4f59.
// ESP32 adaptation modified 2026-10-03.
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: 4712
*/

#include "passthrough.h"
#include <string.h>

// CRC routine adapted from the BSD-licensed routine in serial_4way.c.
/* Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
   Copyright (c) 2005, 2007 Joerg Wunsch
   Copyright (c) 2013 Dave Hylands
   Copyright (c) 2013 Frederic Nadeau
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */
uint16_t pt_crc_xmodem(const uint8_t *data, size_t size)
{
    uint16_t crc = 0;
    for (size_t n = 0; n < size; ++n) {
        crc ^= (uint16_t)data[n] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

int pt_validate_pins(const int pins[PT_MAX_CHANNELS], pt_pin_allowed_fn allowed, void *ctx)
{
    int count = 0;
    bool disabled = false;
    for (int i = 0; i < PT_MAX_CHANNELS; ++i) {
        if (pins[i] == -1) { disabled = true; continue; }
        if (disabled || pins[i] < 0 || !allowed(ctx, pins[i])) return -1;
        for (int j = 0; j < i; ++j) if (pins[j] == pins[i]) return -1;
        ++count;
    }
    return count ? count : -1;
}

void pt_init(pt_t *pt, uint8_t channels, const pt_esc_ops_t *esc,
             pt_output_fn output, void *output_ctx, const uint8_t uid[12], const char board[4])
{
    memset(pt, 0, sizeof(*pt));
    pt->esc = *esc;
    pt->output = output;
    pt->output_ctx = output_ctx;
    pt->channels = channels <= PT_MAX_CHANNELS ? channels : 0;
    pt->mode = 1;
    memcpy(pt->uid, uid, 12);
    memcpy(pt->board, board, 4);
}

static bool msp_reply(pt_t *pt, uint8_t command, const uint8_t *payload, size_t size, bool error)
{
    uint8_t out[261] = {'$', 'M', error ? '!' : '>', (uint8_t)size, command};
    if (size) memcpy(out + 5, payload, size);
    uint8_t crc = (uint8_t)size ^ command;
    for (size_t i = 0; i < size; ++i) crc ^= payload[i];
    out[5 + size] = crc;
    return pt->output(pt->output_ctx, out, size + 6);
}

void pt_set_session_ops(pt_t *pt, const pt_session_ops_t *session)
{
    pt->session = *session;
}

void pt_set_msp_handler(pt_t *pt, pt_msp_command_fn handler, void *ctx)
{
    pt->application_command = handler;
    pt->application_ctx = ctx;
}

static bool builtin_msp(uint8_t command)
{
    switch (command) {
    case 1: case 2: case 3: case 4: case 5: case 36: case 101: case 104:
    case 124: case 130: case 131: case 160: case 245: return true;
    default: return false;
    }
}

void pt_reset_session(pt_t *pt)
{
    pt->esc.release(pt->esc.ctx);
    pt->connected = false;
    pt->fourway = false;
    pt->held = false;
    pt->mode = 1;
}

static void end_session(pt_t *pt, pt_session_end_reason_t reason)
{
    bool held = pt->held;
    pt_reset_session(pt);
    if (held && pt->session.end) pt->session.end(pt->session.ctx, reason);
}

static bool acquire_session(pt_t *pt)
{
    if (pt->held) return true;
    if (pt->session.acquire && !pt->session.acquire(pt->session.ctx)) return false;
    pt->held = true;
    return true;
}

static bool reply_result(pt_t *pt, bool sent)
{
    if (!sent) end_session(pt, PT_SESSION_REPLY_FAILURE);
    return sent;
}

static void handle_msp(pt_t *pt)
{
    uint8_t command = pt->input[4];
    size_t incoming = pt->input[3];
    uint8_t out[PT_MAX_MSP_PAYLOAD] = {0};
    size_t size = 0;
    if (incoming && builtin_msp(command)) {
        reply_result(pt, msp_reply(pt, command, NULL, 0, true));
        return;
    }
    switch (command) {
    case 1:
        if (!acquire_session(pt)) {
            reply_result(pt, msp_reply(pt, command, NULL, 0, true)); return;
        }
        if (pt->fourway) {
            pt->esc.release(pt->esc.ctx);
            pt->connected = false; pt->fourway = false; pt->mode = 1;
        }
        out[0] = 0; out[1] = 1; out[2] = 46; size = 3; break;
    case 2: memcpy(out, "BTFL", 4); size = 4; break;
    case 3: out[0] = 0; out[1] = 3; out[2] = 0; size = 3; break;
    case 4:
        memcpy(out, pt->board, 4);
        out[6] = 2; // Non-flight adapter
        out[7] = 0;
        out[8] = 5; memcpy(out + 9, "ESP32", 5);
        out[14] = 15; memcpy(out + 15, "ESC_PASSTHROUGH", 15);
        out[30] = 4; memcpy(out + 31, "ESP3", 4);
        size = 35;
        break;
    case 5:
        memcpy(out, __DATE__, 11); memcpy(out + 11, __TIME__, 8);
        memcpy(out + 19, "comp003", 7); size = 26; break;
    case 36: size = 4; break;
    case 101:
        size = 22;
        out[13] = 1; // PID profile count expected by the decoder.
        out[16] = 1;
        out[17] = 4; // RX_FAILSAFE no receiver
        break;
    case 104:
        // MSP_MOTOR: eight little-endian uint16_t values.
        size = 16;
        for (unsigned i = 0; i < pt->channels; ++i) {
            out[2 * i] = 0xe8;
            out[2 * i + 1] = 0x03;
        }
        break;
    case 124: size = 6; break;
    case 130: size = 11; break; // No battery sensor.
    case 131:
        size = 10;
        out[0] = 0xe8; out[1] = 3; // min throttle 1000
        out[2] = 0xd0; out[3] = 7; // max throttle 2000
        out[4] = 0xe8; out[5] = 3; // min command 1000
        out[6] = pt->channels;
        break;
    case 160: memcpy(out, pt->uid, 12); size = 12; break;
    case 245:
        if (!acquire_session(pt)) {
            reply_result(pt, msp_reply(pt, command, NULL, 0, true)); return;
        }
        if (pt->session.activate && !pt->session.activate(pt->session.ctx)) {
            bool sent = msp_reply(pt, command, NULL, 0, true);
            end_session(pt, sent ? PT_SESSION_SETUP_FAILURE : PT_SESSION_REPLY_FAILURE);
            return;
        }
        out[0] = pt->channels;
        if (reply_result(pt, msp_reply(pt, command, out, 1, false))) {
            if (!pt->fourway) {
                pt->esc.release(pt->esc.ctx);
                pt->connected = false;
            }
            pt->fourway = true;
        }
        return;
    default:
        if (pt->application_command) {
            bool error = false;
            if (pt->application_command(pt->application_ctx, command, pt->input + 5,
                                        incoming, out, sizeof(out), &size, &error)) {
                if (size > sizeof(out)) { size = 0; error = true; }
                reply_result(pt, msp_reply(pt, command, out, size, error));
                return;
            }
        }
        reply_result(pt, msp_reply(pt, command, NULL, 0, true));
        return;
    }
    reply_result(pt, msp_reply(pt, command, out, size, false));
}

static bool fourway_reply(pt_t *pt, uint8_t command, uint16_t address,
                         const uint8_t *payload, size_t size, uint8_t ack)
{
    uint8_t out[264] = {0x2e, command, address >> 8, address & 255, size == 256 ? 0 : (uint8_t)size};
    memcpy(out + 5, payload, size);
    out[5 + size] = ack;
    uint16_t crc = pt_crc_xmodem(out, 6 + size);
    out[6 + size] = crc >> 8; out[7 + size] = crc & 255;
    return pt->output(pt->output_ctx, out, 8 + size);
}

static void handle_fourway(pt_t *pt)
{
    uint8_t command = pt->input[1];
    uint16_t address = (uint16_t)pt->input[2] << 8 | pt->input[3];
    size_t size = pt->input[4] ? pt->input[4] : 256;
    uint8_t *param = pt->input + 5;
    uint8_t out[256] = {0};
    size_t out_size = 1;
    uint8_t ack = PT_OK;
    bool exit = false;
    uint16_t actual_crc = (uint16_t)pt->input[pt->used - 2] << 8 | pt->input[pt->used - 1];
    if (pt_crc_xmodem(pt->input, pt->used - 2) != actual_crc) ack = PT_INVALID_CRC;
    else if (size != 1 && command != 0x3b) ack = (command == 0x3e || command == 0x40)
                                  ? PT_INVALID_CMD : PT_INVALID_PARAM;
    else if (!pt->fourway && (command == 0x35 || command == 0x37 || command == 0x39 ||
                             command == 0x3a || command == 0x3b || command == 0x3f))
        ack = PT_DEVICE_ERROR;
    else switch (command) {
    case 0x30:
        if (pt->connected && !pt->esc.alive(pt->esc.ctx)) {
            pt->connected = false; ack = PT_DEVICE_ERROR;
        }
        break;
    case 0x31: out[0] = 108; break;
    case 0x32: memcpy(out, "ESP32-4way", 10); out_size = 10; break;
    case 0x33: out[0] = 0; out[1] = 2; out_size = 2; break;
    case 0x34: exit = true; break;
    case 0x35:
        if (param[0] >= pt->channels) ack = PT_INVALID_CHANNEL;
        else {
            if (!pt->esc.reset(pt->esc.ctx, param[0], (address & 255) == 1)) ack = PT_DEVICE_ERROR;
            pt->connected = false;
        }
        break;
    case 0x37:
        if (param[0] >= pt->channels) ack = PT_INVALID_CHANNEL;
        else if (!pt->fourway) ack = PT_DEVICE_ERROR;
        else {
            pt->connected = false;
            out_size = 4;
            pt->selected = param[0];
            if (pt->esc.connect(pt->esc.ctx, param[0], out)) {
                pt->connected = true; pt->mode = out[3];
            } else { memset(out, 0, 4); ack = PT_DEVICE_ERROR; }
        }
        break;
    case 0x3a:
        if (!pt->connected) ack = PT_DEVICE_ERROR;
        else {
            out_size = param[0] ? param[0] : 256;
            if (!pt->esc.read(pt->esc.ctx, address, out_size, out)) {
                out_size = 1; out[0] = 0; ack = PT_DEVICE_ERROR;
            }
        }
        break;
    case 0x39:
    case 0x3b:
        if (!pt->fourway || !pt->connected) ack = PT_DEVICE_ERROR;
        else if ((command == 0x39 && (pt->mode != 1 || !pt->esc.erase_page)) ||
                 (command == 0x3b && ((pt->mode != 1 && pt->mode != 4) ||
                                     !pt->esc.write))) ack = PT_INVALID_CMD;
        else if (command == 0x39 && param[0] > 127) ack = PT_INVALID_PARAM;
        else {
            bool ok = command == 0x39 ? pt->esc.erase_page(pt->esc.ctx, param[0])
                                     : pt->esc.write(pt->esc.ctx, address, size, param);
            if (!ok) { ack = PT_DEVICE_ERROR; pt->connected = false; }
        }
        break;
    case 0x3f:
        if (param[0] == 1 || param[0] == 4) pt->mode = param[0];
        else ack = PT_INVALID_PARAM;
        break;
    default: ack = PT_INVALID_CMD; break;
    }
    bool sent = fourway_reply(pt, command, address, out, out_size, ack);
    if (!sent) end_session(pt, PT_SESSION_REPLY_FAILURE);
    else if (exit && ack == PT_OK) end_session(pt, PT_SESSION_EXIT);
}

void pt_expire(pt_t *pt, uint32_t now_ms)
{
    if (pt->used && (uint32_t)(now_ms - pt->last_byte_ms) >= 250) pt->used = 0;
}

void pt_feed(pt_t *pt, const uint8_t *data, size_t size, uint32_t now_ms)
{
    pt_expire(pt, now_ms);
    for (size_t i = 0; i < size; ++i) {
        uint8_t byte = data[i];
        if (!pt->used && byte != '$' && byte != 0x2f) continue;
        pt->input[pt->used++] = byte;
        pt->last_byte_ms = now_ms;
        if (pt->input[0] == '$') {
            if ((pt->used == 2 && byte != 'M') || (pt->used == 3 && byte != '<')) {
                pt->used = byte == '$' ? 1 : 0;
                if (pt->used) pt->input[0] = '$';
                continue;
            }
            if (pt->used < 6 || pt->used != (size_t)pt->input[3] + 6) continue;
            uint8_t crc = 0;
            for (size_t j = 3; j < pt->used - 1; ++j) crc ^= pt->input[j];
            if (crc == pt->input[pt->used - 1]) {
                if (pt->session.traffic) pt->session.traffic(pt->session.ctx, now_ms);
                handle_msp(pt);
            }
            pt->used = 0;
        } else if (pt->used >= 5) {
            size_t wanted = 7 + (pt->input[4] ? pt->input[4] : 256);
            if (pt->used == wanted) {
                uint16_t crc = (uint16_t)pt->input[wanted - 2] << 8 | pt->input[wanted - 1];
                if (pt_crc_xmodem(pt->input, wanted - 2) == crc && pt->session.traffic)
                    pt->session.traffic(pt->session.ctx, now_ms);
                handle_fourway(pt); pt->used = 0;
            }
        }
    }
}
