// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from Betaflight src/main/io/serial_4way_avrootloader.c
// at commit a25aff92155a9507642304701cc442f915ac4f59.
// ESP32 adaptation modified 2026-10-01.
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
 * for info about Hagens AVRootloader:
 * http://www.mikrocontroller.net/topic/avr-bootloader-mit-verschluesselung
*/

#include "esc_backend.h"
#include <string.h>

static uint16_t boot_crc(const uint8_t *data, size_t size)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xa001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static bool send_command(esc_backend_t *b, const uint8_t *data, size_t size)
{
    if (!size || size > 256) return false;
    uint8_t framed[258];
    memcpy(framed, data, size);
    if (b->connected) {
        uint16_t crc = boot_crc(data, size);
        framed[size++] = crc & 255;
        framed[size++] = crc >> 8;
    }
    return b->wire.send(b->wire.ctx, framed, size);
}

static bool connect_esc(void *ctx, uint8_t channel, uint8_t info[4])
{
    esc_backend_t *b = ctx;
    b->connected = false;
    b->mode = 0;
    memset(info, 0, 4);
    if (!b->wire.select(b->wire.ctx, channel)) return false;
    // Bootloader entry sequence from Betaflight serial_4way_avrootloader.c.
    static const uint8_t init[] = {0,0,0,0,0,0,0,0,0x0d,'B','L','H','e','l','i',0xf4,0x7d};
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        uint8_t reply[9];
        if (!send_command(b, init, sizeof(init)) ||
            !b->wire.receive(b->wire.ctx, reply, sizeof(reply), 30)) continue;
        if (memcmp(reply, "471", 3) || reply[8] != 0x30) continue;
        uint16_t signature = (uint16_t)reply[4] << 8 | reply[5];
        uint8_t mode;
        if (signature > 0xe800 && signature < 0xf900) mode = 1;
        else if (reply[4] > 0 && reply[4] < 0x90 && reply[5] == 6) mode = 4;
        else continue;
        info[0] = reply[5]; info[1] = reply[4]; info[2] = reply[3]; info[3] = mode;
        b->connected = true;
        b->mode = mode;
        return true;
    }
    b->wire.release(b->wire.ctx);
    return false;
}

static bool read_esc(void *ctx, uint16_t address, size_t size, uint8_t *data)
{
    esc_backend_t *b = ctx;
    if (!b->connected || !size || size > 256) return false;
    if (address != 0xffff) {
        uint8_t command[] = {0xff, 0, address >> 8, address & 255};
        uint8_t ack;
        if (!send_command(b, command, sizeof(command)) ||
            !b->wire.receive(b->wire.ctx, &ack, 1, 20) || ack != 0x30) goto fail;
    }
    uint8_t command[] = {0x03, size == 256 ? 0 : (uint8_t)size};
    uint8_t reply[259];
    if (!send_command(b, command, sizeof(command)) ||
        !b->wire.receive(b->wire.ctx, reply, size + 3, 30 + (unsigned)size)) goto fail;
    uint16_t crc = boot_crc(reply, size);
    if (reply[size] != (crc & 255) || reply[size + 1] != (crc >> 8) || reply[size + 2] != 0x30)
        goto fail;
    memcpy(data, reply, size);
    return true;
fail:
    b->connected = false;
    b->wire.release(b->wire.ctx);
    return false;
}

static bool alive_esc(void *ctx)
{
    esc_backend_t *b = ctx;
    if (!b->connected) return true;
    uint8_t command[] = {0xfd, 0};
    uint8_t ack;
    bool ok = send_command(b, command, sizeof(command)) &&
        b->wire.receive(b->wire.ctx, &ack, 1, 20) && ack == 0xc1;
    if (!ok) { b->connected = false; b->wire.release(b->wire.ctx); }
    return ok;
}

static bool reset_esc(void *ctx, uint8_t channel, bool reboot)
{
    esc_backend_t *b = ctx;
    bool ok = b->wire.select(b->wire.ctx, channel);
    if (ok) {
        static const uint8_t reset[] = {0,0,0,0};
        ok = b->wire.send(b->wire.ctx, reset, sizeof(reset));
        if (reboot) b->wire.pulse_low(b->wire.ctx, 300);
    }
    b->connected = false;
    b->wire.release(b->wire.ctx);
    return ok;
}

static void release_esc(void *ctx)
{
    esc_backend_t *b = ctx;
    b->connected = false;
    b->wire.release(b->wire.ctx);
}

static bool set_address(esc_backend_t *b, uint16_t address)
{
    const uint8_t command[] = {0xff, 0, address >> 8, address & 255};
    uint8_t ack;
    return send_command(b, command, sizeof(command)) &&
        b->wire.receive(b->wire.ctx, &ack, 1, 20) && ack == 0x30;
}

static bool erase_page(void *ctx, uint8_t page)
{
    esc_backend_t *b = ctx;
    if (!b->connected || b->mode != 1) return false;
    // SiLabs 4-way page numbers use 512-byte units (including BB51's multiplier).
    uint16_t address = (uint16_t)page << 9;
    // Pages above 127 exceed the 16-bit address range.
    if (page > 127) return false;
    if (!set_address(b, address)) goto fail;
    const uint8_t erase[] = {0x02, 0x01};
    uint8_t ack;
    if (!send_command(b, erase, sizeof(erase)) ||
        !b->wire.receive(b->wire.ctx, &ack, 1, 3000) || ack != 0x30) goto fail;
    return true;
fail:
    b->connected = false;
    b->wire.release(b->wire.ctx);
    return false;
}

static bool write_esc(void *ctx, uint16_t address, size_t size, const uint8_t *data)
{
    esc_backend_t *b = ctx;
    if (!b->connected || b->mode != 1 || !data || !size || size > 256) return false;
    // Address 0xffff keeps the bootloader's current address (like Betaflight)
    if (address != 0xffff && !set_address(b, address)) goto fail;
    const uint8_t buffer_command[] = {0xfe, 0, size == 256 ? 1 : 0, size == 256 ? 0 : (uint8_t)size};
    uint8_t ack;
    if (!send_command(b, buffer_command, sizeof(buffer_command)) ||
        b->wire.receive(b->wire.ctx, &ack, 1, 4)) goto fail;
    if (!send_command(b, data, size) ||
        !b->wire.receive(b->wire.ctx, &ack, 1, 80) || ack != 0x30) goto fail;
    const uint8_t program[] = {0x01, 0x01};
    if (!send_command(b, program, sizeof(program)) ||
        !b->wire.receive(b->wire.ctx, &ack, 1, 500) || ack != 0x30) goto fail;
    return true;
fail:
    b->connected = false;
    b->wire.release(b->wire.ctx);
    return false;
}

void esc_backend_init(esc_backend_t *b, const esc_wire_t *wire)
{
    memset(b, 0, sizeof(*b));
    b->wire = *wire;
}

pt_esc_ops_t esc_backend_ops(esc_backend_t *b)
{
    return (pt_esc_ops_t){.ctx = b, .connect = connect_esc, .read = read_esc,
        .erase_page = erase_page, .write = write_esc,
        .alive = alive_esc, .reset = reset_esc, .release = release_esc};
}
