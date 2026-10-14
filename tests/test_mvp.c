// SPDX-License-Identifier: GPL-3.0-or-later
#include "passthrough.h"
#include "esc_backend.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "trace_fixture.h"

static pt_t pt;
static uint8_t response[264];
static size_t response_size;
static unsigned replies, connects, reads, resets, alives, releases;
static uint8_t channel_seen;
static uint16_t address_seen;
static bool present, read_ok = true;

static bool output(void *ctx, const uint8_t *data, size_t size)
{
    (void)ctx;
    assert(size <= sizeof(response));
    memcpy(response, data, size); response_size = size; ++replies;
    if (data[0] == 0x2e) {
        size_t count = data[4] ? data[4] : 256;
        assert(size == count + 8);
        assert(pt_crc_xmodem(data, size - 2) == ((uint16_t)data[size-2] << 8 | data[size-1]));
    } else {
        uint8_t crc = 0;
        assert(size == (size_t)data[3] + 6);
        for (size_t i = 3; i < size - 1; ++i) crc ^= data[i];
        assert(crc == data[size-1]);
    }
    return true;
}
static bool connect_mock(void *ctx, uint8_t channel, uint8_t info[4])
{
    (void)ctx; ++connects; channel_seen = channel;
    if (!present) return false;
    const uint8_t identity[] = {0xb2, 0xe8, 'c', 1};
    memcpy(info, identity, 4); return true;
}
static bool read_mock(void *ctx, uint16_t address, size_t size, uint8_t *data)
{
    (void)ctx; ++reads; address_seen = address;
    for (size_t i = 0; i < size; ++i) data[i] = (uint8_t)(address + i);
    return read_ok;
}
static bool reset_mock(void *ctx, uint8_t channel, bool reboot)
{ (void)ctx; (void)reboot; ++resets; channel_seen = channel; return true; }
static bool alive_mock(void *ctx) { (void)ctx; ++alives; return present; }
static void release_mock(void *ctx) { (void)ctx; ++releases; }
static void setup(unsigned count, bool esc_present)
{
    replies = connects = reads = resets = alives = releases = 0;
    present = esc_present; read_ok = true; response_size = 0;
    pt_esc_ops_t ops = {.connect=connect_mock, .read=read_mock, .reset=reset_mock,
        .alive=alive_mock, .release=release_mock};
    const uint8_t uid[12] = {0};
    pt_init(&pt, count, &ops, output, NULL, uid, "ESP3");
}
static size_t msp(uint8_t *out, uint8_t cmd, const uint8_t *data, size_t size)
{
    memcpy(out, "$M<", 3); out[3] = size; out[4] = cmd;
    if (size) memcpy(out+5, data, size);
    uint8_t crc = size ^ cmd;
    for (size_t i=0; i<size; ++i) crc ^= data[i];
    out[size+5] = crc; return size+6;
}
static size_t fourway(uint8_t *out, uint8_t cmd, uint16_t address, const uint8_t *data, size_t size)
{
    assert(size >= 1 && size <= 256);
    out[0] = 0x2f; out[1] = cmd; out[2] = address >> 8; out[3] = address;
    out[4] = size == 256 ? 0 : size; memcpy(out+5, data, size);
    uint16_t crc = pt_crc_xmodem(out, size+5);
    out[size+5] = crc >> 8; out[size+6] = crc; return size+7;
}
static uint8_t ack(void) { return response[response_size-3]; }
static void request(uint8_t cmd, uint16_t address, uint8_t param)
{
    uint8_t packet[8];
    size_t size = fourway(packet, cmd, address, &param, 1);
    pt_feed(&pt, packet, size, 1);
}
static void enter(void)
{
    uint8_t packet[6]; pt_feed(&pt, packet, msp(packet,245,NULL,0), 0);
    assert(pt.fourway && response[5] == pt.channels);
}
static bool pin_allowed(void *ctx, int pin) { (void)ctx; return pin >= 0 && pin < 12; }
static void test_pins_channels(void)
{
    for (unsigned count=1; count<=4; ++count) {
        int pins[4] = {-1,-1,-1,-1};
        for (unsigned i=0; i<count; ++i) pins[i] = i;
        assert(pt_validate_pins(pins,pin_allowed,NULL) == (int)count);
        setup(count,true); enter();
        for (unsigned i=0; i<count; ++i) {
            request(0x37,0,i); assert(ack()==PT_OK && channel_seen==i && pt.selected==i);
        }
        unsigned calls = connects;
        request(0x37,0,count); assert(ack()==PT_INVALID_CHANNEL && connects==calls && pt.connected);
        request(0x35,0,count); assert(ack()==PT_INVALID_CHANNEL && resets==0);
    }
    const int invalid[][4] = {{-1,-1,-1,-1},{0,-1,2,-1},{0,0,-1,-1},{12,-1,-1,-1},{-2,-1,-1,-1}};
    for (unsigned i=0; i<sizeof(invalid)/sizeof(invalid[0]); ++i)
        assert(pt_validate_pins(invalid[i],pin_allowed,NULL)==-1);
}
static void test_trace(void)
{
    setup(4,false);
    for (unsigned i=0; i<sizeof(trace)/sizeof(trace[0]); ++i) {
        unsigned before = replies;
        for (size_t j=0; j<trace[i].req_size; ++j) pt_feed(&pt,trace[i].req+j,1,i*100+j);
        assert(replies==before+1 && response[1]==trace[i].rep[1]);
        if (trace[i].req[0]==0x2f) {
            assert(response_size==trace[i].rep_size);
            assert(!memcmp(response,trace[i].rep,response_size));
        } else {
            assert(response[2]=='>' && response[4]==trace[i].req[4]);
        }
    }
    assert(connects==20 && alives==0 && pt.fourway);
}
static void test_reads_rejections_recovery(void)
{
    setup(1,true); enter(); request(0x37,0,0); assert(ack()==0);
    request(0x3a,0x1a00,0); assert(ack()==0 && response_size==264 && address_seen==0x1a00);
    for (unsigned i=0;i<256;++i) assert(response[5+i]==(uint8_t)i);
    request(0x3a,0x1a40,128); assert(ack()==0 && response_size==136);
    unsigned calls = connects+reads+resets+alives+releases;
    const uint8_t rejected[] = {0x38,0x39,0x3b,0x3c,0x3d,0x3e,0x40,0xff};
    uint8_t packet[263], data[256]; memset(data,0xaa,sizeof(data));
    for (unsigned i=0;i<sizeof(rejected);++i) {
        request(rejected[i],0x1a00,0);
        assert(ack()==PT_INVALID_CMD && calls==connects+reads+resets+alives+releases);
        pt_feed(&pt,packet,fourway(packet,rejected[i],0,data,256),10);
        assert(ack()!=PT_OK && calls==connects+reads+resets+alives+releases);
    }
    size_t size = fourway(packet,0x37,0,data,1); packet[size-1]^=1;
    pt_feed(&pt,packet,size,11); assert(ack()==PT_INVALID_CRC && pt.connected);
    assert(calls==connects+reads+resets+alives+releases);
    pt_feed(&pt,packet,msp(packet,214,data,16),12); assert(response[2]=='!');
    assert(calls==connects+reads+resets+alives+releases);
    request(0x30,0,0); assert(ack()==0 && alives==1);
    read_ok=false; request(0x3a,0,1); assert(ack()==PT_DEVICE_ERROR && pt.connected);
    read_ok=true; request(0x3a,0x1a00,1); assert(ack()==PT_OK && pt.connected);
    request(0x37,0,0); assert(ack()==0);
    request(0x34,0,0); assert(ack()==0 && !pt.fourway && !pt.connected && releases>=2);
    request(0x37,0,0); assert(ack()==PT_DEVICE_ERROR);
    enter(); request(0x37,0,0);
    pt_feed(&pt,packet,msp(packet,1,NULL,0),20); assert(!pt.fourway && !pt.connected);
}
static void test_framing(void)
{
    setup(1,false);
    assert(pt_crc_xmodem((const uint8_t *)"123456789",9)==0x31c3);
    uint8_t packet[600]; size_t size = msp(packet,1,NULL,0);
    packet[size-1]^=1; pt_feed(&pt,packet,size,0); assert(replies==0);
    size = msp(packet,245,NULL,0); size += fourway(packet+size,0x31,0,(uint8_t[]){0},1);
    pt_feed(&pt,packet,size,1); assert(replies==2 && ack()==0 && response[5]==108);
    size=msp(packet,1,NULL,0);
    pt_feed(&pt,packet,3,UINT32_MAX-100); pt_expire(&pt,160); assert(pt.used==0);
    pt_feed(&pt,packet,size,161); assert(replies==3);
    pt_feed(&pt,(uint8_t[]){'$', '$'},2,200); assert(pt.used==1);
    pt_feed(&pt,packet+1,size-1,201); assert(replies==4);
    uint32_t random = 42;
    for (unsigned round=0;round<10000;++round) {
        for (unsigned i=0;i<sizeof(packet);++i) {
            random = random*1664525u+1013904223u; packet[i]=random>>24;
        }
        pt_feed(&pt,packet,sizeof(packet),round*300+500);
        assert(pt.used<=PT_MAX_REQUEST);
    }
}

static unsigned extension_calls;
static bool extension_oversize;
static bool extension(void *ctx, uint8_t command, const uint8_t *payload, size_t size,
                      uint8_t *reply, size_t capacity, size_t *reply_size, bool *error)
{
    (void)ctx; ++extension_calls;
    assert(command != 1 && command != 245);
    if (command != 240) return false;
    assert(capacity == PT_MAX_MSP_PAYLOAD);
    assert(size == 2 && payload[0] == 0x42 && payload[1] == 0x13);
    reply[0] = payload[0]; reply[1] = payload[1];
    *reply_size = extension_oversize ? capacity + 1 : 2;
    *error = false;
    return true;
}
static void test_application_commands(void)
{
    setup(1, true); extension_calls = 0; extension_oversize = false;
    pt_set_msp_handler(&pt, extension, NULL);
    uint8_t packet[20], payload[] = {0x42,0x13};
    size_t size = msp(packet, 240, payload, sizeof(payload));
    packet[size - 1] ^= 1;
    pt_feed(&pt, packet, size, 0); assert(extension_calls == 0 && replies == 0);
    packet[size - 1] ^= 1;
    pt_feed(&pt, packet, size, 1);
    assert(extension_calls == 1 && response[2] == '>' && response[3] == 2 && response[5] == 0x42);
    assert(!pt.held && !pt.fourway);
    extension_oversize = true;
    pt_feed(&pt, packet, size, 2);
    assert(extension_calls == 2 && response[2] == '!' && response[3] == 0);
    pt_feed(&pt, packet, msp(packet, 1, payload, sizeof(payload)), 3);
    assert(extension_calls == 2 && response[2] == '!' && !pt.held);
    pt_feed(&pt, packet, msp(packet, 245, payload, sizeof(payload)), 4);
    assert(extension_calls == 2 && response[2] == '!' && !pt.fourway);
    pt_feed(&pt, packet, msp(packet, 1, NULL, 0), 5);
    assert(extension_calls == 2 && pt.held);
    pt_feed(&pt, packet, msp(packet, 214, payload, sizeof(payload)), 6);
    assert(extension_calls == 3 && response[2] == '!');
}

static uint8_t sent[258];
static size_t sent_size;
static unsigned sends, selects, wire_releases, pulses;
static bool bad_crc, bad_ack, no_reply, flash_model, reject_devinfo;
static uint8_t flash_bytes[65536], buffer_bytes[256];
static uint16_t flash_address;
static size_t buffer_size;
static bool awaiting_buffer, buffer_ready;
static unsigned erased, programmed;
enum { INIT, ADDRESS, READ, SET_BUFFER, BUFFER_DATA, PROGRAM, ERASE, ALIVE, RESET };
static unsigned operation, fail_operation = 99;
static bool send_failure;
static uint16_t signature = 0xe8b2;
static uint16_t crc_boot(const uint8_t *bytes, size_t size)
{
    uint16_t crc=0;
    for(size_t i=0;i<size;++i) {
        crc^=bytes[i];
        for(unsigned j=0;j<8;++j) crc=(crc>>1)^((crc&1)?0xa001:0);
    }
    return crc;
}
static bool wire_select(void *ctx,uint8_t channel) { (void)ctx; ++selects; channel_seen=channel; return channel<4; }
static bool wire_send(void *ctx,const uint8_t *data,size_t size)
{
    (void)ctx; assert(size<=258); memcpy(sent,data,size); sent_size=size; ++sends;
    if (!flash_model) return true;
    if (awaiting_buffer) {
        assert(size == buffer_size + 2);
        operation = BUFFER_DATA;
        awaiting_buffer = false;
        buffer_ready = true;
        memcpy(buffer_bytes, data, buffer_size);
    } else if (size==17) {
        operation = INIT;
    } else if (!memcmp(data,(uint8_t[]){0,0,0,0},size < 4 ? size : 4)) {
        assert(size==4); operation=RESET;
    } else {
        switch (data[0]) {
        case 0xff:
            assert(size==6); operation=ADDRESS;
            flash_address=(uint16_t)data[2]<<8|data[3]; break;
        case 3: assert(size==4); operation=READ; break;
        case 0xfe:
            assert(size==6); operation=SET_BUFFER;
            buffer_size=(size_t)data[2]<<8|data[3];
            assert(buffer_size>=1 && buffer_size<=256);
            awaiting_buffer=true; break;
        case 1:
            assert(size==4 && data[1]==1 && buffer_ready); operation=PROGRAM;
            if (operation!=fail_operation) {
                if ((signature & 255)==6) {
                    // AM32's program operation performs its own erase.
                    memcpy(flash_bytes+flash_address,buffer_bytes,buffer_size);
                } else {
                    for(size_t i=0;i<buffer_size;++i)
                        flash_bytes[(uint16_t)(flash_address+i)] &= buffer_bytes[i];
                    flash_address+=buffer_size;
                }
                ++programmed;
            }
            buffer_ready=false; break;
        case 2:
            assert(size==4 && data[1]==1); operation=ERASE;
            if (operation!=fail_operation) {
                memset(flash_bytes+(flash_address&0xfe00),0xff,512); ++erased;
            }
            break;
        case 0xfd: assert(size==4); operation=ALIVE; break;
        default: assert(!"Unexpected bootloader command");
        }
    }
    if (operation != INIT && operation != RESET) {
        uint16_t crc = crc_boot(data,size-2);
        assert(data[size-2]==(crc&255) && data[size-1]==(crc>>8));
    }
    return !(send_failure && operation==fail_operation);
}
static bool wire_receive(void *ctx,uint8_t *data,size_t size,unsigned timeout)
{
    (void)ctx; assert(timeout>0);
    if(no_reply) return false;
    if (flash_model) {
        if (operation==SET_BUFFER && operation!=fail_operation) return false;
        if (operation==fail_operation && operation==ERASE) return false;
        if (operation!=INIT && operation!=READ) {
            assert(size==1);
            data[0] = operation==fail_operation ? 0xc2 :
                operation==ALIVE || (operation==ADDRESS && reject_devinfo && flash_address==0x23)
                ? 0xc1 : 0x30;
            return true;
        }
    }
    if(sent_size==17) {
        assert(size==9); memcpy(data,"471c",4);
        if ((signature & 255)==6) data[3]=0x02; // PA2.
        data[4]=signature>>8; data[5]=signature;
        data[6]=6; data[7]=1; data[8]=bad_ack?0xc1:0x30;
    } else if(sent[0]==0xff || sent[0]==0xfd) {
        assert(size==1); data[0]=bad_ack?0xc2:(sent[0]==0xfd?0xc1:0x30);
    } else {
        assert(sent[0]==3 && sent_size==4 && size>=4);
        for(size_t i=0;i<size-3;++i)
            data[i]=flash_model ? flash_bytes[(uint16_t)(flash_address+i)] : (uint8_t)i;
        if (flash_model) flash_address=(signature & 255)==6 ? 0 : flash_address+size-3;
        uint16_t crc=crc_boot(data,size-3); data[size-3]=(crc&255)^bad_crc;
        data[size-2]=crc>>8; data[size-1]=bad_ack?0xc2:0x30;
    }
    return true;
}
static void wire_release(void *ctx) { (void)ctx; ++wire_releases; awaiting_buffer=buffer_ready=false; }
static bool wire_pulse(void *ctx,unsigned duration) { (void)ctx; assert(duration==300); ++pulses; return true; }
static void test_backend(void)
{
    esc_wire_t wire={.select=wire_select,.send=wire_send,.receive=wire_receive,
        .release=wire_release,.pulse_low=wire_pulse};
    esc_backend_t backend; esc_backend_init(&backend,&wire);
    pt_esc_ops_t ops=esc_backend_ops(&backend); uint8_t info[4],data[256];
    assert(!ops.read(ops.ctx,0,1,data) && sends==0);
    assert(ops.connect(ops.ctx,2,info) && backend.connected && channel_seen==2);
    const uint8_t magic[]={0,0,0,0,0,0,0,0,13,'B','L','H','e','l','i',0xf4,0x7d};
    assert(sent_size==17 && !memcmp(sent,magic,17) && info[0]==0xb2 && info[1]==0xe8 && info[3]==1);
    assert(ops.read(ops.ctx,0x1a00,256,data));
    assert(sent_size==4 && sent[0]==3 && sent[1]==0);
    assert(sent[2]==0 && sent[3]==0xf0); // Independent known CRC vector: 03 00 -> f000.
    for(unsigned i=0;i<256;++i) assert(data[i]==i);
    unsigned before=sends; assert(ops.read(ops.ctx,0xffff,1,data) && sends==before+1);
    assert(ops.alive(ops.ctx) && sent[0]==0xfd && sent_size==4);
    bad_crc=true; assert(!ops.read(ops.ctx,0,1,data) && backend.connected);
    bad_crc=false; assert(ops.read(ops.ctx,0x1a00,1,data));
    bad_crc=false; bad_ack=true; assert(!ops.connect(ops.ctx,0,info)); bad_ack=false;
    signature=0x3506; assert(ops.connect(ops.ctx,1,info) && info[3]==4);
    assert(ops.reset(ops.ctx,1,true) && !backend.connected && pulses==1 && sent_size==4);
    assert(!memcmp(sent,(uint8_t[]){0,0,0,0},4));
    signature=0x9307; assert(!ops.connect(ops.ctx,0,info));
    signature=0xe8b2; no_reply=true; before=sends;
    assert(!ops.connect(ops.ctx,0,info) && sends==before+3); no_reply=false;
    assert(backend.init[0].attempts==3 && backend.init[0].attempt[2].result==ESC_INIT_RX_TIMEOUT);
    assert(backend.init[0].attempt[2].received==ESC_RX_SIZE_UNKNOWN);
    bad_ack=true; assert(!ops.connect(ops.ctx,1,info)); bad_ack=false;
    assert(backend.init[1].attempt[0].result==ESC_INIT_BAD_REPLY);
    assert(backend.init[1].attempt[0].received==9 && backend.init[1].attempt[0].reply[8]==0xc1);
    signature=0x9307; assert(!ops.connect(ops.ctx,2,info));
    assert(backend.init[2].attempt[0].result==ESC_INIT_BAD_SIGNATURE);
    signature=0xe8b2;
}

static unsigned diagnostic_step;
static size_t diagnostic_received;
static bool diagnostic_select(void *ctx, uint8_t channel)
{ (void)ctx; return channel != 3; }
static bool diagnostic_send(void *ctx, const uint8_t *data, size_t size)
{ (void)ctx; assert(data && size==17); return ++diagnostic_step != 1; }
static bool diagnostic_receive(void *ctx, uint8_t *data, size_t size, unsigned timeout)
{
    (void)ctx; assert(size==9 && timeout==30);
    const uint8_t reply[]={'4','7','1',2,0x35,6,6,3,0x30};
    diagnostic_received=diagnostic_step==2 ? 5 : 9;
    memcpy(data,reply,diagnostic_received);
    return diagnostic_received==9;
}
static size_t diagnostic_received_size(void *ctx) { (void)ctx; return diagnostic_received; }
static void test_init_diagnostics(void)
{
    esc_wire_t wire={.select=diagnostic_select,.send=diagnostic_send,.receive=diagnostic_receive,
        .release=wire_release,.pulse_low=wire_pulse,.received_size=diagnostic_received_size};
    esc_backend_t backend; esc_backend_init(&backend,&wire);
    pt_esc_ops_t ops=esc_backend_ops(&backend); uint8_t info[4];
    diagnostic_step=0;
    assert(ops.connect(ops.ctx,0,info) && info[3]==4);
    const esc_init_diagnostic_t *d=&backend.init[0];
    assert(d->attempts==3 && d->attempt[0].result==ESC_INIT_SEND_FAILED && !d->attempt[0].received);
    assert(d->attempt[1].result==ESC_INIT_RX_TIMEOUT && d->attempt[1].received==5);
    assert(!memcmp(d->attempt[1].reply,"471\x02\x35",5) && !d->attempt[1].reply[5]);
    assert(d->attempt[2].result==ESC_INIT_OK && d->attempt[2].received==9);
    assert(!ops.connect(ops.ctx,3,info) && backend.init[3].attempts==1);
    assert(backend.init[3].attempt[0].result==ESC_INIT_SELECT_FAILED);
    assert(!ops.connect(ops.ctx,4,info) && backend.init[0].attempts==3);
    assert(ops.connect(ops.ctx,0,info) && backend.init[0].attempts==1);
    assert(backend.init[0].attempt[1].result==ESC_INIT_NOT_ATTEMPTED && !backend.init[0].attempt[1].reply[0]);
}

typedef struct {
    uint8_t bytes[32];
    size_t size, offset, received;
    unsigned sends, receives;
} init_stream_t;
static bool init_stream_select(void *ctx, uint8_t channel) { (void)ctx; return channel<4; }
static bool init_stream_send(void *ctx, const uint8_t *data, size_t size)
{
    init_stream_t *s=ctx;
    assert(size==17 && data[8]==13); s->offset=0; ++s->sends; return true;
}
static bool init_stream_receive(void *ctx, uint8_t *data, size_t size, unsigned timeout)
{
    init_stream_t *s=ctx;
    assert(size>0 && size<=9 && timeout==30); ++s->receives;
    s->received=s->size-s->offset;
    if (s->received>size) s->received=size;
    memcpy(data,s->bytes+s->offset,s->received); s->offset+=s->received;
    return s->received==size;
}
static size_t init_stream_received(void *ctx) { return ((init_stream_t *)ctx)->received; }
static void init_stream_release(void *ctx) { (void)ctx; }
static void test_init_reply_resync(void)
{
    const uint8_t identity[]={'4','7','1',0x14,0x1f,6,6,1,0x30};
    for (unsigned noise=0;noise<=9;++noise) {
        init_stream_t stream={0};
        memset(stream.bytes,0xff,noise); memcpy(stream.bytes+noise,identity,9); stream.size=noise+9;
        esc_wire_t wire={.ctx=&stream,.select=init_stream_select,.send=init_stream_send,
            .receive=init_stream_receive,.received_size=init_stream_received,.release=init_stream_release};
        esc_backend_t backend; esc_backend_init(&backend,&wire);
        pt_esc_ops_t ops=esc_backend_ops(&backend); uint8_t info[4];
        bool ok=ops.connect(ops.ctx,1,info);
        const esc_init_attempt_t *a=&backend.init[1].attempt[0];
        if (noise<=8) {
            assert(ok && info[0]==6 && info[1]==0x1f && info[2]==0x14 && info[3]==4);
            assert(stream.sends==1 && stream.receives==(noise?2:1));
            assert(a->result==ESC_INIT_OK && a->received==9 && a->skipped==noise);
            assert(!memcmp(a->reply,identity,9));
            for (unsigned i=0;i<noise;++i) assert(a->prefix[i]==0xff);
        } else {
            assert(!ok && stream.sends==3 && stream.receives==3);
        }
    }
    for (unsigned failure=0;failure<3;++failure) {
        init_stream_t stream={0};
        stream.bytes[0]=0xff; memcpy(stream.bytes+1,identity,9); stream.size=10;
        if (failure==0) stream.bytes[9]=0xc1; // Bad ACK.
        if (failure==1) stream.bytes[5]=0x93; // Unsupported MCU signature.
        if (failure==2) stream.size=9; // Missing ACK.
        esc_wire_t wire={.ctx=&stream,.select=init_stream_select,.send=init_stream_send,
            .receive=init_stream_receive,.received_size=init_stream_received,.release=init_stream_release};
        esc_backend_t backend; esc_backend_init(&backend,&wire);
        pt_esc_ops_t ops=esc_backend_ops(&backend); uint8_t info[4];
        assert(!ops.connect(ops.ctx,0,info) && stream.sends==3 && stream.receives==6);
        assert(backend.init[0].attempt[2].skipped==1);
        if (failure==2) assert(backend.init[0].attempt[2].received==8);
    }
}

static void model_setup(esc_backend_t *backend, uint16_t device_signature)
{
    flash_model=true; bad_crc=bad_ack=no_reply=send_failure=reject_devinfo=false;
    sends=selects=wire_releases=pulses=erased=programmed=0;
    awaiting_buffer=buffer_ready=false; fail_operation=99; signature=device_signature;
    flash_address=0; memset(flash_bytes,0xff,sizeof(flash_bytes));
    esc_wire_t wire={.select=wire_select,.send=wire_send,.receive=wire_receive,
        .release=wire_release,.pulse_low=wire_pulse};
    esc_backend_init(backend,&wire);
    setup(2,true); pt.esc=esc_backend_ops(backend);
    enter(); request(0x37,0,0); assert(ack()==PT_OK);
}
static void write_request(uint16_t address, const uint8_t *data, size_t size, bool corrupt)
{
    uint8_t packet[263]; size_t count=fourway(packet,0x3b,address,data,size);
    if(corrupt) packet[count-1]^=1;
    for(size_t i=0;i<count;++i) pt_feed(&pt,packet+i,1,1000+i);
}
static void test_write_transactions(void)
{
    esc_backend_t backend;
    uint8_t data[256]; for(unsigned i=0;i<256;++i) data[i]=(uint8_t)(i*17+29);
    const unsigned sizes[]={1,128,255,256};
    for(unsigned i=0;i<sizeof(sizes)/sizeof(sizes[0]);++i) {
        model_setup(&backend,0xe8b2);
        flash_bytes[0]=0x42; flash_bytes[0x1c00]=0x81;
        memset(flash_bytes+0x1a00,0,512);
        request(0x39,0,13); assert(ack()==PT_OK && erased==1);
        for(unsigned j=0;j<512;++j) assert(flash_bytes[0x1a00+j]==0xff);
        unsigned size=sizes[i];
        write_request(0x1a00,data,size,false);
        assert(ack()==PT_OK && programmed==1 && backend.connected);
        assert(!memcmp(flash_bytes+0x1a00,data,size));
        assert(flash_bytes[0]==0x42 && flash_bytes[0x1c00]==0x81);
        request(0x3a,0x1a00,size==256?0:size);
        assert(ack()==PT_OK && !memcmp(response+5,data,size));
        assert(sent_size==4 && sent[0]==3);
    }
    model_setup(&backend,0xe8b2);
    request(0x39,0,1); assert(ack()==0 && erased==1);
    write_request(0x200,data,255,false); assert(ack()==0 && programmed==1);
    assert(!memcmp(flash_bytes+0x200,data,255));
    unsigned before=sends;
    write_request(0xffff,data,1,false);
    assert(ack()==0 && sends==before+3 && flash_bytes[0x2ff]==data[0]);
    assert(buffer_size==1);
    before=sends;
    write_request(0x1a00,data,255,true); assert(ack()==PT_INVALID_CRC && sends==before);
    request(0x39,0,128); assert(ack()==PT_INVALID_PARAM && sends==before);
    request(0x3f,0,4); request(0x39,0,13);
    assert(ack()==PT_INVALID_CMD && sends==before);
    request(0x3f,0,1); request(0x35,0,0);
    write_request(0x1a00,data,255,false); assert(ack()==PT_DEVICE_ERROR);
    model_setup(&backend,0xe8b5); // BB51 uses 512-byte page numbering in 4-way.
    request(0x39,0,24); assert(ack()==0 && flash_address==0x3000);
    model_setup(&backend,0x3506);
    before=sends; request(0x39,0,13); assert(ack()==PT_INVALID_CMD && sends==before);
    write_request(0xf800,data,184,false); assert(ack()==PT_OK && programmed==1);
    assert(!memcmp(flash_bytes+0xf800,data,184));
}
static void test_write_failures(void)
{
    const unsigned failures[]={ADDRESS,SET_BUFFER,BUFFER_DATA,PROGRAM,ERASE};
    uint8_t data[255]; memset(data,0x75,sizeof(data));
    esc_backend_t backend;
    for(unsigned i=0;i<sizeof(failures)/sizeof(failures[0]);++i) {
        model_setup(&backend,0xe8b2); fail_operation=failures[i];
        if(fail_operation==ERASE) request(0x39,0,13);
        else write_request(0x1a00,data,sizeof(data),false);
        assert(ack()==PT_DEVICE_ERROR && !backend.connected && !pt.connected && wire_releases);
        fail_operation=99; request(0x37,0,0); assert(ack()==PT_OK && backend.connected);
    }
    model_setup(&backend,0xe8b2); fail_operation=BUFFER_DATA; send_failure=true;
    write_request(0x1a00,data,sizeof(data),false);
    assert(ack()==PT_DEVICE_ERROR && !backend.connected && programmed==0);
}

static void test_am32_configurator(void)
{
    const struct { uint16_t signature, eeprom; uint8_t shift; } variants[] = {
        {0x1f06,0x7c00,0}, {0x3506,0xf800,0}, {0x1506,0xe000,0}, {0x2b06,0x7e00,2}
    };
    uint8_t settings[184];
    for (size_t i=0;i<sizeof(settings);++i) settings[i]=(uint8_t)(i*7+1);
    for (size_t i=0;i<sizeof(variants)/sizeof(variants[0]);++i) {
        for (unsigned v3=variants[i].shift ? 1 : 0;v3<2;++v3) {
            esc_backend_t backend;
            uint16_t eeprom=variants[i].eeprom;
            uint16_t filename=v3 ? 0x1080 : eeprom-32;
            model_setup(&backend,variants[i].signature);
            assert(pt.mode==4 && response[5]==6 && response[6]==(variants[i].signature>>8));
            assert(response[7]==0x02 && response[8]==4);
            memcpy(flash_bytes+filename,"AM32_TEST_TARGET",17);
            memcpy(flash_bytes+eeprom,settings,sizeof(settings));
            uint8_t devinfo[64]={0xda,0xe3,0x25,0x59,0xd9,0x63,0xb8,0x4e,
                '4','7','1',0x02,variants[i].signature>>8,6,6,3,0x30,27,variants[i].shift,
                0,0x10,filename&255,filename>>8,eeprom&255,eeprom>>8,0,0};
            memcpy(flash_bytes+0x23,devinfo,sizeof(devinfo));
            reject_devinfo=!v3;
            unsigned released=wire_releases, selected=selects;
            request(0x3a,0x23,64);
            if (v3) {
                assert(ack()==PT_OK && response_size==72 && !memcmp(response+5,devinfo,64));
            } else {
                assert(ack()==PT_DEVICE_ERROR && response_size==9 && response[5]==0);
            }
            assert(pt.connected && backend.connected && wire_releases==released);
            request(0x3a,filename,32);
            assert(ack()==PT_OK && !memcmp(response+5,"AM32_TEST_TARGET",17));
            request(0x3a,eeprom,sizeof(settings));
            assert(ack()==PT_OK && response_size==sizeof(settings)+8);
            assert(!memcmp(response+5,settings,sizeof(settings)));
            assert(selects==selected && wire_releases==released);
            request(0x37,0,0); assert(ack()==PT_OK);
            uint8_t updated[184]; memset(updated,0xa5,sizeof(updated));
            write_request(eeprom,updated,sizeof(updated),false);
            assert(ack()==PT_OK && programmed==1 && erased==0 && buffer_size==184);
            request(0x3a,eeprom,sizeof(updated));
            assert(ack()==PT_OK && !memcmp(response+5,updated,sizeof(updated)));
            request(0x30,0,0); assert(ack()==PT_OK);
            request(0x35,1,0); assert(ack()==PT_OK && pulses==1 && !backend.connected);
            request(0x34,0,0); assert(ack()==PT_OK && !pt.fourway);
        }
    }
    esc_backend_t backend;
    model_setup(&backend,0x3506);
    bad_crc=true; request(0x3a,0xf800,184);
    assert(ack()==PT_DEVICE_ERROR && response_size==9 && response[5]==0);
    bad_crc=false; request(0x3a,0xf800,184); assert(ack()==PT_OK);
    bad_ack=true; request(0x3a,0xf800,184); assert(ack()==PT_DEVICE_ERROR);
    bad_ack=false; no_reply=true;
    request(0x3a,0xf800,184); assert(ack()==PT_DEVICE_ERROR);
    no_reply=false; request(0x3a,0xf800,184); assert(ack()==PT_OK);
    no_reply=true; request(0x30,0,0);
    assert(ack()==PT_DEVICE_ERROR && !pt.connected && !backend.connected);
    no_reply=false;
    const unsigned failures[]={ADDRESS,SET_BUFFER,BUFFER_DATA,PROGRAM};
    for (size_t i=0;i<sizeof(failures)/sizeof(failures[0]);++i) {
        model_setup(&backend,0x3506); fail_operation=failures[i];
        write_request(0xf800,settings,sizeof(settings),false);
        assert(ack()==PT_DEVICE_ERROR && !backend.connected && !pt.connected);
        fail_operation=99; request(0x37,0,0); assert(ack()==PT_OK);
    }
}
int main(void)
{
    test_pins_channels(); puts("PASS: GPIO 0, pin validation, channel counts 1-4 and selection");
    test_trace(); puts("PASS: all 48 captured FC exchanges, including exact no-ESC 4-way replies");
    test_reads_rejections_recovery(); puts("PASS: 128/256-byte reads, CRC rejection, unsupported commands, exit and reconnect");
    test_framing(); puts("PASS: split/coalesced packets, timeout wraparound and 6 MB malformed input");
    test_application_commands(); puts("PASS: application MSP validation, built-in protection and reply bounds");
    test_backend(); puts("PASS: SiLabs/ARM identification, bootloader reads, bad CRC/ACK and reset");
    test_init_diagnostics(); puts("PASS: init diagnostics distinguish TX failure, partial RX, invalid replies and success");
    test_init_reply_resync(); puts("PASS: bounded init resynchronization after leading FF, partial headers, ACK/signature rejection");
    test_write_transactions(); puts("PASS: generic SiLabs page erase, 1/128/255/256-byte writes and read-back");
    test_write_failures(); puts("PASS: erase timeout, buffer/prog ACK failures, send failures and reconnect");
    test_am32_configurator(); puts("PASS: AM32 legacy/v3 devinfo fallback, filename/settings reads, ARM writes and failure recovery");
    puts("All MVP host tests passed (AddressSanitizer + UndefinedBehaviorSanitizer).");
    return 0;
}
