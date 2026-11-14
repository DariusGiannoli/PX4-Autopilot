#!/usr/bin/env python3
"""Native tests of the actual backported C decoder and RPM validity functions.

Extracts functions verbatim, replacing hardware storage/clock only. This does
not validate timer/DMA interrupts, electrical timing or actual ESC operation.
"""
import ctypes
import json
from pathlib import Path
import random
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c'
GCR = [25, 27, 18, 19, 29, 21, 22, 23, 26, 9, 10, 11, 30, 13, 14, 15]


def function(source, name):
    match = re.search(r'(?m)^(?:static )?(?:unsigned|uint8_t|uint32_t|int|bool|void) '+name+r'\([^;{}]*\)\n\{', source)
    assert match, name
    end = match.end()
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def edges(gcr):
    value = gcr | (1 << 20)
    # Invert the receiver's Gray transform, including its leading start bit.
    value ^= value >> 1
    value ^= value >> 2
    value ^= value >> 4
    value ^= value >> 8
    value ^= value >> 16
    bits = f'{value:021b}'
    assert len(bits) == 21 and bits[0] == '1'
    out = [0, 100]
    start = 0
    for i in range(1, 22):
        if i == 21 or bits[i] != bits[start]:
            out.append(out[-1]+20*(i-start))
            start = i
    return out


def encode(payload, crc_flip=0):
    crc = (~(payload ^ (payload >> 4) ^ (payload >> 8))) & 15
    word = (payload << 4) | (crc ^ crc_flip)
    return sum(GCR[(word >> (4*i)) & 15] << (5*i) for i in range(4))


def main():
    source = SOURCE.read_text()
    names = ['nibbles_from_mapped', 'calculate_period', 'erpm_sample_fresh',
             'process_capture_results', 'up_bdshot_get_erpm_sample', 'up_bdshot_get_erpm', 'up_bdshot_channel_status',
             'dshot_motor_data_set']
    harness = '''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#define CHANNEL_CAPTURE_BUFF_SIZE 32
#define MAX_IO_TIMERS 3
#define MAX_TIMER_IO_CHANNELS 8
#define PX4_OK 0
#define PX4_ERROR -1
#define DSHOT_THROTTLE_POSITION 5u
#define DSHOT_TELEMETRY_POSITION 4u
#define NIBBLES_SIZE 4u
#define DSHOT_NUMBER_OF_NIBBLES 3u
#define ONE_MOTOR_DATA_SIZE 16u
#define MOTOR_PWM_BIT_1 14u
#define MOTOR_PWM_BIT_0 7u
typedef uint64_t hrt_abstime;
typedef int irqstate_t;
static uint64_t now_us=1000000;
static uint64_t hrt_absolute_time(void) { return now_us; }
static int px4_enter_critical_section(void) { return 0; }
static void px4_leave_critical_section(int flags) { (void)flags; }
static uint16_t dshot_capture_buffer[4][32];
static uint32_t read_fail_nibble[4],read_fail_crc[4],read_fail_zero[4],read_ok[4];
static int32_t _erpms[8];
static bool _erpms_ready[8],_erpm_capture_valid[8];
static uint64_t _erpm_last_valid[8];
static struct { uint8_t timer_index,timer_channel; } timer_io_channels[8]={{0,1}};
static struct { bool initialized_channels[4]; } timer_configs[3]={{{true,false,false,false}}};
typedef struct { unsigned channel_count_including_gaps,lowest_timer_channel; } io_timers_channel_mapping_element_t;
static struct { io_timers_channel_mapping_element_t element[3]; } io_timers_channel_mapping={{{4,1}}};
static bool _bidirectional;
static uint32_t transmit_buffer[68];
static uint32_t *dshot_output_buffer[3]={transmit_buffer};
static uint8_t output_channel_from_timer_channel(uint8_t timer,uint8_t channel) { (void)timer; return channel; }
'''
    harness += '\n'.join(function(source, name) for name in names)
    harness += '''
unsigned decode(const uint16_t *edges, unsigned count) {
 memset(dshot_capture_buffer,0,sizeof(dshot_capture_buffer));
 memcpy(dshot_capture_buffer[0],edges,count*sizeof(uint16_t));
 return calculate_period(0,0);
}
void feed(const uint16_t *edges,unsigned count,uint64_t now) {
 now_us=now; decode(edges,count); process_capture_results(0,0);
}
int get(uint64_t now,int *rpm) { now_us=now; return up_bdshot_get_erpm(0,rpm); }
int get_sample(uint64_t now,int *rpm,uint64_t *stamp) { now_us=now; return up_bdshot_get_erpm_sample(0,rpm,stamp); }
int invalid_channel(unsigned channel,unsigned timer,unsigned timer_channel) {
 if(channel<8){timer_io_channels[channel].timer_index=timer;timer_io_channels[channel].timer_channel=timer_channel;}
 int rpm=-7;uint64_t stamp=77;
 if(up_bdshot_get_erpm_sample(channel,&rpm,&stamp)!=PX4_ERROR || rpm!=-7 || stamp!=77)return 1;
 if(up_bdshot_channel_status(channel)!=0)return 2;
 dshot_motor_data_set(channel,2047,true);return 0;
}
int online(uint64_t now) { now_us=now; return up_bdshot_channel_status(0); }
unsigned packet(unsigned demand,bool telemetry,bool bidirectional) {
 _bidirectional=bidirectional; dshot_motor_data_set(0,demand,telemetry);
 unsigned value=0;
 for(unsigned i=0;i<16;i++) value=(value<<1)|(transmit_buffer[i*4]==MOTOR_PWM_BIT_1);
 return value;
}
'''
    with tempfile.TemporaryDirectory(prefix='morpheus_bdshot_') as directory:
        directory = Path(directory)
        cfile = directory/'test.c'
        cfile.write_text(harness)
        library = directory/'test.so'
        subprocess.run(['cc', '-std=c11', '-shared', '-fPIC', '-O1',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        str(cfile), '-o', str(library)], check=True)
        dll = ctypes.CDLL(str(library))
        edge_type = ctypes.c_uint16*32
        dll.decode.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint]
        dll.decode.restype = ctypes.c_uint
        dll.feed.argtypes = [ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint, ctypes.c_uint64]
        dll.get_sample.argtypes = [ctypes.c_uint64, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_uint64)]
        dll.invalid_channel.argtypes = [ctypes.c_uint]*3
        dll.get.argtypes = [ctypes.c_uint64, ctypes.POINTER(ctypes.c_int)]
        dll.online.argtypes = [ctypes.c_uint64]
        dll.packet.argtypes = [ctypes.c_uint, ctypes.c_bool, ctypes.c_bool]
        dll.packet.restype = ctypes.c_uint

        for demand in range(65536):
            for telemetry in (False, True):
                # Exercise production payload/checksum for all uint16 inputs.
                # Hardware I/O is mocked; no motor commands are sent.
                payload = ((demand & 2047) << 1) | int(telemetry)
                checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 15
                for bidirectional in (False, True):
                    expected = (payload << 4) | (checksum ^ (15 if bidirectional else 0))
                    assert dll.packet(demand, telemetry, bidirectional) == expected

        def decode(items):
            return dll.decode(edge_type(*items), len(items))

        for payload in range(4096):
            expected = (payload & 511) << (payload >> 9)
            assert decode(edges(encode(payload))) == expected, payload
        for payload in range(4096):
            for bit in (1, 2, 4, 8):
                assert decode(edges(encode(payload, bit))) == 0, (payload, bit)
        invalid_symbols = set(range(32)) - set(GCR)
        for payload in (1, 123, 511, 999, 2047, 4095):
            packet = encode(payload)
            for position in range(4):
                for symbol in invalid_symbols:
                    invalid = (packet & ~(31 << (5*position))) | symbol << (5*position)
                    assert decode(edges(invalid)) == 0, (payload, position, symbol)
        rng = random.Random(20261114)
        for _ in range(10000):
            decode([rng.randrange(65536) for _ in range(rng.randrange(33))])
        assert decode([0, 100, 100]) == 0
        assert decode([0, 100, 100+20*22]) == 0
        # Actual C capture/getter: valid zero, invalid frame and stale sample.
        dll.feed(edge_type(*edges(encode(4095))), len(edges(encode(4095))), 1000000)
        rpm = ctypes.c_int(-123)
        assert dll.get(1000001, ctypes.byref(rpm)) == 0 and rpm.value == 0
        assert dll.online(1000001) == 1
        stamp = ctypes.c_uint64(0)
        assert dll.get_sample(1000002, ctypes.byref(rpm), ctypes.byref(stamp)) == 0 and stamp.value == 1000000
        valid = edges(encode(123))
        dll.feed(edge_type(*valid), len(valid), 2000000)
        assert dll.get(2000001, ctypes.byref(rpm)) == 0 and rpm.value > 0
        dll.feed(edge_type(), 0, 2001000)
        rpm.value = -123
        assert dll.get(2001001, ctypes.byref(rpm)) == -1 and rpm.value == -123
        assert dll.online(2001001) == 1  # A single bad frame is not an ESC loss.
        assert dll.online(2100001) == 0
        dll.feed(edge_type(*valid), len(valid), 3000000)
        assert dll.get(3100001, ctypes.byref(rpm)) == -1
        assert dll.online(2999999) == 0
        for channel in (1, 7, 8, 255):
            for timer, timer_channel in ((3, 1), (0, 0), (0, 5), (255, 255)):
                assert dll.invalid_channel(channel, timer, timer_channel) == 0
    print(json.dumps(dict(passed=True, valid_payloads=4096, corrupt_crc_cases=16384,
                          transmit_packet_cases=262144, normal_encoder_all_uint16_inputs=True,
                          invalid_gcr_symbol_cases=6*4*len(invalid_symbols),
                          malformed_capture_fuzz_cases=10000,
                          validity_cases=['measured_stop', 'valid_rotation', 'invalid_no_refresh',
                                          'single_glitch_online', 'stale_offline', 'future_rejected', 'decode_timestamp_preserved', 'invalid_channel_bounds'],
                          undefined_behavior_sanitizer=True,
                          hardware_validation=False), indent=2))


if __name__ == '__main__':
    main()
