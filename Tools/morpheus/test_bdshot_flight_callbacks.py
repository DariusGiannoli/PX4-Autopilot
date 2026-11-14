#!/usr/bin/env python3
"""Test production pin release, DMA routing, bounded selection and output restore."""
import ctypes,json,subprocess,tempfile
from pathlib import Path
from test_bdshot_offline import function
ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c').read_text()
h=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define MAX_TIMER_IO_CHANNELS 8
#define MAX_NUM_CHANNELS_PER_TIMER 4
#define CHANNEL_CAPTURE_BUFF_SIZE 32
#define DSHOT_CAPTURE_BUFFER_SIZE(n) (64*(n))
#define IOTimerChanMode_CaptureDMA 11
#define IOTimerChanMode_DshotInverted 10
#define IO_TIMER_ALL_MODES_CHANNELS 255
#define STM32_GTIM_CCR1_OFFSET 20
#define DSHOT_BIDIRECTIONAL_DMA_SCR 123
#define PX4_WARN(...) ((void)0)
#define GPIO_MODE_MASK 0xc0000
#define GPIO_AF_MASK 0xf000
#define GPIO_PUPD_MASK 0x30000
#define GPIO_PULLUP 0x10000
#define GPIO_INPUT 0
#define GPIO_FLOAT 0
#define GPIO_ALT 0x80000
#define GPIO_AF1 0x1000
typedef void *DMA_HANDLE;
typedef uint64_t hrt_abstime;




static unsigned _dshot_frequency=300000;
static uint16_t dshot_capture_buffer[4][32];
static int _cc_call;
static struct { unsigned base; struct { int dma_map_ch[4]; } dshot; } io_timers[1]={{0,{{1,2,3,4}}}};
static struct { uint8_t timer_index,timer_channel; unsigned gpio_in; } timer_io_channels[8];
static struct { DMA_HANDLE dma_handle; int capture_channel_index; bool initialized_channels[4]; } timer_configs[1];

static unsigned configs[16],config_count,before_stop,stopped,probe_mode,probe_count,probe_channel;
static unsigned restored_mask,selected_input,selected_dma,selected_capture,selected_request,selected_source;
static uint32_t current_gpio;
static void px4_arch_configgpio(unsigned gpio) {
 if(config_count<16) configs[config_count++]=gpio;
 if(!stopped) ++before_stop;
 current_gpio=gpio;
}
static hrt_abstime hrt_absolute_time(void) { return 100; }
static void stm32_dmastop(DMA_HANDLE h) { (void)h; stopped=1; }
static void stm32_dmafree(DMA_HANDLE h) { (void)h; }
static void io_timer_update_dma_req(unsigned t,bool on) { (void)t;(void)on; }
static void io_timer_unallocate_timer(unsigned t) { (void)t; }
static void up_clean_dcache(uintptr_t a,uintptr_t b) { (void)a;(void)b; }
static void io_timer_unallocate_channel(unsigned c) { (void)c; }
static void io_timer_channel_init(unsigned c,int mode,void *a,void *b) {
 (void)a;(void)b;if(mode==IOTimerChanMode_DshotInverted) restored_mask|=1u<<c;else selected_input=c;current_gpio=timer_io_channels[c].gpio_in;
}
static DMA_HANDLE stm32_dmachannel(int map) { selected_dma=map;return (void *)(uintptr_t)map; }
static void io_timer_set_dshot_capture_mode(unsigned t,unsigned c,unsigned f) { (void)t;(void)f;selected_capture=c; }
static void io_timer_capture_dma_req(unsigned t,unsigned c,bool on) { (void)t;(void)on;selected_request=c; }
static void px4_stm32_dmasetup(DMA_HANDLE h,uint32_t src,uint32_t dst,unsigned n,unsigned flags) {
 (void)h;(void)dst;(void)n;(void)flags;selected_source=src;
}
static void stm32_dmastart(DMA_HANDLE h,void *cb,void *arg,bool half) { (void)h;(void)cb;(void)arg;(void)half; }
static void io_timer_set_enable(bool on,int mode,unsigned mask) { (void)on;(void)mode;(void)mask; }
static void capture_complete_callback(void *arg);
static void hrt_call_after(int *call,hrt_abstime delay,void (*cb)(void *),void *arg) { (void)call;(void)delay;(void)cb;(void)arg; }

static int hrt_callback_perf;
static unsigned processed,processed_channel;
static void perf_begin(int p) { (void)p; }
static void perf_end(int p) { (void)p; }
static void up_invalidate_dcache(uintptr_t a,uintptr_t b) { (void)a;(void)b; }
static void process_capture_results(uint8_t t,uint8_t c) { (void)t;++processed;processed_channel=c; }
'''
for name in ('release_timer_gpio_inputs','select_next_capture_channel','output_channel_from_timer_channel','dma_burst_finished_callback','capture_complete_callback'):
 h+=function(source,name)+'\n'
h+=r'''int check(unsigned mask,unsigned prev) {
 for(unsigned c=0;c<8;c++) {
  timer_io_channels[c].timer_index=c<4?0:1;
  timer_io_channels[c].timer_channel=(c%4)+1;
  timer_io_channels[c].gpio_in=GPIO_ALT|GPIO_AF1|0x340|c;
 }
 timer_io_channels[6].timer_index=0;timer_io_channels[6].timer_channel=0;
 timer_io_channels[7].timer_index=0;timer_io_channels[7].timer_channel=5;
 for(unsigned c=0;c<4;c++) timer_configs[0].initialized_channels[c]=!!(mask&(1u<<c));
 timer_configs[0].capture_channel_index=prev;timer_configs[0].dma_handle=(void *)1;
 config_count=before_stop=stopped=processed=restored_mask=0;
 selected_input=99;
 uint8_t timer=0;dma_burst_finished_callback((void *)1,0,&timer);
 if(mask==0) return selected_input==99 && config_count==0?0:1;
 unsigned expected=(prev+1)%4;while(!(mask&(1u<<expected)))expected=(expected+1)%4;
 if(selected_input!=expected || selected_capture!=expected || selected_request!=expected || selected_dma!=expected+1 || selected_source!=20+4*expected)return 2;
 unsigned n=0;
 for(unsigned c=0;c<4;c++)if(mask&(1u<<c)) {
  unsigned gpio=(timer_io_channels[c].gpio_in&~(GPIO_MODE_MASK|GPIO_AF_MASK|GPIO_PUPD_MASK))|GPIO_INPUT|GPIO_FLOAT;
  if(configs[n++]!=gpio)return 3;
 }
 if(before_stop!=n || config_count!=n)return 4;
 capture_complete_callback(&timer);
 if(restored_mask!=mask || processed!=1 || processed_channel!=expected)return 5;
 if(output_channel_from_timer_channel(0,4)!=8 || output_channel_from_timer_channel(2,0)!=8)return 6;
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='morpheus_flight_callback_') as d:
 d=Path(d);(d/'test.c').write_text(h)
 subprocess.run(['cc','-std=c11','-shared','-fPIC','-O1','-Wno-pointer-to-int-cast','-Wno-int-conversion','-fsanitize=undefined','-fno-sanitize-recover=all',str(d/'test.c'),'-o',str(d/'test.so')],check=True)
 lib=ctypes.CDLL(str(d/'test.so'));lib.check.argtypes=[ctypes.c_uint]*2;n=0
 for mask in range(16):
  for prev in range(4):
   result=lib.check(mask,prev);assert result==0,(mask,prev,result);n+=1
 print(json.dumps({'passed':True,'callback_cases':n,'checks':['early_release_before_DMA_cleanup','same_GPIO_DMA_timer_channel','only_initialized_motor_pins','restore_outputs_and_process_selected_channel','ignore_servo_and_invalid_channels','empty_mask_terminates','missing_mapping_not_output_zero'],'hardware_validation':False},indent=2))
