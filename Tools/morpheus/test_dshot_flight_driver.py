#!/usr/bin/env python3
"""Exercise actual driver methods with native mocks; never talks to an FC."""
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'src/drivers/dshot/DShot.cpp').read_text()

def method(name):
    match = re.search(r'(?m)^(?:bool|int|void) DShot::'+name+r'\([^;{}]*\)\n\{', source)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]

harness = r'''
#include <stdint.h>
#include <cstring>
#include <algorithm>
#include <cassert>
#define MAX_ACTUATORS 8
#define DSHOT_DISARM_VALUE 0
#define DSHOT_MAX_THROTTLE 1999
namespace math { using std::min; }
enum dshot_command_t { DShot_cmd_motor_stop=0, DShot_cmd_save_settings=12 };
struct esc_report_s { uint64_t timestamp=0; int esc_rpm=0; unsigned actuator_function=0; };
struct esc_status_s {
 static constexpr unsigned ESC_CONNECTION_TYPE_DSHOT=5;
 uint64_t timestamp=0;unsigned esc_online_flags=0,esc_armed_flags=0,esc_count=0,esc_connectiontype=0,counter=0;
 esc_report_s esc[8];
};
struct Pub {
 esc_status_s data,published; bool advertised_=false;unsigned publications=0;
 esc_status_s &get(){return data;}
 bool advertised(){return advertised_;}
 void advertise(){advertised_=true;published=data;++publications;}
 void update(){published=data;++publications;}
};
struct Mix {
 struct {bool armed=false;} arm;
 bool armed_functions[8]={true,true,true,true,false,false,false,false};
 auto &armed(){return arm;}
 bool isFunctionSet(unsigned i){return armed_functions[i];}
};
struct IntParam {int value=0;int get(){return value;}};
struct FloatParam {float value=0;float get(){return value;}};
struct Telemetry {bool expectingData(){return false;}int getRequestMotorIndex(){return -1;}};
struct Request {void *load(){return nullptr;}};
struct Command {
 unsigned command=0,motor_mask=255;int num_repetitions=0;bool save=false;
 bool valid(){return num_repetitions>0;}void clear(){num_repetitions=0;}
};
static uint64_t now=1001000;
static unsigned frames=0,commands=0,throttles=0,ready=4,valid_mask=15,fresh_mask=15;
static unsigned sent_channel[8],sent_value[8],sent_kind[8],sent_count;
static uint64_t hrt_absolute_time(){return now;}
static void record(unsigned c,unsigned v,unsigned kind){assert(sent_count<8);sent_channel[sent_count]=c;sent_value[sent_count]=v;sent_kind[sent_count++]=kind;}
static void up_dshot_motor_command(unsigned c,unsigned v,bool t){(void)t;++commands;record(c,v,0);}
static void up_dshot_motor_data_set(unsigned c,unsigned v,bool t){(void)t;++throttles;record(c,v,1);}
static void up_dshot_trigger(){++frames;}
static int up_bdshot_num_erpm_ready(){return ready;}
static int up_bdshot_get_erpm_sample(unsigned c,int *rpm,uint64_t *stamp){
 if(!(valid_mask&(1u<<c)))return -1;
 *rpm=600*(c+1);*stamp=1000000-10*c;return 0;
}
static bool up_bdshot_channel_status(unsigned c){return fresh_mask&(1u<<c);}
class DShot {
public:
 static constexpr unsigned _num_outputs=8;
 bool _outputs_on=true,_waiting_for_esc_info=false,_bidirectional_dshot_enabled=true;
 unsigned _reversible_outputs=0;int _num_motors=4,_last_telemetry_index=-1;
 uint8_t _actuator_functions[8]={101,104,103,102};
 uint16_t _esc_status_counter=0;
 Mix _mixing_output;Pub esc_status_pub;Telemetry *_telemetry=nullptr;Request _request_esc_info;Command _current_command;
 IntParam _param_dshot_3d_enable,_param_dshot_3d_dead_l,_param_dshot_3d_dead_h,_param_mot_pole_count{12};
 FloatParam _param_dshot_min;
 int request_esc_info(){return -1;}
 bool updateOutputs(bool stop_motors,uint16_t outputs[MAX_ACTUATORS],unsigned num_outputs,unsigned num_control_groups_updated);
 void publish_esc_status();
 int handle_new_bdshot_erpm();
};
'''
for name in ('updateOutputs', 'handle_new_bdshot_erpm', 'publish_esc_status'):
    harness += method(name) + '\n'
harness += r'''
int main(){
 unsigned cases=0;
 // Disabled outputs never send packets; stopping overrides every nonzero mixer output.
 for(unsigned demand=0;demand<=65535;demand+=257){
  DShot d;uint16_t outputs[8];std::fill_n(outputs,8,demand);
  sent_count=commands=throttles=frames=0;d._outputs_on=false;
  assert(!d.updateOutputs(false,outputs,4,0));assert(!sent_count && !frames);
  d._outputs_on=true;
  assert(d.updateOutputs(true,outputs,4,0));assert(sent_count==4 && commands==4 && !throttles && frames==1);
  for(unsigned i=0;i<4;i++)assert(sent_kind[i]==0 && sent_value[i]==0 && sent_channel[i]==i);
  cases+=2;
 }
 // Normal flight path passes bounded throttle and explicit zero to the original APIs.
 {
  DShot d;uint16_t outputs[8]={0,1,1999,65535};sent_count=commands=throttles=frames=0;
  assert(d.updateOutputs(false,outputs,4,0));assert(sent_count==4 && frames==1 && commands==1 && throttles==3);
  assert(sent_kind[0]==0 && sent_value[0]==0);
  assert(sent_kind[1]==1 && sent_value[1]==1);
  assert(sent_kind[2]==1 && sent_value[2]==1999 && sent_kind[3]==1 && sent_value[3]==1999);++cases;
 }
 // Preserve physical output -> logical actuator mapping and true decode timestamps.
 for(unsigned mask=0;mask<16;mask++){
  for(unsigned armed=0;armed<2;armed++){
   DShot d;valid_mask=fresh_mask=mask;ready=4;d._mixing_output.arm.armed=armed;
   for(unsigned i=0;i<4;i++){d.esc_status_pub.data.esc[i].timestamp=123;d.esc_status_pub.data.esc[i].esc_rpm=-77;}
   assert(d.handle_new_bdshot_erpm()==1);
   for(unsigned i=0;i<4;i++){
    auto &r=d.esc_status_pub.data.esc[i];
    if(mask&(1u<<i)){assert(r.timestamp==1000000-10*i && r.esc_rpm==10000*(int)(i+1) && r.actuator_function==d._actuator_functions[i]);}
    else {assert(r.timestamp==123 && r.esc_rpm==-77);}
   }
   d.publish_esc_status();assert(d.esc_status_pub.publications==1);
   assert(d.esc_status_pub.published.esc_armed_flags==(armed?15u:0u));
   assert(d.esc_status_pub.published.esc_online_flags==mask);
   // publish resets the internal online mask after publication, but retained timestamp/value stay intact.
   for(unsigned i=0;i<4;i++)if(!(mask&(1u<<i)))assert(d.esc_status_pub.data.esc[i].timestamp==123 && d.esc_status_pub.data.esc[i].esc_rpm==-77);
   ++cases;
  }
 }
 {DShot d;ready=0;assert(d.handle_new_bdshot_erpm()==0);++cases;}
 {DShot d;ready=4;valid_mask=15;d._param_mot_pole_count.value=0;assert(d.handle_new_bdshot_erpm()==1);++cases;}
 return cases==547?0:2;
}
'''
with tempfile.TemporaryDirectory(prefix='morpheus_flight_driver_') as directory:
    directory = Path(directory)
    (directory / 'test.cpp').write_text(harness)
    subprocess.run(['c++', '-std=c++14', '-O1', '-fsanitize=undefined',
                    '-fno-sanitize-recover=all', str(directory / 'test.cpp'),
                    '-o', str(directory / 'test')], check=True)
    subprocess.run([str(directory / 'test')], check=True)
print(json.dumps({'passed': True, 'driver_cases': 547,
                  'checks': ['stopped_override_for_nonzero_mixer_outputs',
                             'disabled_outputs_send_nothing', 'normal_throttle_and_limits',
                             'physical_output_to_logical_motor_mapping',
                             'capture_processing_timestamps_preserved',
                             'invalid_RPM_not_relabelled_zero_or_fresh',
                             'armed_flags_follow_real_arming', 'invalid_pole_count_no_divide_by_zero'],
                  'hardware_io': False}, indent=2))
