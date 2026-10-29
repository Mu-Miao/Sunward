
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dual_pi_test.h"
#include "lab_uart_async.h"
#include "mpu6050.h"
DWT_Type mock_dwt;CoreDebug_Type mock_core;
static uint32_t ms,tick,bat=1320,io_us=3000;
static int off=1,adc_fail,imu_fail,off_during_read,full_tx;
static int led_on, led1, led2, beep_on, signals, key_input;
static unsigned line_inputs=15, line_reads;
static char oled_lines[4][32];
static int a,b;static int16_t gx,gy,gz=-424;
static char rx[2][1024],logs[262144];static unsigned rp[2],wp[2];static int faults[2];
static TIM_HandleTypeDef l,r;static ADC_HandleTypeDef adc_mock;static UART_HandleTypeDef uart_mock;

static uint64_t trace_hash = UINT64_C(14695981039346656037);
static uint64_t trace_events;
static void trace_bytes(const void *data, size_t size) {
 const unsigned char *bytes=data;
 for(size_t i=0;i<size;i++){trace_hash^=bytes[i];trace_hash*=UINT64_C(1099511628211);}
}
static void trace_event(unsigned char kind) {
 ++trace_events;trace_bytes(&kind,sizeof kind);trace_bytes(&ms,sizeof ms);
 trace_bytes(&mock_dwt.CYCCNT,sizeof mock_dwt.CYCCNT);
}

uint32_t HAL_GetTick(void){return ms;}
uint32_t HAL_RCC_GetHCLKFreq(void){return 72000000U;}
void HAL_Delay(uint32_t n){ms+=n;DWT->CYCCNT+=n*72000U;}
int HAL_GPIO_ReadPin(void*p,uint32_t pin){
 if(p==GPIOA){assert(pin==GPIO_PIN_0);return key_input;}
 if(p==GPIOC&&pin==GPIO_PIN_8){++line_reads;return (line_inputs>>0)&1U;}
 if(p==GPIOC&&pin==GPIO_PIN_4){++line_reads;return (line_inputs>>1)&1U;}
 if(p==GPIOC&&pin==GPIO_PIN_9){++line_reads;return (line_inputs>>2)&1U;}
 if(p==GPIOB&&pin==GPIO_PIN_8){++line_reads;return (line_inputs>>3)&1U;}
 assert(p==GPIOC&&pin==GPIO_PIN_13);return off;
}
void HAL_GPIO_WritePin(void*p,uint32_t pin,int value){
 trace_event('g');unsigned port=p==GPIOC?1U:2U;trace_bytes(&port,sizeof port);trace_bytes(&pin,sizeof pin);trace_bytes(&value,sizeof value);
 if(p==GPIOC){
  assert(pin==GPIO_PIN_2||pin==(GPIO_PIN_2|GPIO_PIN_3));
  led1=value==GPIO_PIN_RESET;
  if(pin&GPIO_PIN_3)led2=value==GPIO_PIN_RESET;
  led_on=led1&&led2;
 }
 else{assert(p==GPIOA&&pin==GPIO_PIN_15);beep_on=value==GPIO_PIN_SET;if(beep_on)signals++;}
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*p){(void)p;return adc_fail?HAL_ERROR:HAL_OK;}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*p,uint32_t n){(void)p;(void)n;return adc_fail?HAL_ERROR:HAL_OK;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*p){(void)p;return bat;}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef*p){(void)p;return HAL_OK;}
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw*p){
 DWT->CYCCNT+=io_us*72U;if(off_during_read)off=1;
 if(imu_fail)return HAL_ERROR;
 *p=(MPU6050_Raw){.gx=gx,.gy=gy,.gz=gz};return HAL_OK;
}
HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef*p){(void)p;return HAL_OK;}
HAL_StatusTypeDef LabUART_BeginPort(LabUART_Port c,UART_HandleTypeDef*p){(void)c;(void)p;return HAL_OK;}
int LabUART_TryWritePort(LabUART_Port c,const char*s,uint16_t n){
 if(n>LAB_UART_FRAME_MAX)return 0; /* Same single-frame rejection as production. */
 trace_event('u');trace_bytes(&c,sizeof c);trace_bytes(&n,sizeof n);trace_bytes(s,n);(void)c;if(full_tx)return 0;assert(strlen(logs)+n<sizeof logs);strncat(logs,s,n);return 1;
}
int LabUART_TryWrite(const char*s,uint16_t n){return LabUART_TryWritePort(LAB_UART_USB,s,n);}
int LabUART_TryReadPort(LabUART_Port c,uint8_t*p){if(rp[c]==wp[c])return 0;*p=(uint8_t)rx[c][rp[c]++];return 1;}
int LabUART_TakeRxFaultPort(LabUART_Port c){int n=faults[c];faults[c]=0;return n;}
uint32_t LabUART_RxErrors(void){return 0;}uint32_t LabUART_TxDropped(void){return full_tx?1:0;}
uint32_t LabUART_RxErrorsPort(LabUART_Port c){(void)c;return 0;}
uint32_t LabUART_TxDroppedPort(LabUART_Port c){(void)c;return full_tx?1:0;}
static void motor(int aa,int bb){assert(aa>=-3600&&aa<=3600&&bb>=-3600&&bb<=3600);trace_event('m');trace_bytes(&aa,sizeof aa);trace_bytes(&bb,sizeof bb);a=aa;b=bb;}
void TaskOLED_Begin(uint32_t n){assert(n==72000000U);}
int TaskOLED_SetLines(const char *const text[4]){
 trace_event('o');for(unsigned i=0;i<4;i++)trace_bytes(text[i],strlen(text[i])+1U);
 for(unsigned i=0;i<4;++i){assert(strlen(text[i])<sizeof oled_lines[i]);strcpy(oled_lines[i],text[i]);}return 1;
}
void TaskOLED_Poll(uint32_t n){(void)n;}
uint32_t TaskOLED_LastIOUs(void){return 100U;}
#include "car_internal.h"
static void trace_poll(uint32_t n) {
 DualPITest_Poll(n);
 trace_event('p');
 trace_bytes(&CarApp_TestContext()->heading.phase,sizeof CarApp_TestContext()->heading.phase);
 trace_bytes(&CarApp_TestContext()->heading.available,sizeof CarApp_TestContext()->heading.available);
 trace_bytes(&CarApp_TestContext()->heading.cal_requested,sizeof CarApp_TestContext()->heading.cal_requested);
 trace_bytes(&CarApp_TestContext()->heading.warm_ms,sizeof CarApp_TestContext()->heading.warm_ms);
 trace_bytes(&CarApp_TestContext()->heading.cycles,sizeof CarApp_TestContext()->heading.cycles);
 trace_bytes(&CarApp_TestContext()->heading.samples,sizeof CarApp_TestContext()->heading.samples);
 trace_bytes(&CarApp_TestContext()->heading.io_us,sizeof CarApp_TestContext()->heading.io_us);
 trace_bytes(&CarApp_TestContext()->heading.sum,sizeof CarApp_TestContext()->heading.sum);
 trace_bytes(&CarApp_TestContext()->heading.min,sizeof CarApp_TestContext()->heading.min);
 trace_bytes(&CarApp_TestContext()->heading.max,sizeof CarApp_TestContext()->heading.max);
 trace_bytes(&CarApp_TestContext()->heading.bias,sizeof CarApp_TestContext()->heading.bias);
 trace_bytes(&CarApp_TestContext()->heading.angle,sizeof CarApp_TestContext()->heading.angle);
 trace_bytes(&CarApp_TestContext()->heading.previous_rate,sizeof CarApp_TestContext()->heading.previous_rate);
 trace_bytes(&CarApp_TestContext()->heading.reference,sizeof CarApp_TestContext()->heading.reference);
 trace_bytes(&CarApp_TestContext()->heading.error,sizeof CarApp_TestContext()->heading.error);
 trace_bytes(&CarApp_TestContext()->heading.diff,sizeof CarApp_TestContext()->heading.diff);
 trace_bytes(&CarApp_TestContext()->button.action,sizeof CarApp_TestContext()->button.action);
 trace_bytes(&CarApp_TestContext()->button.raw,sizeof CarApp_TestContext()->button.raw);
 trace_bytes(&CarApp_TestContext()->button.stable,sizeof CarApp_TestContext()->button.stable);
 trace_bytes(&CarApp_TestContext()->button.armed,sizeof CarApp_TestContext()->button.armed);
 trace_bytes(&CarApp_TestContext()->button.press_enabled,sizeof CarApp_TestContext()->button.press_enabled);
 trace_bytes(&CarApp_TestContext()->button.off_pending,sizeof CarApp_TestContext()->button.off_pending);
 trace_bytes(&CarApp_TestContext()->button.press_busy,sizeof CarApp_TestContext()->button.press_busy);
 trace_bytes(&CarApp_TestContext()->button.change_ms,sizeof CarApp_TestContext()->button.change_ms);
 trace_bytes(&CarApp_TestContext()->button.events,sizeof CarApp_TestContext()->button.events);
 trace_bytes(&CarApp_TestContext()->button.hold_ms,sizeof CarApp_TestContext()->button.hold_ms);
 trace_bytes(&CarApp_TestContext()->button.selected_motion,sizeof CarApp_TestContext()->button.selected_motion);
 trace_bytes(&CarApp_TestContext()->line.raw,sizeof CarApp_TestContext()->line.raw);
 trace_bytes(&CarApp_TestContext()->line.black,sizeof CarApp_TestContext()->line.black);
 trace_bytes(&CarApp_TestContext()->line.valid,sizeof CarApp_TestContext()->line.valid);
 trace_bytes(&CarApp_TestContext()->line.count,sizeof CarApp_TestContext()->line.count);
 trace_bytes(&CarApp_TestContext()->line.center_seen,sizeof CarApp_TestContext()->line.center_seen);
 trace_bytes(&CarApp_TestContext()->line.center_ready,sizeof CarApp_TestContext()->line.center_ready);
 trace_bytes(&CarApp_TestContext()->line.invalid_seen,sizeof CarApp_TestContext()->line.invalid_seen);
 trace_bytes(&CarApp_TestContext()->line.center_ms,sizeof CarApp_TestContext()->line.center_ms);
 trace_bytes(&CarApp_TestContext()->line.sample_ms,sizeof CarApp_TestContext()->line.sample_ms);
 trace_bytes(&CarApp_TestContext()->line.invalid_ms,sizeof CarApp_TestContext()->line.invalid_ms);
 trace_bytes(&CarApp_TestContext()->line.error,sizeof CarApp_TestContext()->line.error);
 trace_bytes(&CarApp_TestContext()->line.diff,sizeof CarApp_TestContext()->line.diff);
 trace_bytes(&CarApp_TestContext()->route.motion,sizeof CarApp_TestContext()->route.motion);
 trace_bytes(&CarApp_TestContext()->route.distance_mm,sizeof CarApp_TestContext()->route.distance_mm);
 trace_bytes(&CarApp_TestContext()->route.base,sizeof CarApp_TestContext()->route.base);
 trace_bytes(&CarApp_TestContext()->route.goal,sizeof CarApp_TestContext()->route.goal);
 trace_bytes(&CarApp_TestContext()->route.cruise,sizeof CarApp_TestContext()->route.cruise);
 trace_bytes(&CarApp_TestContext()->route.slow,sizeof CarApp_TestContext()->route.slow);
 trace_bytes(&CarApp_TestContext()->route.distance_count,sizeof CarApp_TestContext()->route.distance_count);
 trace_bytes(&CarApp_TestContext()->route.tracking,sizeof CarApp_TestContext()->route.tracking);
 trace_bytes(&CarApp_TestContext()->route.settle_seen,sizeof CarApp_TestContext()->route.settle_seen);
 trace_bytes(&CarApp_TestContext()->route.brake_ms,sizeof CarApp_TestContext()->route.brake_ms);
 trace_bytes(&CarApp_TestContext()->route.settle_ms,sizeof CarApp_TestContext()->route.settle_ms);
 trace_bytes(&CarApp_TestContext()->route.manual_duration,sizeof CarApp_TestContext()->route.manual_duration);
 trace_bytes(&CarApp_TestContext()->route.run_duration,sizeof CarApp_TestContext()->route.run_duration);
 trace_bytes(&CarApp_TestContext()->route.switch_ms,sizeof CarApp_TestContext()->route.switch_ms);
 trace_bytes(&CarApp_TestContext()->route.start_ms,sizeof CarApp_TestContext()->route.start_ms);
 trace_bytes(&CarApp_TestContext()->route.manual_speed,sizeof CarApp_TestContext()->route.manual_speed);
 trace_bytes(&CarApp_TestContext()->route.switching,sizeof CarApp_TestContext()->route.switching);
 trace_bytes(&CarApp_TestContext()->route.targets,sizeof CarApp_TestContext()->route.targets);
 trace_bytes(&CarApp_TestContext()->route.leg,sizeof CarApp_TestContext()->route.leg);
 trace_bytes(&CarApp_TestContext()->route.edge_seen,sizeof CarApp_TestContext()->route.edge_seen);
 trace_bytes(&CarApp_TestContext()->route.edge_ms,sizeof CarApp_TestContext()->route.edge_ms);
 trace_bytes(&CarApp_TestContext()->route.elapsed_ms,sizeof CarApp_TestContext()->route.elapsed_ms);
 trace_bytes(&CarApp_TestContext()->route.leg_start,sizeof CarApp_TestContext()->route.leg_start);
 trace_bytes(&CarApp_TestContext()->route.start_heading,sizeof CarApp_TestContext()->route.start_heading);
 trace_bytes(&CarApp_TestContext()->power.valid,sizeof CarApp_TestContext()->power.valid);
 trace_bytes(&CarApp_TestContext()->power.sample_ms,sizeof CarApp_TestContext()->power.sample_ms);
 trace_bytes(&CarApp_TestContext()->power.raw,sizeof CarApp_TestContext()->power.raw);
 trace_bytes(&CarApp_TestContext()->signal.active,sizeof CarApp_TestContext()->signal.active);
 trace_bytes(&CarApp_TestContext()->signal.ready_led_on,sizeof CarApp_TestContext()->signal.ready_led_on);
 trace_bytes(&CarApp_TestContext()->signal.ms,sizeof CarApp_TestContext()->signal.ms);
 trace_bytes(&CarApp_TestContext()->ui.render_ms,sizeof CarApp_TestContext()->ui.render_ms);
 trace_bytes(&CarApp_TestContext()->ui.oled_log_ms,sizeof CarApp_TestContext()->ui.oled_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.line_log_ms,sizeof CarApp_TestContext()->ui.line_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.heading_log_ms,sizeof CarApp_TestContext()->ui.heading_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.button_log_ms,sizeof CarApp_TestContext()->ui.button_log_ms);
 trace_bytes(&CarApp_TestContext()->ui.motion_log_ms,sizeof CarApp_TestContext()->ui.motion_log_ms);
 trace_bytes(&CarApp_TestContext()->runtime.state,sizeof CarApp_TestContext()->runtime.state);
 trace_bytes(&CarApp_TestContext()->runtime.active,sizeof CarApp_TestContext()->runtime.active);
 trace_bytes(&CarApp_TestContext()->runtime.primed,sizeof CarApp_TestContext()->runtime.primed);
 trace_bytes(&CarApp_TestContext()->runtime.on_seen,sizeof CarApp_TestContext()->runtime.on_seen);
 trace_bytes(&CarApp_TestContext()->runtime.bluetooth_on,sizeof CarApp_TestContext()->runtime.bluetooth_on);
 trace_bytes(&CarApp_TestContext()->runtime.hz,sizeof CarApp_TestContext()->runtime.hz);
 trace_bytes(&CarApp_TestContext()->runtime.old_cycles,sizeof CarApp_TestContext()->runtime.old_cycles);
 trace_bytes(&CarApp_TestContext()->runtime.old_tick,sizeof CarApp_TestContext()->runtime.old_tick);
 trace_bytes(&CarApp_TestContext()->runtime.old_ms,sizeof CarApp_TestContext()->runtime.old_ms);
 trace_bytes(&CarApp_TestContext()->runtime.on_ms,sizeof CarApp_TestContext()->runtime.on_ms);
 trace_bytes(&CarApp_TestContext()->runtime.dt_us,sizeof CarApp_TestContext()->runtime.dt_us);
 trace_bytes(&CarApp_TestContext()->runtime.fault_wheel,sizeof CarApp_TestContext()->runtime.fault_wheel);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].old_count,sizeof CarApp_TestContext()->wheel.items[i].old_count);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].velocity,sizeof CarApp_TestContext()->wheel.items[i].velocity);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].integral,sizeof CarApp_TestContext()->wheel.items[i].integral);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].command,sizeof CarApp_TestContext()->wheel.items[i].command);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].bad_sign_us,sizeof CarApp_TestContext()->wheel.items[i].bad_sign_us);
 for(unsigned i=0;i<2;i++)trace_bytes(&CarApp_TestContext()->wheel.items[i].stall_us,sizeof CarApp_TestContext()->wheel.items[i].stall_us);
 for(unsigned i=0;i<LAB_UART_COUNT;i++){trace_bytes(CarApp_TestContext()->command.inputs[i].text,sizeof CarApp_TestContext()->command.inputs[i].text);trace_bytes(&CarApp_TestContext()->command.inputs[i].length,sizeof CarApp_TestContext()->command.inputs[i].length);trace_bytes(&CarApp_TestContext()->command.inputs[i].discard,sizeof CarApp_TestContext()->command.inputs[i].discard);trace_bytes(&CarApp_TestContext()->command.inputs[i].ms,sizeof CarApp_TestContext()->command.inputs[i].ms);}
}

static void setup_key(int held){
 ms=tick=0;memset(&mock_dwt,0,sizeof mock_dwt);memset(&mock_core,0,sizeof mock_core);
 l.count=r.count=0;memset(rx,0,sizeof rx);memset(rp,0,sizeof rp);memset(wp,0,sizeof wp);memset(faults,0,sizeof faults);
 logs[0]=0;signals=led_on=led1=led2=beep_on=0;key_input=held;off=1;adc_fail=imu_fail=off_during_read=full_tx=0;io_us=3000;bat=1320;gx=gy=0;gz=-424;
 assert(DualPITest_Begin(&uart_mock,&l,&r,&adc_mock,motor)==HAL_OK);
 DualPITest_EnableHeading(1);
 trace_poll(tick);
}
static void setup(void){setup_key(0);}
static void step(unsigned n,int dl,int dr){
 /* Each call is scheduled from the preceding sample start, not I2C completion. */
 ms+=n;DWT->CYCCNT=CarApp_TestContext()->runtime.old_cycles+n*72000U;tick+=n/10U;
 l.count=(l.count+dl)&65535U;r.count=(r.count-dr)&65535U;
 trace_poll(tick);
}
/* 按当前目标速度补编码器计数，避免长等待被 500ms 堵转保护误停。 */
static void step_roll(void){
 int counts[2];
 for(unsigned w=0;w<2;w++){
  float target=fabsf(CarApp_TestContext()->route.targets[w]);
  counts[w]=(int)(target*.01f*(w?DPT_RIGHT_CPR:DPT_LEFT_CPR)/(DPT_CIRC_M*1000.0f)+.5f);
 }
 step(10,counts[0],counts[1]);
}
static void queue(unsigned c,const char*s){size_t n=strlen(s);assert(wp[c]+n<sizeof rx[c]);memcpy(rx[c]+wp[c],s,n);wp[c]+=(unsigned)n;}
static void cmd(const char*s){queue(0,s);trace_poll(tick);}
static void enable(void){off=0;for(int i=0;i<20;i++)step(10,0,0);}
static void calibrate(void){cmd("CAL!");for(int i=0;i<705;i++)step(10,0,0);assert(CarApp_TestContext()->heading.phase==Y_READY&&CarApp_TestContext()->heading.samples==500&&fabsf(CarApp_TestContext()->heading.bias+424)<.001f&&a==0&&b==0);}
static void head(void){enable();cmd("GO!");step(10,0,0);assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_HEADING&&a==1800&&b==-1800);}
static void release_key(void){key_input=0;for(int i=0;i<4;i++)step(10,0,0);}
static void press_key(void){key_input=1;for(int i=0;i<4;i++)step(10,0,0);}

static float progress(void){return (CarApp_TestContext()->route.distance_mm[0]+CarApp_TestContext()->route.distance_mm[1])*.5f-CarApp_TestContext()->route.leg_start;}
static int ideal_counts;
static float ideal_residue[2];
static void hold(void){
 release_key();key_input=1;for(int i=0;i<154;i++)step(10,0,0);release_key();
 assert(a==0&&b==0&&CarApp_TestContext()->button.action==KEY_MODE&&CarApp_TestContext()->heading.phase!=Y_WARM&&CarApp_TestContext()->heading.phase!=Y_CAL);
}
static void task_start(void){
 setup();line_inputs=15;hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_LINE);
 hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_TASK2);release_key();press_key();release_key();
 for(int i=0;i<705;i++)step(10,0,0);
 assert(CarApp_TestContext()->heading.phase==Y_READY);enable();press_key();
 assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.motion==MOVE_TASK2&&CarApp_TestContext()->route.leg==0&&!CarApp_TestContext()->runtime.bluetooth_on);
 assert(wp[0]==0&&wp[1]==0&&CarApp_TestContext()->route.distance_mm[0]==0&&CarApp_TestContext()->route.elapsed_ms==0);
}
static void endpoint(unsigned leg){
 assert(CarApp_TestContext()->route.leg==leg&&CarApp_TestContext()->runtime.state==RUNNING);
 float goal=leg%2?DPT_TASK2_ARC_MM:DPT_TASK2_STRAIGHT_MM;
 while(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.leg==leg){
  float prog=progress();
  if(leg%2){
   CarApp_TestContext()->heading.angle=CarMath_WrapDegrees(CarApp_TestContext()->route.start_heading-180.0f*(float)(leg/2+prog/DPT_TASK2_ARC_MM));
   line_inputs=prog>=goal?15:9;
  }else{
   CarApp_TestContext()->heading.angle=leg==0?CarApp_TestContext()->route.start_heading:CarMath_WrapDegrees(CarApp_TestContext()->route.start_heading+180);
   line_inputs=prog>=goal?9:15;
  }
  if(ideal_counts){
   int counts[2];
   for(unsigned w=0;w<2;w++){
    float target=fabsf(CarApp_TestContext()->route.targets[w]);
    ideal_residue[w]+=target*.01f*(w?DPT_RIGHT_CPR:DPT_LEFT_CPR)/(DPT_CIRC_M*1000.0f);
    counts[w]=(int)ideal_residue[w];ideal_residue[w]-=(float)counts[w];
   }
   step(10,counts[0],counts[1]);
  }else step(10,610,610);
  /* 出口桥接速度很低（捕线速度），两轮目标仍须前进；本测试的固定 610 计数
     远高于桥接目标，PI 可能恰好输出 0，不能当作停车判据。 */
  if(CarApp_TestContext()->route.edge_seen&&(leg%2))
   assert(CarApp_TestContext()->route.targets[0]>0&&CarApp_TestContext()->route.targets[1]>0);
 }
 assert(leg==3?CarApp_TestContext()->runtime.state==BRAKING:CarApp_TestContext()->runtime.state==RUNNING);
 assert(CarApp_TestContext()->route.leg==(leg==3?3:leg+1));
}
static void fast_to_arc(void){
 task_start();endpoint(0);assert(CarRoute_IsArc(CarApp_TestContext()));
 /* 入弧本拍已按任务四低速接管；喂中间双探头 200ms+60ms 后应提回常速。 */
 for(int i=0;i<26;i++)step_roll();
 assert(!CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->route.edge_follow_resume);
}
/* 停在“刚入弧、仍在低速跟随”的状态，用于跟随本身与边界测试。 */
static void to_arc_entry(void){
 task_start();
 CarApp_TestContext()->route.start_ms=ms-5000U;
 CarApp_TestContext()->route.distance_mm[0]=CarApp_TestContext()->route.distance_mm[1]=
  CarApp_TestContext()->route.leg_start+DPT_TASK2_STRAIGHT_MM-10;
 line_inputs=9;
 for(int i=0;i<4;i++)step(10,0,0);
 assert(CarApp_TestContext()->route.leg==1&&CarRoute_IsArc(CarApp_TestContext()));
 assert(CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->route.base==DPT_TASK2_EDGE_FOLLOW_MMPS);
}
static void near_arc_end(void){
 fast_to_arc();CarApp_TestContext()->route.distance_mm[0]=CarApp_TestContext()->route.distance_mm[1]=CarApp_TestContext()->route.leg_start+DPT_TASK2_ARC_MM-10;
 CarApp_TestContext()->heading.angle=CarMath_WrapDegrees(CarApp_TestContext()->route.start_heading+180);
}
/* Continuous handoff must keep both wheels driving through the arc-exit debounce. */
static void ideal_lap(void){
 task_start();ideal_counts=1;ideal_residue[0]=ideal_residue[1]=0;
 uint32_t began=CarApp_TestContext()->route.start_ms;
 assert(CarApp_TestContext()->route.cruise==DPT_TASK2_STRAIGHT_MMPS);
 endpoint(0);assert(CarApp_TestContext()->route.cruise==DPT_TASK2_ARC_MMPS);
 endpoint(1);assert(CarApp_TestContext()->route.cruise==DPT_TASK2_STRAIGHT_MMPS);
 endpoint(2);assert(CarApp_TestContext()->route.cruise==DPT_TASK2_ARC_MMPS);
 endpoint(3);
 for(int i=0;i<21;i++)step(10,0,0);
 assert(CarApp_TestContext()->runtime.state==DONE&&CarApp_TestContext()->route.start_ms==began);
 printf("IDEAL Task2 one lap: %lu ms; target-following/160mm track, not field acceptance.\n",(unsigned long)CarApp_TestContext()->route.elapsed_ms);
 /* 验收上限 30 秒；理想目标跟随含每个弧 200ms 低速接管与 1.7 秒提速斜坡。 */
 assert(CarApp_TestContext()->route.elapsed_ms<24000);
 ideal_counts=0;
}
static void pwm_cap(void){
 setup();
 CarApp_TestContext()->route.motion=MOVE_TASK2;CarApp_TestContext()->route.targets[0]=300;
 CarApp_TestContext()->wheel.items[0].velocity=0;CarApp_TestContext()->wheel.items[0].integral=0;
 CarWheel_Control(CarApp_TestContext(),0,.01f);
 assert(CarApp_TestContext()->wheel.items[0].command==DPT_TASK2_MAX_CMD);
 CarApp_TestContext()->route.motion=MOVE_ROUTE;CarApp_TestContext()->wheel.items[0].integral=0;
 CarWheel_Control(CarApp_TestContext(),0,.01f);
 assert(CarApp_TestContext()->wheel.items[0].command==DPT_ROUTE_MAX_CMD);
 CarApp_TestContext()->route.motion=MOVE_DISTANCE;CarApp_TestContext()->wheel.items[0].integral=0;
 CarWheel_Control(CarApp_TestContext(),0,.01f);
 assert(CarApp_TestContext()->wheel.items[0].command==DPT_MAX_CMD);
 assert(DPT_TASK2_MAX_CMD>DPT_MAX_CMD&&CarWheel_ConfigValid());
}
/* Entering a slower leg scales both integrals instead of clearing them. */
static void retarget_regression(void){
 task_start();
 CarApp_TestContext()->route.start_ms=ms-5000U; /* past the Task2 ramp */
 CarApp_TestContext()->route.distance_mm[0]=CarApp_TestContext()->route.distance_mm[1]=
  CarApp_TestContext()->route.leg_start+DPT_TASK2_STRAIGHT_MM-10;
 CarApp_TestContext()->route.base=400;
 CarApp_TestContext()->route.targets[0]=CarApp_TestContext()->route.targets[1]=400;
 for(unsigned w=0;w<2;w++)CarApp_TestContext()->wheel.items[w].integral=3500;
 line_inputs=9;
 for(int i=0;i<4;i++)step(10,0,0);
 assert(CarApp_TestContext()->route.leg==1&&CarApp_TestContext()->route.cruise==DPT_TASK2_ARC_MMPS);
 for(unsigned w=0;w<2;w++){
  assert(fabsf(CarApp_TestContext()->wheel.items[w].integral-(3500.0f*DPT_TASK2_EDGE_FOLLOW_MMPS/DPT_TASK2_STRAIGHT_MMPS + DPT_LEFT_KI*DPT_TASK2_EDGE_FOLLOW_MMPS*.01f))<.01f);
  assert(CarApp_TestContext()->wheel.items[w].command>0 && CarApp_TestContext()->wheel.items[w].command<DPT_TASK2_MAX_CMD);
 }
}
/* Task4 parity: capture at arc entry takes over at 80mm/s at once, the
   outermost probe pins +-60, all-white freezes it at 50mm/s, and a stable
   inner line promotes back to the arc cruise. */
static void edge_follow_port(void){
 to_arc_entry();
 line_inputs=7; /* 最左单探头：固定 -60 差速，两轮仍前进。 */
 step_roll();
 assert(CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->route.base==DPT_TASK2_EDGE_FOLLOW_MMPS);
 assert(CarApp_TestContext()->route.targets[0]==DPT_TASK2_EDGE_FOLLOW_MMPS-DPT_TASK2_EDGE_FOLLOW_DIFF_MMPS);
 assert(CarApp_TestContext()->route.targets[1]==DPT_TASK2_EDGE_FOLLOW_MMPS+DPT_TASK2_EDGE_FOLLOW_DIFF_MMPS);
 line_inputs=15; /* 全白冻结 -60，降到捕线速度 50 并夹到内轮 20。 */
 step_roll();
 assert(CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->runtime.state==RUNNING);
 assert(CarApp_TestContext()->route.base==DPT_TASK2_EDGE_FOLLOW_CAPTURE_MMPS);
 assert(CarApp_TestContext()->route.targets[0]==DPT_TASK4_TURN_INNER_MMPS);
 assert(CarApp_TestContext()->route.targets[1]==2*DPT_TASK2_EDGE_FOLLOW_CAPTURE_MMPS-DPT_TASK4_TURN_INNER_MMPS);
 for(int i=0;i<9;i++)step_roll(); /* 100ms，未到 500ms/20mm 桥接预算。 */
 assert(CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->runtime.state==RUNNING);
 line_inputs=9; /* 回中稳定：30ms 恢复 + 60ms 退出 + 200ms 最短跟随。 */
 int ticks=0;
 while(CarApp_TestContext()->route.edge_follow&&ticks++<40)step_roll();
 assert(!CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->route.edge_follow_resume);
 assert(CarApp_TestContext()->route.base>=DPT_TASK2_EDGE_FOLLOW_MMPS&&CarApp_TestContext()->route.base<DPT_TASK2_ARC_MMPS);
 ticks=0;
 while(CarApp_TestContext()->route.edge_follow_resume&&ticks++<250)step_roll();
 assert(!CarApp_TestContext()->route.edge_follow_resume&&CarApp_TestContext()->route.base==DPT_TASK2_ARC_MMPS);
 line_inputs=14; /* 已提回巡线后最右单探头只按 160 边缘降速，不再回到 70。 */
 for(int i=0;i<10;i++)step_roll();
 assert(!CarApp_TestContext()->route.edge_follow&&CarApp_TestContext()->runtime.state==RUNNING);
 assert(CarApp_TestContext()->route.base==DPT_TASK2_EDGE_MMPS);
 assert(CarApp_TestContext()->route.targets[0]==DPT_TASK2_EDGE_MMPS+DPT_LINE_MAX_DIFF*DPT_TASK2_EDGE_MMPS/DPT_LINE_CRUISE_MMPS);
 assert(CarApp_TestContext()->route.targets[1]==DPT_TASK2_EDGE_MMPS-DPT_LINE_MAX_DIFF*DPT_TASK2_EDGE_MMPS/DPT_LINE_CRUISE_MMPS);
}
/* Task4 parity: outermost-only creep is bounded at 120mm and faults. */
static void edge_follow_limit(void){
 to_arc_entry();
 line_inputs=7;
 step_roll();
 CarApp_TestContext()->route.distance_mm[0]=CarApp_TestContext()->route.distance_mm[1]=
  CarApp_TestContext()->route.edge_follow_start_mm+DPT_TASK2_EDGE_FOLLOW_MM;
 step_roll();
 assert(CarApp_TestContext()->runtime.state==LINE_BAD&&CarApp_TestContext()->route.leg==1);
 assert(!CarApp_TestContext()->route.edge_follow&&a==0&&b==0);
}
int main(void){
 /* Independent raw-pattern truth table retains the tuned LF polarity and signs. */
 const int valid[16]={0,1,0,1,0,0,0,1,1,1,0,1,1,1,1,0};
 const float left_at200[16]={0,136,0,120,0,0,0,120,264,200,0,136,280,264,280,0};
 for(unsigned mask=0;mask<16;mask++){
  setup();line_inputs=mask;CarLine_Decode(CarApp_TestContext());assert(CarApp_TestContext()->line.valid==valid[mask]);
  if(valid[mask]){
   CarLine_SetTargets(CarApp_TestContext(), ms,200);assert(fabsf(CarApp_TestContext()->route.targets[0]-left_at200[mask])<.001f);
   assert(fabsf(CarApp_TestContext()->route.targets[1]-(400-left_at200[mask]))<.001f);
   CarLine_SetTargets(CarApp_TestContext(), ms,50);assert(CarApp_TestContext()->route.targets[0]>=30&&CarApp_TestContext()->route.targets[1]>=30&&CarApp_TestContext()->route.targets[0]<=70&&CarApp_TestContext()->route.targets[1]<=70);
  }
 }
 /* Reported BC exit S1256: new endpoint window hands off to CD instead of LOST. */
 fast_to_arc();CarApp_TestContext()->route.distance_mm[0]=CarApp_TestContext()->route.distance_mm[1]=CarApp_TestContext()->route.leg_start+1256.0f;
 CarApp_TestContext()->heading.angle=CarMath_WrapDegrees(CarApp_TestContext()->route.start_heading+180.0f);line_inputs=15;
 for(int i=0;i<4;i++)step(10,0,0);
 assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.leg==2&&signals==2);
 /* Full GPIO-only route crosses white straights and black arcs exactly once. */
 task_start();uint32_t began=CarApp_TestContext()->route.start_ms;
 endpoint(0);assert(signals==1&&beep_on&&led_on);
 assert(CarRoute_UsesLine(CarApp_TestContext())&&!CarRoute_UsesHeading(CarApp_TestContext()));
 endpoint(1);assert(signals==2&&CarApp_TestContext()->route.leg==2&&CarRoute_UsesHeading(CarApp_TestContext())&&!CarRoute_UsesLine(CarApp_TestContext()));
 assert(CarApp_TestContext()->route.task2_straight.seen && fabsf(CarMath_WrapDegrees(CarApp_TestContext()->heading.reference-CarApp_TestContext()->route.task2_straight.entry_yaw))<.001f);
 endpoint(2);assert(signals==3&&CarApp_TestContext()->route.leg==3&&CarRoute_UsesLine(CarApp_TestContext()));
 endpoint(3);assert(signals==3&&!beep_on&&a==0&&b==0);
 for(int i=0;i<21;i++)step(10,0,0);
 assert(CarApp_TestContext()->runtime.state==DONE&&signals==4&&CarApp_TestContext()->signal.active&&led_on&&beep_on);
 assert(CarApp_TestContext()->route.start_ms==began&&CarApp_TestContext()->route.elapsed_ms==(uint32_t)(ms-began));
 assert(CarApp_TestContext()->route.elapsed_ms<30000&&rp[0]==0&&wp[0]==0&&!CarApp_TestContext()->runtime.bluetooth_on);
 for(int i=0;i<50;i++)step(10,0,0);
 assert(CarApp_TestContext()->runtime.state==DONE&&!CarApp_TestContext()->signal.active&&signals==4);
 for(unsigned i=0;i<4;i++)assert(strlen(oled_lines[i])<=21);
 assert(strcmp(oled_lines[0],"TASK 2 V29 GAP")==0&&strstr(oled_lines[2],"DONE DA T"));
 /* Competition PWM headroom; GO20 retains1800. */
 pwm_cap();
 /* Straight->arc handoff keeps PI state, scaled to the slower target. */
 retarget_regression();
 /* Arc entry capture takes over at 80mm/s; Task4 bridge/promotion/bounds. */
 edge_follow_port();
 edge_follow_limit();
 /* Continuous legs at Task4-class speeds must finish one lap well under 30s. */
 ideal_lap();
 /* Repeated same-route request must not reset distance/timing while running. */
 task_start();uint32_t saved=CarApp_TestContext()->route.start_ms;cmd("TASK2!");assert(CarApp_TestContext()->route.start_ms==saved&&CarApp_TestContext()->route.leg==0);
 /* Modes cycle 1 -> half -> 2 -> 1; reboot is still task 1. */
 setup();hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_LINE);hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_TASK2);
 for(int i=0;i<200;i++)step(10,0,0);assert(CarApp_TestContext()->button.selected_motion==MOVE_TASK2&&CarApp_TestContext()->heading.phase==Y_NONE);
 hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_TASK3);hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_TASK4);hold();assert(CarApp_TestContext()->button.selected_motion==MOVE_ROUTE);setup();assert(CarApp_TestContext()->button.selected_motion==MOVE_ROUTE);
 /* Half-circle user's current tuned parameters and independent auto-stop survive. */
 setup();line_inputs=9;cmd("LF20!");calibrate();enable();press_key();
 assert(CarApp_TestContext()->route.motion==MOVE_LINE&&CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.goal==DPT_LINE_MM);
 assert(CarApp_TestContext()->route.cruise==200&&CarApp_TestContext()->route.run_duration==DPT_LINE_TIMEOUT_MS);
 line_inputs=11;step(10,0,0);assert(CarApp_TestContext()->route.targets[0]<CarApp_TestContext()->route.targets[1]);line_inputs=13;step(10,0,0);assert(CarApp_TestContext()->route.targets[0]>CarApp_TestContext()->route.targets[1]);
 line_inputs=9;for(int i=0;i<1500&&CarApp_TestContext()->runtime.state==RUNNING;i++)step(10,600,600);
 assert(CarApp_TestContext()->runtime.state==BRAKING);for(int i=0;i<21;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state==DONE&&signals==1);
 /* Early black on a white straight cannot prematurely declare B. */
 task_start();line_inputs=9;for(int i=0;i<50;i++)step(10,600,600);
 assert(CarApp_TestContext()->route.leg==0&&signals==0&&CarApp_TestContext()->runtime.state==RUNNING);
 /* Missing arc entrance cannot keep driving indefinitely or falsely beep. */
 task_start();line_inputs=15;
 while(CarApp_TestContext()->runtime.state==RUNNING)step(10,600,600);
 assert(CarApp_TestContext()->runtime.state==LINE_LOST&&CarApp_TestContext()->route.leg==0&&signals==0&&a==0&&b==0);
 /* Mid-arc white/all-black/split are faults, not C/DA endpoints. */
 const unsigned bad[]={15,0,6};
 for(unsigned k=0;k<3;k++){
  fast_to_arc();line_inputs=bad[k];step(10,0,0);
  /* 全白按任务四桥接继续前进；全黑/分裂立即停输出等预算。 */
  if(k==0)assert(a>0&&b<0);else assert(a==0&&b==0);
  assert(CarApp_TestContext()->runtime.state==RUNNING);
  int ticks=0;
  while(CarApp_TestContext()->runtime.state==RUNNING&&ticks++<80)step_roll();
  assert(CarApp_TestContext()->runtime.state==(k?LINE_BAD:LINE_LOST)&&CarApp_TestContext()->route.leg==1&&signals==1&&!CarApp_TestContext()->signal.active);
  line_inputs=9;for(int i=0;i<30;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state!=RUNNING&&a==0&&b==0);
 }
 /* Near-end white only succeeds with the expected 180-degree heading. */
 near_arc_end();CarApp_TestContext()->heading.angle=CarApp_TestContext()->route.start_heading;line_inputs=15;
 int ticks=0;
 while(CarApp_TestContext()->runtime.state==RUNNING&&ticks++<80)step_roll();
 assert(CarApp_TestContext()->runtime.state==LINE_LOST&&signals==1&&CarApp_TestContext()->route.leg==1);
 near_arc_end();line_inputs=15;step(10,0,0);
 assert(CarApp_TestContext()->route.edge_seen&&CarApp_TestContext()->runtime.state==RUNNING&&a>0&&b<0);
 assert(CarApp_TestContext()->route.targets[0]>0&&CarApp_TestContext()->route.targets[1]>0);
 for(int i=0;i<3;i++)step(10,0,0);assert(CarApp_TestContext()->route.leg==2&&CarApp_TestContext()->runtime.state==RUNNING&&signals==2);
 /* Exit bounce resets the stable-white timer, not an endpoint. */
 near_arc_end();line_inputs=15;step(10,0,0);step(10,0,0);line_inputs=9;step(10,0,0);
 assert(CarApp_TestContext()->route.leg==1&&!CarApp_TestContext()->route.edge_seen&&signals==1);
 /* Never drive beyond a missing exit line: bad geometry is bounded. */
 fast_to_arc();line_inputs=9;CarApp_TestContext()->route.distance_mm[0]=CarApp_TestContext()->route.distance_mm[1]=CarApp_TestContext()->route.leg_start+DPT_TASK2_ARC_MM+DPT_TASK2_EXIT_WINDOW_MM+1;
 step(10,0,0);assert(CarApp_TestContext()->runtime.state==LINE_BAD&&a==0&&b==0&&signals==1);
 /* STOP/OFF/RX faults beat same-poll edge transitions and motor commits. */
 for(unsigned leg=0;leg<4;leg++)for(int fault=0;fault<6;fault++){
  task_start();for(unsigned n=0;n<leg;n++)endpoint(n);
  release_key();int sig=signals;
  if(fault==0){off=1;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==OFF);}
  if(fault==1){press_key();assert(CarApp_TestContext()->runtime.state==STOPPED);}
  if(fault==2){cmd("STOP!TASK2!");assert(CarApp_TestContext()->runtime.state==STOPPED);}
  if(fault==3){faults[0]=1;step(10,0,0);assert(CarApp_TestContext()->runtime.state==UART_BAD);}
  if(fault==4){imu_fail=1;step(10,0,0);assert(CarApp_TestContext()->runtime.state==YAW_BAD);}
  if(fault==5){off_during_read=1;step(10,0,0);assert(CarApp_TestContext()->runtime.state==OFF);}
  assert(a==0&&b==0&&signals==sig&&!CarApp_TestContext()->signal.active&&!CarApp_TestContext()->route.tracking);
 }
 /* Both serial-port STOP/RX faults outrank simultaneous task2 selection/start. */
 for(unsigned port=0;port<2;port++)for(int fault=0;fault<2;fault++){
  task_start();assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_ERROR);
  cmd("STOP!");assert(DualPITest_EnableBluetooth(&uart_mock)==HAL_OK);
  if(fault)faults[port]=1;else queue(port,"STOP!");queue(1-port,"TASK2!");
  trace_poll(tick);assert(CarApp_TestContext()->runtime.state==(fault?UART_BAD:STOPPED)&&a==0&&b==0);
 }
 /* Full TX only drops telemetry and cannot change task2 sequencing. */
 fast_to_arc();full_tx=1;line_inputs=9;step(10,600,600);assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.leg==1);
 /* A new key restarts from A state after a fault, not from its interrupted leg. */
 fast_to_arc();line_inputs=15;int lost=0;
 while(CarApp_TestContext()->runtime.state==RUNNING&&lost++<80)step_roll();
 assert(CarApp_TestContext()->runtime.state==LINE_LOST);
 release_key();press_key();assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.leg==0&&CarApp_TestContext()->route.distance_mm[0]==0);
 /* Heading guard applies on straights, is disabled during intended arcs. */
 task_start();CarApp_TestContext()->heading.angle=45;step(10,0,0);assert(CarApp_TestContext()->runtime.state==YAW_BAD);
 fast_to_arc();CarApp_TestContext()->heading.angle=45;step(10,600,600);assert(CarApp_TestContext()->runtime.state==RUNNING);
 /* Timeout/FAST/GAP/LOW/ADC/STALL/REV still latch; no false finish. */
 for(int fault=0;fault<7;fault++){
  task_start();
  if(fault==0){CarApp_TestContext()->route.start_ms=ms-DPT_TASK2_TIMEOUT_MS;trace_poll(tick);assert(CarApp_TestContext()->runtime.state==TIMEOUT);}
  if(fault==1){step(10,2000,100);assert(CarApp_TestContext()->runtime.state==FAST);}
  if(fault==2){step(20,100,100);assert(CarApp_TestContext()->runtime.state==GAP);}
  if(fault==3){bat=1100;for(int i=0;i<10;i++)step(10,100,100);assert(CarApp_TestContext()->runtime.state==LOW);}
  if(fault==4){adc_fail=1;for(int i=0;i<10;i++)step(10,100,100);assert(CarApp_TestContext()->runtime.state==ADC_BAD);}
  if(fault==5){for(int i=0;i<60;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state==STALL);}
  if(fault==6){for(int i=0;i<5;i++)step(10,-40,100);assert(CarApp_TestContext()->runtime.state==REVERSE);}
  assert(a==0&&b==0&&signals==0&&!CarApp_TestContext()->route.tracking);
 }
 /* Startup cannot be queued without READY; held boot or across ON never starts. */
 setup();hold();hold();enable();press_key();assert(CarApp_TestContext()->runtime.state!=RUNNING&&CarApp_TestContext()->button.action==KEY_WAIT);
 setup_key(1);for(int i=0;i<300;i++)step(10,0,0);assert(CarApp_TestContext()->button.selected_motion==MOVE_ROUTE&&CarApp_TestContext()->button.events==0);
 task_start();release_key();press_key();assert(CarApp_TestContext()->runtime.state==STOPPED);off=1;release_key();press_key();off=0;
 for(int i=0;i<200;i++)step(10,0,0);assert(CarApp_TestContext()->runtime.state!=RUNNING);
 /* Running OLED, signal duration and elapsed calculation survive wrap. */
 task_start();CarApp_TestContext()->route.start_ms=ms=0xfffffff0U;CarApp_TestContext()->runtime.old_ms=CarApp_TestContext()->power.sample_ms=CarApp_TestContext()->ui.motion_log_ms=CarApp_TestContext()->ui.heading_log_ms=CarApp_TestContext()->ui.button_log_ms=CarApp_TestContext()->ui.oled_log_ms=CarApp_TestContext()->ui.render_ms=CarApp_TestContext()->ui.line_log_ms=ms;
 for(int i=0;i<30;i++)step(10,600,600);assert(CarApp_TestContext()->runtime.state==RUNNING&&CarApp_TestContext()->route.leg==0);
 assert(strlen(oled_lines[2])<=21);
 cmd("STOP!");assert(CarApp_TestContext()->route.elapsed_ms==300);
 assert(sizeof CarCommand_Banner-1<=160);
 (void)head;
 puts("PASS: calibrated1256mm BC endpoint regression; Task2 GPIO-only AB->BC->CD->DA->A four-point signals and final settled stop; white straight/black arc switching gated by distance, debounce and end heading; Task2 400/300mm/s continuous legs with Task4-parity arc handling (80mm/s takeover at capture, fixed +-60 outermost, 500ms/20mm/12deg white and 35mm/18deg gap bridges, 30ms recovery, 60ms/200ms promotion, ramp resume, 120mm/2.5s/45deg fault bounds), Task4 PI retarget and 3600 PWM cap; preserved half-circle settings and task1 regressions; four-mode keys, no BLE required, no premature entry/false arc loss endpoint; bounded missing edges, STOP/OFF/RX/IMU/FAST/GAP/LOW/ADC/STALL/REV/timeout protections, restart/held-key suppression, OLED limits and time wrap. Simulation is not field acceptance.");
 printf("TRACE hash=%016llx events=%llu\n",(unsigned long long)trace_hash,(unsigned long long)trace_events);
 return 0;
}
