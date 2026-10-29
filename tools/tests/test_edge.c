/* Production core/scheduler; recorded sensor inputs are not a new field test. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
#include "fixtures/v18_entry_onsets.h"

static void pattern(CarTask3Input *in,unsigned mask)
{
 line_inputs=mask;CarLine_Decode(app());
 in->line_valid=app()->line.valid;in->black_count=app()->line.count;
 in->line_error=app()->line.error;
 in->centered=in->line_valid && in->black_count==2 && fabsf(in->line_error)<.1f;
 in->outermost=in->line_valid && in->black_count==1 && fabsf(in->line_error)>2.5f;
 in->side_probe=in->line_valid && in->black_count==1;
}
static void core_entry(CarTask3 *t,CarTask3Input *in,CarTask3Output *o,
    unsigned leg,unsigned mask,int historical,uint32_t now)
{
 setup();memset(t,0,sizeof *t);memset(in,0,sizeof *in);
 CarTask3Config c=CarTask4_Config();c.start_from_ab=0;c.edge_follow=!historical;
 in->now_ms=now;in->enabled=in->heading_ready=1;
 assert(CarTask3_Start(t,&c,in));t->phase=T3_CAPTURE;t->leg=leg;t->pending_start_signal=0;
 in->yaw_deg=CarMath_WrapDegrees(t->start_heading_deg+t->diagonal_angle_deg+(leg==3?180:0));
 pattern(in,mask);
 for(unsigned n=0;n<4;n++){in->now_ms+=10;CarTask3_Update(t,in,o);}
 assert(t->phase==(historical?T3_ARC_ENTRY:T3_ARC));
 assert(t->edge_follow_active==!historical && o->signal_point==T3_NO_POINT);
}
static void tick_core(CarTask3 *t,CarTask3Input *in,CarTask3Output *o,unsigned mask)
{
 pattern(in,mask);in->now_ms+=10;CarTask3_Update(t,in,o);
}
static void replay(const EntryFrame *frames,unsigned count,int historical,CarTask3 *t,
    CarTask3Input *in,CarTask3Output *out)
{
 core_entry(t,in,out,1,7,historical,1000);
 uint32_t origin=in->now_ms;float yaw=in->yaw_deg;
 for(unsigned row=1;row<count && t->phase!=T3_FAULT;row++) {
  const EntryFrame *a=&frames[row-1],*b=&frames[row];
  for(unsigned time=a->ms+10;time<=b->ms && t->phase!=T3_FAULT;time+=10) {
   float f=(float)(time-a->ms)/(float)(b->ms-a->ms);
   in->distance_mm=a->mm+(b->mm-a->mm)*f;
   in->yaw_deg=CarMath_WrapDegrees(yaw+a->turn+(b->turn-a->turn)*f);
   pattern(in,time==b->ms?b->mask:a->mask);
   in->now_ms=origin+time;CarTask3_Update(t,in,out);
   assert(out->signal_point==T3_NO_POINT);
   if(!historical && t->phase!=T3_FAULT) {
    assert(t->phase==T3_ARC && t->edge_follow_active && out->control==T3_EDGE_FOLLOW);
    assert(out->base_mmps<=80 && out->left_mmps>=20 && out->right_mmps<=140);
   }
  }
 }
}
static void recorded_regression(void)
{
 CarTask3 t;CarTask3Input in;CarTask3Output o;
 replay(v18_distance_onset,sizeof v18_distance_onset/sizeof *v18_distance_onset,1,&t,&in,&o);
 assert(t.phase==T3_FAULT && t.fault==T3_EDGE && in.distance_mm>60);
 replay(v18_distance_onset,sizeof v18_distance_onset/sizeof *v18_distance_onset,0,&t,&in,&o);
 assert(t.phase==T3_ARC && t.edge_follow_active && in.distance_mm>60 && o.base_mmps==80);
 assert(o.left_mmps==20 && o.right_mmps==140);
 float origin=t.leg_start_mm;
 for(unsigned n=0;n<7;n++){in.distance_mm+=.7f;tick_core(&t,&in,&o,11);}
 assert(t.phase==T3_ARC && !t.edge_follow_active && o.control==T3_FOLLOW_LINE);
 assert(o.base_mmps==80 && t.leg_start_mm==origin);
 /* Same recorded sustained-white input must still stop. A new controller
    cannot be credited with changing a previously recorded physical path. */
 replay(v18_white_onset,sizeof v18_white_onset/sizeof *v18_white_onset,0,&t,&in,&o);
 assert(t.phase==T3_FAULT && t.fault==T3_LOST && t.loss_limits==T3_LOSS_TIME);
 assert(t.loss_duration_ms==500 && o.control==T3_HALT);
 puts("Recorded V18 onset: old60mm F6 reproduced; V28 tracks at20/140 past60mm and accepts stable inner probe; recorded500ms white still F7, no false B.");
}
static void promotion_regression(void)
{
 for(unsigned leg=1;leg<=3;leg+=2)for(unsigned side=0;side<2;side++) {
  CarTask3 t;CarTask3Input in;CarTask3Output o;
  unsigned outer=side?14:7,inner=side?13:11;
  core_entry(&t,&in,&o,leg,outer,0,UINT32_MAX-100);
  float origin=t.leg_start_mm;
  for(unsigned n=0;n<70;n++) {
   in.distance_mm+=.7f;tick_core(&t,&in,&o,outer);
   assert(t.phase==T3_ARC && t.edge_follow_active && o.base_mmps==80);
   assert(side?o.left_mmps==140:o.right_mmps==140);
  }
  for(unsigned n=0;n<5;n++)tick_core(&t,&in,&o,inner);
  assert(t.edge_follow_active);
  tick_core(&t,&in,&o,outer);assert(t.edge_follow_active && !t.edge_seen);
  for(unsigned n=0;n<6;n++){tick_core(&t,&in,&o,inner);assert(t.edge_follow_active);}
  tick_core(&t,&in,&o,inner);
  assert(!t.edge_follow_active && o.control==T3_FOLLOW_LINE && o.base_mmps==80);
  assert(t.leg_start_mm==origin && o.signal_point==T3_NO_POINT);
  tick_core(&t,&in,&o,inner);assert(o.base_mmps>80 && o.base_mmps<=81.01f);
 }
 /* Inner capture cannot release full speed before200ms. */
 CarTask3 t;CarTask3Input in;CarTask3Output o;
 core_entry(&t,&in,&o,1,9,0,1000);
 for(unsigned n=0;n<19;n++){tick_core(&t,&in,&o,9);assert(t.edge_follow_active && o.base_mmps==80);}
 tick_core(&t,&in,&o,9);assert(!t.edge_follow_active && o.base_mmps==80);
}
static void recovery_regression(void)
{
 CarTask3 t;CarTask3Input in;CarTask3Output o;
 core_entry(&t,&in,&o,1,7,0,1000);
 tick_core(&t,&in,&o,15);uint32_t lost=t.lost_start_ms;float diff=t.entry_diff_mmps;
 assert(t.loss_seek && t.gap_seek && o.left_mmps==20 && o.right_mmps==80 && o.base_mmps==50);
 for(unsigned n=0;n<35;n++){in.distance_mm+=.4f;tick_core(&t,&in,&o,15);}
 for(unsigned n=0;n<3;n++) {
  tick_core(&t,&in,&o,13); /* Wrong-side unconfirmed frames cannot flip direction. */
  assert(t.lost_seen && t.lost_start_ms==lost && t.entry_diff_mmps==diff && o.left_mmps<o.right_mmps);
 }
 tick_core(&t,&in,&o,11);assert(!t.lost_seen && t.edge_follow_active);
 for(unsigned n=0;n<7;n++)tick_core(&t,&in,&o,11);
 assert(!t.edge_follow_active && o.control==T3_FOLLOW_LINE);
 for(unsigned budget=0;budget<4;budget++) {
  core_entry(&t,&in,&o,1,7,0,1000);tick_core(&t,&in,&o,15);lost=t.lost_start_ms;
  if(budget==0) {
   for(unsigned n=0;n<49;n++) {
    tick_core(&t,&in,&o,n%2?15:11);
    assert(t.lost_start_ms==lost && t.phase==T3_ARC);
   }
  } else if(budget==1)in.distance_mm=t.lost_start_mm+35;
  else if(budget==2)in.yaw_deg=CarMath_WrapDegrees(t.lost_yaw_deg+18.1f);
  else in.distance_mm=t.lost_start_mm-5.1f;
  tick_core(&t,&in,&o,15);
  assert(t.phase==T3_FAULT && t.fault==(budget==3?T3_DISTANCE:T3_LOST) && o.control==T3_HALT);
  if(budget!=3)assert(t.loss_limits==(budget==0?T3_LOSS_TIME:budget==2?T3_LOSS_ANGLE:T3_LOSS_DISTANCE));
 }
}
static void guard_regression(void)
{
 for(unsigned event=0;event<8;event++) {
  CarTask3 t;CarTask3Input in;CarTask3Output o;
  core_entry(&t,&in,&o,1,7,0,1000);
  if(event==0)t.phase_ms=in.now_ms-2490;
  if(event==1)in.distance_mm=120;
  if(event==2)in.yaw_deg=CarMath_WrapDegrees(t.entry_yaw_deg+45.1f);
  if(event==3)in.stop_requested=1;
  if(event==4)in.sample_age_ms=21;
  if(event==5)in.yaw_deg=NAN;
  if(event==6)in.enabled=0;
  if(event==7)in.protection_fault=1;
  tick_core(&t,&in,&o,7);
  assert(t.phase==T3_FAULT && o.control==T3_HALT && o.reset_controllers);
  assert(t.fault==(event<2?T3_EDGE:event==2?T3_HEADING:
   event==4 || event==5?T3_SENSOR:T3_ABORT));
 }
 /* White near the entrance cannot become B, even with the endpoint yaw. */
 CarTask3 t;CarTask3Input in;CarTask3Output o;
 core_entry(&t,&in,&o,1,7,0,1000);
 t.entry_yaw_deg=in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg+180);
 for(unsigned n=0;n<51 && t.phase!=T3_FAULT;n++) {
  tick_core(&t,&in,&o,15);assert(o.signal_point==T3_NO_POINT && t.leg==1);
 }
 assert(t.phase==T3_FAULT && t.fault==T3_LOST);
 /* After promotion, both endpoint distance and yaw are still required. */
 core_entry(&t,&in,&o,1,9,0,1000);
 for(unsigned n=0;n<20;n++)tick_core(&t,&in,&o,9);
 assert(!t.edge_follow_active);
 in.distance_mm=t.config.arc_mm;
 in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg+150);
 for(unsigned n=0;n<4;n++){tick_core(&t,&in,&o,15);assert(o.signal_point==T3_NO_POINT && t.leg==1);}
 in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg+180);
 for(unsigned n=0;n<4;n++)tick_core(&t,&in,&o,15);
 assert(t.phase==T3_ALIGN && t.leg==2 && o.signal_point==T3_POINT_B);
}
static void scheduler_entry(unsigned leg,unsigned mask)
{
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 for(unsigned n=0;n<7000 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_CAPTURE);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.phase==T3_CAPTURE && app()->route.task3.config.edge_follow);
 line_inputs=mask;
 for(unsigned n=0;n<4;n++)step(10,100,100);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC && app()->route.task3.edge_follow_active);
}
static void scheduler_regression(void)
{
 for(unsigned leg=1;leg<=3;leg+=2)for(unsigned side=0;side<2;side++) {
  scheduler_entry(leg,side?14:7);
  float ref=app()->route.task4.ac_reference_deg;unsigned arcs=app()->route.task4.arc_number;
  assert(arcs==(leg==1?1:2));
  assert(app()->route.targets[side?0:1]==140 && app()->route.targets[side?1:0]==20);
  assert(a>=0 && b<=0 && app()->route.base==80);
  for(unsigned n=0;n<100;n++) {
   step(10,100,100);assert(app()->runtime.state==RUNNING && app()->route.task3.edge_follow_active);
  }
  line_inputs=15;step(10,100,100);
  assert(app()->route.targets[side?0:1]==80 && app()->route.targets[side?1:0]==20);
  line_inputs=side?13:11;
  for(unsigned n=0;n<12;n++)step(10,100,100);
  assert(app()->runtime.state==RUNNING && !app()->route.task3.edge_follow_active);
  assert(app()->route.task4.arc_number==arcs && app()->route.task4.ac_reference_deg==ref);
 }
 for(unsigned invalid=0;invalid<2;invalid++) {
  scheduler_entry(1,7);line_inputs=invalid?5:0;
  step(10,100,100);assert(a==0 && b==0);
  for(unsigned n=0;n<20;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 }
 for(unsigned boundary=0;boundary<2;boundary++) {
  scheduler_entry(1,7);CarTask3 *t=&app()->route.task3;
  if(boundary==0)t->phase_ms=ms-2490;
  else t->phase_start_mm=(app()->route.distance_mm[0]+app()->route.distance_mm[1])*.5f-120;
  step(10,0,0);assert(app()->runtime.state==LINE_BAD && t->fault==T3_EDGE);
  assert(app()->route.edge_trace.seen && app()->route.edge_trace.phase==T3_ARC);
  assert(app()->route.edge_trace.limits==(boundary==0?T3_LOSS_TIME:T3_LOSS_DISTANCE));
 }
 for(unsigned event=0;event<12;event++) {
  scheduler_entry(1,7);TestState expected=STOPPED;
  switch(event) {
  case 0:queue(0,"STOP!TASK4!");break;
  case 1:off=1;expected=OFF;break;
  case 2:imu_fail=1;expected=YAW_BAD;break;
  case 3:adc_fail=1;app()->power.sample_ms=ms-100;expected=ADC_BAD;break;
  case 4:bat=1000;app()->power.sample_ms=ms-100;expected=LOW;break;
  case 5:faults[0]=1;expected=UART_BAD;break;
  case 6:step(30,100,100);expected=GAP;break;
  case 7:step(10,2000,0);expected=FAST;break;
  case 8:for(unsigned n=0;n<5;n++)step(10,-177,176);expected=REVERSE;break;
  case 9:for(unsigned n=0;n<60 && app()->runtime.state==RUNNING;n++)step(10,0,0);expected=STALL;break;
  case 10:app()->route.start_ms=ms-160000;expected=TIMEOUT;break;
  case 11:release_key();press_key();break;
  }
  step(10,0,0);assert(app()->runtime.state==expected && a==0 && b==0);
  for(unsigned n=0;n<3;n++)step(10,0,0);assert(app()->runtime.state==expected && a==0 && b==0);
 }
 /* A forged edge control cannot bypass phase/line/fault authorization. */
 scheduler_entry(1,7);
 CarTask3Output o={.control=T3_EDGE_FOLLOW,.base_mmps=500,.left_mmps=-500,.right_mmps=900};
 CarTask3Port_Apply(app(),&o,ms);assert(app()->route.targets[0]==20 && app()->route.targets[1]==140);
 app()->route.task3.edge_follow_active=0;CarTask3Port_Apply(app(),&o,ms);assert(a==0 && b==0);
}
int main(void)
{
 CarTask3Config c=CarTask4_Config();assert(CarTask3_ConfigValid(&c));
 assert(c.edge_follow && !CarRoute_FigureEightBaseConfig().edge_follow);
 for(unsigned bad=0;bad<7;bad++) {
  c=CarTask4_Config();
  if(bad==0)c.edge_follow=2;
  if(bad==1)c.edge_follow_mmps=81;
  if(bad==2)c.edge_follow_diff_mmps=61;
  if(bad==3)c.edge_follow_diff_mmps=NAN;
  if(bad==4)c.edge_follow_mm=151;
  if(bad==5)c.edge_follow_ms=3001;
  if(bad==6)c.edge_follow_deg=61;
  assert(!CarTask3_ConfigValid(&c));
 }
 recorded_regression();promotion_regression();recovery_regression();guard_regression();scheduler_regression();
 puts("PASS: V19 low-speed edge capture both curves/sides, stable-only promotion, preserved distance/count/reference, bounded white recovery and exact deadlines, all fault stops, endpoint guards and port authorization. Software only.");
 return 0;
}
