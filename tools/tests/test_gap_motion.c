/* Real scheduler regression for a short white LF04 gap. Ideal encoders only. */
#define CAR_TASK3_TEST_MAIN previous_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void ready_normal(unsigned task,int enabled)
{
 if(task==2)fast_to_arc();
 else {
  setup();line_inputs=15;hold();hold();hold();if(task==4)hold();
  calibrate();enable();press_key();
  for(unsigned n=0;n<6000 && !(app()->route.task3.phase==T3_ARC &&
      !app()->route.task3.edge_follow_active && app()->route.base>=299);n++) {
   assert(app()->runtime.state==RUNNING);step_route();
  }
 }
 line_inputs=9;
 for(unsigned n=0;n<350 && app()->route.base<299;n++)step_roll();
 for(unsigned n=0;n<15;n++)step_roll();
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
 assert(app()->route.base==300 && app()->route.task3.normal_trusted);
 app()->route.task3.config.short_gap_ms=enabled?30:0;
}
static void short_gap_no_pause(void)
{
 for(unsigned task=2;task<=4;task++)for(unsigned whites=1;whites<=3;whites++) {
  ready_normal(task,1);
  uint32_t count=app()->route.task3.short_gap_count,loss=0;
  unsigned points=signals;
  for(unsigned n=0;n<whites+4;n++) {
   line_inputs=n<whites?15:9;step_roll();
   assert(app()->runtime.state==RUNNING && signals==(int)points);
   assert(app()->route.targets[0]==300 && app()->route.targets[1]==300);
   assert(app()->wheel.items[0].command>0 && app()->wheel.items[1].command>0);
   if(n==0){loss=app()->route.task3.lost_start_ms;assert(app()->route.task3.short_gap_active);}
   if(n<whites+3)assert(app()->route.task3.lost_seen && app()->route.task3.lost_start_ms==loss);
  }
  assert(!app()->route.task3.lost_seen && !app()->route.task3.short_gap_active);
  assert(app()->route.task3.short_gap_count==count+1);
 }
 /* Disabled baseline reproduces the V28 zero-PWM transient; same real scheduler. */
 ready_normal(3,0);line_inputs=15;step_roll();
 assert(app()->route.base==40 && app()->wheel.items[0].command==0 && app()->wheel.items[1].command==0);
 line_inputs=9;for(unsigned n=0;n<4;n++)step_roll();
 assert(!app()->route.task3.lost_seen && app()->route.base<60);
 puts("Short white1..3 frames + original30ms confirmation: Tasks2/3/4 retain300mm/s targets and positive PI PWM. Disabled V28 baseline drops to40mm/s and zero PWM.");
}
static void downgrade_and_chatter(void)
{
 ready_normal(3,1);line_inputs=15;
 for(unsigned n=0;n<4;n++)step_roll();
 assert(app()->route.task3.lost_seen && !app()->route.task3.short_gap_active);
 assert(app()->route.base<=80 && app()->route.targets[0]<=140 && app()->route.targets[1]<=140);
 uint32_t origin=app()->route.task3.lost_start_ms;
 for(unsigned n=0;n<5;n++){line_inputs=n&1?15:9;step_roll();}
 assert(app()->route.task3.lost_start_ms==origin && app()->route.task3.short_gap_count==1);
 ready_normal(3,1);line_inputs=15;step_roll();origin=app()->route.task3.lost_start_ms;
 line_inputs=9;step_roll();assert(app()->route.task3.short_gap_active);
 line_inputs=15;step_roll();assert(!app()->route.task3.short_gap_active && app()->route.base<=80);
 for(unsigned n=0;n<5;n++){line_inputs=n&1?9:15;step_roll();}
 assert(app()->route.task3.lost_seen && app()->route.task3.lost_start_ms==origin);
 /* Outermost reappearance is not trusted normal-line recovery. */
 ready_normal(3,1);line_inputs=15;step_roll();line_inputs=7;step_roll();
 assert(!app()->route.task3.short_gap_active && app()->route.base<=80);
 /* Bad input patterns stop immediately, even within the short allowance. */
 for(unsigned pattern=0;pattern<=5;pattern+=5) {
  ready_normal(3,1);line_inputs=15;step_roll();line_inputs=pattern;step_roll();
  assert(a==0 && b==0 && !app()->route.task3.short_gap_active);
 }
}
static void original_guards_win(void)
{
 for(unsigned event=0;event<7;event++) {
  ready_normal(3,1);line_inputs=15;step_roll();assert(app()->route.task3.short_gap_active);
  TestState expected=LINE_LOST;
  if(event==0){app()->route.task3.lost_start_mm=progress()+app()->route.task3.leg_start_mm-20;}
  if(event==1){app()->heading.angle+=12.1f;}
  if(event==2){app()->route.task3.lost_start_ms=ms-490;}
  if(event==3){off=1;expected=OFF;}
  if(event==4){queue(0,"STOP!");expected=STOPPED;}
  if(event==5){imu_fail=1;expected=YAW_BAD;}
  if(event==6){step(30,0,0);expected=GAP;}
  step(10,0,0);
  assert(app()->runtime.state==expected && a==0 && b==0 && !app()->route.task3.short_gap_active);
  if(event==2)assert(app()->route.task3.loss_duration_ms==500);
 }
 /* Cold/no-history/EDGE/low-speed modes cannot acquire the exemption. */
 to_arc_entry();line_inputs=15;step_roll();assert(!app()->route.task3.short_gap_active);
 CarTask3Config c=CarTask4_Config();c.short_gap_ms=31;assert(!CarTask3_ConfigValid(&c));
}
static void frozen_differential(void)
{
 ready_normal(4,1);line_inputs=13;for(unsigned n=0;n<30;n++)step_roll();
 float left=app()->route.targets[0],right=app()->route.targets[1];
 assert(left>right && left<=400 && right>=20);
 line_inputs=15;step_roll();assert(app()->route.task3.short_gap_active);
 assert(app()->route.targets[0]==left && app()->route.targets[1]==right);
 line_inputs=13;for(unsigned n=0;n<3;n++) {
  step_roll();assert(app()->route.targets[0]==left && app()->route.targets[1]==right);
 }
 step_roll();assert(!app()->route.task3.lost_seen && app()->runtime.state==RUNNING);
 assert(app()->route.base==300);
}
int main(void)
{
 short_gap_no_pause();downgrade_and_chatter();original_guards_win();frozen_differential();
 puts("PASS: short white continuity, unchanged stable confirmation, one-way downgrade/chatter deadline, outer/invalid rejection, original distance/angle/time/STOP/OFF/IMU/GAP priority; frozen trusted steering and normal speed recovery.");
 return 0;
}
