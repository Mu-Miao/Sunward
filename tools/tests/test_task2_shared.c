/* Shared-arc output parity and CD reference handoff. Synthetic inputs only. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void ignore_motor(int x,int y){(void)x;(void)y;}
static void shared_arc_parity(void)
{
 for(unsigned outer_side=0;outer_side<2;outer_side++) {
  to_arc_entry();CarTask3 ref=app()->route.task3;
  assert(ref.phase==T3_ARC && ref.leg==3 && ref.config.history_gap && ref.config.edge_follow);
  CarTask3Config four=CarTask4_Config();
  assert(ref.config.arc_mmps==four.arc_mmps && ref.config.edge_follow_mmps==four.edge_follow_mmps);
  assert(ref.config.arc_entry_confirm_ms==four.arc_entry_confirm_ms && ref.config.white_seek_ms==four.white_seek_ms);
  for(unsigned n=0;n<95;n++) {
   line_inputs=n<25?(outer_side?14:7):n<35?15:(outer_side?13:11);
   step(10,30,30);assert(app()->runtime.state==RUNNING);
   CarTask3Input in=CarTask3Port_Input(app(),ms,1,0,0);CarTask3Output output;
   CarTask3_Update(&ref,&in,&output);
   assert(ref.phase==app()->route.task3.phase && ref.fault==T3_OK);
   assert(ref.edge_follow_active==app()->route.task3.edge_follow_active && ref.lost_seen==app()->route.task3.lost_seen);
   CarAppContext expected=*app();expected.route.motion=MOVE_TASK4;expected.route.task3=ref;expected.runtime.drive=ignore_motor;
   CarTask3Port_Apply(&expected,&output,ms);
   for(unsigned w=0;w<2;w++)assert(fabsf(expected.route.targets[w]-app()->route.targets[w])<.001f);
  }
 }
}
static void cd_reference_and_ramp(void)
{
 for(unsigned side=0;side<2;side++) {
  near_arc_end();float nominal=app()->route.task2_arc_exit_heading;
  float actual=CarMath_WrapDegrees(nominal+(side?12:-12));
  app()->heading.angle=actual;line_inputs=15;
  for(unsigned n=0;n<4;n++)step(10,0,0);
  assert(app()->route.leg==2 && app()->runtime.state==RUNNING && app()->route.task2_straight.seen);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-actual))<.001f);
  assert(fabsf(CarMath_WrapDegrees(app()->route.task2_straight.nominal_heading-nominal))<.001f);
  assert(fabsf(app()->heading.error)<.001f && app()->route.targets[0]==50 && app()->route.targets[1]==50);
  for(unsigned n=0;n<50;n++)step_roll();
  assert(app()->route.base>=174 && app()->route.base<=176);
  assert(fabsf(app()->route.targets[0]-app()->route.targets[1])<.001f);
  /* A measured encoder asymmetry is recorded, not silently treated as gyro drift. */
  for(unsigned n=0;n<50;n++)step(10,400,600);
  assert(app()->route.task2_straight.wheel_yaw_deg>10);
  assert(fabsf(app()->route.task2_straight.gyro_delta_deg)<.001f);
  assert(app()->route.task2_straight.right_mm>app()->route.task2_straight.left_mm);
  cmd("STOP!");assert(a==0 && b==0);
 }
}
static void moving_arc_guards(void)
{
 CarTask3Config cfg=CarTask4_Config();CarTask3 t={0};CarTask3Input i={.enabled=1,.heading_ready=1,.line_valid=1,.black_count=2,.centered=1,.left_mmps=300,.right_mmps=300,.now_ms=0xfffffff0U};
 assert(CarTask3_BeginArc(&t,&cfg,&i,-180,-1));uint32_t origin=t.phase_ms;
 i.now_ms+=10;assert(!CarTask3_BeginArc(&t,&cfg,&i,-180,-1) && t.phase_ms==origin);
 CarTask3Output o;CarTask3_Update(&t,&i,&o);assert(t.phase==T3_ARC && o.control==T3_EDGE_FOLLOW);
 i.now_ms+=10;i.stop_requested=1;CarTask3_Update(&t,&i,&o);assert(t.phase==T3_FAULT&&o.control==T3_HALT);
 memset(&t,0,sizeof t);i.stop_requested=0;i.line_valid=0;i.black_count=0;
 assert(!CarTask3_BeginArc(&t,&cfg,&i,0,-1) && t.fault==T3_SENSOR);
 to_arc_entry();cmd("STOP!");assert(app()->route.task3.phase==T3_FAULT);
 off=1;release_key();cmd("TASK3!");calibrate();enable();press_key();assert(app()->runtime.state==RUNNING && app()->route.motion==MOVE_TASK3);
}
int main(void)
{
 shared_arc_parity();cd_reference_and_ramp();moving_arc_guards();
 puts("PASS: Task2 calls shared Task3/4 arc core/port, mirrored loss/recovery output parity; +/-12deg CD handoff without nominal kick, smooth ramp, encoder/gyro discrepancy evidence; moving-entry safety and subsequent Task3 restart.");
 return 0;
}
