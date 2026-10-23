/* Full geometric alignment: reject early field departures, retain the vertex-search
   and startup guards, and verify the low-speed rolling differential vertex turn. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static float nominal_heading(unsigned leg)
{
 CarTask3 *t=&app()->route.task3;
 return CarMath_WrapDegrees(t->start_heading_deg+
  (leg==2?180+2*t->diagonal_angle_deg:t->diagonal_angle_deg+(leg==3?180:0)));
}
static void turn_start(unsigned mode)
{
 if(mode==3)task3_start();
 else {setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();}
 assert(app()->runtime.state==RUNNING);
}
static void settle_at(float angle,CarTask3Phase aligning,CarTask3Phase settled)
{
 app()->heading.angle=angle;
 for(unsigned n=0;n<10 && app()->route.task3.phase==aligning;n++) {
  step(10,0,0);assert(a==0 && b==0);
 }
 assert(app()->route.task3.phase==settled);
 for(unsigned n=0;n<30 && app()->route.task3.phase==settled;n++) {
  step(10,0,0);assert(a==0 && b==0);
 }
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase!=settled);
}
static void vertex_full_tangent(void)
{
 for(unsigned leg=0;leg<=2;leg+=2) {
  CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output o;
  CarTask3Input in={.heading_ready=1,.enabled=1};
  assert(CarTask3_Start(&t,&c,&in));
  t.phase=T3_VERTEX_SEEK;t.leg=leg;t.pending_start_signal=0;
  float target=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg+(leg==2?180:0));
  in.distance_mm=t.diagonal_mm;t.phase_start_mm=in.distance_mm;
  in.yaw_deg=CarMath_WrapDegrees(target+(leg==0?-5:5));
  for(unsigned n=0;n<5;n++) {
   in.now_ms+=10;CarTask3_Update(&t,&in,&o);
   assert(t.phase==T3_VERTEX_SEEK && !t.seek_aligned && o.control==T3_TURN_FORWARD);
   assert(fabsf(CarMath_WrapDegrees(o.heading_deg-target))<.001f);
  }
  in.yaw_deg=target;
  for(unsigned n=0;n<=c.settle_ms/10U;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&o);assert(o.control==T3_HALT);}
  assert(t.seek_aligned);in.now_ms+=10;CarTask3_Update(&t,&in,&o);
  assert(o.control==T3_HOLD_HEADING && fabsf(CarMath_WrapDegrees(o.heading_deg-target))<.001f);
 }
}
int main(void)
{
 assert(CarTask3_ConfigValid(&(CarTask3Config){0})==0);
 CarTask3Config c=CarRoute_Task3Config();assert(CarTask3_ConfigValid(&c));
 assert(c.align_tolerance_deg==3);
 CarTask3Config unified=CarTask4_Config();
 assert(unified.align_tolerance_deg==3 && unified.b_align_tolerance_deg==3 && unified.d_align_tolerance_deg==3);
 vertex_full_tangent();
 /* Both modes reject the reported early A departure and keep the original fixed AC reference. */
 for(unsigned mode=3;mode<=4;mode++) {
  turn_start(mode);float ac=app()->route.task3.start_heading_deg;
  assert(fabsf(ac+38.65981f)<.001f);
  app()->heading.angle=-30.95f;
  for(unsigned n=0;n<5;n++) {
   step(10,0,0);assert(app()->route.task3.phase==T3_START_ALIGN);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]==0);
   assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-ac))<.001f);
  }
  settle_at(ac,T3_START_ALIGN,T3_START_SETTLE);
  assert(app()->route.task3.phase==T3_DIAGONAL && app()->route.task3.start_heading_deg==ac);
  if(mode==4)assert(app()->route.task4.ac_reference_deg==ac);
  line_inputs=15;step(10,0,0);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-ac))<.001f);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
 }
 /* Every vertex uses the full geometric target and both wheels keep rolling.
    Only the 3-degree heading window hands the line back to capture/diagonal. */
 for(unsigned mode=3;mode<=4;mode++)for(unsigned leg=1;leg<=3;leg++) {
  turn_start(mode);float ac=app()->route.task3.start_heading_deg;
  for(unsigned n=0;n<3500 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ROLL_TURN);n++) {
   assert(app()->runtime.state==RUNNING);step_route();
  }
  assert(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ROLL_TURN);
  float nominal=nominal_heading(leg);
  app()->heading.angle=CarMath_WrapDegrees(nominal+(leg==3?5:-5));
  for(unsigned n=0;n<5;n++) {
   step(10,0,0);assert(app()->route.task3.phase==T3_ROLL_TURN);
   assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-nominal))<.001f);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
  }
  app()->heading.angle=nominal;step(10,0,0);
  assert(app()->route.task3.start_heading_deg==ac);
  if(mode==4)assert(app()->route.task4.ac_reference_deg==ac);
  if(leg==2) {
   assert(app()->route.task3.phase==T3_DIAGONAL);
   line_inputs=15;step(10,0,0);
   /* The rolling turn leaves the complete BD target as the diagonal reference. */
   assert(fabsf(CarMath_WrapDegrees(nominal-app()->heading.angle))<1.0f);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
  } else {
   assert(app()->route.task3.phase==T3_CAPTURE);
   line_inputs=15;step(10,0,0);
   assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-nominal))<.001f);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
   line_inputs=leg==1?13:11;
   for(unsigned n=0;n<4;n++)step(10,100,100);
   assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
  }
 }
 /* Replay second-lap C: -7.58 degrees stays in the rolling turn and only the
    3-degree window may enter capture. */
 turn_start(4);
 for(unsigned n=0;n<9500 && !(app()->route.task4.completed_laps==1 && app()->route.task3.leg==1 && app()->route.task3.phase==T3_ROLL_TURN);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task4.completed_laps==1 && app()->route.task3.leg==1 && app()->route.task3.phase==T3_ROLL_TURN);
 uint32_t cturn=app()->route.task3.phase_ms;float cgoal=nominal_heading(1);
 line_inputs=15;app()->heading.angle=-7.58f;
 for(unsigned n=0;n<5;n++) {
  step(10,0,0);assert(app()->route.task3.phase==T3_ROLL_TURN && app()->route.task3.phase_ms==cturn);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-cgoal))<.001f);
 }
 app()->heading.angle=CarMath_WrapDegrees(cgoal-2);step(10,0,0);
 assert(app()->route.task3.phase==T3_CAPTURE);
 step(10,0,0);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-cgoal))<.001f);
 /* Reproduce the second-lap B departure angle from the field log. It must keep
    rolling toward BD, never begin the diagonal at -149 degrees. */
 turn_start(4);
 for(unsigned n=0;n<9500 && !(app()->route.task4.completed_laps==1 && app()->route.task3.leg==2 && app()->route.task3.phase==T3_ROLL_TURN);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task4.completed_laps==1 && app()->route.task3.leg==2 && app()->route.task3.phase==T3_ROLL_TURN);
 float goal=nominal_heading(2),origin=app()->route.task3.leg_start_mm;
 app()->heading.angle=-149.16f;step(10,0,0);
 assert(app()->route.task3.phase==T3_ROLL_TURN && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-goal))<.001f);
 /* B keeps the same 3-degree gate: 3.2 outside, 2.8 inside. */
 app()->heading.angle=CarMath_WrapDegrees(goal-3.2f);step(10,0,0);
 assert(app()->route.task3.phase==T3_ROLL_TURN && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 app()->heading.angle=CarMath_WrapDegrees(goal-2.8f);step(10,0,0);
 assert(app()->route.task3.phase==T3_DIAGONAL && fabsf(progress())<.01f);
 assert(app()->route.task3.leg_start_mm>origin);
 assert(fabsf(CarMath_WrapDegrees(goal-app()->heading.angle))<3.0f);
 /* Disabling the B-specific tolerance still keeps the full geometric heading. */
 turn_start(4);
 for(unsigned n=0;n<3500 && !(app()->route.task3.leg==2 && app()->route.task3.phase==T3_ROLL_TURN);n++)step_route();
 app()->route.task3.config.b_full_align=0;step(10,0,0);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-nominal_heading(2)))<.001f);
 CarTask3Config bcfg=CarTask4_Config();assert(bcfg.b_full_align && bcfg.b_align_tolerance_deg==3 && CarTask3_ConfigValid(&bcfg));
 bcfg.b_align_tolerance_deg=0;assert(!CarTask3_ConfigValid(&bcfg));
 bcfg.b_align_tolerance_deg=NAN;assert(!CarTask3_ConfigValid(&bcfg));
 bcfg.b_align_tolerance_deg=4;assert(!CarTask3_ConfigValid(&bcfg));
 bcfg=CarTask4_Config();bcfg.b_full_align=2;assert(!CarTask3_ConfigValid(&bcfg));
 /* Reported DA departure at -172.14 degrees must keep rolling toward the
    full 180-degree tangent instead of using a single edge probe to finish it. */
 turn_start(4);
 for(unsigned n=0;n<4500 && !(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ROLL_TURN);n++)step_route();
 assert(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ROLL_TURN);
 float dgoal=nominal_heading(3);
 assert(fabsf(CarMath_WrapDegrees(dgoal+180))<.01f);
 line_inputs=14;app()->heading.angle=-172.14f;
 for(unsigned n=0;n<5;n++) {
  step(10,0,0);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ROLL_TURN);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-dgoal))<.001f);
 }
 app()->heading.angle=CarMath_WrapDegrees(dgoal+3.2f);step(10,0,0);
 assert(app()->route.task3.phase==T3_ROLL_TURN);
 app()->heading.angle=CarMath_WrapDegrees(dgoal+2.8f);step(10,0,0);
 assert(app()->route.task3.phase==T3_CAPTURE);
 line_inputs=15;step(10,0,0);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-dgoal))<.001f);
 /* Existing off-centre feedback/capture remains available after full alignment. */
 line_inputs=14;for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
 for(unsigned variant=0;variant<3;variant++) {
  turn_start(4);
  for(unsigned n=0;n<4500 && !(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ROLL_TURN);n++)step_route();
  assert(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ROLL_TURN);
  if(variant==0) {
   app()->route.task3.config.d_full_align=0;step(10,0,0);
   assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-nominal_heading(3)))<.001f);
  } else {
   if(variant==1)app()->route.task3.phase_ms=ms-app()->route.task3.config.align_timeout_ms;
   else app()->route.task3.phase_start_mm-=2.0f*app()->route.task3.config.turn_travel_limit_mm+1;
   step(10,0,0);
   assert(app()->runtime.state==YAW_BAD && app()->route.task3.fault==T3_TURN && a==0 && b==0);
  }
 }
 CarTask3Config dcfg=CarTask4_Config();assert(dcfg.d_full_align && dcfg.d_align_tolerance_deg==3 && CarTask3_ConfigValid(&dcfg));
 dcfg.d_align_tolerance_deg=0;assert(!CarTask3_ConfigValid(&dcfg));
 dcfg.d_align_tolerance_deg=NAN;assert(!CarTask3_ConfigValid(&dcfg));
 dcfg.d_align_tolerance_deg=4;assert(!CarTask3_ConfigValid(&dcfg));
 dcfg=CarTask4_Config();dcfg.d_full_align=2;assert(!CarTask3_ConfigValid(&dcfg));
 assert(!CarRoute_Task3Config().d_full_align);
 puts("PASS: full A/B/C/D and vertex-search headings, reported -30.95deg A / second-lap -7.58deg C rejection, unified 3-degree tolerance including B/D 2.8 acceptance and 3.2 rejection, low-speed rolling differential vertex turns on both wheels, original turn budget, fixed references and off-centre capture. Software only.");
 return 0;
}
