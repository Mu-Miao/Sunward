/* Reduced stationary turning: keep geometric references, settle gates, and directional capture. */
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
 assert(app()->runtime.state==RUNNING && app()->route.task3.config.turn_relief_deg==5);
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
static void gentle_diagonal(float reference)
{
 line_inputs=15;step(10,0,0);
 float first=fabsf(CarMath_WrapDegrees(reference-app()->heading.angle));
 assert(first>4.9f && first<5.1f);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-reference))<.001f);
 for(unsigned n=0;n<100;n++) {
  float left=app()->route.targets[0],right=app()->route.targets[1];
  assert(left>0 && right>0);
  /* Hypothetical 140mm ideal wheel plant, not a vehicle acceptance model. */
  app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+(right-left)*.01f/140*57.2957795131f);
  int dl=CarMath_Round(left*.01f*DPT_LEFT_CPR/(DPT_CIRC_M*1000));
  int dr=CarMath_Round(right*.01f*DPT_RIGHT_CPR/(DPT_CIRC_M*1000));
  step(10,dl,dr);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_DIAGONAL && a>=0 && b<=0);
 }
 assert(fabsf(CarMath_WrapDegrees(reference-app()->heading.angle))<first*.5f);
}
int main(void)
{
 CarTask3Config c=CarRoute_Task3Config();assert(c.turn_relief_deg==5 && CarTask3_ConfigValid(&c));
 c.turn_relief_deg=NAN;assert(!CarTask3_ConfigValid(&c));
 c.turn_relief_deg=-1;assert(!CarTask3_ConfigValid(&c));
 c.turn_relief_deg=11;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.radius_mm=1;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.turn_relief_deg=0;assert(CarTask3_ConfigValid(&c));
 assert(CarTask3_DefaultConfig().turn_relief_deg==0);
 /* Both modes retain the exact AC reference while stationary A turning ends five degrees early. */
 for(unsigned mode=3;mode<=4;mode++) {
  turn_start(mode);float ac=app()->route.task3.start_heading_deg;
  assert(fabsf(ac+38.65981f)<.001f);
  float partial=CarMath_WrapDegrees(ac+5);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-partial))<.001f);
  settle_at(partial,T3_START_ALIGN,T3_START_SETTLE);
  assert(app()->route.task3.phase==T3_DIAGONAL && app()->route.task3.start_heading_deg==ac);
  if(mode==4)assert(app()->route.task4.ac_reference_deg==ac);
  gentle_diagonal(ac);
 }
 /* Task4 B fully aligns; other reduced targets and settle gates remain intact. */
 for(unsigned mode=3;mode<=4;mode++)for(unsigned leg=1;leg<=3;leg++) {
  turn_start(mode);float ac=app()->route.task3.start_heading_deg;
  for(unsigned n=0;n<3500 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ALIGN);n++) {
   assert(app()->runtime.state==RUNNING);step_route();
  }
  assert(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ALIGN);
  int full_b=mode==4 && leg==2;
  float nominal=nominal_heading(leg),partial=full_b?nominal:CarMath_WrapDegrees(nominal+(leg==3?5:-5));
  step(10,0,0);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-partial))<.001f);
  settle_at(partial,T3_ALIGN,T3_ALIGN_SETTLE);
  assert(app()->route.task3.start_heading_deg==ac);
  if(mode==4)assert(app()->route.task4.ac_reference_deg==ac);
  if(leg==2) {
   assert(app()->route.task3.phase==T3_DIAGONAL);
   line_inputs=15;step(10,0,0);
   if(full_b) {
    assert(fabsf(CarMath_WrapDegrees(nominal-app()->heading.angle))<1.0f);
    assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
   } else {
    assert(app()->route.targets[1]>app()->route.targets[0]);
    gentle_diagonal(nominal);
   }
  } else {
   assert(app()->route.task3.phase==T3_CAPTURE);
   line_inputs=15;step(10,0,0);
   assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-partial))<.001f);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
   line_inputs=leg==1?13:11;
   for(unsigned n=0;n<4;n++)step(10,100,100);
   assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
  }
 }
 /* Zero relief reproduces full geometric alignment, permitting a simple parameter rollback. */
 turn_start(3);app()->route.task3.config.turn_relief_deg=0;step(10,0,0);
 assert(app()->heading.reference==app()->route.task3.start_heading_deg);
 /* Reproduce the second-lap B departure angle from the field log. It must
    continue a stationary forward-wheel turn, never begin BD at -149 degrees. */
 turn_start(4);
 for(unsigned n=0;n<9500 && !(app()->route.task4.completed_laps==1 && app()->route.task3.leg==2 && app()->route.task3.phase==T3_ALIGN);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task4.completed_laps==1 && app()->route.task3.leg==2 && app()->route.task3.phase==T3_ALIGN);
 float goal=nominal_heading(2),origin=app()->route.task3.leg_start_mm;
 app()->heading.angle=-149.16f;step(10,0,0);
 assert(app()->route.task3.phase==T3_ALIGN && app()->route.targets[0]==0 && app()->route.targets[1]>0);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-goal))<.001f);
 /* Being inside the previous 3-degree gate is no longer enough at B. */
 app()->heading.angle=CarMath_WrapDegrees(goal-2);step(10,0,0);
 assert(app()->route.task3.phase==T3_ALIGN && app()->route.targets[0]==0 && app()->route.targets[1]>0);
 app()->heading.angle=CarMath_WrapDegrees(goal-.8f);
 for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(app()->route.task3.phase==T3_ALIGN_SETTLE && a==0 && b==0);
 app()->heading.angle=CarMath_WrapDegrees(goal-1.2f);step(10,0,0);
 assert(app()->route.task3.phase==T3_ALIGN && app()->route.task3.leg_start_mm==origin && a==0 && b==0);
 settle_at(goal,T3_ALIGN,T3_ALIGN_SETTLE);
 assert(app()->route.task3.phase==T3_DIAGONAL && fabsf(progress())<.01f);
 assert(fabsf(CarMath_WrapDegrees(goal-app()->heading.angle))<1.0f);
 /* Toggling the scoped option off restores the original B relief target. */
 turn_start(4);
 for(unsigned n=0;n<3500 && !(app()->route.task3.leg==2 && app()->route.task3.phase==T3_ALIGN);n++)step_route();
 app()->route.task3.config.b_full_align=0;step(10,0,0);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-(nominal_heading(2)-5)))<.001f);
 CarTask3Config bcfg=CarTask4_Config();assert(bcfg.b_full_align && bcfg.b_align_tolerance_deg==1 && CarTask3_ConfigValid(&bcfg));
 bcfg.b_align_tolerance_deg=0;assert(!CarTask3_ConfigValid(&bcfg));
 bcfg.b_align_tolerance_deg=NAN;assert(!CarTask3_ConfigValid(&bcfg));
 bcfg.b_align_tolerance_deg=4;assert(!CarTask3_ConfigValid(&bcfg));
 bcfg=CarTask4_Config();bcfg.b_full_align=2;assert(!CarTask3_ConfigValid(&bcfg));
 puts("PASS: Task4 B full BD alignment within 1 degree before settled departure, second-lap -149deg rejection, settle drift recheck, preserved turn guards and scoped rollback; other 5-degree targets, fixed references and line capture unchanged. Software only.");
 return 0;
}
