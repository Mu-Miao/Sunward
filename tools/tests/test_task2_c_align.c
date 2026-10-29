/* Task2 C->D handoff using production scheduler and an ideal wheel/yaw model. */
#define CAR_TASK3_TEST_MAIN preceding_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static float begin_cd(float error)
{
 near_arc_end();float nominal=app()->route.task2_arc_exit_heading;
 app()->heading.angle=CarMath_WrapDegrees(nominal-error);line_inputs=15;
 for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.leg==2 && signals==2);
 assert(fabsf(CarMath_WrapDegrees(app()->heading.reference-nominal))<.001f);
 assert(app()->route.task2_c_align.active==(fabsf(error)>3));
 return nominal;
}
static void ideal_alignment_tick(void)
{
 app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+
  (app()->route.targets[RIGHT]-app()->route.targets[LEFT])*.01f/DPT_TRACK_WIDTH_MM*57.2957795131f);
 step_roll();
}
static void residual_alignment_and_distance(void)
{
 const float errors[]={-19.5f,-12,-5,5,12,19.5f};
 for(unsigned j=0;j<sizeof errors/sizeof *errors;j++) {
  float nominal=begin_cd(errors[j]),origin=app()->route.leg_start;
  assert(!CarRoute_UsesHeading(app()) && !CarRoute_UsesLine(app()));
  assert(errors[j]<0?app()->route.targets[0]>app()->route.targets[1]:app()->route.targets[1]>app()->route.targets[0]);
  unsigned n;
  for(n=0;n<150 && app()->route.task2_c_align.active;n++) {
   assert(app()->route.targets[0]>=20 && app()->route.targets[1]>=20);
   assert(app()->route.targets[0]<=100 && app()->route.targets[1]<=100);
   ideal_alignment_tick();
   assert(app()->runtime.state==RUNNING && signals==2 && a>=0 && b<=0);
   assert(app()->route.leg_start==origin);
  }
  assert(!app()->route.task2_c_align.active && n<150 && CarRoute_UsesHeading(app()));
  assert(fabsf(CarMath_WrapDegrees(nominal-app()->heading.angle))<=3.001f);
  assert(app()->route.task2_c_align.travel_mm>0 && app()->route.task2_c_align.travel_mm<60);
  assert(fabsf(progress()-app()->route.task2_c_align.travel_mm)<.002f);
  assert(app()->route.base==50 && app()->route.task2_straight.entry_ms==ms);
  printf("IDEAL Task2 C residual%+.1fdeg: %lums/%.2fmm counted in CD; final_error=%.2fdeg. Not field validation.\n",
   errors[j],(unsigned long)app()->route.task2_c_align.elapsed_ms,app()->route.task2_c_align.travel_mm,
   CarMath_WrapDegrees(nominal-app()->heading.angle));
  /* CD acceleration starts now; time spent correcting C must not release400. */
  app()->heading.angle=nominal;
  for(unsigned k=0;k<50;k++)step_roll();
  assert(app()->route.base>=174.9f && app()->route.base<=175.1f);
  /* C stays the origin: the D acquisition window still starts at800mm. */
  app()->route.distance_mm[0]=app()->route.distance_mm[1]=origin+799;
  line_inputs=9;for(unsigned k=0;k<5;k++)step(10,0,0);
  assert(app()->route.leg==2 && signals==2);
  app()->route.distance_mm[0]=app()->route.distance_mm[1]=origin+800.1f;
  for(unsigned k=0;k<4;k++)step(10,0,0);
  assert(app()->route.leg==3 && signals==3);
  cmd("STOP!");assert(!app()->route.task2_c_align.active && a==0 && b==0);
 }
}
static void small_angle_and_confirmation(void)
{
 const float small[]={-3,-2,0,2,3};
 for(unsigned i=0;i<5;i++) {
  begin_cd(small[i]);assert(!app()->route.task2_c_align.active && app()->route.base==50);
  assert(fabsf(app()->heading.error)<=3.001f && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 }
 float nominal=begin_cd(-12);
 app()->heading.angle=nominal;step(10,0,0);step(10,0,0);step(10,0,0);
 assert(app()->route.task2_c_align.active);
 app()->heading.angle=CarMath_WrapDegrees(nominal-4);step(10,0,0);
 assert(app()->route.task2_c_align.active && !app()->route.task2_c_align.stable_seen);
 app()->heading.angle=nominal;
 for(unsigned k=0;k<3;k++){step(10,0,0);assert(app()->route.task2_c_align.active);}
 step(10,0,0);assert(!app()->route.task2_c_align.active && app()->route.base==50);
}
static void bound_and_stop_priority(void)
{
 for(unsigned event=0;event<12;event++) {
  float nominal=begin_cd(-12);TestState expected=YAW_BAD;
  switch(event) {
   case 0:app()->route.task2_c_align.start_ms=ms-1490;break;
   case 1:app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task2_c_align.start_mm+60;break;
   case 2:app()->heading.angle=CarMath_WrapDegrees(nominal-21);break;
   case 3:off=1;expected=OFF;break;
   case 4:queue(0,"STOP!TASK2!");expected=STOPPED;break;
   case 5:imu_fail=1;break;
   case 6:faults[0]=1;expected=UART_BAD;break;
   case 7:app()->route.start_ms=ms-45000;expected=TIMEOUT;break;
   case 8:step(30,0,0);expected=GAP;break;
   case 9:app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task2_c_align.start_mm-6;break;
   case 10:bat=1000;app()->power.sample_ms=ms-100;expected=LOW;break;
   case 11:step(10,2000,0);expected=FAST;break;
  }
  step(10,0,0);
  assert(app()->runtime.state==expected && !app()->route.task2_c_align.active && a==0 && b==0);
  step(10,0,0);assert(app()->runtime.state==expected && a==0 && b==0);
 }
 /* Alignment, timeout and stable confirmation also survive millis wrap. */
 float nominal=begin_cd(-12);ms=UINT32_MAX-100;app()->runtime.old_ms=ms;
 app()->route.start_ms=ms-5000;app()->route.task2_c_align.start_ms=ms;
 app()->power.sample_ms=ms;app()->line.sample_ms=ms;
 for(unsigned n=0;n<145 && app()->route.task2_c_align.active;n++)ideal_alignment_tick();
 assert(app()->runtime.state==RUNNING && !app()->route.task2_c_align.active);
 assert(fabsf(CarMath_WrapDegrees(nominal-app()->heading.angle))<=3.001f);
}
int main(void)
{
 residual_alignment_and_distance();small_angle_and_confirmation();bound_and_stop_priority();
 puts("PASS: under/overturn and angle wrap, fixed CD heading, moving correction before acceleration, original C distance and D window, no duplicate C signals, unchanged3deg/30ms confirmation and1500ms/60mm bounds; STOP/OFF/IMU/UART/LOW/FAST/GAP/45s priorities and millis wrap.");
 return 0;
}
