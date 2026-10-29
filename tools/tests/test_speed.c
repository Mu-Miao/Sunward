/* Cross-task speed margins and unchanged recovery/stop contracts. Synthetic HAL. */
#define CAR_TASK3_TEST_MAIN task3_regression_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void all_line_patterns(void)
{
 const Motion modes[]={MOVE_TASK2,MOVE_TASK3,MOVE_TASK4};float peak=0;
 for(unsigned m=0;m<3;m++)for(unsigned raw=0;raw<16;raw++) {
  setup();app()->runtime.state=RUNNING;app()->route.motion=modes[m];
  line_inputs=raw;CarLine_Decode(app());CarLine_SetTargets(app(),ms,300);
  if(!app()->line.valid)continue;
  for(unsigned k=0;k<200;k++){ms+=10;CarLine_SetTargets(app(),ms,300);}
  for(unsigned w=0;w<2;w++) {
   float t=app()->route.targets[w];assert(t>0 && t<=396.01f);
   if(t>peak)peak=t;
  }
 }
 assert(peak>=370 && peak<=390.01f);
 printf("Normal arc raw-pattern target peak: %.1f mm/s; competition FAST guard500; other modes350.\n",peak);
}
static void preserved_budgets(void)
{
 CarTask3Config three=CarRoute_Task3Config(),four=CarTask4_Config();
 assert(three.diagonal_mmps==400 && three.arc_mmps==300 && three.acceleration_mmps2==250);
 assert(four.diagonal_mmps==400 && four.arc_mmps==300 && four.acceleration_mmps2==250);
 assert(three.deadline_ms==40000 && four.deadline_ms==80000 && DPT_TASK4_TIMEOUT_MS==160000);
 for(unsigned n=0;n<2;n++) {
  CarTask3Config *c=n?&four:&three;
  assert(c->edge_follow && c->continuous_motion && c->rolling_entry);
  assert(c->edge_follow_mmps==80 && c->edge_follow_diff_mmps==60);
  assert(c->edge_follow_mm==120 && c->edge_follow_deg==45 && c->edge_follow_ms==2500);
  assert(c->arc_entry_min_ms==200 && c->arc_entry_confirm_ms==60);
  assert(c->turn_inner_mmps==20 && c->turn_mmps==120 && c->turn_travel_limit_mm==120);
  assert(c->white_seek_ms==500 && c->white_seek_mm==50 && c->white_seek_deg==12);
  assert(c->da_gap_mm==50 && c->da_gap_deg==18 && c->recovery_ms==30);
  assert(c->arc_acceleration_mmps2==100 && CarTask3_ConfigValid(c));
 }
 /* Reject values beyond this experimental profile; recovery budgets stay fixed. */
 four.diagonal_mmps=401;assert(!CarTask3_ConfigValid(&four));
 four=CarTask4_Config();four.arc_mmps=301;assert(!CarTask3_ConfigValid(&four));
 assert(DPT_TASK2_EDGE_FOLLOW_MMPS==80 && DPT_TASK2_EDGE_FOLLOW_DIFF_MMPS==60);
 assert(DPT_ROUTE_CRUISE_MMPS==400 && DPT_ROUTE_SLOW_MM==300);
 assert(DPT_ROUTE_CRUISE_MMPS/DPT_ROUTE_SLOW_MM<1.5f);
 assert(DPT_ROUTE_MAX_CMD==3600 && DPT_TASK2_MAX_CMD==3600 && DPT_TASK3_MAX_CMD==3600 && DPT_TASK4_MAX_CMD==3600);
 assert(DPT_TASK2_TIMEOUT_MS==45000 && DPT_ROUTE_TIMEOUT_MS==13500);
}
static void speed_limit_isolation(void)
{
 const Motion modes[]={MOVE_ROUTE,MOVE_TASK2,MOVE_TASK3,MOVE_TASK4,MOVE_TEST,MOVE_HEADING,MOVE_DISTANCE,MOVE_LINE,MOVE_FORWARD,MOVE_BACK,MOVE_LEFT,MOVE_RIGHT};
 for(unsigned n=0;n<sizeof modes/sizeof modes[0];n++)for(unsigned wheel=0;wheel<2;wheel++) {
  setup();enable();app()->runtime.state=RUNNING;app()->route.motion=modes[n];
  float limit=n<4?500:350;assert(CarWheel_SpeedLimit(modes[n])==limit);
  app()->route.targets[wheel]=200;app()->wheel.items[wheel].velocity=limit-1;
  CarWheel_Control(app(),wheel,.01f);assert(app()->runtime.state==RUNNING);
  app()->wheel.items[wheel].velocity=limit+1;CarWheel_Control(app(),wheel,.01f);
  assert(app()->runtime.state==FAST && a==0 && b==0);
  step(10,0,0);assert(app()->runtime.state==FAST && a==0 && b==0);
 }
}
static void task2_approach(void)
{
 for(unsigned leg=0;leg<=2;leg+=2) {
  task_start();
  if(leg==2){endpoint(0);endpoint(1);app()->route.task2_straight.entry_ms=ms-5000;}
  app()->route.start_ms=ms-5000;line_inputs=15;
  app()->heading.angle=app()->heading.reference;
  const float distances[]={800,880,970},cd_expected[]={400,300,160};
  unsigned before=signals;
  for(unsigned n=0;n<3;n++) {
   app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.leg_start+distances[n];
   if(n==1)for(unsigned w=0;w<2;w++)app()->wheel.items[w].integral=3200;
   step(10,0,0);
   assert(app()->runtime.state==RUNNING && app()->route.leg==leg && signals==(int)before);
   float expected=leg==0?400:cd_expected[n];
   assert(app()->route.base==expected && app()->route.targets[0]>0 && app()->route.targets[1]>0);
   if(n==1)for(unsigned w=0;w<2;w++) {
    float expected_i=leg==0?3200:2400;
    assert(fabsf(app()->wheel.items[w].integral-expected_i)<.01f);
   }
  }
  line_inputs=9;for(unsigned n=0;n<4;n++)step(10,0,0);
  assert(app()->route.leg==leg+1 && signals==(int)before+1 && app()->route.edge_follow);
  assert(app()->route.base==80 && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 }
}
int main(void)
{
 all_line_patterns();preserved_budgets();speed_limit_isolation();task2_approach();
 puts("PASS: bounded normal arc targets; competition500/other350 FAST isolation, latch/no restart; Task2 AB cruise until capture, CD approach/integral reduction; fixed EDGE/turn/search budgets and deadlines.");
 return 0;
}
