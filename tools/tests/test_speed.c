/* Cross-task speed margins and unchanged recovery/stop contracts. Synthetic HAL. */
#define CAR_TASK3_TEST_MAIN task3_regression_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void all_line_patterns(void)
{
 const Motion modes[]={MOVE_TASK2,MOVE_TASK3,MOVE_TASK4};float peak=0;
 for(unsigned m=0;m<3;m++)for(unsigned raw=0;raw<16;raw++) {
  setup();app()->runtime.state=RUNNING;app()->route.motion=modes[m];
  line_inputs=raw;CarLine_Decode(app());CarLine_SetTargets(app(),ms,250);
  if(!app()->line.valid)continue;
  for(unsigned w=0;w<2;w++) {
   float t=app()->route.targets[w];assert(t>0 && t<=330.01f);
   if(t>peak)peak=t;
  }
 }
 assert(fabsf(peak-330)<.01f);
 printf("Normal arc raw-pattern target peak: %.1f mm/s; measured-speed FAST guard remains350.\n",peak);
}
static void preserved_budgets(void)
{
 CarTask3Config three=CarRoute_Task3Config(),four=CarTask4_Config();
 assert(three.diagonal_mmps==280 && three.arc_mmps==250 && three.acceleration_mmps2==180);
 assert(four.diagonal_mmps==280 && four.arc_mmps==250 && four.acceleration_mmps2==180);
 assert(three.deadline_ms==40000 && four.deadline_ms==80000 && DPT_TASK4_TIMEOUT_MS==160000);
 for(unsigned n=0;n<2;n++) {
  CarTask3Config *c=n?&four:&three;
  assert(c->edge_follow && c->continuous_motion && c->rolling_entry);
  assert(c->edge_follow_mmps==70 && c->edge_follow_diff_mmps==50);
  assert(c->edge_follow_mm==120 && c->edge_follow_deg==45 && c->edge_follow_ms==2500);
  assert(c->arc_entry_min_ms==200 && c->arc_entry_confirm_ms==60);
  assert(c->turn_inner_mmps==20 && c->turn_mmps==100 && c->turn_travel_limit_mm==120);
  assert(c->white_seek_ms==500 && c->white_seek_mm==20 && c->white_seek_deg==12);
  assert(c->da_gap_mm==35 && c->da_gap_deg==18 && c->recovery_ms==30);
  assert(c->arc_acceleration_mmps2==100 && CarTask3_ConfigValid(c));
 }
 /* Do not widen the existing validated core configuration ceiling. */
 four.diagonal_mmps=281;assert(!CarTask3_ConfigValid(&four));
 four=CarTask4_Config();four.arc_mmps=251;assert(!CarTask3_ConfigValid(&four));
 assert(DPT_TASK2_EDGE_FOLLOW_MMPS==70 && DPT_TASK2_EDGE_FOLLOW_DIFF_MMPS==50);
 assert(DPT_ROUTE_CRUISE_MMPS==270 && DPT_ROUTE_SLOW_MM==180);
 assert(DPT_ROUTE_CRUISE_MMPS/DPT_ROUTE_SLOW_MM==1.5f);
 assert(DPT_ROUTE_MAX_CMD==2200 && DPT_TASK2_MAX_CMD==2200 && DPT_TASK3_MAX_CMD==2200 && DPT_TASK4_MAX_CMD==2200);
 assert(DPT_TASK2_TIMEOUT_MS==45000 && DPT_ROUTE_TIMEOUT_MS==13500);
}
int main(void)
{
 all_line_patterns();preserved_budgets();
 puts("PASS: all valid arc patterns stay within330mm/s targets; unchanged EDGE/turn/search budgets, deadlines, PWM and core speed ceilings.");
 return 0;
}
