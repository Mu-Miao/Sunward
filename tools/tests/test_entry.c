/* Exercise capture/entry/arc through actual GPIO decoding, scheduler and wheel PI.
 * Sensor/encoder inputs are synthetic; these tests do not prove field stability. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void entry_start(unsigned leg)
{
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 for(unsigned n=0;n<5000 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_CAPTURE);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.phase==T3_CAPTURE && app()->route.task3.leg==leg);
 app()->route.task3.config.edge_follow=0;
 line_inputs=leg==1?7:14;
 for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC_ENTRY);
 assert(app()->route.task4.arc_number==(leg==1?1:2));
 assert(app()->route.task3_trace.valid_seen && !app()->route.task3_trace.previous_seen);
}

static void enter_for(unsigned count)
{
 for(unsigned n=0;n<count;n++) {
  step(10,0,0);
  if(app()->runtime.state!=RUNNING || app()->route.task3.phase!=T3_ARC_ENTRY)
   fprintf(stderr,"entry fixture: leg=%u phase=%u fault=%u state=%u elapsed=%lu loss=%lu pattern=%u\n",
    app()->route.task3.leg,app()->route.task3.phase,app()->route.task3.fault,app()->runtime.state,
    (unsigned long)(ms-app()->route.task3.phase_ms),(unsigned long)(ms-app()->route.task3.lost_start_ms),line_inputs);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC_ENTRY);
 }
}

static void handoff_regression(void)
{
 for(unsigned leg=1;leg<=3;leg+=2) {
  entry_start(leg);
  uint32_t start=app()->route.start_ms;
  float origin=app()->route.task3.leg_start_mm,ac=app()->route.task4.ac_reference_deg;
  /* An outer probe alone cannot release normal arc speed, even after 200ms. */
  enter_for(22);
  assert(app()->route.base==50 && app()->route.targets[0]>0 && app()->route.targets[1]>0);
  assert(app()->route.targets[0]<=70 && app()->route.targets[1]<=70);
  /* A short white gap freezes the last trusted correction, with both wheels forward. */
  line_inputs=15;enter_for(8);
  assert(app()->route.task3.loss_seek && app()->route.base==50);
  assert(app()->route.targets[leg==1?0:1]==30 && app()->route.targets[leg==1?1:0]==70);
  uint32_t loss=app()->route.task3.lost_start_ms;
  /* Inner single probes can complete entry; a centred pair is not mandatory. */
  line_inputs=leg==1?13:11;
  for(unsigned n=0;n<6;n++) {
   enter_for(1);assert(a>0 && b<0 && app()->route.task3.lost_start_ms==loss);
   assert(app()->route.task3.lost_seen); /* Moving confirmation cannot renew the budget. */
  }
  step(10,0,0);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0 && app()->route.base==50);
  assert(app()->route.task4.arc_number==(leg==1?1:2) && app()->route.task4.completed_laps==0);
  assert(app()->route.task3.leg_start_mm==origin && app()->route.start_ms==start);
  assert(app()->route.task4.ac_reference_deg==ac && !app()->route.task3.lost_seen);
 }
 /* Centred capture still receives at least 200ms of low-speed arc feedforward. */
 entry_start(1);line_inputs=9;enter_for(19);
 assert(app()->route.targets[0]==40 && app()->route.targets[1]==60);
 step(10,177,176);
 assert(app()->route.task3.phase==T3_ARC && app()->route.base==50);
 assert(progress()>0 && progress()<1); /* Handoff must not discard travelled arc distance. */
}

static void bounds_regression(void)
{
 /* A transient line frame must not continually restart the original white deadline. */
 entry_start(1);line_inputs=15;enter_for(1);
 uint32_t loss=app()->route.task3.lost_start_ms;
 for(unsigned n=0;n<49;n++) {
  line_inputs=n%2?15:9;step(10,30,30);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC_ENTRY);
  assert(app()->route.task3.lost_start_ms==loss);
 }
 line_inputs=9;step(10,0,0);
 assert(app()->runtime.state==LINE_LOST && app()->route.task3.loss_limits&T3_LOSS_TIME && a==0 && b==0);
 /* Recovery on the exact deadline cannot outrank the timeout. */
 entry_start(1);line_inputs=15;
 for(unsigned n=0;n<44;n++) {
  step(10,30,30);assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC_ENTRY);
 }
 line_inputs=9;enter_for(6);step(10,0,0);
 assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 /* Wrong/noncontiguous patterns stop immediately; subsequent white stays stopped. */
 for(unsigned pattern=0;pattern<2;pattern++) {
  entry_start(3);line_inputs=pattern?5:0;enter_for(1);assert(a==0 && b==0);
  line_inputs=15;enter_for(5);assert(a==0 && b==0 && !app()->route.task3.loss_seek);
 }
 for(unsigned bound=0;bound<5;bound++) {
  entry_start(1);line_inputs=9;
  if(bound==0)app()->route.task3.phase_ms=ms-1490U;
  else if(bound==1)app()->route.task3.phase_start_mm-=60;
  else if(bound==2)app()->heading.angle=CarMath_WrapDegrees(app()->route.task3.entry_yaw_deg+25.1f);
  else {
   line_inputs=15;enter_for(1);
   if(bound==3)app()->route.task3.lost_start_mm-=35;
   else {app()->route.task3.lost_yaw_deg=CarMath_WrapDegrees(app()->heading.angle-18.1f);}
  }
  step(10,0,0);
  assert(app()->runtime.state!=RUNNING && a==0 && b==0);
  if(bound<2) {
   assert(app()->route.task3.fault==T3_EDGE && app()->route.edge_trace.phase==T3_ARC_ENTRY);
   assert(app()->route.edge_trace.limits==(bound==0?T3_LOSS_TIME:T3_LOSS_DISTANCE));
  } else if(bound==2)assert(app()->route.task3.fault==T3_HEADING);
  else assert(app()->route.task3.fault==T3_LOST &&
    app()->route.task3.loss_limits==(bound==3?T3_LOSS_DISTANCE:T3_LOSS_ANGLE));
  line_inputs=9;for(unsigned n=0;n<10;n++)step(10,0,0);
  assert(app()->runtime.state!=RUNNING && a==0 && b==0);
 }
}

static void recorded_onset_regression(void)
{
 /* A V11 success showed outer probe for ~650ms, white for ~150ms, then
    inner probe around 50mm/20deg. These synthetic boundaries preserve that
    opportunity; they do not replay or predict the slower V13 trajectory. */
 entry_start(3);float origin=app()->route.task3.leg_start_mm;
 float yaw=app()->route.task3.entry_yaw_deg;
 for(unsigned n=0;n<64;n++) {
  step(10,177,176);
  assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC_ENTRY);
 }
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=origin+41;
 app()->heading.angle=CarMath_WrapDegrees(yaw-14);line_inputs=15;enter_for(15);
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=origin+50;
 app()->heading.angle=CarMath_WrapDegrees(yaw-20.5f);line_inputs=11;enter_for(6);step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC);
 assert(fabsf(progress()-50)<.01f && app()->route.task4.arc_number==2);
 /* CB can need a right correction despite the nominal left arc: losing
    the outer-right probe must not reverse this established correction. */
 entry_start(1);line_inputs=14;enter_for(1);
 assert(app()->route.targets[0]>app()->route.targets[1]);
 line_inputs=15;enter_for(1);assert(app()->route.targets[0]>app()->route.targets[1]);
}

static void protection_regression(void)
{
 for(unsigned event=0;event<10;event++) {
  entry_start(event%2?3:1);line_inputs=15;enter_for(2);
  switch(event) {
  case 0:queue(0,"STOP!TASK4!");break;
  case 1:off=1;break;
  case 2:imu_fail=1;break;
  case 3:adc_fail=1;app()->power.sample_ms=ms-100;break;
  case 4:bat=1000;app()->power.sample_ms=ms-100;break;
  case 5:faults[0]=1;break;
  case 6:step(30,0,0);break;
  case 7:step(10,2000,0);break;
  case 8:app()->route.start_ms=ms-160000U;break;
  case 9:release_key();press_key();break;
  }
  step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0);
 }
 entry_start(1);line_inputs=9;
 for(unsigned n=0;n<5;n++)step(10,-177,-176);
 assert(app()->runtime.state==REVERSE && a==0 && b==0);
 entry_start(3);
 for(unsigned n=0;n<60 && app()->runtime.state==RUNNING;n++)step(10,0,0);
 assert(app()->runtime.state==STALL && a==0 && b==0);
 /* Corrupt/stale inputs still fail closed inside the new phase. */
 for(unsigned which=0;which<3;which++) {
  CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.enabled=1,.heading_ready=1};assert(CarTask3_Start(&t,&c,&in));
  t.phase=T3_ARC_ENTRY;t.leg=1;t.pending_start_signal=0;
  in.now_ms=10;in.line_valid=1;in.black_count=2;in.centered=1;
  if(which==0)in.sample_age_ms=21;
  else if(which==1)in.heading_ready=0;
  else in.yaw_deg=NAN;
  CarTask3_Update(&t,&in,&out);assert(t.fault==T3_SENSOR && out.control==T3_HALT);
 }
 /* Wrap-safe entry/recovery deadlines, including yaw crossing +180/-180. */
 CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output out;
 CarTask3Input in={.enabled=1,.heading_ready=1,.now_ms=UINT32_MAX-100U,.yaw_deg=179};
 assert(CarTask3_Start(&t,&c,&in));t.phase=T3_ARC_ENTRY;t.leg=1;
 t.pending_start_signal=0;t.entry_yaw_deg=179;
 in.line_valid=1;in.black_count=2;in.centered=1;in.yaw_deg=-179;
 for(unsigned n=0;n<20;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);}
 assert(t.phase==T3_ARC && out.control==T3_FOLLOW_LINE);
}

static unsigned flow_zero_requests;
static void (*flow_motor)(int,int);
static void monitor_flow_motor(int a_cmd,int b_cmd)
{
 CarTask3 *t=&app()->route.task3;
 if(app()->runtime.state==RUNNING && (t->leg&1U) &&
    (t->phase==T3_ALIGN || t->phase==T3_CAPTURE || t->phase==T3_ARC_ENTRY) &&
    app()->route.targets[0]+app()->route.targets[1]==0 && a_cmd==0 && b_cmd==0)
  ++flow_zero_requests; /* Distinguish explicit HALT from PI coasting above target speed. */
 flow_motor(a_cmd,b_cmd);
}

static void rolling_regression(void)
{
 assert(CarTask4_Config().rolling_entry && !CarRoute_Task3Config().rolling_entry);
 CarTask3Config bad=CarTask4_Config();bad.rolling_entry=2;assert(!CarTask3_ConfigValid(&bad));
 for(unsigned diagonal=0;diagonal<=2;diagonal+=2) {
  CarTask3Config c=CarTask4_Config();c.start_from_ab=0;c.edge_follow=0;
  CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.enabled=1,.heading_ready=1,.now_ms=1000};
  assert(CarTask3_Start(&t,&c,&in));t.leg=diagonal;t.pending_start_signal=0;
  in.distance_mm=t.diagonal_mm-40;
  in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+(diagonal?180+2*t.diagonal_angle_deg-c.bd_turn_relief_deg:0));
  in.left_mmps=in.right_mmps=50;in.line_valid=1;in.black_count=2;in.centered=1;
  for(unsigned n=0;n<4;n++) {
   in.now_ms+=10;in.distance_mm+=.5f;CarTask3_Update(&t,&in,&out);
   assert(out.control!=T3_HALT && !out.reset_controllers);
   if(n<3)assert(t.phase==T3_DIAGONAL && out.control==T3_TURN_FORWARD && out.left_mmps>0 && out.right_mmps>0);
  }
  assert(t.phase==T3_ALIGN && t.leg==diagonal+1 &&
   out.signal_point==(diagonal?T3_POINT_D:T3_POINT_C));
  float tangent=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg+(diagonal?180:0));
  uint32_t turn_start=t.phase_ms;
  /* Moving angle confirmation still rejects an excursion outside3deg,
     resets only the debounce, and retains the original turn deadline. */
  for(unsigned n=0;n<3;n++) {
   in.yaw_deg=CarMath_WrapDegrees(tangent+2.8f);in.now_ms+=10;
   CarTask3_Update(&t,&in,&out);assert(t.phase==T3_ALIGN && out.control==T3_HOLD_HEADING);
  }
  in.yaw_deg=CarMath_WrapDegrees(tangent+3.2f);in.now_ms+=10;CarTask3_Update(&t,&in,&out);
  assert(t.phase==T3_ALIGN && !t.edge_seen && t.phase_ms==turn_start && out.control==T3_TURN_FORWARD);
  for(unsigned n=0;n<4;n++) {
   in.yaw_deg=tangent;in.now_ms+=10;in.distance_mm+=.5f;CarTask3_Update(&t,&in,&out);
   assert(out.control==T3_HOLD_HEADING && !out.reset_controllers && out.base_mmps==50);
   if(n<3)assert(t.phase==T3_ALIGN);
  }
  assert(t.phase==T3_CAPTURE && in.left_mmps==50); /* No stopped-wheel prerequisite. */
  for(unsigned n=0;n<4;n++) {
   in.now_ms+=10;in.distance_mm+=.5f;CarTask3_Update(&t,&in,&out);
   assert(out.control!=T3_HALT && !out.reset_controllers);
  }
  assert(t.phase==T3_ARC_ENTRY && out.control==T3_ENTER_ARC && out.left_mmps>0 && out.right_mmps>0);
 }
 /* Continuous C/D handoff cannot outrank the original turn budgets,
    stale sensors, STOP/OFF, capture limits or diagonal endpoint window. */
 for(unsigned event=0;event<8;event++) {
  CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.enabled=1,.heading_ready=1,.now_ms=1000};assert(CarTask3_Start(&t,&c,&in));
  t.phase=T3_ALIGN;t.leg=3;t.pending_start_signal=0;
  in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg+180);
  in.line_valid=1;in.black_count=2;in.centered=1;in.left_mmps=in.right_mmps=50;
  if(event==0)t.phase_ms=in.now_ms-c.align_timeout_ms;
  else if(event==1)t.phase_start_mm=-c.turn_travel_limit_mm-1;
  else if(event==2)in.stop_requested=1;
  else if(event==3)in.enabled=0;
  else if(event==4)in.sample_age_ms=21;
  else if(event==5) {t.phase=T3_CAPTURE;t.phase_ms=in.now_ms-c.capture_timeout_ms;}
  else if(event==6) {t.phase=T3_CAPTURE;t.phase_start_mm=-c.capture_limit_mm-1;}
  else {
   t.phase=T3_DIAGONAL;t.leg=2;in.distance_mm=t.diagonal_mm+c.entry_window_mm+1;
   in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+180+2*t.diagonal_angle_deg-c.bd_turn_relief_deg);
  }
  in.now_ms+=10;CarTask3_Update(&t,&in,&out);
  assert(t.phase==T3_FAULT && out.control==T3_HALT && out.reset_controllers);
 }
 /* Actual scheduler over all eight joins: no C/D BRAKE/SETTLE phases or
    explicit zero-drive requests; A/B/final settled stops remain covered. */
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 flow_motor=app()->runtime.drive;flow_zero_requests=0;app()->runtime.drive=monitor_flow_motor;
 unsigned vertices=0;
 for(unsigned n=0;n<14000 && app()->runtime.state==RUNNING;n++) {
  unsigned old_leg=app()->route.task3.leg;step_route();CarTask3 *t=&app()->route.task3;
  if((old_leg==0 || old_leg==2) && t->leg==old_leg+1)++vertices;
  assert(!(t->phase==T3_VERTEX_BRAKE && !(t->leg&1U)));
  assert(!(t->phase==T3_ALIGN_SETTLE && (t->leg&1U)));
  if((t->leg&1U) && (t->phase==T3_ALIGN || t->phase==T3_CAPTURE || t->phase==T3_ARC_ENTRY))
   assert(app()->route.targets[0]+app()->route.targets[1]>0);
 }
 assert(app()->runtime.state==DONE && vertices==8 && flow_zero_requests==0);
 assert(app()->route.task4.completed_laps==4 && app()->route.task4.arc_number==8 && signals==17);
 app()->runtime.drive=flow_motor;
 puts("PASS: eight continuous C/D joins, moving30ms vertex/3deg tangent confirmation, no parked entry/zero-drive handoff, retained budgets/stop priority and unchanged four-lap counting. Software only.");
}

int main(void)
{
 CarTask3Config c=CarTask4_Config();assert(CarTask3_ConfigValid(&c));
 assert(CarRoute_Task3Config().arc_entry_ms==0);
 c.arc_entry_ms=1501;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_confirm_ms=c.lost_ms;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_diff_mmps=NAN;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_diff_mmps=50;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_mm=61;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_deg=26;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_mmps=51;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.arc_entry_ms=0;assert(CarTask3_ConfigValid(&c));
 handoff_regression();bounds_regression();recorded_onset_regression();protection_regression();rolling_regression();
 puts("PASS: C/D low-speed arc entry, 200ms minimum/60ms inner-or-centred confirmation, bounded white gaps/chatter, preserved arc distance/counts, exact deadlines, fail-closed patterns/protections and time/yaw wrap. Software only.");
 return 0;
}
