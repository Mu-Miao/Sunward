/* Default production search behaviour and actual scheduler integration. No hardware access. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void seek_start(unsigned task)
{
 if(task==3) {task3_start();app()->route.task3.config=CarRoute_Task3Config();}
 else {
  setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
  assert(app()->route.motion==MOVE_TASK4 && app()->runtime.state==RUNNING);
  assert(app()->route.task3.config.white_seek_ms==500 && app()->route.task3.config.vertex_seek_ms==3000);
 }
}
static void seek_arc(unsigned task)
{
 seek_start(task);reach_phase(T3_ARC);line_inputs=9;step(10,177,176);
 assert(app()->route.task3.arc_valid_seen);
}
static void seek_d(void)
{
 seek_start(3);
 for(unsigned n=0;n<3500 && !(app()->route.task3.leg==2 && app()->route.task3.phase==T3_DIAGONAL);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.phase==T3_DIAGONAL && app()->route.task3.leg==2);
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+1281;
 line_inputs=15;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_VERTEX_SEEK && a==0 && b==0);
}
static void da_dropout(void)
{
 seek_start(4);
 for(unsigned n=0;n<4500 && !(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ARC);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ARC);
 /* Historical 80/20 profile remains covered separately from V6 geometry. */
 app()->route.task3.config.da_outer_gap=0;
 app()->route.task3.config.history_gap=0;
 app()->route.task3.phase_ms=ms-2000U;
 app()->route.task4.resume_pending=app()->route.task4.resume_active=0;
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+186;
 app()->heading.angle=156;line_inputs=14;step(10,0,0);
 assert(app()->line.error==3 && app()->route.targets[0]==168 && app()->route.targets[1]==72);
 line_inputs=15;step(10,414,167);
 assert(app()->runtime.state==RUNNING && app()->route.task3.loss_seek);
 assert(app()->route.task3_trace.lost_raw[0]==1 && app()->route.task3_trace.valid_raw[0]==0);
 assert(CarMath_Round(app()->route.task3_trace.velocity[0])==141 && CarMath_Round(app()->route.task3_trace.velocity[1])==57);
}
static void extended_task4_regression(void)
{
 /* Reproduce the reported DA snapshots with synthetic HAL inputs. Crossing
    300ms is allowed only inside the unchanged distance/angle envelope. */
 for(unsigned action=0;action<4;action++) {
  da_dropout();
  uint32_t loss=app()->route.task3.lost_start_ms,start=app()->route.start_ms;
  float origin=app()->route.task3.leg_start_mm,reference=app()->route.task4.ac_reference_deg;
  for(unsigned n=0;n<35;n++) {
   app()->heading.angle=156-(float)(uint32_t)(ms+10-loss)/75.0f;
   step(10,100,100);
   assert(app()->runtime.state==RUNNING && app()->route.task3.lost_start_ms==loss);
   assert(app()->route.task3.loss_seek && app()->route.targets[0]>app()->route.targets[1]);
   assert(app()->route.targets[0]==80 && app()->route.targets[1]==20);
  }
  assert((uint32_t)(ms-loss)==350);
  if(action==0) {
   line_inputs=9;
   for(unsigned n=0;n<3;n++){step(10,0,0);assert(a==0 && b==0 && app()->route.task3.lost_seen);}
   step(10,0,0);
   assert(app()->runtime.state==RUNNING && !app()->route.task3.lost_seen);
   assert(app()->route.task3.phase==T3_ARC && app()->route.task3.leg==3 && app()->route.task4.completed_laps==0);
   assert(app()->route.start_ms==start && app()->route.task3.leg_start_mm==origin && app()->route.task4.ac_reference_deg==reference);
   assert(app()->route.base<=50.01f);
  } else {
   if(action==1)queue(0,"STOP!");
   else if(action==2)off=1;
   else imu_fail=1;
   step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0);
  }
 }
 puts("PASS: Task4 reported DA edge/dropout snapshots, recovery after 350ms, unchanged 30ms confirmation, fixed route references and STOP/OFF/IMU priority in the extended interval. Synthetic inputs only.");
}
static void gap_start(unsigned side)
{
 seek_arc(4);app()->route.task3.config.history_gap=0;line_inputs=side?7:14;step(10,177,176);
 assert(app()->route.task3_trace.valid_error==(side?-3.0f:3.0f));
 line_inputs=15;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.loss_seek && !app()->line.valid);
 assert(app()->route.targets[side?0:1]==20 && app()->route.targets[side?1:0]==80);
 assert(app()->wheel.items[side?0:1].command>0);
}
static void measured_da_gap_start(unsigned side)
{
 seek_arc(4);
 app()->route.task3.config.history_gap=0; /* Preserve V6-only regression scope. */
 /* Inject a DA arc fixture, then use actual GPIO decoding and scheduler. */
 app()->route.task3.leg=3;
 app()->route.task3.leg_start_mm=(app()->route.distance_mm[0]+app()->route.distance_mm[1])*.5f;
 app()->heading.angle=-180;
 line_inputs=side?7:14;step(10,177,176);
 assert(app()->route.task3.arc_outer_valid);
 line_inputs=15;step(10,0,0);
 assert(app()->runtime.state==RUNNING && app()->route.task3.gap_seek && !app()->line.valid);
 assert(app()->route.targets[side?0:1]==20 && app()->route.targets[side?1:0]==140);
}
static void history_arc(unsigned leg)
{
 seek_start(4);
 for(unsigned n=0;n<5000 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ARC);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.leg==leg && app()->route.task3.phase==T3_ARC);
 assert(app()->route.task4.arc_number==(leg==1?1:2));
 line_inputs=9;step(10,0,0);
}
static void history_gap_regression(void)
{
 for(unsigned leg=1;leg<=3;leg+=2)for(unsigned side=0;side<2;side++) {
  history_arc(leg);unsigned arc=app()->route.task4.arc_number;
  line_inputs=side?11:13;step(10,0,0);
  assert(app()->line.error==(side?-1:1) && app()->route.task3_trace.previous_seen);
  assert(app()->route.task3_trace.previous_error==0);
  uint8_t before[4];memcpy(before,app()->line.raw,4);
  line_inputs=15;step(10,0,0);
  assert(app()->runtime.state==RUNNING && app()->route.task3.gap_seek);
  assert(app()->route.task3_trace.lost_arc_number==arc && app()->route.task3_trace.lost_direction==(side?-1:1));
  assert(memcmp(app()->route.task3_trace.valid_raw,before,4)==0);
  assert(app()->route.targets[side?0:1]==20 && app()->route.targets[side?1:0]==140);
  uint32_t loss=app()->route.task3.lost_start_ms;
  uint32_t now=(ms/400U+1U)*400U;app()->ui.motion_log_ms=now-400U;CarUI_Log(app(),now,0);
  assert(strstr(logs,side?"PRE=1101 PRE2=1001 HAS2=1 DIR=L GAP=1":"PRE=1011 PRE2=1001 HAS2=1 DIR=R GAP=1"));
  /* One unconfirmed opposite probe cannot overwrite the two trusted frames. */
  line_inputs=side?14:7;step(10,0,0);assert(a==0 && b==0);
  line_inputs=15;step(10,0,0);
  assert(memcmp(app()->route.task3_trace.valid_raw,before,4)==0);
  assert(app()->route.task3.lost_start_ms==loss && app()->route.task3_trace.lost_direction==(side?-1:1));
  line_inputs=side?7:14;
  for(unsigned n=0;n<3;n++){step(10,0,0);assert(a==0 && b==0 && app()->route.task3.lost_seen);}
  step(10,0,0);
  assert(app()->runtime.state==RUNNING && !app()->route.task3.lost_seen && app()->route.task4.arc_number==arc);
  assert(app()->route.base<=50.01f);
 }
 /* Stable centre has no side cue: CB defaults left, DA defaults right.
    A centre transition uses the preceding trusted frame before that fallback. */
 for(unsigned leg=1;leg<=3;leg+=2)for(unsigned cue=0;cue<3;cue++) {
  history_arc(leg);
  if(cue){line_inputs=cue==1?13:11;step(10,0,0);}
  line_inputs=9;step(10,0,0);line_inputs=15;step(10,0,0);
  int direction=cue==1?-1:(cue==2?1:(leg==1?-1:1));
  assert(!app()->route.task3.gap_seek && app()->route.task3_trace.lost_direction==direction);
  assert(direction>0?app()->route.targets[0]>app()->route.targets[1]:app()->route.targets[0]<app()->route.targets[1]);
 }
 /* Crossing from CB to DA discards CB frames and increments exactly once. */
 history_arc(1);line_inputs=13;step(10,0,0);assert(app()->route.task3_trace.previous_seen);
 for(unsigned n=0;n<5000 && !(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ARC);n++)step_route();
 assert(app()->route.task3.leg==3 && app()->route.task3.phase==T3_ARC && app()->route.task4.arc_number==2);
 assert(!app()->route.task3_trace.previous_seen);
 /* The new inner-probe CB branch retains the deadline and fault latch. */
 history_arc(1);line_inputs=13;step(10,0,0);line_inputs=15;step(10,0,0);
 app()->route.task3.lost_start_ms=ms-490;
 step(10,0,0);assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 line_inputs=13;for(unsigned n=0;n<5;n++)step(10,0,0);assert(app()->runtime.state==LINE_LOST);
 CarTask3Config c=CarTask4_Config();assert(c.history_gap);c.history_gap=2;assert(!CarTask3_ConfigValid(&c));
 assert(!CarRoute_Task3Config().history_gap);
 puts("PASS: real arc-entry counts, CB/DA inner-probe gap recovery, two trusted raw frames, frozen side/trend/curve direction, chatter rejection, no duplicate recovery count, arc history reset, inner-gap timeout latch and HIST telemetry. Software only.");
}
static void measured_da_gap_regression(void)
{
 for(unsigned side=0;side<2;side++) {
  measured_da_gap_start(side);
  uint32_t loss=app()->route.task3.lost_start_ms,start=app()->route.start_ms;
  float origin=app()->route.task3.leg_start_mm,reference=app()->route.task4.ac_reference_deg;
  for(unsigned n=0;n<34;n++) {
   app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+(side?.4f:-.4f));
   step(10,side?59:412,side?412:59);
   assert(app()->runtime.state==RUNNING && app()->route.task3.gap_seek && !app()->line.valid);
  }
  assert(app()->route.task3.loss_limits==0);
  assert(fabsf(CarMath_WrapDegrees(app()->heading.angle-app()->route.task3.lost_yaw_deg))>12);
  assert((app()->route.distance_mm[0]+app()->route.distance_mm[1])*.5f-app()->route.task3.lost_start_mm>20);
  /* An opposite-side unconfirmed glitch cannot turn the trusted sweep around. */
  line_inputs=side?14:7;step(10,0,0);assert(a==0 && b==0);
  line_inputs=15;step(10,0,0);
  assert(app()->route.targets[side?0:1]==20 && app()->route.targets[side?1:0]==140);
  assert(app()->route.task3.lost_start_ms==loss);
  line_inputs=side?11:13;
  for(unsigned n=0;n<3;n++){step(10,0,0);assert(app()->route.task3.gap_seek && a==0 && b==0);}
  step(10,0,0);
  assert(app()->runtime.state==RUNNING && !app()->route.task3.gap_seek && !app()->route.task3.lost_seen);
  assert(app()->route.base<=50.01f && app()->route.targets[0]>0 && app()->route.targets[1]>0);
  assert(app()->route.start_ms==start && app()->route.task3.leg_start_mm==origin && app()->route.task4.ac_reference_deg==reference);
  /* Losing an inner probe afterwards gets the old 20mm/12deg budget. */
  line_inputs=15;step(10,0,0);assert(!app()->route.task3.gap_seek);
  app()->route.task3.lost_start_mm-=20;step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && app()->route.task3.loss_limits==T3_LOSS_DISTANCE);
 }
 for(unsigned limit=0;limit<5;limit++) {
  measured_da_gap_start(0);
  if(limit==0 || limit==4) {
   for(unsigned n=0;n<49;n++)step(10,100,100);
   if(limit==4)line_inputs=13; /* A valid sample cannot beat the exact deadline. */
  } else if(limit==1)app()->route.task3.lost_start_mm-=35;
  else if(limit==2)app()->heading.angle-=18.1f;
  else app()->route.task3.lost_start_mm+=6;
  step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0 && app()->route.task3.gap_seek);
  assert(app()->route.task3.loss_limits==(limit==0 || limit==4?T3_LOSS_TIME:(limit==2?T3_LOSS_ANGLE:T3_LOSS_DISTANCE)));
  uint32_t now=(ms/1200U+1U)*1200U+400U;app()->ui.motion_log_ms=now-400U;CarUI_Log(app(),now,0);
  assert(strstr(logs,"GAP=1"));
  line_inputs=13;for(unsigned n=0;n<5;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 }
 for(unsigned f=0;f<7;f++) {
  measured_da_gap_start(0);
  switch(f) {
   case 0:off=1;break;
   case 1:queue(0,"STOP!");break;
   case 2:imu_fail=1;break;
   case 3:adc_fail=1;app()->power.sample_ms=ms-100;break;
   case 4:faults[0]=1;break;
   case 5:app()->route.start_ms=ms-DPT_TASK4_TIMEOUT_MS;break;
   case 6:step(30,0,0);break;
  }
  step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0);
 }
 CarTask3Config c=CarTask4_Config();assert(c.da_outer_gap && c.da_gap_mm==35 && c.da_gap_deg==18 && c.white_seek_ms==500);
 c.da_gap_mm=NAN;assert(!CarTask3_ConfigValid(&c));c=CarTask4_Config();c.da_gap_mm=36;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.da_gap_deg=19;assert(!CarTask3_ConfigValid(&c));c=CarTask4_Config();c.da_outer_gap=2;assert(!CarTask3_ConfigValid(&c));
 assert(!CarRoute_Task3Config().da_outer_gap);
 puts("PASS: measured-spacing DA-only outer-probe bridge, mirrored 140/20 targets, synthetic recovery beyond old 20mm/12deg, trusted-direction retention, 30ms confirmation and ramp, fixed 500ms/35mm/18deg bounds, inner-probe scope, fault priority, no restart and GAP telemetry. Synthetic inputs, not physical coverage acceptance.");
}
static void gap_regression(void)
{
 /* An O1-to-O2 (or O4-to-O3) white gap is still invalid, but may be swept
    toward the adjacent channel. Feeding that channel here is synthetic QA,
    not evidence that the real vehicle will cross its physical gap in budget. */
 for(unsigned side=0;side<2;side++) {
  gap_start(side);uint32_t start=app()->route.start_ms,loss=app()->route.task3.lost_start_ms;
  float origin=app()->route.task3.leg_start_mm,reference=app()->route.task4.ac_reference_deg;
  line_inputs=side?14:7;step(10,0,0);assert(a==0 && b==0);
  line_inputs=15;step(10,0,0);
  assert(app()->route.task3.lost_start_ms==loss && app()->route.targets[side?0:1]==20 && app()->route.targets[side?1:0]==80);
  assert(app()->route.task3_trace.valid_error==(side?-3.0f:3.0f));
  line_inputs=side?11:13; /* Adjacent inner probe. */
  for(unsigned n=0;n<3;n++){step(10,0,0);assert(app()->route.task3.lost_seen && a==0 && b==0);}
  step(10,0,0);
  assert(app()->runtime.state==RUNNING && !app()->route.task3.loss_seek && !app()->route.task3.lost_seen);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0 && app()->route.base<=50.01f);
  assert(app()->route.start_ms==start && app()->route.task3.leg_start_mm==origin && app()->route.task4.ac_reference_deg==reference);
  /* This edge-specific sweep does not change Task3's two-wheel bridge. */
  seek_arc(3);line_inputs=side?7:14;step(10,177,176);line_inputs=15;step(10,0,0);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0 && app()->route.base==40);
 }
 /* A sweep cannot bypass any existing fault boundary or restart after F7. */
 for(unsigned side=0;side<2;side++)for(unsigned limit=0;limit<4;limit++) {
  gap_start(side);
  if(limit==0) {
   for(unsigned n=0;n<49;n++)step(10,side?0:30,side?30:0);
  } else if(limit==1)app()->route.task3.lost_start_mm-=20;
  else if(limit==2)app()->heading.angle+=side?13:-13;
  else app()->route.task3.lost_start_mm+=6;
  step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
  assert(app()->route.task3.loss_limits==(limit==0?T3_LOSS_TIME:(limit==2?T3_LOSS_ANGLE:T3_LOSS_DISTANCE)));
  line_inputs=side?11:13;for(unsigned n=0;n<10;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 }
 puts("PASS: Task4 mirrored outer-to-inner probe gap sweeps, positive inner drive, retained direction under unconfirmed glitches, 30ms recovery, all original budgets and no restart; Task3 bridge unchanged. Synthetic inputs only.");
}
static void reverse_regression(void)
{
 for(unsigned side=0;side<2;side++) {
  unsigned inner=side?0:1;gap_start(side);
  for(unsigned n=0;n<4;n++) {
   step(10,side?-30:100,side?100:-30);
   assert(app()->runtime.state==RUNNING);
  }
  step(10,side?-30:100,side?100:-30);
  assert(app()->runtime.state==REVERSE && a==0 && b==0);
  assert(app()->runtime.fault_wheel==(side?'L':'R'));
  assert(app()->route.task3.phase==T3_FAULT && app()->route.task3.fault==T3_ABORT);
  assert(app()->route.reverse_trace.seen && app()->route.reverse_trace.phase==T3_ARC);
  assert(app()->route.reverse_trace.targets[inner]==20 && app()->route.reverse_trace.targets[1-inner]==80);
  assert(CarMath_Round(app()->route.reverse_trace.velocity[inner])==-10);
  float yaw=app()->route.reverse_trace.yaw;
  off=1;app()->heading.angle+=10;app()->ui.render_ms=ms-200;step(10,0,0);
  assert(app()->runtime.state==REVERSE && app()->route.reverse_trace.yaw==yaw);
  assert(strstr(oled_lines[2],side?"REV-L":"REV-R") && strstr(oled_lines[1],"P8 F2 AT5"));
  assert(strstr(oled_lines[3],side?"T20/80 V-10/34":"T80/20 V34/-10"));
  for(unsigned row=0;row<4;row++)assert(strlen(oled_lines[row])<=21);
  app()->route.elapsed_ms=159900;app()->route.task4.completed_laps=3;
  app()->ui.render_ms=ms-200;step(10,0,0);
  assert(strstr(oled_lines[2],"4/4 SEEK T159.9"));
  for(unsigned row=0;row<4;row++)assert(strlen(oled_lines[row])<=21);
  /* The D4 zero-target rollback hypothesis is reproducible as a protection
     input: the original reverse guard must still work for a zero target. */
  gap_start(side);app()->route.targets[inner]=0;app()->wheel.items[inner].velocity=-10;
  app()->runtime.dt_us=10000;
  for(unsigned n=0;n<5;n++)CarWheel_Control(app(),inner,.01f);
  assert(app()->runtime.state==REVERSE && app()->route.reverse_trace.targets[inner]==0 && a==0 && b==0);
 }
 /* A new task clears stale REV evidence; running cannot display an old snapshot. */
 seek_start(4);assert(!app()->route.reverse_trace.seen);
 puts("PASS: unchanged 50ms reverse guard for positive/zero targets, both fault wheels, pre-cancel phase and velocity snapshots, OLED REV-L/R, snapshot retention and fresh-start clearing. Synthetic encoder inputs only.");
}
int main(void)
{
 history_gap_regression();
 measured_da_gap_regression();
 reverse_regression();
 gap_regression();
 extended_task4_regression();
 CarTask3Config c=CarRoute_Task3Config();assert(CarTask3_ConfigValid(&c));
 assert(c.white_seek_ms==300 && CarTask4_Config().white_seek_ms==500);
 c.white_seek_ms=500;assert(CarTask3_ConfigValid(&c));
 c.white_seek_ms=501;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.white_seek_mm=21;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.white_seek_deg=13;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.vertex_seek_ms=3001;assert(!CarTask3_ConfigValid(&c));
 c=CarRoute_Task3Config();c.vertex_seek_mm=NAN;assert(!CarTask3_ConfigValid(&c));
 /* Full single/four-lap route under actual default config, without field dynamics claims. */
 for(unsigned task=3;task<=4;task++) {
  seek_start(task);
  for(unsigned n=0;n<12500 && app()->runtime.state==RUNNING;n++)step_route();
  assert(app()->runtime.state==DONE && a==0 && b==0);
  assert(signals==(task==3?5:17));
  if(task==4)assert(app()->route.task4.completed_laps==4);
 }
 /* The reported D-side miss now enters bounded search, never fabricates a D signal. */
 seek_d();int before=signals;
 step(10,0,0);assert(app()->route.targets[0]>0 && app()->route.targets[1]==0 && signals==before);
 line_inputs=11;
 for(unsigned n=0;n<3;n++){step(10,0,0);assert(app()->route.task3.phase==T3_VERTEX_SEEK && a==0 && b==0 && signals==before);}
 step(10,0,0);assert(app()->route.task3.phase==T3_VERTEX_BRAKE && signals==before+1);
 for(unsigned n=0;n<500 && app()->route.task3.phase!=T3_ARC;n++)step_route();
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC && app()->route.task3.leg==3);
 /* Search is terminated by original global vertex window, local time/distance and yaw. */
 for(unsigned limit=0;limit<4;limit++) {
  seek_d();before=signals;
  switch(limit) {
   case 0:app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+1401;break;
   case 1:app()->route.task3.phase_ms=ms-3000;break;
   case 2:app()->route.task3.phase_start_mm-=80;break;
   case 3:app()->heading.angle+=60;break;
  }
  step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0 && signals==before);
 }
 /* White vertex search aligns and settles before advancing, never reverses. */
 seek_d();
 /* Use production tangent formula directly for D (initial AB reference +180 degrees). */
 app()->heading.angle=CarMath_WrapDegrees(app()->route.task3.start_heading_deg+app()->route.task3.diagonal_angle_deg+180);
 for(unsigned n=0;n<21;n++){step(10,0,0);assert(a==0 && b==0);}
 assert(app()->route.task3.seek_aligned);step(10,0,0);
 assert(a>0 && b<0 && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 /* Both turns must remain halted while wheels still move or yaw leaves tolerance. */
 seek_start(3);reach_phase(T3_ALIGN_SETTLE);uint32_t turn_start=app()->route.task3.phase_ms;
 for(unsigned n=0;n<10;n++){step(10,177,176);assert(app()->route.task3.phase==T3_ALIGN_SETTLE && a==0 && b==0);}
 app()->heading.angle=CarMath_WrapDegrees(app()->heading.reference+5);step(10,0,0);
 assert(app()->route.task3.phase==T3_ALIGN && app()->route.task3.phase_ms==turn_start && a==0 && b==0);
 /* Transient all-white arc interval bridges slowly while preserving valid=false and fixed budgets. */
 for(unsigned task=3;task<=4;task++) {
  seek_arc(task);uint32_t start=app()->route.start_ms;float origin=app()->route.task3.leg_start_mm;
  line_inputs=15;step(10,0,0);
  assert(!app()->line.valid && app()->route.task3.loss_seek && a>0 && b<0);
  assert(app()->route.base==40 && app()->route.targets[0]<=56 && app()->route.targets[1]<=56);
  uint32_t loss=app()->route.task3_trace.lost_ms;
  for(unsigned n=0;n<12;n++)step(10,100,100);
  assert(app()->runtime.state==RUNNING && app()->route.task3_trace.lost_ms==loss);
  line_inputs=9;
  for(unsigned n=0;n<3;n++){step(10,0,0);assert(a==0 && b==0 && app()->route.task3_trace.lost_seen);}
  step(10,0,0);assert(app()->runtime.state==RUNNING && !app()->route.task3.lost_seen && !app()->route.task3_trace.lost_seen);
  assert(app()->route.base<=50.01f && app()->route.start_ms==start && app()->route.task3.leg_start_mm==origin);
 }
 /* An unconfirmed opposite-side glitch cannot replace the last trusted steering direction. */
 seek_arc(3);line_inputs=14;step(10,177,176);
 assert(app()->route.task3_trace.valid_error>0);
 line_inputs=15;step(10,0,0);assert(app()->route.targets[0]>app()->route.targets[1]);
 line_inputs=7;step(10,0,0);assert(app()->line.error<0 && a==0 && b==0);
 line_inputs=15;step(10,0,0);
 assert(app()->route.task3_trace.valid_error>0 && app()->route.targets[0]>app()->route.targets[1]);
 /* Every original bound is enforced and diagnosed in both tasks, including
    an exact deadline, late valid sample, reverse travel and simultaneous limits. */
 for(unsigned task=3;task<=4;task++)for(unsigned limit=0;limit<6;limit++) {
  seek_arc(task);line_inputs=15;step(10,0,0);
  if(limit==0 || limit==3) {
   /* Keep encoders moving above the stall threshold while staying below the
      distance limit, so this isolates the timeout rather than STALL at 500ms. */
   for(unsigned n=0;n<(task==3?29U:49U);n++)step(10,task==4?30:0,task==4?30:0);
   if(limit==3)line_inputs=9;
  } else if(limit==1)app()->route.task3.lost_start_mm-=20;
  else if(limit==4)app()->route.task3.lost_start_mm+=6;
  else if(limit==5) {
   app()->route.task3.lost_start_mm-=20;
   app()->heading.angle+=13;
  } else app()->heading.angle+=13;
  step(10,0,0);assert(app()->runtime.state==LINE_LOST && a==0 && b==0 && app()->route.task3.fault==T3_LOST);
  CarTask3 *t=&app()->route.task3;
  unsigned expected=limit==0 || limit==3?T3_LOSS_TIME:
   (limit==1 || limit==4?T3_LOSS_DISTANCE:(limit==5?T3_LOSS_DISTANCE|T3_LOSS_ANGLE:T3_LOSS_ANGLE));
  assert(t->loss_limits==expected);
  if(limit==0 || limit==3)assert(t->loss_duration_ms==(task==3?300U:500U));
  if(limit==1 || limit==5)assert(fabsf(t->loss_distance_mm-20)<.001f);
  if(limit==4)assert(fabsf(t->loss_distance_mm+6)<.001f);
  if(limit==2 || limit==5) {
   /* The scheduler now integrates a learned bias in this frame as well.
      Verify the actual fused angle is captured, rather than assuming the
      injected 13-degree change bypasses the production heading sample. */
   assert(fabsf(t->loss_turn_deg-CarMath_WrapDegrees(app()->heading.angle-t->lost_yaw_deg))<.001f);
   assert(t->loss_turn_deg>12.9f && t->loss_turn_deg<13.1f);
  }
  uint32_t duration=t->loss_duration_ms;
  uint32_t show=app()->route.task3_trace.lost_ms+4000U;
  app()->ui.render_ms=show-200;CarUI_UpdateOLED(app(),show,1);
  assert(strstr(oled_lines[1],expected==T3_LOSS_TIME?"CB LIMIT TIME":
   (expected==T3_LOSS_DISTANCE?"CB LIMIT DIST":(expected==T3_LOSS_ANGLE?"CB LIMIT ANGLE":"CB LIMIT MULTI"))));
  assert(strstr(oled_lines[3],"MS") && strstr(oled_lines[3]," D") && strstr(oled_lines[3]," A"));
  for(unsigned row=0;row<4;row++)assert(strlen(oled_lines[row])<=21);
  line_inputs=9;for(unsigned n=0;n<20;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST && a==0 && b==0 && t->loss_limits==expected && t->loss_duration_ms==duration);
 }
 /* No valid history, all-black and separated-black patterns never authorize positive search. */
 for(unsigned task=3;task<=4;task++)for(unsigned pattern=0;pattern<3;pattern++) {
  seek_start(task);reach_phase(T3_ARC);
  if(pattern==0) {
   /* Explicitly remove history: V13 normally seeds it at the confirmed handoff. */
   app()->route.task3.arc_valid_seen=0;
   memset(&app()->route.task3_trace,0,sizeof app()->route.task3_trace);
  }
  if(pattern) {line_inputs=9;step(10,0,0);}
  line_inputs=pattern==0?15:(pattern==1?0:5);step(10,0,0);
  assert(a==0 && b==0 && !app()->route.task3.loss_seek);
  for(unsigned n=0;n<(task==3?10U:20U);n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST);
 }
 /* Alternating white/valid samples do not renew either task's deadline. */
 for(unsigned task=3;task<=4;task++) {
  seek_arc(task);line_inputs=15;step(10,0,0);uint32_t loss=app()->route.task3.lost_start_ms;
  for(unsigned n=0;n<(task==3?29U:49U);n++) {line_inputs=n%2?15:9;step(10,0,0);assert(app()->route.task3.lost_start_ms==loss);}
  step(10,0,0);assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 }
 /* STOP, enable, sensor, scheduler and total watchdog override either search phase. */
 for(unsigned phase=0;phase<2;phase++)for(unsigned f=0;f<7;f++) {
  if(phase)seek_d();else {seek_arc(3);line_inputs=15;step(10,0,0);}
  switch(f) {
   case 0:off=1;break;
   case 1:queue(0,"STOP!");break;
   case 2:imu_fail=1;break;
   case 3:adc_fail=1;app()->power.sample_ms=ms-100;break;
   case 4:faults[0]=1;break;
   case 5:app()->route.start_ms=ms-40000;break;
   case 6:step(30,0,0);break;
  }
  step(10,0,0);assert(app()->runtime.state!=RUNNING && a==0 && b==0);
 }
 puts("PASS: default Task3/4 completion, D miss bounded acquisition without false point signals, settled turn/yaw recheck, all-white bridge/reacquisition, all bounds, invalid/no-history rejection, chatter deadline and fault priority. Software only.");
 return 0;
}
