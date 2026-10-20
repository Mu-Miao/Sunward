/* Full production Task4 continuity. Synthetic inputs, no hardware access. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void continuous_start(void)
{
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 assert(app()->runtime.state==RUNNING && app()->route.motion==MOVE_TASK4);
 assert(app()->route.task3.config.continuous_motion && app()->route.task3.config.rolling_entry);
 assert(app()->route.task3.config.turn_inner_mmps==20 && app()->route.task3.config.turn_travel_limit_mm==120);
 assert(app()->route.targets[0]>0 && app()->route.targets[1]>0 && a>0 && b<0);
}

static void reach_continuous(unsigned lap,unsigned leg,CarTask3Phase phase)
{
 for(unsigned n=0;n<14000 && app()->runtime.state==RUNNING &&
     !(app()->route.task4.completed_laps==lap && app()->route.task3.leg==leg && app()->route.task3.phase==phase);n++)step_route();
 assert(app()->runtime.state==RUNNING && app()->route.task4.completed_laps==lap &&
  app()->route.task3.leg==leg && app()->route.task3.phase==phase);
}

static unsigned zero_handoffs;
static void (*motor_before_monitor)(int,int);
static void monitor_motor(int ac,int bc)
{
 CarTask3Phase p=app()->route.task3.phase;
 if(app()->runtime.state==RUNNING && p!=T3_FINAL_BRAKE && p!=T3_DONE && p!=T3_FAULT &&
    app()->route.targets[0]+app()->route.targets[1]==0 && ac==0 && bc==0)++zero_handoffs;
 motor_before_monitor(ac,bc);
}

static void four_lap_regression(void)
{
 continuous_start();motor_before_monitor=app()->runtime.drive;app()->runtime.drive=monitor_motor;zero_handoffs=0;
 float residue[2]={0,0},reference=app()->route.task4.ac_reference_deg;
 unsigned previous_laps=0,joins=0,turns=0;CarTask3Phase previous=T3_IDLE;
 for(unsigned n=0;n<16500 && app()->runtime.state==RUNNING;n++) {
  CarTask3 *t=&app()->route.task3;float p=progress();
  if(t->phase==T3_DIAGONAL) {
   app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+
    (t->leg==2?180+2*t->diagonal_angle_deg-t->config.bd_turn_relief_deg:0));
   line_inputs=p>=t->diagonal_mm?9:15;
  } else if(t->phase==T3_ARC) {
   line_inputs=p>=t->config.arc_mm?15:9;
   app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+t->diagonal_angle_deg+
    (t->leg==1?fminf(p/t->config.arc_mm,1)*180:180-fminf(p/t->config.arc_mm,1)*180));
  } else if(t->phase==T3_CAPTURE || t->phase==T3_ARC_ENTRY)line_inputs=9;
  else if(t->phase==T3_ALIGN || t->phase==T3_START_ALIGN || t->phase==T3_VERTEX_SEEK) {
   if(t->phase==T3_VERTEX_SEEK)line_inputs=p>=t->diagonal_mm?9:15;
   app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+
    (app()->route.targets[1]-app()->route.targets[0])*.01f/160*57.2957795131f);
  }
  if(t->phase!=previous && (t->phase==T3_ALIGN || t->phase==T3_START_ALIGN))++turns;
  previous=t->phase;
  assert(t->phase!=T3_VERTEX_BRAKE && t->phase!=T3_ALIGN_SETTLE && t->phase!=T3_START_SETTLE);
  int counts[2];
  for(unsigned w=0;w<2;w++) {
   float target=app()->route.targets[w];
   if(t->phase!=T3_FINAL_BRAKE)assert(target>0 && target<=336);
   residue[w]+=target*.01f*(w?DPT_RIGHT_CPR:DPT_LEFT_CPR)/(DPT_CIRC_M*1000);
   counts[w]=(int)residue[w];residue[w]-=counts[w];
  }
  step(10,counts[0],counts[1]);
  if(app()->route.task4.completed_laps>previous_laps && app()->route.task4.completed_laps<4) {
   ++joins;assert(app()->route.task3.phase==T3_START_ALIGN);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
   assert(!app()->route.task3.pending_start_signal && app()->route.task3.start_heading_deg==reference);
  }
  previous_laps=app()->route.task4.completed_laps;
  assert(a>=0 && b<=0);
 }
 if(app()->runtime.state!=DONE) {
  const CarTask3 *t=&app()->route.task3;
  fprintf(stderr,"continuous run: state=%d phase=%d fault=%d laps=%u arcs=%u ms=%lu turn_ms=%lu travel=%.2f progress=%.2f yaw=%.2f target=%.2f\n",
   app()->runtime.state,t->phase,t->fault,app()->route.task4.completed_laps,app()->route.task4.arc_number,
   (unsigned long)app()->route.elapsed_ms,(unsigned long)(ms-t->phase_ms),
   (app()->route.distance_mm[0]+app()->route.distance_mm[1])*.5f-t->phase_start_mm,
   progress(),app()->heading.angle,app()->heading.reference);
 }
 assert(app()->runtime.state==DONE && app()->route.task4.completed_laps==4 && app()->route.task4.arc_number==8);
 assert(joins==3 && turns==16 && signals==17 && zero_handoffs==0 && a==0 && b==0);
 assert(app()->route.elapsed_ms<160000);
 printf("IDEAL continuous four-lap schedule: %lu ms; target-following/160mm track, not field acceptance.\n",(unsigned long)app()->route.elapsed_ms);
 app()->runtime.drive=motor_before_monitor;
}

static void dual_turn_regression(void)
{
 continuous_start();CarTask3Output out={.control=T3_TURN_FORWARD,.heading_deg=0,.left_mmps=80,.right_mmps=20};
 app()->heading.angle=30;CarTask3Port_Apply(app(),&out,ms);
 assert(app()->route.targets[0]==80 && app()->route.targets[1]==20);
 app()->heading.angle=7.5f;CarTask3Port_Apply(app(),&out,ms);
 assert(app()->route.targets[0]==55 && app()->route.targets[1]==20);
 out.left_mmps=20;out.right_mmps=80;app()->heading.angle=-7.5f;CarTask3Port_Apply(app(),&out,ms);
 assert(app()->route.targets[0]==20 && app()->route.targets[1]==55);
 /* A, B, C, D all confirm angles with moving wheels and emit motion
    on the same update that enters the next straight/capture phase. */
 for(unsigned leg=0;leg<4;leg++) {
  CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output o;
  CarTask3Input in={.enabled=1,.heading_ready=1,.now_ms=1000};assert(CarTask3_Start(&t,&c,&in));
  t.leg=leg;t.phase=leg?T3_ALIGN:T3_START_ALIGN;t.pending_start_signal=0;
  float target=CarMath_WrapDegrees(t.start_heading_deg+(leg==0?0:
   (leg==2?180+2*t.diagonal_angle_deg-c.bd_turn_relief_deg:t.diagonal_angle_deg+(leg==3?180:0))));
  in.yaw_deg=CarMath_WrapDegrees(target-8);in.left_mmps=in.right_mmps=50;
  in.now_ms+=10;CarTask3_Update(&t,&in,&o);
  assert(o.control==T3_TURN_FORWARD && o.left_mmps>0 && o.right_mmps>0 && !o.reset_controllers);
  in.yaw_deg=target;
  for(unsigned n=0;n<4;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&o);assert(o.control!=T3_HALT && !o.reset_controllers);}
  assert(t.phase==(leg&1?T3_CAPTURE:T3_DIAGONAL));
 }
}

static void moving_recovery_regression(void)
{
 continuous_start();reach_continuous(0,1,T3_ARC);line_inputs=9;step(10,30,30);
 line_inputs=15;step(10,30,30);uint32_t lost=app()->route.task3.lost_start_ms;
 assert(app()->route.task3.loss_seek && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 line_inputs=9;
 for(unsigned n=0;n<3;n++) {
  step(10,30,30);assert(app()->runtime.state==RUNNING && app()->route.task3.lost_seen);
  assert(app()->route.targets[0]>0 && app()->route.targets[1]>0 && app()->route.task3.lost_start_ms==lost);
 }
 step(10,30,30);assert(!app()->route.task3.lost_seen && app()->route.task3.phase==T3_ARC);
 /* Fresh all-white loss with one-frame glitches cannot renew its500ms deadline. */
 line_inputs=15;step(10,30,30);lost=app()->route.task3.lost_start_ms;
 for(unsigned n=0;n<49;n++) {
  line_inputs=n%2?15:9;step(10,30,30);
  assert(app()->runtime.state==RUNNING && app()->route.task3.lost_start_ms==lost);
 }
 step(10,30,30);assert(app()->runtime.state==LINE_LOST && a==0 && b==0);
 /* Non-white invalid patterns still stop; continuous is not blind driving. */
 for(unsigned pattern=0;pattern<2;pattern++) {
  continuous_start();reach_continuous(0,1,T3_ARC);line_inputs=pattern?5:0;
  step(10,30,30);assert(a==0 && b==0 && !app()->route.task3.loss_seek);
  for(unsigned n=0;n<20;n++)step(10,0,0);
  assert(app()->runtime.state==LINE_LOST);
 }
 /* The missing-vertex search has moving angle confirmation too. */
 CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output o;
 CarTask3Input in={.enabled=1,.heading_ready=1,.now_ms=1000};assert(CarTask3_Start(&t,&c,&in));
 t.phase=T3_VERTEX_SEEK;t.pending_start_signal=0;in.distance_mm=t.diagonal_mm;t.phase_start_mm=in.distance_mm;
 in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+t.diagonal_angle_deg);
 in.left_mmps=in.right_mmps=40;
 for(unsigned n=0;n<4;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&o);assert(o.control==T3_HOLD_HEADING && o.base_mmps==40);}
 assert(t.seek_aligned);
}

static void fault_regression(void)
{
 for(unsigned event=0;event<12;event++) {
  continuous_start();reach_continuous(0,2,T3_ALIGN);
  TestState expected=STOPPED;
  switch(event) {
  case 0:queue(0,"STOP!TASK4!");break;
  case 1:off=1;expected=OFF;break;
  case 2:imu_fail=1;expected=YAW_BAD;break;
  case 3:adc_fail=1;app()->power.sample_ms=ms-100;expected=ADC_BAD;break;
  case 4:bat=1000;app()->power.sample_ms=ms-100;expected=LOW;break;
  case 5:faults[0]=1;expected=UART_BAD;break;
  case 6:step(30,30,30);expected=GAP;break;
  case 7:step(10,2000,0);expected=FAST;break;
  case 8:for(unsigned n=0;n<5;n++)step(10,-177,176);expected=REVERSE;break;
  case 9:for(unsigned n=0;n<60 && app()->runtime.state==RUNNING;n++)step(10,0,0);expected=STALL;break;
  case 10:app()->route.task3.phase_ms=ms-4000;expected=YAW_BAD;break;
  case 11:app()->route.task3.phase_start_mm-=121;expected=YAW_BAD;break;
  }
  step(10,0,0);assert(app()->runtime.state==expected && a==0 && b==0);
  for(unsigned n=0;n<5;n++)step(10,0,0);assert(app()->runtime.state==expected && a==0 && b==0);
 }
 continuous_start();reach_continuous(1,0,T3_START_ALIGN);app()->route.start_ms=ms-160000;
 step(10,30,30);assert(app()->runtime.state==TIMEOUT && a==0 && b==0 && app()->route.task4.completed_laps==1);
 /* Moving lap restart rejects missing sensors/STOP before emitting output. */
 for(unsigned event=0;event<3;event++) {
  CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.enabled=1,.heading_ready=1};assert(CarTask3_Start(&t,&c,&in));
  t.phase=T3_DONE;in.left_mmps=in.right_mmps=50;
  if(event==0)in.stop_requested=1;else if(event==1)in.sample_age_ms=21;else in.heading_ready=0;
  assert(!CarTask3_NextLap(&t,&in,&out) && t.phase==T3_FAULT && out.control==T3_HALT);
 }
}

int main(void)
{
 CarTask3Config c=CarTask4_Config();assert(CarTask3_ConfigValid(&c));
 c.continuous_motion=2;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.turn_inner_mmps=0;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.turn_inner_mmps=NAN;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.turn_travel_limit_mm=121;assert(!CarTask3_ConfigValid(&c));
 assert(!CarRoute_Task3Config().continuous_motion && CarRoute_Task3Config().turn_travel_limit_mm==80);
 four_lap_regression();dual_turn_regression();moving_recovery_regression();fault_regression();
 puts("PASS: production Task4 continuous A/B/C/D and3 rolling lap joins, both wheels positive through16 turns, no normal zero-drive handoffs, final settled completion, bounded moving recovery/search and retained fault stops. Software only.");
 return 0;
}
