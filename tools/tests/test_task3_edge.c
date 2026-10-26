#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void continuous_start(void)
{
 setup();line_inputs=15;hold();hold();hold();calibrate();enable();press_key();
 assert(app()->runtime.state==RUNNING && app()->route.motion==MOVE_TASK3);
 assert(app()->route.task3.config.continuous_motion && app()->route.task3.config.rolling_entry);
 assert(app()->route.task3.config.turn_inner_mmps==20 && app()->route.task3.config.turn_travel_limit_mm==120);
 assert(app()->route.targets[0]>0 && app()->route.targets[1]>0 && a>0 && b<0);
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

static void one_lap_regression(void)
{
 continuous_start();motor_before_monitor=app()->runtime.drive;app()->runtime.drive=monitor_motor;zero_handoffs=0;
 float residue[2]={0,0};
 unsigned turns=0,arcs=0;CarTask3Phase previous=T3_IDLE;
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
  if(t->phase!=previous && t->phase==T3_ARC)++arcs;
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
  assert(!app()->route.task4.active && app()->route.task4.completed_laps==0);
  assert(a>=0 && b<=0);
 }
 if(app()->runtime.state!=DONE) {
  const CarTask3 *t=&app()->route.task3;
  fprintf(stderr,"Task3 run: state=%d phase=%d fault=%d laps=%u arcs=%u ms=%lu turn_ms=%lu travel=%.2f progress=%.2f yaw=%.2f target=%.2f\n",
   app()->runtime.state,t->phase,t->fault,app()->route.task4.completed_laps,app()->route.task4.arc_number,
   (unsigned long)app()->route.elapsed_ms,(unsigned long)(ms-t->phase_ms),
   (app()->route.distance_mm[0]+app()->route.distance_mm[1])*.5f-t->phase_start_mm,
   progress(),app()->heading.angle,app()->heading.reference);
 }
 assert(app()->runtime.state==DONE && app()->route.task4.completed_laps==0 && arcs==2);
 assert(turns==4 && signals==5 && zero_handoffs==0 && a==0 && b==0);
 assert(app()->route.elapsed_ms<40000);
 printf("IDEAL Task3 one-lap continuous schedule: %lu ms; target-following/160mm track, not field acceptance.\n",(unsigned long)app()->route.elapsed_ms);
 app()->runtime.drive=motor_before_monitor;
}

static void scheduler_entry(unsigned leg,unsigned mask)
{
 setup();line_inputs=15;hold();hold();hold();calibrate();enable();press_key();
 for(unsigned n=0;n<7000 && !(app()->route.task3.leg==leg && app()->route.task3.phase==T3_CAPTURE);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.phase==T3_CAPTURE && app()->route.task3.config.edge_follow);
 line_inputs=mask;
 for(unsigned n=0;n<4;n++)step(10,100,100);
 assert(app()->runtime.state==RUNNING && app()->route.task3.phase==T3_ARC && app()->route.task3.edge_follow_active);
}

static void edge_and_timeout(void)
{
 for(unsigned leg=1;leg<=3;leg+=2)for(unsigned side=0;side<2;side++) {
  scheduler_entry(leg,side?14:7);
  assert(app()->route.motion==MOVE_TASK3 && app()->route.run_duration==40000);
  assert(app()->route.targets[side?0:1]==120 && app()->route.targets[side?1:0]==20);
  line_inputs=15;step(10,100,100);
  assert(app()->route.targets[side?0:1]==80 && app()->route.targets[side?1:0]==20);
  line_inputs=side?13:11;
  for(unsigned n=0;n<8;n++)step(10,30,30);
  assert(app()->runtime.state==RUNNING && !app()->route.task3.lost_seen);
 }
 scheduler_entry(1,7);line_inputs=15;
 for(unsigned n=0;n<51 && app()->runtime.state==RUNNING;n++)step(10,30,30);
 assert(app()->runtime.state==LINE_LOST && a==0 && b==0 && signals==2);
 for(unsigned final=0;final<2;final++) {
  continuous_start();
  app()->route.start_ms=ms-39990;
  app()->route.task3.start_ms=ms-39990;
  if(final)app()->route.task3.phase=T3_FINAL_BRAKE;
  step(10,0,0);assert(app()->runtime.state==TIMEOUT && a==0 && b==0);
 }
}
static void task3_guards(void)
{
 for(unsigned event=0;event<10;event++) {
  scheduler_entry(1,7);TestState expected=STOPPED;
  switch(event) {
   case 0:queue(0,"STOP!TASK3!");break;
   case 1:off=1;expected=OFF;break;
   case 2:imu_fail=1;expected=YAW_BAD;break;
   case 3:adc_fail=1;app()->power.sample_ms=ms-100;expected=ADC_BAD;break;
   case 4:bat=1000;app()->power.sample_ms=ms-100;expected=LOW;break;
   case 5:faults[0]=1;expected=UART_BAD;break;
   case 6:step(30,30,30);expected=GAP;break;
   case 7:step(10,2000,0);expected=FAST;break;
   case 8:for(unsigned n=0;n<5;n++)step(10,-177,176);expected=REVERSE;break;
   case 9:for(unsigned n=0;n<60 && app()->runtime.state==RUNNING;n++)step(10,0,0);expected=STALL;break;
  }
  step(10,0,0);assert(app()->runtime.state==expected && a==0 && b==0);
 }
}
int main(void)
{
 CarTask3Config c=CarRoute_Task3Config(),four=CarTask4_Config();
 assert(c.deadline_ms==40000 && four.deadline_ms==80000);
 assert(c.edge_follow && c.continuous_motion && c.rolling_entry && c.arc_entry_ms>0);
 assert(c.diagonal_mmps==four.diagonal_mmps && c.arc_mmps==four.arc_mmps);
 assert(c.turn_inner_mmps==20 && c.turn_mmps==100);
 assert(c.edge_follow_mmps==70 && c.edge_follow_diff_mmps==50);
 one_lap_regression();edge_and_timeout();task3_guards();
 puts("PASS: production Task3 EDGE, continuous one lap, five point signals, zero intermediate stops, 40s timeout and scheduler guards.");
 return 0;
}
