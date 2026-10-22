/* V14-based anticipation: actual core and scheduler, synthetic sensors only. */
#define CAR_TASK3_TEST_MAIN legacy_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void preview_regression(void)
{
 for(unsigned leg=0;leg<=2;leg+=2) {
  for(unsigned event=0;event<8;event++) {
   CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output o;
   CarTask3Input in={.now_ms=1000,.enabled=1,.heading_ready=1};
   assert(CarTask3_Start(&t,&c,&in));
   t.leg=leg;t.phase=T3_DIAGONAL;t.pending_start_signal=0;
   in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+(leg?180+2*t.diagonal_angle_deg-c.bd_turn_relief_deg:0));
   in.distance_mm=t.diagonal_mm-50;in.left_mmps=in.right_mmps=160;
   in.line_valid=in.centered=1;in.black_count=2;
   if(event==1)in.distance_mm=t.diagonal_mm-c.entry_window_mm-1;
   if(event==2){in.line_valid=in.centered=0;in.black_count=0;}
   if(event==3){in.line_valid=in.centered=0;in.black_count=4;}
   if(event==4)in.stop_requested=1;
   if(event==5)in.sample_age_ms=21;
   if(event==6)in.distance_mm=t.diagonal_mm+c.entry_window_mm+1;
   if(event==7)in.yaw_deg=CarMath_WrapDegrees(in.yaw_deg+21);
   in.now_ms+=10;CarTask3_Update(&t,&in,&o);
   if(event>=4) {
    assert(t.phase==T3_FAULT && o.control==T3_HALT && o.reset_controllers);
   } else if(event) {
    assert(t.phase==T3_DIAGONAL && o.control==T3_HOLD_HEADING && !t.edge_seen);
   } else {
    assert(t.phase==T3_DIAGONAL && t.leg==leg && o.signal_point==T3_NO_POINT);
    assert(o.control==T3_TURN_FORWARD && !o.reset_controllers && o.retarget_integrals);
    assert(o.left_mmps>0 && o.right_mmps>0 && o.left_mmps!=o.right_mmps);
    assert(leg?o.left_mmps>o.right_mmps:o.left_mmps<o.right_mmps);
    for(unsigned n=0;n<3;n++) {
     in.now_ms+=10;CarTask3_Update(&t,&in,&o);
     assert(o.control==T3_TURN_FORWARD && !o.reset_controllers);
     assert(o.signal_point==(n==2?(leg?T3_POINT_D:T3_POINT_C):T3_NO_POINT));
    }
    assert(t.phase==T3_ALIGN && t.leg==leg+1);
   }
  }
  /* A single candidate cannot report a point or advance the leg. */
  CarTask3Config c=CarTask4_Config();CarTask3 t={0};CarTask3Output o;
  CarTask3Input in={.now_ms=1000,.enabled=1,.heading_ready=1};
  assert(CarTask3_Start(&t,&c,&in));t.phase=T3_DIAGONAL;t.leg=leg;t.pending_start_signal=0;
  in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+(leg?180+2*t.diagonal_angle_deg-c.bd_turn_relief_deg:0));
  in.distance_mm=t.diagonal_mm-50;in.line_valid=in.centered=1;in.black_count=2;
  in.now_ms+=10;CarTask3_Update(&t,&in,&o);assert(o.control==T3_TURN_FORWARD);
  in.line_valid=in.centered=0;in.black_count=0;
  in.now_ms+=10;CarTask3_Update(&t,&in,&o);
  assert(t.phase==T3_DIAGONAL && t.leg==leg && !t.edge_seen && o.signal_point==T3_NO_POINT);
  assert(o.control==T3_HOLD_HEADING && !o.reset_controllers);
 }
}

static uint32_t schedule(int parked)
{
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 assert(app()->runtime.state==RUNNING && app()->route.motion==MOVE_TASK4);
 if(parked) {
  /* Restore the supplied V14 route profile, including its original turn limit. */
  CarTask3 *t=&app()->route.task3;
  t->config.rolling_entry=t->config.continuous_motion=0;
  t->config.turn_travel_limit_mm=80;
  t->config.turn_mmps=80;
  app()->route.targets[0]=80;app()->route.targets[1]=0;
 }
 float residue[2]={0,0};
 for(unsigned n=0;n<16500 && app()->runtime.state==RUNNING;n++) {
  CarTask3 *t=&app()->route.task3;float p=progress();
  if(t->phase==T3_DIAGONAL) {
   app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+
    (t->leg==2?180+2*t->diagonal_angle_deg-t->config.bd_turn_relief_deg:0));
   line_inputs=p>=t->diagonal_mm-45?9:15;
  } else if(t->phase==T3_ARC) {
   line_inputs=p>=t->config.arc_mm?15:9;
   app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+t->diagonal_angle_deg+
    (t->leg==1?fminf(p/t->config.arc_mm,1)*180:180-fminf(p/t->config.arc_mm,1)*180));
  } else if(t->phase==T3_CAPTURE || t->phase==T3_ARC_ENTRY)line_inputs=9;
  else if(t->phase==T3_ALIGN || t->phase==T3_START_ALIGN || t->phase==T3_VERTEX_SEEK) {
   app()->heading.angle=CarMath_WrapDegrees(app()->heading.angle+
    (app()->route.targets[1]-app()->route.targets[0])*.01f/160*57.2957795131f);
  }
  int counts[2];
  for(unsigned w=0;w<2;w++) {
   residue[w]+=app()->route.targets[w]*.01f*(w?DPT_RIGHT_CPR:DPT_LEFT_CPR)/(DPT_CIRC_M*1000);
   counts[w]=(int)residue[w];residue[w]-=counts[w];
  }
  step(10,counts[0],counts[1]);assert(a>=0 && b<=0);
 }
 if(app()->runtime.state!=DONE)fprintf(stderr,"preview schedule parked=%d state=%d phase=%d fault=%d laps=%u\n",parked,
  app()->runtime.state,app()->route.task3.phase,app()->route.task3.fault,app()->route.task4.completed_laps);
 assert(app()->runtime.state==DONE && app()->route.task4.completed_laps==4 && signals==17 && a==0 && b==0);
 return app()->route.elapsed_ms;
}

int main(void)
{
 preview_regression();
 uint32_t before=schedule(1),after=schedule(0);
 assert(after<before);
 printf("IDEAL same-cruise four laps, 160mm track, 45mm probe lead: V14=%lu ms FLOW=%lu ms; software schedule only.\n",
  (unsigned long)before,(unsigned long)after);
 puts("PASS: C/D first-candidate turn, original window and debounce, no false point on glitches, STOP/sensor/heading/distance priority, four-lap comparison.");
 return 0;
}
