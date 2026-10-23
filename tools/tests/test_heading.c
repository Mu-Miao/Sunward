/* dev 005533e is a reference; these test the current gated implementation. */
#define CAR_TASK3_TEST_MAIN heading_task3_regression_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN

static void kernel_regression(void)
{
 assert(CarHeading_ConfigValid() && DPT_TRACK_WIDTH_MM==160);
 float bias=0,rate=0;
 for(unsigned n=0;n<1000;n++)rate=CarHeading_FuseRate(.4f,200,200,.01f,1,&bias);
 assert(bias>.35f && bias<.45f && fabsf(rate)<.05f);
 bias=.3f;
 float turn=(300.0f-100.0f)*57.2957795131f/160.0f;
 rate=CarHeading_FuseRate(turn+.3f,100,300,.01f,1,&bias);
 assert(bias==.3f && fabsf(rate-turn)<.0001f);
 rate=CarHeading_FuseRate(.4f,200,200,.01f,0,&bias);
 assert(bias==.3f && fabsf(rate-.1f)<.0001f);
 CarHeading_FuseRate(.4f,10,10,.01f,1,&bias);assert(bias==.3f);
 CarHeading_FuseRate(.4f,-200,-200,.01f,1,&bias);assert(bias==.3f);
 CarHeading_FuseRate(.4f,500,500,.01f,1,&bias);assert(bias==.3f);
 CarHeading_FuseRate(10,200,200,.01f,1,&bias);assert(bias==.3f);
 bias=0;
 for(unsigned n=0;n<3000;n++)CarHeading_FuseRate(2.0f,200,200,.01f,1,&bias);
 assert(bias==1.0f);
 assert(isnan(CarHeading_FuseRate(NAN,200,200,.01f,1,&bias)) && bias==1);
 assert(isnan(CarHeading_FuseRate(0,INFINITY,200,.01f,1,&bias)) && bias==1);
 assert(isnan(CarHeading_FuseRate(0,200,200,.1f,1,&bias)) && bias==1);
 assert(isnan(CarHeading_FuseRate(0,200,200,.01f,1,NULL)));
 bias=NAN;assert(isnan(CarHeading_FuseRate(0,200,200,.01f,1,&bias)));
}

static CarAppContext *fusion_context(Motion motion, CarTask3Phase phase)
{
 setup();CarAppContext *c=app();
 c->heading.phase=Y_READY;c->heading.bias=-424;c->heading.cycles=DWT->CYCCNT;
 c->heading.previous_rate=c->heading.previous_gyro_rate=0; /* Same seed as a stationary CAL. */
 c->runtime.state=RUNNING;c->route.motion=motion;c->route.task3.phase=phase;c->route.switching=0;
 c->route.targets[0]=c->route.targets[1]=200;
 c->wheel.items[0].velocity=c->wheel.items[1].velocity=200;
 gz=-424+52;
 return c;
}
static void sample_heading(unsigned n,int enabled)
{
 for(unsigned i=0;i<n;i++) {
  ms+=10;DWT->CYCCNT+=504000U; /* +3ms mock MPU read gives a 10ms frame. */
  CarHeading_Sample(app(),ms,enabled,1);
  assert(app()->heading.phase==Y_READY);
 }
}
static void integration_regression(void)
{
 assert(DPT_HEADING_FUSION_ENABLE==0);
 CarAppContext *c;
 for(unsigned task=3;task<=4;task++) {
  c=fusion_context(task==3?MOVE_TASK3:MOVE_TASK4,T3_DIAGONAL);
  sample_heading(800,1);
  assert(!c->heading.fusion_active && !c->heading.fusion_learning && !c->heading.fusion_window_seen);
  assert(c->heading.bias_rate==0 && c->heading.gyro_angle>3 && c->heading.gyro_angle<3.3f);
  assert(fabsf(c->heading.angle-c->heading.gyro_angle)<.00001f);
  CarTask3Phase phases[]={T3_START_ALIGN,T3_START_SETTLE,T3_ALIGN,T3_ALIGN_SETTLE,
   T3_CAPTURE,T3_ARC,T3_VERTEX_SEEK,T3_FINAL_BRAKE};
  for(unsigned i=0;i<sizeof phases/sizeof phases[0];i++) {
   c->route.task3.phase=phases[i];sample_heading(30,1);
   assert(c->heading.bias_rate==0 && !c->heading.fusion_active && !c->heading.fusion_learning);
   assert(fabsf(c->heading.angle-c->heading.gyro_angle)<.00001f);
  }
  /* Reproduce the observed negative learned bias: it must not rotate the
     reported heading while the static-CAL gyro rate is zero, even if stale. */
  c->heading.bias_rate=-.34f;
  c->heading.angle=c->heading.gyro_angle=-44.93f;
  c->heading.previous_rate=c->heading.previous_gyro_rate=0;gz=-424;
  c->wheel.items[0].velocity=c->wheel.items[1].velocity=0;
  c->runtime.state=STOPPED;sample_heading(300,1);
  assert(c->heading.angle==-44.93f && c->heading.gyro_angle==-44.93f);
  assert(!c->heading.fusion_active && !c->heading.fusion_learning);
 }
 /* Pure-gyro integration survives both counter wraps. */
 c=fusion_context(MOVE_TASK4,T3_DIAGONAL);ms=UINT32_MAX-100U;DWT->CYCCNT=UINT32_MAX-200000U;
 c->heading.cycles=DWT->CYCCNT;sample_heading(30,1);
 assert(c->heading.bias_rate==0 && fabsf(c->heading.angle-c->heading.gyro_angle)<.00001f);
 /* Task1/2 and independent tests keep the original pure-gyro path. */
 Motion unchanged[]={MOVE_ROUTE,MOVE_DISTANCE,MOVE_HEADING,MOVE_TASK2};
 for(unsigned i=0;i<sizeof unchanged/sizeof unchanged[0];i++) {
  c=fusion_context(unchanged[i],T3_DIAGONAL);c->heading.bias_rate=.3f;sample_heading(100,1);
  assert(!c->heading.fusion_active && c->heading.bias_rate==.3f);
  assert(fabsf(c->heading.angle-c->heading.gyro_angle)<.00001f);
 }
 c=fusion_context(MOVE_TASK4,T3_DIAGONAL);sample_heading(100,1);
 c->heading.bias_rate=-.34f;
 c->runtime.state=IDLE;CarHeading_BeginCalibration(c,ms,0);
 assert(c->heading.phase==Y_WARM && c->heading.bias_rate==0 && c->heading.gyro_angle==0 && !c->heading.fusion_window_seen);
 /* Keep explicit OFF diagnostics available even though fusion is disabled. */
 c=fusion_context(MOVE_TASK4,T3_DIAGONAL);gz=-424;sample_heading(300,1);
 c->heading.available=1;c->ui.heading_log_ms=ms-1000;logs[0]=0;
 CarUI_Log(c,ms,1);
 assert(strstr(logs,"FUS Y100=0 G100=0 B100=0 W100=0 LEARN=0 TRACK=160 EN=0"));
}

static void vertex_diagnostics_regression(void)
{
 task3_start();
 for(unsigned n=0;n<3500 && !(app()->route.task3.leg==2 && app()->route.task3.phase==T3_DIAGONAL);n++)step_route();
 assert(app()->route.task3.leg==2 && app()->route.task3.phase==T3_DIAGONAL);
 /* First prove the vehicle has left B, then inject black at 1165mm. Endpoint
    recognition is line-event driven and the diagnostic records recognition,
    not a claim that nominal 1280.6mm was physically reached. */
 line_inputs=15;
 for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(app()->route.task3.diagonal_armed);
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=app()->route.task3.leg_start_mm+1165;
 app()->heading.angle=join_target();app()->heading.gyro_angle=88;app()->heading.bias_rate=.2f;
 line_inputs=14;
 for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(app()->route.vertex_trace.seen && app()->route.vertex_trace.point==T3_POINT_D);
 assert(app()->route.vertex_trace.distance_mm==1165 && app()->route.vertex_trace.raw[0]==0);
 assert(app()->route.vertex_trace.lap==1 && fabsf(app()->route.vertex_trace.gyro_yaw-88)<.01f);
 float yaw=app()->route.vertex_trace.yaw;
 reach_phase(T3_ARC);line_inputs=14;step(10,0,0);line_inputs=15;step(10,0,0);
 for(unsigned n=0;n<11;n++)step(10,0,0);
 assert(app()->runtime.state==LINE_LOST);
 uint32_t show=app()->route.task3_trace.lost_ms+6000U;
 app()->ui.render_ms=show-200;CarUI_UpdateOLED(app(),show,1);
 assert(strcmp(oled_lines[1],"D1 S1165 L0111")==0 && strstr(oled_lines[3],"G88 B20"));
 assert(app()->route.vertex_trace.yaw==yaw);
 for(unsigned row=0;row<4;row++)assert(strlen(oled_lines[row])<=21);
}

static void edge_diagnostics_regression(void)
{
 CarTask3Phase phases[]={T3_DIAGONAL,T3_VERTEX_SEEK,T3_VERTEX_SEEK,T3_CAPTURE,T3_CAPTURE,T3_ARC};
 for(unsigned k=0;k<6;k++) {
  task3_start();CarAppContext *c=app();CarTask3 *t=&c->route.task3;
  t->config=CarRoute_Task3Config(); /* Exercise the production D-seek budgets. */
  assert(!c->route.edge_trace.seen && !c->route.vertex_trace.seen);
  t->leg=k>=3?3U:2U;t->phase=phases[k];t->leg_start_mm=0;
  float distance=k==0?t->diagonal_mm+t->config.entry_window_mm+1:
    (k==5?t->config.arc_mm+t->config.exit_window_mm+1:1323);
  float travel=k==2?80:(k==4?61:42);
  uint32_t age=k==1?2990U:(k==3?1990U:1000U);
  t->phase_start_mm=distance-travel;t->phase_ms=ms-age;
  c->route.distance_mm[0]=c->route.distance_mm[1]=distance;
  c->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+180+2*t->diagonal_angle_deg);
  line_inputs=15;step(10,0,0);
  assert(c->runtime.state==LINE_BAD && t->fault==T3_EDGE && a==0 && b==0);
  assert(c->route.edge_trace.seen && c->route.edge_trace.phase==phases[k]);
  assert(c->route.edge_trace.limits==((k==1 || k==3)?T3_LOSS_TIME:T3_LOSS_DISTANCE));
  assert(c->route.edge_trace.duration_ms==age+10 && c->route.edge_trace.travel_mm==travel);
  uint32_t show=c->route.edge_trace.ms;
  c->ui.render_ms=show-200;CarUI_UpdateOLED(c,show,1);
  assert(strstr(oled_lines[1],k==0?"BD DIAG DIST":(k==1?"BD SEEK TIME":
    (k==2?"BD SEEK DIST":(k==3?"DA CAP TIME":(k==4?"DA CAP DIST":"DA ARC DIST"))))));
  assert(strstr(oled_lines[3],"MS") && strstr(oled_lines[3]," D"));
  float yaw=c->route.edge_trace.yaw;
  off=1;line_inputs=0;c->heading.angle=0;step(10,0,0);
  show+=2000;c->ui.render_ms=show-200;CarUI_UpdateOLED(c,show,0);
  assert(strstr(oled_lines[1],"LF1111") && strstr(oled_lines[3],"P8 F6 S"));
  assert(c->route.edge_trace.yaw==yaw);
  for(unsigned row=0;row<4;row++)assert(strlen(oled_lines[row])<=21);
 }
}

static void diagnostic_logging_regression(void)
{
 CarAppContext *c=fusion_context(MOVE_TASK4,T3_ARC);CarTask3 *t=&c->route.task3;
 t->config=CarTask4_Config();t->leg=3;t->leg_start_mm=4000;
 c->route.distance_mm[0]=c->route.distance_mm[1]=4176;
 c->route.elapsed_ms=31000;c->route.goal=20300;
 c->route.targets[0]=168;c->route.targets[1]=72;
 c->wheel.items[0].velocity=141;c->wheel.items[1].velocity=57;
 c->wheel.items[0].command=c->wheel.items[1].command=3000;
 c->wheel.items[0].integral=c->wheel.items[1].integral=2200;
 c->ui.motion_log_ms=0;logs[0]=0;CarUI_Log(c,400,1);
 assert(strstr(logs,"NAV ST=1 P=5 F=0 SEG=DA LAP_DONE=0 ELAPSED_MS=31000 S=176"));
 assert(strstr(logs,"DPI ") && strstr(logs,"M=TASK4 TL=168 TR=72"));
 /* Parked snapshots must survive a later move; UART mocks enforce the real
    160-byte maximum, unlike the previous unbounded mock implementation. */
 c->runtime.state=LINE_LOST;t->phase=T3_FAULT;t->fault=T3_LOST;
 t->loss_duration_ms=500;t->loss_distance_mm=16;t->loss_turn_deg=-5;t->loss_limits=T3_LOSS_TIME;
 c->route.task3_trace.lost_seen=1;c->route.task3_trace.lost_yaw=179;
 memset(c->route.task3_trace.lost_raw,1,4);memset(c->route.task3_trace.valid_raw,1,4);
 c->route.task3_trace.valid_raw[0]=0;
 c->route.task3_trace.targets[0]=168;c->route.task3_trace.targets[1]=72;
 c->route.task3_trace.velocity[0]=141;c->route.task3_trace.velocity[1]=57;
 c->route.vertex_trace.seen=1;c->route.vertex_trace.point=T3_POINT_D;c->route.vertex_trace.lap=1;
 c->route.vertex_trace.distance_mm=1278;c->route.vertex_trace.yaw=-142;
 c->route.vertex_trace.gyro_yaw=-145;c->route.vertex_trace.bias_rate=-.16f;
 memset(c->route.vertex_trace.raw,1,4);c->route.vertex_trace.raw[3]=0;
 c->heading.angle=-30;
 for(unsigned now=800;now<=2000;now+=400)CarUI_Log(c,now,0);
 assert(strstr(logs,"NAV ST=16 P=8 F=7 SEG=DA"));
 assert(strstr(logs,"LOSS MS=500 D100=1600 A100=-500 LIMIT=1 RAW=1111 PRE=0111 Y100=17900 TL=168 TR=72 VL=141 VR=57"));
 assert(strstr(logs,"ENTRY PT=D LAP=1 S100=127800 RAW=1110 Y100=-14200 G100=-14500 B100=-16"));
 t->fault=T3_EDGE;c->runtime.state=LINE_BAD;c->route.edge_trace.seen=1;
 c->route.edge_trace.phase=T3_VERTEX_SEEK;c->route.edge_trace.duration_ms=3000;
 c->route.edge_trace.travel_mm=42;c->route.edge_trace.limits=T3_LOSS_TIME;
 memset(c->route.edge_trace.raw,1,4);c->route.edge_trace.yaw=-174;
 CarUI_Log(c,2800,0);
 assert(strstr(logs,"EDGE AT=12 MS=3000 D100=4200 LIMIT=1 RAW=1111 Y100=-17400"));
 /* Pressure remains a non-blocking drop, and cannot change a latched fault. */
 full_tx=1;CarUI_Log(c,3200,0);assert(c->runtime.state==LINE_BAD && t->fault==T3_EDGE);
}

static void bd_distance_regression(void)
{
 for(unsigned task=3;task<=4;task++)for(unsigned leg=0;leg<=2;leg+=2) {
  CarTask3Config cfg=task==4?CarTask4_Config():CarRoute_Task3Config();
  assert(cfg.bd_full_distance==0);
  cfg.start_from_ab=0;
  CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.now_ms=1000,.heading_ready=1,.enabled=1};
  assert(CarTask3_Start(&t,&cfg,&in));t.leg=leg;t.pending_start_signal=0;
  in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+(leg==2?180+2*t.diagonal_angle_deg:0));
  /* Remaining on the A/B start line cannot pre-arm or trigger C/D. */
  in.distance_mm=25;in.line_valid=1;in.black_count=1;in.outer_probe_hit=1;
  unsigned signals_seen=0;
  for(unsigned k=0;k<8;k++) {in.now_ms+=10;CarTask3_Update(&t,&in,&out);signals_seen+=out.signal_point!=T3_NO_POINT;}
  assert(t.phase==T3_DIAGONAL && !t.diagonal_armed && !t.edge_seen && signals_seen==0);
  assert(out.control==T3_HOLD_HEADING && out.base_mmps>0 && !out.reset_controllers);
  /* Stable all-white departure arms the next-line detector. */
  in.line_valid=0;in.black_count=0;in.outer_probe_hit=0;
  for(unsigned k=0;k<4;k++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);}
  assert(t.diagonal_armed && t.phase==T3_DIAGONAL);
  /* 远离理论终点时，只有 2/3 中间探头的图样不能提前换段。 */
  in.distance_mm=350;in.line_valid=1;in.black_count=2;in.centered=1;in.outer_probe_hit=0;
  for(unsigned k=0;k<4;k++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);}
  assert(t.phase==T3_DIAGONAL && !t.edge_seen && out.control==T3_HOLD_HEADING);
  /* The line may occur well before the nominal diagonal distance. */
  in.distance_mm=400;in.line_valid=1;in.black_count=1;in.centered=0;in.outer_probe_hit=1;
  for(unsigned k=0;k<4;k++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);signals_seen+=out.signal_point!=T3_NO_POINT;}
  assert(t.phase==T3_ROLL_TURN && signals_seen==1);
 }
 /* Distance remains only a runaway guard; nominal distance itself is not a transition. */
 for(unsigned limit=0;limit<2;limit++) {
  CarTask3Config cfg=CarTask4_Config();cfg.start_from_ab=0;
  CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.now_ms=1000,.heading_ready=1,.enabled=1};
  assert(CarTask3_Start(&t,&cfg,&in));t.leg=2;t.pending_start_signal=0;
  in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+180+2*t.diagonal_angle_deg);
  in.black_count=0;in.distance_mm=t.diagonal_mm;
  for(unsigned n=0;n<4;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);}
  assert(t.phase==T3_DIAGONAL && t.diagonal_armed && out.control==T3_HOLD_HEADING);
  if(limit==0)in.distance_mm=t.diagonal_mm+cfg.entry_window_mm+1;
  else in.yaw_deg+=cfg.diagonal_tolerance_deg+1;
  in.now_ms+=10;CarTask3_Update(&t,&in,&out);
  assert(t.phase==T3_FAULT && out.control==T3_HALT && out.signal_point==T3_NO_POINT);
 }
 CarTask3Config cfg=CarTask4_Config();cfg.bd_full_distance=2;assert(!CarTask3_ConfigValid(&cfg));
 puts("PASS: AC/BD handoff requires stable departure then stable line, independent of nominal distance; runaway distance and heading guards retained.");
}

static void bd_probe_window_regression(void)
{
 /* Any valid C/D line position is accepted after departure. A short glitch
    cannot switch phases and the starting black line can never arm itself. */
 for(unsigned count=1;count<=3;count++) {
  CarTask3Config cfg=CarTask4_Config();cfg.start_from_ab=0;
  assert(!cfg.bd_full_distance && cfg.b_full_align && cfg.b_align_tolerance_deg==3);
  CarTask3 t={0};CarTask3Output out;
  CarTask3Input in={.now_ms=1000,.heading_ready=1,.enabled=1};
  assert(CarTask3_Start(&t,&cfg,&in));t.leg=2;t.pending_start_signal=0;
  in.yaw_deg=CarMath_WrapDegrees(t.start_heading_deg+180+2*t.diagonal_angle_deg);
  in.distance_mm=20;
  in.line_valid=1;in.black_count=count;in.outer_probe_hit=1;
  for(unsigned n=0;n<5;n++) {
   in.now_ms+=10;CarTask3_Update(&t,&in,&out);
   assert(t.phase==T3_DIAGONAL && !t.diagonal_armed && !t.edge_seen && out.control==T3_HOLD_HEADING && out.signal_point==T3_NO_POINT);
  }
  in.line_valid=0;in.black_count=0;in.outer_probe_hit=0;
  for(unsigned n=0;n<4;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);}
  assert(t.diagonal_armed && !t.edge_seen);
  /* Two valid frames followed by white are a rejected glitch. */
  in.distance_mm=300;in.line_valid=1;in.black_count=count;in.outer_probe_hit=1;
  for(unsigned n=0;n<2;n++){in.now_ms+=10;CarTask3_Update(&t,&in,&out);assert(t.edge_seen && out.control==T3_HOLD_HEADING && out.base_mmps==cfg.capture_mmps);}
  in.line_valid=0;in.black_count=0;in.outer_probe_hit=0;in.now_ms+=10;CarTask3_Update(&t,&in,&out);
  assert(t.phase==T3_DIAGONAL && !t.edge_seen && out.control==T3_HOLD_HEADING);
  in.distance_mm=count==3?t.diagonal_mm+cfg.entry_window_mm+50:400;
  in.line_valid=1;in.black_count=count;in.outer_probe_hit=1;
  for(unsigned n=0;n<3;n++) {
   in.now_ms+=10;CarTask3_Update(&t,&in,&out);
   assert(t.phase==T3_DIAGONAL && out.control==T3_HOLD_HEADING && !out.reset_controllers && out.base_mmps==cfg.capture_mmps && out.signal_point==T3_NO_POINT);
  }
  in.now_ms+=10;CarTask3_Update(&t,&in,&out);
  assert(t.phase==T3_ROLL_TURN && out.signal_point==T3_POINT_D);
  if(count<3)assert(in.distance_mm<t.diagonal_mm-cfg.entry_window_mm);
  in.now_ms+=10;CarTask3_Update(&t,&in,&out);assert(out.signal_point==T3_NO_POINT);
 }
 /* Actual foreground scheduler, GPIO decoding, stop output and D snapshot. */
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 for(unsigned n=0;n<4500 && !(app()->route.task3.leg==2 && app()->route.task3.phase==T3_DIAGONAL);n++) {
  assert(app()->runtime.state==RUNNING);step_route();
 }
 assert(app()->route.task3.leg==2 && app()->route.task3.phase==T3_DIAGONAL);
 CarTask3 *t=&app()->route.task3;
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=t->leg_start_mm+400;
 app()->heading.angle=CarMath_WrapDegrees(t->start_heading_deg+180+2*t->diagonal_angle_deg);
 line_inputs=15;
 for(unsigned n=0;n<4;n++)step(10,0,0);
 assert(t->diagonal_armed && app()->route.targets[0]>0 && app()->route.targets[1]>0);
 int before=signals;line_inputs=14;
 for(unsigned n=0;n<3;n++) {
  step(10,0,0);assert(a>0 && b<0 && signals==before && t->phase==T3_DIAGONAL);
 }
 step(10,0,0);
 assert(t->phase==T3_ROLL_TURN && signals==before+1 && a==0 && b==0);
 assert(app()->route.vertex_trace.seen && app()->route.vertex_trace.point==T3_POINT_D);
 assert(fabsf(app()->route.vertex_trace.distance_mm-400)<.1f);
 puts("PASS: event-driven BD rejects its starting line and short glitches, accepts stable 1/2/3-probe endpoints at arbitrary distance, and actual GPIO scheduler halts with a D snapshot.");
}

int main(void)
{
 kernel_regression();integration_regression();vertex_diagnostics_regression();edge_diagnostics_regression();diagnostic_logging_regression();bd_distance_regression();bd_probe_window_regression();
 puts("PASS: inactive retained fusion kernel, static-CAL pure-gyro control in Task3/4 phases, no learning or stale negative-bias correction, no injected-bias drift at rest, wrap, calibration reset, explicit EN=0 diagnostics, unchanged Task1/2 and vertex/fault snapshots. Software only.");
 puts("PASS: production 160-byte UART frame cap, Task4 DPI/NAV delivery, parked F7 LOSS/ENTRY and F6 EDGE snapshots, moved-vehicle retention and full-TX non-blocking fault preservation.");
 return 0;
}
