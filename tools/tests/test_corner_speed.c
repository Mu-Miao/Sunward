/* Real common core/port with an ideal differential-drive turning model. */
#define CAR_TASK3_TEST_MAIN previous_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static float target_for(const CarTask3 *t,unsigned leg)
{
 return CarMath_WrapDegrees(t->start_heading_deg+(leg==0?0:
  (leg==2?180+2*t->diagonal_angle_deg-t->config.bd_turn_relief_deg:t->diagonal_angle_deg+(leg==3?180:0))));
}
/* V27's 100/20, 15-degree taper and35mm/s minimum differential;
   ideal wheels, same160mm track and3-degree final tolerance. */
static unsigned v27_reference_turn(void)
{
 float error=38.66f;unsigned n=0;
 while(error>3 && n++<500) {
  float outer=fminf(100,fmaxf(55,100*fminf(1,error/15)));
  error-=(outer-20)*.01f/160*57.2957795131f;
 }
 return n*10+40;
}
static void all_point_turns(void)
{
 unsigned old=v27_reference_turn();
 for(unsigned motion=MOVE_TASK3;motion<=MOVE_TASK4;motion++)for(unsigned leg=0;leg<4;leg++)for(int sign=-1;sign<=1;sign+=2) {
  setup();app()->runtime.state=RUNNING;app()->route.motion=(Motion)motion;
  CarTask3Input in={.now_ms=1000,.enabled=1,.heading_ready=1};
  CarTask3Config c=motion==MOVE_TASK3?CarRoute_Task3Config():CarTask4_Config();
  CarTask3 *t=&app()->route.task3;assert(CarTask3_Start(t,&c,&in));
  t->leg=leg;t->phase=leg?T3_ALIGN:T3_START_ALIGN;t->pending_start_signal=0;
  float goal=target_for(t,leg);in.yaw_deg=CarMath_WrapDegrees(goal-sign*38.66f);
  unsigned n;
  for(n=0;n<400 && t->phase==(leg?T3_ALIGN:T3_START_ALIGN);n++) {
   in.now_ms+=10;app()->heading.angle=in.yaw_deg;CarTask3Output o;
   CarTask3_Update(t,&in,&o);assert(t->phase!=T3_FAULT && o.control!=T3_HALT);
   CarTask3Port_Apply(app(),&o,in.now_ms);
   float left=app()->route.targets[0],right=app()->route.targets[1];
   assert(left>0 && right>0 && left<=120.001f && right<=120.001f);
   in.yaw_deg=CarMath_WrapDegrees(in.yaw_deg+(right-left)*.01f/160*57.2957795131f);
   in.distance_mm+=(left+right)*.005f;in.left_mmps=left;in.right_mmps=right;
  }
  assert(t->phase==(leg&1?T3_CAPTURE:T3_DIAGONAL));
  assert(fabsf(CarMath_WrapDegrees(goal-in.yaw_deg))<=3.001f);
  assert(n*10<old && in.distance_mm<120);
  if(motion==MOVE_TASK3 && leg==0 && sign==1)
   printf("IDEAL38.66deg point turn: V27=%ums V28=%ums; same160mm track/3deg tolerance, not field timing.\n",old,n*10);
 }
}
static void task2_valid_capture_vs_loss(void)
{
 for(unsigned leg=1;leg<=3;leg+=2)for(int sign=-1;sign<=1;sign+=2) {
  setup();app()->runtime.state=RUNNING;app()->route.motion=MOVE_TASK2;app()->route.leg=leg;
  CarTask3Input in={.now_ms=1000,.distance_mm=1000,.enabled=1,.heading_ready=1,
    .line_valid=1,.black_count=1,.outermost=1,.side_probe=1,.line_error=3*sign};
  CarTask3Config c=CarTask4_Config();CarTask3 *t=&app()->route.task3;
  assert(CarTask3_BeginArc(t,&c,&in,180,-1));
  app()->line.valid=1;app()->line.count=1;
  CarTask3Output o;CarTask3_Update(t,&in,&o);CarTask3Port_Apply(app(),&o,in.now_ms);
  assert(app()->route.targets[sign>0?0:1]==140 && app()->route.targets[sign>0?1:0]==20);
  in.now_ms+=10;in.line_valid=0;in.black_count=0;in.outermost=in.side_probe=0;
  app()->line.valid=0;app()->line.count=0;
  CarTask3_Update(t,&in,&o);CarTask3Port_Apply(app(),&o,in.now_ms);
  assert(t->loss_seek && app()->route.targets[sign>0?0:1]==80 && app()->route.targets[sign>0?1:0]==20);
  uint32_t deadline=in.now_ms;
  for(unsigned n=0;n<50;n++){in.now_ms+=10;CarTask3_Update(t,&in,&o);CarTask3Port_Apply(app(),&o,in.now_ms);}
  assert(t->phase==T3_FAULT && t->lost_start_ms==deadline && t->loss_duration_ms==500);
  assert(app()->route.targets[0]==0 && app()->route.targets[1]==0 && a==0 && b==0);
 }
}
int main(void)
{
 CarTask3Config c=CarTask4_Config();assert(CarTask3_ConfigValid(&c));
 c.turn_mmps=121;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.edge_follow_mmps=81;assert(!CarTask3_ConfigValid(&c));
 c=CarTask4_Config();c.edge_follow_diff_mmps=61;assert(!CarTask3_ConfigValid(&c));
 assert(CarRoute_FigureEightBaseConfig().turn_mmps==80);
 assert(DPT_LINE_SMOOTH_KP==48 && DPT_LINE_SMOOTH_KD==.40f && DPT_LINE_SMOOTH_DIFF_RATE==1600);
 all_point_turns();task2_valid_capture_vs_loss();
 puts("PASS: both directions at A/B/C/D settle within original3deg/120mm; faster valid Task2 capture, original white speed/time retained; profile limits and V27 normal-PD isolation.");
 return 0;
}
