/* User-requested50mm all-white cap; normal and history gap boundary tests. */
#define CAR_TASK3_TEST_MAIN previous_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void begin_white(CarTask3 *t,CarTask3Input *i,CarTask3Output *o,CarTask3Config c,int history,int edge)
{
 memset(t,0,sizeof *t);memset(i,0,sizeof *i);c.edge_follow=(uint8_t)edge;
 i->now_ms=1000;i->distance_mm=1000;i->heading_ready=i->enabled=1;
 i->line_valid=1;i->black_count=history?1:2;i->centered=!history;
 i->outermost=i->side_probe=(uint8_t)history;i->line_error=history?3:0;
 assert(CarTask3_BeginArc(t,&c,i,180,-1));
 i->now_ms+=10;i->line_valid=i->black_count=i->centered=i->outermost=i->side_probe=0;
 CarTask3_Update(t,i,o);
 assert(t->phase==T3_ARC && t->loss_seek && t->gap_seek==history && o->control!=T3_HALT);
}
static void distance_boundary(CarTask3Config c)
{
 for(int history=0;history<=1;history++)for(int edge=0;edge<=1;edge++) {
  CarTask3 t;CarTask3Input i;CarTask3Output o;begin_white(&t,&i,&o,c,history,edge);
  uint32_t origin=t.lost_start_ms;
  const float below[]={20,35,49.9f};
  for(unsigned n=0;n<3;n++) {
   i.now_ms+=10;i.distance_mm=t.lost_start_mm+below[n];CarTask3_Update(&t,&i,&o);
   assert(t.phase==T3_ARC && t.lost_start_ms==origin && o.control!=T3_HALT && o.signal_point==T3_NO_POINT);
  }
  i.now_ms+=10;i.distance_mm=t.lost_start_mm+50;CarTask3_Update(&t,&i,&o);
  assert(t.phase==T3_FAULT && t.fault==T3_LOST && t.loss_limits==T3_LOSS_DISTANCE);
  assert(fabsf(t.loss_distance_mm-50)<.001f && o.control==T3_HALT && o.signal_point==T3_NO_POINT);
 }
}
static void unchanged_time_angle_and_recovery(CarTask3Config c)
{
 for(int history=0;history<=1;history++) {
  CarTask3 t;CarTask3Input i;CarTask3Output o;begin_white(&t,&i,&o,c,history,0);
  uint32_t origin=t.lost_start_ms;
  for(unsigned n=0;n<49;n++){i.now_ms+=10;i.distance_mm+=.5f;CarTask3_Update(&t,&i,&o);assert(t.phase==T3_ARC);}
  i.now_ms+=10;CarTask3_Update(&t,&i,&o);
  assert(t.phase==T3_FAULT && t.loss_limits==T3_LOSS_TIME && t.loss_duration_ms==500 && t.lost_start_ms==origin);
  begin_white(&t,&i,&o,c,history,0);i.now_ms+=10;i.yaw_deg=history?18.1f:12.1f;
  CarTask3_Update(&t,&i,&o);assert(t.phase==T3_FAULT && t.loss_limits==T3_LOSS_ANGLE);
  begin_white(&t,&i,&o,c,history,0);origin=t.lost_start_ms;
  i.distance_mm=t.lost_start_mm+40;
  for(unsigned n=0;n<4;n++) {
   i.now_ms+=10;i.line_valid=1;i.black_count=2;i.centered=1;i.line_error=0;
   CarTask3_Update(&t,&i,&o);
   assert(t.phase==T3_ARC && o.signal_point==T3_NO_POINT);
   if(n<3)assert(t.lost_seen && t.lost_start_ms==origin);
  }
  assert(!t.lost_seen && o.control==T3_FOLLOW_LINE);
 }
}
int main(void)
{
 /* Task2 really receives the Task4 configuration through its adapter. */
 to_arc_entry();CarTask3Config two=app()->route.task3.config;
 CarTask3Config profiles[]={two,CarRoute_Task3Config(),CarTask4_Config()};
 for(unsigned task=0;task<3;task++) {
  CarTask3Config c=profiles[task];
  assert(c.white_seek_mm==50 && c.da_gap_mm==50 && c.white_seek_ms==500);
  assert(c.white_seek_deg==12 && c.da_gap_deg==18 && c.recovery_ms==30);
  distance_boundary(c);unchanged_time_angle_and_recovery(c);
  c.white_seek_mm=c.da_gap_mm=51;assert(!CarTask3_ConfigValid(&c));
  c=profiles[task];c.da_gap_mm=51;assert(!CarTask3_ConfigValid(&c));
 }
 assert(CarRoute_FigureEightBaseConfig().white_seek_mm==20); /* Historical profile only. */
 assert(DPT_TASK2_C_ALIGN_LIMIT_MM==60 && DPT_TASK2_C_ALIGN_TIMEOUT_MS==1500);
 puts("PASS: real Task2/3/4 config white/history50mm;20/35/49.9mm continue and exactly50mm faults; original500ms/12deg/18deg stop first,30ms recovery/no false endpoint, reject>50, historical20 and C-align60 unchanged. Synthetic boundary inputs only.");
 return 0;
}
