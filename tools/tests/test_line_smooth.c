/* Normal-line setpoint continuity; full scheduler protections remain in other suites. */
#define CAR_TASK3_TEST_MAIN previous_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
#include "fixtures/v26_normal_line.h"
static void normal_setup(Motion mode)
{
 setup();app()->runtime.state=RUNNING;app()->runtime.dt_us=10000;
 app()->route.motion=mode;app()->route.targets[0]=app()->route.targets[1]=300;
 app()->route.base=300;app()->route.task3.phase=T3_ARC;
}
static void line_tick(unsigned mask,float base,unsigned dt)
{
 ms+=dt;line_inputs=mask;CarLine_Decode(app());CarLine_SetTargets(app(),ms,base);
}
static void steps_and_derivative(void)
{
 const Motion modes[]={MOVE_TASK2,MOVE_TASK3,MOVE_TASK4};
 for(unsigned mode=0;mode<3;mode++) {
  normal_setup(modes[mode]);line_tick(9,300,10);
  float last=app()->line.diff;
  line_tick(13,300,10);
  assert(app()->line.smooth.d_term>0 && app()->line.smooth.d_term<=18.001f);
  assert(app()->line.diff>last && app()->line.diff-last<=16.001f);
  for(unsigned k=0;k<80;k++) {
   last=app()->line.diff;line_tick(13,300,10);
   assert(fabsf(app()->line.diff-last)<=16.001f);
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
  }
  assert(app()->line.diff>70 && app()->line.diff<74);
  assert(fabsf(app()->line.smooth.d_term)<.001f);
  line_tick(9,300,10);assert(app()->line.smooth.d_term<0);
  for(unsigned k=0;k<80;k++)line_tick(9,300,10);
  assert(fabsf(app()->line.diff)<.01f);
  /* Duplicate same-time call must not advance the filter or acceleration. */
  float left=app()->route.targets[0],right=app()->route.targets[1];
  CarLine_SetTargets(app(),ms,300);
  assert(left==app()->route.targets[0] && right==app()->route.targets[1]);
  CarLine_SetTargets(app(),ms,50);assert(app()->route.base==50 && fabsf(app()->line.diff)<=20);
  /* D is limited independently on a sign reversal, including timer wrap. */
  ms=UINT32_MAX-20;CarLine_ResetControl(app());line_tick(13,300,10);
  for(unsigned k=0;k<6;k++) {
   last=app()->line.diff;line_tick(k&1?11:13,300,10);
   assert(fabsf(app()->line.smooth.d_term)<=18.001f);
   assert(fabsf(app()->line.diff-last)<=16.001f);
  }
 }
}
static void transition_and_safety(void)
{
 normal_setup(MOVE_TASK4);line_tick(13,300,10);
 /* Outermost correction retains the previous immediate 120mm/s safety cap. */
 line_tick(7,300,10);
 assert(app()->route.base==120 && !app()->line.smooth.active);
 assert(app()->route.targets[0]==72 && app()->route.targets[1]==168);
 float old_base=120;
 for(unsigned k=0;k<20;k++) {
  line_tick(9,300,10);assert(app()->route.base<=old_base+2.001f);
  assert(app()->route.base>=old_base);old_base=app()->route.base;
 }
 assert(app()->route.base<=160.001f && app()->line.smooth.d_term==0);
 line_tick(13,50,10);assert(app()->route.base==50);
 assert(app()->route.targets[0]<=70.001f && app()->route.targets[1]>=29.999f);
 /* Any loss takes the original zero-drive path, not the smoother. */
 line_tick(15,300,10);
 assert(a==0 && b==0 && !app()->line.smooth.active);
 assert(app()->route.targets[0]==0 && app()->route.targets[1]==0);
 for(unsigned k=0;k<10;k++)line_tick(15,300,10);
 assert(app()->runtime.state==LINE_LOST);
 normal_setup(MOVE_TASK4);line_tick(13,300,10);
 CarTask3Output o={.control=T3_TURN_FORWARD,.left_mmps=100,.right_mmps=20,.heading_deg=-90};
 app()->heading.angle=0;CarTask3Port_Apply(app(),&o,ms+10);
 assert(!app()->line.smooth.active && app()->route.targets[0]==100 && app()->route.targets[1]==20);
 o=(CarTask3Output){.control=T3_FOLLOW_LINE,.base_mmps=70};
 ms+=20;line_inputs=13;CarLine_Decode(app());CarTask3Port_Apply(app(),&o,ms);
 assert(app()->line.smooth.d_term==0 && app()->route.base<=62.001f);
 CarApp_Stop(app(),STOPPED,'-');CarTask3Port_Apply(app(),&o,ms+10);
 assert(a==0 && b==0 && app()->route.targets[0]==0 && app()->route.targets[1]==0);
 assert(!app()->line.smooth.active);
 /* The independent legacy semicircle retains its original P output. */
 normal_setup(MOVE_LINE);line_tick(13,200,10);
 assert(app()->route.targets[0]==264 && app()->route.targets[1]==136);
 assert(!app()->line.smooth.active);
}
static void recorded_normal_patterns(void)
{
 normal_setup(MOVE_TASK4);line_tick(9,300,10);
 float old_previous=0,new_previous=0,old_variation=0,new_variation=0,max_step=0;
 for(unsigned row=0;row<sizeof v26_normal_masks/sizeof *v26_normal_masks;row++) {
  for(unsigned n=0;n<10;n++) {
   line_tick(v26_normal_masks[row],300,10);
   float old=96*app()->line.error,current=app()->line.diff;
   old_variation+=fabsf(old-old_previous);new_variation+=fabsf(current-new_previous);
   max_step=fmaxf(max_step,fabsf(current-new_previous));old_previous=old;new_previous=current;
   assert(app()->route.targets[0]>0 && app()->route.targets[1]>0);
  }
 }
 printf("V26 sensor-pattern replay at equal300: differential variation %.1f -> %.1f; max10ms differential change %.1f mm/s. Not a field test.\n",old_variation,new_variation,max_step);
 assert(old_variation>200 && new_variation<old_variation*.90f && max_step<=16.001f);
}
int main(void)
{
 assert(CarLine_ConfigValid());steps_and_derivative();transition_and_safety();recorded_normal_patterns();
 puts("PASS: shared normal PD transient/steady state, bounded D and differential, wrap/duplicate call, EDGE immediate cap, rising base/deceleration, loss/stop priority, mode reset and legacy P isolation.");
 return 0;
}
