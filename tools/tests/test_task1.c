/* Production one-metre route; synthetic encoder/gyro inputs, no hardware. */
#define main task2_regression_main
#include "test_task2.c"
#undef main
static CarAppContext *app(void){return CarApp_TestContext();}
static void start_one_metre(void)
{
 setup();calibrate();enable();press_key();
 assert(app()->runtime.state==RUNNING && app()->route.motion==MOVE_ROUTE);
 assert(app()->route.goal==1000 && app()->route.cruise==270 && app()->route.slow==180);
 assert(app()->route.targets[0]==40 && app()->route.run_duration==13500 && !signals);
}
static void ideal_schedule(void)
{
 start_one_metre();uint32_t start=app()->route.start_ms;float residue[2]={0,0};
 float previous=40;unsigned fast=0,slow=0;
 for(unsigned n=0;n<1350 && app()->runtime.state==RUNNING;n++) {
  float base=app()->route.base;
  assert(base>=40 && base<=270 && base<=40+200*(ms-start)*.001f+.01f);
  if(base==270)fast++;
  if(base<previous){assert(progress()>820);slow++;}
  previous=base;int counts[2];
  for(unsigned w=0;w<2;w++) {
   float target=app()->route.targets[w];assert(target>=0 && target<=285);
   residue[w]+=target*.01f*(w?DPT_RIGHT_CPR:DPT_LEFT_CPR)/(DPT_CIRC_M*1000);
   counts[w]=(int)residue[w];residue[w]-=counts[w];
  }
  step(10,counts[0],counts[1]);assert(!signals && a>=0 && b<=0);
 }
 assert(app()->runtime.state==BRAKING && fast && slow && a==0 && b==0);
 assert(progress()>=1000 && progress()<1001);
 for(unsigned n=0;n<5;n++)step(10,30,30);assert(!signals);
 for(unsigned n=0;n<21;n++)step(10,0,0);
 assert(app()->runtime.state==DONE && signals==1 && led_on && beep_on);
 uint32_t elapsed=ms-start;assert(elapsed<6500);
 for(unsigned n=0;n<50;n++)step(10,0,0);
 assert(signals==1 && !led_on && !beep_on && a==0 && b==0);
 assert(strcmp(oled_lines[0],"TASK 1 V3 FAST")==0);
 printf("IDEAL Task1 target-following including coast/settle: %lu ms; simulation only.\n",(unsigned long)elapsed);
}
static void deceleration_integrals(void)
{
 start_one_metre();app()->route.start_ms=ms-3000;
 app()->route.base=270;app()->route.targets[0]=app()->route.targets[1]=270;
 app()->route.distance_mm[0]=app()->route.distance_mm[1]=900;
 for(unsigned w=0;w<2;w++) {
  app()->wheel.items[w].integral=1800;
  app()->wheel.items[w].bad_sign_us=20000;
  app()->wheel.items[w].stall_us=100000;
 }
 uint16_t count[2]={app()->route.distance_count[0],app()->route.distance_count[1]};
 CarRoute_Update(app(),ms,count);assert(app()->route.base==150);
 for(unsigned w=0;w<2;w++) {
  assert(fabsf(app()->wheel.items[w].integral-1000)<.01f);
  assert(app()->wheel.items[w].bad_sign_us==20000 && app()->wheel.items[w].stall_us==100000);
 }
}
static void route_faults(void)
{
 for(unsigned event=0;event<6;event++) {
  start_one_metre();TestState expected=STOPPED;
  switch(event) {
   case 0:step(10,2000,0);expected=FAST;break;
   case 1:for(unsigned n=0;n<5;n++)step(10,-177,176);expected=REVERSE;break;
   case 2:for(unsigned n=0;n<60 && app()->runtime.state==RUNNING;n++)step(10,0,0);expected=STALL;break;
   case 3:app()->route.start_ms=ms-13500;step(10,30,30);expected=TIMEOUT;break;
   case 4:gz=30000;for(unsigned n=0;n<10 && app()->runtime.state==RUNNING;n++)step(10,30,30);expected=YAW_BAD;break;
   case 5:release_key();press_key();break;
  }
  assert(app()->runtime.state==expected && a==0 && b==0 && !signals);
 }
}
int main(void)
{
 ideal_schedule();deceleration_integrals();route_faults();
 puts("PASS: Task1 270mm/s ramp, early slowdown/integral reduction, metre stop/coast/settle/single signal, timeout and wheel protections.");
 return 0;
}
