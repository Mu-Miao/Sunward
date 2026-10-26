#define CAR_TASK3_TEST_MAIN ignored_task3_main
#include "test_task3.c"
#undef CAR_TASK3_TEST_MAIN
static void dump_config(void)
{
 CarTask3Config c=CarTask4_Config();
 printf("CFG4 span_mm=%a\n",(double)c.span_mm);
 printf("CFG4 radius_mm=%a\n",(double)c.radius_mm);
 printf("CFG4 arc_mm=%a\n",(double)c.arc_mm);
 printf("CFG4 start_from_ab=%lu\n",(unsigned long)c.start_from_ab);
 printf("CFG4 bd_full_distance=%lu\n",(unsigned long)c.bd_full_distance);
 printf("CFG4 b_full_align=%lu\n",(unsigned long)c.b_full_align);
 printf("CFG4 b_align_tolerance_deg=%a\n",(double)c.b_align_tolerance_deg);
 printf("CFG4 bd_turn_relief_deg=%a\n",(double)c.bd_turn_relief_deg);
 printf("CFG4 d_full_align=%lu\n",(unsigned long)c.d_full_align);
 printf("CFG4 d_align_tolerance_deg=%a\n",(double)c.d_align_tolerance_deg);
 printf("CFG4 diagonal_mmps=%a\n",(double)c.diagonal_mmps);
 printf("CFG4 arc_mmps=%a\n",(double)c.arc_mmps);
 printf("CFG4 start_mmps=%a\n",(double)c.start_mmps);
 printf("CFG4 acceleration_mmps2=%a\n",(double)c.acceleration_mmps2);
 printf("CFG4 arc_acceleration_mmps2=%a\n",(double)c.arc_acceleration_mmps2);
 printf("CFG4 turn_mmps=%a\n",(double)c.turn_mmps);
 printf("CFG4 capture_mmps=%a\n",(double)c.capture_mmps);
 printf("CFG4 entry_window_mm=%a\n",(double)c.entry_window_mm);
 printf("CFG4 exit_window_mm=%a\n",(double)c.exit_window_mm);
 printf("CFG4 turn_travel_limit_mm=%a\n",(double)c.turn_travel_limit_mm);
 printf("CFG4 capture_limit_mm=%a\n",(double)c.capture_limit_mm);
 printf("CFG4 approach_mm=%a\n",(double)c.approach_mm);
 printf("CFG4 minimum_mmps=%a\n",(double)c.minimum_mmps);
 printf("CFG4 diagonal_tolerance_deg=%a\n",(double)c.diagonal_tolerance_deg);
 printf("CFG4 arc_tolerance_deg=%a\n",(double)c.arc_tolerance_deg);
 printf("CFG4 align_tolerance_deg=%a\n",(double)c.align_tolerance_deg);
 printf("CFG4 deadline_ms=%lu\n",(unsigned long)c.deadline_ms);
 printf("CFG4 settle_ms=%lu\n",(unsigned long)c.settle_ms);
 printf("CFG4 brake_timeout_ms=%lu\n",(unsigned long)c.brake_timeout_ms);
 printf("CFG4 align_timeout_ms=%lu\n",(unsigned long)c.align_timeout_ms);
 printf("CFG4 capture_timeout_ms=%lu\n",(unsigned long)c.capture_timeout_ms);
 printf("CFG4 debounce_ms=%lu\n",(unsigned long)c.debounce_ms);
 printf("CFG4 lost_ms=%lu\n",(unsigned long)c.lost_ms);
 printf("CFG4 sample_max_age_ms=%lu\n",(unsigned long)c.sample_max_age_ms);
 printf("CFG4 white_seek_ms=%lu\n",(unsigned long)c.white_seek_ms);
 printf("CFG4 vertex_seek_ms=%lu\n",(unsigned long)c.vertex_seek_ms);
 printf("CFG4 white_seek_mm=%a\n",(double)c.white_seek_mm);
 printf("CFG4 white_seek_deg=%a\n",(double)c.white_seek_deg);
 printf("CFG4 seek_mmps=%a\n",(double)c.seek_mmps);
 printf("CFG4 vertex_seek_mm=%a\n",(double)c.vertex_seek_mm);
 printf("CFG4 da_outer_gap=%lu\n",(unsigned long)c.da_outer_gap);
 printf("CFG4 history_gap=%lu\n",(unsigned long)c.history_gap);
 printf("CFG4 da_gap_mm=%a\n",(double)c.da_gap_mm);
 printf("CFG4 da_gap_deg=%a\n",(double)c.da_gap_deg);
 printf("CFG4 recovery_ms=%lu\n",(unsigned long)c.recovery_ms);
 printf("CFG4 arc_entry_ms=%lu\n",(unsigned long)c.arc_entry_ms);
 printf("CFG4 arc_entry_min_ms=%lu\n",(unsigned long)c.arc_entry_min_ms);
 printf("CFG4 arc_entry_confirm_ms=%lu\n",(unsigned long)c.arc_entry_confirm_ms);
 printf("CFG4 arc_entry_mm=%a\n",(double)c.arc_entry_mm);
 printf("CFG4 arc_entry_deg=%a\n",(double)c.arc_entry_deg);
 printf("CFG4 arc_entry_mmps=%a\n",(double)c.arc_entry_mmps);
 printf("CFG4 arc_entry_diff_mmps=%a\n",(double)c.arc_entry_diff_mmps);
 printf("CFG4 arc_entry_line_gain=%a\n",(double)c.arc_entry_line_gain);
 printf("CFG4 rolling_entry=%lu\n",(unsigned long)c.rolling_entry);
 printf("CFG4 continuous_motion=%lu\n",(unsigned long)c.continuous_motion);
 printf("CFG4 turn_inner_mmps=%a\n",(double)c.turn_inner_mmps);
 printf("CFG4 edge_follow=%lu\n",(unsigned long)c.edge_follow);
 printf("CFG4 edge_follow_mmps=%a\n",(double)c.edge_follow_mmps);
 printf("CFG4 edge_follow_diff_mmps=%a\n",(double)c.edge_follow_diff_mmps);
 printf("CFG4 edge_follow_mm=%a\n",(double)c.edge_follow_mm);
 printf("CFG4 edge_follow_deg=%a\n",(double)c.edge_follow_deg);
 printf("CFG4 edge_follow_ms=%lu\n",(unsigned long)c.edge_follow_ms);
}
int main(void) {
 dump_config();
 task_start();trace_hash=UINT64_C(14695981039346656037);trace_events=0;
 for(unsigned leg=0;leg<4;leg++)endpoint(leg);
 for(unsigned n=0;n<100 && app()->runtime.state==BRAKING;n++)step(10,0,0);
 assert(app()->runtime.state==DONE);
 printf("TASK2_RUN hash=%016llx events=%lu elapsed=%lu signals=%u\n",(unsigned long long)trace_hash,(unsigned long)trace_events,(unsigned long)app()->route.elapsed_ms,signals);
 setup();line_inputs=15;hold();hold();hold();hold();calibrate();enable();press_key();
 trace_hash=UINT64_C(14695981039346656037);trace_events=0;
 for(unsigned k=0;k<14000 && app()->runtime.state==RUNNING;k++)step_route();
 assert(app()->runtime.state==DONE && signals==17 && app()->route.task4.completed_laps==4);
 printf("TASK4_RUN hash=%016llx events=%lu elapsed=%lu signals=%u\n",(unsigned long long)trace_hash,(unsigned long)trace_events,(unsigned long)app()->route.elapsed_ms,signals);
 return 0;
}
