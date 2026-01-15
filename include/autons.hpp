#pragma once

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

void leftQual();
void rightQual();
void leftElim();
void rightElim();
void skills();
void soloAWP();

float get_heading(float current_x, float current_y, float dest_x, float dest_y);
float get_distance(float current_x, float current_y, float dest_x, float dest_y);
void match_load_procedure(int times, double wiggle_amount);