#pragma once

void automode_init(void);
void automode_task(void *p);
void on_board_callback(void);
void data_reci_task(void *p);
extern float auto_exp_height;
extern int auto_mode, auto_turnoff;
extern int16_t auto_dx , auto_dy;
extern uint8_t heart_beat_c ;
extern int16_t auto_dx_speed, auto_dy_speed;
extern int16_t dx_previous,dy_previous;
extern int flag_moving;
