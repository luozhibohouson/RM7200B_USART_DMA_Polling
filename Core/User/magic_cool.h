#ifndef __MAGIC_COOL_H__
#define __MAGIC_COOL_H__

#include "stdint.h"

void magic_cool_key_scan(uint8_t key1, uint8_t key2, uint8_t key3);
void magic_cool_set_adcfreq(void);
void magic_cool_set_limt(uint32_t freq_min, uint32_t freq_max);
void magic_cool_config(void);
// int magic_cool_voltage_closeloop(uint32_t vol_target, uint32_t vol_err, uint32_t timeout);
int magic_cool_voltage_closeloop_dcdc(uint32_t vol_target, uint32_t vol_err, uint32_t timeout);
int magic_cool_calc_impedance(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq);
void magic_cool_run_impedance(void);
void magic_cool_freq_track_phase(void);
void magic_cool_freq_track_current(void);
void magic_cool_freq_track(void);
void magic_cool_run(void);
void magic_cool_set_target_vol(uint32_t vol);
void magic_cool_config_freq(uint32_t opt);
void magic_cool_set_mode(uint32_t mode);

#endif  // __MAGIC_COOL_H__
