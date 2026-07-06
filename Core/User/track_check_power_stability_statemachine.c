// 定义状态枚举
typedef enum {
    TRACK_STABLE = 0,        // 稳定：正常P&O
    TRACK_UNSTABLE,          // 不稳定：计数
    TRACK_HUGE_UNSTABLE,     // 严重不稳定：大步长扫描
    TRACK_RESETTING          // 恢复稳定：重置参数
} track_state_e;

// 状态机当前状态
static track_state_e track_cur_state = TRACK_STABLE;

// 状态机入口（替代原来的 track_check_power_stability）
static bool track_check_power_stability_sm(uint8_t n)
{
    // 1. 采样功率 =========================
    adc_sample_t sample = {0};
    uint32_t pwrx = 0;

    for (int i = 0; i < n; i++) {
        DELAY_MS_OR_RETURN(10, false);
        sample = magic_cool_calc_current(ENABLE);
        pwrx += calculate_power(sample);
    }
    pwrx /= n;

    // 2. 计算差值 =========================
    uint32_t diff = abs_i(magic_cool_pwr_max - pwrx);
    uint8_t percent = diff * 100 / magic_cool_pwr_max;

    // 打印（可选）
    printf("freq:%d power:%d diff:%d %%:%d state:%d\r\n",
           pwm_get_freq(), pwrx, diff, percent, track_cur_state);

    // 3. 状态机 =========================
    switch (track_cur_state)
    {
        // ======================
        // 状态0：稳定
        // ======================
        case TRACK_STABLE:
        {
            // 稳定 → 直接返回
            if (percent < track_state.pwr_proxth) {
                return true;
            }

            // 不稳定 → 进入计数
            track_cur_state = TRACK_UNSTABLE;
            track_state.pwr_diff_cnt = 0;
            return true;
        }

        // ======================
        // 状态1：轻微不稳定（连续计数）
        // ======================
        case TRACK_UNSTABLE:
        {
            // 恢复稳定 → 回去
            if (percent < track_state.pwr_proxth) {
                track_cur_state = TRACK_STABLE;
                track_state.pwr_diff_cnt = 0;
                return true;
            }

            // 严重不稳定 → 进入大扫描
            if (percent >= PWR_PROXTH_MAX) {
                track_cur_state = TRACK_HUGE_UNSTABLE;
                track_state.pwr_high_diff_cnt = 0;
                return true;
            }

            // 连续N次 → 允许P&O
            track_state.pwr_diff_cnt++;
            if (track_state.pwr_diff_cnt >= track_state.perturb_cnt) {
                track_state.pwr_diff_cnt = 0;
                return false; // 进入P&O
            }

            return true;
        }

        // ======================
        // 状态2：严重不稳定 → 大扫描
        // ======================
        case TRACK_HUGE_UNSTABLE:
        {
            // 连续2次 → 触发大扫描
            track_state.pwr_high_diff_cnt++;
            if (track_state.pwr_high_diff_cnt >= 2) {
                // 设置大扫描参数
                track_state.freq_step = FREQ_HIGH_TEMP_STEP;
                track_state.freq_range = FREQ_HIGH_TEMP_RANGE;
                track_state.pwr_proxth = PWR_PROXTH_MIN;
                track_state.perturb_step = 50;
                track_state.perturb_cnt = 0;
                track_state.is_huge_stability_scan = true;
                track_state.normal_pwr_max = magic_cool_pwr_max;

                track_state.pwr_high_diff_cnt = 0;
                printf("→ 进入大扫描模式\r\n");
            }

            // 检查是否恢复正常功率
            bool is_back = (pwrx >= track_state.normal_pwr_max * 97 / 100);
            if (is_back) {
                track_cur_state = TRACK_RESETTING;
                track_state.pwr_proxth_reset_cnt = 0;
            }

            return true;
        }

        // ======================
        // 状态3：恢复稳定 → 重置参数
        // ======================
        case TRACK_RESETTING:
        {
            bool is_back = (pwrx >= track_state.normal_pwr_max * 97 / 100);
            if (!is_back) {
                track_cur_state = TRACK_HUGE_UNSTABLE;
                track_state.pwr_proxth_reset_cnt = 0;
                return true;
            }

            // 连续10次恢复 → 重置为正常参数
            track_state.pwr_proxth_reset_cnt++;
            if (track_state.pwr_proxth_reset_cnt >= 10) {
                track_reset_state();  // 你的重置函数
                track_cur_state = TRACK_STABLE;
                printf("→ 恢复正常模式\r\n");
            }

            return true;
        }

        default:
            track_cur_state = TRACK_STABLE;
            return true;
    }
}