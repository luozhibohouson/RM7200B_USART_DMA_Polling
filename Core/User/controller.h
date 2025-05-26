#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "stdint.h"
#include "hal_conf.h"

// #include "version.h"


/*****************************************************************/
// 功能开关
#define RM_MAGIC_CLEAR        0  // 超声清洗代码开关
#define RM_MAGIC_COOL         1  // 气泵代码开关
#define RM_MAGIC_ENG          0  // 压电马达开关

/*****************************************************************/
// 电源控制
#define POWER_OFF             0  // 关闭12V电源
#define POWER_LOWVOL          1  // 输出电压
#define POWER_HIGHVOL         2  // 输出高电压

/*****************************************************************/
//
#define USE_AIR_FLOWMETER               0  // 是否启用流量计

#define MAGIC_COOL_DIFF                 1  // 是否是差分模式

#define MAGIC_COOL_PID                  0  // 是否启用PID算法

// 电压参数计算方式： 1. 通过最大值，最小值求Vpp，2. 通过极大值，极小值平均求VPP
#define MAGIC_COOL_VPP_MAXMIN           0   // 通过最大值，最小值求Vpp
#define MAGIC_COOL_VPP_AVG              1   // 通过极大值，极小值平均求VPP
#define MAGIC_COOL_VPP_DEFAULT          MAGIC_COOL_VPP_AVG   // 默认通过最大值，最小值求Vpp

// 电压参数是否在ADC转换完后计算 1. 是，2. 否
#define MAGIC_COOL_VPP_RMS              1   // 计算有效值
#define MAGIC_COOL_ADC_CENTER           1   // 是否处理数据中心对称

// 阻抗计算方式： 1. 通过Vpp，Ipp计算，2. 通过有效值计算，3. 其他
#define MAGIC_COOL_IMPEDANCE_VPP        0   // 通过Vpp，Ipp计算
#define MAGIC_COOL_IMPEDANCE_RMS        1   // 通过有效值计算
#define MAGIC_COOL_IMPEDANCE_DEFAULT    2   // 默认通过Vpp，Ipp计算

// 相位检测方式： 1. FFT求相位差，2. 点积求相位差，3. 过零比较器求相位差，4. 其他
#define MAGIC_COOL_PHASE_FFT            0   // FFT求相位差
#define MAGIC_COOL_PHASE_DOT            1   // 点积求相位差
#define MAGIC_COOL_PHASE_CAPTURE        2   // 过零比较器求相位差
#define MAGIC_COOL_PHASE_OTHER          3   // 其他求相位差
#define MAGIC_COOL_PHASE_DEFAULT        MAGIC_COOL_PHASE_OTHER   // 默认求相位差

// 追频方式： 1. 阻抗最小点追频，2. 相位差最小点追频，3. 其他
#define MAGIC_COOL_TRACK_IMPEDANCE      0   // 阻抗最小点追频
#define MAGIC_COOL_TRACK_PHASE          1   // 相位差最小点追频
#define MAGIC_COOL_TRACK_CURRENT        2   // 电流追频？？最小点追频还是最大点追频
#define MAGIC_COOL_TRACK_OTHER          3   // 其他
#define MAGIC_COOL_TRACK_DEFAULT        MAGIC_COOL_TRACK_CURRENT   // 默认阻抗最小点追频

// 检测直流电流，1. 是，2. 否
#define MAGIC_COOL_DC_CURRENT_LOW       1   // 低端电流
#define MAGIC_COOL_DC_CURRENT_HIGH      2   // 高端电流
#define MAGIC_COOL_DC_CURRENT_ALL       3   // 全部
#define MAGIC_COOL_DC_CURRENT_DEFAULT   MAGIC_COOL_DC_CURRENT_LOW   // 除能

// 阻抗追频时，差值小于阈值时不变，大于阈值时改变
#define IMPEDANCE_THRESHOLD             0.01

// 相位追频方式的优缺点
// 1. FFT求相位差：
// 优点：受干扰小，受谐波影响小
// 缺点：计算复杂，速度慢，精度低受FFT个数限制，容易受到噪声干扰
// 2. 点积求相位差：
// 优点：精度高
// 缺点：计算复杂，速度慢，受谐波影响大，只适用于谐波小的场合
// 3. 过零比较器求相位差：
// 优点：精度高
// 缺点：计算简单，速度快，受干扰影响大，精度受到过零比较点影响
// 4. 其他求相位差：
// 优点：
// 缺点：




void key_scan(void);
void set_power_enable(uint32_t enable);
void uart_cmd_process(void);

void set_compare_dcoffset(int len);
void rm_magic_config(void);
void rm_magic_run(void);

#endif  // __CONTROLLER_H__
