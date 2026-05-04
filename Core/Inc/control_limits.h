/**
 * @file control_limits.h
 * @brief Fixed protection thresholds for this STM32F303 BLDC controller.
 */

#ifndef INC_CONTROL_LIMITS_H_
#define INC_CONTROL_LIMITS_H_

// fault/protection thresholds
#define MOTOR_OVER_LOAD_CNT_LIMIT (3000)

#define THR_MOTOR_SHORT_CURRENT (10.00f)
#define THR_MOTOR_OVER_CURRENT (1.00f)
#define THR_MOTOR_LOAD_CURRENT_LIMIT (1.00f)
#define THR_MOTOR_EST_CURRENT_LIMIT (3.00f)
#define THR_BATTERY_UNVER_VOLTAGE (18.0f)
#define THR_BATTERY_OVER_VOLTAGE (35.0f)
#define THR_MOTOR_OVER_TEMPERATURE (70)
#define THR_FET_OVER_TEMPERATURE (80)
#define THR_NO_CONNECTED_TEPERATURE (120)

#endif /* INC_CONTROL_LIMITS_H_ */
