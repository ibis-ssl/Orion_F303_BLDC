/**
 * @file foc_control.h
 * @brief Shared voltage-FOC output helpers for RUN, diagnostics, and calibration.
 */

#ifndef INC_FOC_CONTROL_H_
#define INC_FOC_CONTROL_H_

#include <stdbool.h>

float focControlRawPositiveElectricalAngle(int raw);
float focControlRawNegativeElectricalAngle(int raw);
float focControlBaseElectricalAngle(bool motor);
float focControlPhaseAdvance(float speed_rps);
float focControlAxisOffset(void);
float focControlTorqueSign(void);
void focControlAdjustPhaseAdvance(float delta_rad);
void focControlResetPhaseAdvance(void);
float focControlGetPhaseAdvanceTrim(void);
void focControlApplyVoltage(bool motor, float output_voltage, float speed_rps, float voltage_limit);
void focControlApplyFixedAngleVoltage(bool motor, float electrical_angle, float output_voltage, float voltage_limit);

#endif /* INC_FOC_CONTROL_H_ */
