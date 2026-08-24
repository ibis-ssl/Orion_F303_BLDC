/**
 * @file foc_diagnostic.h
 * @brief FOC/SinePWM diagnostic mode control.
 */

#ifndef INC_FOC_DIAGNOSTIC_H_
#define INC_FOC_DIAGNOSTIC_H_

#include <stdbool.h>

bool isFocDiagnosticActive(void);
void startFocDiagnosticMode(void);
void stopFocDiagnosticMode(void);
void toggleFocDiagnosticMode(void);
void adjustFocDiagnosticPhaseAdvance(float delta_rad);
void resetFocDiagnosticPhaseAdvance(void);
void printFocDiagnosticAngleState(void);
void focDiagnosticMode(void);
void focDiagnosticProcess_itr(bool motor);

#endif /* INC_FOC_DIAGNOSTIC_H_ */
