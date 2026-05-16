#ifndef INC_DIAGNOSTICS_H_
#define INC_DIAGNOSTICS_H_

#include <stdbool.h>
#include <stdint.h>

void printRuntimeDiagnostics(void);
void runIoCheckOnce(void);
void runFocMathCheckOnce(void);
void setFocMathSelfTestResult(bool ok, uint16_t a, uint16_t b, uint16_t c, bool limited);

#endif /* INC_DIAGNOSTICS_H_ */
