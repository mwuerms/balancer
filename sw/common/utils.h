/*
 * utils.h
 *
 *  Created on: Feb 26, 2026
 *      Author: m.egli
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#ifndef BITV
#define BITV(x) (1 << x)
#else
#error BITV defined, check for compatibility
#endif

/* template to use later on
uint32_t primask;

primask = __get_PRIMASK();  // Save current interrupt state
__disable_irq();            // Disable all interrupts

// --- Critical Section Start ---
/ * Your critical code here * /
// --- Critical Section End ---

__set_PRIMASK(primask);     // Restore previous interrupt state
*/



#endif /* INC_UTILS_H_ */
