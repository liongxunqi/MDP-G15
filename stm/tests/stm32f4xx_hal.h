#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { int unused; } I2C_HandleTypeDef;
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
#endif
