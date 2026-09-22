#ifndef __IR_H
#define __IR_H

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Sharp GP2Y0A21YK analog IR distance sensors, one each side.
 *
 *   IR_L  PC0 -> ADC1_IN10
 *   IR_R  PC1 -> ADC1_IN11
 *
 * ADC1 runs free: scan mode over both channels, continuous conversion, DMA in
 * circular mode into a two-word buffer. Nothing has to start a conversion or
 * wait for one - the buffer always holds the latest pair and reading it is
 * just a memory access. That matters because these are polled from the 10 ms
 * control tick.
 *
 * Sampling time is 480 cycles at 10.5 MHz ADC clock, about 47 us per channel.
 * Long, deliberately: the Sharp's output is a lightly filtered analog line and
 * the long aperture is free noise rejection.
 *
 * ---------------------------------------------------------------------------
 * THE RESPONSE CURVE IS NOT LINEAR, AND THE LOW END IS NOT USABLE
 *
 * Output voltage rises as an object gets closer. MEASURED on this robot,
 * both sensors mounted, matt black target: counts climb monotonically all the
 * way in to 5 cm. There is no fold-back above that, so the classic "1.4 V
 * means either 25 cm or 4 cm" ambiguity was NOT observed on these units.
 *
 * The low end is unusable anyway, for a different reason - the output
 * SATURATES. At 5 cm the left sensor reads 3.10 V, close to the part's
 * ceiling, and the fit below maps both 5 cm and 8 cm to about 9 cm. It cannot
 * tell them apart, so a reading there is a plausible lie either way.
 *
 * IR_MIN_VALID_CM therefore stays at 10 and still earns its keep. Mount the
 * sensors so nothing can get inside 10 cm of them, and treat a dropout as
 * "too close", never as "clear".
 *
 * Above ~80 cm the output flattens into the noise floor. The RIGHT sensor
 * gets there sooner than the left - see the fit constants below.
 * ------------------------------------------------------------------------- */

/* Usable span, cm. Outside this the reading is reported as no reading. */
#define IR_MIN_VALID_CM         10U
#define IR_MAX_VALID_CM         80U

/* Median filter depth, in samples taken one per control tick. 5 ticks is
 * 50 ms of lag, which is nothing next to how much these sensors twitch. */
#define IR_MEDIAN_N             5U

/* ADC reference and full scale. VREF+ is tied to VDDA on this board. */
#define IR_ADC_VREF             3.3f
#define IR_ADC_FULL_SCALE       4095.0f

/* Potential divider between the sensor output and the ADC pin, if fitted.
 *
 *      IR_DIVIDER_RATIO = V_at_pin / V_at_sensor
 *
 * 1.0 means the sensor output goes straight to PC0/PC1. A 1k/2.2k divider,
 * wired sensor -> 1k -> pin -> 2.2k -> ground, would give 2.2/(1.0+2.2)
 * = 0.6875.
 *
 * *** RESOLVED: 1.0. THERE IS NO DIVIDER ON THE IR LINES. ***
 *
 * The kit issues exactly one 1k and one 2.2k resistor and they belong to the
 * HC-SR04 echo line, so there is nothing left to build an IR divider from.
 * Last year's firmware also read these pins bare, and its 0.80 V obstacle
 * threshold lines up with the fit below at 1.0 rather than at 0.6875.
 *
 * It matters more than it looks. The fit below converts pin volts to
 * centimetres through a power law with an exponent near -1.1, so an unnoticed
 * divider does not shift the readings by 31% - it shifts them by
 * 0.6875^-1.1, about 50%, and it does so smoothly across the whole range.
 * The numbers stay plausible the entire time, which is exactly what makes it
 * hard to spot.
 *
 * A note on which is correct, checked against DS8626 Rev 12:
 *
 * Table 7 lists PC0 and PC1 as "I/O  FT (5)", and footnote 5 reads:
 *
 *     "FT = 5 V tolerant except when in analog mode or oscillator mode"
 *
 * So they ARE 5 V tolerant as digital pins - but the tolerance is explicitly
 * switched off in analog mode, which is exactly the mode the ADC uses them
 * in. In this design they must stay inside VREF+ (tied to VDDA = 3.3 V here
 * by R4/R33 on the C30D schematic).
 *
 * The GP2Y0A21YK is a 5 V part whose output peaks near 3.1 V at close range,
 * so it does fit under 3.3 V unaided - with very little margin, and none at
 * all if VDDA sags or the sensor runs a little hot.
 *
 * Table 47 is the stronger argument for fitting the divider: PC0 and PC1 are
 * on the list of pins where the allowed negative injected current is 0 mA,
 * "NA" for positive. A 5 V sensor driving an unpowered MCU - which is what
 * happens whenever the sensor rail comes up first - injects current through
 * the pin protection. The 1 k series leg limits it; a direct wire does not.
 *
 * PC7, where the ultrasonic echo lands, is FT and stays FT because it is used
 * as a digital AF input, not an analog one.
 *
 * So the injection risk on these two pins is real and currently unmitigated.
 * If it ever needs addressing, fit a SERIES resistor only, with no leg to
 * ground: it limits injected current without dividing the signal, so this
 * ratio stays 1.0 and nothing has to be recalibrated. The 480-cycle sampling
 * time already absorbs the extra source impedance. */
#define IR_DIVIDER_RATIO        1.0f

/* Power-law fit: distance_cm = A * volts ^ B. PER CHANNEL, deliberately.
 *
 * MEASURED on this robot, sensors mounted in their final positions, matt black
 * obstacle as the target, eleven points from 5 to 80 cm. These replace the
 * datasheet-typical pair (27.86, -1.15), which under-read by 5 to 14 percent
 * across the whole range.
 *
 * THE TWO UNITS ARE NOT INTERCHANGEABLE. At the same distance the left reads
 * about 20% more counts than the right, steadily from 15 through 50 cm, so one
 * shared fit cannot serve both - which is why raw_to_cm() takes the constants
 * as arguments rather than reading them from here directly.
 *
 * That gap is the PARTS, not the mounting. Both sensors were checked and sit
 * square at the same height, so do not go looking for a crooked bracket. It
 * follows that any obstacle THRESHOLD has to be per channel as well - a single
 * count value means different distances on the two sides.
 *
 * The right is the weaker unit. It reads 0.248 V at 80 cm, below the 0.30 V
 * floor in raw_to_cm(), so it reports no-reading past roughly 65 cm while the
 * left is good to 80. Do not expect the two to agree at long range.
 *
 * Re-measure these if a sensor is ever remounted or replaced. */
#define IR_L_FIT_A              30.40f
#define IR_L_FIT_B              (-1.099f)
#define IR_R_FIT_A              24.94f
#define IR_R_FIT_B              (-0.979f)

void IR_Init(ADC_HandleTypeDef *hadc);

/* One sample into the median filters. Call once per control tick. Cheap -
 * it only reads the DMA buffer, no conversion is started or waited on. */
void IR_Update(void);

/* Filtered distance in cm, or SENSOR_NO_READING when out of range. */
uint16_t IR_LeftCm(void);
uint16_t IR_RightCm(void);

/* Latest single ADC sample, for bring-up. If these do not move when you wave
 * your hand at the sensor, the problem is wiring or the ADC, not the curve
 * fit. Too noisy to calibrate against - use IR_LeftFiltered() for that. */
uint16_t IR_LeftRaw(void);
uint16_t IR_RightRaw(void);

/* Median-filtered counts, same filter the cm conversion runs on. This is the
 * number to write down when building the lookup table or picking an obstacle
 * threshold. Reads 0 until the filter has filled, IR_MEDIAN_N ticks in. */
uint16_t IR_LeftFiltered(void);
uint16_t IR_RightFiltered(void);

#endif /* __IR_H */
