/*
 * dsp_lib.h - miscellaneous DSP stuff for ESP32S2 Audio
 * 03-13-22 E. Brombaugh
 */

#ifndef __dsp_lib__
#define __dsp_lib__

#include <stdint.h>

uint8_t dsp_gethyst(int16_t *oldval, int16_t newval);
uint8_t dsp_ratio_hyst_arb(uint16_t *old, uint16_t in, uint8_t range);


/*
 * signed saturation to 16-bit
 */
#if 0
/* as regular code */
inline int16_t dsp_ssat16(int32_t in)
{
	in = in > 32767 ? 32767 : in;
	in = in < -32768 ? -32768 : in;
	return in;
}
#else
/* as inline assembly - faster? */
inline int16_t dsp_ssat16(int32_t in)
{
	int32_t out;
	asm("clamps %0, %1, 15" : "=ar" (out) : "as" (in));
	return out;
}
#endif

/**
  \brief   Signed Saturate
  \details Saturates a signed value.
  \param [in]  ARG1  Value to be saturated
  \param [in]  ARG2  Bit position to saturate to (8..23) minus 1
  \return             Saturated value
 */
#define __SSAT(ARG1,ARG2) \
__extension__ \
({                          \
  int32_t __RES, __ARG1 = (ARG1); \
  asm("clamps %0, %1, %2" : "=ar" (__RES) :  "as" (__ARG1), "I" (ARG2) ); \
  __RES; \
 })

#endif

