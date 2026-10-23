#include "io_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "periph.h"
#include "f3hw.h"
#include "common.h"

// BIF and B2IF read or cleared together in the rt. LL only has one helper
// per flag, and two SR accesses would cost CCM the rt path does not have.
#define TIM8_BRK_FLAGS (TIM_SR_BIF | TIM_SR_B2IF)

/**
* ## Brief
* `io` is the hardware I/O of the F3 (HV board). It reads the phase currents, phase voltages, DC link voltage and temperatures from the ADCs, calibrates the current offsets at boot, rebuilds a phase current the ADC could not sample, switches the power stage on and off, raises the HV fault codes (overcurrent, overvoltage, overtemperature, driver fault) and runs the short-circuit brake. It also drives the status LED, the brake output and the DAC reference of the overcurrent comparators. It is loaded by `stm32f303/src/main.c` as `io0` (rt_prio 1, so it runs right after `ls0` and `angle0`). The F3 rt runs once per PWM period, at `PWM_FREQ`, a build option (10, 15 or 20 kHz, default 15 kHz); tick counts below scale with it where noted.
*
* ## Component Explanation
*
* 1. **Wiring on the F3** (fixed in main.c):
* - Inputs: `io0.hv_en = ls0.en`, `io0.sbrake = ls0.sbrake`, `io0.dac = ls0.dac`, `io0.max_cur = ls0.oc_cur`, `io0.ignore_fault_pin = ls0.ignore_fault_pin`, `io0.led = ls0.fault`.
* - Outputs: `iu/iv/iw` go to `dq0`, `u/v/w` to `ls0.u_fb/v_fb/w_fb`, `ur/vr/wr` to `emf0`, `udc_duty` to `hv0.udc` and `ls0.udc_duty` (for `pwm_volt`), `udc` to `ls0.dc_volt`, `hv_temp`/`hv_temp_ok`/`mot_temp` to `ls0`, `fault` to `ls0.fault_in` (sent to the F4), `sbrake_on` to `hv0.sbrake`.
* - `max_cur` on the F3 is the F4's `hv0.max_cur` before its `scale` (in practice `conf0.max_ac_cur`), so the trip and the short brake chop stay at the rated current when `fault0` derates the command limit. An older F4 that does not send it gives 0, and the trip is then `ABS_MAX_CURRENT` only.
*
* 2. **ADC readout** (rt):
* - ADC1/2 and ADC3/4 run as dual simultaneous pairs, triggered by TIM8 at the counter extreme where the low sides conduct (set up in main.c). Each pair converts three current samples (61.5 cycles each, about 3.1 us together) and one voltage. Each rt tick waits for both DMA transfers; if they have not finished after about 50 us the rt is stopped (`hal_stop`) and the watchdog resets the F3.
* - The phase currents are the mean of the three samples, converted with the shunt amplifier constants (3 mOhm shunt, gain 16) into amps (folded into one multiply and one subtraction), then the offset is subtracted and the sign inverted.
* - `ur`, `vr`, `wr` are the phase voltages to ground from one unfiltered ADC sample per PWM period, scaled by the resistor divider. They feed `emf0`.
* - `u`, `v`, `w` and `udc` are low pass filtered with `750 / PWM_FREQ` new value per tick (5 % at 15 kHz), a time constant of about 1.3 ms at any PWM frequency. `udc` is for display, `pwm_volt` and the trips.
* - `udc_duty` is the same link sample filtered with 50 % new value per tick (about 1.4 ticks), for `hv0`'s duty division and `ls0.pwm_volt`, so link sag on acceleration and the rise on regen reach the duty within about 0.1 ms (at 15 kHz).
* - `iabs` is the largest of `|iu|`, `|iv|`, `|iw|` (after the reconstruction below).
*
* 3. **Phase current reconstruction** (rt, `recon` > 0, the default):
* - A phase's low side shunt only carries its current while its low side is on. From the TIM8 compare values `io` knows each phase's low side half pulse (`ARR - CCR`, in timer ticks): the half after the counter extreme comes from the compare loaded at this extreme (`io` runs before `hv0` rewrites it; read through `PWM_U/V/W`), the half before from last tick's.
* - A phase is flagged when the half after is shorter than the sample window (`ADC_CUR_WINDOW_TICKS`, the three conversions plus 0.5 us ring down, about 3.6 us) or the half before is shorter than the dead time (2.0 us). This happens near full duty: with `min_off` 3 us the low side is on for only part of the window, and the phase would read low, which the current loop answers by pushing harder (a torque reversal at speed).
* - The flagged phase is rebuilt from the other two, `iu = -(iv + iw)` and so on (no neutral, so the three sum to 0). The other two are valid nearly always, as a phase at the top of the range forces the others down.
* - Two phases are flagged together only when both sit within the window of the top, right at the voltage limit: at the default `min_off` about 0.2 % of the electrical turn on the full `pwm_volt` circle, none at 95 % of it. `hv0` does not prevent it. Then only the one with the shortest pulse is rebuilt, from a partner that is marginal but the better of the two, and the other is left as sampled.
*
* 4. **Offset calibration** (rt, at start):
* - Ticks 0..99 are skipped. Ticks 100..199 average the current readings into the offsets. At tick 200, an offset above 5 A raises `HV_CURRENT_OFFSET_FAULT`. After that the offsets are output on `uo`, `vo`, `wo`, and normal operation starts. This takes about 13 ms at 15 kHz (20 ms at 10 kHz, 10 ms at 20 kHz). Before that, the bridge cannot be enabled.
* - The motor must not carry current while this runs (the F4 does not enable the bridge this early after boot).
*
* 5. **Faults** (rt):
* - `HV_TEMP_ERROR`: `hv_temp` > 110 degC (`ABS_MAX_TEMP`) for about 0.33 ms (`PWM_FREQ / 3000` ticks, 5 at 15 kHz).
* - `HV_VOLT_ERROR`: `udc` > 400 V (`ABS_MAX_VOLT`) for about 0.33 ms.
* - `HV_OVERCURRENT_FILTERED`: `iabs` > 0.95 * 35 A for about 0.33 ms, a filtered peak value.
* - `HV_OVERCURRENT_PEAK`: `iabs` > `oc_lim` in two ticks in a row (one sample over it is noise: a rebuilt phase carries the noise of the other two), or `iabs` > 35 A in a single tick (`ABS_MAX_CURRENT`, the shunt measurement range, not a module rating). This is the main software trip, the one that follows the configured current:
* ```c
* oc_lim = max_cur > 0 ? CLAMP(oc_k * max_cur, oc_min, 35) : 35;
* ```
* - `HV_FAULT_ERROR`: the driver's fault pin (PB7, active low) for about 0.33 ms while enabled, unless `ignore_fault_pin` is set.
* - `HV_OVERCURRENT_HW`: the bridge is enabled but TIM8's MOE bit has been cleared or a break flag (BIF/B2IF) is set. The flags catch a break that lands between the enable edge's MOE and driver enable writes and leaves MOE set again once the comparator releases. The comparators COMP2 (U, PA7), COMP4 (V, PB0) and COMP1 (W, PA1) compare the phase currents with the DAC reference (`dac`, on DAC1 channel 1, output buffer off) and trip the TIM8 break inputs in hardware (COMP4 on BRK, COMP1 and COMP2 on BRK2), all three through the break digital filter (0xC, fDTS/16 with N = 8, about 0.9 us).
* - `HV_UNDERVOLT_ERROR`: the same break, but taken with the PVD active: the F3's 3.3 V, which comes from HV, fell below about 2.7 V (PVD level 6, main.c), which breaks TIM8 in hardware through SYSCFG PVD_LOCK. The break interrupt latches it and `io`'s nrt relabels the `HV_OVERCURRENT_HW` that rt latched.
* - Second shutdown path: the break also raises `TIM8_BRK_IRQHandler` (main.c, NVIC priority 0; the rt on TIM8_UP runs at 1 and SysTick at 14, so it preempts the rt). It clears MOE, sets the driver enable pin PA15 high (the IPM's ITRIP) within about a microsecond instead of at the next rt tick (one to two PWM periods later, 67 to 133 us at 15 kHz), so the IPM turns its gate driver off on its own and holds it off for at least 40 us, and then switches the break interrupt (BIE) off so the still-set flag cannot re-enter. `io`'s nrt re-arms it, with the break flags cleared, whenever the bridge is not enabled and not braking; the fault itself is still reported by rt as above.
* - The 0.33 ms filters use `err_filter` with a limit of `PWM_FREQ / 3000` (5 at 15 kHz): +1 per tick with the error, -0.001 without (-0.01 for the fault pin), and trip at 99 % of the limit (4.95 at 15 kHz).
* - `fault` shows the current fault code (0 = none) every tick.
*
* 6. **Enable** (rt):
* - On the rising edge of `hv_en` stale break flags are cleared and the break interrupt is re-armed (a break while idle is not this enable's; a comparator still active keeps MOE off, which the `HV_OVERCURRENT_HW` check reports), then TIM8 MOE is set and the driver enable pin (PA15) is pulled low (enabled). `ls0` holds `hv_en` at 0 until the F4 has sent its whole configuration (after a boot, or after a link gap of 10 ms or more).
* - When a fault is set while enabled, MOE is cleared (all six outputs go to their idle state at once, as the comparators do in hardware) and the driver enable pin is set high, in the same tick, a fault pin trip included. The fault stays latched until `hv_en` goes to 0.
* - With `hv_en` = 0 and no braking, MOE is cleared, the driver enable pin is set high, and the fault is cleared (unless `sbrake` is still requested, see below). `HV_CURRENT_OFFSET_FAULT` is the exception: it is found once at boot and stays until reset, also over later trip codes (an overcurrent from the bad offset cannot hide it).
*
* 7. **Short-circuit braking** (rt, while `hv_en` = 0):
* - When `sbrake` > 0 (from the F4's flag, or from `ls0` on a link loss), the offset calibration is done, and no fault has been seen since the last rising edge of `hv_en`, `io` brakes: `sbrake_on` = 1, which makes `hv0` hold all compares at 0, so all three low sides are on and the back EMF drives current round the windings; the energy stays in the motor. The driver enable pin is pulled low.
* - The first 2 ticks keep MOE off, so the zero compares reach the timer first; stale break flags are cleared when braking starts.
* - After that the current is chopped per tick: `iabs` above the limit clears MOE for the next tick (the current then flows back into the DC link through the diodes), otherwise MOE is set.
* ```c
* lim = sbrake_cur > 0 ? sbrake_cur : (max_cur > 0 ? max_cur : 10);
* lim = MIN(lim, 0.8 * oc_lim);
* ```
* - Braking is refused after any fault since the last enable (a failed switch plus three low sides on would be another short), and before the first enable after power up.
* - While braking, the fault pin (unfiltered), a TIM8 break flag (`HV_OVERCURRENT_HW`) and all the checks of item 5 are watched. A fault stops the braking for good (until the next enable edge), turns MOE and the driver off, and stays reported in `fault` until `sbrake` goes to 0, so the F4 sees it and drops the request instead of retrying.
*
* 8. **Temperatures, LED and brake** (nrt):
* - The thermistor voltages are converted into degC with a beta model (85 kOhm at 25 degC, B = 4092), for `hv_temp` (3.9 kOhm pullup, ADC4 rank 2) and for `mot_temp` (10 kOhm pullup behind a 51k/10k divider, ADC4 rank 3). A resistance below 500 Ohm (above about 200 degC) reads as 0 degC. The logarithm is a single precision approximation (error well below 0.001 K). `mot_temp` is filtered with 1 % new value per nrt call.
* - The IPM's NTC shares the VFO pin, which the module pulls low while the driver enable pin (HV_EN) is high. So `hv_temp` is updated only while HV_EN is low and the pin is above 0.3 V: while enabled, more than `PWM_FREQ / 200` rt ticks (5 ms) after the enable edge, or while short-circuit braking, 5 ms (by `tick_ms()`) after braking started (the low sides heat then too). The first reading after power up or after a hold is taken as is, later ones are filtered with 1 % per nrt call.
* - Otherwise `hv_temp` is held and decays toward the coolest live reading since power up, with a time constant of 90 s, so a fan or derating keyed to it lets go after the bridge stops. Before the first reading `hv_temp` reads 0.
* - The LED blinks `led` times (it is wired to `ls0.fault`), or 2 times while the HAL is not running normally.
* - If the brake circuit on PB2 reads high at start, `brk_present` is set to 1 and the pin becomes an output. `brk` > 0 then drives it low, otherwise high.
*
* {{% hint warning %}}
* - Both thermistors use the same NTC constants. This is specific to the HV board and to the motor sensor it was built for.
* - `brk`, `oc_k`, `oc_min`, `sbrake_cur` and `recon` are not wired in main.c and not carried by the F4 link, so they can only be changed from the F3 terminal.
* {{% /hint %}}
*/

HAL_COMP(io);

HAL_PIN(led);  // *input*, Number of LED blinks, from ls0.fault

//phase current
HAL_PIN(iu);  // *output*, U phase current (A)
HAL_PIN(iv);  // *output*, V phase current (A)
HAL_PIN(iw);  // *output*, W phase current (A)
//total current
HAL_PIN(iabs);  // *output*, Largest absolute phase current (A)

HAL_PIN(recon);  // *parameter*, > 0 rebuilds a phase that could not be sampled from the other two, default 1
HAL_PIN(recon_phase);  // *output*, Phase rebuilt this tick, 0 = none, 1 = u, 2 = v, 3 = w

HAL_PIN(max_cur);  // *input*, Rated maximum current for the software trip and the brake chop (A), from ls0.oc_cur, 0 = trip at 35 A only
HAL_PIN(oc_k);  // *parameter*, Software overcurrent trip as a factor on max_cur, default 1.3
HAL_PIN(oc_min);  // *parameter*, Lower limit for the software overcurrent trip (A), default 5
HAL_PIN(oc_lim);  // *output*, Software overcurrent trip level in force, at most 35 A (A)

//phase voltage
HAL_PIN(u);  // *output*, U phase voltage (V), low pass filtered
HAL_PIN(v);  // *output*, V phase voltage (V), low pass filtered
HAL_PIN(w);  // *output*, W phase voltage (V), low pass filtered
//dclink voltage
HAL_PIN(udc);  // *output*, DC link voltage (V), low pass filtered (tau about 1.3 ms), for ls0 and the trips
HAL_PIN(udc_duty);  // *output*, DC link voltage (V), fast filter (50 % per tick), for hv0.udc and ls0.udc_duty (pwm_volt)

HAL_PIN(ur);  // *output*, U phase voltage to ground (V), one unfiltered sample per PWM period, for emf0
HAL_PIN(vr);  // *output*, V phase voltage to ground (V), one unfiltered sample per PWM period, for emf0
HAL_PIN(wr);  // *output*, W phase voltage to ground (V), one unfiltered sample per PWM period, for emf0

//driver temperature
HAL_PIN(hv_temp);  // *output*, Power stage temperature (degC), live while enabled or braking, else held and decaying
HAL_PIN(hv_temp_ok);  // *output*, hv_temp state, 0 = never read, 1 = live, 2 = held
//motor temperature
HAL_PIN(mot_temp);  // *output*, Motor temperature (degC)

//ADC offset outputs
HAL_PIN(uo);  // *output*, U current offset from the boot calibration (A)
HAL_PIN(vo);  // *output*, V current offset from the boot calibration (A)
HAL_PIN(wo);  // *output*, W current offset from the boot calibration (A)

//DAC value for comparators
HAL_PIN(dac);  // *input*, Overcurrent comparator reference, raw DAC value 0..4095, from ls0.dac

//comparator outputs
HAL_PIN(cu);  // *output*, U overcurrent comparator output (COMP2)
HAL_PIN(cv);  // *output*, V overcurrent comparator output (COMP4)
HAL_PIN(cw);  // *output*, W overcurrent comparator output (COMP1)

//enable in
HAL_PIN(hv_en);  // *input*, Enable the power stage, from ls0.en

//fault output
HAL_PIN(fault);  // *output*, HV fault code, 0 = none, see Faults
HAL_PIN(ignore_fault_pin);  // *input*, Ignore the driver fault pin if > 0, from ls0.ignore_fault_pin

HAL_PIN(brk_present);  // *output*, Brake circuit detected at start
HAL_PIN(brk);  // *input*, Brake output, > 0 drives the brake pin low, not wired

HAL_PIN(sbrake);  // *input*, Short-circuit braking request while hv_en = 0, from ls0.sbrake
HAL_PIN(sbrake_cur);  // *parameter*, Braking chop threshold (A), 0 = max_cur (10 A if that is 0), at most 0.8 * oc_lim, default 0
HAL_PIN(sbrake_on);  // *output*, 1 while braking, to hv0.sbrake which holds all compares at 0


volatile uint32_t adc_12_buf[ADC_SEQ_LEN];
volatile uint32_t adc_34_buf[ADC_SEQ_LEN];

struct io_ctx_t {
  float u_offset;
  float v_offset;
  float w_offset;
  float overtemp_error;
  float overvoltage_error;
  float overcurrent_error;
  float fault_pin_error;
  uint32_t offset_count;
  uint32_t offset_bad;  // the boot offset check failed: the fault stays until reset
  uint32_t oc_ticks;    // ticks in a row with iabs over oc_lim
  uint32_t hv_temp;
  uint32_t mot_temp;
  uint32_t fault;
  uint32_t enabled;
  uint32_t en_ticks;  // rt ticks since the enable edge, saturating
  uint32_t sbrake_ok;     // no trip since the last enable edge
  uint32_t sbrake_ticks;  // ticks since braking started
  int32_t lo_u;  // low-side half pulse of the previous tick, in timer ticks
  int32_t lo_v;
  int32_t lo_w;
  float hv_temp_min;    // coolest live hv_temp since power up, the decay target
  uint32_t hv_temp_ms;  // tick_ms() at the last nrt pass
  uint32_t sbrake_ms;   // tick_ms() at the last nrt pass not braking
};

#define ARES 4096.0  // analog resolution, 12 bit
#define ADC(a) ((a) / ARES * AREF)

#define HV_TEMP_PULLUP 3900
#define HV_R(a) (HV_TEMP_PULLUP / (AREF / (a)-1))
// u/v/w and udc low pass: 0.05 per tick at 15 kHz, tau 1.33 ms at any rate
#define IO_LP_K (750.0 / PWM_FREQ)
// err_filter trip: 5 bad ticks at 15 kHz, about 0.33 ms at any rate
#define IO_ERR_TICKS (PWM_FREQ / 3000.0)
#define HV_TEMP_SETTLE (PWM_FREQ / 200)  // rt ticks after enable before VFO counts as released, 5 ms
#define HV_TEMP_SETTLE_MS 5  // the same after short braking starts [ms]
#define HV_TEMP_MIN_V 0.3  // below this the pin is held low, not an NTC reading [V]
#define HV_TEMP_TAU 90.0   // decay of the held value, X cooled 42.4 to 31.2 C in about 140 s [s]

#define MOT_TEMP_PULLUP 10000
#define MOT_TEMP_PULLMID 51000
#define MOT_TEMP_PULLDOWN 10000
#define MOT_TEMP_REF 15.26
#define MOT_REF(a) ((a) * (MOT_TEMP_PULLMID + MOT_TEMP_PULLDOWN) / MOT_TEMP_PULLDOWN)
#define MOT_R(a) (MOT_TEMP_PULLUP / (MOT_TEMP_REF / (a)-1))

#define VOLT(a) ((a) / (ARES) * (AREF) / (VDIVDOWN) * ((VDIVUP) + (VDIVDOWN)))
//#define TEMP(a) (log10f((a) * (AREF) / (ARES) * (TPULLUP) / ((AREF) - (a) * (AREF) / (ARES))) * (-53.0) + 290.0)

#define SHUNT_GAIN 16.0

#define AMP(a, gain) (((a)*AREF / ARES / (gain)-AREF / (SHUNT_PULLUP + SHUNT_SERIE) * SHUNT_SERIE) / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE))
// AMP and VOLT folded to one multiply for the rt path; sum is the sum of
// ADC_CUR_SAMPLES samples
#define AMP_K ((float)(AREF / ARES / SHUNT_GAIN / ADC_CUR_SAMPLES / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE)))
#define AMP_0 ((float)(AREF / (SHUNT_PULLUP + SHUNT_SERIE) * SHUNT_SERIE / (SHUNT * SHUNT_PULLUP) * (SHUNT_PULLUP + SHUNT_SERIE)))
#define AMPN(sum) ((float)(sum)*AMP_K - AMP_0)
#define VOLT_K ((float)(AREF / ARES / VDIVDOWN * (VDIVUP + VDIVDOWN)))

// single precision natural log for r2temp, in place of the soft-float double log():
// x = m * 2^e with m in [0.707, 1.414), ln m = 2 atanh(s), s = (m - 1) / (m + 1),
// |s| < 0.172, series to s^7, error below 2e-6 (under 0.0001 K here); x > 0, finite
static float ln_f(float x) {
  union {
    float f;
    uint32_t i;
  } u   = {x};
  int e = (int)((u.i >> 23) & 0xff) - 127;
  u.i   = (u.i & 0x007fffff) | 0x3f800000;  // m in [1, 2)
  if(u.f > 1.41421356) {
    u.f *= 0.5;
    e++;
  }
  float s  = (u.f - 1.0) / (u.f + 1.0);
  float s2 = s * s;
  return e * 0.69314718 + 2.0 * s * (1.0 + s2 * (1.0 / 3.0 + s2 * (1.0 / 5.0 + s2 * (1.0 / 7.0))));
}

float r2temp(float r) {
    if (r < 500)  // above about 200 C, past any data on the part
      return 0;

    const float B  = 4092.0;      // Beta coefficient
    const float T0 = 298.15;      // reference temp in Kelvin (25 degC)
    const float R0 = 85000.0;     // resistance at 25 degC, in ohms
    float tempK = 1.0 / (1.0 / T0 + (1.0 / B) * ln_f(r / R0));
    return tempK - 273.15;        // convert to Celsius
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  PIN(brk_present) = 0.0;
  PIN(brk)         = 0.0;

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  LL_GPIO_StructInit(&GPIO_InitStruct);
  //LED
  GPIO_InitStruct.Pin   = LED_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  // BRK
  GPIO_InitStruct.Pin   = BRK_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  LL_GPIO_Init(BRK_PORT, &GPIO_InitStruct);

  if(LL_GPIO_IsInputPinSet(BRK_PORT, BRK_PIN)) {  // BRK circuit detected
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BRK_PORT, &GPIO_InitStruct);
    PIN(brk_present) = 1.0;
  }

  // the ADC DMA is set up in main.c, before the ADCs start (RM0316 15.5.4)

  ctx->offset_count      = 0;
  ctx->u_offset          = 0.0;
  ctx->v_offset          = 0.0;
  ctx->w_offset          = 0.0;
  ctx->fault             = NO_ERROR;
  ctx->offset_bad        = 0;
  ctx->oc_ticks          = 0;
  ctx->overtemp_error    = 0;
  ctx->overvoltage_error = 0;
  ctx->overcurrent_error = 0;
  ctx->fault_pin_error   = 0;
  ctx->hv_temp           = 0;
  ctx->hv_temp_min       = 0.0;
  ctx->hv_temp_ms        = tick_ms();
  ctx->sbrake_ms         = ctx->hv_temp_ms;
  ctx->mot_temp          = 0;
  ctx->enabled           = 0;
  ctx->en_ticks          = 0;
  PIN(hv_temp_ok)        = 0.0;
  ctx->sbrake_ok         = 0;
  ctx->sbrake_ticks      = 0;
  // no previous tick yet: long enough that the first one is not flagged
  ctx->lo_u              = 0x7FFFFFFF;
  ctx->lo_v              = 0x7FFFFFFF;
  ctx->lo_w              = 0x7FFFFFFF;
  PIN(recon)             = 1.0;
  PIN(recon_phase)       = 0.0;


#ifdef HV_EN_PIN
  GPIO_InitStruct.Pin   = HV_EN_PIN;
  GPIO_InitStruct.Mode  = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull  = LL_GPIO_PULL_NO;
  LL_GPIO_Init(HV_EN_PORT, &GPIO_InitStruct);
#endif

#ifdef HV_FAULT_PIN
  GPIO_InitStruct.Pin  = HV_FAULT_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(HV_FAULT_PORT, &GPIO_InitStruct);
#endif
  PIN(dac) = 0;
  PIN(sbrake)     = 0.0;
  PIN(sbrake_cur) = 0.0;
  PIN(sbrake_on)  = 0.0;
  PIN(oc_k)   = 1.3;
  PIN(oc_min) = 5.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  // The sequence ends ~6 us after the tick; if the ADC DMA stalls (it only
  // requests once both ADCs of a pair finish) waiting here forever would hang
  // the rt with irqs of its priority blocked. Give up after 1000 polls
  // (two DMA ISR reads each, roughly 110-140 us at 72 MHz, longer than a PWM
  // period) and stop the rt instead: the watchdog then resets the F3.
  for(uint32_t n = 0; !(LL_DMA_IsActiveFlag_TC1(DMA1) && LL_DMA_IsActiveFlag_TC5(DMA2)); n++) {
    if(n > 1000) {
      hal_stop();
      return;
    }
  }

  LL_DMA_ClearFlag_TC1(DMA1);
  LL_DMA_ClearFlag_TC5(DMA2);

  // ranks 1-3 of each pair are current samples, rank 4 the voltage (periph.c)
  uint32_t a12 = adc_12_buf[0] + adc_12_buf[1] + adc_12_buf[2];
  uint32_t a34 = adc_34_buf[0] + adc_34_buf[1] + adc_34_buf[2];

  if(ctx->offset_count < 100) {
    ctx->offset_count++;
  } else if(ctx->offset_count < 100 + 100) {
    ctx->w_offset += AMP((float)(a12 & 0xFFFF) / (float)ADC_CUR_SAMPLES, SHUNT_GAIN) / 100.0;
    ctx->u_offset += AMP((float)(a12 >> 16) / (float)ADC_CUR_SAMPLES, SHUNT_GAIN) / 100.0;
    ctx->v_offset += AMP((float)(a34 & 0xFFFF) / (float)ADC_CUR_SAMPLES, SHUNT_GAIN) / 100.0;
    ctx->offset_count++;
  } else if(ctx->offset_count < 100 + 100 + 1) {
    if(ABS(ctx->u_offset) > 5.0 || ABS(ctx->v_offset) > 5.0 || ABS(ctx->w_offset) > 5.0) {
      ctx->fault      = HV_CURRENT_OFFSET_FAULT;
      ctx->offset_bad = 1;
    }
    ctx->offset_count++;
  } else {
    PIN(uo)       = ctx->u_offset;
    PIN(vo)       = ctx->v_offset;
    PIN(wo)       = ctx->w_offset;
    PIN(iw)       = -AMPN(a12 & 0xFFFF) + ctx->w_offset;  // 1u
    PIN(iu)       = -AMPN(a12 >> 16) + ctx->u_offset;
    PIN(iv)       = -AMPN(a34 & 0xFFFF) + ctx->v_offset;
    PIN(wr)       = (float)(adc_12_buf[ADC_SEQ_LEN - 1] & 0xFFFF) * VOLT_K;
    PIN(vr)       = (float)(adc_12_buf[ADC_SEQ_LEN - 1] >> 16) * VOLT_K;
    PIN(ur)       = (float)(adc_34_buf[ADC_SEQ_LEN - 1] & 0xFFFF) * VOLT_K;
    PIN(w)        = PIN(wr) * IO_LP_K + PIN(w) * (1.0 - IO_LP_K);  // 0.6u
    PIN(v)        = PIN(vr) * IO_LP_K + PIN(v) * (1.0 - IO_LP_K);
    PIN(u)        = PIN(ur) * IO_LP_K + PIN(u) * (1.0 - IO_LP_K);
    float udc_raw = (float)(adc_34_buf[ADC_SEQ_LEN - 1] >> 16) * VOLT_K;
    PIN(udc)      = udc_raw * IO_LP_K + PIN(udc) * (1.0 - IO_LP_K);
    PIN(udc_duty) = udc_raw * 0.5 + PIN(udc_duty) * 0.5;

    // Which phase could not be sampled. TIM8 is centre aligned and the ADC is
    // triggered at the counter extreme where the low sides conduct, so a
    // phase's low side is on from (half its off pulse) before the extreme,
    // less the dead time the DTG adds to its turn-on, to (half its off pulse)
    // after it. The half before came from the compare value of the previous
    // tick, the half after from the one loaded at this extreme, which is what
    // TIM8->CCRx reads now (io runs before hv rewrites it). The sample window
    // sits in the half after. So the sample is good only if the half after
    // outlasts the window and the half before outlasts the dead time.
    //
    // The default min_off of 3 us against the 2.0 us dead time leaves a phase
    // at maximum duty with its low side on for 1 us inside the sample window
    // (ADC_CUR_WINDOW_TICKS), and while its current flows into the bridge the
    // shunt carries nothing during the dead time, so the phase reads low and the loop
    // pushes harder. That is a torque reversal at speed. Raising min_off
    // costs output voltage; instead rebuild the phase from the other two,
    // which are valid nearly always (a phase at the top of the range forces
    // the others down): with no neutral the three sum to zero.
#ifdef PWM_INVERT
    int32_t lo_u = (int32_t)PWM_U;
    int32_t lo_v = (int32_t)PWM_V;
    int32_t lo_w = (int32_t)PWM_W;
#else
    int32_t arr  = (int32_t)LL_TIM_GetAutoReload(TIM8);
    int32_t lo_u = arr - (int32_t)PWM_U;
    int32_t lo_v = arr - (int32_t)PWM_V;
    int32_t lo_w = arr - (int32_t)PWM_W;
#endif
    PIN(recon_phase) = 0.0;
    if(PIN(recon) > 0.0) {
      int bad_u = lo_u < ADC_CUR_WINDOW_TICKS || ctx->lo_u < PWM_DEADTIME_TICKS;
      int bad_v = lo_v < ADC_CUR_WINDOW_TICKS || ctx->lo_v < PWM_DEADTIME_TICKS;
      int bad_w = lo_w < ADC_CUR_WINDOW_TICKS || ctx->lo_w < PWM_DEADTIME_TICKS;
      // Two phases are flagged together only when both sit within the window
      // of the top, i.e. right at the voltage limit: at the default min_off
      // about 0.2 % of the electrical turn on the full pwm_volt circle,
      // none at 95 % of it. hv.c does not prevent it. Then the one with the
      // shortest pulse is rebuilt from the other, which is marginal but the
      // better of the two, and the other is left as sampled.
      if(bad_u && (!bad_v || lo_u <= lo_v) && (!bad_w || lo_u <= lo_w)) {
        PIN(iu)          = -(PIN(iv) + PIN(iw));
        PIN(recon_phase) = 1.0;
      } else if(bad_v && (!bad_w || lo_v <= lo_w)) {
        PIN(iv)          = -(PIN(iu) + PIN(iw));
        PIN(recon_phase) = 2.0;
      } else if(bad_w) {
        PIN(iw)          = -(PIN(iu) + PIN(iv));
        PIN(recon_phase) = 3.0;
      }
    }
    ctx->lo_u = lo_u;
    ctx->lo_v = lo_v;
    ctx->lo_w = lo_w;
    PIN(iabs)     = MAX3(ABS(PIN(iu)), ABS(PIN(iv)), ABS(PIN(iw)));
    ctx->hv_temp  = adc_34_buf[1];  // ADC4 rank 2: the second hv_temp sample, settled
    if(!ctx->enabled) {
      ctx->en_ticks = 0;
    } else if(ctx->en_ticks < 0xFFFF) {
      ctx->en_ticks++;
    }
    ctx->mot_temp = adc_34_buf[2];  // ADC4 rank 3

    if(err_filter(&(ctx->overtemp_error), IO_ERR_TICKS, 0.001, PIN(hv_temp) > ABS_MAX_TEMP)) {
      ctx->fault = HV_TEMP_ERROR;
    }

    if(err_filter(&(ctx->overvoltage_error), IO_ERR_TICKS, 0.001, PIN(udc) > ABS_MAX_VOLT)) {
      ctx->fault = HV_VOLT_ERROR;
    }

    // filtered peak: iabs above 95 % of the range for IO_ERR_TICKS
    if(err_filter(&(ctx->overcurrent_error), IO_ERR_TICKS, 0.001, PIN(iabs) > ABS_MAX_CURRENT * 0.95)) {
      ctx->fault = HV_OVERCURRENT_FILTERED;
    }

    PIN(oc_lim) = PIN(max_cur) > 0.0 ? CLAMP(PIN(oc_k) * PIN(max_cur), PIN(oc_min), ABS_MAX_CURRENT) : ABS_MAX_CURRENT;
    ctx->oc_ticks = PIN(iabs) > PIN(oc_lim) ? ctx->oc_ticks + 1 : 0;
    if(ctx->oc_ticks >= 2) {
      ctx->fault = HV_OVERCURRENT_PEAK;
    }

    if(PIN(iabs) > ABS_MAX_CURRENT) {
      ctx->fault = HV_OVERCURRENT_PEAK;
    }

    // a later trip (an oc from the bad offset itself) must not hide it
    if(ctx->offset_bad) {
      ctx->fault = HV_CURRENT_OFFSET_FAULT;
    }

    PIN(fault) = ctx->fault;

    if(PIN(hv_en) > 0.0) {
      PIN(sbrake_on)    = 0.0;
      ctx->sbrake_ticks = 0;
      if(!ctx->enabled) {  //rising edge of enable
        // a break while idle is not this enable's: clear it and re-arm the
        // second shutdown here, nrt may not have run since. A comparator
        // still active keeps MOE off, which the check below reports.
        WRITE_REG(TIM8->SR, ~TIM8_BRK_FLAGS);
        LL_TIM_EnableIT_BRK(TIM8);
        //set timer master out enable
        LL_TIM_EnableAllOutputs(TIM8);
#ifdef HV_EN_PIN
        //clear driver enable pin
        LL_GPIO_ResetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
        ctx->enabled   = 1;
        ctx->sbrake_ok = ctx->fault == NO_ERROR;
      }
      if(ctx->fault != NO_ERROR) {
        ctx->sbrake_ok = 0;
      }
      if(ctx->fault == NO_ERROR) {
#ifdef HV_FAULT_PIN
        //read fault pin from driver
        if(PIN(ignore_fault_pin) <= 0.0 && err_filter(&(ctx->fault_pin_error), IO_ERR_TICKS, 0.01, LL_GPIO_IsInputPinSet(HV_FAULT_PORT, HV_FAULT_PIN) == HV_FAULT_POLARITY)) {
          ctx->fault = HV_FAULT_ERROR;
        }
#endif
        //Master out enable is cleared by timer break input.
        //Timer break input is connected to comperators
        // the flags too: a break between the enable edge's MOE and HV_EN
        // writes can leave MOE set again once the comparator releases
        if(!LL_TIM_IsEnabledAllOutputs(TIM8) || (TIM8->SR & TIM8_BRK_FLAGS)) {
          ctx->fault = HV_OVERCURRENT_HW;
        }
      }
      if(ctx->fault != NO_ERROR) {  // also a fault pin trip found just above
        ctx->fault_pin_error = 0;
        // a software trip (oc_lim, temperature, voltage, fault pin) takes the
        // bridge off in this tick too, not only the driver enable: MOE off
        // puts all six outputs in their OSSI idle state at once (RM0316 table 122)
        LL_TIM_DisableAllOutputs(TIM8);
#ifdef HV_EN_PIN
        //set driver enable pin
        LL_GPIO_SetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
      }
    } else if(PIN(sbrake) > 0.0 && ctx->sbrake_ok && ctx->offset_count > 200) {
      ctx->enabled = 0;
      if(ctx->sbrake_ticks == 0) {
        // a break flag left from before braking is not ours to judge
        WRITE_REG(TIM8->SR, ~TIM8_BRK_FLAGS);
      }
      if(ctx->fault == NO_ERROR) {
#ifdef HV_FAULT_PIN
        if(PIN(ignore_fault_pin) <= 0.0 && LL_GPIO_IsInputPinSet(HV_FAULT_PORT, HV_FAULT_PIN) == HV_FAULT_POLARITY) {
          ctx->fault = HV_FAULT_ERROR;
        }
#endif
        // the comparators clear MOE and set a break flag even while this
        // code holds MOE off for a chop, so the flag is the trip signal here
        if(TIM8->SR & TIM8_BRK_FLAGS) {
          ctx->fault = HV_OVERCURRENT_HW;
        }
      }

      if(ctx->fault != NO_ERROR) {
        // tripped while braking: off for good, and the fault goes to the f4
        ctx->sbrake_ok    = 0;
        ctx->sbrake_ticks = 0;
        PIN(sbrake_on)    = 0.0;
        LL_TIM_DisableAllOutputs(TIM8);
#ifdef HV_EN_PIN
        LL_GPIO_SetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
      } else {
        float lim = PIN(sbrake_cur) > 0.0 ? PIN(sbrake_cur) : (PIN(max_cur) > 0.0 ? PIN(max_cur) : 10.0);
        lim       = MIN(lim, 0.8 * PIN(oc_lim));

        PIN(sbrake_on) = 1.0;
#ifdef HV_EN_PIN
        LL_GPIO_ResetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
        // two ticks for hv0's zero compares to reach the timer
        if(ctx->sbrake_ticks < 2) {
          ctx->sbrake_ticks++;
          LL_TIM_DisableAllOutputs(TIM8);
        } else if(PIN(iabs) > lim) {
          LL_TIM_DisableAllOutputs(TIM8);
        } else {
          LL_TIM_EnableAllOutputs(TIM8);
        }
      }
    } else {
      ctx->enabled = 0;
      // a trip during braking stays reported until the request goes away,
      // so the f4 sees it and drops the request instead of retrying
      // the offset fault comes back before PIN(fault) next tick (offset_bad)
      if(PIN(sbrake) <= 0.0) {
        ctx->fault = NO_ERROR;
      }
      ctx->sbrake_ticks = 0;
      PIN(sbrake_on)    = 0.0;
      LL_TIM_DisableAllOutputs(TIM8);
#ifdef HV_EN_PIN
      //set driver enable pin
      LL_GPIO_SetOutputPin(HV_EN_PORT, HV_EN_PIN);
#endif
    }

    if(PIN(brk) > 0.0) {
      LL_GPIO_ResetOutputPin(BRK_PORT, BRK_PIN);
    } else {
      LL_GPIO_SetOutputPin(BRK_PORT, BRK_PIN);
    }
  }

  //dac output for comperators
  DAC1->DHR12R1 = CLAMP((uint32_t)PIN(dac), 0, 4095);

  //comperator outputs for debugging
  // COMP2 watches PA7 = U, COMP4 PB0 = V, COMP1 PA1 = W (schematic A_IU/A_IV/A_IW)
  PIN(cu) = (COMP2->CSR & COMP_CSR_COMPxOUT) > 0;
  PIN(cv) = (COMP4->CSR & COMP_CSR_COMPxOUT) > 0;
  PIN(cw) = (COMP1->CSR & COMP_CSR_COMPxOUT) > 0;
}

void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct io_ctx_t *ctx      = (struct io_ctx_t *)ctx_ptr;
  struct io_pin_ctx_t *pins = (struct io_pin_ctx_t *)pin_ptr;

  // A break with the PVD active (main.c pvd_trip) is an undervoltage of the
  // F3's own 3.3 V, not an overcurrent: relabel the latched rt fault. Done
  // here, in flash, because CCM is full; irqs off so the rt can't change the
  // fault between the test and the write. The latch is dropped once the
  // bridge is idle with no fault, so a stale one can't relabel a later trip.
  extern volatile uint32_t pvd_trip;
  __disable_irq();
  if(pvd_trip && ctx->fault == HV_OVERCURRENT_HW) {
    ctx->fault = HV_UNDERVOLT_ERROR;
  } else if(!ctx->enabled && ctx->fault == NO_ERROR) {
    pvd_trip = 0;
  }
  __enable_irq();

  uint32_t led = (uint32_t)PIN(led);

  if(hal.hal_state != HAL_OK2) {
    led = 2;
  }

  if(BLINK(led) > 0) {
    LL_GPIO_SetOutputPin(LED_PORT, LED_PIN);
  } else {
    LL_GPIO_ResetOutputPin(LED_PORT, LED_PIN);
  }

  // re-arm the break interrupt (second shutdown via HV_EN, main.c) while the
  // bridge is idle. Done here, in flash, because CCM is full. A trip while
  // enabled is already latched by rt from MOE, and braking clears the flags
  // itself when it starts, so clearing them here loses nothing.
  if(!ctx->enabled && PIN(sbrake_on) <= 0.0 && !LL_TIM_IsEnabledIT_BRK(TIM8)) {
    LL_TIM_ClearFlag_BRK(TIM8);
    LL_TIM_ClearFlag_BRK2(TIM8);
    LL_TIM_EnableIT_BRK(TIM8);
  }

  uint32_t now    = tick_ms();
  float dt        = (now - ctx->hv_temp_ms) * 0.001;
  ctx->hv_temp_ms = now;

  // short braking keeps HV_EN low, so VFO carries the NTC then too, and the
  // low side dies are heating
  if(PIN(sbrake_on) <= 0.0) {
    ctx->sbrake_ms = now;
  }
  int hv_en_low = (ctx->enabled && ctx->en_ticks > HV_TEMP_SETTLE) || (PIN(sbrake_on) > 0.0 && now - ctx->sbrake_ms >= HV_TEMP_SETTLE_MS);

  float hv_v = ADC(ctx->hv_temp >> 16);
  if(hv_en_low && hv_v > HV_TEMP_MIN_V) {
    float t = r2temp(HV_R(hv_v));
    // first reading since power up or after a hold: start from it, not from
    // 0 or from the decayed estimate
    PIN(hv_temp)     = PIN(hv_temp_ok) == 1.0 ? t * 0.01 + PIN(hv_temp) * 0.99 : t;
    ctx->hv_temp_min = PIN(hv_temp_ok) > 0.0 ? MIN(ctx->hv_temp_min, PIN(hv_temp)) : t;
    PIN(hv_temp_ok)  = 1.0;
  } else if(PIN(hv_temp_ok) > 0.0) {
    // a frozen value never cools, so a fan or derate keyed to it never lets
    // go: decay it toward the coolest reading seen. dt is a few ms against a
    // 90 s tau, so a first order step needs no expf (not linked on the f3)
    PIN(hv_temp) -= (PIN(hv_temp) - ctx->hv_temp_min) * MIN(dt / HV_TEMP_TAU, 1.0);
    PIN(hv_temp_ok) = 2.0;
  }
  PIN(mot_temp) = r2temp(MOT_R(MOT_REF(ADC(ctx->mot_temp >> 16)))) * 0.01 + PIN(mot_temp) * 0.99;
}

hal_comp_t io_comp_struct = {
    .name      = "io",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct io_ctx_t),
    .pin_count = sizeof(struct io_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
