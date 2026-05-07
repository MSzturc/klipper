# Heater Diagnostics

## What this is

When something goes wrong with a heater — a thermistor wire breaks, a
sensor falls off the bed, an ADC reads outside its calibrated range —
Klipper triggers an MCU shutdown and prints an error line. The stock
shutdown text names the failure ("ADC out of range") but doesn't say
*which* heater is to blame. On a printer with one heater that's no
problem; on a printer with hotend, bed, chamber, and a couple of
generic heaters, finding the right thermistor takes longer than it
should.

This fork adds two small quality-of-life touches on the diagnostic
path:

1. **ADC-out-of-range shutdowns name the offending heater.** When the
   firmware reports an ADC range fault, the shutdown message now lists
   each heater whose `last_temp` is outside its configured `[min_temp,
   max_temp]` window, with the actual reading and the configured
   bounds. Goes in alongside the per-ADC-sensor diagnostic that already
   ships in stock Klipper.
2. **`PrinterHeaters.lookup_heater` accepts the full section name.**
   Other modules (and macros that go through `printer.lookup_object`)
   can now resolve a heater by `"generic_heater chamber"` as well as by
   the bare `"chamber"` short name.

Neither change adds a new G-code command. They tighten the loop between
"something failed" and "I know what failed."

## When to use this

Both features are passive — they kick in automatically when relevant.
You don't reach for them; you just notice they're there the first time
a thermistor falls off mid-print.

## Things to know

- **The heater diagnostic appears in addition to the per-ADC-sensor
  one.** Klipper already emits a "Sensor 'X' temperature Y not in range
  A:B" line during `ADC out of range` shutdowns, naming the ADC sensor
  whose voltage clipped. The new heater diagnostic adds parallel
  information from the heater's perspective: which heater (by section
  name) saw a `last_temp` outside its safety window, and what the
  configured `min_temp`/`max_temp` were. The two viewpoints can name
  different things — the ADC sensor name (e.g. `MAX31865 chamber_rtd`)
  and the heater section name (e.g. `heater_generic chamber`) for the
  same physical failure — which is helpful for diagnosis.
- **The diagnostic only fires for heaters with `is_adc_faulty()`.** All
  built-in heater classes implement it. A future third-party heater
  module without the method would silently be skipped rather than
  masking the original ADC error with an `AttributeError`.
- **`lookup_heater("<prefix> <name>")` strips the prefix once.** Anything
  before the first space is dropped before the lookup. There is no
  validation that the prefix is a known section type — the call
  succeeds as long as the part after the first space is the heater's
  registered short name.

## Credits

Based on the ADC out-of-range diagnostics from the Kalico community
(Rogerio Goncalves, KalicoCrew PR #182), the companion `hasattr` guard
(KalicoCrew PR #252), and the `lookup_heater` shorthand fix
(KalicoCrew PR `9bd0a03`). Re-derived against Klipper's existing
`error_mcu.add_clarify` extension hook so the heater diagnostic
composes cleanly with the per-ADC-sensor one.
