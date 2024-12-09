Welcome to the Klipper project!

It contains the following additions:
- Added Support for BDSensor - https://github.com/markniu/Bed_Distance_sensor
- Fixed TMC5160 Driver - https://github.com/Klipper3d/klipper/pull/6644
- Added Normalized Fan PWM power - https://github.com/DangerKlippers/danger-klipper/pull/44
- Added Curve Fan Control - https://github.com/DangerKlippers/danger-klipper/pull/193/files
- Rotate log file at every Restart - https://github.com/KalicoCrew/kalico/pull/181
- Allow Plugins to be installed into klipper without marking klipper installation as 'dirty'
- Added RELOAD_GCODE_MACROS to reload config withour restaring klipper - https://github.com/KalicoCrew/kalico/pull/305
- Improved Sensorless Homing implementation - https://github.com/KalicoCrew/kalico/pull/90 / https://github.com/KalicoCrew/kalico/pull/65
- Added optional MCU support - https://github.com/KalicoCrew/kalico/pull/339
- Enable 'force move' by default - https://github.com/KalicoCrew/kalico/pull/135
- Enable 'exclude_object' by default https://github.com/KalicoCrew/kalico/pull/306
- Enable 'respond' by default https://github.com/KalicoCrew/kalico/pull/306
- Allow config includes to use subfolder globs - https://github.com/Klipper3d/klipper/pull/6375
- Auto Backup printer.cfg with all includes - https://github.com/KalicoCrew/kalico/pull/153
- Execute a linux command/script from within Klipper - https://github.com/Klipper3d/klipper/pull/2173
- Be able to define a cpu for every MCU