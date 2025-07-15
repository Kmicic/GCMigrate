/*
  ===== Version 1.6 =====
- Lightweight libraries:
- The microDS3231 library is used for the clock
- The microLiquidCrystal_I2C library is used for the display
- The microDS18B20 library is used for ds18b20
- The GyverBME280 library is used for BME280
- A little bit of code optimization
- Fixed a bug with the output of inverse relay states in the DEBUG window
- Added the ability to work with negative temperatures in Sensor mode
- Fixed the failure to remember SP and PP settings in Service
- ServoSmooth has been updated, servo operation has been improved
- Fixed a bug with saving settings
- Your settings will be reset when switching to 1.5!
- In PID, the setting has been changed to decimal fractions
- BME and Dallas display the temperature in decimal fractions
- The settings steps have been changed to smaller ones
- The time setting in the service has been fixed
- Added PID mode for channels 1 and 2 (low-frequency PWM). Channels are marked *
- For the "reverse" PID mode, you need to set negative coefficients!
- In the week, you can select the on time less than the off time
- More memory optimization
- Added "fast rotation" of the encoder: the step of changing the value increases with fast rotation
- Slightly optimized PID
- Values ​​with a dot in the graphs
- Automatic graph scale
- The structure of the settings menu has been changed
- Added a date setting in the settings menu
- Sunday is now the number 7 (was 0)
- Fixed the time setting in Timer RTC
- Correct display and operation of Servo channels operating as a relay
- Improved PID operation
- Added names for additional sensors, improved design
- The polling period and graph period have been moved to the settings
- EVEN MORE OPTIMIZATION
- Added support for the MH-Z19 carbon throttle sensor. It is possible to disable autocalibration.
- Fixed a bug in graphs
- PID and DAWN have been converted into a linear menu
- Optimized graphs
- Added the ability to disable servo smoothness to relieve memory
- Optimized memory and display output
- Fixed a bug with the drive when exiting the service
- Minor changes in the service window
- Added a start menu with service and reset (can be enabled optionally)
- Minor memory optimization
- Fixed the display of a relay with a low level in the service
- Added a schedule for PID. Daily and for a period in days
- Fixed a bug with the backlight
- Fixed critical errors with servo (pid and dawn)
- FIXED BUGS WITH SAVING SETTINGS
- Strong optimization of RAM
- Graphs are now saved for all periods (15 days, 15 hours, 15 minutes)
- When selecting a sensor, the values ​​​​are immediately equated to its reading (pid, sensor)
- Added PID debug mode (output graphs to PC)
- Minor optimization of dawn
- Fixed BME280 for negative temperatures (update the library)
- Added servo direction setting for PID (on the main servo tab, Direction)
- The system now knows that the door was opened in manual mode and works correctly
- Added a limitation of the integral component in PID
- Redesign of the list of settings
- Added PID autotuner
- Added support for several ds18b20 with different operating modes
- Improved ds18b20 stability (library updated)
- In Sensor mode, the system works when setting the same threshold values
- Fixed a bug linking servo and relay channels
- In the PID controller for inert systems, it makes sense to raise T, the logic of operation has changed slightly
- Setting the drive timeout in tenths of a second
- Improved responsiveness and accuracy of the encoder

1.6.1
- Increased the drive timeout limit
- When switching, the settings will be reset!

1.6.2 - added support for HTU21D (replaces BME280)
1.6.3 - fixed a bug with servo (there was a compilation error)
*/
