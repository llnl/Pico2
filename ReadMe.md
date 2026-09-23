README
2_picoinjector_firmware.ino 
is a firmware code file:
Reads a laser input signal from analog pin A1 on an Arduino Due using 12-bit ADC resolution.
Classifies the ADC voltage into four states: STATE0, STATE1, STATE2, and STATE3 based on low, medium, and high voltage thresholds.
Controls two injectors: Injector 1 turns on for medium/high signals, while Injector 2 turns on for low or high signals.
Uses hysteresis, quiet-time checks, and a falling-edge filter to prevent false injector firing from noise or rising voltage transitions.
Schedules injector pin changes with timed delays and fixed ON durations, while also tracking state counts, durations, voltage averages, and speed-related metrics.

ImageJ Macros.zip 
contains several imageJ macros used to analyze video data. 

LLNL-CODE-2024515 