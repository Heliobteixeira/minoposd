# minoposd

This a modified version of the original firmware minOPOSD, available at:
https://code.google.com/p/minoposd/
The changes were made to work with the Micro KVOSD boards.

The battery voltage displays correctly but needs calibration.

The RSSI is enabled by default and is assumed to be transmitted by a PPM channel (#7) over UAVTalk (from the CC3D) as described in here:
https://wiki.openpilot.org/display/WIKI/OSD+-+How+to+Access+Telemetry+Data#OSD-HowtoAccessTelemetryData-RSSI(ReceivedSignalStrengthIndicator)
This RSSI feed method also works with telemetry enabled Frsky Receivers (X8R, X6R) but it is needed to configure the Taranis Tx to send the scaled (100) RSSI signal to an aditional channel back to the Rx.
