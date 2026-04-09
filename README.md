## LIFT TASK README

NOTE TO REVIEWERS: This is a functional prototype — not a perfectly realistic one! (No structure or appearance of an actual lift)

Check the demonstration video:

https://youtu.be/D_R22rbgrxI?feature=shared

GOING UP AND DOWN announcement added:
https://youtube.com/shorts/ys9nrm650FM?si=cprH3Ltj1R5hzCUx

### Establish and flash DTCM in dctm folder
- `make clean`
- `make all`
- `make flash`

### BUILD APP in app folder
- `go mod init app`
- `go get go.bug.st/serial`
- `go build -o app.exe`

### RUN APP in app folder (Requires windows operation system for SAPI)
- `.\app.exe`

### System architecture

The system is built around a simple main control loop running on a microcontroller.
Input handling, lift logic, motor control, and door control are managed by the DTCM.

Button inputs are read through digital I/O pins and processed using a software debounce mechanism.
Floor requests are stored in compact data structures and evaluated continuously while the lift is running.

A hardware timer is configured to generate a PWM signal that controls a servo motor which represents lift movement between floors.
Door behaviour and timing are handled in software using time counters and state variables.

The system communicates lift events over a serial connection, allowing an external application to react to movement and door events.
