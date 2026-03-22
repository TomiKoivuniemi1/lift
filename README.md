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

## Technical answers:

### How floor requests are stored, processed, and prioritized within the DTCM system?
The DTCM continuously scans inputs and adds them to queue without interrupting lift movement.
Floor requests are stored as bitmasks where each bit represents one floor (0–3).
Separate bitmasks are used for cabin requests and up and down hall requests.
The lift only stops at floors with active requests in that direction.

### The project can simulate lift movement between four distinct floors.
There are four floors 0 -> 3. The postitioning can be verified on the demonstartion video.

### The lift uses a servo motor to control motion between floors.
This is demonstrated in the video. Servo moves between the floors marked in the cardboard. (In real world it would move the cable to move the lift between floors).

### The lift supports smooth acceleration and deceleration.
The delays and steps of servo angles smooths the movement. On the video you can see the smooth moving even if going straight from floor 0 to 3. 

### The system correctly handles floor requests made from inside the lift (internal buttons (0–3))
Can be seen on demonstration video

### The system correctly handles floor requests made from outside the lift (up/down buttons)
Can be seen on demonstration video

### The lift implements directional priority logic when servicing requests.
Can be seen on demonstration video

### The system can accept new requests while the lift is moving.
Can be seen on demonstration video

### The project includes a visual floor display using a stepper motor.
Can be seen on demonstration video

### The current direction of travel is clearly displayed.
Can be heared on demonstration video (Moving down/up is announced)

### The system includes a return-home function triggered on power-up.
Can be seen on demonstration video (Returns to floor 0 after power restoration)

### The lift cannot move unless the 'doors closed' indicator is active.
Can be heared on demonstration video (Doors closing is announced before allowing moving to next floor)

### The code is written in pure C and compiled directly without using the Arduino IDE.
Main.c and makefile are in pure C. No arduino commands used. The app for audio announcements is in GO but is separate.

### The project is correctly flashed to the Arduino using command-line tools.
The upload process is with Makefile and is documented in readme.

### The student can explain how they configured and used microcontroller registers to manage I/O and timers.
Digital I/O pins are configured directly using the microcontroller’s DDR and PORT registers.
Button pins are set as inputs with internal pull-up resistors enabled and output pins are configured for the servo control signal.
A hardware timer (Timer1) is configured using its control registers to generate a PWM signal for the servo motor.

### The student can describe the use of timers or interrupt routines in coordinating lift logic and I/O.
The timer period and compare values are set so the servo receives a 50 Hz control signal and the pulse width is adjusted to move the lift smoothly between floors.

### The student can explain how to find and use the datasheet and pinout for their Arduino model.
The files are included in the documentation floder:
A000066-datasheet.pdf — Register addresses, bit definitions, timer configuration options of the ATmega328P microcontroller.
A000066-full-pinout.pdf — Internal MCU pins (PB1 etc) and the physical Arduino header pins (D9 etc) showing exactly where to connect each component.

### The student can explain any debounce strategy used for button inputs.
Button inputs are debounced in software by checking that a button is still pressed after a short delay.
This prevents false triggers caused by mechanical button bounce.

### The README.md includes a description of the system architecture.
It does.

### A video has been provided which demonstrates the required functionality.
It is.

### A visual circuit schematic is provided showing all components and connections for the lift system.
Can be found in the root.

### The student has implemented at least one bonus feature without breaking the required default behaviour.
Not really. People may use leds etc real world indicators in this, i decided to use sound and build an app around it, maybe that counts, it's up to you.