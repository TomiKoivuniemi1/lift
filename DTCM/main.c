#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD 9600UL
#define UBRR_VALUE ((F_CPU/16/BAUD)-1)
#define MOVE_STEP_MS  120u
#define MOVE_STEP_DEG 1
#define DOOR_CLOSED_PAUSE_MS 4000u
#define SERVO_MIN_DELAY  4
#define SERVO_MAX_DELAY  70
#define ACCEL_STEPS      45
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

// Universal asynchronous receiver-transmitter (UART)
static void uart_init(void){
  DDRD |= (1<<PD1);                       // TX (D1) as output
  UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
  UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);     // 8N1
  UCSR0B = (1<<TXEN0);                    // TX enable
}

static void uart_putc(char c){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = c;
}

static void uart_print(const char *s){
  while(*s) uart_putc(*s++);
}

static void uart_EVENT(const char *tag){
  uart_print("EVENT ");
  uart_print(tag);
  uart_putc('\n');
}

static void uart_EVENT_floor(const char *tag, uint8_t floor){
  uart_print("EVENT ");
  uart_print(tag);
  uart_putc(' ');
  uart_putc('0' + floor);
  uart_putc('\n');
}

// Servo control
static void servo_init(void){
  DDRB  |= (1<<PB1);                            // D9 as output
  TCCR1A = (1<<COM1A1) | (1<<WGM11);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler 8
  ICR1   = 39999;                               // 20 ms (50 Hz)
}

static void servo_write_angle(uint8_t angle){
  const uint16_t min=2000, max=4000;            // 1.0–2.0 ms
  OCR1A = min + ((uint32_t)(max-min)*angle)/180;
}

static const uint8_t floors_deg[4] = {30, 70, 110, 150};

// Buttons
#define BTN_F0_INS        PB5
#define BTN_F1_INS        PB4
#define BTN_F2_INS        PB3
#define BTN_F3_INS        PB2
#define BTN_DOOR_OPEN     PD7
#define BTN_DOOR_CLOSE    PD6
#define BTN_F0_UP         PD5
#define BTN_F1_DOWN       PD4
#define BTN_F1_UP         PD3
#define BTN_F2_DOWN       PD2
#define BTN_F2_UP         PC5
#define BTN_F3_DOWN       PC4

static void io_init(void){
  // PORTB: inside buttons inputs with pull-ups
  DDRB  &= ~((1<<BTN_F0_INS)|(1<<BTN_F1_INS)|(1<<BTN_F2_INS)|(1<<BTN_F3_INS));
  PORTB |=  ((1<<BTN_F0_INS)|(1<<BTN_F1_INS)|(1<<BTN_F2_INS)|(1<<BTN_F3_INS));

  // PORTD: outside buttons inputs with pull-ups
  DDRD  &= ~((1<<BTN_DOOR_OPEN)|(1<<BTN_DOOR_CLOSE)|(1<<BTN_F0_UP)|(1<<BTN_F1_DOWN)|(1<<BTN_F1_UP)|(1<<BTN_F2_DOWN));
  PORTD |=  ((1<<BTN_DOOR_OPEN)|(1<<BTN_DOOR_CLOSE)|(1<<BTN_F0_UP)|(1<<BTN_F1_DOWN)|(1<<BTN_F1_UP)|(1<<BTN_F2_DOWN));

  // PORTC: A4 and A5 inputs with pull-ups
  DDRC  &= ~((1<<BTN_F2_UP)|(1<<BTN_F3_DOWN));
  PORTC |=  ((1<<BTN_F2_UP)|(1<<BTN_F3_DOWN));
}

static void scan_buttons_and_latch_requests(void);
static void doors_tick(void);

// Counter for uptime
static uint32_t uptime_ms = 0;

static inline void delay_ms_raw(uint16_t ms){
  while(ms--){
    _delay_ms(1);
    uptime_ms++;
  }
}

static inline void delay_ms(uint16_t ms){
  while(ms--){
    _delay_ms(1);
    uptime_ms++;
    scan_buttons_and_latch_requests();
    doors_tick();
  }
}

// Debounce and rising-edge detect for all PIN
static bool pressed_edge(volatile uint8_t *pin_reg, uint8_t mask, uint8_t *state){
  bool down = ((*pin_reg & mask) == 0);   // active-low with pull-up

  if(down){
    delay_ms_raw(10);
    down = ((*pin_reg & mask) == 0);
  }

  bool edge = (down && !(*state));
  *state = (uint8_t)down;
  return edge;
}

// Lift state
typedef enum { DIR_IDLE=0, DIR_UP=1, DIR_DOWN=2 } dir_t;
static bool floor_is_ahead(uint8_t floor, dir_t dir, const uint8_t floors_deg[4]);
static void announce_new_request(uint8_t floor, const uint8_t floors_deg[4]);
static volatile uint8_t g_curFloor = 0;
static volatile uint8_t g_curDeg   = 0;
static volatile dir_t   g_dir      = DIR_IDLE;

// Floor request storage
static uint8_t req_cabin;
static uint8_t req_up;
static uint8_t req_down;
static uint8_t bit_floor(uint8_t f){ return (uint8_t)(1u << f); } // Convert floor number into bitmask

// Check if requests
static bool any_requests(void){
  return (req_cabin | req_up | req_down) != 0;
}

static bool doors_closed = true;

static int8_t last_move_to = -1;

static void event_move_to(uint8_t floor){
  if(last_move_to == (int8_t)floor) return;
  last_move_to = (int8_t)floor;
  uart_EVENT_floor("MOVE_TO", floor);
}

// Door timing
#define DOOR_HOLD_MS 3000u
#define DOOR_CLOSE_PREMOVE_MS 300u

static uint32_t door_opened_at = 0;
static bool door_auto_close_armed = false;

static void set_doors_closed(bool closed){
  doors_closed = closed;
  uart_EVENT(closed ? "DOORS_CLOSED" : "DOORS_OPEN");
}

static void doors_open_auto(void){
  set_doors_closed(false);
  door_opened_at = uptime_ms;
  door_auto_close_armed = true;
}

static void doors_close_now(void){
  set_doors_closed(true);
  door_auto_close_armed = false;
}

static void doors_tick(void){
  if(!door_auto_close_armed) return;
  if(doors_closed) { door_auto_close_armed = false; return; }
  if((uint32_t)(uptime_ms - door_opened_at) >= DOOR_HOLD_MS){
    doors_close_now();
  }
}

static bool should_stop_at(uint8_t floor, dir_t dir){
  uint8_t m = bit_floor(floor);
  if(req_cabin & m) return true;

  // Receive floor calls only to current direction
  if(dir == DIR_UP){
    if(req_up & m) return true;
    if(floor == 3 && (req_down & m)) return true;
  } else if(dir == DIR_DOWN){
    if(req_down & m) return true;
    if(floor == 0 && (req_up & m)) return true;
  } else {
    // If moving to no direction accept all requests
    if((req_up | req_down) & m) return true;
  }
  return false;
}

static void clear_served_requests(uint8_t floor, dir_t dir){
  uint8_t m = (uint8_t)~bit_floor(floor);

  // Cabin request is always served when stopped
  req_cabin &= m;

  // Serve only one direction at a time
  if(dir == DIR_UP){
    req_up &= m;
    // Clear if top floor and stopped for DOWN
    if(floor == 3) req_down &= m;
  } else if(dir == DIR_DOWN){
    req_down &= m;
    //Clear if ground floor and stopped for UP
    if(floor == 0) req_up &= m;
  } else {
    // Idle stop clears all
    req_up &= m;
    req_down &= m;
  }
}

// Button scanning during movement
static uint8_t sF0=0, sF1=0, sF2=0, sF3=0;
static uint8_t sOpen=0, sClose=0;
static uint8_t sF0Up=0, sF1Down=0, sF1Up=0, sF2Down=0;
static uint8_t sF2Up=0, sF3Down=0;

static void scan_buttons_and_latch_requests(void){
  if(pressed_edge(&PIND, (uint8_t)(1<<BTN_DOOR_OPEN), &sOpen)){
    set_doors_closed(false);
    door_opened_at = uptime_ms;
    door_auto_close_armed = true;
  }
  if(pressed_edge(&PIND, (uint8_t)(1<<BTN_DOOR_CLOSE), &sClose)){
    doors_close_now();
  }

  if(pressed_edge(&PINB, (uint8_t)(1<<BTN_F0_INS), &sF0)){
    req_cabin |= bit_floor(0);
    announce_new_request(0, floors_deg);
  }
  if(pressed_edge(&PINB, (uint8_t)(1<<BTN_F1_INS), &sF1)){
    req_cabin |= bit_floor(1);
    announce_new_request(1, floors_deg);
  }
  if(pressed_edge(&PINB, (uint8_t)(1<<BTN_F2_INS), &sF2)){
    req_cabin |= bit_floor(2);
    announce_new_request(2, floors_deg);
  }
  if(pressed_edge(&PINB, (uint8_t)(1<<BTN_F3_INS), &sF3)){
    req_cabin |= bit_floor(3);
    announce_new_request(3, floors_deg);
  }

  if(pressed_edge(&PIND, (uint8_t)(1<<BTN_F0_UP), &sF0Up)){
    req_up |= bit_floor(0);
    announce_new_request(0, floors_deg);
  }
  if(pressed_edge(&PIND, (uint8_t)(1<<BTN_F1_DOWN), &sF1Down)){
    req_down |= bit_floor(1);
    announce_new_request(1, floors_deg);
  }
  if(pressed_edge(&PIND, (uint8_t)(1<<BTN_F1_UP), &sF1Up)){
    req_up |= bit_floor(1);
    announce_new_request(1, floors_deg);
  }
  if(pressed_edge(&PIND, (uint8_t)(1<<BTN_F2_DOWN), &sF2Down)){
    req_down |= bit_floor(2);
    announce_new_request(2, floors_deg);
  }

  if(pressed_edge(&PINC, (uint8_t)(1<<BTN_F2_UP), &sF2Up)){
    req_up |= bit_floor(2);
    announce_new_request(2, floors_deg);
  }
  if(pressed_edge(&PINC, (uint8_t)(1<<BTN_F3_DOWN), &sF3Down)){
    req_down |= bit_floor(3);
    announce_new_request(3, floors_deg);
  }
}

static bool has_requests_up_from(uint8_t floor){
  uint8_t all = (req_cabin | req_up);
  for(uint8_t f=floor+1; f<4; f++){
    if(all & bit_floor(f)) return true;
  }
  return false;
}

static bool has_requests_down_from(uint8_t floor){
  uint8_t all = (req_cabin | req_down);
  for(int8_t f=(int8_t)floor-1; f>=0; f--){
    if(all & bit_floor((uint8_t)f)) return true;
  }
  return false;
}

static void failsafe(uint8_t *curDeg, uint8_t targetDeg){
  int16_t c = *curDeg;
  int16_t t = targetDeg;
  int8_t step = (t > c) ? +1 : -1;

  while(c != t){
    c += step;
    *curDeg = (uint8_t)c;
    servo_write_angle(*curDeg);
    delay_ms(MOVE_STEP_MS);
  }
}

// count direction
static int8_t furthest_requested_in_dir(uint8_t curFloor, dir_t dir){
  if(dir == DIR_UP){
    for(int8_t f = 3; f > (int8_t)curFloor; f--){
      uint8_t m = bit_floor((uint8_t)f);
      if((req_cabin & m) || (req_up & m) || (f == 3 && (req_down & m))) return f;
    }
    return -1;
  } else {
  for(int8_t f = (int8_t)curFloor - 1; f >= 0; f--){
    uint8_t m = bit_floor((uint8_t)f);
    if((req_cabin & m) || (req_down & m) || (f == 0 && (req_up & m))) return f;
    }
    return -1;
  }
}

static bool near_floor_deg(uint8_t curDeg, uint8_t floorDeg){
  int16_t d = (int16_t)curDeg - (int16_t)floorDeg;
  if(d < 0) d = -d;
  return d <= 1;
}

static void travel_in_direction_until_no_more(
  dir_t dir,
  uint8_t *curFloor,
  uint8_t *curDeg,
  const uint8_t floors_deg[4]
) {
  if(!doors_closed){
    doors_close_now();
    for(uint16_t i = 0; i < DOOR_CLOSE_PREMOVE_MS; i += 10){
      delay_ms(10);
    }
  }

  delay_ms(DOOR_CLOSED_PAUSE_MS);

  int8_t targetFloor = furthest_requested_in_dir(*curFloor, dir);
  if(targetFloor < 0) return;

  uint8_t targetDeg = floors_deg[(uint8_t)targetFloor];
  event_move_to((uint8_t)targetFloor);

  int16_t c = *curDeg;
  int16_t t = targetDeg;
  int8_t step = (t > c) ? +MOVE_STEP_DEG : -MOVE_STEP_DEG;
  uint8_t tot=(t>c)?(t-c):(c-t);

  while(c != t){
    c += step;
    if((step > 0 && c > t) || (step < 0 && c < t)) c = t;

    *curDeg = (uint8_t)c;
    g_curDeg = *curDeg;
    servo_write_angle(*curDeg);

    for(uint8_t f=0; f<4; f++){
      if(near_floor_deg(*curDeg, floors_deg[f])){
        if(f != *curFloor){ *curFloor=f; g_curFloor=*curFloor; }
        if(should_stop_at(f, dir)){
          event_move_to(f);
          uart_EVENT_floor("ARRIVED", f);
          last_move_to=-1;
          clear_served_requests(f, dir);
          doors_open_auto();
          return;
        }
      }
    }

    uint8_t rem=(t>c)?(t-c):(c-t), prog=tot-rem, j=(prog<ACCEL_STEPS)?prog:((rem<ACCEL_STEPS)?rem:ACCEL_STEPS), k=ACCEL_STEPS-j;
    delay_ms(SERVO_MIN_DELAY + (uint16_t)k*(SERVO_MAX_DELAY-SERVO_MIN_DELAY)/ACCEL_STEPS);
  }
}

static bool floor_is_ahead(uint8_t floor, dir_t dir, const uint8_t floors_deg[4]){
  uint8_t fd = floors_deg[floor];

  if(dir == DIR_UP)   return fd > (uint8_t)(g_curDeg + 1);
  if(dir == DIR_DOWN) return fd + 1 < g_curDeg;
  return false;
}

static void announce_new_request(uint8_t floor, const uint8_t floors_deg[4]){
  if(!doors_closed) return;
  if(g_dir == DIR_IDLE) return;

  if(floor_is_ahead(floor, g_dir, floors_deg)){
    event_move_to(floor);
  }
}

static dir_t last_dir = (dir_t)255;

static void event_dir(dir_t dir){
  if(last_dir == dir) return;
  last_dir = dir;

  if(dir == DIR_UP)         uart_EVENT("DIR_UP");
  else if(dir == DIR_DOWN)  uart_EVENT("DIR_DOWN");
  else                      uart_EVENT("DIR_IDLE");
}

int main(void){
  io_init();
  servo_init();
  uart_init();
  dir_t dir = DIR_IDLE;
  uint8_t current_floor = 1;
  uint8_t current_deg   = floors_deg[1];
  g_curFloor = current_floor;
  g_curDeg   = current_deg;
  g_dir      = dir;
  event_dir(dir);
  servo_write_angle(current_deg);
  failsafe(&current_deg, floors_deg[0]);
  current_floor = 0;
  g_curFloor = current_floor;
  g_curDeg   = current_deg;
  doors_close_now();


  for(;;){
  scan_buttons_and_latch_requests();
  doors_tick();

  if(any_requests()){
    // Current floor request
    if(should_stop_at(current_floor, dir)){
      clear_served_requests(current_floor, dir);
      doors_open_auto();
    }
    else if(doors_closed){

      // Decide direction
      if(dir == DIR_IDLE){
        if(has_requests_up_from(current_floor)) {
          dir = DIR_UP; 
          g_dir = dir;
          event_dir(dir);
        }
        else if(has_requests_down_from(current_floor)) {
          dir = DIR_DOWN; 
          g_dir = dir;
          event_dir(dir);
        }
        else {
          uint8_t all = (req_cabin | req_up | req_down);

          for(uint8_t f = current_floor + 1; f < 4; f++){
            if(all & bit_floor(f)) { dir = DIR_UP; break; }
          }

          if(dir == DIR_IDLE){
            for(int8_t f = (int8_t)current_floor - 1; f >= 0; f--){
              if(all & bit_floor((uint8_t)f)) { dir = DIR_DOWN; break; }
            }
          }

          g_dir = dir;
          event_dir(dir);
        }
      }

      // Move in chosen direction
      if(dir == DIR_UP){
        if(furthest_requested_in_dir(current_floor, DIR_UP) >= 0){
          travel_in_direction_until_no_more(DIR_UP, &current_floor, &current_deg, floors_deg);
        } else if(furthest_requested_in_dir(current_floor, DIR_DOWN) >= 0){
          dir = DIR_DOWN;
          g_dir = dir;
          event_dir(dir);
        } else {
          dir = DIR_IDLE;
          g_dir = dir;
          event_dir(dir);
        }
      }
      else if(dir == DIR_DOWN){
        if(furthest_requested_in_dir(current_floor, DIR_DOWN) >= 0){
          travel_in_direction_until_no_more(DIR_DOWN, &current_floor, &current_deg, floors_deg);
        } else if(furthest_requested_in_dir(current_floor, DIR_UP) >= 0){
          dir = DIR_UP;
          g_dir = dir;
        } else {
          dir = DIR_IDLE;
          g_dir = dir;
        }
      }
    }
  }
  else {
    dir = DIR_IDLE;
    g_dir = dir;
    event_dir(dir);
  }

  delay_ms(5);
}
}