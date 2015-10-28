/* issues

- figure out what this thing is going to do
- implement it...

*/

#include <stdio.h>

// asf
#include "delay.h"
#include "compiler.h"
#include "flashc.h"
#include "preprocessor.h"
#include "print_funcs.h"
#include "intc.h"
#include "pm.h"
#include "gpio.h"
#include "spi.h"
#include "sysclk.h"

// skeleton
#include "types.h"
#include "events.h"
#include "i2c.h"
#include "init.h"
#include "interrupts.h"
#include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
#include "ftdi.h"
#include "twi.h"

// this
#include "conf_board.h"
#include "ii.h"
	
#define FIRSTRUN_KEY 0x22

#define L2 12
#define L1 7
#define L0 4

#define COLUMN_VIEW   0
#define COLUMN_SELECT 5
#define COLUMN_PAGE   6
#define COLUMN_MUTE   7
#define COLUMN_TRACK  8
#define NUM_TRACKS    8
#define PAGE_SIZE     8
#define NUM_PAGES     8
#define NUM_STEPS    64

#define FIRST_PAGE    0
#define LAST_PAGE     7

u8 key_count = 0;
// u8 edit_row, mode = 0, prev_mode = 0;
// s8 kcount = 0;
// 
// const u8 sign[8][8] = {{0,0,0,0,0,0,0,0},         // o
//        {0,24,24,126,126,24,24,0},     // +
//        {0,0,0,126,126,0,0,0},       // -
//        {0,96,96,126,126,96,96,0},     // >
//        {0,6,6,126,126,6,6,0},       // <
//        {0,102,102,24,24,102,102,0},   // * rnd
//        {0,120,120,102,102,30,30,0},   // <> up/down
//        {0,126,126,102,102,126,126,0}};  // [] return

const u8 outs[8] = {B00, B01, B02, B03, B04, B05, B06, B07};

// basic structure
//
// 8 tracks
// 64 steps per track (arranged as 8 groups of 8 steps)
// 8 sub positions per step (bits in a u8)
//
// - 8 mutes (one per track)
// - 1 play head (could this be expanded to one per track?)
//

typedef enum {
  eViewVertical = 0,
  eView8x8,
  eView4x16,
  eView2x32,
  eView1x64
} view_mode_t;

typedef struct {
  u8 trigs[NUM_TRACKS][NUM_STEPS];  // 8 tracks, 8 groups * 8 steps
  u8 flags[NUM_TRACKS];      // bitfield containing track level mutes, freezes, etc
  u8 selected_page, in_page, out_page;
} hl_set_t;

typedef const struct {
  u8 fresh;
  u8 preset_select;
  u8 glyph[8][8];
  hl_set_t sets[8];
} nvram_data_t;

hl_set_t s;
view_mode_t view = eView8x8;

u8 selected_track = 0;
bool follow = true;           // selected follows play head

u8 preset_mode, preset_select, front_timer;
u8 glyph[8];

volatile u8 position = 0;  // playhead position: [0,63]
volatile u8 page = 0;      // position / 8: [0,7]
volatile u8 step = 0;      // position % 8; [0,7]
volatile u8 tick = 0;

volatile u16 outputs[8] = {0, 0, 0, 0, 0, 0, 0, 0};

u8 position_queue = 0;
bool position_queued = false;

u8 clock_phase;
u16 clock_time, clock_temp;

u16 adc[4];
u8 SIZE, LENGTH, VARI;

u8 held_keys[32], key_times[256];

// TODO: refresh functions pointer? give better name
typedef void(*re_t)(void);
re_t re;
re_t refresh_trig;
re_t refresh_ctrl;

typedef void(*kph_t)(u8 x, u8 y, u8 z);
kph_t handle_trig;
kph_t handle_ctrl;


__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;




////////////////////////////////////////////////////////////////////////////////
// prototypes

static void refresh(void);
static void refresh_mono(void);
static void refresh_preset(void);

static void refresh_trig_8x8(void);
static void refresh_trig_4x16(void);
static void refresh_trig_2x32(void);
static void refresh_trig_1x64(void);
static void refresh_trig_vert(void);
static void refresh_trig_clear(void);

static void refresh_ctrl_home(void);
static void refresh_select_control(u8 column, u8 seleted, view_mode_t view);
static void refresh_radio_column(u8 column, u8 low, u8 high, u8 selected);

static void clock(u8 phase);

static void position_set(u8 p);
static void position_advance(s8 step);

static void hl_set_clear(hl_set_t *s);

static void trig_toggle(u8 track, u8 page, u8 step);
static void trig_clear(u8 track, u8 page, u8 step);

static void view_set(view_mode_t view);
static u8 view_page_count(view_mode_t view);
static u8 view_track_count(view_mode_t view);
static u8 view_steps(view_mode_t view);
static void view_pages(view_mode_t view, u8 focus_page, u8 *low_page, u8 *high_page);
static void view_tracks(view_mode_t view, u8 focus_track, u8 *low_track, u8 *high_track);

static void track_mute_set(u8 track, bool mute);
static void track_mute_toggle(u8 track);
static bool track_mute_enabled(u8 track);

// start/stop monome polling/refresh timers
extern void timers_set_monome(void);
extern void timers_unset_monome(void);

// check the event queue
static void check_events(void);

// handler protos
static void handler_None(s32 data) { ;; }
static void handler_KeyTimer(s32 data);
static void handler_Front(s32 data);
static void handler_ClockNormal(s32 data);

// application handler
static void handle_null_press(u8 x, u8 y, u8 z);

static void handle_trig_press_8x8(u8 x, u8 y, u8 z);
static void handle_trig_press_4x16_or_2x32(u8 x, u8 y, u8 z);
static void handle_trig_press_1x64(u8 x, u8 y, u8 z);
static void handle_trig_press_vert(u8 x, u8 y, u8 z);

static void handle_ctrl_home(u8 x, u8 y, u8 z);

u8 flash_is_fresh(void);
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);

static void hilo_process_ii(uint8_t i, int d);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application clock code

void clock(u8 phase, u8 ticks) {
  u8 v;
  
  if (ticks == 0) {
    gpio_set_gpio_pin(B10);

    if (position_queued) {
      position_set(position_queue);
      position_queued = false;
    }
    else {
      position_advance(1);
    }
    monomeFrameDirty++;
    //monome_set_quadrant_flag(1);
    //monome_set_quadrant_flag(0);
  }

  // STOPPED HERE: TODO:
  // manual triggers should have priority
  // outputs are tick counters which are set by triggers (or manual
  // trigs)
  // they should be high if outputs[t] > 0 and low otherwise
  // need to figure out how to represent gate versus trigger
  // need to figure out how to deal with shifts within a step
  // ..just realized that the ticks notion probably is broken w/
  // external clocks. do we have to implement a clock multiplier?
		
  for (u8 t = 0; t < NUM_TRACKS; t++) {
    if (outputs[t] > 0) {
      gpio_set_gpio_pin(out[t]);
      outputs[t]--;
    }
    else if (!track_mute_enabled(t)) {
      outputs[t] = s.trigs[t][position];
      if 
      outputs[t] = 0; // for UI feedback
      gpio_set_gpio_pin(outs[t]);
    }
  }

  if (ticks == 4) {
    gpio_clr_gpio_pin(B10);
    // low 
    for (u8 i = 0; i < 8; i++) {
      gpio_clr_gpio_pin(outs[i]);
    }

  }
}

static void position_set(u8 p) {
  position = p % NUM_STEPS;
  page = position / PAGE_SIZE;
  step = position % PAGE_SIZE;
  if (follow) s.selected_page = page;
  //print_dbg("\r\nposition (set): ");
  //print_dbg_ulong(position);
}

static void position_advance(s8 delta) {
  u8 new_pos = (position + delta) % NUM_STEPS;
  u8 out_pos = (s.out_page + 1) * PAGE_SIZE; // range [8, 64]
  u8 in_pos;
  if (new_pos >= out_pos) {  // check this; off by one?
    in_pos = s.in_page * PAGE_SIZE;
    position_set(in_pos + (new_pos - out_pos));
  }
  else {
    position_set(new_pos);
  }
  // TODO: handle backwards looping
}

static void hl_set_clear(hl_set_t *s) {
  u8 i, st;
  for (i = 0; i < NUM_TRACKS; i++) {
    s->flags[i] = 0;
    for (st = 0; st < NUM_STEPS; st++) {
      s->trigs[i][st] = 0;
    }
  }
  s->selected_page = FIRST_PAGE;
  s->in_page = FIRST_PAGE;
  s->out_page = LAST_PAGE;
}

inline static void trig_toggle(u8 track, u8 page, u8 step) {
  print_dbg("\r\n trig_toggle;");
  print_dbg(" tk: "); 
  print_dbg_ulong(track); 
  print_dbg(" pg: "); 
  print_dbg_ulong(page); 
  print_dbg(" st: ");
  print_dbg_ulong(step);
  u8 v = s.trigs[track][page * PAGE_SIZE + step] ^= 1; 
  print_dbg(" v: ");
  print_dbg_ulong(v);
}

inline static void trig_clear(u8 track, u8 page, u8 step) {
  s.trigs[track][page * PAGE_SIZE + step] = 0;
}

static void track_mute_set(u8 track, bool mute) {
  if (mute)
    s.flags[track] |= 0b0001;
  else
    s.flags[track] &= 0b1110;
}

static void track_mute_toggle(u8 track) {
  track_mute_set(track, !track_mute_enabled(track));
}

static bool track_mute_enabled(u8 track) {
  return s.flags[track] & 0b0001;
}


////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer  = { .next = NULL, .prev = NULL };



static void clockTimer_callback(void* o) {  
  if (clock_external == 0) {
    clock_phase = 1;
    ticks = (ticks + 1) % 8;
    if (ticks > 3) clock_phase = 0;
    (*clock_pulse)(clock_phase, ticks);
  }
}

static void keyTimer_callback(void* o) {  
  static event_t e;
  e.type = kEventKeyTimer;
  e.data = 0;
  event_post(&e);
}

static void adcTimer_callback(void* o) {  
  static event_t e;
  e.type = kEventPollADC;
  e.data = 0;
  event_post(&e);
}


// monome polling callback
static void monome_poll_timer_callback(void* obj) {
  // asynchronous, non-blocking read
  // UHC callback spawns appropriate events
  ftdi_read();
}

// monome refresh callback
static void monome_refresh_timer_callback(void* obj) {
  if (monomeFrameDirty > 0) {
    static event_t e;
    e.type = kEventMonomeRefresh;
    event_post(&e);
  }
}

// monome: start polling
void timers_set_monome(void) {
  timer_add(&monomePollTimer, 20, &monome_poll_timer_callback, NULL);
  timer_add(&monomeRefreshTimer, 30, &monome_refresh_timer_callback, NULL);
}

// monome stop polling
void timers_unset_monome(void) {
  timer_remove(&monomePollTimer);
  timer_remove(&monomeRefreshTimer); 
}



////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) {
  ftdi_setup(); 
}

static void handler_FtdiDisconnect(s32 data) { 
  timers_unset_monome();
}

static void handler_MonomeConnect(s32 data) {
  print_dbg("\r\n// monome connect /////////////////"); 
  key_count = 0;
  SIZE = monome_size_x();
  LENGTH = SIZE - 1;
  print_dbg("\r monome size: ");
  print_dbg_ulong(SIZE);
  VARI = monome_is_vari();
  print_dbg("\r monome vari: ");
  print_dbg_ulong(VARI);

  // TODO: remove non-varibright support?
  if (VARI) re = &refresh;
  else re = &refresh_mono;

  timers_set_monome();
}

static void handler_MonomePoll(s32 data) {
  monome_read_serial();
}

static void handler_MonomeRefresh(s32 data) {
  if (monomeFrameDirty) {
    if (preset_mode == 0) (*re)();
    else refresh_preset();
		
    (*monome_refresh)();
  }
}


static void handler_Front(s32 data) {
  print_dbg("\r\n FRONT HOLD");

  if (data == 0) {
    front_timer = 15;
    if (preset_mode) preset_mode = 0;
    else preset_mode = 1;
  }
  else {
    front_timer = 0;
  }

  monomeFrameDirty++;
}

static void handler_PollADC(s32 data) {
  u16 i;
  adc_convert(&adc);

  // CLOCK POT INPUT
  i = adc[0];
  i = i>>2;
  if (i != clock_temp) {
    // 500ms - 12ms
    clock_time = 12500 / (i + 25);
    // FIXME: there seems to be a decent amount of noise from the front pot
    // so the timer interval is being changed possibly introducing some
    // jitter. Should we filter?
    //print_dbg("\r\n clock (ms): ");
    //print_dbg_ulong(clock_time);

    timer_set(&clockTimer, clock_time);
  }
  clock_temp = i;
}

static void handler_SaveFlash(s32 data) {
  flash_write();
}

static void handler_KeyTimer(s32 data) {
  static u16 i1;

  if (front_timer) {
    if (front_timer == 1) {
      static event_t e;
      e.type = kEventSaveFlash;
      event_post(&e);

      preset_mode = 0;
      front_timer--;
    }
    else front_timer--;
  }

  for (i1 = 0; i1 < key_count; i1++) {
    if (key_times[held_keys[i1]]) {
      if (--key_times[held_keys[i1]] == 0) {
	if (preset_mode == 1) {
	  if (held_keys[i1] % 16 == 0) {
	    preset_select = held_keys[i1] / 16;
	    static event_t e;
	    e.type = kEventSaveFlash;
	    event_post(&e);
	    preset_mode = 0;
	  }
	}

	print_dbg("\r long press: "); 
	print_dbg_ulong(held_keys[i1]);
      }
    }
  }
}

static void handler_ClockNormal(s32 data) {
  clock_external = !gpio_get_pin_value(B09); 
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application grid code

static void handler_MonomeGridKey(s32 data) { 
  u8 x, y, z, index, i1, found;
  monome_grid_key_parse_event_data(data, &x, &y, &z);
  print_dbg("\r\n monome event; x: "); 
  print_dbg_ulong(x); 
  print_dbg("; y: "); 
  print_dbg_ulong(y); 
  print_dbg("; z: 0x"); 
  print_dbg_hex(z);

  //// TRACK LONG PRESSES
  index = y*16 + x;
  if (z) {
    held_keys[key_count] = index;
    key_count++;
    key_times[index] = 10;		//// THRESHOLD key hold time
  } else {
    found = 0; // "found"
    for(i1 = 0; i1<key_count; i1++) {
      if(held_keys[i1] == index) 
	found++;
      if(found) 
	held_keys[i1] = held_keys[i1+1];
    }
    key_count--;

    // FAST PRESS
    if (key_times[index] > 0) {
      if (preset_mode == 1) {
	if (x == 0 && y != preset_select) {
	  preset_select = y;
	  for (i1 = 0; i1 < 8; i1++)
	    glyph[i1] = flashy.glyph[preset_select][i1];
	}
	else if (x == 0 && y == preset_select) {
	  flash_read();
	  preset_mode = 0;
	}
	monomeFrameDirty++;	
      }
      // print_dbg("\r\nfast press: ");
      // print_dbg_ulong(index);
      // print_dbg(": ");
      // print_dbg_ulong(key_times[index]);
    }
  }
	
  // PRESET SCREEN
  if (preset_mode) {
    // glyph magic
    if (z && x > 7) {
      glyph[y] ^= 1<<(x-8);
    }
    monomeFrameDirty++;	
  }
  else {
    if (x > COLUMN_MUTE) {
      (*handle_trig)(x - (COLUMN_MUTE + 1), y, z); // position in first quad
    }
    else if (x == COLUMN_MUTE) {
      if (z) {
	track_mute_toggle(y);
	print_dbg("\r\n mute toggle: ");
	print_dbg_ulong(y);
      }
      monomeFrameDirty++;
    }
    else if (x == COLUMN_PAGE) {
      static bool selection_held = false;
      if (z) {
	print_dbg("\r\n page press; ");
	if (selection_held && y != s.selected_page) {
	  s.in_page = s.selected_page;
	  s.out_page = y;
	  selection_held = false;
	  follow = true;
	  print_dbg(" loop in: ");
	  print_dbg_ulong(s.in_page);
	  print_dbg(" out: ");
	  print_dbg_ulong(s.out_page);
	}
	else if (y == s.selected_page) {
	  // re-pressed same page, return to follow
	  selection_held = false;
	  follow = true;
	  print_dbg(" already selected; clear hold => follow");
	}
	else {
	  s.selected_page = y;
	  selection_held = true;
	  follow = false;
	  print_dbg( "selecting: ");
	  print_dbg_ulong(s.selected_page);
	}
	monomeFrameDirty++;
      }
      else if (y == s.selected_page) {
	print_dbg("\r\n page lift; clear hold");
	selection_held = false; // lifted on same key_count
      }
    }
    else {
      //print_dbg("\r\n ctrl press");
      (*handle_ctrl)(x, y, z);
    }
  } // not preset mode
}

static void handle_null_press(u8 x, u8 y, u8 z) {}

static void handle_trig_press_8x8(u8 x, u8 y, u8 z) {	
  if (z) {
    trig_toggle(y, s.selected_page, x);
    monomeFrameDirty++;
  }
}

static void handle_trig_press_4x16_or_2x32(u8 x, u8 y, u8 z) {
  u8 track_low, track_high, page_low, page_high;
  u8 page_count, track, page;
  
  if (z) {
    // FIME: this works but feels expensive
    view_tracks(view, selected_track, &track_low, &track_high);
    view_pages(view, s.selected_page, &page_low, &page_high);
    page_count = view_page_count(view);
    track = track_low + (y / page_count);
    page = page_low + (y % page_count);
    trig_toggle(track, page, x);
    monomeFrameDirty++;
  }
}

static void handle_trig_press_1x64(u8 x, u8 y, u8 z) {	
  if (z) {
    trig_toggle(selected_track, y, x);
    monomeFrameDirty++;
  }
}

static void handle_trig_press_vert(u8 x, u8 y, u8 z) {	
  //print_dbg("\r\n handle_trig_press_vert");
  if (z && x == 0) {
    trig_toggle(selected_track, s.selected_page, y);
    monomeFrameDirty++;
  }
}

static void handle_ctrl_home(u8 x, u8 y, u8 z) {
  if (x == 0) {
    if (y < 5 && z) {
      // view buttons
      view = (view_mode_t)y;
      view_set(view);
      print_dbg("\r\n view_select: ");
      print_dbg_ulong(view);
      monomeFrameDirty++;
    }
  }
  else if (x == 1) {
    if (y == 0 && z) {
      // reset
      position_queue = s.in_page * PAGE_SIZE;
      position_queued = true;
      monomeFrameDirty++;
    }
  }		   
  else if (x == COLUMN_SELECT) {
    if (z) {
      selected_track = y;
      print_dbg("\r\n track_select: ");
      print_dbg_ulong(selected_track);
      monomeFrameDirty++;
    }
  }
  else if (y == 6) {
    if (x < 4 && z) {
      outputs[x] = 1;
      monomeFrameDirty++;
    }
  }
  else if (y == 7) {
    if (x < 4 && z) {
      outputs[x + 4] = 1;
      monomeFrameDirty++;
    }
  }
    
}

////////////////////////////////////////////////////////////////////////////////
// application grid redraw

// returns the number of visible pages per view mode
static u8 view_page_count(view_mode_t view) {
  switch (view) {
  case eView8x8:
    return 1;
  case eView4x16:
    return 2;
  case eView2x32:
    return 4;
  case eView1x64:
    return 8;
  case eViewVertical:
    return 1;
  }
  print_dbg("\r\n view_page_count; unhandled view value: ");
  print_dbg_ulong(view);
  return 0;
}

static u8 view_track_count(view_mode_t view) {
  switch (view) {
  case eView8x8:
    return 8;
  case eView4x16:
    return 4;
  case eView2x32:
    return 2;
  case eView1x64:
    return 1;
  case eViewVertical:
    return 1;
  }
  print_dbg("\r\n view_track_count; unhandled view value: ");
  print_dbg_ulong(view);
  return 0;
}

inline static u8 view_steps(view_mode_t view) {
  return view_page_count(view) * PAGE_SIZE;
}

static void view_pages(view_mode_t view, u8 focus_page, u8 *low_page, u8 *high_page) {
  u8 size = view_page_count(view);
  *low_page = (focus_page / size) * size;
  *high_page = *low_page + size;
}

static void view_tracks(view_mode_t view, u8 focus_track, u8 *low_track, u8 *high_track) {
  u8 size = view_track_count(view);
  *low_track = (focus_track / size) * size;
  *high_track = *low_track + size;
}

static void refresh(void) {
  (*refresh_trig)();
  (*refresh_ctrl)();
}

static void refresh_null(void) {}

static void refresh_trig_clear(void) {
  // TODO: do this with fewer ops
  for (u8 x = 8; x < 16; x++)
    for (u8 y = 0; y < 8; y++)
      monomeLedBuffer[x + y] = 0;
}

static void refresh_trig_8x8(void) {
  u8 i, t, in;
  u32 led;
  // scrolling view; pages of 8...
  for (t = 0; t < NUM_TRACKS; t++) {
    //tart = t * 16 + COLUMN_TRACK;
    // draw the trigs for one track
    in = s.selected_page * PAGE_SIZE; // starting offset into trigs
    for (i = 0; i < PAGE_SIZE; i++) {
      led = monome_xy_idx(COLUMN_TRACK + i, t);
      if (s.trigs[t][in + i])
	monomeLedBuffer[led] = L1;
      else
	monomeLedBuffer[led] = 0;
    }		

    if (page == s.selected_page) {
      // draw the playhead for one track
      monome_led_set(COLUMN_TRACK + step, t, L2);
    }	
  }
}

static void refresh_trig_4x16(void) {
  u8 i, t, start_row, in, out, v;
  
  // figure out which tracks are visible
  u8 track_low, track_high;
  view_tracks(view, selected_track, &track_low, &track_high);
  
  // figure out in and out steps (visible)
  u8 in_page, out_page;
  view_pages(view, s.selected_page, &in_page, &out_page);
  
  in = in_page * PAGE_SIZE;
  out = out_page * PAGE_SIZE;

  bool playhead_visible = in_page <= page && page < out_page;
  
  for (t = track_low; t < track_high; t++) {
    start_row = 2 * (t % 4);
    for (i = in; i < out; i++) {
      v = s.trigs[t][i] ? L1 : 0;
      monome_led_set(COLUMN_TRACK + (i % 8), start_row + ((i % 16) / 8), v);
    }
    // playhead
    if (playhead_visible) {
      monome_led_set(COLUMN_TRACK + step, start_row + (page % 2), L2);
    }
  }
}

static void refresh_trig_2x32(void) {	
  u8 i, t, start_row, in, out, v;
	
  // figure out which tracks are visible
  u8 track_low, track_high;
  view_tracks(view, selected_track, &track_low, &track_high);
 
  // figure out in and out positions (visible)
  u8 in_page, out_page;
  view_pages(view, s.selected_page, &in_page, &out_page);
  
  in = in_page * PAGE_SIZE;
  out = out_page * PAGE_SIZE;

  /* print_dbg("\r\n rt2x32; pin:"); */
  /* print_dbg_ulong(in_page); */
  /* print_dbg(" pout: "); */
  /* print_dbg_ulong(out_page); */
  /* print_dbg(" sin: "); */
  /* print_dbg_ulong(in); */
  /* print_dbg(" sout: "); */
  /* print_dbg_ulong(out); */
  
  bool playhead_visible = in_page <= page && page < out_page;

  for (t = track_low; t < track_high; t++) {
    start_row = 4 * (t % 2);
    for (i = in; i < out; i++) {
      v = s.trigs[t][i] ? L1 : 0;
      monome_led_set(COLUMN_TRACK + (i % 8), start_row + ((i % 32) / 8), v);
    }

    // playhead
    if (playhead_visible) {
      monome_led_set(COLUMN_TRACK + step, start_row + (page % 4), L2);
    }
  }
}

static void refresh_trig_1x64(void) {
  // triggers
  for (u8 y = 0; y < NUM_PAGES; y++) {
    for (u8 x = 0; x < PAGE_SIZE; x++) {
      u8 i = y * PAGE_SIZE + x;
      if (s.trigs[selected_track][i])
	monome_led_set(COLUMN_TRACK + x, y, L1);
      else
	monome_led_set(COLUMN_TRACK + x, y, 0);
    }
  }
	
  // playhead
  monome_led_set(COLUMN_TRACK + step, page, L2);
}

static void refresh_trig_vert(void) {
  u8 page_low, page_high, in;

  refresh_trig_clear(); // since we only draw first column
  
  view_pages(view, s.selected_page, &page_low, &page_high);
  in = page_low * PAGE_SIZE;

  for (u8 i = 0; i < PAGE_SIZE; i++) {
    if (s.trigs[selected_track][in + i])
      monome_led_set(COLUMN_TRACK, i, L1);
    else
      monome_led_set(COLUMN_TRACK, i, 0);
  }

  // draw the playhead
  if (page == s.selected_page) {
    monome_led_set(COLUMN_TRACK, step, L2);
  }	
}

static void view_set(view_mode_t view) {
  switch (view) {
  case eViewVertical:
    refresh_trig = &refresh_trig_vert;
    handle_trig = &handle_trig_press_vert;
    break;
  case eView8x8:
    refresh_trig = &refresh_trig_8x8;
    handle_trig = &handle_trig_press_8x8;
    break;
  case eView4x16:
    refresh_trig = &refresh_trig_4x16;
    handle_trig = &handle_trig_press_4x16_or_2x32;
    break;
  case eView2x32:
    refresh_trig = &refresh_trig_2x32;
    handle_trig = &handle_trig_press_4x16_or_2x32;
    break;
  case eView1x64:
    refresh_trig = &refresh_trig_1x64;
    handle_trig = &handle_trig_press_1x64;
    break;
  default:
    refresh_trig = &refresh_trig_8x8;
    handle_trig = &handle_trig_press_8x8;
  }
}

static void refresh_select_control(u8 column, u8 selected, view_mode_t view) {
  u8 i, start, stop;
  switch (view) {
  case eView2x32:
    start = (selected >> 1) * 2;
    stop = start + 2;
    break;
  case eView4x16:
    start = (selected >> 2) * 4;
    stop = start + 4;
    break;
  default: // eView1x64 || eViewVertical || eView8x8
    start = stop = 0;
    break;
  }
	
  // clear
  for (i = 0; i < NUM_TRACKS; i++) {
    monome_led_set(column, i, 0);
  }
		
  // selection range
  for (i = start; i < stop; i++) {
    monome_led_set(column, i, L0);
  }
		
  // selection
  monome_led_set(column, selected, L2);
}


static void refresh_ctrl_home(void) {
  u8 v, p;
	
  // mutes column
  for (u8 m = 0; m < NUM_TRACKS; m++) {
    v = track_mute_enabled(m) ? 0 : L0;
    monome_led_set(COLUMN_MUTE, m, v); // TODO: implement flags
  }
	
  // handle page control
  for (p = 0; p < NUM_PAGES; p++) {
    monome_led_set(COLUMN_PAGE, p, 0);
  }
  
  // show the loop points	
  if (s.in_page != FIRST_PAGE || s.out_page != LAST_PAGE) {
    for (p = s.in_page; p <= s.out_page; p++) {
      monome_led_set(COLUMN_PAGE, p, L0);
    }
  }
  monome_led_set(COLUMN_PAGE, page, L1);
  monome_led_set(COLUMN_PAGE, s.selected_page, L2);
	
  // handle track select control
  refresh_select_control(COLUMN_SELECT, selected_track, view);

  // handle view control
  refresh_radio_column(COLUMN_VIEW, 0, 4, view);

  // handle position queued (aka reset)
  monome_led_set(1, 0, position_queued ? L0 : 0);

  // handle manual triggers
  for (u8 i = 0; i < NUM_TRACKS; i++) {
    monome_led_set(i % 4, 6 + (i >> 2), outputs[i] ? L1 : 0);
  }
}

static void refresh_radio_column(u8 column, u8 low, u8 high, u8 selected) {
  for (u8 i = low; i <= high; i++) {
    if (i == selected)
      monome_led_set(column, i, L2);
    else
      monome_led_set(column, i, 0);
  }
}

// application grid redraw without varibright
static void refresh_mono(void) {
  // TODO: this isn't going to work, fix or remove support
  refresh();
}


static void refresh_preset() {
  u8 i1,i2;
	
  for (i1 = 0; i1 < 128; i1++) {
    monomeLedBuffer[i1] = 0;
  }
	
  monomeLedBuffer[preset_select * 16] = 11;
	
  for (i1 = 0; i1 < 8; i1++) {
    for (i2 = 0; i2 < 8; i2++) {
      if (glyph[i1] & (1 << i2)) {
	monomeLedBuffer[i1 * 16 + i2 + 8] = 11;
      }
    }
  }
	
  monome_set_quadrant_flag(0);
  monome_set_quadrant_flag(1);
}



static void hilo_process_ii(uint8_t i, int d) {
  // switch(i) {
  // 	case MP_PRESET:
  // 		if(d<0 || d>7)
  // 			break;
  // 		preset_select = d;
  // 		flash_read();
  // 		break;
  // 	case MP_RESET:
  // 		if(d) {
  // 			m.positions[0] = m.points[0]+1;
  // 			for(int n=1;n<8;n++)
  // 				m.positions[n] = m.points[n];
  // 		}
  // 		break;
  // 	case MP_SYNC:
  // 		if(d) {
  // 			m.positions[0] = m.points[0]+1;
  // 			for(int n=1;n<8;n++)
  // 				m.positions[n] = m.points[n];
  // 			timer_set(&clockTimer,clock_time);
  // 			clock_phase = 1;
  // 			(*clock_pulse)(clock_phase);
  // 		}
  // 		break;
  // 	case MP_MUTE:
  // 		if(d<1 || d>8)
  // 			break;
  // 		d--;
  // 		m.mutes[d] = 1;
  // 		break;
  // 	case MP_UNMUTE:
  // 		if(d<1 || d>8)
  // 			break;
  // 		d--;
  // 		m.mutes[d] = 0;
  // 		break;
  // 	case MP_FREEZE:
  // 		if(d<1 || d>8)
  // 			break;
  // 		d--;
  // 		m.freezes[d] = 1;
  // 		break;
  // 	case MP_UNFREEZE:
  // 		if(d<1 || d>8)
  // 			break;
  // 		d--;
  // 		m.freezes[d] = 0;
  // 		break;
  // 	default:
  // 		break;
  // }
  // print_dbg("\r\nmp: ");
  // print_dbg_ulong(i);
  // print_dbg(" ");
  // print_dbg_ulong(d);
}


// assign event handlers
static inline void assign_main_event_handlers(void) {
  app_event_handlers[kEventFront]	        = &handler_Front;
  // app_event_handlers[kEventTimer]               = &handler_Timer;
  app_event_handlers[kEventPollADC]	        = &handler_PollADC;
  app_event_handlers[kEventKeyTimer]            = &handler_KeyTimer;
  app_event_handlers[kEventSaveFlash]           = &handler_SaveFlash;
  app_event_handlers[kEventClockNormal]         = &handler_ClockNormal;
  app_event_handlers[kEventFtdiConnect]	        = &handler_FtdiConnect ;
  app_event_handlers[kEventFtdiDisconnect]      = &handler_FtdiDisconnect ;
  app_event_handlers[kEventMonomeConnect]       = &handler_MonomeConnect ;
  app_event_handlers[kEventMonomeDisconnect]	= &handler_None ;
  app_event_handlers[kEventMonomePoll]          = &handler_MonomePoll ;
  app_event_handlers[kEventMonomeRefresh]	= &handler_MonomeRefresh ;
  app_event_handlers[kEventMonomeGridKey]	= &handler_MonomeGridKey ;
}

// app event loop
void check_events(void) {
  static event_t e;
  if (event_next(&e)) {
    (app_event_handlers)[e.type](e.data);
  }
}

// flash commands
u8 flash_is_fresh(void) {
  return (flashy.fresh != FIRSTRUN_KEY);
}

// write fresh status
void flash_unfresh(void) {
  flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
}

void flash_write(void) {
  print_dbg("\r\n write preset: ");
  print_dbg_ulong(preset_select);
  flashc_memcpy((void *)&flashy.sets[preset_select], &s, sizeof(s), true);
  flashc_memcpy((void *)&flashy.glyph[preset_select], &glyph, sizeof(glyph), true);
  flashc_memset8((void*)&(flashy.preset_select), preset_select, 1, true);
}

void flash_read(void) {
  print_dbg("\r\n read preset ");
  print_dbg_ulong(preset_select);
	
  // TODO: reset position, page, and step????
	
  for (u8 t = 0; t < NUM_TRACKS; t++) {
    for (u8 i = 0; i < NUM_STEPS; i++) {
      s.trigs[t][i] = flashy.sets[preset_select].trigs[t][i];
    }
    s.flags[t] = flashy.sets[preset_select].flags[t];
  }
  s.selected_page = flashy.sets[preset_select].selected_page;
  s.in_page = flashy.sets[preset_select].in_page;
  s.out_page = flashy.sets[preset_select].out_page;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// main

int main(void) {
  u8 i1;

  sysclk_init();

  init_dbg_rs232(FMCK_HZ);

  init_gpio();
  assign_main_event_handlers();
  init_events();
  init_tc();
  init_spi();
  init_adc();

  irq_initialize_vectors();
  register_interrupts();
  cpu_irq_enable();

  init_usb_host();
  init_monome();

  init_i2c_slave(0x30);

  print_dbg("\r\n\n// hilo //////////////////////////////// ");
  print_dbg_ulong(sizeof(flashy));

  print_dbg(" ");
  print_dbg_ulong(sizeof(s));

  if (flash_is_fresh()) {
    print_dbg("\r\nfirst run.");
    flash_unfresh();
    flashc_memset32((void*)&(flashy.preset_select), 0, 4, true);

    // clear out the active set
    hl_set_clear(&s);
		
    // init all presets to current set
    for (i1 = 0; i1 < 8; i1++) {
      flashc_memcpy((void *)&flashy.sets[i1], &s, sizeof(s), true);
      glyph[i1] = (1<<i1);
      flashc_memcpy((void *)&flashy.glyph[i1], &glyph, sizeof(glyph), true);
    }
  }
  else {
    // load from flash at startup
    preset_select = flashy.preset_select;
    flash_read();
    for (i1 = 0; i1 < 8; i1++)
      glyph[i1] = flashy.glyph[preset_select][i1];
  }

  LENGTH = 15;
  SIZE = 16;

  view_set(eView8x8);

  handle_ctrl = &handle_ctrl_home;
  refresh_ctrl = &refresh_ctrl_home;

  re = &refresh;

  process_ii = &hilo_process_ii;

  clock_pulse = &clock;
  clock_external = !gpio_get_pin_value(B09);

  timer_add(&clockTimer, 15, &clockTimer_callback, NULL);
  timer_add(&keyTimer, 50, &keyTimer_callback, NULL);
  timer_add(&adcTimer, 100, &adcTimer_callback, NULL);
  clock_temp = 10000; // out of ADC range to force tempo
	
  s.selected_page = FIRST_PAGE;
  follow = true;

  while(true) {
    check_events();
  }
}
	
