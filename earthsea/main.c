// B00 is EDGE


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
#include "twi.h"

// skeleton
#include "types.h"
#include "events.h"
#include "init.h"
#include "interrupts.h"
#include "monome.h"
#include "timers.h"
#include "adc.h"
#include "util.h"
#include "ftdi.h"
#include "i2c.h"
#include "midi.h"
#include "notes.h"

// this
#include "conf_board.h"
#include "ii.h"
	

#define FIRSTRUN_KEY 0x22

#define SHAPE_COUNT 5
#define POT_HYSTERESIS 48
#define EVENTS_PER_PATTERN 128
#define SLEW_CV_OFF_THRESH 4000

#define MIDI_NOTE_MAX 120

const u16 SHAPE_PATTERN[16] = {256, 288, 160, 384, 272, 292, 84, 448, 273, 432, 325, 168, 336, 276, 162};
const u8 SHAPE_OFF_Y[9] = {0, 1, 1, 0, 0, 2, 2, 0, 0};

const u8 EDGE_GLYPH[3][4] = {{7,5,5,13}, {15,9,9,9}, {15,0,0,0} };

// step = 4096.0 / (10 ocatave * 12.0 semitones per octave)
// [int(n * step) for n in xrange(0,128)]
const u16 SEMI[128] = { 
	0, 34, 68, 102, 136, 170, 204, 238, 273, 307, 341, 375, 409, 443, 477, 512,
	546, 580, 614, 648, 682, 716, 750, 785, 819, 853, 887, 921, 955, 989, 1024,
	1058, 1092, 1126, 1160, 1194, 1228, 1262, 1297, 1331, 1365, 1399, 1433, 1467,
	1501, 1536, 1570, 1604, 1638, 1672, 1706, 1740, 1774, 1809, 1843, 1877, 1911,
	1945, 1979, 2013, 2048, 2082, 2116, 2150, 2184, 2218, 2252, 2286, 2321, 2355,
	2389, 2423, 2457, 2491, 2525, 2560, 2594, 2628, 2662, 2696, 2730, 2764, 2798,
	2833, 2867, 2901, 2935, 2969, 3003, 3037, 3072, 3106, 3140, 3174, 3208, 3242,
	3276, 3310, 3345, 3379, 3413, 3447, 3481, 3515, 3549, 3584, 3618, 3652, 3686,
	3720, 3754, 3788, 3822, 3857, 3891, 3925, 3959, 3993, 4027, 4061, 4096, 4130,
	4164, 4198, 4232, 4266, 4300, 4334
};

const u16 EXP[256] = {0, 0, 0, 1, 2, 2, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22, 24, 25, 27, 29, 31,
	33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 54, 56, 58, 60, 63, 65, 68, 70, 72, 75, 77, 80, 83, 85, 88, 91, 93,
	96, 99, 101, 104, 107, 110, 113, 116, 119, 122, 125, 128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 159, 162,
	165, 168, 172, 175, 178, 182, 185, 189, 192, 195, 199, 202, 206, 209, 213, 217, 220, 224, 227, 231, 235, 238,
	242, 246, 250, 253, 257, 261, 265, 268, 272, 276, 280, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328,
	332, 336, 341, 345, 349, 353, 357, 362, 366, 370, 374, 379, 383, 387, 392, 396, 400, 405, 409, 414, 418, 423, 427,
	432, 436, 441, 445, 450, 454, 459, 463, 468, 473, 477, 482, 487, 491, 496, 501, 505, 510, 515, 520, 525, 529, 534,
	539, 544, 549, 554, 559, 563, 568, 573, 578, 583, 588, 593, 598, 603, 608, 613, 618, 623, 629, 634, 639, 644, 649,
	654, 659, 665, 670, 675, 680, 686, 691, 696, 701, 707, 712, 717, 723, 728, 733, 739, 744, 749, 755, 760, 766, 771,
	777, 782, 788, 793, 799, 804, 810, 815, 821, 826, 832, 838, 843, 849, 855, 860, 866, 872, 877, 883, 889, 894, 900,
	906, 912, 917, 923, 929, 935, 941, 946, 952, 958, 964, 970, 976, 982, 988, 994, 1000, 1006, 1012};
	
// step = 0.9 / 128
// [int((math.log10(0.1 + (i * step)) + 1) * 4096) for i in xrange(0,128)]
const u16 LOG[128] = { 
	0, 120, 234, 340, 440, 535, 626, 711, 793, 872, 947, 1019, 1088, 1154, 1219,
	1281, 1340, 1398, 1454, 1509, 1561, 1613, 1663, 1711, 1758, 1804, 1849, 1893,
	1935, 1977, 2017, 2057, 2096, 2134, 2172, 2208, 2244, 2279, 2313, 2347, 2380,
	2413, 2445, 2476, 2507, 2537, 2567, 2596, 2625, 2653, 2681, 2709, 2736, 2762,
	2789, 2815, 2840, 2865, 2890, 2915, 2939, 2962, 2986, 3009, 3032, 3055, 3077,
	3099, 3121, 3142, 3163, 3184, 3205, 3226, 3246, 3266, 3286, 3306, 3325, 3344,
	3363, 3382, 3400, 3419, 3437, 3455, 3473, 3491, 3508, 3525, 3543, 3559, 3576,
	3593, 3610, 3626, 3642, 3658, 3674, 3690, 3705, 3721, 3736, 3752, 3767, 3782,
	3797, 3811, 3826, 3840, 3855, 3869, 3883, 3897, 3911, 3925, 3939, 3952, 3966,
	3979, 3993, 4006, 4019, 4032, 4045, 4058, 4070, 4083
};

// step = 1.0 / 128
// [int(math.pow(i * step, 2) * 4096) for i in xrange(0,128)]
const u16 EXP2[128] = {
	0, 0, 1, 2, 4, 6, 9, 12, 16, 20, 25, 30, 36, 42, 49, 56, 64, 72, 81, 90, 100,
	110, 121, 132, 144, 156, 169, 182, 196, 210, 225, 240, 256, 272, 289, 306,
	324, 342, 361, 380, 400, 420, 441, 462, 484, 506, 529, 552, 576, 600, 625,
	650, 676, 702, 729, 756, 784, 812, 841, 870, 900, 930, 961, 992, 1024, 1056,
	1089, 1122, 1156, 1190, 1225, 1260, 1296, 1332, 1369, 1406, 1444, 1482, 1521,
	1560, 1600, 1640, 1681, 1722, 1764, 1806, 1849, 1892, 1936, 1980, 2025, 2070,
	2116, 2162, 2209, 2256, 2304, 2352, 2401, 2450, 2500, 2550, 2601, 2652, 2704,
	2756, 2809, 2862, 2916, 2970, 3025, 3080, 3136, 3192, 3249, 3306, 3364, 3422,
	3481, 3540, 3600, 3660, 3721, 3782, 3844, 3906, 3969, 4032
};


typedef enum { eStandard, eFixed, eDrone } eEdge;
typedef enum { mNormal, mSlew, mEdge, mSelect, mBank } eMode;
typedef enum { rOff, rArm, rRec } rStatus;

typedef struct {
	u8 shape;
	u8 x;
	u8 y;
	u16 interval;
} pattern_event_t;

typedef struct {
	pattern_event_t e[EVENTS_PER_PATTERN];
	u8 length;
	u16 total_time;
	u8 loop;
	s8 x;
	s8 y;
} pattern_t;

typedef struct {
	u16 latch;
	u8 hys;
} ain_t;

typedef struct {
	u16 now;
	u16 target;
	u16 slew;
	u16 step;
	s32 delta;
	u32 a;
} aout_t;


typedef struct {
	u8 port_active;
	u16 port_time;
	aout_t aout[4];
} es_suspend_t;



typedef struct {
	eEdge edge;
	u16 edge_fixed_time;

	u8 p_select;
	u8 shape_on;
	u8 arp;
	u16 port_time;

	u16 cv[8][3];
	u16 slew[8][3];

	u8 help[16][8];

	pattern_t p[16];
} es_set;

typedef const struct {
	u8 fresh;
	u8 preset_select;
	u8 glyph[8][8];
	es_set es[8];
} nvram_data_t;

es_set es;
es_suspend_t suspend;

u8 preset_mode, preset_select, front_timer;
u8 glyph[8];

u8 held_keys[32], key_times[256], key_map[256], min_x, min_y;
s16 key_count;

u8 root_x, root_y, last_x, last_y, shape_on, legato, singled;
s16 shape_key_count;

eMode mode;

u16 edge_counter;
u16 edge_state;

u8 shape_counter;

u8 clock_phase;
u16 clock_time, clock_temp;

u16 adc[4];
u16 adc_last[4];


ain_t ain[3];
aout_t aout[4];

u8 p_select, arp;

u8 p_playing;
u8 p_play_pos;
rStatus r_status;
u8 arm_key;
u8 selected;
u16 rec_timer;
u8 rec_position;
u16 p_timer;
u16 p_timer_total;
u8 blinker;
u8 all_edit;

u8 clock_mode;

u8 midi_legato;
u8 vel_shape, track_shape;

s8 move_x, move_y;

//this
u16 port_time;

u8 port_active, port_edit, port_toggle;
volatile u8 slew_active;


u8 SIZE, LENGTH, VARI;

typedef void(*re_t)(void);
re_t re;


// NVRAM data structure located in the flash array.
__attribute__((__section__(".flash_nvram")))
static nvram_data_t flashy;




////////////////////////////////////////////////////////////////////////////////
// prototypes

static void refresh(void);
static void refresh_mono(void);
static void refresh_preset(void);
static void clock(u8 phase);

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

u8 flash_is_fresh(void);
void flash_unfresh(void);
void flash_write(void);
void flash_read(void);

static void shape(u8 s, u8 x, u8 y);
static void pattern_shape(u8 s, u8 x, u8 y);

void rec_arm(void);
void rec_start(void);
void rec_stop(void);
void rec(u8 shape, u8 x, u8 y); 
void play(void);
void stop(void);

void pattern_linearize(void);
void pattern_time_half(void);
void pattern_time_double(void);

void reset_hys(void);


static void es_process_ii(uint8_t i, int d);
static void es_midi_process_ii(uint8_t i, int d);


void reset_hys() {
	u8 i1;

	for(i1=0;i1<3;i1++) {
		ain[i1].hys = 0;
		ain[i1].latch = adc_last[i1] = adc[i1];
	}
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application clock code

void clock(u8 phase) {
;;
}



////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t cvTimer = { .next = NULL, .prev = NULL };
static softTimer_t adcTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer  = { .next = NULL, .prev = NULL };
static softTimer_t midiPollTimer = { .next = NULL, .prev = NULL };


static void aout_write(void) {
	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x31);
	spi_write(SPI,aout[2].now>>4);
	spi_write(SPI,aout[2].now<<4);
	spi_write(SPI,0x31);
	spi_write(SPI,aout[0].now>>4);
	spi_write(SPI,aout[0].now<<4);
	spi_unselectChip(SPI,DAC_SPI);
	
	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x38);
	spi_write(SPI,aout[3].now>>4);
	spi_write(SPI,aout[3].now<<4);
	spi_write(SPI,0x38);
	spi_write(SPI,aout[1].now>>4);
	spi_write(SPI,aout[1].now<<4);
	spi_unselectChip(SPI,DAC_SPI);
}

static void cvTimer_callback(void* o) { 
	u8 i;
	
	if(slew_active) {
		for(i=0;i<4;i++) {
			if(aout[i].step) {
				aout[i].step--;
				
				if(aout[i].step == 0) {
					aout[i].now = aout[i].target;
				}
				else {
					aout[i].a += aout[i].delta;
					aout[i].now = aout[i].a >> 16;
				}
				
				monomeFrameDirty++;
			}
		}
		aout_write();
	}
}

static void clockTimer_callback(void* o) {  
	u16 s;
	u8 i1, i2;

	// static event_t e;
	// e.type = kEventTimer;
	// e.data = 0;
	// event_post(&e);
	// print_dbg("\r\ntimer.");

	// clock_phase++;
	// if(clock_phase>1) clock_phase=0;
	// (*clock_pulse)(clock_phase);

	if(shape_counter) {
		if(shape_counter == 1) {
			s = 0;
			for(i1=0;i1<3;i1++)
				if(min_y + i1 < 16)
				for(i2=0;i2<3;i2++) {
					s <<= 1;
					if(min_x + i2 < 16)
					if(key_map[(min_y+i1)*16+(min_x+i2)]) s |= 1;
				}
			// print_dbg("\r\nfound shape pattern: ");
			// print_dbg_ulong(s);

			for(i1=0;i1<15;i1++)
				if(s == SHAPE_PATTERN[i1]) break;

			if(i1 < 9) {
				if(r_status != rOff) 
					rec(i1, min_x, min_y + SHAPE_OFF_Y[i1]);
				shape(i1, min_x, min_y + SHAPE_OFF_Y[i1]);
				legato = 1;
			}
			else if(i1 < 15) {
				// MAGICS
				if(i1==9)
					all_edit = 1;
				else if(i1==10)
					pattern_linearize();
				else if(i1==11)
					pattern_time_half();
				else if(i1==12)
					pattern_time_double();
				else if(i1==13) {
					if(p_select<15)
						p_select++;
					else
						p_select = 0;
					play();
				}
				else if(i1==14) {
					if(p_select > 0)
						p_select--;
					else
						p_select = 15;
					play();
				}
			}
			else {
				if(r_status != rOff)
					rec(0,last_x,last_y);
				shape(0, last_x, last_y);
				legato = 1;
			}
		}
		
		shape_counter--;
	}

	if(edge_counter) {
		if(edge_counter == 1) {
			gpio_clr_gpio_pin(B00);
			edge_state = 0;
			monomeFrameDirty++;
			// print_dbg("\r\ntrig done.");
		}

		edge_counter--;
	}

	if(r_status == rRec) {
		rec_timer++;
	}

	if(clock_mode == 0) {
		if(p_playing) {
			if(p_timer == 0) {

				if(p_play_pos == es.p[p_select].length && !es.p[p_select].loop) {
					// print_dbg("\r\nPATTERN DONE");
					p_playing = 0;
				}
				else {
					if(p_play_pos == es.p[p_select].length && es.p[p_select].loop) {
						// print_dbg("\r\nLOOP");
						p_play_pos = 0;
						p_timer_total = 0;
					}

					u8 i = p_play_pos;
					p_timer = es.p[p_select].e[i].interval;

					s8 x = es.p[p_select].e[i].x + es.p[p_select].x;
					s8 y = es.p[p_select].e[i].y + es.p[p_select].y;

					if(x<0) x = 0;
					else if(x>15) x=15;
					if(y<0) y = 0;
					else if(y>7) y=7;


					// print_dbg("\r\n");
					// print_dbg_ulong(i);
					// print_dbg(" : ");
					// print_dbg_ulong(es.p[p_select].e[i].shape);
					// print_dbg(" @ (");
					// print_dbg_ulong(es.p[p_select].e[i].x);
					// print_dbg(", ");
					// print_dbg_ulong(es.p[p_select].e[i].y);
					// print_dbg(")   NEXT: ");
					// print_dbg_ulong(es.p[p_select].e[i].interval);

					pattern_shape(es.p[p_select].e[i].shape, (u8)x, (u8)y);


					p_play_pos++;

				}
			}
			else p_timer--;

			p_timer_total++;

			monomeFrameDirty++;
		}
	}


	if(r_status == rRec || all_edit || !VARI) {
		blinker++;
		if(blinker == 48)
			blinker = 0;
		if(blinker == 0 || blinker == 24)
			monomeFrameDirty++;
	}
}

void rec_arm() {
	stop();
	// print_dbg("\r\narm");

	r_status = rArm;
}

void rec_start() {
	rec_timer = 0;
	r_status = rRec;
	// print_dbg("\r\nrec");
}

void rec_stop() {
	u8 i;

	// print_dbg("\r\nstopped rec");

	// set final length
	if(rec_timer) es.p[p_select].e[rec_position-1].interval = rec_timer-1;
	else es.p[p_select].e[rec_position-1].interval = 1;

	es.p[p_select].length = rec_position;

	es.p[p_select].total_time = 0;

	es.p[p_select].x = 0;
	es.p[p_select].y = 0;

	for(i=0;i<rec_position;i++) {
		es.p[p_select].total_time += es.p[p_select].e[i].interval;

		// print_dbg("\r\n");
		// print_dbg_ulong(i);
		// print_dbg(" : ");
		// print_dbg_ulong(es.p[p_select].e[i].shape);
		// print_dbg(" @ (");
		// print_dbg_ulong(es.p[p_select].e[i].x);
		// print_dbg(", ");
		// print_dbg_ulong(es.p[p_select].e[i].y);
		// print_dbg(") + ");
		// print_dbg_ulong(es.p[p_select].e[i].interval);
	}

	// print_dbg("\r\ntotal time: ");
	// print_dbg_ulong(es.p[p_select].total_time);

	r_status = rOff;
}

void rec(u8 shape, u8 x, u8 y) {
	if(r_status == rArm) {
 		es.p[p_select].e[0].shape = shape;
		es.p[p_select].e[0].x = x;
		es.p[p_select].e[0].y = y;
		rec_position = 1;
		rec_start();
	}
	else {
		es.p[p_select].e[rec_position].shape = shape;
		es.p[p_select].e[rec_position].x = x;
		es.p[p_select].e[rec_position].y = y;
		if(rec_timer) es.p[p_select].e[rec_position-1].interval = rec_timer-1;
		else es.p[p_select].e[rec_position-1].interval = 1;
		
		rec_position++;
		rec_timer = 0;
		if(rec_position == EVENTS_PER_PATTERN)
			rec_stop();
	}
}

void play() {
	p_timer = 0;
	p_play_pos = 0;
	p_timer_total = 0;
	p_playing = 1;

	// print_dbg("\r\nPLAY");
}

void stop() {
	p_playing = 0;
	if(es.edge == eStandard) {
		gpio_clr_gpio_pin(B00);
		edge_state = 0;
		// legato = 0;
	}
}


void pattern_linearize() {
	u8 i, note, rest;

	note = es.p[p_select].e[0].interval;
	rest = es.p[p_select].e[1].interval;


	for(i=0;i<es.p[p_select].length+1;i++)
		if(i%2)
			es.p[p_select].e[i].interval = rest;
		else
			es.p[p_select].e[i].interval = note;

	es.p[p_select].total_time = 0;

	for(i=0;i<es.p[p_select].length;i++) {
		es.p[p_select].total_time += es.p[p_select].e[i].interval;

		// print_dbg("\r\n");
		// print_dbg_ulong(i);
		// print_dbg(" : ");
		// print_dbg_ulong(es.p[p_select].e[i].shape);
		// print_dbg(" @ (");
		// print_dbg_ulong(es.p[p_select].e[i].x);
		// print_dbg(", ");
		// print_dbg_ulong(es.p[p_select].e[i].y);
		// print_dbg(") + ");
		// print_dbg_ulong(es.p[p_select].e[i].interval);
	}
 }

void pattern_time_half() {
	u8 i;

	for(i=0;i<es.p[p_select].length+1;i++) {
		es.p[p_select].e[i].interval = es.p[p_select].e[i].interval >> 1;
		if(!es.p[p_select].e[i].interval) es.p[p_select].e[i].interval = 1;
	}

	es.p[p_select].total_time = 0;

	for(i=0;i<es.p[p_select].length;i++) {
		es.p[p_select].total_time += es.p[p_select].e[i].interval;
	}
}

void pattern_time_double() {
	u8 i;

	for(i=0;i<es.p[p_select].length+1;i++)
			es.p[p_select].e[i].interval = es.p[p_select].e[i].interval << 1;

	es.p[p_select].total_time = 0;

	for(i=0;i<es.p[p_select].length;i++) {
		es.p[p_select].total_time += es.p[p_select].e[i].interval;
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

//midi polling callback
static void midi_poll_timer_callback(void* obj) {
  // asynchronous, non-blocking read
  // UHC callback spawns appropriate events
  midi_read();
}

// monome polling callback
static void monome_poll_timer_callback(void* obj) {
  // asynchronous, non-blocking read
  // UHC callback spawns appropriate events
	ftdi_read();
}

// monome refresh callback
static void monome_refresh_timer_callback(void* obj) {
	if(monomeFrameDirty > 0) {
		static event_t e;
		e.type = kEventMonomeRefresh;
		event_post(&e);
	}
}

// monome: start polling
void timers_set_monome(void) {
	// print_dbg("\r\n setting monome timers");
	timer_add(&monomePollTimer, 20, &monome_poll_timer_callback, NULL );
	timer_add(&monomeRefreshTimer, 30, &monome_refresh_timer_callback, NULL );
}

// monome stop polling
void timers_unset_monome(void) {
	// print_dbg("\r\n unsetting monome timers");
	timer_remove( &monomePollTimer );
	timer_remove( &monomeRefreshTimer ); 
}



////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) {
	// print_dbg("\r\n// ftdi connect ///////////////////");
	ftdi_setup();
}

static void handler_FtdiDisconnect(s32 data) {
	// print_dbg("\r\n// ftdi disconnect ////////////////");
	timers_unset_monome();
	// event_t e = { .type = kEventMonomeDisconnect };
	// event_post(&e);
	timer_remove( &adcTimer );
}

static void handler_MonomeConnect(s32 data) {
	// print_dbg("\r\n// monome connect /////////////////"); 
	key_count = 0;
	SIZE = monome_size_x();
	LENGTH = SIZE - 1;
	// print_dbg("\r monome size: ");
	// print_dbg_ulong(SIZE);
	VARI = monome_is_vari();
	// print_dbg("\r monome vari: ");
	// print_dbg_ulong(VARI);

	if(VARI) re = &refresh;
	else re = &refresh_mono;
	
	shape_key_count = 0;
	key_count = 0;

	timers_set_monome();

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);


	// turn on ADC polling, reset hysteresis
	adc_convert(&adc);
	reset_hys();
	timer_add(&adcTimer,61,&adcTimer_callback, NULL);
}

static void handler_MonomePoll(s32 data) { monome_read_serial(); }
static void handler_MonomeRefresh(s32 data) {
	if(monomeFrameDirty) {
		if(preset_mode == 0) (*re)(); //refresh_mono();
		else refresh_preset();

		(*monome_refresh)();
	}
}


static void handler_Front(s32 data) {
	// print_dbg("\r\n //// FRONT HOLD");

	if(data == 0) {
		front_timer = 15;
		if(preset_mode) preset_mode = 0;
		else preset_mode = 1;
	}
	else {
		front_timer = 0;
	}

	monomeFrameDirty++;
}

static void handler_PollADC(s32 data) {
	u8 i,n;

	adc_convert(&adc);

	for(i=0;i<3;i++) {
		if(ain[i].hys) {
			if(port_edit == 1) {
				if(i == 0) {
					aout[3].slew = port_time = ((adc[i] + adc_last[i])>>1 >> 4);
					// print_dbg("\r\nportamento: ");
					// print_dbg_ulong(port_time);
				}
			}
			else if(all_edit) {
				if(mode == mNormal) {
					for(n=0;n<8;n++)
						es.cv[n][i] = aout[i].target = (adc[i] + adc_last[i])>>1;
					aout[i].step = 5; // smooth out the input
					aout[i].delta = ((aout[i].target - aout[i].now)<<16) / aout[i].step;
					aout[i].a = aout[i].now<<16;
				}
				else if(mode == mSlew) {
					for(n=0;n<8;n++)
						es.slew[n][i] = aout[i].slew = (adc[i] + adc_last[i])>>1;
				}
			}
			else if(mode == mNormal) {
				es.cv[shape_on][i] = aout[i].target = (adc[i] + adc_last[i])>>1;
				aout[i].step = 5; // smooth out the input
				aout[i].delta = ((aout[i].target - aout[i].now)<<16) / aout[i].step;
				aout[i].a = aout[i].now<<16;
			}
			else if(mode == mSlew)
				es.slew[shape_on][i] = aout[i].slew = (adc[i] + adc_last[i])>>1;
			else if(mode == mEdge && i == 0)
				es.edge_fixed_time = ((adc[i] + adc_last[i])>>1 >> 4);


			monomeFrameDirty++;
		}
		else if(abs(((adc[i] + adc_last[i])>>1) - ain[i].latch) > POT_HYSTERESIS)
			ain[i].hys = 1;

		adc_last[i] = adc[i];
	}

	// print_dbg("\r\nadc:\t"); print_dbg_ulong((adc[0] + adc_last[0])>>1);
	// print_dbg("\t"); print_dbg_ulong((adc[1] + adc_last[1])>>1);
	// print_dbg("\t"); print_dbg_ulong((adc[2] + adc_last[2])>>1);
	// print_dbg("\t"); print_dbg_ulong((adc[3] + adc_last[3])>>1);

}

static void handler_SaveFlash(s32 data) {
	flash_write();
}

static void handler_KeyTimer(s32 data) {
	static u16 i1;

	if(front_timer) {
		if(front_timer == 1) {
			static event_t e;
			e.type = kEventSaveFlash;
			event_post(&e);

			preset_mode = 0;
			front_timer--;
		}
		else front_timer--;
	}

	for(i1=0;i1<key_count;i1++) {
		if(key_times[held_keys[i1]])
		if(--key_times[held_keys[i1]]==0) {
			if(preset_mode == 1) {
				if(held_keys[i1] % 16 == 0) {
					preset_select = held_keys[i1] / 16;
					// flash_write();
					static event_t e;
					e.type = kEventSaveFlash;
					event_post(&e);
					preset_mode = 0;
					monomeFrameDirty++;
				}
			}

			// print_dbg("\rlong press: "); 
			// print_dbg_ulong(held_keys[i1]);
		}
	}
}


static void handler_ClockNormal(s32 data) {
	// print_dbg("\r\nclock norm int");
	// clock_external = !gpio_get_pin_value(B09); 
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application grid code

static void handler_MonomeGridKey(s32 data) { 
	u8 x, y, z, index, i1, found;

	monome_grid_key_parse_event_data(data, &x, &y, &z);
	// print_dbg("\r\n monome event; x: "); 
	// print_dbg_hex(x); 
	// print_dbg("; y: 0x"); 
	// print_dbg_hex(y); 
	// print_dbg("; z: 0x"); 
	// print_dbg_hex(z);



	//// TRACK LONG PRESSES
	index = y*16 + x;
	if(z) {
		held_keys[key_count] = index;
		key_count++;
		key_times[index] = 10;		//// THRESHOLD key hold time
		if(x == 0 && y == 7) key_times[index] = 3;
	} else {
		found = 0; // "found"
		for(i1 = 0; i1<key_count; i1++) {
			if(held_keys[i1] == index) 
				found++;
			if(found) 
				held_keys[i1] = held_keys[i1+1];
		}
		key_count--;

		if(key_count < 1)
			key_count = 0;

		// FAST PRESS
		if(key_times[index] > 0) {

			// PRESET MODE FAST PRESS DETECT
			if(preset_mode == 1) {
				if(x == 0 && y != preset_select) {
					preset_select = y;
					for(i1=0;i1<8;i1++)
						glyph[i1] = flashy.glyph[preset_select][i1];
				}
 				else if(x==0 && y == preset_select) {
					flash_read();

					reset_hys();
					for(i1=0;i1<3;i1++) {
						aout[i1].target = aout[i1].now = es.cv[shape_on][i1];
						aout[i1].slew = es.slew[shape_on][i1];
						aout[i1].step = 0;
					}

					preset_mode = 0;
				}

				monomeFrameDirty++;
			}
			else {
				if(x == 0 && y == 7) {
					if(port_toggle)
						port_toggle = 1;
				}
			}
			// print_dbg("\r\nfast press: ");
			// print_dbg_ulong(index);
			// print_dbg(": ");
			// print_dbg_ulong(key_times[index]);
		}
	}

	// print_dbg("\r\ncount: ");
	// print_dbg_ulong(key_count);
	// print_dbg("\r\nmin x: ");
	// print_dbg_ulong(min_x);
	// print_dbg("\r\nmin y: ");
	// print_dbg_ulong(min_y);


	// PRESET SCREEN
	if(preset_mode) {
		// glyph magic
		if(z && x>7) {
			glyph[y] ^= 1<<(x-8);
			monomeFrameDirty++;	
		}
	}
	// NOT PRESET
	else {
		key_map[y*16+x] = z;

		if(x == 0) {
			// PLAY
 			if(y==0) {
				if(z) {
					if(r_status != rOff) {
						if(r_status == rRec) {
							rec_stop();
							es.p[p_select].loop = 1;
							r_status = rOff;
							play();
						}
						else {
							r_status = rOff;
							play();
						}
					}
					else if(arm_key) {
						play();
						selected = 1;
					}
					else if(mode == mSelect) {
						mode = mBank;
					}
					else if(p_playing) {
						stop();
					}
					else {
						play();
					}
				}
			}
			// SELECT
			else if(y==1) {
				
				if(z && mode != mBank) {
					r_status = rOff;
					mode = mSelect;
					// print_dbg("\r\nmode: select");
				}
				else if((z == 0 && mode != mBank) || (z && mode == mBank)) {
					mode = mNormal;
					// print_dbg("\r\nmode: normal");
				}
			}
			// REC
			else if(y==2) {
				arm_key = z;
 				if(z) {
					if(r_status == rOff) {
						selected = 0;
					}
					else if(r_status == rArm) {
						r_status = rOff;
						selected = 1;
						// print_dbg("\r\narm off");
					}
					else if(r_status == rRec) {
						rec_stop();
						r_status = rOff;
						selected = 1;
					}
				}
				else {
					if(r_status == rOff && !selected) {
						rec_arm();

						if(mode == mBank)
							mode = mNormal;

						// turn off pattern juggle mode
						// mode = mNormal;
					}
				}
			}
			// PATTERN loop
			else if(y==3) {
				if(z) {
					es.p[p_select].loop ^= 1;
				}
				
			}
			// ARP mode
			else if(y==4) {
				if(z) {
					arp ^= 1;
				}
			}
			// MODE: EDGE
			else if(y==5) {
				reset_hys();

				if(z)
					mode = mEdge;
				else
					mode = mNormal;
			}
			// PORT
			else if(y==7) {
				reset_hys();

				if(mode == mEdge) {
					es.edge_fixed_time = 0;
				}
				else {
					if(z) {
						if(port_toggle == 1) {
							port_active = 0;
							aout[3].slew = 0;
							port_toggle = 0;
						}
						else {
							port_toggle = 2;
							port_active = 1;
							port_edit = 1;
							aout[3].slew = EXP[port_time];
						}
					}
					else {
						port_edit = 0;
						if(port_toggle == 2) {
							port_active = 0;
							aout[3].slew = 0;
							port_toggle = 0;
						}
					}
				}
			}
			// MODE: SLEW
			else if(y==6) {
				reset_hys();

				if(z)
					mode = mSlew;
				else
					mode = mNormal;
			}

			monomeFrameDirty++;
		}
		// NORMAL DETECTION
		else if(z) {
			shape_key_count++;

			if(x < min_x) min_x = x;
			if(y < min_y) min_y = y;

			// all-edit breakout
			if(all_edit) {
				reset_hys();
				all_edit = 0;
			}

			if(mode == mSlew) {
				es.help[x][y] ^= 1;
			}

			// EDGE MODE
			if(mode == mEdge) {
				shape_counter = 0;

				if(y==7) {
					es.edge_fixed_time = x*16;
				}
				else if(x < 6) es.edge = eStandard;
				else if(x < 11) es.edge = eFixed;
				else es.edge = eDrone;

				monomeFrameDirty++;
			}
			// SELECT
			else if(mode == mSelect || mode == mBank) {
				if(x>1 && x < 6 && y > 1 && y < 6) {
					stop();
					p_select = (y-2)*4+(x-2);
					if(mode == mBank)
						play();
					// print_dbg("\r\nselected ");
					// print_dbg_ulong(p_select);
				}
			}
			// SHAPE DETECT
			else if(!legato) {
				if(key_count == 1) {
					shape_counter = SHAPE_COUNT;
				}
				else if(abs(x - last_x) > 2 || abs(y - last_y) > 2) {
					shape_counter = 0;

					if(r_status != rOff) rec(0,x,y);
					shape(0,x,y);
				}
				else {
					shape_counter = 5;
				}

				last_x = x;
				last_y = y;
			}
			// LEGATO
			else {
				if(r_status != rOff) rec(0,x,y);
				shape(0,x,y);
			}
		}
		// key up
		else {
			shape_counter = 0;

			shape_key_count--;

			if(shape_key_count < 1) {
				shape_key_count = 0;

		 		min_x = 15;
				min_y = 7;

				if(es.edge == eStandard) {
					if(r_status != rOff)
						rec(100,x,y);
					gpio_clr_gpio_pin(B00);
					edge_state = 0;
				}

				legato = 0;

				monomeFrameDirty++;
			}
 		}
	}
}



static void shape(u8 s, u8 x, u8 y) {
	u8 i;

	// print_dbg("\r\nfound shape: ");
	// print_dbg_ulong(s);

	// PATTERN PLAY MODE
	if(arp && r_status == rOff && s<4) {
		es.p[p_select].x = x - es.p[p_select].e[0].x;
		es.p[p_select].y = y - es.p[p_select].e[0].y;
	}
	else if(s<4) {
		// cv_pos = SCALES[0][x] + (7-y)*170;
		// cv_pos = SEMI[x+(7-y)*5];
		aout[3].target = SEMI[x+(7-y)*5];
		// aout[3].target = TONE[x*scale[scale_x]+(7-y)*scale[scale_y]];

		if(port_active) {
			aout[3].step = (aout[3].slew >> 2) + 1;
			aout[3].delta = ((aout[3].target - aout[3].now)<<16) / aout[3].step;
			aout[3].a = aout[3].now<<16;
		}
		else {
			aout[3].now = aout[3].target;
		}
	}

	if(!arp && r_status == rOff && !port_active) {

		spi_selectChip(SPI,DAC_SPI);

		spi_write(SPI,0x38);	// update B
		spi_write(SPI,aout[3].now>>4);
		spi_write(SPI,aout[3].now<<4);

		spi_write(SPI,0x80);	// update B
		spi_write(SPI,0xff);
		spi_write(SPI,0xff);

		spi_unselectChip(SPI,DAC_SPI);
	}

	if(s == 0)
		singled = 1;
	else {
		if(shape_on != (s-1)) {
			shape_on = s-1;

			for(i=0;i<3;i++) {
				// don't change CV if above thresh
				if(es.slew[shape_on][i] < SLEW_CV_OFF_THRESH) {
					aout[i].target = es.cv[shape_on][i];
					aout[i].slew = es.slew[shape_on][i];

					aout[i].step = EXP[aout[i].slew >> 4] + 1;
					aout[i].delta = ((aout[i].target - aout[i].now)<<16) / aout[i].step;
					aout[i].a = aout[i].now<<16;
				}
			}

			reset_hys();
		}

		singled = 0;
	}

	if(es.edge == eDrone) {
		if(root_x == x && root_y == y && edge_state) {
			gpio_clr_gpio_pin(B00);
			edge_state = 0;
		}
		else {
			gpio_set_gpio_pin(B00);
			edge_state = 1;
		}
	}
	else if(s<5) {
		gpio_set_gpio_pin(B00);
		edge_state = 1;

		if(es.edge == eFixed) {
			edge_counter = (EXP[es.edge_fixed_time]>>2) + 2;
			// print_dbg("\r\ntrig fixed: ");
			// print_dbg_ulong(edge_counter);
		}
	}

	if(arp && r_status == rOff && s<5 && !legato)
		play();

	root_x = x;
	root_y = y;

	monomeFrameDirty++;
}


// this gets called by the pattern recorder
static void pattern_shape(u8 s, u8 x, u8 y) {
	u8 i;

	// print_dbg("\r\nfound shape: ");
	// print_dbg_ulong(s);

	if(s == 100) {
		if(es.edge == eStandard) {
			gpio_clr_gpio_pin(B00);
			edge_state = 0;
		}
	}
	else {
		// cv_pos = SCALES[0][x] + (7-y)*170;

		aout[3].target = SEMI[x+(7-y)*5];
		// aout[3].target = TONE[x*scale[scale_x]+(7-y)*scale[scale_y]];


		if(port_active) {
			aout[3].step = (aout[3].slew >> 2) + 1;
			aout[3].delta = ((aout[3].target - aout[3].now)<<16) / aout[3].step;
			aout[3].a = aout[3].now<<16;
		}
		else {
			aout[3].now = aout[3].target;
		}

		if(!port_active) {
			spi_selectChip(SPI,DAC_SPI);

			spi_write(SPI,0x38);	// update B
			spi_write(SPI,aout[3].now>>4);
			spi_write(SPI,aout[3].now<<4);

			spi_write(SPI,0x80);	// update B
			spi_write(SPI,0xff);
			spi_write(SPI,0xff);

			spi_unselectChip(SPI,DAC_SPI);
		}

		if(s == 0) {
			singled = 1;
		}
		else {
			if(shape_on != (s-1)) {
				shape_on = s-1;

				for(i=0;i<3;i++) {
					// don't change CV if above thresh
					if(es.slew[shape_on][i] < SLEW_CV_OFF_THRESH) {
						aout[i].target = es.cv[shape_on][i];
						aout[i].slew = es.slew[shape_on][i];

						aout[i].step = EXP[aout[i].slew >> 4] + 1;
						aout[i].delta = ((aout[i].target - aout[i].now)<<16) / aout[i].step;
						aout[i].a = aout[i].now<<16;
					}
				}

				reset_hys();
			}

			singled = 0;
		}

		if(es.edge == eDrone) {
			if(root_x == x && root_y == y && edge_state) {
				gpio_clr_gpio_pin(B00);
				edge_state = 0;
			}
			else {
				gpio_set_gpio_pin(B00);
				edge_state = 1;
			}
		}
		else if(s<5) {
			gpio_set_gpio_pin(B00);
			edge_state = 1;

			if(es.edge == eFixed) {
				edge_counter = (EXP[es.edge_fixed_time]>>2) + 2;
				// print_dbg("\r\ntrig fixed: ");
				// print_dbg_ulong(edge_counter);
			}
		}


		root_x = x;
		root_y = y;
	}

	monomeFrameDirty++;
}





////////////////////////////////////////////////////////////////////////////////
// application grid redraw
static void refresh() {
	u8 i1, i2, i3;

	// CLEAR // FIXME: optimize? 
	// for(i1=0;i1<128;i1++) monomeLedBuffer[i1] = 0;
	for(i1=0;i1<8;i1++) {
		for(i2=0;i2<16;i2++)
			monomeLedBuffer[i1*16+i2] = es.help[i2][i1] << 2;
	}
	

	// REC STATUS
	if(r_status == rArm) monomeLedBuffer[32] = 7;
	else if(r_status == rRec) monomeLedBuffer[32] = 11 + 4 * (blinker < 24);

	// LOOP and MODE 
	if(es.p[p_select].loop) monomeLedBuffer[48] = 11;
	if(arp) monomeLedBuffer[64] = 11;

	// PATTERN PLAY MODE
	if(mode == mBank) monomeLedBuffer[16] = 11;

	// PATTERN INDICATION
	if(p_playing) {
		i2 = p_timer_total / (es.p[p_select].total_time / 16);

		for(i1=0;i1<16;i1++)
			if(i1 < i2) monomeLedBuffer[i1] = 4;

		monomeLedBuffer[0] = 15;
	}
	else if(es.p[p_select].length != 0) monomeLedBuffer[0] = 4;

	// PORT INDICATOR
	if(port_toggle) monomeLedBuffer[112] = port_toggle * 4 + 3;

	// EDGE SELECT
	if(mode == mEdge) {
		for(i1=0;i1<3;i1++) {
			for(i2=0;i2<4;i2++) {
				for(i3=0;i3<4;i3++) {
					if(((EDGE_GLYPH[i1][i2] >> i3) & 1))
						monomeLedBuffer[34 + (i1*5) + i2*16 + i3] = 7 + (es.edge == i1) * 4;
					// else
						// monomeLedBuffer[34 + (i1*5) + i2*16 + i3] = 4;
				}
			}
		}

		if(es.edge == eFixed) {
			for(i1=0;i1<16;i1++) {
				monomeLedBuffer[112+i1] = 4;
			}

			monomeLedBuffer[112 + (es.edge_fixed_time>>4)] = 11;
		}
	}
	// SELECT PATTERN
	else if(mode == mSelect || mode == mBank) {
		for(i1=0;i1<4;i1++)
			for(i2=0;i2<4;i2++)
				if(es.p[i1*4+i2].length) monomeLedBuffer[i1*16+i2+34] = 7;
				else monomeLedBuffer[i1*16+i2+34] = 4;
				

		monomeLedBuffer[34 + (p_select%4) + (p_select / 4) * 16] = 15;

	}
	// STATE	
	else {
		if(arp)
			monomeLedBuffer[(es.p[p_select].e[0].y + es.p[p_select].y) * 16 + es.p[p_select].x + es.p[p_select].e[0].x] = 7;

		if(port_active)
			for(i1=0;i1<(port_time>>4)+1;i1++)
				monomeLedBuffer[i1 + 16] = 4;

		// SLEW INDICATORS
		if(mode == mSlew) {
			if(es.slew[shape_on][0] < SLEW_CV_OFF_THRESH) {
				for(i1=0;i1<(aout[0].slew>>8);i1++)
					monomeLedBuffer[80+i1] =  4;
				monomeLedBuffer[80+(aout[0].slew>>8)] =  11;
			}

			if(es.slew[shape_on][1] < SLEW_CV_OFF_THRESH) {
				for(i1=0;i1<(aout[1].slew>>8);i1++)
					monomeLedBuffer[96+i1] =  4;
				monomeLedBuffer[96+(aout[1].slew>>8)] =  11;
			}

			if(es.slew[shape_on][2] < SLEW_CV_OFF_THRESH) {
				for(i1=0;i1<(aout[2].slew>>8);i1++)
					monomeLedBuffer[112+i1] =  4;
				monomeLedBuffer[112+(aout[2].slew>>8)] =  11;
			}
		}
		// CV POSITIONS
		else {
			for(i1=min(aout[0].target>>8,aout[0].now>>8);i1<max(aout[0].target>>8,aout[0].now>>8)+1;i1++)
				monomeLedBuffer[80+i1] = 4;
			monomeLedBuffer[80+(aout[0].now>>8)] = 4 + (es.slew[shape_on][0] < SLEW_CV_OFF_THRESH) * 3;


			for(i1=min(aout[1].target>>8,aout[1].now>>8);i1<max(aout[1].target>>8,aout[1].now>>8)+1;i1++)
				monomeLedBuffer[96+i1] = 4;
			monomeLedBuffer[96+(aout[1].now>>8)] = 4 + (es.slew[shape_on][1] < SLEW_CV_OFF_THRESH) * 3;

			for(i1=min(aout[2].target>>8,aout[2].now>>8);i1<max(aout[2].target>>8,aout[2].now>>8)+1;i1++)
				monomeLedBuffer[112+i1] = 4;
			monomeLedBuffer[112+(aout[2].now>>8)] = 4 + (es.slew[shape_on][2] < SLEW_CV_OFF_THRESH) * 3;
		}

		if(!edge_state)
			monomeLedBuffer[root_y*16+root_x] = 11;
		else
			monomeLedBuffer[root_y*16+root_x] = 15;

		if(all_edit) {
			monomeLedBuffer[(root_y)*16+root_x] = 7 + 4 * (blinker < 24);
			if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x] = 7 + 4 * (blinker < 24);
			if(root_y > 0 && root_x < 15) monomeLedBuffer[(root_y-1)*16+root_x+1] = 7 + 4 * (blinker < 24);
			if(root_x < 15) monomeLedBuffer[(root_y)*16+root_x+1] = 7 + 4 * (blinker < 24);
		}
		else if(!singled) {
			if(shape_on == 0) 
				{ if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x] = 11; }
			else if(shape_on == 1)
				{ if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x+1] = 11; }
			else if(shape_on == 2) monomeLedBuffer[(root_y)*16+root_x+1] = 11;
			else if(shape_on == 3) monomeLedBuffer[(root_y+1)*16+root_x+1] = 11;
			else if(shape_on == 4) {
				if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x] = 11; 
				if(root_y > 1) monomeLedBuffer[(root_y-2)*16+root_x] = 11; }
			else if(shape_on == 5) { 
				if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x+1] = 11; 
				if(root_y > 1) monomeLedBuffer[(root_y-2)*16+root_x+2] = 11; }
			else if(shape_on == 6) { monomeLedBuffer[(root_y)*16+root_x+1] = 11; monomeLedBuffer[(root_y)*16+root_x+2] = 11; }
			else if(shape_on == 7) { monomeLedBuffer[(root_y+1)*16+root_x+1] = 11; monomeLedBuffer[(root_y+2)*16+root_x+2] = 11; }
		}
	}

	// MODIFIERS/MODE
	if(mode == mEdge) monomeLedBuffer[80] = 15;
	if(mode == mSlew) monomeLedBuffer[96] = 15;

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}


// application grid redraw without varibright
static void refresh_mono() {
	u8 i1, i2, i3;

	// CLEAR // FIXME: optimize? 
	for(i1=0;i1<128;i1++) monomeLedBuffer[i1] = 0;

	// REC STATUS
	if(r_status == rArm) monomeLedBuffer[32] = 15;
	else if(r_status == rRec) monomeLedBuffer[32] = 15 * (blinker < 24);

	// LOOP and MODE 
	if(es.p[p_select].loop) monomeLedBuffer[48] = 15;
	if(arp) monomeLedBuffer[64] = 15;

	// PATTERN PLAY MODE
	if(mode == mBank) monomeLedBuffer[16] = 15;

	// PATTERN INDICATION
	if(p_playing) {
		i2 = p_timer_total / (es.p[p_select].total_time / 16);

		for(i1=0;i1<16;i1++)
			if(i1 < i2) monomeLedBuffer[i1] = 15;
	}
	else if(es.p[p_select].length != 0) monomeLedBuffer[0] = 15;

	// PORT INDICATOR
	if(port_toggle) monomeLedBuffer[112] = 15;

	// EDGE SELECT
	if(mode == mEdge) {
		for(i1=0;i1<3;i1++) {
			for(i2=0;i2<4;i2++) {
				for(i3=0;i3<4;i3++) {
					if(((EDGE_GLYPH[i1][i2] >> i3) & 1)) {
						if(es.edge == i1)
							monomeLedBuffer[34 + (i1*5) + i2*16 + i3] = (blinker < 24) * 15;
						else
							monomeLedBuffer[34 + (i1*5) + i2*16 + i3] = 15;
					}
					// else
						// monomeLedBuffer[34 + (i1*5) + i2*16 + i3] = 4;
				}
			}
		}

		if(es.edge == eFixed) 
			monomeLedBuffer[112 + (es.edge_fixed_time>>4)] = 15;
	}
	// SELECT PATTERN
	else if(mode == mSelect || mode == mBank) {
		for(i1=0;i1<4;i1++)
			for(i2=0;i2<4;i2++)
				monomeLedBuffer[i1*16+i2+34] = 15;

		monomeLedBuffer[34 + (p_select%4) + (p_select / 4) * 16] = (blinker < 24) * 15;

	}
	// STATE	
	else {
		if(arp)
			monomeLedBuffer[(es.p[p_select].e[0].y + es.p[p_select].y) * 16 + es.p[p_select].x + es.p[p_select].e[0].x] = 15;

		if(port_active)
			for(i1=0;i1<(port_time>>4)+1;i1++)
				monomeLedBuffer[i1 + 16] = 15;

		// SLEW INDICATORS
		if(mode == mSlew) {
			if(es.slew[shape_on][0] < SLEW_CV_OFF_THRESH) {
				// for(i1=0;i1<(aout[0].slew>>8);i1++)
					// monomeLedBuffer[80+i1] =  4;
				monomeLedBuffer[80+(aout[0].slew>>8)] =  15;
			}

			if(es.slew[shape_on][1] < SLEW_CV_OFF_THRESH) {
				// for(i1=0;i1<(aout[1].slew>>8);i1++)
					// monomeLedBuffer[96+i1] =  4;
				monomeLedBuffer[96+(aout[1].slew>>8)] =  15;
			}

			if(es.slew[shape_on][2] < SLEW_CV_OFF_THRESH) {
				// for(i1=0;i1<(aout[2].slew>>8);i1++)
					// monomeLedBuffer[112+i1] =  4;
				monomeLedBuffer[112+(aout[2].slew>>8)] =  15;
			}
		}
		// CV POSITIONS
		else {
			// for(i1=min(aout[0].target>>8,aout[0].now>>8);i1<max(aout[0].target>>8,aout[0].now>>8)+1;i1++)
				// monomeLedBuffer[80+i1] = 4;
			monomeLedBuffer[80+(aout[0].now>>8)] = 15;//4 + (es.slew[shape_on][0] < SLEW_CV_OFF_THRESH) * 3;


			// for(i1=min(aout[1].target>>8,aout[1].now>>8);i1<max(aout[1].target>>8,aout[1].now>>8)+1;i1++)
				// monomeLedBuffer[96+i1] = 4;
			monomeLedBuffer[96+(aout[1].now>>8)] = 15;//4 + (es.slew[shape_on][1] < SLEW_CV_OFF_THRESH) * 3;

			// for(i1=min(aout[2].target>>8,aout[2].now>>8);i1<max(aout[2].target>>8,aout[2].now>>8)+1;i1++)
				// monomeLedBuffer[112+i1] = 4;
			monomeLedBuffer[112+(aout[2].now>>8)] = 15;//4 + (es.slew[shape_on][2] < SLEW_CV_OFF_THRESH) * 3;
		}

		// if(!edge_state)
			// monomeLedBuffer[root_y*16+root_x] = 11;
		// else
			monomeLedBuffer[root_y*16+root_x] = 15;

		if(all_edit) {
			monomeLedBuffer[(root_y)*16+root_x] = 15 * (blinker < 24);
			monomeLedBuffer[(root_y-1)*16+root_x] = 15 * (blinker < 24);
			monomeLedBuffer[(root_y-1)*16+root_x+1] = 15 * (blinker < 24);
			monomeLedBuffer[(root_y)*16+root_x+1] = 15 * (blinker < 24);
		}
		else if(!singled) {
			if(shape_on == 0) monomeLedBuffer[(root_y-1)*16+root_x] = 15;
			else if(shape_on == 1) 
				{ if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x+1] = 15; }
			else if(shape_on == 2) monomeLedBuffer[(root_y)*16+root_x+1] = 15;
			else if(shape_on == 3) monomeLedBuffer[(root_y+1)*16+root_x+1] = 15;
			else if(shape_on == 4) { 
				if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x] = 15; 
				if(root_y > 1) monomeLedBuffer[(root_y-2)*16+root_x] = 15; }
			else if(shape_on == 5) { 
				if(root_y > 0) monomeLedBuffer[(root_y-1)*16+root_x+1] = 15; 
				if(root_y > 1) monomeLedBuffer[(root_y-2)*16+root_x+2] = 15; }
			else if(shape_on == 6) { monomeLedBuffer[(root_y)*16+root_x+1] = 15; monomeLedBuffer[(root_y)*16+root_x+2] = 15; }
			else if(shape_on == 7) { monomeLedBuffer[(root_y+1)*16+root_x+1] = 15; monomeLedBuffer[(root_y+2)*16+root_x+2] = 15; }
		}
	}

	// MODIFIERS/MODE
	if(mode == mEdge) monomeLedBuffer[80] = 15;
	if(mode == mSlew) monomeLedBuffer[96] = 15;

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}


static void refresh_preset() {
	u8 i1,i2;

	for(i1=0;i1<128;i1++)
		monomeLedBuffer[i1] = 0;

	monomeLedBuffer[preset_select * 16] = 11;

	for(i1=0;i1<8;i1++)
		for(i2=0;i2<8;i2++)
			if(glyph[i1] & (1<<i2))
				monomeLedBuffer[i1*16+i2+8] = 11;

	monome_set_quadrant_flag(0);
	monome_set_quadrant_flag(1);
}





static void es_process_ii(uint8_t i, int d) {
	// print_dbg("\r\nes: ");
	// print_dbg_hex(i);
	// print_dbg(" ");
	// print_dbg_ulong(d);

	switch(i) {
		case ES_PRESET:
			if(d<0 || d>7)
				break;
			preset_select = d;
			flash_read();
			monomeFrameDirty++;
			break;
		case ES_MODE:
			if(d)
				clock_mode = 1;
			else
				clock_mode = 0;
			break;
		case ES_CLOCK:
			if(d && clock_mode) {
				if(p_play_pos == es.p[p_select].length && !es.p[p_select].loop) {
					// print_dbg("\r\nPATTERN DONE");
					p_playing = 0;
				}
				else {
					if(p_play_pos == es.p[p_select].length && es.p[p_select].loop) {
						// print_dbg("\r\nLOOP");
						p_play_pos = 0;
						p_timer_total = 0;
					}

					u8 i = p_play_pos;
					p_timer_total += es.p[p_select].e[i].interval;

					s8 x = es.p[p_select].e[i].x + es.p[p_select].x;
					s8 y = es.p[p_select].e[i].y + es.p[p_select].y;

					if(x<0) x = 0;
					else if(x>15) x=15;
					if(y<0) y = 0;
					else if(y>7) y=7;


					// print_dbg("\r\n");
					// print_dbg_ulong(i);
					// print_dbg(" : ");
					// print_dbg_ulong(es.p[p_select].e[i].shape);
					// print_dbg(" @ (");
					// print_dbg_ulong(es.p[p_select].e[i].x);
					// print_dbg(", ");
					// print_dbg_ulong(es.p[p_select].e[i].y);
					// print_dbg(")   NEXT: ");
					// print_dbg_ulong(es.p[p_select].e[i].interval);

					pattern_shape(es.p[p_select].e[i].shape, (u8)x, (u8)y);


					p_play_pos++;

				}

				monomeFrameDirty++;
			}
			break;
		case ES_RESET:
			if(d) {
				if(r_status == rRec) {
					rec_stop();
					r_status = rOff;
					play();
				}
				play();
			}
			break;
		case ES_PATTERN:
			if(d < 0 || d > 15)
				break;
			if(p_playing) {
				stop();
				p_select = d;
				play();
			}
			else {
				stop();
				p_select = d;
			}
			break;
		case ES_TRANS:
			d = (short)d;
			es.p[p_select].x = (d % 5);
			es.p[p_select].y = -(d / 5);
			break;
		case ES_STOP:
			if(d) {
				if(r_status == rRec) {
					rec_stop();
					r_status = rOff;
				}
				stop();
			}
			break;
		case ES_TRIPLE:
			if(d<1 || d>4)
				break;
			pattern_shape(d+4,root_x,root_y);
			break;
		case ES_MAGIC:
			if(d==1)
				pattern_time_double();
			else if(d==2)
				pattern_time_half();
			else if(d==3)
				pattern_linearize();
 			break;
		default:
			break;
	}
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// application midi code

inline static u16 blend(u8 num, u8 mix) {
	// num: [0, 127]
	// mix: [0, 100], 0 == logish, 100 == expish
	u32 m;
	u16 v;
	
	if (mix == 0) {
		v = LOG[num];
	}
	else if (mix >= 100) {
		v = EXP2[num];
	}
	else {
		m = ((LOG[num] * (99 - mix)) + (EXP2[num] * mix)) / 99;
		v = (u16)m;
	}
	
	return v;
}

inline static void aout_set_pitch(u8 num) {
	aout[3].target = SEMI[num];
	if (port_active) { // portemento active
		aout[3].step = (aout[3].slew >> 2) + 1;
		aout[3].delta = ((aout[3].target - aout[3].now)<<16) / aout[3].step;
		aout[3].a = aout[3].now<<16;
	}
	else {
		aout[3].now = aout[3].target;
	}	
}

inline static void aout_set_velocity(u16 vel) {
	// TODO: support slewing on velocity when in legato mode but only when
	// restoring a previously played note
	// aout[2].target = vel << 5; // 128 << 5 == 4096; 12-bit dac
	aout[2].target = blend(vel, vel_shape);
	aout[2].now = aout[2].target;
	// print_dbg(" __vo: ");
	// print_dbg_ulong(aout[2].target);
}

inline static void aout_set_tracking(u16 num) {
	// aout[1].target = num << 5; // 128 << 5 == 4096; 12-bit dac
	aout[1].target = blend(num, track_shape);
	aout[1].now = aout[1].target;
	// print_dbg(" __to: ");
	// print_dbg_ulong(aout[1].target);
}

inline static void aout_set_a0(u8 num) {
	// TODO: support non-linear mappings?
	aout[0].target = num << 5;
	aout[0].now = aout[0].target;
}

static void aout_clear(void) {
	for (u8 i = 0; i < 4; i++) {
		aout[i].step = 0;
		aout[i].now = aout[i].target = 0;
		// slew? a? delta?
	}
}

static void midi_note_on(u8 ch, u8 num, u8 vel) {
	// print_dbg("\r\n midi_note_on(), ch: ");
	// print_dbg_ulong(ch);
	// print_dbg(" num: ");
	// print_dbg_ulong(num);
	// print_dbg(" vel: ");
	// print_dbg_ulong(vel);

	if (num > MIDI_NOTE_MAX)
		// drop notes outside CV range
		return

	// keep track of held notes for legato
	notes_hold(num, vel);
		
	// suspend slew cvTimer
	slew_active = 0;
	aout_set_pitch(num);
	aout_set_velocity(vel);
	aout_set_tracking(num);
	aout_write();
	
	// print_dbg("\r\n    dac // p:");
	// print_dbg_ulong(aout[3].target);
	// print_dbg(" v: ");
	// print_dbg_ulong(aout[2].target);
	// print_dbg(" t: ");
	// print_dbg_ulong(aout[1].target);
	
	gpio_set_gpio_pin(B00);
	
	slew_active = 1;
	
	reset_hys(); // FIXME: why? is this really correct?
}

static void midi_note_off(u8 ch, u8 num, u8 vel) {
	// print_dbg("\r\n midi_note_off(), ch: ");
	// print_dbg_ulong(ch);
	// print_dbg(" num: ");
	// print_dbg_ulong(num);
	// print_dbg(" vel: ");
	// print_dbg_ulong(vel);

	if (num > MIDI_NOTE_MAX)
		// drop notes outside CV range
		return
		
	notes_release(num);
	
	if (midi_legato) {
		const held_note_t *prior = notes_get(kNotePriorityLast);
		if (prior) {
			// print_dbg("\r\n   prior // num: ");
			// print_dbg_ulong(prior->num);
			// print_dbg(" vel: ");
			// print_dbg_ulong(prior->vel);
	
			slew_active = 0;
			aout_set_pitch(prior->num);
			aout_set_velocity(prior->vel);
			aout_set_tracking(prior->num);
			aout_write();
			slew_active = 1;
			// retrigger?
		}
		else {
			gpio_clr_gpio_pin(B00);
		}
	}
	else {
		// no legato mode
		gpio_clr_gpio_pin(B00);
	}
}

static void midi_channel_pressure(u8 ch, u8 val) {
	// print_dbg("\r\n midi_channel_pressure(), ch: ");
	// print_dbg_ulong(ch);
	// print_dbg(" val: ");
	// print_dbg_ulong(val);	
}

static void midi_pitch_bend(u8 ch, u16 bend) {
	// print_dbg("\r\n midi_pitch_bend(), ch: ");
	// print_dbg_ulong(ch);
	// print_dbg(" bend: ");
	// print_dbg_ulong(bend);	
}

static void midi_control_change(u8 ch, u8 num, u8 val) {
	// print_dbg("\r\n midi_control_change(), ch: ");
	// print_dbg_ulong(ch);
	// print_dbg(" num: ");
	// print_dbg_ulong(num);	
	// print_dbg(" val: ");
	// print_dbg_ulong(val);	
	
	if (num == 1) {
		// mod wheel
		// TODO: set a small amount of slewing
		slew_active = 0;
		aout_set_a0(val);
		aout_write(); // should we just write the one?
		slew_active = 1;
	}
}


static void handler_MidiPollADC(s32 data) {
	u8 i;
	u16 cv;
	
	adc_convert(&adc);
	
	for (i = 0; i < 3; i++) {
		if (ain[i].hys) {
			switch (i) {
				case 0:
					// portamento
					port_time = ((adc[i] + adc_last[i]) >> 1 >> 4);
					aout[3].slew = EXP[port_time];
					// slew is [0, 256]
					// print_dbg("\r\nadc 0 / portamento: ");
					// print_dbg_ulong(port_time);
					// print_dbg(" exp: ");
					// print_dbg_ulong(aout[3].slew);
					break;
				case 1:
					// cv is [0, 4096] -- 4038 seems like the max returned from converter
					cv = ((adc[i] + adc_last[i]) >> 1) / 40;  // [0, ~20]
					track_shape = (u8)cv;
					// print_dbg("\r\ntrack shape: ");
					// print_dbg_ulong(cv);
					break;
				case 2:
					cv = ((adc[i] + adc_last[i]) >> 1) / 40; // [0, ~20]
					vel_shape = (u8)cv;
					// print_dbg("\r\nvel shape: ");
					// print_dbg_ulong(cv);
					break;
			}
		}
		else if (abs(((adc[i] + adc_last[i]) >> 1) - ain[i].latch) > POT_HYSTERESIS) {
			ain[i].hys = 1;
		}
		adc_last[i] = adc[i];
	}
}

static void handler_MidiConnect(s32 data) {
	// print_dbg("\r\nmidi connect: 0x");
	// print_dbg_hex(data);
	
	// disable es and stash global state
	timer_remove(&clockTimer);
	//timer_remove(&adcTimer); // done by handler_FtdiDisconnect
	suspend.port_active = port_active;
	for (u8 i = 0; i < 4; i++)
		suspend.aout[i] = aout[i];
		
	process_ii = &es_midi_process_ii;
	
	notes_init();
	midi_legato = 1; // FIXME: allow this to be controlled!
	port_active = 1; // FIXME: allow this to be controlled!

	// reset outputs
	slew_active = 0;
	aout_clear();
	aout_write();
	gpio_clr_gpio_pin(B00);
	slew_active = 1;

	// switch handlers
	app_event_handlers[ kEventPollADC ] = &handler_MidiPollADC;

	reset_hys();
	
	// install timers
	timer_add(&adcTimer, 27, &adcTimer_callback, NULL);
	timer_add(&midiPollTimer, 13, &midi_poll_timer_callback, NULL);
}

static void handler_MidiDisconnect(s32 data) {
	// print_dbg("\r\nmidi disconnect: 0x");
	// print_dbg_hex(data);

	// remove midi related timers
	timer_remove(&midiPollTimer);
	timer_remove(&adcTimer); // remove ours

	// reset outputs
	slew_active = 0;
	aout_clear();
	aout_write();
	gpio_clr_gpio_pin(B00);
	slew_active = 1;

	reset_hys();

  // re-enable es
	app_event_handlers[ kEventPollADC ] = &handler_PollADC;
	
	// restore es global state
	port_active = suspend.port_active;
	for (u8 i = 0; i < 4; i++)
		aout[i] = suspend.aout[i];
	
	process_ii = &es_process_ii;
	
	// FIXME: copied from main()
	//timer_add(&adcTimer, 5, &adcTimer_callback, NULL); // done by handler_MonomeConnect
	timer_add(&clockTimer, 6, &clockTimer_callback, NULL);
}

static void handler_MidiPacket(s32 raw) {
	static u8 com;
	static u8 ch, num, val;
	static u16 bend;

	u32 data = (u32)raw;

	// print_dbg("\r\nmidi packet: 0x");
	// print_dbg_hex(data);

	// FIXME: ch seems to always be 0?

	// check status byte  
  com = (data & 0xf0000000) >> 28;
  ch  = (data & 0x0f000000) >> 24;
	switch (com) {
  case 0x9:
		// note on
  	num = (data & 0xff0000) >> 16;
  	val = (data &   0xff00) >> 8;
		if (val == 0)
			// note on with zero velocity is note off (per midi spec)
			midi_note_off(ch, num, val);
		else
			midi_note_on(ch, num, val);
		break;
	case 0x8:
		// note off (with velocity)
    num = (data & 0xff0000) >> 16;
    val = (data &   0xff00) >> 8;
		midi_note_off(ch, num, val);
		break;
	case 0xd:
		// channel pressure
		val = (data & 0x7f0000) >> 16;
		midi_channel_pressure(ch, val);
		break;
	case 0xe:
		// pitch bend
		bend = ((data & 0x00ff0000) >> 16) | ((data & 0xff00) >> 1);
		midi_pitch_bend(ch, bend);
		break;
	case 0xb:
		// control change
		num = (data & 0xff0000) >> 16;
    val = (data &   0xff00) >> 8;
		midi_control_change(ch, num, val);
		break;
	default:
		// TODO: poly pressure, program change, chanel mode *, rtc, etc
		break;
  }
}

static void handler_MidiRefresh(s32 data) {
	// print_dbg("\r\nmidi refresh: 0x");
	// print_dbg_hex(data);
}

static void es_midi_process_ii(uint8_t i, int d) {
	// for now we do nothing
	// tt clocked arp would be fun
}






// assign event handlers
static inline void assign_main_event_handlers(void) {
	app_event_handlers[ kEventFront ]	= &handler_Front;
	// app_event_handlers[ kEventTimer ]	= &handler_Timer;
	app_event_handlers[ kEventPollADC ]	= &handler_PollADC;
	app_event_handlers[ kEventKeyTimer ] = &handler_KeyTimer;
	app_event_handlers[ kEventSaveFlash ] = &handler_SaveFlash;
	app_event_handlers[ kEventClockNormal ] = &handler_ClockNormal;
	app_event_handlers[ kEventFtdiConnect ]	= &handler_FtdiConnect ;
	app_event_handlers[ kEventFtdiDisconnect ]	= &handler_FtdiDisconnect ;
	app_event_handlers[ kEventMonomeConnect ]	= &handler_MonomeConnect ;
	app_event_handlers[ kEventMonomeDisconnect ]	= &handler_None ;
	app_event_handlers[ kEventMonomePoll ]	= &handler_MonomePoll ;
	app_event_handlers[ kEventMonomeRefresh ]	= &handler_MonomeRefresh ;
	app_event_handlers[ kEventMonomeGridKey ]       = &handler_MonomeGridKey ;

	app_event_handlers[ kEventMidiConnect ]	    = &handler_MidiConnect ;
	app_event_handlers[ kEventMidiDisconnect ]  = &handler_MidiDisconnect ;
	app_event_handlers[ kEventMidiPacket ]      = &handler_MidiPacket ;
	app_event_handlers[ kEventMidiRefresh ]     = &handler_MidiRefresh ;
}

// app event loop
void check_events(void) {
	static event_t e;
	if( event_next(&e) ) {
		(app_event_handlers)[e.type](e.data);
	}
}

// flash commands
u8 flash_is_fresh(void) {
  return (flashy.fresh != FIRSTRUN_KEY);
  // flashc_memcpy((void *)&flashy.fresh, &i, sizeof(flashy.fresh),   true);
  // flashc_memset32((void*)&(flashy.fresh), fresh_MAGIC, 4, true);
  // flashc_memset((void *)nvram_data, 0x00, 8, sizeof(*nvram_data), true);
}

// write fresh status
void flash_unfresh(void) {
  flashc_memset8((void*)&(flashy.fresh), FIRSTRUN_KEY, 4, true);
}

void flash_write(void) {
	// print_dbg("\r write preset ");
	// print_dbg_ulong(preset_select);

	es.shape_on = shape_on;
	es.p_select = p_select;
	es.arp = arp;
	es.port_time = port_time;

	flashc_memcpy((void *)&flashy.es[preset_select], &es, sizeof(es), true);
	flashc_memcpy((void *)&flashy.glyph[preset_select], &glyph, sizeof(glyph), true);
	flashc_memset8((void*)&(flashy.preset_select), preset_select, 1, true);
}

void flash_read(void) {
	// print_dbg("\r\n read preset ");
	// print_dbg_ulong(preset_select);

	u8 i1,i2;

	p_select = flashy.es[preset_select].p_select;
	shape_on = flashy.es[preset_select].shape_on;
	arp = flashy.es[preset_select].arp;
	port_time = flashy.es[preset_select].port_time;

	es.edge = flashy.es[preset_select].edge;
	es.edge_fixed_time = flashy.es[preset_select].edge_fixed_time;

	for(i1=0;i1<8;i1++) {
		for(i2=0;i2<8;i2++) {
			es.cv[i1][i2] = flashy.es[preset_select].cv[i1][i2];
			es.slew[i1][i2] = flashy.es[preset_select].slew[i1][i2];;
		}
		for(i2=0;i2<16;i2++) {
			es.help[i2][i1] = flashy.es[preset_select].help[i2][i1];;
		}
	}

	for(i1=0;i1<16;i1++) {
		es.p[i1].length = flashy.es[preset_select].p[i1].length;
		es.p[i1].total_time = flashy.es[preset_select].p[i1].total_time;
		es.p[i1].loop = flashy.es[preset_select].p[i1].loop;
		es.p[i1].x = flashy.es[preset_select].p[i1].x;
		es.p[i1].y = flashy.es[preset_select].p[i1].y;

		for(i2=0;i2<128;i2++) {
			es.p[i1].e[i2].shape = flashy.es[preset_select].p[i1].e[i2].shape;
			es.p[i1].e[i2].x = flashy.es[preset_select].p[i1].e[i2].x;
			es.p[i1].e[i2].y = flashy.es[preset_select].p[i1].e[i2].y;
			es.p[i1].e[i2].interval = flashy.es[preset_select].p[i1].e[i2].interval;
		}
	}
}






////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// main

int main(void)
{
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

	init_i2c_slave(0x50);


	print_dbg("\r\n\n// earthsea! //////////////////////////////// ");
	print_dbg_ulong(sizeof(flashy));

	root_x = 15;
	root_y = 0;
	min_x = 15;
	min_y = 7;
	singled = 1;

	r_status = rOff;

	gpio_clr_gpio_pin(B00);

	monomeFrameDirty++;


	u8 i1, i2;

	if(flash_is_fresh()) {
		print_dbg("\r\nfirst run.");
		flash_unfresh();

		flashc_memset8((void*)&(flashy.preset_select), 0, 1, true);

		// clear out some reasonable defaults
 		es.edge = eStandard;
		es.edge_fixed_time = 10;
		es.port_time = 20;

		for(i1=0;i1<8;i1++) {
			for(i2=0;i2<8;i2++) {
				es.cv[i1][i2] = 0;
				es.slew[i1][i2] = 0;
			}

			for(i2=0;i2<16;i2++) {
				es.help[i2][i1] = 0;
			}
		}

		for(i1=0;i1<16;i1++) {
			es.p[i1].length = 0;
			es.p[i1].total_time = 0;
			es.p[i1].loop = 0;
		}

		// save all presets, clear glyphs
		for(i1=0;i1<8;i1++) {
			flashc_memcpy((void *)&flashy.es[i1], &es, sizeof(es), true);
			glyph[i1] = (1<<i1);
			flashc_memcpy((void *)&flashy.glyph[i1], &glyph, sizeof(glyph), true);
		}

	}
	else {
		// load from flash at startup
		
		preset_select = flashy.preset_select;
		flash_read();
		for(i1=0;i1<8;i1++)
			glyph[i1] = flashy.glyph[preset_select][i1];

		reset_hys();
		for(i1=0;i1<3;i1++) {
			aout[i1].target = aout[i1].now = es.cv[shape_on][i1];
			aout[i1].slew = es.slew[shape_on][i1];
			aout[i1].step = 0;
		}
	}


	LENGTH = 15;
	SIZE = 16;

	re = &refresh;

	process_ii = &es_process_ii;

	clock_pulse = &clock;
	// clock_external = !gpio_get_pin_value(B09);

	// setup daisy chain for two dacs
	spi_selectChip(SPI,DAC_SPI);
	spi_write(SPI,0x80);
	spi_write(SPI,0xff);
	spi_write(SPI,0xff);
	spi_unselectChip(SPI,DAC_SPI);
	
	// ensure cvTimer_callback does something
	slew_active = 1;

	timer_add(&clockTimer,6,&clockTimer_callback, NULL);
	timer_add(&cvTimer,5,&cvTimer_callback, NULL);
	timer_add(&keyTimer,51,&keyTimer_callback, NULL);
	// timer_add(&adcTimer,61,&adcTimer_callback, NULL);
	clock_temp = 10000; // out of ADC range to force tempo


	while (true) {
		check_events();
	}
}
