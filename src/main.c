//
// STC15F204EA DIY LED Clock
// Copyright 2016, Jens Jensen
//

#include "stc15.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "ds1302.h"
#include "led.h"

#define FOSC    11059200
#define BAUD    9600

// clear wdt
#define WDT_CLEAR()    (WDT_CONTR |= 1 << 4)

// alias for relay and buzzer outputs, using relay to drive led for indication of main loop status
// only for revision with stc15f204ea
#ifdef stc15f204ea
#define RELAY   P1_4
#define BUZZER  P1_5
#else // revision with stc15w408as
#define RELAY   P1_4
#define LED     P1_5
#endif

// adc channels for sensors
#define ADC_LIGHT 6
#define ADC_TEMP  7

// button switch aliases
// SW3 only for revision with stc15w408as
#ifdef stc15w408as
#define SW3     P1_4
#define S3      2
#endif
#define SW2     P3_0
#define S2      1
#define SW1     P3_1
#define S1      0

// display mode states
enum keyboard_mode {
  K_NORMAL,
  K_WAIT_S1,
  K_WAIT_S2,
  K_SET_HOUR,
  K_SET_MINUTE,
  K_SET_HOUR_12_24,
  K_SEC_DISP,
  K_TEMP_DISP,
  K_DATE_DISP,
  K_DATE_SWDISP,
  K_SET_MONTH,
  K_SET_DAY,
  K_WEEKDAY_DISP,
  K_DEBUG
};

// display mode states
enum display_mode {
  M_NORMAL,
  M_SET_HOUR_12_24,
  M_SEC_DISP,
  M_TEMP_DISP,
  M_DATE_DISP,
  M_WEEKDAY_DISP,
  M_DEBUG
};

// NMEA receive state
enum nmea_state {
	NM_UNKNOWN,
	NM_HEADER,
	NM_ZDATIME,
	NM_ZDAFRACRIONSECONDS,
	NM_ZDADAY,
	NM_ZDAMONTH,
	NM_ZDAYEAR,
	NM_ZDATZHOUR,
	NM_ZDATZMINUTE,
	NM_ZDACHECKSUM
};
volatile uint8_t zda_state = NM_UNKNOWN;
volatile uint8_t zda_state_pos = 0;
volatile uint8_t zda_checksum = 0;

#define set_zda_state(s) zda_state = s; zda_state_pos = 0;

volatile uint8_t gpstm_table[8];
volatile __bit gpstm_needupdate = 0;
volatile int8_t tz_bias_hour = 3;
volatile int8_t tz_bias_minute = 0;

/* ------------------------------------------------------------------------- */
const int month_days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

//date functions
uint16_t get_days()
{
	uint8_t year = ds_split2int(gpstm_table[DS_ADDR_YEAR]);
	uint8_t month = ds_split2int(gpstm_table[DS_ADDR_MONTH]) - 1;
	uint16_t result = (year * 365) + (year >> 2); //3650 days per year + leap years for every quad
	uint8_t m;
 	if ((year & 0x3) || month > 1) {
		//year after leap or march
		result++;
	};
	for (m = 0; m < month; m++) {
		result += month_days[m];
	};
	result += ds_split2int(gpstm_table[DS_ADDR_DAY]);
	return result;
}

void set_days(uint16_t days)
{
	uint8_t year = days / 366;
	uint16_t day = days % 366;
	uint8_t month = 0;
	uint8_t leap_years = (year >> 2) + ((year & 0x3) ? 1 : 0);
	day += year - leap_years;
	while (1) {
		uint8_t m_days = month_days[month] + ((month == 1 && (year & 0x3) == 0) ? 1 : 0);
		if (day <= m_days) {
			break;
		}
		day -= m_days;
		month++;
		if (month == 12) {
			year++;
			month = 0;
		};
	};

	gpstm_table[DS_ADDR_DAY] = ds_int2bcd(day);
	gpstm_table[DS_ADDR_MONTH] = ds_int2bcd(month + 1);
	gpstm_table[DS_ADDR_YEAR] = ds_int2bcd(year);
	gpstm_table[DS_ADDR_WEEKDAY] = (days + 4) % 7 + 1;
}

void adjust_timezone()
{
	int8_t hours = ds_split2int(gpstm_table[DS_ADDR_HOUR]);
	int8_t minutes = ds_split2int(gpstm_table[DS_ADDR_MINUTES]);
	uint16_t days = get_days();

	minutes += tz_bias_minute;
	if (minutes < 0) {
		hours--;
		minutes += 60;
	} else if(minutes >=60)	{
		hours++;
		minutes -= 60;
	};
	
	hours += tz_bias_hour;
	if (hours < 0) {
		days--;
		hours += 24;
	} else if (hours >= 24) {
		days++;
		hours -= 24;
	};

	gpstm_table[DS_ADDR_MINUTES] = ds_int2bcd(minutes);
	gpstm_table[DS_ADDR_HOUR] = ds_int2bcd(hours);
	set_days(days);
}
/* ------------------------------------------------------------------------- */


void _delay_ms(uint8_t ms)
{
  // delay function, tuned for 11.092 MHz clock
  // optimized to assembler
  ms; // keep compiler from complaining?
  __asm;
  ; dpl contains ms param value
    delay$ :
  mov	b, #8; i
    outer$ :
  mov	a, #243; j
    inner$ :
  djnz acc, inner$
    djnz b, outer$
    djnz dpl, delay$
    __endasm;
}

// GLOBALS
uint8_t  count;     // was uint16 - 8 seems to be enough
uint16_t temp;      // temperature sensor value
uint8_t  lightval;  // light sensor value
uint16_t  raw_lightval;  // light sensor value

volatile uint8_t displaycounter;
volatile uint8_t _100us_count;
volatile uint8_t _10ms_count;

uint8_t dmode = M_NORMAL;     // display mode state
uint8_t kmode = K_NORMAL;
uint8_t smode, lmode;

volatile __bit  display_colon;         // flash colon
__bit  flash_01;
__bit  flash_23;
__bit  beep = 1;

volatile __bit  S1_LONG;
volatile __bit  S1_PRESSED;
volatile __bit  S2_LONG;
volatile __bit  S2_PRESSED;
volatile __bit  S3_LONG;
volatile __bit  S3_PRESSED;

volatile uint8_t debounce[3];      // switch debounce buffer
volatile uint8_t switchcount[3];
#define SW_CNTMAX 80

void timer0_isr() __interrupt 1 __using 1
{
  // display refresh ISR
  // cycle thru digits one at a time
  uint8_t digit = displaycounter % 4;

  // turn off all digits, set high    
  P3 |= 0x3C;

  // auto dimming, skip lighting for some cycles
  if (displaycounter % lightval < 4) {
    // fill digits
    P2 = dbuf[digit];
    // turn on selected digit, set low
    P3 &= ~(0x4 << digit);
  }
  displaycounter++;

  //  divider: every 10ms
  if (++_100us_count == 100) {
    _100us_count = 0;
    _10ms_count++;

    // colon blink stuff, 500ms
    if (_10ms_count == 50) {
      display_colon = !display_colon;
      _10ms_count = 0;
    }

    // switch read, debounce:
    // increment count if settled closed
    if ((debounce[0]) == 0x00) {
      // down for at least 8 ticks
      S1_PRESSED = 1;
      switchcount[0]++;
    } else {
      // released or bounced, reset state            
      S1_PRESSED = 0;
      switchcount[0] = 0;
    }

    if ((debounce[1]) == 0x00) {
      // down for at least 8 ticks            
      S2_PRESSED = 1;
      switchcount[1]++;
    } else {
      // released or bounced, reset state
      S2_PRESSED = 0;
      switchcount[1] = 0;
    }

#ifdef stc15w408as
    if ((debounce[2]) == 0x00) {
      // down for at least 8 ticks            
      S3_PRESSED = 1;
      switchcount[2]++;
    } else {
      // released or bounced, reset state
      S3_PRESSED = 0;
      switchcount[2] = 0;
    }
#endif

    // debouncing stuff
   // keep resetting halfway if held long
    if (switchcount[0] > SW_CNTMAX)
    {
      switchcount[0] = SW_CNTMAX; S1_LONG = 1;
    }
    if (switchcount[1] > SW_CNTMAX)
    {
      switchcount[1] = SW_CNTMAX; S2_LONG = 1;
    }
#ifdef stc15w408as
    if (switchcount[2] > SW_CNTMAX)
    {
      switchcount[2] = SW_CNTMAX; S3_LONG = 1;
    }
#endif

    // read switch positions into sliding 8-bit window
    debounce[0] = (debounce[0] << 1) | SW1;
    debounce[1] = (debounce[1] << 1) | SW2;
#ifdef stc15w408as
    debounce[2] = (debounce[2] << 1) | SW3;
#endif
  }
}

void Timer0Init(void)		//100us @ 11.0592MHz
{
  TL0 = 0xA4;		//Initial timer value
  TH0 = 0xFF;		//Initial timer value
  TF0 = 0;		//Clear TF0 flag
  TR0 = 1;		//Timer0 start run
  ET0 = 1;        // enable timer0 interrupt
  EA = 1;         // global interrupt enable
}

void processUartData(uint8_t data) 
{
	if (gpstm_needupdate == 1) {
		//if local time still not updated with gps time 
		return;
	}
	if (data == '$') {
		set_zda_state(NM_HEADER);
		//set to * so last * annihilate it
		zda_checksum = '*';
	} else if (zda_state == NM_UNKNOWN) {
		//not in interesting state
		return;
	} else if (zda_state == NM_ZDACHECKSUM) {
		uint8_t digit;
		if (data >= '0' && data <= '9') {
			digit = data - '0';
		} else if (data >= 'A' && data <= 'F') {
			digit = data - 'A' + 10;
		} else {
			set_zda_state(NM_UNKNOWN);
			return;
		}
		if (zda_state_pos == 0) {
			zda_checksum ^= digit << 4;
			zda_state_pos = 1;
		} else {
			//completed
			if (zda_checksum == digit) {
				gpstm_needupdate = 1;
			}
			set_zda_state(NM_UNKNOWN);
		}

	} else {
		zda_checksum ^= data;
		if (zda_state == NM_HEADER) {
			if (zda_state_pos == 0 && data == 'G') {
			} else if (zda_state_pos == 1 && data == 'P') {
			} else if (zda_state_pos == 2 && data == 'Z') {
			} else if (zda_state_pos == 3 && data == 'D') {
			} else if (zda_state_pos == 4 && data == 'A') {
			} else if (zda_state_pos == 5 && data == ',') {
				set_zda_state(NM_ZDATIME);
				return;
			} else {
				set_zda_state(NM_UNKNOWN);
				return;
			}
			zda_state_pos++;
		} else if (zda_state == NM_ZDATIME) {
			if (data == '.' && zda_state_pos == 6) {
				set_zda_state(NM_ZDAFRACRIONSECONDS);
				return;
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
				return;
			} else {
				uint8_t time_part;
				if (zda_state_pos < 2) {
					time_part = DS_ADDR_HOUR;
				} else if (zda_state_pos < 4) {
					time_part = DS_ADDR_MINUTES;
				} else if (zda_state_pos < 6) {
					time_part = DS_ADDR_SECONDS;
				} else {
					set_zda_state(NM_UNKNOWN);
					return;
				};
				if ((zda_state_pos & 0x01) == 0) {
					gpstm_table[time_part] = (data - '0') << 4;
				} else {
					gpstm_table[time_part] = gpstm_table[time_part] + (data - '0');
				}
				zda_state_pos++;
			}
		} else if (zda_state == NM_ZDAFRACRIONSECONDS) {
			if (data == ',') {
				set_zda_state(NM_ZDADAY);
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
			}
		} else if (zda_state == NM_ZDADAY) {
			if (data == ',') {
				set_zda_state(NM_ZDAMONTH);
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
			} else {
				if (zda_state_pos == 0) {
					gpstm_table[DS_ADDR_DAY] = (data - '0') << 4;
				} else if (zda_state_pos == 1) {
					gpstm_table[DS_ADDR_DAY] = gpstm_table[DS_ADDR_DAY] + (data - '0');
				} else {
					set_zda_state(NM_UNKNOWN);
					return;
				}
				zda_state_pos++;
			}
		} else if (zda_state == NM_ZDAMONTH) {
			if (data == ',') {
				set_zda_state(NM_ZDAYEAR);
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
			} else {
				if (zda_state_pos == 0) {
					gpstm_table[DS_ADDR_MONTH] = (data - '0') << 4;
				} else if (zda_state_pos == 1) {
					gpstm_table[DS_ADDR_MONTH] = gpstm_table[DS_ADDR_MONTH] + (data - '0');
				} else {
					set_zda_state(NM_UNKNOWN);
					return;
				}
				zda_state_pos++;
			}
		} else if (zda_state == NM_ZDAYEAR) {
			if (data == ',') {
				set_zda_state(NM_ZDATZHOUR);
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
			} else {
				if (zda_state_pos < 2) {
					//centuries, skip
				} else if (zda_state_pos == 2) {
					gpstm_table[DS_ADDR_YEAR] = (data - '0') << 4;
				} else if (zda_state_pos == 3) {
					gpstm_table[DS_ADDR_YEAR] = gpstm_table[DS_ADDR_YEAR] + (data - '0');
				} else {
					set_zda_state(NM_UNKNOWN);
					return;
				}
				zda_state_pos++;
			}
		} else if (zda_state == NM_ZDATZHOUR) {
			if (data == ',') {
				set_zda_state(NM_ZDATZMINUTE);
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
			}
		} else if (zda_state == NM_ZDATZMINUTE) {
			if (data == '*') {
				set_zda_state(NM_ZDACHECKSUM);
			} else if (data < '0' || data > '9') {
				set_zda_state(NM_UNKNOWN);
			}
		}
	}
}

void uart() __interrupt 4 __using 1
{
	if (RI) {
		RI = 0;
		processUartData(SBUF);
	}
	if (TI) {
		TI = 0; //clear TI flag
	}

}

void checkDateNeedAdjust() {
	//to prevent need time rolling, check only if seconds between 30 and 40
	if (rtc_table[DS_ADDR_SECONDS] > 0x30 && rtc_table[DS_ADDR_SECONDS] < 0x40) {
		int8_t part_delta;
		//adjust from gmt to local timezone
		adjust_timezone();
		
		part_delta = (rtc_table[DS_ADDR_SECONDS] >> 4 - gpstm_table[DS_ADDR_SECONDS] >> 4) * 10 + (rtc_table[DS_ADDR_SECONDS] & 0xF - gpstm_table[DS_ADDR_SECONDS] & 0xF);
		if (H12_24) {
			uint8_t hours = ds_split2int(rtc_table[DS_ADDR_HOUR] & DS_MASK_HOUR24); // hours in 24h format (0-23, 0-11=>am , 12-23=>pm)
			uint8_t b = DS_MASK_AMPM_MODE;
			if (hours >= 12) { hours -= 12; b |= 0x20; }	// pm
			if (hours == 0) { hours = 12; } 		//12am
			gpstm_table[DS_ADDR_HOUR] = b | ds_int2bcd(hours);
		}
		if (
			(part_delta > 2 || part_delta < -2)	//2 second difference, need adjustment
			|| rtc_table[DS_ADDR_MINUTES] != gpstm_table[DS_ADDR_MINUTES]
			|| rtc_table[DS_ADDR_HOUR] != gpstm_table[DS_ADDR_HOUR]
			|| rtc_table[DS_ADDR_DAY] != gpstm_table[DS_ADDR_DAY]
			|| rtc_table[DS_ADDR_MONTH] != gpstm_table[DS_ADDR_MONTH]
			|| rtc_table[DS_ADDR_YEAR] != gpstm_table[DS_ADDR_YEAR]
			) {

			//update date
			ds_writebyte(DS_ADDR_SECONDS, gpstm_table[DS_ADDR_SECONDS]);
			ds_writebyte(DS_ADDR_MINUTES, gpstm_table[DS_ADDR_MINUTES]);
			ds_writebyte(DS_ADDR_HOUR, gpstm_table[DS_ADDR_HOUR]);
			ds_writebyte(DS_ADDR_DAY, gpstm_table[DS_ADDR_DAY]);
			ds_writebyte(DS_ADDR_MONTH, gpstm_table[DS_ADDR_MONTH]);
			ds_writebyte(DS_ADDR_YEAR, gpstm_table[DS_ADDR_YEAR]);

		} else {
			//update not needed
		}
	}
	gpstm_needupdate = 0;


}

#define getkeypress(a) a##_PRESSED

void update_temp(){
	uint16_t newtemp = getADCResult(ADC_TEMP);
	//adjust temperature
	newtemp = 76 - newtemp * 64 / 637;
  temp = newtemp + (cfg_table[CFG_TEMP_BYTE] & CFG_TEMP_MASK) - 4;
	
}

void update_lightval(){
	uint16_t new_lightval = getADCResult8(ADC_LIGHT);
	if(new_lightval < raw_lightval){
		//dim instantly
		raw_lightval = new_lightval << 8;
	}else{
		//slowly increase light
		raw_lightval += new_lightval << 6 - raw_lightval >> 2;
	};
	
	if (raw_lightval <= 32 * 256) {
		lightval = 4;
	} else if (raw_lightval <= 128 * 256) {
		//div by 8
		lightval = raw_lightval >> 11;
	} else {
		//adjust
		lightval = (raw_lightval >> 8) - 112;
	}

}

/*********************************************/
int main()
{
  // SETUP
  // set photoresistor & ntc pins to open-drain output
  P1M1 |= (1 << 6) | (1 << 7);
  P1M0 |= (1 << 6) | (1 << 7);

  //set UART pins @ 3.6 & 3.7
  P_SW1 = P_SW1 & ~0xC0 | 0x40;
  //no parity
  SCON = 0x50;
  //Set port speed
  T2L = (65536 - (FOSC / 4 / BAUD));
  T2H = (65536 - (FOSC / 4 / BAUD))>>8;
  //
  AUXR = 0x15;
  //enable interrupt
  ES = 1;

  // init rtc
  ds_init();
  // init/read ram config
  ds_ram_config_init();

  // uncomment in order to reset minutes and hours to zero.. Should not need this.
  //ds_reset_clock();    

  Timer0Init(); // display refresh & switch read

  // LOOP
  while (1)
  {

    //RELAY = 0;
    _delay_ms(100);
    //RELAY = 1;

    // sample adc, run frequently
    if ((count % 4) == 0) {
			//update temperature value
      update_temp();

      // auto-dimming
			update_lightval();
    }

    ds_readburst(); // read rtc
		if (gpstm_needupdate == 1) {
			checkDateNeedAdjust();
		}

    // keyboard decision tree
    switch (kmode) {

    case K_SET_HOUR:
      flash_01 = !flash_01;
      if (!flash_01) {
        if (getkeypress(S2)) ds_hours_incr();
        if (getkeypress(S1)) kmode = K_SET_MINUTE;
      }
      break;

    case K_SET_MINUTE:
      flash_01 = 0;
      flash_23 = !flash_23;
      if (!flash_23) {
        if (getkeypress(S2)) ds_minutes_incr();
        if (getkeypress(S1)) kmode = K_SET_HOUR_12_24;
      }
      break;

    case K_SET_HOUR_12_24:
      dmode = M_SET_HOUR_12_24;
      if (getkeypress(S2)) ds_hours_12_24_toggle();
      if (getkeypress(S1)) kmode = K_NORMAL;
      break;

    case K_TEMP_DISP:
      dmode = M_TEMP_DISP;
      if (getkeypress(S1))
      {
        uint8_t offset = cfg_table[CFG_TEMP_BYTE] & CFG_TEMP_MASK;
        offset++; offset &= CFG_TEMP_MASK;
        cfg_table[CFG_TEMP_BYTE] = (cfg_table[CFG_TEMP_BYTE] & ~CFG_TEMP_MASK) | offset;
      }
      if (getkeypress(S2)) kmode = K_DATE_DISP;
      break;

    case K_DATE_DISP:
      dmode = M_DATE_DISP;
      if (getkeypress(S1)) { kmode = K_WAIT_S1; lmode = CONF_SW_MMDD ? K_SET_DAY : K_SET_MONTH; smode = K_DATE_SWDISP; }
      if (getkeypress(S2)) kmode = K_WEEKDAY_DISP;
      break;

    case K_DATE_SWDISP:
      CONF_SW_MMDD = !CONF_SW_MMDD;
      kmode = K_DATE_DISP;
      break;

    case K_SET_MONTH:
      flash_01 = !flash_01;
      if (!flash_01) {
        if (getkeypress(S2)) { ds_month_incr(); }
        if (getkeypress(S1)) { flash_01 = 0; kmode = CONF_SW_MMDD ? K_DATE_DISP : K_SET_DAY; }
      }
      break;

    case K_SET_DAY:
      flash_23 = !flash_23;
      if (!flash_23) {
        if (getkeypress(S2)) { ds_day_incr(); }
        if (getkeypress(S1)) {
          flash_23 = 0; kmode = CONF_SW_MMDD ? K_SET_MONTH : K_DATE_DISP;
        }
      }
      break;

    case K_WEEKDAY_DISP:
      dmode = M_WEEKDAY_DISP;
      if (getkeypress(S1)) ds_weekday_incr();
      if (getkeypress(S2)) kmode = K_NORMAL;
      break;

    case K_DEBUG:
      dmode = M_DEBUG;
      if (count > 100) kmode = K_NORMAL;
      if (S1_PRESSED || S2_PRESSED) count = 0;
      break;

    case K_SEC_DISP:
      dmode = M_SEC_DISP;
      if (getkeypress(S1) || (count > 100)) { kmode = K_NORMAL; }
      if (getkeypress(S2)) { ds_sec_zero(); }
      break;

    case K_WAIT_S1:
      count = 0;
      if (!S1_PRESSED) {
        if (S1_LONG) { S1_LONG = 0; kmode = lmode; } else { kmode = smode; }
      }
      break;

    case K_WAIT_S2:
      count = 0;
      if (!S2_PRESSED) {
        if (S2_LONG) { S2_LONG = 0; kmode = lmode; } else { kmode = smode; }
      }
      break;

    case K_NORMAL:
    default:
      flash_01 = 0;
      flash_23 = 0;

      dmode = M_NORMAL;

      if (S1_PRESSED) { kmode = K_WAIT_S1; lmode = K_SET_HOUR; smode = K_SEC_DISP; }
      //if (S2_PRESSED) { kmode = K_WAIT_S2; lmode=K_DEBUG;    smode=K_TEMP_DISP; }
      if (S2_PRESSED) { kmode = K_TEMP_DISP; }
#ifdef stc15w408as
      if (!S3_PRESSED) {
        if (S3_LONG) { S3_LONG = 0; LED = !LED; }
      }
#endif

    };

    // display execution tree

    clearTmpDisplay();

    switch (dmode) {
    case M_NORMAL:
      if (flash_01) {
        dotdisplay(1, display_colon);
      } else {
        if (!H12_24) {
          filldisplay(0, (rtc_table[DS_ADDR_HOUR] >> 4)&(DS_MASK_HOUR24_TENS >> 4), 0);	// tenhour 
        } else {
          if (H12_TH) filldisplay(0, 1, 0);	// tenhour in case AMPM mode is on, then '1' only is H12_TH is on
        }
        filldisplay(1, rtc_table[DS_ADDR_HOUR] & DS_MASK_HOUR_UNITS, display_colon);
      }

      if (flash_23) {
        dotdisplay(2, display_colon);
        dotdisplay(3, H12_24&H12_PM);	// dot3 if AMPM mode and PM=1
      } else {
        filldisplay(2, (rtc_table[DS_ADDR_MINUTES] >> 4)&(DS_MASK_MINUTES_TENS >> 4), display_colon);	//tenmin
        filldisplay(3, rtc_table[DS_ADDR_MINUTES] & DS_MASK_MINUTES_UNITS, H12_24 & H12_PM);  		//min
      }
      break;

    case M_SET_HOUR_12_24:
      if (!H12_24) {
        filldisplay(1, 2, 0); filldisplay(2, 4, 0);
      } else {
        filldisplay(1, 1, 0); filldisplay(2, 2, 0);
      }
      filldisplay(3, LED_h, 0);
      break;

    case M_SEC_DISP:
      dotdisplay(0, display_colon);
      dotdisplay(1, display_colon);
      filldisplay(2, (rtc_table[DS_ADDR_SECONDS] >> 4)&(DS_MASK_SECONDS_TENS >> 4), 0);
      filldisplay(3, rtc_table[DS_ADDR_SECONDS] & DS_MASK_SECONDS_UNITS, 0);
      break;

    case M_DATE_DISP:
      if (flash_01) {
        dotdisplay(1, 1);
      } else {
        if (!CONF_SW_MMDD) {
          filldisplay(0, rtc_table[DS_ADDR_MONTH] >> 4, 0);	// tenmonth ( &MASK_TENS useless, as MSB bits are read as '0')
          filldisplay(1, rtc_table[DS_ADDR_MONTH] & DS_MASK_MONTH_UNITS, 1);
        } else {
          filldisplay(2, rtc_table[DS_ADDR_MONTH] >> 4, 0);	// tenmonth ( &MASK_TENS useless, as MSB bits are read as '0')
          filldisplay(3, rtc_table[DS_ADDR_MONTH] & DS_MASK_MONTH_UNITS, 0);
        }
      }
      if (!flash_23) {
        if (!CONF_SW_MMDD) {
          filldisplay(2, rtc_table[DS_ADDR_DAY] >> 4, 0);		      // tenday   ( &MASK_TENS useless)
          filldisplay(3, rtc_table[DS_ADDR_DAY] & DS_MASK_DAY_UNITS, 0);
        }     // day       
        else {
          filldisplay(0, rtc_table[DS_ADDR_DAY] >> 4, 0);		      // tenday   ( &MASK_TENS useless)
          filldisplay(1, rtc_table[DS_ADDR_DAY] & DS_MASK_DAY_UNITS, 1);
        }     // day       
      }
      break;

    case M_WEEKDAY_DISP:
      filldisplay(1, LED_DASH, 0);
      filldisplay(2, rtc_table[DS_ADDR_WEEKDAY], 0);		//weekday ( &MASK_UNITS useless, all MSBs are '0')
      filldisplay(3, LED_DASH, 0);
      break;

    case M_TEMP_DISP:
      filldisplay(0, ds_int2bcd_tens(temp), 0);
      filldisplay(1, ds_int2bcd_ones(temp), 0);
      filldisplay(2, CONF_C_F ? LED_f : LED_c, 1);
      // if (temp<0) filldisplay( 3, LED_DASH, 0);  -- temp defined as uint16, cannot be <0
      break;

    case M_DEBUG:
      filldisplay(0, switchcount[0] >> 4, S1_LONG);
      filldisplay(1, switchcount[0] & 15, S1_PRESSED);
      filldisplay(2, switchcount[1] >> 4, S2_LONG);
      filldisplay(3, switchcount[1] & 15, S2_PRESSED);
      break;
    }

    __critical{ updateTmpDisplay(); }

      // save ram config
    ds_ram_config_write();

    if (S1_PRESSED || S2_PRESSED && !(S1_LONG || S2_LONG)) {
      // try to dampen button over-response
      _delay_ms(100);
    }

    // reset long presses when button released
    if (!S1_PRESSED && S1_LONG) {
      S1_LONG = 0;
    }
    if (!S2_PRESSED && S2_LONG) {
      S2_LONG = 0;
    }

    count++;
    WDT_CLEAR();
  }
}
/* ------------------------------------------------------------------------- */
