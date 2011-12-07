#define	AT2SO	1
/*
 * what: rx2at - RC receiver servo signal to ASCII conversion
 * who:	 miru
 * when: April 2011 ... November 2011
 */
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
const char version[] PROGMEM = "@(#) rx2at 0.11 20111106";

/* switch setup, S_GEAR-(2 pos), S_AUX1-(2 or 3 pos) *
 * S_LAND S_FMOD
 * S_AUX1 S_AUX1 one 3 position switch (e.g. DX7)
 * S_GEAR S_GEAR one 2 position switch
 * S_GEAR S_AUX1 two switches (e.g. DX6)
 * S_AUX1 S_GEAR two switches */
#define	S_LAND	S_AUX1
#define	S_FMOD	S_AUX1

/* transmitter mode, channel assignments of the sticks
 * T_MODE 1 -> left: V-ELEV H-RUDD right: V-THRO H-AILE
 * T_MODE 4 -> left: V-THRO H-AILE right: V-ELEV H-RUDD
 * T_MODE 2 -> left: V-THRO H-RUDD right: V-ELEV H-AILE, US common mode
 * T_MODE 3 -> left: V-ELEV H-AILE right: V-THRO H-RUDD, US reversed */
#define	T_MODE	2

/* drone configuration choices
 * outdoor:   TRUE or FALSE
 * no_shell:  TRUE or FALSE
 * max_euler: 0 ... 0.52 max pitch/roll angle [rad]
 * max_vz:    200 ... 2000 max climb speed [mm/s]
 * max_yaw    0.7 ... 6.11 max yaw speed [rad/s]
 * max_alt:   500 ... 5000 altitude limit [mm], 10000 is OFF
 *           outdoor,no_shell,max_euler,max_vz,max_yaw,max_alt */
const char cfg1[] PROGMEM = "TRUE,TRUE,0.35,1500,3.5,10000";	/* S standard */
const char cfg2[] PROGMEM = "TRUE,TRUE,0.52,2000,6.1,10000";	/* W wild (max) */
const char cfg3[] PROGMEM = "FALSE,FALSE,0.21,700,1.75,2000";	/* N normal */
const char cfg4[] PROGMEM = "FALSE,FALSE,0.10,700,1.50,2000";	/* E easy */

/* Visible Low Battery Alert
 * the program can watch the battery capacity (percentage) on the drone, if it
 * goes below VLBA_THR the VLBA mechanism triggers. The drone's LEDs flash in RED
 * and the VLBA gets activated (blinking or steady), set to 0 to turn VLBA off */
#define	VLBA_THR	15	/* 0 or > 60 turns it off */
#define	VLBA_POL	1	/* 1-active high, 0-active low */
#define	VLBA_BLINK	1	/* 1-blink, 0-no blink */

/* Receiver signal loss input
 * some RX/TX setups don't implement a recognizable 'failsafe', however in some
 * cases one can monitor the signal to the 'connected' LED to recognize a TX loss.
 * If you want to use this feature, connect the signal to the pin labelled A3 and
 * enable the feature below.
 */
#define	FSAF_ENA	-1	/* -1 disable, 0 TX loss A3=0, 1 TX loss A3=1 */

#define	BAUD_GPS	57600	/* GPS serial speed (max 57600!) */

/* companion program arguments
 * -t      - allow telnet connections to drone
 * -f      - allow ftp write on drone (used to upload new version by iDev) */
#define	AT2SOARGS	"-t"

#define	LOOPHZ		25	/* loop frequency */

void port_setup(void)
{
	PORTB = 0b00001100;	/* enable pull-up for SETUP inputs */
/* (label 13 on Arduino) comes with a LED on this pin */
#define	LEDON()	PORTB |=  _BV(5)
#define	LEDOF()	PORTB &= ~_BV(5)
#define	LEDTG()	PORTB ^=  _BV(5)
	DDRB  |= _BV(5);	/* output */

/* (label 11 on Arduino) ground for RX debug with terminal emulator */
/* (label 10 on Arduino) ground for RX debug with terminal emulator */
#define	SETUP() (((PINB & _BV(2)) == 0) || ((PINB & _BV(3)) == 0))

/* (label  9 on Arduino) VLBA output pin */
#if	VLBA_POL == 1
#define	VLBON()	PORTB |=  _BV(1)
#define	VLBOF()	PORTB &= ~_BV(1)
#else
#define	VLBOF()	PORTB |=  _BV(1)
#define	VLBON()	PORTB &= ~_BV(1)
#endif
#define	VLBTG()	PORTB ^=  _BV(1)
	DDRB  |= _BV(1);	/* output */

	PORTC = 0b00000000;
	DDRC  = 0b00000000;
/* (label A0 on Arduino) GPS serial port in */
#define	SSI_RXI	_BV(0)
/* (label A1 on Arduino) GPS serial port out */
#define	SSI_TXO	_BV(1)
	PORTC |=  SSI_RXI;	/* enable pullup on RXI */
	DDRC  &= ~SSI_RXI;	/* RXI is input */
	PORTC |=  SSI_TXO;	/* set TXO idle */
	DDRC  |=  SSI_TXO;	/* TXO is output */
/* (label A3 on Arduino) Tx signal loss input */
#if	FSAF_ENA >= 0
	PORTC |= _BV(3);	/* enable pullup on FSAF input */
#endif
#define	TXDSC()	(FSAF_ENA >= 0 && ((PINC&_BV(3))^(FSAF_ENA?0:_BV(3))))

	PORTD = PCMSK2;   	/* enable pull-ups for receiver inputs */
	DDRD  = 0b00000000;
	MCUCR = 0b00000000;	/* PUD=0 clear global pull-up disable */
}

void uart_setup(void)
{
	UCSR0B = 0b00000000;	/* rxcie0,txcie0,udrie0,rxen0,txen0,ucsz02,rxb80,txb80 */
	UCSR0C = 0b00000110;	/* umsel01,umsel00,upm01,upm00,usbs0,UCSZ01,UCSZ00,ucpol0 */
	UCSR0A = 0b00000000;	/* rxc0,txc0,udre0,fe0,dor0,upe0,u2x0,mpcm0 */
#if	F_CPU == 16000000UL	/* (see NOTES USART) */
	UBRR0  = 16;
	UCSR0A |= _BV(U2X0);
#else
	UBRR0  = (F_CPU/(16UL*115200UL)-1);
#endif
}

void uart_38400(void)
{
	UCSR0B = 0b00000000;	/* rxcie0,txcie0,udrie0,rxen0,txen0,ucsz02,rxb80,txb80 */
	UCSR0C = 0b00000110;	/* umsel01,umsel00,upm01,upm00,usbs0,UCSZ01,UCSZ00,ucpol0 */
	UBRR0  = (F_CPU/(16UL*38400UL)-1);
	UCSR0A = 0b00000000;	/* rxc0,txc0,udre0,fe0,dor0,upe0,U2X0,mpcm0 */
}

void tmr0_setup(void)
{
#define	CMAX	250
#if	(F_CPU/(1L*BAUD_GPS)) < CMAX /* F_CPU=16Mhz 76800, 115200 ...*/
	TCCR0B = 0b00000001;	/* icnc1,ices1,-,wgm03,wgm02,cs02,cs01,CS00  F_CPU/1 */
#define	TMR0PRSC	1L
#elif	(F_CPU/(8L*BAUD_GPS)) < CMAX /* F_CPU=16Mhz 9600, 19200, 38400, 57600 ...*/
	TCCR0B = 0b00000010;	/* icnc1,ices1,-,wgm03,wgm02,cs02,CS01,cs00  F_CPU/8 */
#define	TMR0PRSC	8L
#elif	(F_CPU/(64L*BAUD_GPS)) < CMAX /* F_CPU=16Mhz 1200, 2400, 4800 */
	TCCR0B = 0b00000011;	/* icnc1,ices1,-,wgm03,wgm02,cs02,CS01,CS00  F_CPU/64 */
#define	TMR0PRSC	64L
#elif	(F_CPU/(256L*BAUD_GPS)) < CMAX
	TCCR0B = 0b00000100;	/* icnc1,ices1,-,wgm03,wgm02,CS02,cs01,cs00  F_CPU/256 */
#define	TMR0PRSC	256L
#else	/* crawling... */
	TCCR0B = 0b00000101;	/* icnc1,ices1,-,wgm03,wgm02,CS02,cs01,CS00  F_CPU/1024 */
#define	TMR0PRSC	1024L
#endif
#undef	CMAX
#define	TMR0CNT	(F_CPU/(TMR0PRSC*BAUD_GPS))
#if	TMR0CNT < 80
#define	TMR0HBT	((TMR0CNT*1L)/10L)
#else
#define	TMR0HBT	((TMR0CNT*3L)/10L)
#endif
	OCR0A  = TMR0CNT - 1;
	TIMSK0 = 0b00000000;	/* -,-,-,-,-,ocie0b,ocie0a,toie0 */
	TCCR0A = 0b00000010;	/* com0a1,com0a0,com0b1,com0b0,-,-,WGM01,wgm00  CTC mode */
}

void tmr1_setup(void)
{
#define	F_TM1	(F_CPU/8UL)	/* (see NOTES TIMER1) */
	TCCR1A = 0b00000000;	/* com1a1,com1a0,com1b1,com1b0,-,-,wgm11,wgm10 */
	TCCR1B = 0b00000010;	/* icnc1,ices1,-,wgm13,wgm12,cs12,CS11,cs10 F_CPU/8 */
	TCCR1C = 0b00000000;	/* foc1a,foc1b,-,-,-,-,-,- */
	TIMSK1 = 0b00000001;	/* -,-,icie1,-,-,ocie1b,ocie1a,TOIE1 */
}

#define	NEL(x)	(sizeof(x)/sizeof(x[0]))
#define	ABS(x)	((x)>=0?(x):-(x))

typedef unsigned char	u08_t;
typedef volatile u08_t	v08_t;
typedef unsigned short	u16_t;
typedef volatile u16_t	v16_t;
typedef unsigned long	u32_t;

typedef struct {
	u32_t	dcst;	/* drone status */
	u08_t	cbat;	/* battery percentage (is actually signed) */
}	__attribute__ ((packed)) m2a_t;

typedef struct {	/* radio signal sample */
	u08_t	smp;
	u32_t	tmr;
}	__attribute__ ((packed)) rsm_t;

struct {
	v16_t	t1ov;	/* timer 1 overflow count */

	struct { /* UART is connected to drone */
		/* circular receive buffer (r == w -> empty) */
		v08_t	rxw;
		v08_t	rxr;
		u08_t	rxb[32];
		/* circular transmit buffer (r == w -> empty) */
		v08_t	txw;
		v08_t	txr;
		u08_t	txb[200];
	}	sio;

	struct { /* messages from drone */
		v08_t	ack;
		v08_t	cst;
		v08_t	nda;
		union {
			u08_t	b[0];
			m2a_t	m2a;
		}	dat;
	}	drm;

	struct { /* 'soft' serial port for GPS */
		/* receiver */
		v08_t	rxi;	/* last value of input SSI_RXI */
		u08_t	rxm;	/* data bit mask register */
		u08_t	rxd;	/* data shift register */
		v08_t	rxw;
		v08_t	rxr;
		u08_t	rxb[BAUD_GPS/(10L*LOOPHZ)+9];
		/* transmitter */
		u08_t	txm;
		u08_t	txd;
		v08_t	txw;
		v08_t	txr;
		u08_t	txb[64];
	} 	ssi;
	struct { /* GPS data assembly */
		u08_t	cst;
		u08_t	cks;
		char	typ;
#define	GP_GGA	1
#define	GP_RMC	2
#define	GP_VTG	3
#define	GP_GSA	4
#define	GP_GSV	5
#define	GP_GLL	6
		u08_t	nda;
		char	dat[90];
		char	upd;
		char	loc[64];
	}	gps;

	struct {
		u08_t	stat;
		u08_t	tick;
		char	cbat;
		u08_t	mcst;
		u08_t	mctk;
		u08_t	mcsy;
#define	MC_R	0x13
	}	vlba;

	struct { /* RC receiver signals connected to port D */
		u08_t	fms;	/* flight mode (rx_read()) */
		u08_t	fsq;	/* flight mode sequencer */
		u08_t	fst;	/* flight mode sequencer tic */
#define	FMS_LAND	0
#define	FMS_FM_1	1	/* hover on ROL/PTC sticks centered */
#define	FMS_FM_2	2	/* idle up, no hovering on ROL/PTC sticks centered */
#define	FMS_FM_3	3	/* GPS position hold */
		u08_t	eft;	/* signals with frame time error */
		u08_t	ept;	/* signals with pulse time error */
		u08_t	smp;	/* last signal sample */
		u08_t	rsr;	/* radio sampling buffer */
		u08_t	rsw;
		rsm_t	rsb[32];
#define	S_AILE	0
#define	S_ELEV	1
#define	S_THRO	2
#define	S_RUDD	3
#define	S_AUX1	4
#define	S_GEAR	5
#define	S_NCHN	6
#if   T_MODE == 1 || T_MODE == 4
	#define	SH_THRO	S_AILE
	#define	SH_ELEV	S_RUDD
#else
	#define	SH_THRO	S_RUDD
	#define	SH_ELEV	S_AILE
#endif
		struct chn {
			u08_t	msk; /* port D mask for signal */
			u32_t	tup; /* timer1 when it went up */
			u32_t	ftm; /* time between the last two ups */
			int	dbn; /* signal dead band */
			int	dur; /* how long signal was up */
			int	val; /* interpreted value (rx_read()) */
		}	chn[S_NCHN];
	}	rxs;
/* mapping to drone */
#define S_ROL	S_AILE
#define S_PTC	S_ELEV
#define S_GAZ	S_THRO
#define S_YAW	S_RUDD
#define	S_SIG	(_BV(S_ROL)|_BV(S_PTC)|_BV(S_GAZ)|_BV(S_YAW)|_BV(S_LAND)|_BV(S_FMOD))

	v16_t	dcnt;	/* delay count for ms_dly() */

	/* drone program in flash */
	struct {
		u16_t	adr;
		u16_t	siz;
		u16_t	cks;
	}	arm;

	char	pad;	/* drone control <0 emergency, >0fly, else land */
	u08_t	eewen;	/* EEPROM */
	union {
		u08_t	eedat[4];
		struct {
			u16_t	eemag;
			u08_t	cfg;
			u08_t	eecks;
		};
	};
#define	EE_MAG	0x0728
}	gl;

ISR(TIMER1_OVF_vect)
{
	gl.t1ov++;
}

inline void itic(u16_t *t)
{
	t[1] = gl.t1ov;
	t[0] = TCNT1;
	if ((TIFR1 & _BV(TOV1)) && t[0] < 0xffff) t[1]++;
}

u32_t tic(void)
{
	union { u16_t uw[2]; u32_t ul; } t;
	u08_t	srg;

	srg = SREG; cli();
	itic(t.uw);
	SREG = srg;
	return t.ul;
}

void bzero(void *d, int n)
{
	u08_t	*b = (u08_t *)d;

	while (--n >= 0) *b++ = 0;
}

void ms_dly(int ndcnt)
{
	int	i;

	for (; ndcnt; ndcnt--)
		for (i = gl.dcnt; --i >= 0; )
			__asm__ __volatile__ (" nop");
}

void msdly_cali(void)
{
	u32_t	tmr;

	gl.dcnt = 2000;
	tmr = tic();
	ms_dly(100);
	tmr = tic() - tmr;
	gl.dcnt = (int)((200UL*F_TM1)/tmr);
}

void blip(unsigned char nb)
{
	while (nb--) {
		LEDON(); ms_dly( 20);
		LEDOF(); ms_dly(180);
	}
}

/*
 * 'soft' serial port for GPS on PORTC
 */
ISR(TIMER0_COMPA_vect)
{
	u08_t	i;

	if (gl.ssi.txm == 0) {
		if (PORTC & SSI_TXO) { /* was stop bit or pause */
			if (gl.ssi.txr != gl.ssi.txw) {
				gl.ssi.txd = gl.ssi.txb[i = gl.ssi.txr];
				gl.ssi.txr = ++i >= NEL(gl.ssi.txb) ? 0 : i;
				PORTC &= ~SSI_TXO;	/* send start bit */
			}
			else TIMSK0 &= ~_BV(OCIE0A);	/* compa interrupt OFF */
		}
		else { /* was start bit */
			gl.ssi.txm = 1;
			if (gl.ssi.txd & 01) PORTC |= SSI_TXO;
		}
	}
	else {
		gl.ssi.txm <<= 1;
		if (gl.ssi.txm == 0 || (gl.ssi.txd & gl.ssi.txm))
			PORTC |=  SSI_TXO;
		else	PORTC &= ~SSI_TXO;
	}
}

ISR(TIMER0_COMPB_vect)
{
	u08_t	i;

	i = PINC & SSI_RXI;
	if (gl.ssi.rxm == 0xff) { /* start bit */
		gl.ssi.rxd = 0;
		gl.ssi.rxm = 1;
	}
	else {
		if (i) gl.ssi.rxd |= gl.ssi.rxm;
		if (gl.ssi.rxm) gl.ssi.rxm <<= 1; /* more bits to sample */
		else {				/* just sampled stop bit */
			TIMSK0 &= ~_BV(OCIE0B);	/* compb interrupt OFF */
			if (i) {		/* stop bit is valid */
				gl.ssi.rxb[i = gl.ssi.rxw] = gl.ssi.rxd;
				if (++i >= NEL(gl.ssi.rxb)) i = 0;
				if (i != gl.ssi.rxr) gl.ssi.rxw = i;
				gl.ssi.rxi |= SSI_RXI;
			}
			PCMSK1 |= SSI_RXI;	/* pin change interrupt ON */
		}
	}
}

ISR(PCINT1_vect)
{
	u08_t	rxi, tmp;

	rxi = PINC & SSI_RXI;		/* sample input */
	tmp = rxi ^ gl.ssi.rxi;		/* compare to last sample */
	if (tmp & ~rxi) {		/* if it changed and is 0 now -> received start bit */
		PCMSK1 &= ~SSI_RXI;	/* pin change interrupt OFF */
		TIFR0 |= _BV(OCF0B);
		tmp = TCNT0;
		if ((tmp += TMR0HBT) >= TMR0CNT) tmp -= TMR0CNT;
		OCR0B = tmp;
		gl.ssi.rxm = 0xff;	/* tell timer irq, next sample is start bit */
		TIMSK0 |= _BV(OCIE0B);	/* compB interrupt ON */
	}
	gl.ssi.rxi = rxi;		/* save input */
}

void ssi_sc(char c) /* send character */
{
	u08_t	i;

	gl.ssi.txb[i = gl.ssi.txw] = c;
	if (++i >= NEL(gl.ssi.txb)) i = 0;
	if (i == gl.ssi.txr) return;
	gl.ssi.txw = i;
	if (TIMSK0 & _BV(OCIE0A)) return;
	TIFR0  |= _BV(OCF0A);		/* clear compa flag */
	TIMSK0 |= _BV(OCIE0A);		/* compa interrupt ON */
}

void ssi_snx(u08_t n) { if ((n &= 0x0f) <= 9) ssi_sc('0'+n); else ssi_sc('A'-10+n); }
void ssi_sbx(u08_t b) { ssi_snx(b>>4); ssi_snx(b>>0); }

int ssi_rc(void) /* read next character received */
{
	u08_t	i;
	int	b;

	if (gl.ssi.rxw == gl.ssi.rxr) return -1;
	b = (int)gl.ssi.rxb[i = gl.ssi.rxr];
	gl.ssi.rxr = ++i >= NEL(gl.ssi.rxb) ? 0 : i;
	return b;
}

char *nmf(char *s, char i)
{
	if (i == 0) return s;
	if (i <  0) return 0;
	while (--i >= 0 && *s)
		for (s++; *s && *s != ','; s++);
	return i >= 0 ? 0 : ++s;
}

char *cpf(char *s, char *d) /* copy field */
{
	while (s && (*d = *s++) && *d != ',') d++;
	*d = 0;
	return d;
}

char *ddmm2deg(char *s, char *dst)
{
	union { short i; struct { char lo, hi; }; } u; /* give compiler some hints... */
	char	n, k, *e, *r, *d = dst;
	int	i;

	if (s && *s != ',') {
		for (e = s; *e && *e != ','; e++);
		if (*e == ',' && (e[1] == 'S' || e[1] == 'W')) *d++ = '-';
		for (r = d; s < e && (*d = *s++) >= '0' && *d <= '9'; d++);
		u.i = i = 0;
		if (d > r) u.lo = *(--d) - '0', i += u.i;	/* pick up full minutes */
		if (d > r) u.lo = *(--d) - '0', i += u.i * 10;
		for (*d++ = '.', n = 7; --n >= 0; *d++ = k) {	/* pick up frac minutes */
			i *= 10;
			if (s < e && (u.lo = *s - '0') >= 0 && u.lo <= 9) i += u.i, s++;
			for (k = '0'; i >= 60; k++, i -= 60);
		}
	}
	*d = 0;
	return d;
}

void gps_upd(void)
{
	char	*d = gl.gps.dat;
	u16_t	id;
	u08_t	mt;

	if (d[0] != 'G' || d[1] != 'P') return;
	mt = 0;
	id = (d[3]<<8)|(u08_t)d[4];
	switch (d[2]) {
	case 'G':    if (id == (('G'<<8)|'A')) mt = GP_GGA;
		else if (id == (('S'<<8)|'V')) mt = GP_GSV;
		else if (id == (('S'<<8)|'A')) mt = GP_GSA;
		else if (id == (('L'<<8)|'L')) mt = GP_GLL;
		break;
	case 'R': if (id == (('M'<<8)|'C')) mt = GP_RMC; break;
	case 'V': if (id == (('T'<<8)|'G')) mt = GP_VTG; break;
	}
	switch ((gl.gps.typ = mt)) {
	case GP_GGA:
		d = cpf(nmf(gl.gps.dat,6),gl.gps.loc);		/* position fix indicator */
		*d++ = ','; d = cpf(nmf(gl.gps.dat,7),d);	/* # of satellies used */
		*d++ = ','; d = cpf(nmf(gl.gps.dat,1),d);	/* UTC time */
		*d++ = ','; d = ddmm2deg(nmf(gl.gps.dat,2),d);	/* latitude  (degrees) */
		*d++ = ','; d = ddmm2deg(nmf(gl.gps.dat,4),d);	/* longitude (degrees) */
		*d++ = ','; d = cpf(nmf(gl.gps.dat,8),d);	/* HDOP, H dilution of precision */
		*d++ = ','; d = cpf(nmf(gl.gps.dat,9),d);	/* altitude */
		gl.gps.upd = 1;
		break;
	}
}

void gps_rcv(void)
{
	int	r;
	u08_t	c, n;

	while ((r = ssi_rc()) >= 0) {
		c = r;
		/* NMEA: $..........*hh\r\n */
		n = gl.gps.cst;
		switch (gl.gps.cst) {
		case 6: /* message stays valid until next '$' arrives */
		case 0: if (c != '$') break;
			gl.gps.cks = gl.gps.nda = 0;
			gl.gps.cst = 1;
			gl.gps.typ = 0;
			break;
		case 1: if (c == '*') {
				gl.gps.dat[gl.gps.nda] = 0;
				gl.gps.cst++;
				break;
			}
			if (c < ' ' || c > '~' || gl.gps.nda >= NEL(gl.gps.dat)) {
				gl.gps.cst = 0;
				break;
			}
			gl.gps.dat[gl.gps.nda++] = c;
			gl.gps.cks ^= c;
			break;
		case 2:	gl.gps.cst = 0;
			     if (c >= '0' && c <= '9') c -= '0';
			else if (c >= 'A' && c <= 'F') c -= 'A'-10;
			else break;
			gl.gps.cks ^= c<<4;
			gl.gps.cst = 3;
			break;
		case 3: gl.gps.cst = 0;
			     if (c >= '0' && c <= '9') c -= '0';
			else if (c >= 'A' && c <= 'F') c -= 'A'-10;
			else break;
			gl.gps.cst = gl.gps.cks == c ? 4 : 0;
			break;
		case 4: gl.gps.cst = c == '\r' ? 5 : 0; break;
		case 5: gl.gps.cst = c == '\n' ? 6 : 0;
			if (gl.gps.cst) gps_upd();
			break;
		}
	}
}

/*
 * RC receiver
 */
#define	MS2TIC(ms)	(u32_t)((double)F_TM1*(double)(ms)*1e-3)
#define	TIC2MS(tic)	(u32_t)((tic)/(F_TM1/1000L))
#define	TIC2MS10(tic)	(u32_t)((tic)/(F_TM1/10000L))
#define	TIC2US(tic)	(u32_t)((tic)/(F_TM1/1000000L))

#define	PTMAX	MS2TIC(2.05)	/* pulse should be shorter than that */
#define	PTMIN	MS2TIC(0.95)	/* pulse should be longer than that */
#define	PTREF	MS2TIC(1.50)	/* nominal center position */
#define	PTRNG	MS2TIC(0.40)	/* +- nominal spread */

#define	FTMIN	MS2TIC( 8.0)	/* min frametime */
#define	FTMAX	MS2TIC(35.0)	/* max frametime */

ISR(PCINT2_vect) /* port D signal change interrupt */
{
	union { rsm_t *r; u08_t *b; } p;
	u08_t	i;

	p.r = gl.rxs.rsb;
	i = gl.rxs.rsw;
	p.b += i * sizeof(*p.r);
	p.r->smp = PIND;
	itic((u16_t *)&p.r->tmr);
	if (++i >= NEL(gl.rxs.rsb)) i = 0;
	if (i != gl.rxs.rsr) gl.rxs.rsw = i;
}

void rx_proc(void)
{
	u32_t	tmr, dtm;
	char	k;
	u08_t	i, smp, chg, lsm;

	lsm = gl.rxs.smp;
	for (k = NEL(gl.rxs.rsb); --k >= 0 && (i = gl.rxs.rsr) != gl.rxs.rsw; ) {
		smp = gl.rxs.rsb[i].smp;
		tmr = gl.rxs.rsb[i].tmr;
		gl.rxs.rsr = ++i >= NEL(gl.rxs.rsb) ? 0 : i;
		chg = (smp ^ lsm) & PCMSK2;
		lsm = smp;
		for (i = 0; chg && i < NEL(gl.rxs.chn); i++)
			if (chg & gl.rxs.chn[i].msk) {
				dtm = tmr - gl.rxs.chn[i].tup;
				if (smp & gl.rxs.chn[i].msk)
					gl.rxs.chn[i].ftm = dtm, gl.rxs.chn[i].tup = tmr;
				else	gl.rxs.chn[i].dur = dtm;
				chg &= ~gl.rxs.chn[i].msk;
			}
	}
	gl.rxs.smp = lsm;
}

int rx_p2v(int dur, int dbn, u08_t inv)
{
	float	f;

	dur -= PTREF;
	if (dur >= 0) {
		if ((dur -= dbn) <= 0) return 0;
	}
	else {
		if ((dur += dbn) >= 0) return 0;
	}
	f = ((float)dur * 1000.0) / (float)(PTRNG - dbn);
	if (inv) f = -f;
	return (int)f;
}

u08_t rx_read(void)
{
	u08_t	eft, ept, mch, srg, inv;
	u32_t	ft;
	int	dt;
	struct chn *c;

	ept = TXDSC() ? (_BV(NEL(gl.rxs.chn))-1) : 0;
	eft = 0;
	mch = _BV(NEL(gl.rxs.chn)-1);
	inv = _BV(S_AUX1);
	rx_proc();
	for (c = gl.rxs.chn + NEL(gl.rxs.chn); --c >= gl.rxs.chn; mch >>= 1) {
		srg = SREG; cli(); ft = c->ftm; dt = c->dur; SREG = srg;
		c->val = 0;
		     if (ft < FTMIN || ft > FTMAX) eft |= mch;
		else if (dt < PTMIN || dt > PTMAX) ept |= mch;
		else c->val = rx_p2v(dt,c->dbn,inv&mch);
	}
	gl.rxs.ept = ept;
	gl.rxs.eft = eft;

	dt = gl.rxs.chn[S_LAND].val;
	switch (S_LAND) {
	case S_GEAR: gl.rxs.fms =      dt < 0 ? FMS_LAND : FMS_FM_1; break;
	case S_AUX1: gl.rxs.fms = dt ? dt < 0 ? FMS_LAND : FMS_FM_2 : FMS_FM_1; break;
	}
	if (S_FMOD != S_LAND && gl.rxs.fms != FMS_LAND) {
		dt = gl.rxs.chn[S_FMOD].val;
		switch (S_FMOD) {
		case S_GEAR: gl.rxs.fms = dt < 0 ? FMS_FM_1 : FMS_FM_2; break;
		case S_AUX1: gl.rxs.fms = dt < 0 ? FMS_FM_1 : FMS_FM_2; break;
		}
	}

	switch (gl.rxs.fsq) {
	default: gl.rxs.fsq = 0; break;
	case 0: if (gl.rxs.fms == FMS_FM_2) gl.rxs.fsq = 1; break;
	case 1: if (gl.rxs.fms == FMS_FM_1) {
			gl.rxs.fst = 15; /* tics * 1/LOOPHZ = 0.6 sec */
			gl.rxs.fsq = 2;
			gl.rxs.fms = FMS_FM_2;
		}
		else if (gl.rxs.fms == FMS_LAND) gl.rxs.fsq = 0;
		break;
	case 2: if ((gl.rxs.fst -= 1) == 0 || gl.rxs.fms == FMS_LAND) gl.rxs.fsq = 0;
		else if (gl.rxs.fms == FMS_FM_2) {
			gl.rxs.fsq = 3;
			gl.rxs.fms = FMS_FM_3;
			if (gl.vlba.mcsy == 0) gl.vlba.mcsy = MC_R;
		}
		else gl.rxs.fms = FMS_FM_2;
		break;
	case 3: if (ept | eft) gl.rxs.fsq = 0;
		else if (gl.rxs.fms != FMS_FM_2) gl.rxs.fsq = 0;
#if	T_MODE == 2 || T_MODE == 3
		else if (gl.rxs.chn[S_AILE].val != 0) gl.rxs.fsq = 0;
#endif
		else if (gl.rxs.chn[S_ELEV].val != 0) gl.rxs.fsq = 0;
		else gl.rxs.fms = FMS_FM_3;
		break;
	}
	return (ept | eft);
}

/*
 * uart receiver
 */
ISR(USART_RX_vect) /* receiver interrupt */
{
	u08_t	i;

	gl.sio.rxb[i = gl.sio.rxw] = UDR0;
	if (++i >= NEL(gl.sio.rxb)) i = 0;
	if (i != gl.sio.rxr) gl.sio.rxw = i;
}

int sio_rc(void) /* read next character received */
{
	u08_t	i;
	int	b;

	if (gl.sio.rxw == gl.sio.rxr) return -1;
	b = (int)gl.sio.rxb[i = gl.sio.rxr];
	gl.sio.rxr = ++i >= NEL(gl.sio.rxb) ? 0 : i;
	return b;
}

char sio_rf(void) /* flush sio receive buffer */
{
	if (gl.sio.rxr == gl.sio.rxw) return 0;
	gl.sio.rxr = gl.sio.rxw;
	return 1;
}

/*
 * uart transmitter
 */
ISR(USART_UDRE_vect) /* tx empty interrupt */
{
	u08_t	i;

	if (gl.sio.txr != gl.sio.txw) {
		UDR0 = gl.sio.txb[i = gl.sio.txr];
		gl.sio.txr = ++i >= NEL(gl.sio.txb) ? 0 : i;
	}
	if (gl.sio.txr == gl.sio.txw)
		UCSR0B &= ~_BV(UDRIE0); /* no more, disable TX IRQ */
}

void sio_sc(char c) /* send character */
{
	u08_t	i;

	gl.sio.txb[i = gl.sio.txw] = c;
	if (++i >= NEL(gl.sio.txb)) i = 0;
	if (i != gl.sio.txr) {
		gl.sio.txw = i;
		UCSR0B |= _BV(UDRIE0); /* enable TX IRQ */
	}
}

int sio_sr(void) /* txb free space */
{
	int	i;

	i = (gl.sio.txr - gl.sio.txw);
	i += NEL(gl.sio.txb) - 1;
	if (i > NEL(gl.sio.txb)) i -= NEL(gl.sio.txb);
	return i;
}

void sio_tw(void) /* wait for tx empty */
{
	while (gl.sio.txr != gl.sio.txw);
}

void sio_ss(char *s) /* send string from ram */
{
	char	c;

	while ((c = *s++)) sio_sc(c);
}

#define	SIO_SP(str)	sio_sp(PSTR(str))

void sio_sp(const char *p) /* send string from flash */
{
	char	c;

	while ((c = pgm_read_byte(p++))) sio_sc(c);
}

/*
 * convert to ASCII and transmit
 */
/* send hex */
void sio_snx(u08_t n) { n &= 0x0f; if (n <= 9) sio_sc('0'+n); else sio_sc('A'-10+n); }
void sio_sbx(u08_t b) { sio_snx(b>>4); sio_snx(b>>0); }
void sio_swx(u16_t w) { sio_sbx(w>>8); sio_sbx(w>>0); }

void sio_sid(char lc, int i) /* send integer decimal */
{
	u08_t	fw, nd, sn, b[16];

	fw = sn = nd = 0;
	if (lc > 0) sio_sc(lc);
	else if (lc < 0) fw = -lc;
	if (i < 0) i = -i, sn = '-';
	do b[nd++] = (i%10) + '0'; while ((i /= 10));
	if (sn) b[nd++] = sn;
	for (sn = nd; sn < fw; sn++) sio_sc(' ');
	do sio_sc(b[--nd]); while (nd);
}

void sio_sld(long i) /* send long decimal */
{
	u08_t	nd, sn, b[16];

	sn = nd = 0;
	if (i < 0) i = -i, sn = '-';
	do b[nd++] = (i%10) + '0'; while ((i /= 10));
	if (sn) sio_sc(sn);
	do sio_sc(b[--nd]); while (nd);
}

/*
 * messages from drone
 */
#define	C_ACK	0x01
void sio_rcv(void)
{
	int	r;
	char	c;

	while ((r = sio_rc()) >= 0)
		if ((c = r) <= 0) gl.drm.cst = 0;
		else if (c == C_ACK) gl.drm.ack = 1, gl.drm.cst = 0;
		else switch (gl.drm.cst) {
		case 0: if (c == '$') gl.drm.cst = 1, gl.drm.nda = 0; break;
		case 1: if (c == '*') {
				if (gl.drm.nda == sizeof(gl.drm.dat.m2a))
					gl.vlba.cbat = gl.drm.dat.m2a.cbat;
				gl.drm.cst = 0;
				break;
			}
			/* FALLTHROUGH */
		case 2: if (c >= '0' && c <= '9') c -= '0';
			else if (c >= 'A' && c <= 'F') c -= 'A'-10;
			else if (c >= 'a' && c <= 'f') c -= 'a'-10;
			else c = -1;
			if (c < 0 || gl.drm.nda >= sizeof(gl.drm.dat)) {
				gl.drm.cst = 0;
				break;
			}
			if (gl.drm.cst == 1)
				gl.drm.dat.b[gl.drm.nda] = (c<<4);
			else	gl.drm.dat.b[gl.drm.nda++] |= c;
			gl.drm.cst ^= 3;
			break;
		}
}

/*
 * start EEPROM write cycle
 */
void save_var(void)
{
	u08_t	i;

	gl.eemag = EE_MAG;
	gl.eecks = 0;
	for (i = 0; i < (NEL(gl.eedat)-1); i++) gl.eecks += gl.eedat[i];
	gl.eewen = 0;
}

/*
 * drone program piggy back in flash
 */
u16_t rdflash(u16_t adr); /* read flash word */
void _rdflash(void)	/* addr (r24:r25), return (r24:r25) */
{
asm("rdflash:     ");
asm(" push r31    ");
asm(" push r30    ");
asm(" movw r30,r24");
asm(" lpm  r24,Z+ ");
asm(" lpm  r25,Z  ");
asm(" pop  r30    ");
asm(" pop  r31    ");
}

void at2so_check(u16_t adr)
{
	u16_t	mag, siz, cks;

	if (adr == 0) return;
	mag = rdflash(adr); adr += 2;
	siz = rdflash(adr); adr += 2;
	cks = rdflash(adr); adr += 2;
	if (mag != 0x55aa) return;
	if (siz < ( 1*1024)) return;
	if (siz > (20*1024)) return;
	gl.arm.adr = adr;
	gl.arm.siz = siz;
	gl.arm.cks = cks;
}

#define	C_STRT	0x02

void at2so_exec(void)
{
	union { u16_t w; u08_t b[2]; } v;
	u16_t	cks, adr, cnt;

	if (gl.arm.adr != 0) { /* piggy back present */
		LEDOF();
		/* put tty we are talking to into raw mode, will be
		 * sending binary data -> NO interpretation please! */
		SIO_SP("stty -echo -iexten pass8 raw\n");
		ms_dly(50); sio_rf();
		/* 'dd' is the program of choice to receive the data, need to
		 * tell it how much is coming, so it quits after the transfer.
		 * 'dd' on drone is the 'amputated' version without bells and
		 * whistles. */
		SIO_SP("dd of=/tmp/at2so.arm.gz bs=1 count=");
		sio_sid(0,gl.arm.siz);
		sio_sc('\n');
		ms_dly(50); sio_rf();
		for (adr = gl.arm.adr, cnt = cks = 0;;) {
			v.w = rdflash(adr+cnt); /* reads 2 bytes from flash */
			sio_sc(v.b[0]); cks += v.b[0];
			if (++cnt >= gl.arm.siz) break;
			sio_sc(v.b[1]); cks += v.b[1];
			if (++cnt >= gl.arm.siz) break;
			if ((cnt & 0x1f) == 0) {
				LEDTG(); /* makes LED look like it is dimmed */
				sio_tw();
				ms_dly(1);
			}
		}
		ms_dly(50); sio_rf();
		/* put tty back to 'normal' */
		SIO_SP("stty sane\n"); ms_dly(50); sio_rf();
		while (cnt != gl.arm.siz || cks != gl.arm.cks) { /* upload bad */
			blip(1); ms_dly(1000);
			blip(3); ms_dly(1000);
		}
		LEDON();
		/* unzip it */
		SIO_SP("gunzip /tmp/at2so.arm.gz\n"); ms_dly(50); sio_rf();
		/* make it executable */
		SIO_SP("chmod +x /tmp/at2so.arm\n"); ms_dly(50); sio_rf();
		SIO_SP("/tmp");		/* location where piggy back was put */
	}
	else SIO_SP("/data/video");	/* most likely location of preinstalled */
	SIO_SP("/at2so.arm");		/* companion program */
	SIO_SP(" "AT2SOARGS);		/* companion program arguments */
	ms_dly(25); sio_rf();		/* wait out echo */
	sio_sc('\n');			/* and launch */
	while (sio_rc() != C_STRT);	/* wait for start character */
}

void gps_cmd(char *b)
{
	u08_t	cks;

	ssi_sc('$'); sio_sc('$');
	for (cks = 0; *b; b++) {
		cks ^= *b;
		ssi_sc(*b); sio_sc(*b);
	}
	ssi_sc('*'); sio_sc('*');
	ssi_sbx(cks); sio_sbx(cks);
	ssi_sc('\r');
	ssi_sc('\n');
}

#define	HZ2TIC(hz)	((double)F_TM1/(double)(hz))
#define	DRUPD		(u32_t)HZ2TIC(LOOPHZ)	/* loop cycle time in timer 1 ticks */

/*
 * RC receiver interface debug/setup loop
 */
void csi(char *a)   { SIO_SP("\033["); sio_ss(a); }
void csirow(int  r) { SIO_SP("\033["); sio_sid(0,r); SIO_SP(";1H"); }
void csilco(char c) { SIO_SP("\033[0;"); sio_sbx(0x30+c); sio_sc('m'); }
void csihco(char c) { SIO_SP("\033[1;"); sio_sbx(0x30+c); sio_sc('m'); }

#define	RCLR	"0J"
#define	LCLR	"K"

#define	DLN0	1
#define	DLN1	2
#define	DLN2	3
#define	DLN3	4
#define	DLNRXI	6
#define	DLNRXC	7
#define	DLNLOC	14
#define	DLNGPO	15  /* GPS command line edit */
#define	DLNGPS	16  /* last command sent */
#define	DLNGPI	17

#define	BLACK	0
#define	RED	1
#define	GREEN	2
#define	YELLOW	3
#define	BLUE	4
#define	MAGENTA	5
#define	CYAN	6
#define	WHITE	7

u08_t rdbcfg(u08_t spmcsr, u16_t adr);
void _rdbcfg(void) /* spmcsr (r24), addr (r23:r22), return (r24) */
{
asm("rdbcfg:");
asm(" push r31");
asm(" push r30");
asm(" movw r30,r22");
asm(" out  0x37,r24");
asm(" lpm  r24,Z   ");
asm(" pop  r30");
asm(" pop  r31");
}

void rdcfg(u08_t *buf)
{
	u08_t	srg;

	srg = SREG; cli();
	buf[0] = rdbcfg(0x09,3); /* high fuse */
	buf[1] = rdbcfg(0x09,0); /* low fuse */
	buf[2] = rdbcfg(0x09,2); /* ext fuse */
	buf[3] = rdbcfg(0x09,1); /* lockbits */
	buf[4] = rdbcfg(0x21,0); /* signature byte */
	buf[5] = rdbcfg(0x21,2); /* signature byte */
	buf[6] = rdbcfg(0x21,4); /* signature byte */
	SREG = srg;
}

#define	TYTHR	500	/* throttle/yaw stick threshold for FTRIM/EMERGENCY */

void rx_loop(void)
{
	u08_t	i, k, msk;
	int	rxs, yaw;
	long	tmp;
	u32_t	nxt;
	u08_t	cfg[7];
	char	buf[NEL(gl.ssi.txb)-4];

AGAIN:
	sio_tw();
	SIO_SP("\033[2J\033[H\033[0m");	/* clear screen, home, clear attributes */
	csi(RCLR);
	csirow(DLN0); csihco(CYAN);
	sio_sp(version+5);
	SIO_SP(", at2so ");
	if (gl.arm.adr == 0) SIO_SP("not ");
	SIO_SP("attached");

	sio_tw();
	csirow(DLN1); csilco(WHITE);
	SIO_SP(" loop ");
	tmp = TIC2MS10(DRUPD);
	sio_sld(tmp/10); sio_sid('.',tmp%10);
	SIO_SP(" ms, sio 115200 bps, gps ");
	sio_sld(BAUD_GPS); SIO_SP(" bps");

	csirow(DLN2); csilco(WHITE);
	SIO_SP("stick +-"); sio_sld(PTRNG); SIO_SP(" points");
	SIO_SP(", dcnt="); sio_sid(0,gl.dcnt);

	sio_tw();
	rdcfg(cfg);
	csirow(DLN3);
	csilco(WHITE); SIO_SP("cpusg ");
	i = cfg[4]; msk = 0xff; k = 0x1e; csihco((i&msk)==k?GREEN:RED); sio_sbx(i);
	i = cfg[5]; msk = 0xff; k = 0x95; csihco((i&msk)==k?GREEN:RED); sio_sbx(i);
	i = cfg[6]; msk = 0xff; k = 0x0f; csihco((i&msk)==k?GREEN:RED); sio_sbx(i);
	csilco(WHITE); SIO_SP(" fuses ");
	sio_sbx(cfg[0]); sio_sc('-');	/* high */
	sio_sbx(cfg[1]); sio_sc('-');	/* low */
	sio_sbx(cfg[2]); sio_sc('-');	/* extended */
	sio_sbx(cfg[3]);		/* lockbits */

	csirow(DLNRXI); csihco(CYAN);
	SIO_SP("-RX-  f[ms] p[us] value \r\n");
	nxt = tic() + DRUPD;
	k = 0;
	while (1) {
		sio_tw();
		LEDON();
		while ((tmp = nxt - tic()) > 0) {
			gps_rcv();
			if (tmp < MS2TIC(4.0)) continue;
			if (gl.gps.cst != 6) continue;
			if (sio_sr() < 64) continue;
			if (gl.gps.typ) continue; /* message recognized */
			gl.gps.cst = 0;
			csirow(DLNGPI); csi(LCLR); csihco(RED);
			sio_ss(gl.gps.dat);
		}
		LEDOF();
		nxt = tic() + DRUPD;

		rxs = rx_read();
		for (i = 0; i < NEL(gl.rxs.chn); i++) {
			if (sio_sr() < 64) sio_tw();
			csirow(DLNRXC+i);
			msk = _BV(i);
			csihco(rxs&msk?RED:GREEN);
			switch (i) {
			case S_THRO: SIO_SP("THRO"); break;
			case S_AILE: SIO_SP("AILE"); break;
			case S_ELEV: SIO_SP("ELEV"); break;
			case S_RUDD: SIO_SP("RUDD"); break;
			case S_GEAR: SIO_SP("GEAR"); break;
			case S_AUX1: SIO_SP("AUX1"); break;
			}
			csihco(gl.rxs.eft&msk?RED:GREEN);
			tmp = TIC2MS10(gl.rxs.chn[i].ftm);
			sio_sid(-5,tmp/10); sio_sid('.',tmp%10);
			if (gl.rxs.ept&msk) csihco(RED);
			sio_sid(-6,TIC2US(gl.rxs.chn[i].dur));
			sio_sid(-6,gl.rxs.chn[i].val);
			if (i == S_LAND) {
				if (rxs) SIO_SP("     ");
				else 
					switch (gl.rxs.fms) {
					case FMS_LAND: SIO_SP(" LAND"); break;
					case FMS_FM_1: SIO_SP(" FM_1"); break;
					case FMS_FM_2: SIO_SP(" FM_2"); break;
					case FMS_FM_3: SIO_SP(" FM_3"); break;
					default:       SIO_SP(" ????"); break;
					}
			}
			else if (i == SH_THRO) {
				if (rxs || gl.rxs.fms != FMS_LAND) SIO_SP("     ");
				else {
					yaw = gl.rxs.chn[SH_THRO].val;
					     if (yaw <= -TYTHR) SIO_SP(" TRIM");
					else if (yaw >=  TYTHR) SIO_SP(" ESTP");
					else SIO_SP("     ");
				}
			}
		}
		if (gl.gps.upd) {
			sio_tw();
			gl.gps.upd = 0;
			csirow(DLNLOC); csi(LCLR); csihco(GREEN);
			SIO_SP("GL="); sio_ss(gl.gps.loc);
		}
		if ((rxs = sio_rc()) == C_STRT) goto AGAIN;
		if (rxs > 0) {
			sio_tw();
			i = rxs;
			if (i >= ' ' && i <= '~' && k < (NEL(buf)-1)) {
				if (i >= 'a' && i <= 'z') i += 'A'-'a';
				buf[k++] = i;
			}
			else if (i == '\b' && k > 0) k--;
			else if (i == '\r') {
				csirow(DLNGPS); csi(LCLR); csilco(WHITE);
				buf[k] = 0, gps_cmd(buf), k = 0;
			}
			csirow(DLNGPO); csi(LCLR); csihco(WHITE);
			if (sio_sr() < 64) sio_tw();
			if (k) SIO_SP("gps: $");
			for (i = 0; i < k; i++) sio_sc(buf[i]);
			if (k) sio_sc('_');
		}
		SIO_SP("\r\n");
	}
}

/*
 * DRONE configuration and control loop
 */
u08_t snd_cfg(char i)
{
	const char	*p;
	char		c;
	u08_t		k = 1;

	SIO_SP("RX=C");
	if (i >= 0 && i <= 5) SIO_SP(",control:");
	switch (i) {
	case  0: SIO_SP("outdoor"); break;
	case  1: SIO_SP("flight_without_shell"); break;
	case  2: SIO_SP("euler_angle_max"); break;
	case  3: SIO_SP("control_vz_max"); break;
	case  4: SIO_SP("control_yaw"); break;
	case  5: SIO_SP("altitude_max"); break;
	default: k = 0; break;
	}
	if (k) {
		sio_sc(',');
		switch (gl.cfg) {
		default:
		case 0: p = cfg1; break;
		case 1: p = cfg2; break;
		case 2: p = cfg3; break;
		case 3: p = cfg4; break;
		}
		while (i >= 0 && (c = pgm_read_byte(p++)))
			if (c == ' ') continue;
			else if (c == ',') i--;
			else if (i == 0) sio_sc(c);
	}
	sio_sc('\r');
	return k;
}

void dr_loop(void)
{
	u08_t	s, i, k, t, led;
	int	val;
	long	tmp;
	u32_t	nxt;

	val = 0;
	led = 0;
	s = i = k = 0;
	/* This is it, this program is talking to the program on the drone now.
	 * The drone program makes no judgment calls nor looks at time, just
	 * translates what comes from here....
	 * 0,1,2) send configuration commands
	 * 3+)    your actions on TX/RX are in charge */
	sio_rf();
	nxt = tic() + DRUPD;
	while (1) {
		SIO_SP("RX=");
		if (gl.pad < 0) sio_sc('E');
		else if (gl.pad == 0 || s < 3) sio_sc('0');
		else
			switch (gl.rxs.fms) {
			default: sio_sc('0'); break;
			case FMS_FM_1: case FMS_FM_2:
				sio_sid(0,gl.rxs.fms);
				sio_sid(',',gl.rxs.chn[S_ROL].val);
				sio_sid(',',gl.rxs.chn[S_PTC].val);
				sio_sid(',',gl.rxs.chn[S_GAZ].val);
				sio_sid(',',gl.rxs.chn[S_YAW].val);
				break;
			case FMS_FM_3:
				SIO_SP("3,0,0");
				sio_sid(',',gl.rxs.chn[S_GAZ].val);
				sio_sid(',',gl.rxs.chn[S_YAW].val);
				break;
			}
		sio_sc('\r');
		if (gl.gps.upd > 0) {
			gl.gps.upd = 0;
			SIO_SP("GL="); sio_ss(gl.gps.loc); sio_sc('\r');
		}
		sio_sc('\n');

		/* EEPROM update */
		if (gl.eewen < NEL(gl.eedat) && (EECR & _BV(EEPE)) == 0) {
			EEAR = (int)gl.eewen;
			EEDR = gl.eedat[gl.eewen];
			t = SREG; cli();
			EECR = _BV(EEMPE);
			EECR |= _BV(EEPE);
			SREG = t;
			gl.eewen++;
		}
		/* keep loop on track
		 * drone has a deadline, don't want to be too pushy or late */
		gl.drm.ack = 0;
		while ((tmp = nxt - tic()) > 0) {
			gps_rcv();
			sio_rcv();
		}
		nxt = tic() + DRUPD;

		if (s < 3) rx_read();
		else { /* configuration is done */
			/* LED blink */
			if (led == 0) LEDTG();
			else if ((led >>= 1) == 0) {
				led = 1<<5;
				if (i <= k) LEDON();
				if (++i >= 6) i = 0;
			}
			else LEDOF();
			/* VLBA */
			if (VLBA_THR > 0 && VLBA_THR <= 60) {
				if (++gl.vlba.tick >= 5) {
					gl.vlba.tick = 0;
					switch (gl.vlba.stat) {
					case 0: if (gl.vlba.cbat <= VLBA_THR) {
							VLBON();
							SIO_SP("RX=L,2,5.0,0\r");
							gl.vlba.stat = 1;
						}
						break;
					case 1: if (gl.vlba.cbat > VLBA_THR) {
							VLBOF();
							SIO_SP("RX=L,2,5.0,1\r");
							gl.vlba.stat = 0;
						}
						else VLBON();
						break;
					}
				}
				else if (VLBA_BLINK && gl.vlba.stat && gl.vlba.tick == 2) VLBOF();
			}
			/* VLBA morse code flashing */
			if (++gl.vlba.mctk >= 6) {
				gl.vlba.mctk = 0;
				if (gl.vlba.stat == 0) {
					switch (gl.vlba.mcst) {
					case 0: if (gl.vlba.mcsy == 0) break;
						/* FALLTHROUGH */
					case 1: gl.vlba.mcst = (gl.vlba.mcsy&07) == 0 ? 0 :
						gl.vlba.mcsy&_BV(2+(gl.vlba.mcsy&07)) ? 0x80 : 0x82;
						break;
					case 2: gl.vlba.mcst++; break;
					case 3: gl.vlba.mcst++; break;
					case 4: gl.vlba.mcst = gl.vlba.mcsy = 0; break;
					case 0x80: gl.vlba.mcst++; break;
					case 0x81: gl.vlba.mcst++; break;
					case 0x82: gl.vlba.mcst = (--gl.vlba.mcsy & 07) ? 1 : 2; break;
					}
					if (gl.vlba.mcst & 0x80) VLBON();
					else if (gl.vlba.mcst)   VLBOF();
				}
			}
			if (rx_read() & S_SIG) { /* radio quit or got disconnected from TX */
				gl.pad = 0;
				s = 9; led = 0;
				continue;
			}
		}

		switch (s) {
		/*
		 * drone configuration
		 */
		case 0: /* send configuration/setup commands */
			if ((k = snd_cfg(i++))) s = 1;
			else s = 2, i--, k = 50, LEDOF();
			break;
		case 1: if (--k == 0) s = 0; break;
		case 2: if (gl.drm.ack) s = 3, i = 0; /* received C_ACK from drone */
			else if (--k == 0) s = 0, LEDON();
			break;
		/*
		 * drone on ground, or getting there
		 */
		case 3:
			if (gl.rxs.fms == FMS_LAND) {
				/* check throttle horizontal stick */
				val = gl.rxs.chn[SH_THRO].val;
				if (ABS(val) > TYTHR) {
					if (val > 0) gl.pad = -1;	/* right -> EMERGENCY */
					else {				/* left  -> FTRIM */
						SIO_SP("RX=F\r");
						SIO_SP("RX=L,1,5.0,1\r");
						if (gl.vlba.mcsy == 0) gl.vlba.mcsy = MC_R;
					}
					s = 4;
					break;
				}
				/* check throttle stick */
				val = gl.rxs.chn[S_THRO].val;
				if (val > TYTHR) {
					led = 1;
					k = gl.cfg;
					i = 0;
					s = 5;
					break;
				}
				break;
			}
			if (gl.eewen < NEL(gl.eedat)) break; /* wait for EEPROM write to complete */
			if (gl.rxs.chn[S_THRO].val) break; /* throttle center for launch */
			gl.pad = 1, s = 10; /* launch */
			break;
		case 4: /* waiting for throttle horizontal release */
			if (gl.rxs.fms != FMS_LAND) break;
			val = gl.rxs.chn[SH_THRO].val;
			if (ABS(val) < (TYTHR/2)) gl.pad = 0, s = 3;
			break;
		case 5: /* display/set configuration */
			val = gl.rxs.chn[S_THRO].val;
			if (ABS(val) < (TYTHR/2)) {
				led = 0;
				if (k == gl.cfg) s = 3;
				else {
					gl.cfg = k;
					save_var();
					i = 0;
					s = 0;
				}
				break;
			}
			k = gl.cfg;
			val = gl.rxs.chn[S_ELEV].val;
			if (ABS(val) > TYTHR) k = val > 0 ? 0 : 2;
			val = gl.rxs.chn[SH_ELEV].val;
			if (ABS(val) > TYTHR) k = val > 0 ? 3 : 1;
			break;
		case 9: /* radio loss recovery */
			if (gl.rxs.fms != FMS_LAND) break; /* want FSW on LAND */
			s = 3;
			break;
		/*
		 * drone airborne
		 */
		case 10: if (gl.rxs.fms == FMS_LAND) gl.pad = 0, s = 3; break;
		}
	}
}

int main(void)
{
	u08_t	i, j, k;

	bzero(&gl,sizeof(gl));
	for (EECR = 0; gl.eewen < NEL(gl.eedat); gl.eewen++) {
		EEAR = (int)gl.eewen;
		EECR |= _BV(EERE);
		gl.eedat[gl.eewen] = EEDR;
	}
	for (i = k = 0; i < (NEL(gl.eedat)-1); i++) k += gl.eedat[i];
	if (gl.eemag != EE_MAG || gl.eecks != k)
		gl.cfg = 0;

#ifdef	AT2SO	/* (see NOTES AT2SO) */
	extern	u16_t flash_at2so(void);
	at2so_check(flash_at2so());
#endif

	/* RC receiver hardware bits on port D */
	gl.rxs.chn[S_THRO].msk = _BV(7);
	gl.rxs.chn[S_AILE].msk = _BV(6);
	gl.rxs.chn[S_ELEV].msk = _BV(5);
	gl.rxs.chn[S_RUDD].msk = _BV(4);
	gl.rxs.chn[S_GEAR].msk = _BV(3);
	gl.rxs.chn[S_AUX1].msk = _BV(2);

	/* dead bands */
	gl.rxs.chn[S_THRO].dbn = MS2TIC(0.06);
	gl.rxs.chn[S_AILE].dbn = MS2TIC(0.04);
	gl.rxs.chn[S_ELEV].dbn = MS2TIC(0.04);
	gl.rxs.chn[S_RUDD].dbn = MS2TIC(0.04);
	gl.rxs.chn[S_GEAR].dbn = MS2TIC(0.20);
	gl.rxs.chn[S_AUX1].dbn = MS2TIC(0.20);
	/* setup PCMSK */
	PCMSK2 = 0;
	PCMSK2 |= gl.rxs.chn[S_THRO].msk;
	PCMSK2 |= gl.rxs.chn[S_AILE].msk;
	PCMSK2 |= gl.rxs.chn[S_ELEV].msk;
	PCMSK2 |= gl.rxs.chn[S_RUDD].msk;
	PCMSK2 |= gl.rxs.chn[S_GEAR].msk;
	PCMSK2 |= gl.rxs.chn[S_AUX1].msk;

	tmr0_setup();
	tmr1_setup();
	port_setup();
	uart_setup();
	VLBOF();

	if (!SETUP()) {
		/* wait for drone to turn on it's TX */
		LEDON();
		while (!(PIND & _BV(0))); /* USART RX input */
		LEDOF();
	}

	gl.rxs.smp = PIND;
	gl.ssi.rxi = PORTC & SSI_RXI;
	sei();	/* turn interrupts on */
	msdly_cali();
	/* turn USART on */
	UCSR0B |= _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0);
	/* turn RC receiver sampling on */
	PCICR |= _BV(PCIE2);
	/* turn ssi receiver on */
	PCMSK1 |= SSI_RXI;
	PCICR  |= _BV(PCIE1);

	if (SETUP()) {
		/* wait for start character from user */
		while (sio_rc() != C_STRT) blip(5), ms_dly(1000);
		rx_loop();
		/* NOTREACHED */
	}

	/* wait for drone to boot, make sure radio works and has 'sane' state */
#define	NS_BOOT	16
#define	NSQUIET	5
	for (i = j = k = 0; 1; ) {
		if (sio_rf() || j < NS_BOOT) i = 0;
		else if (i < NSQUIET) i++;
		if (rx_read() & S_SIG) k = 0;
		else if (gl.rxs.fms != FMS_LAND) k = 1;
		else if (j < NS_BOOT) k = 2;
		else if (i < NSQUIET) k = 3;
		else break; /* all conditions met, go on */
		switch (k) {
		case 0: blip(1); ms_dly(800); break; /* radio not ready */
		case 1: blip(2); ms_dly(600); break; /* flight mode not LAND */
		case 2: blip(3); ms_dly(400); break; /* drone still booting */
		case 3: blip(4); ms_dly(200); break; /* drone still talking */
		}
		if (j < NS_BOOT) j++;
	}

	/* start shell on drone */
	sio_sc('\r'); ms_dly(100); sio_rf();
	sio_sc('\r'); ms_dly(50); sio_rf(); /* do it twice... */

	if ((UCSR0A & _BV(U2X0))) { /* is using double speed on serial transmitter */
		/* -> change baudrate to 38400 */
		SIO_SP("stty 38400\n");
		ms_dly(30); sio_rf();
		uart_38400();
		ms_dly(20); sio_rf();
		UCSR0B |= _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0);
		sio_sc('\r'); ms_dly( 50); sio_rf(); /* clear residuals if any */
	}

	LEDON();
	at2so_exec();	/* load/launch companion program on drone */
	LEDOF();

	dr_loop();
	/* NOTREACHED */
	return 0;
}


void _flash_at2so(void) {
asm("flash_at2so:");
asm(" call eod_at2so");
asm(" .word 0x55aa"); /* magic number */
asm(" .word  11243"); /* size */
asm(" .word 0x0f15"); /* checksum */
asm(" .byte 31,139,8,8,18,229,182,78,2,3,97,116,50,115,111,46");
asm(" .byte 97,114,109,0,180,91,15,116,148,85,118,127,223,204,4,134");
asm(" .byte 48,200,7,140,238,108,50,234,135,68,119,86,3,124,33,17");
asm(" .byte 145,131,18,11,10,110,80,241,95,107,15,174,33,132,104,210");
asm(" .byte 13,73,74,70,69,107,187,192,208,93,234,194,138,134,158,166");
asm(" .byte 172,118,230,84,246,200,161,108,23,149,90,214,170,39,130,122");
asm(" .byte 208,181,221,136,116,75,41,71,231,15,223,138,160,167,88,209");
asm(" .byte 69,69,167,191,251,189,251,102,94,62,190,4,237,57,13,231");
asm(" .byte 249,254,124,239,222,119,223,189,247,221,123,223,125,227,15,175");
asm(" .byte 93,116,157,97,24,66,253,5,68,66,80,207,92,47,68,3");
asm(" .byte 106,171,149,198,68,69,131,176,68,24,223,170,69,21,125,239");
asm(" .byte 9,55,11,17,126,87,22,219,144,37,36,100,25,37,36,108");
asm(" .byte 195,106,89,92,132,40,21,252,61,72,223,208,111,88,35,203");
asm(" .byte 36,33,75,168,60,85,254,173,150,101,214,82,89,42,120,140");
asm(" .byte 190,203,49,195,45,59,48,208,62,86,174,75,223,65,175,152");
asm(" .byte 191,148,138,225,150,83,232,159,98,186,20,141,11,1,179,112");
asm(" .byte 141,44,150,144,69,125,187,217,73,46,23,62,127,10,126,122");
asm(" .byte 103,199,178,233,157,203,167,118,118,116,221,187,106,90,111,247");
asm(" .byte 180,122,57,110,50,237,11,110,188,157,121,41,97,104,124,2");
asm(" .byte 202,5,40,231,11,185,143,56,202,183,80,206,65,169,68,249");
asm(" .byte 54,74,4,37,202,107,141,67,137,161,84,161,156,135,114,46");
asm(" .byte 74,24,165,90,163,199,240,161,49,200,107,234,127,33,166,133");
asm(" .byte 254,192,38,49,6,101,60,247,77,174,39,113,61,218,7,39");
asm(" .byte 209,62,209,103,252,191,81,14,254,72,182,233,251,79,81,142");
asm(" .byte 104,253,147,40,39,180,126,12,4,159,214,250,115,52,92,19");
asm(" .byte 153,54,245,103,241,222,245,254,191,19,127,126,92,158,63,19");
asm(" .byte 248,44,173,79,12,169,213,250,107,72,79,180,254,46,210,11");
asm(" .byte 173,127,41,230,47,214,250,19,208,191,83,235,31,70,105,215");
asm(" .byte 250,163,241,61,169,245,147,164,110,90,255,55,40,27,180,62");
asm(" .byte 157,137,126,173,255,43,148,173,90,255,231,40,59,181,62,53");
asm(" .byte 95,208,250,239,163,236,211,250,219,81,14,104,253,1,148,172");
asm(" .byte 214,255,51,148,15,180,254,107,164,247,90,255,86,15,191,55");
asm(" .byte 147,110,172,215,248,7,189,94,65,250,60,83,52,55,223,179");
asm(" .byte 162,187,171,185,55,217,178,50,217,220,44,154,191,119,95,243");
asm(" .byte 45,109,247,116,244,38,219,86,206,235,108,233,237,109,235,165");
asm(" .byte 201,247,180,182,54,247,18,64,29,0,90,218,90,150,117,52");
asm(" .byte 223,219,117,127,71,215,242,230,214,158,158,230,158,149,182,255");
asm(" .byte 112,29,193,182,202,133,122,187,91,127,208,150,20,237,201,238");
asm(" .byte 174,78,247,191,189,162,183,173,179,173,53,41,90,150,117,175");
asm(" .byte 76,2,65,219,202,149,93,221,205,157,221,173,45,201,142,238");
asm(" .byte 46,177,12,120,68,178,181,183,45,217,146,76,174,20,43,219");
asm(" .byte 90,150,139,21,109,43,90,123,30,16,203,30,108,91,217,141");
asm(" .byte 145,214,251,238,94,217,189,66,116,116,183,38,59,69,239,3");
asm(" .byte 32,122,133,184,167,45,153,236,88,209,214,125,247,242,150,7");
asm(" .byte 68,107,103,119,111,155,232,238,105,235,162,113,34,161,171,101");
asm(" .byte 69,27,22,238,90,158,236,6,242,123,24,121,75,107,107,91");
asm(" .byte 111,47,104,32,122,153,25,43,90,58,186,196,253,43,59,146");
asm(" .byte 109,162,43,217,221,78,95,91,87,181,52,183,36,219,86,117");
asm(" .byte 36,197,130,121,243,154,235,167,93,46,22,44,186,254,15,230");
asm(" .byte 53,207,152,214,32,237,128,252,23,20,122,239,108,255,130,238");
asm(" .byte 153,53,240,111,26,159,83,58,3,183,95,242,192,88,58,229");
asm(" .byte 243,12,101,3,12,241,39,218,57,142,118,116,140,163,53,154");
asm(" .byte 248,251,206,22,67,156,11,131,176,16,245,121,24,91,68,53");
asm(" .byte 16,47,166,26,4,221,70,53,12,196,29,84,99,222,157,84");
asm(" .byte 195,208,44,165,26,134,98,57,213,48,80,237,84,99,229,78");
asm(" .byte 170,97,168,122,168,198,74,73,170,97,196,86,81,13,163,242");
asm(" .byte 16,213,32,102,53,213,48,28,235,168,134,114,173,167,26,70");
asm(" .byte 102,3,213,48,114,155,168,134,81,219,76,53,12,92,63,213");
asm(" .byte 48,136,143,83,13,195,151,161,26,70,113,43,213,48,4,219");
asm(" .byte 168,134,241,219,65,53,140,104,184,113,234,251,203,132,56,30");
asm(" .byte 94,253,226,209,80,118,170,19,202,166,157,115,178,143,100,195");
asm(" .byte 39,94,114,218,15,145,242,63,146,31,247,198,134,124,251,231");
asm(" .byte 47,56,170,189,84,107,223,161,181,23,107,237,133,90,187,81");
asm(" .byte 107,207,210,218,182,214,78,104,109,75,107,199,180,182,169,181");
asm(" .byte 195,90,91,104,237,83,159,149,219,39,180,246,81,173,157,213");
asm(" .byte 218,7,181,246,160,214,222,167,181,7,180,246,110,173,189,83");
asm(" .byte 107,111,211,218,25,106,239,204,20,68,54,83,8,153,79,28");
asm(" .byte 25,103,101,114,33,11,188,21,83,29,115,32,237,132,6,166");
asm(" .byte 58,17,145,118,34,118,218,249,101,177,120,124,27,202,17,240");
asm(" .byte 58,254,42,116,19,117,20,227,81,124,15,218,143,100,133,213");
asm(" .byte 247,158,16,183,20,46,40,78,55,158,42,22,143,101,15,74");
asm(" .byte 123,105,98,142,176,222,118,232,155,97,101,130,194,218,91,129");
asm(" .byte 57,185,163,144,171,5,88,146,171,176,55,225,251,173,5,200");
asm(" .byte 54,236,206,231,118,61,230,145,188,27,151,150,61,95,104,231");
asm(" .byte 84,208,188,49,31,31,188,33,31,19,99,157,184,137,98,87");
asm(" .byte 59,97,123,172,19,51,171,157,195,128,127,116,74,42,251,228");
asm(" .byte 228,76,46,96,103,114,193,58,170,83,217,20,218,134,213,154");
asm(" .byte 45,218,129,188,109,167,242,69,43,152,15,3,14,244,56,84");
asm(" .byte 27,24,115,113,88,213,78,63,112,60,54,9,56,0,27,163");
asm(" .byte 49,154,7,154,126,87,44,86,73,24,240,140,224,48,151,214");
asm(" .byte 14,216,173,217,8,230,225,152,28,163,190,176,223,118,198,217");
asm(" .byte 77,142,156,251,182,59,135,230,199,121,157,56,230,210,183,113");
asm(" .byte 214,18,109,253,121,238,250,10,103,80,220,146,59,94,44,62");
asm(" .byte 28,113,219,153,156,24,252,73,62,180,243,137,35,196,187,127");
asm(" .byte 125,243,205,55,21,31,162,224,131,9,62,68,193,7,147,215");
asm(" .byte 158,10,90,17,115,84,170,254,101,232,11,59,51,201,176,51");
asm(" .byte 100,146,142,9,59,91,112,233,229,125,5,120,46,209,102,98");
asm(" .byte 28,115,11,68,11,142,251,49,162,199,203,199,32,244,68,225");
asm(" .byte 182,65,119,208,94,155,165,249,58,142,48,227,70,124,241,188");
asm(" .byte 154,59,29,125,16,176,69,245,175,148,60,221,18,213,232,160");
asm(" .byte 182,105,17,191,82,58,125,219,93,124,118,107,158,232,15,251");
asm(" .byte 240,68,44,156,250,126,8,252,136,105,122,17,243,240,35,54");
asm(" .byte 2,63,162,160,89,167,35,198,123,137,185,252,200,22,20,95");
asm(" .byte 136,111,151,0,38,166,237,9,102,112,75,76,219,19,204,230");
asm(" .byte 195,145,97,248,166,230,17,207,136,119,132,111,140,134,111,26");
asm(" .byte 224,97,62,171,76,166,5,184,183,235,124,213,233,162,241,160");
asm(" .byte 157,205,61,57,23,242,176,170,93,156,106,14,225,34,248,3");
asm(" .byte 208,217,136,168,118,158,152,40,142,131,254,28,201,41,68,60");
asm(" .byte 196,216,82,51,237,92,58,169,60,110,178,30,154,220,38,248");
asm(" .byte 147,197,226,27,113,141,47,212,142,177,124,162,30,249,4,234");
asm(" .byte 166,72,29,102,249,132,6,155,242,98,253,139,71,97,45,174");
asm(" .byte 81,242,73,64,62,9,200,167,22,242,73,120,228,147,24,65");
asm(" .byte 62,22,240,90,26,29,9,230,67,66,151,15,157,75,200,232");
asm(" .byte 244,228,166,220,116,192,37,52,25,193,197,108,73,104,50,130");
asm(" .byte 75,122,152,248,84,9,152,199,130,70,54,206,231,250,177,96");
asm(" .byte 48,27,52,215,100,227,3,248,38,50,133,199,173,148,219,7");
asm(" .byte 45,185,4,236,139,48,223,114,108,243,154,188,129,253,13,92");
asm(" .byte 8,29,180,30,195,183,39,179,180,230,24,109,205,179,201,81");
asm(" .byte 167,127,56,57,38,52,57,254,26,114,60,108,52,229,126,98");
asm(" .byte 66,94,208,35,162,231,180,213,148,195,249,58,118,88,52,229");
asm(" .byte 136,118,163,46,83,24,115,69,42,63,123,194,208,57,126,114");
asm(" .byte 253,8,114,173,213,248,73,237,4,203,85,241,25,122,178,157");
asm(" .byte 206,96,132,101,170,228,123,24,56,3,216,127,208,44,203,88");
asm(" .byte 217,163,26,237,252,89,150,212,197,176,73,250,90,237,132,236");
asm(" .byte 62,23,151,58,27,116,38,232,108,4,103,0,151,189,38,171");
asm(" .byte 206,96,216,149,117,198,149,105,128,207,187,178,207,126,123,65");
asm(" .byte 56,242,124,156,109,173,110,211,21,239,160,19,65,240,98,82");
asm(" .byte 209,14,230,227,88,219,176,214,162,30,235,126,123,15,124,85");
asm(" .byte 120,160,108,207,71,173,106,214,223,91,114,6,93,73,64,199");
asm(" .byte 108,208,64,116,89,37,26,190,25,109,214,215,164,205,98,218");
asm(" .byte 44,31,218,66,68,27,209,96,165,242,94,250,160,103,199,34");
asm(" .byte 76,135,178,235,227,1,131,239,135,148,205,114,249,111,245,57");
asm(" .byte 17,15,108,144,109,15,201,36,196,103,218,107,91,149,108,35");
asm(" .byte 144,109,88,176,109,7,174,48,203,148,240,33,100,172,12,179");
asm(" .byte 172,229,254,48,110,73,31,236,149,179,60,211,180,182,255,58");
asm(" .byte 202,167,153,140,71,241,150,214,34,127,68,235,141,229,245,20");
asm(" .byte 239,195,229,61,191,33,125,143,92,195,245,37,246,90,217,31");
asm(" .byte 198,103,132,177,94,196,110,202,187,244,152,153,194,35,95,22");
asm(" .byte 143,19,63,118,33,14,160,53,149,127,167,181,51,24,11,243");
asm(" .byte 88,88,243,115,163,248,172,24,108,211,105,252,170,43,83,249");
asm(" .byte 49,87,73,250,232,91,187,149,118,250,2,123,178,65,107,109");
asm(" .byte 118,207,185,153,220,222,233,20,147,44,203,230,46,200,20,30");
asm(" .byte 13,4,48,190,44,187,144,98,15,178,65,224,93,3,218,17");
asm(" .byte 200,164,214,173,37,191,107,97,179,251,140,151,179,47,79,4");
asm(" .byte 252,92,58,227,173,46,190,48,207,85,231,241,4,226,171,87");
asm(" .byte 159,253,251,71,205,239,8,113,225,95,30,191,217,198,94,35");
asm(" .byte 188,215,173,245,105,71,233,232,44,236,197,141,183,192,207,77");
asm(" .byte 245,114,29,26,159,195,114,13,40,127,13,251,50,11,248,149");
asm(" .byte 76,8,102,63,244,203,44,197,67,242,219,11,246,219,185,224");
asm(" .byte 172,76,110,30,138,107,183,48,103,22,225,5,204,177,171,200");
asm(" .byte 14,192,254,89,132,127,237,144,245,207,99,91,217,131,239,36");
asm(" .byte 135,133,144,67,19,228,160,240,183,19,126,179,186,100,227,79");
asm(" .byte 92,252,114,78,239,159,242,244,79,92,242,178,107,247,66,26");
asm(" .byte 221,237,88,43,202,251,30,188,118,175,251,253,160,129,58,91");
asm(" .byte 237,208,185,239,25,200,20,116,125,77,213,145,190,254,20,178");
asm(" .byte 79,101,7,173,189,185,239,159,83,182,171,202,254,41,187,27");
asm(" .byte 128,221,61,60,190,252,61,132,239,21,248,126,179,6,67,244");
asm(" .byte 185,116,127,131,117,79,8,166,147,233,29,12,148,225,86,157");
asm(" .byte 133,222,186,179,208,187,211,135,222,139,61,244,158,98,122,191");
asm(" .byte 238,186,138,222,193,235,184,14,150,225,86,159,133,222,138,179");
asm(" .byte 208,187,206,135,222,207,198,121,248,203,114,119,215,109,28,105");
asm(" .byte 221,117,37,122,163,108,71,148,255,9,176,29,137,122,236,72");
asm(" .byte 59,235,236,224,228,189,146,55,53,114,45,210,97,131,207,80");
asm(" .byte 169,111,145,31,64,155,117,79,155,227,234,121,163,59,39,229");
asm(" .byte 206,105,196,156,128,178,79,92,71,112,142,109,156,227,125,119");
asm(" .byte 27,130,206,178,178,85,22,206,175,5,219,88,99,202,120,148");
asm(" .byte 226,192,158,25,105,103,199,229,210,222,182,207,72,151,226,135");
asm(" .byte 26,246,165,63,102,63,21,241,241,89,186,47,10,178,77,117");
asm(" .byte 239,47,50,246,115,105,141,178,207,86,126,7,215,252,75,35");
asm(" .byte 236,211,162,236,211,162,12,119,149,199,167,69,217,167,69,45");
asm(" .byte 137,131,218,17,246,111,239,3,63,125,55,249,14,21,130,237");
asm(" .byte 164,187,145,138,55,23,193,118,168,88,4,241,91,229,250,186");
asm(" .byte 180,179,117,114,159,27,159,16,29,147,56,30,13,74,255,88");
asm(" .byte 37,227,168,84,158,232,184,5,99,87,243,247,144,198,119,245");
asm(" .byte 125,177,220,111,213,76,175,239,212,246,25,84,251,196,252,90");
asm(" .byte 230,71,8,52,108,187,92,202,238,20,108,238,182,139,83,206");
asm(" .byte 9,91,142,145,221,138,66,47,110,63,93,116,99,234,40,199");
asm(" .byte 114,81,142,241,253,120,166,244,110,56,26,16,167,205,142,186");
asm(" .byte 119,69,121,231,244,234,170,247,78,224,213,35,93,190,208,162");
asm(" .byte 74,200,165,160,116,58,204,244,121,245,160,151,228,238,222,67");
asm(" .byte 211,78,0,119,102,186,147,239,192,62,99,158,184,158,244,50");
asm(" .byte 115,79,89,55,239,122,231,6,215,191,172,198,252,131,240,35");
asm(" .byte 119,93,221,148,15,185,247,218,121,121,117,158,239,154,150,41");
asm(" .byte 252,227,23,197,227,165,56,2,235,65,159,14,45,98,31,55");
asm(" .byte 31,245,129,154,20,206,71,218,245,183,33,179,41,31,176,214");
asm(" .byte 100,137,78,117,239,246,224,45,16,222,153,69,240,156,247,172");
asm(" .byte 211,23,251,188,88,84,244,77,4,125,20,187,197,235,198,58");
asm(" .byte 89,224,255,0,235,93,55,183,41,31,195,26,194,220,232,44");
asm(" .byte 184,16,120,81,43,90,141,90,233,75,126,233,161,183,6,244");
asm(" .byte 134,89,175,166,64,135,98,117,165,248,177,106,189,155,239,200");
asm(" .byte 20,158,155,178,55,119,62,197,86,248,102,200,251,77,85,39");
asm(" .byte 190,53,12,144,156,154,242,227,69,223,209,241,98,195,209,40");
asm(" .byte 207,9,72,62,84,45,116,121,158,41,236,2,124,163,72,59");
asm(" .byte 27,177,54,217,55,27,227,100,75,108,140,45,249,66,234,151");
asm(" .byte 188,79,166,29,90,203,43,119,157,7,134,97,100,159,161,7");
asm(" .byte 5,31,62,172,163,28,206,228,175,199,135,21,26,31,66,242");
asm(" .byte 78,51,100,239,179,212,222,107,246,186,119,33,181,119,196,70");
asm(" .byte 85,164,75,243,61,123,247,163,85,207,239,168,188,198,115,184");
asm(" .byte 91,237,174,147,247,24,149,35,113,227,84,232,132,138,181,76");
asm(" .byte 182,103,23,177,125,163,115,102,208,61,187,193,181,249,110,76");
asm(" .byte 50,5,229,185,201,75,114,116,174,16,151,87,237,174,91,226");
asm(" .byte 198,40,99,216,238,5,44,186,151,159,9,179,155,97,40,102");
asm(" .byte 166,121,223,226,188,136,11,99,150,97,12,13,166,116,174,172");
asm(" .byte 234,82,204,13,27,116,233,72,235,144,29,251,77,177,248,240");
asm(" .byte 215,137,141,137,47,196,131,209,236,171,8,198,2,93,126,60");
asm(" .byte 82,231,59,172,114,76,35,208,16,102,59,225,222,27,70,226");
asm(" .byte 101,34,147,179,32,83,230,139,59,127,184,220,12,78,160,208");
asm(" .byte 239,255,202,127,145,79,126,110,70,19,202,146,156,37,170,49");
asm(" .byte 86,237,250,48,194,245,80,145,109,168,150,151,81,119,150,239");
asm(" .byte 99,29,53,127,55,207,211,239,92,228,199,148,95,97,250,102");
asm(" .byte 19,220,77,124,143,27,110,239,17,94,43,198,246,152,120,85");
asm(" .byte 195,60,53,93,91,180,223,169,16,251,29,17,130,172,205,213");
asm(" .byte 185,81,212,14,203,246,104,107,191,19,168,165,120,126,109,46");
asm(" .byte 128,51,35,115,36,251,29,138,239,139,102,32,15,61,47,208");
asm(" .byte 188,144,185,215,185,40,154,201,21,77,163,52,86,65,99,177");
asm(" .byte 161,99,163,104,172,86,222,71,96,255,128,127,239,16,59,63");
asm(" .byte 36,103,200,251,62,80,44,110,151,242,79,149,238,160,42,231");
asm(" .byte 164,238,43,148,47,32,222,184,57,34,232,174,48,247,148,252");
asm(" .byte 243,72,184,225,135,182,71,60,246,223,132,76,195,124,231,87");
asm(" .byte 50,77,0,143,146,205,205,35,200,240,46,206,149,12,39,139");
asm(" .byte 144,60,151,91,70,154,19,68,124,33,52,125,189,158,113,210");
asm(" .byte 222,105,143,137,179,156,207,24,231,123,93,157,48,165,61,13");
asm(" .byte 3,15,229,118,84,191,250,243,226,241,4,223,145,99,223,68");
asm(" .byte 7,6,100,44,96,10,210,131,65,148,151,81,222,194,94,54");
asm(" .byte 100,139,3,208,135,70,156,175,129,117,185,208,0,228,252,10");
asm(" .byte 100,63,16,41,141,85,208,216,190,161,99,163,6,88,31,40");
asm(" .byte 150,181,54,184,250,96,216,107,220,56,192,176,87,231,213,158");
asm(" .byte 99,172,243,251,17,55,140,200,59,138,13,32,71,147,242,115");
asm(" .byte 62,185,211,184,107,231,202,182,229,106,230,65,233,156,145,77");
asm(" .byte 0,111,13,232,50,233,39,241,217,136,201,182,187,222,28,94");
asm(" .byte 15,60,220,231,177,19,151,17,46,240,37,14,254,236,251,170");
asm(" .byte 120,60,42,206,212,143,139,56,70,81,185,11,165,187,42,78");
asm(" .byte 137,88,67,237,191,138,103,221,252,17,231,8,62,230,216,41");
asm(" .byte 4,90,43,52,90,71,105,180,142,246,208,170,229,71,74,54");
asm(" .byte 205,244,232,253,250,251,239,190,93,241,172,153,99,158,131,136");
asm(" .byte 11,197,228,114,204,19,241,196,60,127,250,217,208,24,162,13");
asm(" .byte 49,132,62,215,149,7,248,241,51,156,153,136,40,231,230,97");
asm(" .byte 235,42,31,170,147,177,81,18,245,38,196,97,157,110,191,218");
asm(" .byte 249,227,171,83,128,79,229,73,87,77,75,250,140,110,172,227");
asm(" .byte 135,151,252,242,102,236,231,146,175,206,180,151,19,248,222,18");
asm(" .byte 6,159,198,104,124,170,212,248,52,86,231,147,210,33,186,231");
asm(" .byte 195,15,208,155,203,96,141,188,195,12,114,12,64,253,87,40");
asm(" .byte 54,169,127,219,245,177,181,88,195,143,46,162,251,126,166,73");
asm(" .byte 190,53,72,59,181,222,74,59,20,75,61,107,63,150,179,1");
asm(" .byte 95,9,189,120,8,99,126,56,14,107,240,202,223,81,28,37");
asm(" .byte 38,200,124,145,14,51,71,208,251,210,70,156,203,12,238,12");
asm(" .byte 27,29,67,221,51,193,155,0,120,55,82,28,149,199,37,99");
asm(" .byte 16,247,177,154,101,254,113,74,204,42,191,157,184,126,176,174");
asm(" .byte 124,7,80,111,26,186,174,6,88,183,12,205,94,171,24,124");
asm(" .byte 180,246,126,115,21,251,52,83,187,139,156,80,185,65,158,127");
asm(" .byte 153,250,206,49,178,247,45,230,108,180,76,98,90,20,254,136");
asm(" .byte 202,93,171,183,46,31,28,228,55,188,120,134,219,143,123,183");
asm(" .byte 212,236,83,161,88,124,126,164,119,28,253,158,27,71,204,72");
asm(" .byte 62,133,228,212,8,25,145,141,138,105,57,91,178,203,139,103");
asm(" .byte 72,27,83,193,119,198,184,254,30,96,53,229,126,72,54,7");
asm(" .byte 243,106,103,208,219,222,222,236,222,137,153,220,30,206,159,45");
asm(" .byte 135,143,125,52,24,204,146,238,17,61,253,149,67,243,231,180");
asm(" .byte 214,201,201,105,167,111,226,158,236,158,139,100,222,141,230,42");
asm(" .byte 255,65,103,255,108,56,83,149,195,231,54,110,156,149,202,63");
asm(" .byte 163,125,167,124,254,222,49,103,210,208,227,67,3,225,238,244");
asm(" .byte 193,125,227,220,71,242,148,83,121,220,131,247,23,30,188,113");
asm(" .byte 237,189,98,38,217,94,208,84,188,28,182,100,102,42,127,18");
asm(" .byte 113,183,202,49,232,111,151,231,50,47,253,222,183,226,158,119");
asm(" .byte 65,130,61,96,12,125,159,184,95,163,137,214,208,113,199,61");
asm(" .byte 111,133,113,237,205,6,129,228,22,242,27,182,207,126,15,0");
asm(" .byte 207,105,207,94,87,122,246,90,163,191,99,129,126,183,15,253");
asm(" .byte 245,123,207,144,63,66,218,209,248,207,127,27,116,243,167,74");
asm(" .byte 47,77,79,252,173,206,122,173,118,78,244,179,192,185,85,247");
asm(" .byte 125,108,234,112,115,236,214,114,188,198,54,109,254,169,51,109");
asm(" .byte 154,138,233,149,220,189,111,47,42,63,31,49,165,109,209,115");
asm(" .byte 10,42,142,143,112,238,33,50,204,27,68,152,207,117,152,115");
asm(" .byte 52,31,210,251,225,89,222,4,136,78,90,179,194,110,66,89");
asm(" .byte 226,140,7,158,4,108,117,208,218,255,158,190,158,254,62,225");
asm(" .byte 197,23,255,175,242,29,33,236,131,115,202,12,196,67,192,75");
asm(" .byte 112,17,240,231,29,142,45,43,172,37,165,123,234,127,242,152");
asm(" .byte 95,140,170,232,36,185,189,100,55,229,94,178,151,228,138,141");
asm(" .byte 193,60,213,83,26,50,37,159,166,214,160,216,237,16,227,11");
asm(" .byte 49,254,223,106,248,85,236,164,227,215,223,225,93,124,11,229");
asm(" .byte 59,185,242,149,42,238,34,220,7,61,184,15,248,224,214,227");
asm(" .byte 50,221,199,148,238,120,90,60,164,203,141,112,208,111,33,182");
asm(" .byte 23,165,173,140,114,60,175,199,225,240,31,219,189,249,28,181");
asm(" .byte 94,63,189,17,52,164,157,242,189,177,175,116,110,232,62,127");
asm(" .byte 1,234,6,172,189,14,115,250,41,23,86,39,227,112,242,241");
asm(" .byte 61,24,75,212,201,92,88,123,52,13,154,50,133,203,127,95");
asm(" .byte 60,126,71,131,140,11,40,95,6,159,91,88,140,111,175,35");
asm(" .byte 182,110,108,144,111,3,255,114,53,157,135,148,171,127,33,123");
asm(" .byte 173,60,35,22,229,168,82,142,250,205,5,189,111,156,71,122");
asm(" .byte 237,89,119,20,214,61,85,95,126,35,81,111,54,124,183,127");
asm(" .byte 94,135,119,243,160,60,47,236,147,119,149,121,210,62,55,191");
asm(" .byte 17,252,40,253,55,79,97,124,225,63,8,177,14,101,245,14");
asm(" .byte 33,106,127,33,196,82,20,157,47,127,13,187,185,170,94,210");
asm(" .byte 212,168,221,73,130,28,63,26,156,107,139,96,95,125,44,179");
asm(" .byte 24,191,115,124,231,179,161,241,158,201,57,134,40,227,94,173");
asm(" .byte 114,178,160,63,106,73,222,197,38,165,157,7,193,187,80,125");
asm(" .byte 153,215,225,73,146,215,135,62,45,30,127,16,107,108,130,207");
asm(" .byte 60,21,72,187,62,243,83,196,71,166,40,159,59,202,93,127");
asm(" .byte 196,99,7,49,135,250,31,106,115,44,166,141,198,223,227,241");
asm(" .byte 93,60,175,192,125,194,111,112,238,237,143,64,203,170,25,82");
asm(" .byte 142,55,94,117,166,28,3,204,251,219,64,23,249,105,157,79");
asm(" .byte 234,221,80,241,227,75,216,61,181,95,147,229,21,100,248,249");
asm(" .byte 20,163,204,240,151,243,44,150,179,161,201,217,228,28,187,219");
asm(" .byte 214,198,40,215,54,141,115,120,106,221,119,61,235,126,151,228");
asm(" .byte 134,253,13,76,72,59,23,126,46,191,133,24,247,134,58,255");
asm(" .byte 189,198,1,211,83,55,116,127,65,150,167,162,139,226,172,69");
asm(" .byte 117,254,123,24,203,123,80,251,13,179,78,142,82,177,40,215");
asm(" .byte 126,58,107,213,75,125,185,18,235,125,160,189,23,214,212,203");
asm(" .byte 119,188,25,66,204,62,96,159,73,119,77,253,90,231,5,91");
asm(" .byte 210,220,143,243,190,147,98,249,134,62,135,236,224,102,216,94");
asm(" .byte 146,179,242,77,53,62,177,101,63,191,155,86,152,242,141,228");
asm(" .byte 45,214,15,154,115,169,207,124,147,127,115,225,29,167,220,97");
asm(" .byte 2,241,58,225,160,53,137,39,155,44,87,199,74,119,241,41");
asm(" .byte 159,82,142,247,204,216,188,191,85,190,145,216,240,33,13,40");
asm(" .byte 141,40,119,160,220,137,210,137,210,131,146,68,81,182,186,233");
asm(" .byte 221,27,242,182,49,214,105,152,128,82,39,223,51,41,78,177");
asm(" .byte 57,151,121,93,64,28,146,57,19,249,86,64,99,243,2,162");
asm(" .byte 74,253,174,134,222,22,182,25,178,111,240,156,59,48,246,36");
asm(" .byte 198,108,206,121,102,12,249,246,78,123,163,53,66,238,188,62");
asm(" .byte 23,126,2,248,58,1,62,141,98,164,56,219,153,191,50,196");
asm(" .byte 177,254,93,66,68,158,41,151,87,254,233,255,94,122,158,150");
asm(" .byte 53,225,60,249,180,255,28,115,167,16,68,47,217,186,91,64");
asm(" .byte 47,209,73,122,65,111,251,148,167,85,126,181,245,83,233,155");
asm(" .byte 18,158,223,22,45,192,126,9,198,244,129,249,67,134,177,60");
asm(" .byte 48,115,24,38,226,3,115,61,195,212,104,48,211,216,239,80");
asm(" .byte 172,54,67,198,82,111,12,7,255,231,62,240,180,230,100,172");
asm(" .byte 217,52,183,41,191,121,180,180,135,207,124,41,117,52,161,217");
asm(" .byte 195,31,125,201,247,98,237,247,55,53,218,247,191,240,249,110");
asm(" .byte 105,223,239,243,249,30,133,76,109,206,237,147,29,136,93,33");
asm(" .byte 109,244,11,147,83,206,56,254,70,188,255,66,72,158,132,53");
asm(" .byte 29,33,186,127,79,126,86,237,85,27,255,152,199,149,172,244");
asm(" .byte 57,187,71,165,75,185,172,111,195,239,98,238,49,218,251,214");
asm(" .byte 81,114,239,189,188,247,15,133,92,191,130,226,106,49,188,236");
asm(" .byte 143,124,226,47,251,223,138,225,229,248,31,159,248,203,225,215");
asm(" .byte 98,120,125,121,227,19,127,125,25,96,152,168,15,204,139,12");
asm(" .byte 19,247,192,64,181,171,18,229,216,187,82,201,249,157,176,40");
asm(" .byte 197,88,253,231,166,157,31,140,17,165,189,213,104,243,149,220");
asm(" .byte 247,107,243,219,49,191,153,231,215,104,191,69,163,249,74,15");
asm(" .byte 94,211,230,207,194,252,91,121,190,165,189,105,134,75,111,150");
asm(" .byte 153,220,243,218,252,48,230,47,224,249,180,31,146,217,169,144");
asm(" .byte 148,217,40,150,25,217,140,58,126,75,74,148,239,50,85,53");
asm(" .byte 229,220,89,101,29,199,28,118,233,183,47,123,28,239,239,206");
asm(" .byte 148,206,190,126,122,100,157,127,249,244,200,58,255,43,159,239");
asm(" .byte 113,237,251,211,62,223,77,237,45,243,140,92,1,231,255,104");
asm(" .byte 239,42,239,118,42,40,121,240,234,105,201,131,3,83,210,206");
asm(" .byte 81,196,61,148,23,127,214,92,155,83,60,137,193,119,214,75");
asm(" .byte 126,28,186,150,227,10,242,181,91,49,127,119,189,204,35,5");
asm(" .byte 230,164,10,110,28,30,203,228,54,96,60,67,227,192,65,111");
asm(" .byte 85,15,161,191,137,231,25,218,188,229,24,95,197,243,200,231");
asm(" .byte 220,134,246,86,126,235,165,61,30,58,121,230,30,207,99,255");
asm(" .byte 20,229,122,34,215,38,215,231,168,55,126,174,43,149,31,255");
asm(" .byte 154,254,94,241,42,227,201,249,16,223,84,76,246,63,56,251");
asm(" .byte 79,105,191,43,93,64,254,73,12,245,89,139,48,246,119,66");
asm(" .byte 250,44,122,71,250,25,120,183,9,241,213,98,138,63,68,83");
asm(" .byte 254,89,107,117,110,29,250,54,245,77,234,175,201,149,124,153");
asm(" .byte 231,44,42,62,168,183,33,138,243,20,14,253,108,222,171,219");
asm(" .byte 58,15,142,215,61,56,220,88,143,215,213,113,220,61,130,237");
asm(" .byte 233,226,247,82,186,243,31,152,32,105,248,200,67,195,29,35");
asm(" .byte 216,161,219,62,41,195,247,15,3,255,189,17,108,210,124,13");
asm(" .byte 254,206,97,224,231,48,124,204,7,190,65,131,47,154,217,130");
asm(" .byte 31,252,52,134,143,251,192,215,104,240,70,188,41,127,210,186");
asm(" .byte 102,8,236,100,192,110,66,172,117,227,191,165,242,77,55,209");
asm(" .byte 91,236,174,163,227,197,142,163,229,122,179,251,54,75,111,213");
asm(" .byte 7,27,100,252,188,148,218,53,41,231,255,67,151,213,93,119");
asm(" .byte 29,226,51,138,225,118,34,150,219,64,185,0,241,224,252,126");
asm(" .byte 212,91,169,192,224,101,191,42,22,243,40,71,80,126,135,114");
asm(" .byte 20,229,4,202,169,175,202,239,251,223,125,231,134,60,197,223");
asm(" .byte 137,201,125,206,207,231,150,121,18,168,205,20,30,253,184,124");
asm(" .byte 207,162,248,236,6,232,186,254,123,61,117,255,186,158,126,11");
asm(" .byte 203,241,121,13,253,214,3,190,154,112,169,223,240,133,181,115");
asm(" .byte 126,133,247,183,52,218,155,99,144,223,122,137,239,150,252,237");
asm(" .byte 200,150,48,255,150,38,240,53,225,46,148,112,15,15,205,251");
asm(" .byte 220,90,56,151,239,46,66,251,77,8,205,191,134,227,12,178");
asm(" .byte 1,148,99,248,8,241,16,238,20,199,16,199,231,163,156,63");
asm(" .byte 181,0,115,211,231,82,71,54,107,122,115,197,151,229,223,79");
asm(" .byte 12,71,219,107,197,226,236,145,242,227,122,254,224,41,232,33");
asm(" .byte 221,123,55,3,199,73,186,251,215,201,251,9,232,57,116,164");
asm(" .byte 65,190,149,208,250,135,163,100,203,55,226,110,73,255,15,74");
asm(" .byte 249,55,7,33,206,221,158,255,177,107,19,10,155,128,103,23");
asm(" .byte 224,54,216,242,238,41,236,141,164,143,5,242,3,42,207,59");
asm(" .byte 19,115,255,183,184,235,129,142,170,58,243,247,189,153,9,8");
asm(" .byte 17,102,146,128,49,101,151,55,4,93,197,36,188,153,36,136");
asm(" .byte 24,106,32,225,159,130,14,33,80,142,101,75,38,201,32,216");
asm(" .byte 64,210,100,180,112,214,61,153,201,16,202,241,100,49,165,120");
asm(" .byte 182,167,186,205,139,160,165,221,216,14,172,235,73,45,122,38");
asm(" .byte 128,202,90,78,5,75,207,161,74,219,247,222,152,149,21,182");
asm(" .byte 198,214,118,93,255,48,251,251,222,189,147,121,25,130,210,218");
asm(" .byte 174,3,47,223,253,243,221,123,191,251,221,239,251,238,159,119");
asm(" .byte 239,125,187,109,235,15,187,197,250,195,180,223,91,243,134,228");
asm(" .byte 67,240,15,240,57,68,50,12,247,247,133,187,5,238,167,224");
asm(" .byte 222,140,254,150,214,44,54,207,238,27,126,5,182,104,61,252");
asm(" .byte 13,240,55,192,255,18,252,203,225,95,15,255,122,248,143,194");
asm(" .byte 159,2,15,14,162,158,5,40,243,128,143,239,239,162,99,140");
asm(" .byte 3,100,131,42,248,158,44,170,255,62,33,91,95,64,221,169");
asm(" .byte 239,162,189,49,20,86,48,78,221,157,5,125,163,239,192,127");
asm(" .byte 253,187,212,5,234,187,150,171,163,188,155,116,176,156,239,215");
asm(" .byte 160,244,231,242,47,79,127,60,63,147,254,103,72,79,125,23");
asm(" .byte 241,135,246,239,180,192,189,27,246,180,30,58,29,201,122,207");
asm(" .byte 105,252,46,195,231,221,229,156,207,85,128,103,64,107,47,210");
asm(" .byte 236,86,249,222,16,251,218,53,225,255,146,167,51,236,245,164");
asm(" .byte 125,227,195,34,156,137,181,148,31,131,47,179,192,171,52,142");
asm(" .byte 2,94,156,131,125,63,43,246,247,166,223,139,21,9,217,158");
asm(" .byte 9,26,173,241,141,170,29,58,230,231,52,198,84,61,41,123");
asm(" .byte 193,35,251,158,32,55,127,199,67,235,238,251,23,18,94,151");
asm(" .byte 62,83,145,173,249,227,234,114,178,131,14,51,149,153,139,79");
asm(" .byte 218,39,116,158,232,61,245,199,177,244,172,244,115,217,249,188");
asm(" .byte 232,57,121,105,44,61,71,124,92,111,62,47,122,94,124,127");
asm(" .byte 44,61,100,23,47,126,142,237,213,147,213,94,143,211,222,165");
asm(" .byte 207,145,63,63,0,61,180,255,42,189,31,250,195,75,169,73");
asm(" .byte 227,217,197,130,70,137,245,158,114,88,235,19,113,60,71,240");
asm(" .byte 156,33,55,250,186,139,111,240,131,199,78,60,51,240,40,120");
asm(" .byte 230,147,191,145,239,173,171,32,184,43,99,91,23,255,102,149");
asm(" .byte 121,74,238,27,62,253,110,234,194,73,192,87,1,79,248,251");
asm(" .byte 248,186,150,160,147,246,145,13,202,124,140,252,203,119,133,30");
asm(" .byte 114,251,81,116,0,225,71,17,166,1,38,0,31,7,124,1");
asm(" .byte 240,219,128,71,0,247,1,62,7,216,11,56,8,216,34,242");
asm(" .byte 150,50,235,154,69,219,17,119,8,113,97,63,175,59,217,218");
asm(" .byte 167,225,175,65,63,73,125,105,131,8,167,118,185,41,189,239");
asm(" .byte 81,244,103,244,78,199,254,46,64,177,197,75,226,189,227,251");
asm(" .byte 144,179,103,125,63,183,230,69,131,239,102,214,197,105,221,255");
asm(" .byte 146,240,219,223,255,79,181,229,33,139,122,62,131,60,94,88");
asm(" .byte 200,243,248,94,86,30,255,61,78,30,127,238,251,145,151,83");
asm(" .byte 169,34,170,183,245,30,206,198,107,190,127,64,79,46,5,109");
asm(" .byte 105,190,44,247,125,54,190,208,158,197,103,253,188,78,237,89");
asm(" .byte 117,26,188,74,190,144,206,28,190,131,231,17,204,202,227,123");
asm(" .byte 127,37,190,220,56,14,95,236,103,64,210,186,114,0,114,159");
asm(" .byte 56,199,199,129,39,1,207,209,129,252,95,49,86,141,167,30");
asm(" .byte 207,6,60,17,60,187,241,156,3,142,110,27,115,88,103,56");
asm(" .byte 124,25,121,188,134,202,244,93,110,207,115,169,15,182,246,40");
asm(" .byte 232,201,155,188,180,198,138,126,212,205,199,9,147,223,165,61");
asm(" .byte 26,168,7,244,230,164,130,180,194,95,12,72,103,65,178,251");
asm(" .byte 42,167,176,65,148,215,89,133,231,85,8,92,218,183,112,113");
asm(" .byte 28,220,54,129,123,81,224,94,11,220,13,234,229,54,158,112");
asm(" .byte 107,5,46,19,52,202,192,85,199,177,119,132,91,40,112,11");
asm(" .byte 4,238,255,140,164,46,164,121,74,124,34,126,62,3,158,205");
asm(" .byte 104,202,240,43,189,215,167,80,172,207,199,149,99,214,60,33");
asm(" .byte 174,114,217,144,197,184,234,181,145,177,251,86,36,186,150,0");
asm(" .byte 237,55,87,188,215,224,99,103,140,143,81,238,51,110,62,246");
asm(" .byte 56,49,146,105,239,105,214,62,2,190,14,90,40,198,146,86");
asm(" .byte 125,196,184,227,37,27,46,201,56,199,225,249,165,113,254,201");
asm(" .byte 134,227,18,235,233,83,175,48,159,120,121,36,51,238,176,228");
asm(" .byte 23,249,252,27,194,134,70,50,178,77,239,124,169,14,118,217");
asm(" .byte 243,214,51,54,222,62,168,146,9,125,195,143,253,173,150,60");
asm(" .byte 200,211,91,245,45,22,122,69,239,190,103,8,93,97,246,246");
asm(" .byte 22,103,26,175,7,173,30,219,59,186,77,98,95,71,88,232");
asm(" .byte 226,169,121,125,233,119,20,201,227,228,6,93,168,223,219,131");
asm(" .byte 34,92,70,248,128,8,191,108,92,110,211,195,255,20,239,104");
asm(" .byte 237,251,199,211,251,44,206,164,82,207,181,204,235,27,78,159");
asm(" .byte 109,90,143,122,172,159,199,229,253,0,228,101,229,141,152,239");
asm(" .byte 195,31,152,206,215,56,170,230,137,253,199,222,99,70,117,78");
asm(" .byte 223,240,214,17,62,87,152,141,240,66,218,127,140,176,219,71");
asm(" .byte 50,251,143,115,17,254,44,112,157,200,135,193,61,59,209,101");
asm(" .byte 210,188,56,189,199,215,9,252,185,54,252,115,149,192,159,117");
asm(" .byte 140,214,82,146,55,219,194,143,35,252,133,82,30,94,108,11");
asm(" .byte 31,64,248,225,185,60,252,111,210,225,144,139,125,55,208,248");
asm(" .byte 154,211,124,208,213,55,60,221,22,183,253,6,122,159,193,227");
asm(" .byte 246,185,172,245,208,228,31,223,225,245,216,80,73,243,195,216");
asm(" .byte 112,125,37,183,13,162,95,255,193,109,54,185,185,75,98,111");
asm(" .byte 87,84,102,230,17,245,211,250,134,175,181,201,32,141,3,74");
asm(" .byte 109,248,119,0,223,137,50,25,210,220,251,106,151,121,215,59");
asm(" .byte 84,255,23,196,124,122,44,156,192,190,139,167,231,252,25,204");
asm(" .byte 15,110,160,115,77,160,229,68,5,231,121,24,238,35,182,249");
asm(" .byte 11,181,215,57,204,53,114,178,202,158,98,43,91,65,217,71");
asm(" .byte 48,47,153,152,226,245,123,168,130,230,23,177,225,112,5,183");
asm(" .byte 87,233,250,73,182,52,249,72,211,131,52,31,94,226,105,150");
asm(" .byte 87,208,94,248,216,48,189,139,189,104,75,243,222,59,153,52");
asm(" .byte 57,72,179,1,105,126,43,210,20,88,115,184,216,112,174,109");
asm(" .byte 110,71,105,222,178,165,249,95,200,114,9,210,24,34,141,110");
asm(" .byte 205,93,98,195,103,203,185,29,75,167,121,221,150,230,34,210");
asm(" .byte 124,68,241,226,188,53,217,141,28,177,111,215,90,207,164,253");
asm(" .byte 176,158,216,91,217,107,135,212,135,254,1,115,236,135,196,59");
asm(" .byte 81,251,156,190,88,204,245,179,199,132,1,7,202,113,199,76");
asm(" .byte 154,107,74,30,107,15,167,78,227,66,198,246,14,83,127,56");
asm(" .byte 52,87,51,158,40,37,91,24,213,103,186,37,147,246,242,82");
asm(" .byte 223,224,153,69,123,226,34,134,59,157,94,205,164,183,206,55");
asm(" .byte 138,179,235,180,118,114,17,99,161,51,226,44,29,228,249,16");
asm(" .byte 205,121,206,32,254,140,152,251,92,13,157,7,229,207,70,231");
asm(" .byte 102,249,211,233,172,205,162,179,66,208,73,251,165,46,94,37");
asm(" .byte 157,236,51,210,153,144,62,157,206,199,125,99,233,164,119,177");
asm(" .byte 68,231,78,49,87,186,26,58,215,75,159,141,206,194,171,160");
asm(" .byte 243,61,117,44,157,111,170,156,206,115,162,15,191,26,58,227");
asm(" .byte 236,179,209,217,198,62,157,206,149,89,116,86,9,58,205,143");
asm(" .byte 198,218,156,119,127,155,209,81,58,51,188,1,121,63,233,96");
asm(" .byte 23,162,252,44,74,50,251,92,208,123,120,158,66,103,86,239");
asm(" .byte 177,174,105,234,124,6,3,1,58,131,2,59,194,222,159,99");
asm(" .byte 45,82,177,65,49,38,89,143,231,36,230,57,58,158,247,240");
asm(" .byte 76,132,191,22,240,212,81,198,100,143,215,122,23,64,103,207");
asm(" .byte 213,234,210,255,138,86,99,30,23,208,12,23,171,64,223,175");
asm(" .byte 74,44,177,207,35,39,30,245,28,127,101,196,243,34,158,29");
asm(" .byte 160,245,192,157,154,241,228,189,97,157,5,66,135,97,59,94");
asm(" .byte 119,85,239,4,223,20,212,221,11,222,205,2,143,138,71,253");
asm(" .byte 243,216,218,164,26,121,126,183,44,121,146,210,116,240,232,37");
asm(" .byte 45,233,157,222,195,247,255,178,54,147,185,13,83,150,242,147");
asm(" .byte 142,121,154,81,60,79,132,43,237,38,83,147,38,209,240,37");
asm(" .byte 218,19,83,93,107,42,250,3,240,179,231,242,126,166,25,126");
asm(" .byte 87,47,230,149,253,102,254,148,94,125,205,180,67,58,217,47");
asm(" .byte 37,176,4,56,123,76,137,213,37,243,129,35,39,122,138,215");
asm(" .byte 0,239,216,204,67,186,188,90,50,49,142,185,151,37,30,66");
asm(" .byte 93,71,172,50,165,156,213,116,118,124,129,12,136,244,11,250");
asm(" .byte 221,113,163,129,197,141,150,4,104,172,222,105,238,156,3,232");
asm(" .byte 106,79,174,101,108,142,44,109,72,106,137,56,72,139,155,206");
asm(" .byte 162,126,221,229,142,26,168,211,249,158,68,28,237,23,71,125");
asm(" .byte 251,117,41,7,245,3,157,191,199,88,129,49,31,173,155,1");
asm(" .byte 95,115,120,238,104,53,20,53,38,79,198,156,86,81,214,152");
asm(" .byte 19,232,28,179,178,151,182,186,190,94,144,160,179,122,141,166");
asm(" .byte 39,87,51,124,121,124,13,183,128,206,167,39,26,95,243,192");
asm(" .byte 175,206,136,158,118,203,218,105,71,117,189,238,44,138,106,40");
asm(" .byte 183,31,229,30,116,86,235,198,204,234,122,243,90,202,171,122");
asm(" .byte 31,241,229,149,2,208,172,40,205,166,234,212,12,143,28,49");
asm(" .byte 124,5,221,22,141,185,213,205,60,92,70,184,51,98,157,233");
asm(" .byte 164,112,159,83,27,117,51,86,145,148,230,21,155,210,116,175");
asm(" .byte 140,186,203,82,96,73,94,127,42,245,54,218,220,160,54,167");
asm(" .byte 179,129,105,121,32,89,133,76,208,56,64,146,153,38,81,122");
asm(" .byte 7,243,81,95,154,143,250,230,195,95,120,224,149,56,173,179");
asm(" .byte 22,69,192,79,201,13,254,123,162,197,132,39,45,217,135,50");
asm(" .byte 162,126,184,231,83,155,116,22,116,39,167,78,143,38,105,236");
asm(" .byte 65,241,84,30,242,245,32,95,146,53,9,227,95,15,250,53");
asm(" .byte 143,180,180,215,112,172,219,139,178,125,146,124,93,52,159,211");
asm(" .byte 172,18,143,29,116,31,14,201,173,116,151,150,244,131,7,44");
asm(" .byte 192,215,44,255,5,244,95,9,71,94,237,134,28,52,44,38");
asm(" .byte 188,111,1,47,162,196,141,163,208,223,32,173,101,79,209,140");
asm(" .byte 84,69,94,94,202,89,158,63,37,223,155,135,180,208,48,127");
asm(" .byte 146,252,233,124,166,172,66,31,73,50,229,25,74,198,144,94");
asm(" .byte 82,122,141,116,28,149,79,242,104,15,147,87,123,32,151,144");
asm(" .byte 63,33,239,105,58,188,63,141,27,27,82,169,73,180,31,101");
asm(" .byte 96,40,78,107,35,121,163,208,219,165,43,106,163,233,62,134");
asm(" .byte 54,162,182,115,68,44,25,145,161,107,247,160,204,54,228,155");
asm(" .byte 74,104,201,9,39,123,146,222,59,103,232,197,247,206,112,231");
asm(" .byte 178,10,79,46,171,244,180,50,54,153,116,210,209,224,213,123");
asm(" .byte 138,134,140,158,5,199,12,105,157,197,67,15,248,143,182,142");
asm(" .byte 37,103,208,121,244,163,123,116,162,247,155,230,147,186,236,205");
asm(" .byte 49,123,147,79,234,172,1,254,53,79,65,143,171,146,146,190");
asm(" .byte 39,63,5,153,150,115,215,37,29,43,143,147,14,205,217,163");
asm(" .byte 199,13,87,224,223,245,156,134,167,244,93,211,186,140,39,139");
asm(" .byte 162,70,183,75,51,126,56,41,98,236,25,214,140,143,19,245");
asm(" .byte 230,132,220,13,49,204,31,30,150,165,175,36,53,61,163,55");
asm(" .byte 109,104,55,57,152,99,74,238,93,144,165,8,218,218,171,15");
asm(" .byte 64,182,115,3,225,4,228,123,8,241,39,136,62,186,47,73");
asm(" .byte 130,124,175,135,92,207,99,97,146,143,159,203,30,233,53,196");
asm(" .byte 159,101,36,207,136,43,20,178,63,57,45,251,129,102,211,93");
asm(" .byte 9,94,145,252,187,34,134,236,229,237,179,127,106,175,238,43");
asm(" .byte 120,66,143,233,223,53,246,79,77,72,109,105,157,32,252,10");
asm(" .byte 224,187,184,94,32,127,147,112,201,174,216,113,45,188,225,61");
asm(" .byte 134,10,92,79,101,204,32,189,33,92,159,51,1,156,200,152");
asm(" .byte 124,73,143,232,28,172,28,148,76,46,251,163,118,65,254,0");
asm(" .byte 237,156,3,187,193,88,37,221,225,228,145,131,14,179,11,114");
asm(" .byte 231,80,247,3,39,63,9,157,227,56,106,140,228,198,64,91");
asm(" .byte 26,197,247,230,234,212,158,180,255,46,218,208,107,196,26,246");
asm(" .byte 122,134,97,95,136,111,68,3,233,13,149,75,113,164,139,164");
asm(" .byte 147,132,3,222,79,162,244,180,150,36,109,234,165,187,28,138");
asm(" .byte 144,143,165,147,142,77,123,9,215,67,122,70,115,189,116,94");
asm(" .byte 157,5,209,81,93,164,60,201,255,197,235,162,150,251,147,100");
asm(" .byte 109,131,37,107,181,186,67,247,234,144,47,131,108,252,93,180");
asm(" .byte 55,15,246,92,170,195,92,195,223,109,204,170,138,25,242,108");
asm(" .byte 205,240,214,117,27,202,134,110,131,53,19,15,167,152,14,182");
asm(" .byte 22,180,172,147,62,174,62,96,58,86,238,36,29,153,179,95");
asm(" .byte 141,27,141,116,175,69,195,58,221,17,120,81,79,251,165,28");
asm(" .byte 232,220,203,120,116,132,235,191,208,229,134,165,202,148,128,230");
asm(" .byte 205,101,17,239,104,154,172,184,129,79,136,43,145,62,33,29");
asm(" .byte 226,114,244,71,141,107,201,238,213,105,198,141,95,233,54,114");
asm(" .byte 26,209,47,98,62,250,196,23,99,70,151,87,51,74,142,198");
asm(" .byte 141,55,208,22,212,190,116,254,146,89,109,11,250,134,180,228");
asm(" .byte 47,200,38,32,28,97,86,31,144,214,137,59,83,169,135,29");
asm(" .byte 137,181,186,156,88,199,236,253,137,165,23,198,20,211,59,125");
asm(" .byte 143,49,80,189,239,138,250,176,0,249,230,232,221,198,124,64");
asm(" .byte 46,27,21,144,147,74,9,54,204,106,111,162,131,108,229,35");
asm(" .byte 214,157,68,149,198,55,0,211,239,19,30,70,26,10,163,115");
asm(" .byte 220,212,78,91,145,38,29,71,54,44,45,95,171,144,38,90");
asm(" .byte 221,107,108,7,140,5,246,26,77,128,255,128,120,89,82,248");
asm(" .byte 56,1,227,5,178,151,81,53,238,113,48,191,103,241,201,17");
asm(" .byte 79,13,158,42,240,138,206,79,21,87,181,211,125,25,9,201");
asm(" .byte 173,36,24,243,114,168,54,29,190,158,246,181,166,230,70,101");
asm(" .byte 201,157,148,93,17,216,211,4,100,173,33,143,198,5,242,180");
asm(" .byte 104,50,85,48,4,126,5,243,144,167,81,76,125,190,178,216");
asm(" .byte 92,237,232,133,13,108,50,61,249,104,35,31,227,54,212,253");
asm(" .byte 53,208,161,155,178,43,64,115,252,5,146,51,64,107,52,11");
asm(" .byte 52,232,27,217,111,90,35,187,132,49,64,169,213,111,175,78");
asm(" .byte 118,77,214,116,12,98,28,14,216,27,210,171,168,155,250,108");
asm(" .byte 77,151,93,238,36,149,241,7,58,231,241,106,171,49,49,177");
asm(" .byte 18,125,113,157,238,70,95,220,53,57,162,49,165,241,48,210");
asm(" .byte 244,171,114,236,144,117,110,2,125,128,60,205,139,114,21,176");
asm(" .byte 162,75,150,212,154,60,106,231,40,198,108,224,129,65,60,160");
asm(" .byte 243,242,68,191,108,141,109,252,244,254,71,162,180,18,83,169");
asm(" .byte 76,234,63,60,41,103,158,181,167,132,236,131,44,69,138,41");
asm(" .byte 94,158,189,23,249,70,252,112,251,168,158,157,206,152,197,35");
asm(" .byte 138,91,92,50,130,244,154,167,166,106,68,66,127,229,137,204");
asm(" .byte 6,239,43,226,18,242,148,36,87,36,223,162,205,122,215,68");
asm(" .byte 99,56,209,175,97,92,66,125,207,93,21,49,234,255,44,185");
asm(" .byte 156,106,235,147,8,223,101,243,203,190,172,254,137,198,53,9");
asm(" .byte 77,74,143,97,190,140,124,36,127,141,44,87,214,152,51,230");
asm(" .byte 130,87,74,157,217,117,123,13,141,187,158,243,248,99,186,251");
asm(" .byte 40,189,247,108,52,105,156,164,202,251,45,126,19,93,10,218");
asm(" .byte 194,35,226,122,88,175,225,67,220,0,108,165,117,102,21,246");
asm(" .byte 68,187,101,134,222,127,59,217,18,63,108,73,185,135,238,19");
asm(" .byte 160,247,3,82,66,209,35,78,212,179,32,238,113,35,76,46");
asm(" .byte 7,174,28,51,250,243,98,70,175,47,170,203,82,46,248,181");
asm(" .byte 58,25,117,107,190,253,51,163,190,152,170,249,36,180,111,167");
asm(" .byte 114,212,252,24,109,207,251,158,53,163,109,207,219,79,133,110");
asm(" .byte 230,154,209,2,77,210,156,61,70,191,147,240,235,204,148,218");
asm(" .byte 126,20,237,61,4,156,33,226,185,197,75,132,87,162,95,185");
asm(" .byte 158,181,39,101,137,189,134,184,211,144,7,51,2,217,161,62");
asm(" .byte 152,234,67,117,14,50,75,150,204,30,245,91,70,186,94,150");
asm(" .byte 156,12,49,216,127,140,143,92,110,7,100,76,254,208,210,207");
asm(" .byte 8,226,202,17,39,153,81,26,59,185,60,14,200,16,143,131");
asm(" .byte 237,72,64,150,250,111,39,157,246,27,196,11,186,175,75,30");
asm(" .byte 58,5,252,211,249,116,143,131,196,20,212,155,153,163,117,145");
asm(" .byte 212,124,146,47,140,53,243,101,201,151,127,173,101,243,253,150");
asm(" .byte 205,39,25,185,198,178,249,229,36,139,69,144,23,75,134,156");
asm(" .byte 214,180,128,231,211,233,140,140,202,24,185,29,194,61,94,187");
asm(" .byte 44,176,218,101,177,213,46,104,19,35,130,62,144,198,15,146");
asm(" .byte 31,99,121,180,137,226,135,125,71,155,72,108,13,221,1,226");
asm(" .byte 251,71,229,9,83,126,145,100,114,141,129,58,42,100,163,251");
asm(" .byte 225,238,135,155,108,178,87,90,99,120,125,53,10,217,224,126");
asm(" .byte 184,251,225,38,155,235,240,199,201,174,122,232,236,21,181,35");
asm(" .byte 198,112,15,83,30,246,118,164,54,164,246,235,196,24,250,74");
asm(" .byte 109,247,136,53,246,251,243,218,225,184,173,29,92,130,151,207");
asm(" .byte 90,246,180,220,248,145,213,14,154,113,216,178,165,229,86,127");
asm(" .byte 74,188,253,103,164,165,240,1,178,147,162,205,126,77,182,215");
asm(" .byte 123,42,249,4,65,223,233,228,30,192,3,136,31,89,134,49");
asm(" .byte 98,4,118,170,141,250,56,205,56,160,245,13,207,188,144,186");
asm(" .byte 160,185,251,134,39,105,143,160,158,143,232,172,247,126,125,197");
asm(" .byte 55,227,198,200,55,158,159,40,85,211,123,224,239,12,79,68");
asm(" .byte 254,19,104,175,26,245,133,129,21,38,221,221,120,13,171,55");
asm(" .byte 156,1,151,14,188,155,24,230,89,180,246,233,76,140,197,149");
asm(" .byte 145,190,74,224,18,158,211,247,157,183,198,230,181,211,202,11");
asm(" .byte 227,74,211,53,52,54,46,157,142,238,200,64,218,243,239,117");
asm(" .byte 51,70,15,181,3,173,137,23,42,252,110,74,153,238,166,164");
asm(" .byte 245,108,180,25,83,246,78,115,93,72,189,93,187,139,177,237");
asm(" .byte 140,223,43,74,119,76,74,12,179,7,159,191,188,162,114,222");
asm(" .byte 173,243,111,91,180,184,166,118,201,210,244,157,206,139,234,231");
asm(" .byte 212,220,179,234,75,181,203,22,90,238,149,75,106,23,150,248");
asm(" .byte 74,124,234,252,10,191,255,214,202,249,21,37,62,126,55,179");
asm(" .byte 133,119,247,210,21,203,22,150,204,162,180,179,0,102,137,240");
asm(" .byte 250,186,149,11,75,42,75,84,198,70,243,32,184,180,190,110");
asm(" .byte 197,170,133,60,44,80,179,10,25,115,119,221,146,165,150,211");
asm(" .byte 10,95,117,79,237,146,133,37,126,114,175,90,177,166,6,206");
asm(" .byte 18,191,138,255,170,90,82,142,63,163,52,149,103,209,212,28");
asm(" .byte 10,135,154,194,11,56,216,24,222,209,22,130,100,33,207,166");
asm(" .byte 214,109,225,246,214,150,5,2,110,108,9,61,24,106,161,251");
asm(" .byte 139,241,220,23,218,22,106,15,182,44,216,22,124,176,57,24");
asm(" .byte 14,110,108,14,109,109,37,58,234,235,214,46,33,184,165,45");
asm(" .byte 28,108,108,9,117,40,165,97,101,91,48,172,148,46,29,39");
asm(" .byte 108,145,18,168,91,82,119,207,218,250,21,119,47,83,74,219");
asm(" .byte 148,181,181,1,197,171,148,118,40,62,255,173,101,42,254,249");
asm(" .byte 148,210,210,230,182,214,246,176,34,41,165,247,43,181,119,47");
asm(" .byte 170,71,72,184,85,161,253,37,182,252,2,202,138,187,3,107");
asm(" .byte 235,149,69,53,53,75,2,245,99,203,95,202,230,134,194,77");
asm(" .byte 115,183,108,11,133,155,203,80,149,77,108,235,131,74,86,144");
asm(" .byte 221,223,248,64,27,213,29,244,141,13,84,30,82,58,66,205");
asm(" .byte 202,172,142,133,155,194,109,205,74,233,215,45,184,112,150,242");
asm(" .byte 197,236,220,168,252,175,110,105,105,9,182,180,40,165,203,215");
asm(" .byte 6,20,43,138,177,142,150,80,168,77,241,101,149,79,89,95");
asm(" .byte 158,65,199,142,109,77,99,235,177,72,212,17,124,10,55,181");
asm(" .byte 141,50,198,95,110,113,166,238,158,128,117,143,240,220,230,208");
asm(" .byte 131,115,195,225,29,148,118,226,33,41,26,143,179,157,228,174");
asm(" .byte 126,140,220,145,79,116,47,182,220,177,171,198,191,146,123,145");
asm(" .byte 229,142,254,69,243,185,154,60,255,82,229,218,221,19,191,159");
asm(" .byte 234,36,157,109,0,140,35,48,12,120,240,82,170,83,3,140");
asm(" .byte 68,30,139,196,255,53,213,121,0,254,229,79,243,112,229,71");
asm(" .byte 60,252,228,33,30,30,63,204,195,55,63,155,234,28,0,252");
asm(" .byte 104,16,97,128,251,126,156,234,124,134,224,115,169,206,65,64");
asm(" .byte 245,39,60,29,123,158,135,159,124,129,135,247,36,82,157,71");
asm(" .byte 0,7,135,82,157,9,202,239,152,192,123,145,135,247,190,204");
asm(" .byte 195,103,159,224,225,111,158,224,225,234,127,240,240,221,128,199");
asm(" .byte 1,207,1,158,0,172,125,5,121,3,206,120,53,213,121,10");
asm(" .byte 240,226,217,84,231,25,192,150,215,83,157,103,1,19,191,1");
asm(" .byte 46,224,41,51,213,169,3,190,159,76,117,190,9,152,59,156");
asm(" .byte 234,60,15,248,237,139,156,31,236,3,226,199,15,35,71,62");
asm(" .byte 224,124,209,63,224,225,61,187,49,250,218,157,185,135,62,253");
asm(" .byte 148,9,120,191,184,207,126,112,23,191,203,222,253,6,191,227");
asm(" .byte 158,238,235,255,130,184,147,158,238,119,175,130,127,134,237,238");
asm(" .byte 253,205,81,126,63,126,238,78,126,143,125,73,23,191,31,127");
asm(" .byte 173,196,239,135,119,139,187,240,211,119,221,87,5,37,203,38");
asm(" .byte 159,18,119,231,123,196,189,250,231,187,185,91,239,230,119,155");
asm(" .byte 79,20,119,220,19,188,148,74,181,106,8,79,1,82,90,244");
asm(" .byte 223,173,141,221,236,79,254,213,218,238,33,62,190,235,255,231");
asm(" .byte 161,223,145,96,166,220,234,155,138,111,86,130,97,127,71,171");
asm(" .byte 2,51,234,83,252,170,15,63,117,158,21,119,14,118,26,207");
asm(" .byte 162,57,240,88,247,176,51,137,238,16,119,85,214,47,97,57");
asm(" .byte 206,137,210,53,82,158,179,64,154,38,93,231,40,148,174,151");
asm(" .byte 138,228,18,105,89,77,205,2,229,166,53,173,15,180,55,133");
asm(" .byte 218,119,40,203,110,185,69,89,185,37,28,162,108,97,167,111");
asm(" .byte 43,173,84,111,86,42,202,42,203,208,155,148,117,108,238,8");
asm(" .byte 183,195,106,177,178,45,219,194,161,246,54,86,182,173,53,28");
asm(" .byte 42,91,180,120,69,105,56,120,31,43,219,28,236,216,204,202");
asm(" .byte 154,119,108,235,216,177,149,195,112,59,43,187,111,219,3,101");
asm(" .byte 15,134,218,59,232,46,119,187,103,35,226,218,67,45,132,199");
asm(" .byte 29,109,45,97,202,121,11,254,134,67,219,241,119,19,60,136");
asm(" .byte 106,165,126,136,149,45,170,91,85,134,96,42,158,59,183,52");
asm(" .byte 111,231,248,27,131,237,237,193,29,28,63,237,190,191,169,221");
asm(" .byte 34,33,184,117,75,19,138,109,69,118,60,155,198,142,14,158");
asm(" .byte 1,93,248,190,165,241,129,112,8,254,166,214,173,91,67,219");
asm(" .byte 194,127,138,44,76,22,114,79,242,100,125,215,66,202,124,83");
asm(" .byte 193,254,173,134,124,241,141,5,194,179,190,63,33,241,123,221");
asm(" .byte 237,223,106,160,159,87,124,39,66,22,58,177,25,120,199,109");
asm(" .byte 241,105,125,249,59,81,182,44,116,165,4,14,93,124,107,67");
asm(" .byte 98,153,239,82,204,21,250,34,11,221,202,117,114,157,202,166");
asm(" .byte 239,86,150,209,11,210,137,70,23,255,78,131,115,244,251,32");
asm(" .byte 226,251,26,66,143,200,77,186,164,1,175,218,70,115,26,127");
asm(" .byte 141,248,222,132,44,116,81,119,113,29,180,215,131,252,27,108");
asm(" .byte 120,164,187,231,93,92,167,157,162,126,105,188,144,160,53,71");
asm(" .byte 216,150,193,28,110,103,178,249,215,96,195,179,116,7,142,149");
asm(" .byte 210,88,60,122,190,106,195,163,111,158,184,39,142,253,80,194");
asm(" .byte 68,1,191,102,199,163,179,99,95,206,196,217,203,253,186,77");
asm(" .byte 14,10,129,87,8,188,1,249,114,188,78,27,222,241,95,225");
asm(" .byte 249,123,240,84,186,28,239,97,142,215,214,197,196,247,86,54");
asm(" .byte 242,239,74,228,102,225,61,42,190,37,226,96,233,239,162,100");
asm(" .byte 226,236,120,253,226,219,31,14,97,123,171,174,128,247,180,160");
asm(" .byte 143,240,232,190,245,234,43,224,29,22,60,113,176,244,247,86");
asm(" .byte 248,183,86,92,89,237,251,19,91,126,100,179,171,130,140,61");
asm(" .byte 196,46,111,143,132,13,111,16,120,131,192,43,25,167,220,151");
asm(" .byte 68,190,132,71,119,199,191,9,188,193,107,46,111,183,159,114");
asm(" .byte 156,182,116,56,225,221,50,142,94,190,33,220,170,240,79,109");
asm(" .byte 100,108,78,22,158,100,163,45,253,187,13,120,111,142,243,77");
asm(" .byte 150,255,3,255,205,23,205,88,103,0,0,0");
asm("eod_at2so:");
asm(" pop r25"); /* pull return address from stack */
asm(" pop r24");
asm(" clc"); /* multiply by 2 to get data address */
asm(" rol r24");
asm(" rol r25");
asm(" ret");
}

