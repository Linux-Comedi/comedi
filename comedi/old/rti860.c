/*****************************************************************
  Linux device driver for RTI860 hardware
  
  Copyright (C) 1995 Ralf Haller
            (C) 1995, 1996 Harald Kirsch (kir@iitb.fhg.de)

	    Fraunhofer Institut für 
	    Informations- und Datenverarbeitung (IITB)
	    Fraunhoferstr. 1
	    76131 Karlsruhe

*****************************************************************/
/*
 *  values for ioctl
 */
#define RTI860_SET_PAR	1
#define RTI860_GET_PAR	2
#define RTI860_START	3
#define RTI860_RESET	4

/*
 * flags
 */
#define RTI860_F_PRESENT	(1<<0)
#define RTI860_F_OPEN		(1<<1)
#define RTI860_F_BUSY		(1<<2)
#define RTI860_F_IRQ		(1<<3)
#define RTI860_F_RESTART	(1<<4)

/* The following flag(s) can be changed by the user */
#define RTI860_F_USER		(RTI860_F_RESTART)

typedef enum {
  Channel16    =-16,
  Channel15    =-15,
  Channel14    =-14,
  Channel13    =-13,
  Channel12    =-12,
  Channel11    =-11,
  Channel10    =-10,
  Channel9     =-9,
  Channel8     =-8,
  Channel7     =-7,
  Channel6     =-6,
  Channel5     =-5,
  Channel4     =-4,
  Channel3     =-3,
  Channel2     =-2,
  Channel1     =-1,
  Channel1_4   = 0,
  Channel5_8   = 1,
  Channel9_12  = 2,
  Channel13_16 = 3,
  Channel1_16  = 4,
  Channel1_8   = 5,
  Channel9_16  = 6
} Channel;

typedef enum {
  Free         = 0,
  BereichPlus  = 1,
  BereichMinus = 2,
  PegelPlus    = 3,
  PegelMinus   = 4,
  TTLTrigger   = 5
} Trigger;

/*
 * parameter struct used for ioctl
 */
struct rti860_t
{
  unsigned int flags;		/* flags */
  unsigned int base;		/* base address */
  short channel;		/* channels to scan, see enum above */
  unsigned short trigger;	/* trigger mode, see enum above */
  signed char level;		/* trigger level, a value between -128
				   and 127 */
  int tics;			/* >0: use the 5MHz internal clock with
				   ticks as the divider.
				   <0: use the external clock with ticks
				   as the divider.
				   In both cases the sampling frequency
				   is 1.0/(ticks*base_frequency) */
  int irq;			/* interrupt channel, 0 if none */
  unsigned timeout;		/* timeout in seconds */
  unsigned sleep_time;		/* sleep time in polling mode in jiffies
				 */
};

typedef struct rti860_t Rti860;

#define F2TICS(f)	(5e6/(f))
#define TICS2F(tics)	(5e6/(tics))
/*****************************************************************
  Linux device driver for RTI860 hardware
  
  Copyright (C) 1995 Ralf Haller
            (C) 1995, 1996 Harald Kirsch (kir@iitb.fhg.de)

	    Fraunhofer Institut für 
	    Informations- und Datenverarbeitung (IITB)
	    Fraunhoferstr. 1
	    76131 Karlsruhe

*****************************************************************/


#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/malloc.h>
#include <linux/fcntl.h>
#include <linux/timer.h>
#include <linux/delay.h>
 
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/system.h>
 
#include "rti860.h"
 
/* major device number for rti860 device */
#define RTI860_MAJOR 58
/* IMPORTANT: check MAX_CHRDEV in major.h (MAX_CHRDEV >= RTI860_MAJOR) */
 
/* maximum number of minors */
#define RTI860_NO 24
#define RTI860_MAX 5
 
/* size of internal memory (256k) */
#define	RTI860_MEMSIZE 0x00040000L
#define RTI860_MEMCHUNK RTI860_MEMSIZE / 32
 
/* minimal and maximal 200ns-tics */
#define RTI860_MAX_TICS	655350000
#define RTI860_MIN_TICS	25		/* 200kHz */
 
#define INDEX(a) (minor_index[a])
#define SEC2JIFFY(a)    ((a)*HZ)
#define TICS2USEC(a)	((a)/5)		/* one tic is 200ns */

 

/* delay for hardware im ms */
#define delay(a) __delay(a*loops_per_msec)

/*
 *  hardware registers
 */
#define status_reg          ((rti860[INDEX(minor)].base) + 0x00)
#define control_reg         ((rti860[INDEX(minor)].base) + 0x02)
#define muxdata_reg         ((rti860[INDEX(minor)].base) + 0x04)
#define softcon_reg         ((rti860[INDEX(minor)].base) + 0x06)
#define adcdata_reg         ((rti860[INDEX(minor)].base) + 0x08)
#define adcmem_reg          ((rti860[INDEX(minor)].base) + 0x0A)
#define datamem_reg         ((rti860[INDEX(minor)].base) + 0x0C)
#define hostmem_reg         ((rti860[INDEX(minor)].base) + 0x0E)
#define adchstmem_reg       ((rti860[INDEX(minor)].base) + 0x10)
#define muxmemscan_reg      ((rti860[INDEX(minor)].base) + 0x12)
#define data9513_reg        ((rti860[INDEX(minor)].base) + 0x14)
#define ctrl9513_reg        ((rti860[INDEX(minor)].base) + 0x16)
#define flgsclr_reg         ((rti860[INDEX(minor)].base) + 0x18)
#define trigger_reg         ((rti860[INDEX(minor)].base) + 0x1A)
#define indaddr_reg         ((rti860[INDEX(minor)].base) + 0x1C)

/*
 * NB. we must include the kernel idenfication string to install the module.
 */
#include <linux/version.h>
static char kernel_version[] = UTS_RELEASE;

/* parameters for all possible rti860s, can be changed with ioctl */
static Rti860 rti860[RTI860_MAX] = 
{{0,0L,Channel1_4,Free,0,5000,0,5,10},	/* default */
 {0,0L,Channel1_4,Free,0,5000,0,5,10},
 {0,0L,Channel1_4,Free,0,5000,0,5,10},
 {0,0L,Channel1_4,Free,0,5000,0,5,10},
 {0,0L,Channel1_4,Free,0,5000,0,5,10}};

static void rti860_timer( unsigned long );

/* timer for polling mode */
static struct timer_list timer[RTI860_MAX] =
{{NULL,NULL,0,0,rti860_timer},
 {NULL,NULL,0,1,rti860_timer},
 {NULL,NULL,0,2,rti860_timer},
 {NULL,NULL,0,3,rti860_timer}};

/* index to rti860-struct */
static int minor_index[RTI860_NO]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* number of channels for rti860.channel */
/* static int num_channels[]={4,4,4,4,16,8,8}; */

/* wait queue for rti860 device driver */
static struct wait_queue *rti860_wait_q;

/* irq counts */
static unsigned short irq_count[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* number of delay loops per msec */
static int loops_per_msec;

/*****
  Der folgende Typ beschreibt ein Counter Mode Register des 9513. Die
  Bits sind in dieser merkwürdigen Reihenfolge, weil der 80x86 die
  lo-Bytes bei niedrigen Adressen abspeichert.
*****/
typedef struct s_CounterMode {
  unsigned output:3;
  unsigned increm:1;
  unsigned bcdCount:1;
  unsigned repeat:1;
  unsigned reloadFrom:1;
  unsigned specialGate:1;
  unsigned source:4;
  unsigned sourceEdge:1;
  unsigned gate:3;
} CounterMode;
#define cm2us(x) (* (unsigned short*)&(x) )

#if 0
/****************************************************************************
 * read one counter of the 9513
 ****************************************************************************/
static void read_counter( unsigned int minor, unsigned *c )
{

  int i;

  /***** 9513 in 16-bit mode */
  outw(0xFFEF, ctrl9513_reg);

  /***** Copy to hold register */
  outw(0xFFA0+0x1F, ctrl9513_reg);
  
  /***** Select the first counter */
  outw(0xFF19, ctrl9513_reg); 

  for( i=0 ; i<5 ; i++ ) {

    /***** Read the counter */
    c[i] = inw(data9513_reg);	/* hold register */
}
}
#endif

/****************************************************************************
 * reset the 9513
 ****************************************************************************/
static void reset9513( unsigned int minor )
{
  outw( 0x0000, control_reg );		/* Control-register zurücksetzen */
  delay(1);
  /**** 9513 zweimal zurücksetzen */
  outw( 0xFFFF, ctrl9513_reg ); delay(1);
  outb( 0xFF, ctrl9513_reg );  delay(1);

  /***** 
    Betriebsart setzen, d.h. Master-Mode Register adressieren und dann
    den Wert 0x3100 als zwei einzelne Bytes ausgeben. Der Mode 0x3100
    bedeutet:
    ---Bitpositionen---   ----eingestellte Funktion bei 0x3100----
    1000 0000 0000 0000 : Frequenzscalierung binär (nicht BCD)
    0100 0000 0000 0000 : automatisches Adreßincrement
    0010 0000 0000 0000 : 16-bit Datenbus (ab sofort)
    0001 0000 0000 0000 : FOUT Aus
    0000 1111 0000 0000 : FOUT Divider = Division durch 1
    0000 0000 1111 0000 : FOUT-Quelle ist E1
    0000 0000 0000 1100 : Comparator 1 und 2 aus
    0000 0000 0000 0000 : Keine Funktion als Uhr (time-of-day) 
  *****/
  outb( 0x17, ctrl9513_reg ); delay(1);
  outb( 0x00, data9513_reg ); delay(1);
  outb( 0x31, data9513_reg ); delay(1);
  
  /*****
    Zähler 1 wird initialisiert:
    0x0005 heißt: kein Gating, zähle steigende Flanke, zähle Ausgang
    von Zähler 0 (?), kein special gate, nachladen vom Loadregister,
    einmal zählen, binär zählen, abwärts zählen, active low terminal
    count pulse
  *****/
  outw( 0xFF01, ctrl9513_reg ); delay(1);
  outw( 0x0005, data9513_reg ); delay(1);

  /* Eine A/D-Wandlung auslösen (warum?) */
  outw( 0x1234, softcon_reg ); delay(1);

  /*****
    Zähler 1 erneut initialisieren. (warum?)
    0x0002 wie oben, außer Ausgabe auf 'terminal count toggled'.
    Anschließend wird der Ausgabepin definiert auf 0 gesetzt.
  *****/
  outw( 0xFF01, ctrl9513_reg ); delay(1);    
  outw( 0x0002, data9513_reg ); delay(1);
  outw( 0xFFE1, ctrl9513_reg ); delay(1);

  /*****
    Zähler 3 identisch mit Zähler 1 initialisieren. Ausgabepin auf 1, da
    bei 0 die A/D-Wandlung angehalten wird: DONE-Signal. 
  *****/
  outw( 0xFF03, ctrl9513_reg ); delay(1);
  outw( 0x0002, data9513_reg ); delay(1);
  outw( 0xFFE3, ctrl9513_reg ); delay(1);

  /*****
    Zähler 5 identisch mit Zähler 1 initialisieren.
  *****/
  outw( 0xFF05, ctrl9513_reg ); delay(1);
  outw( 0x0002, data9513_reg ); delay(1);
  outw( 0xFFE5, ctrl9513_reg ); delay(1);

  /* Kontrollregister zurücksetzen */  
  outw( 0x0000, control_reg ); delay(1);
}

/****************************************************************************
 * set the trigger
 ****************************************************************************/
static void init_overflow_counter( unsigned int minor )
/*****
  Zähler 4 und 5 auf dem 9513 sind wohl dazu gedacht, die
  Speicherüberlaufbits anzutreiben (vgl. RTI860-Handbuch S.4-10 und
  S.B-13). Beide werden hier initialisiert, so daß nach der halben
  Speichergröße jeweils ein positiver Puls am pin O5 des 9513
  ausgegeben wird. Zähler 4 lassen wir bis 4 zählen. Damit treibt
  er Zähler 5, den wir jeweils bis 0x8000 zählen lassen, so daß insgesamt
  bis 2^17 = halbe Speichergröße der RTI860 gezählt wird.
*****/
{
  CounterMode mode;

  /***** Zähler 4 und 5 abschalten */
  outw(0xFFC0 + (1<<(5-1)) + (1<<(4-1)), ctrl9513_reg); delay(1);

  /***** Zähler 4 */
  mode.gate = 0;
  mode.sourceEdge = 0;
  mode.source = 4;
  mode.specialGate = 0;
  mode.reloadFrom = 0;
  mode.repeat = 1;
  mode.bcdCount = 0;
  mode.increm = 0;
  mode.output = 1;
  outw( 0xFF04, ctrl9513_reg );		delay(1);
  outw( cm2us(mode), data9513_reg );	delay(1); /* Mode */
  outw( 0x0004, data9513_reg );  	delay(1); /* Loadregister */
  outw( 0x0000, data9513_reg );      	delay(1); /* Holdregister */
  outw( 0xFF48, ctrl9513_reg ); 	delay(1); /* Lade von Loadregister */

  /***** Zähler 5 wird von Zähler 4 betrieben */
  mode.gate = 0;
  mode.sourceEdge = 0;
  mode.source = 0;
  mode.specialGate = 0;
  mode.reloadFrom = 0;
  mode.repeat = 1;
  mode.bcdCount = 0;
  mode.increm = 0;
  mode.output = 1;
  outw( 0xFF05, ctrl9513_reg ); 	delay(1);
  outw( cm2us(mode), data9513_reg ); 	delay(1); /* Mode */
  outw( 0x8000, data9513_reg ); 	delay(1); /* Loadregister */
  outw( 0x0000, data9513_reg );      	delay(1); /* Holdregister */
  outw( 0xFF50, ctrl9513_reg ); 	delay(1); /* Lade von Loadregister */
}

/*****************************************************************/
static void 
set_trigger(unsigned int minor)

{
  static unsigned short masks[] = {
    0x000c, 0x0003, 0x0002, 0x0001, 0x0000, 0x0008
  };
  unsigned int mask;

  mask = masks[rti860[INDEX(minor)].trigger];
  mask |= (0x8000 + rti860[INDEX(minor)].level * 128) & 0xFF00;
  outw(mask, trigger_reg); delay(5);
}

/*****************************************************************/
/*
  Tell the board, which channels to scan.
*/
static void
set_inputmux(unsigned int minor)
{
  unsigned short I;
  unsigned base=8;

  outw( 0x0000, control_reg ); delay(1);

  if( rti860[INDEX(minor)].channel >= 0 ) {
    switch( rti860[INDEX(minor)].channel ) {
    case Channel1_16:
      for (I = 0; I < 16; I++) {
	outw(I, muxmemscan_reg ); delay(1);
	outw(15-I, muxdata_reg); delay(1);
      }
      break;
    case Channel1_8:
      base = 0;
    case Channel9_16:
      for(I=0; I<16; I++) {
	outw(I, muxmemscan_reg); delay(1);
	outw((15-I)%8 + base, muxdata_reg); delay(1);
      }
      break;
    case Channel1_4:
    case Channel5_8:
    case Channel9_12:
    case Channel13_16:
      outw( 0x0000, control_reg );
      for (I = 0; I<16; I++) {
	outw(I, muxmemscan_reg); delay(1);
	outw(((15-I) & 0x03) + (rti860[INDEX(minor)].channel << 2), 
	     muxdata_reg); delay(1);
      }
      break;
    default: 
      printk(KERN_DEBUG "Wrong mode in set_inputmux!\n");
    }
  } else {
    outw( 0x0000, control_reg );
    for(I = 0; I < 16; I++) {
      outw(I, muxmemscan_reg); delay(1);
      outw(-rti860[INDEX(minor)].channel-1, muxdata_reg); delay(1);
    }
  }

  /***** Set start of scan to the last mux-data register */
  outw(0x0F, muxmemscan_reg);	delay(1);
  delay(1);
}
/****************************************************************************
 * initialize counter 1
 ****************************************************************************/
void 
init_clock(unsigned int minor)
/*****
  Zähler 1 des 9513 ist der Teiler entweder f"ur die interne clock, die
  mit 5MHz l"auft, oder aber auch f"ur die externe clock.
*****/
{
  unsigned scale, prescale;
  unsigned mode;
  /*****
    Die folgenden Zeilen initialisieren Counter 1. Counter 1
    erzeugt den Takt für die A/D-Wandlung.
    Der Modus=0x9025 + (Vorteiler+0x000B)<<8
    bedeutet (vgl. S.2-141 im Am9513-Handbuch):
          Gating Control: active hi gate 1      1110 0000 0000 0000
	  Source Edge:    Count on Falling Edge 0001 0000 0000 0000
	  Count Source:   F_Vorteiler           0000 1111 0000 0000
	  Special Gate:   disabled              0000 0000 1000 0000
	  Reload from:    Load Register         0000 0000 0100 0000
	  Repetitively:   TRUE                  0000 0000 0010 0000
	  Count Type:     binary                0000 0000 0001 0000
	  Direction:      count down            0000 0000 0000 1000
	  Output:         Active Lo Pulse       0000 0000 0000 0111
    Der Mode entspricht 'Mode E' S.2-131 im Handbuch.
  *****/

  if( rti860[INDEX(minor)].tics < 0 ) {
    /* select SRC1 as counter input */
    mode = 0x9025 | (1<<8);
    scale = -rti860[INDEX(minor)].tics;
  } else {
    for(prescale=0,scale=rti860[INDEX(minor)].tics;
	scale > 65535;	  
	prescale++,scale/=16);
    /*****
      Select F1...F5, i.e. a prescaler-output, as counter input
      The available prescale values are 1, 16, 256, 4096 and 65536.
      The value of prescale must be 0,1,2,3 or 4.
    ****/
    mode = 0x9025 | ((prescale+0xB)<<8);
  }

  outw( 0xFF01, ctrl9513_reg );	delay(1); /* address counter 1 */
  outw( mode, data9513_reg );	delay(1); /* set mode */
  outw( scale, data9513_reg );	delay(1); /* set load register */
  outw( 0x0000, data9513_reg ); delay(1); /* set hold register */
  outw( 0xFF41, ctrl9513_reg );	delay(1); /* load counter with load
					     register */
}

/****************************************************************************
 * initialize counter 2&3
 ****************************************************************************/
static void init_posttrigger_counter( unsigned int minor )
/*****
  Zähler 2&3 zählt A/D-Wandlungen nach dem Trigger. Sobald 2&3 auf 0
  zählt, hört die RTI860 mit A/D-Wandlungen auf. Deshalb werden 2&3
  in RTI860_Start() bei kontinuierlichem Modus nicht scharfgemacht
*****/
{
  /*****
    Der Modus=0x1221 bedeutet (vgl. S.2-141 im Am9513-Handbuch):
        Gating Control: No Gating             1110 0000 0000 0000
        Source Edge:    Count on Falling Edge 0001 0000 0000 0000
        Count Source:   SRC 2                 0000 1111 0000 0000
        Special Gate:   disabled              0000 0000 1000 0000
        Reload from:    Load Register         0000 0000 0100 0000
        Repetitively:   TRUE                  0000 0000 0010 0000
        Count Type:     binary                0000 0000 0001 0000
        Direction:      count down            0000 0000 0000 1000
        Output:         Active Hi Pulse       0000 0000 0000 0111
    Es stellt sich die Frage, warum das Loadregister gerade auf
    samples-1 gesetzt wird, wenn sowieso kontinuierlich gearbeitet
    wird.
  *****/
  outw( 0xFF02, ctrl9513_reg ); delay(1);  /* Adressieren */
  outw( 0x1221, data9513_reg ); delay(1);  /* Modus setzen */
  outw( 333, data9513_reg ); 	delay(1);  /* Loadregister */
  outw( 0x0000, data9513_reg ); delay(1);  /* Holdregister */
  outw( 0xFF42, ctrl9513_reg ); delay(1); /* Counter:=Loadregister */

  /*****
    Counter 3 zählt zusammen mit Counter 2 die gewandelten Daten. Der
    Output von Counter 3 ist sogar am Ausgabeport der RTI860 zu finden
    --- als DONE-Signal. Außderdem hält die Karte tatsächlich an,
    wenn der Ausgang von Counter 3 auf lo geht.
  *****/
  outw( 0xFF03, ctrl9513_reg );    /* Source: Counter 3 - single count */
  delay(1);                         /* TC toggle */ 
  outw( 0x0002, data9513_reg );    /* Source: Counter 2 */
  delay(1);                         /* single count, TC toggle */
  outw( 0x0003, data9513_reg );    /* Loadregister = 3 */
  delay(1);
  outw( 0x0000, data9513_reg );    /* Holdregister = 0 */
  delay(1);
  outw( 0xFF44, ctrl9513_reg );    /* Load Counter 3 with Loadregister*/
  delay(1);
  outw( 0xFFF3, ctrl9513_reg );    /* Counter 3 steppen erstes mal */
  delay(1);
  outw( 0xFFF3, ctrl9513_reg );    /* zweites mal */
  delay(1);
  outw( 0xFFF3, ctrl9513_reg );    /* drittes mal */
  delay(1);
  outw( 0xFFE3|8, ctrl9513_reg );    /* Ausgang Counter 3 auf high */
  delay(1);

  /*****
    Eine A/D-Wandlung auslösen, damit  Flipflop U82 (S.6,Schaltplan)
    auch sicher durchschaltet. 
    Die Methode hat letztlich doch nicht funktioniert. Deshalb wird
    jetzt im reset9513 der Ausgang von counter 3 bereits auf High, statt
    auf Low gesetzt. Das scheint zu funktionieren.
  *****/
  /*outw( 0x1234, softcon_reg ); delay(1);*/
 }

/*****************************************************************/
/*
  Clear the board memory
*/
static void 
rti860_clearmem( unsigned int minor )
{
  int i;
  /***** Setze ADC Zieladresse auf 0 */
  outw( 0x8100, adchstmem_reg );	delay(1); /* most sign. bits */
  outw( 0x8100, adchstmem_reg );	delay(1);
  outw( 0x1000-20, adcmem_reg );	delay(1); /* least sign. bits */
  for(i=0; i<40; i++) {
    outw( 0, control_reg );
  }

  outw( 0x8100, adchstmem_reg );	delay(1); /* most sign. bits */
  outw( 0x8100, adchstmem_reg );	delay(1);
  outw( 0x0000, adcmem_reg );		delay(1); /* least sign. bits */

  /***** Setze Leseadresse auf 0 */
  outw( 0x0001, adchstmem_reg );	delay(1); /* most sign. bits */
  outw( 0x0001, adchstmem_reg );	delay(1);
  outw( 0x0000, hostmem_reg );		delay(1); /* least sign. bits */
}
/*****************************************************************/
/*
  initialize the rti860
*/
static void 
rti860_init( unsigned int minor )
{

  reset9513(minor); delay(5);
  rti860_clearmem(minor );

  set_inputmux(minor);
  set_trigger(minor);
}

/****************************************************************************
 * start the rti860
 ****************************************************************************/
static void rti860_start( unsigned int minor )
/*****
  Startet die A/D-Wandlung auf der Karte mit den in RTI860_Init
  vorgegebenen Parametern. Die A/D-Daten werden im Speicher der Karte
  abgelegt. Wurde Betriebsart=Fix gewählt, so beendet die Karte
  selbständig die Messung nach der vorgegebenen Anzahl der
  Einzelmessungen. Andernfalls muß die Karte mit RTI860_Stop oder
  RTI_860_Reset angehalten werden.
  Hinweis: Diese Prozedur setzt den Speicher der Karte nicht zurück.
*****/
{
  unsigned short control;

  reset9513(minor);
  set_inputmux(minor);
  set_trigger(minor);

  /***** Steuerwort der RTI860 setzen: */
  control = (1<<1)	/* Memory Mode Enable */
          | (1<<2)     	/* Scan Mode Enable */
          | (1<<5);    	/* Continuous Transfer Mode Enable */

  if( rti860[INDEX(minor)].channel >= 0 )
    control |= (1<<8);	/* Simultaneous Mode Enable */

  if( rti860[INDEX(minor)].irq > 0 ) {
    control |= (1<<14);	/* continuous transfer interrupt */
    control |= (1<<13);	/* error interrupt */
  }
  outw_p(control, control_reg); delay(1);

  init_posttrigger_counter(minor);
  init_clock(minor);
  init_overflow_counter(minor);

  rti860_clearmem( minor );

  /***** 
    clear the overflow flags of the 9513, 
    in particular reset the DONE-bit.
  *****/
  outw_p(0x0000, flgsclr_reg); delay(1);
  outw_p(0x8000, flgsclr_reg); delay(1);

  /***** Zähler 1,4,5 scharfmachen */
  outw_p(0xFF39, ctrl9513_reg); delay(1);

  /***** Wandlung starten */
  outw_p(control|1, control_reg); delay(1);
}
/*****************************************************************/
/*
  Beendet die A/D-Wandlungen auf der Karte. Der Speicher der Karte
  wird *nicht* gelöscht. Die Karte kann ohne Neuinitialisierung wieder
  gestartet werden.
*/
#if 0
static void 
rti860_stop( unsigned int minor )
{
  reset9513( minor );
}
#endif

#ifdef PROFILE
/****************************************************************************
 * get the current readaddress
 ****************************************************************************/
static unsigned rti860_get_readaddress( unsigned int minor )
{
  return
    ((unsigned)(inw(adchstmem_reg) & 0x0003) << 16)
    + inw_p(hostmem_reg);
}

/****************************************************************************
 * get the current writeaddress
 ****************************************************************************/
static unsigned rti860_get_writeaddress( unsigned int minor)
{
  unsigned short hiBits, hiTmp;
  unsigned writeAdr;
  /*****
    Zu der folgenden Konstruktion siehe den Kommentar in Fuellstand.
  *****/
  do {
    hiBits = inw(adchstmem_reg) & 0x0300;
    writeAdr = inw(adcmem_reg);
    hiTmp = inw(adchstmem_reg) & 0x0300;
  } while( hiBits!=hiTmp );
  writeAdr +=  ( (unsigned)hiBits << 8 );

  return writeAdr;
}
#endif
/*****************************************************************/
/*
 get the current amount of data in the memory
*/
static unsigned 
rti860_get_filling( unsigned int minor )
{
  unsigned writeAdr, readAdr;
  unsigned short hiBits, hiTmp;

  /***** 
    Das Problem beim Auslesen der Schreibadresse ist, daß zwei
    Speicherzellen aus der Karte gelesen werden müssen, und es ist
    nicht ausgeschlossen, daß die Karte die Schreibadresse zwischen
    den beiden Zugriffen erhöht. Wenn diese Erhöhung über die
    0xFFFF-Marke hinausgeht, passen die beiden gelesenen Werte nicht
    zueinander, egal in welcher Reihenfolge man sie einliest. Deshalb
    die folgende Konstruktion:
  *****/
  do {
    hiBits = inw(adchstmem_reg) & 0x0300;
    writeAdr = inw(adcmem_reg);
    hiTmp = inw(adchstmem_reg) & 0x0300;
  } while( hiBits!=hiTmp );
  writeAdr +=  ( (unsigned)hiBits << 8 );

  hiBits = inw(adchstmem_reg) & 0x0003;
  readAdr = inw(hostmem_reg) + ( (unsigned)hiBits << 16 );
  
  if( writeAdr>=readAdr ){
    return writeAdr - readAdr;
  } else {
    return writeAdr + RTI860_MEMSIZE - readAdr;
  }
}

/****************************************************************************
 * interrupt service routine
 ****************************************************************************/
static void rti860_interrupt( int irq, struct pt_regs *regs )
{
  /*printk("irq%d\n",irq);*/
  /* wake us up */

  wake_up(&rti860_wait_q);
}

/****************************************************************************
 * timer service routine
 ****************************************************************************/
static void rti860_timer( unsigned long data )
{
  /*printk("timer%ld\n",data);*/
  /* wake us up */
  wake_up(&rti860_wait_q);
}

/****************************************************************************
 * allocate the interrupt
 ****************************************************************************/
static void rti860_alloc_irq( int minor )
{
  int ret;

  /* irq already allocated from driver ? */
  if( irq_count[rti860[INDEX(minor)].irq] == 0 ) {
      
    /* get the interrupt */
    ret = request_irq(rti860[INDEX(minor)].irq, 
		      rti860_interrupt, 
		      0/*SA_INTERRUPT*/, 
		      "rti860",
		      NULL);
    if( ret ) {
      /* no interrupt */
      printk(KERN_WARNING 
	     "cannot allocate irq%d forrti860 device, "
	     "switching to polling mode.\n",
	     rti860[INDEX(minor)].irq);
      rti860[INDEX(minor)].flags &= ~RTI860_F_IRQ;
      return;
    }
    else {
      /* got interrupt */
      printk(KERN_INFO "allocated irq%d for rti860 device\n",
	     rti860[INDEX(minor)].irq);
    }
  }

  /* increase irq count */
  irq_count[rti860[INDEX(minor)].irq]++;

  /* set flags */
  rti860[INDEX(minor)].flags |= RTI860_F_IRQ;

}

/****************************************************************************
 * free the interrupt
 ****************************************************************************/
static void rti860_free_irq(int minor)
{
  /* interrupt to free ? */
  if( rti860[INDEX(minor)].flags & RTI860_F_IRQ ) {

    /* decrease irq_count */
    if( irq_count[rti860[INDEX(minor)].irq] == 0 ) {
      printk("rti860: irq_count less than zero! setting back...\n");
      irq_count[rti860[INDEX(minor)].irq] = 0;
    }

    if( --irq_count[rti860[INDEX(minor)].irq] == 0 ) {
      /* free interrupt and reset flags */
      free_irq( rti860[INDEX(minor)].irq, NULL );
      printk(KERN_INFO "freed irq%d of rti860 device\n",
	     rti860[INDEX(minor)].irq);
    }

    /* reset flags */
    rti860[INDEX(minor)].flags &= ~RTI860_F_IRQ;

  }
}

/****************************************************************************
 * read data from hardware
 ****************************************************************************/
static int rti860_read(struct inode *inode, struct file *file, char *buf,
		       int count)
{
  int ret, pre, post;
  int index=0;
  unsigned num_samples = 0;
  unsigned int minor = MINOR(inode->i_rdev);
#if 0
  static unsigned long jiffies1,jiffies2;
  unsigned dummy;
#endif
#if 0
  unsigned c[5],w,i;
  static unsigned long mittel,stdabw,min,max,n;
  static unsigned a[100];
#endif

#if 0
  jiffies1=jiffies;
#endif

  /***** only even byte count allowed */
  count &= 0xFFFFFFFE;

  /* check user memory */
  ret = verify_area( VERIFY_WRITE, buf, count );
  if( ret ) return ret;

  /***** start the driver and the device, if not yet busy. */
  if( (rti860[INDEX(minor)].flags & RTI860_F_BUSY) == 0 ) {

    if( rti860[INDEX(minor)].irq > 0 ) {
      /* we want an interupt, so get it */
      rti860_alloc_irq(minor);
    }
    /* reset and start the hardware */
    rti860_start(minor);

    rti860[INDEX(minor)].flags |= RTI860_F_BUSY;
  }

  num_samples = rti860_get_filling(minor);

  /***** read all the data */
  while( index < count ) {

    /***** Check for overflow and restart device in restart-mode */
    if( inw(status_reg) & (1<<6) ) {
      if( rti860[INDEX(minor)].flags & RTI860_F_RESTART ) {
	rti860_start(minor);
      } else {
	/* return data already read or error if nothing read */
	return index ? index : -EIO;
      }
    }

    /***** 
      We are about to enter a wait-loop below which includes some
      sleeping. In polling mode, or if hardware fifo overflow happens at
      the wrong time, there will be no more samples arriving anymore for
      a long time. If this time is longer than the timeout, the process
      will get an ETIME error, however only, if an ioctl() specified a
      certain timeout.
    *****/
    if( rti860[INDEX(minor)].timeout ) {
      current->timeout = 
	jiffies + SEC2JIFFY(rti860[INDEX(minor)].timeout);
    }

    /***** This is the wait-loop */
    while( !(num_samples=rti860_get_filling(minor)) ) {

      /* non blocked ? */
      if( file->f_flags & O_NONBLOCK ) return index;

      /***** fire a timer, if in polling mode */
      if((rti860[INDEX(minor)].flags & RTI860_F_IRQ) == 0 ){

	/* check if timer already set */
	if( timer[INDEX(minor)].next == NULL 
	    && timer[INDEX(minor)].prev == NULL ) {

	  timer[INDEX(minor)].expires = 
	    jiffies + rti860[INDEX(minor)].sleep_time;
	  add_timer( &(timer[INDEX(minor)]) );
	}
      }

      /***** fall asleep until interrupt or timer arrives */
      interruptible_sleep_on(&rti860_wait_q);

      if (current->signal & ~current->blocked) {
	return index;
      }
    
      /***** timeout occured ? */
      if( current->timeout==0 ) return (-ETIME);
    }
    
    if( num_samples > (count-index)/sizeof(short) )
      num_samples = (count-index)/sizeof(short);

    pre = inw(adchstmem_reg)&0x2;

    for( ; num_samples>0; num_samples--) {
      /* read data and copy to user space */
      /* division by 16 because the 4 lower bits are not used */ 
      put_fs_word( (short)inw_p(datamem_reg)/16, &(buf[index]));
      index = index + sizeof(short);

      post = inw(adchstmem_reg)&0x2;
      if( pre != post ) {
	outw( 0x8000, flgsclr_reg );
      }
    }
  }

#if 0
  printk("%ld %ld %d\n",
	 jiffies-jiffies1,
	 jiffies-jiffies2,
	 dummy);
  jiffies2=jiffies;
#endif

  return index;
}

/****************************************************************************
 * test if data can be read
 ****************************************************************************/
static int rti860_select(struct inode *inode, struct file *file, 
			 int sel_type, select_table *wait)
{
  unsigned int minor = MINOR(inode->i_rdev);

  /* ready for read ? */
  if( sel_type == SEL_IN ) {

    /* data available ? */
    if( rti860_get_filling(minor) == 0 ) {

      /* insert in wait queue */
      select_wait(&rti860_wait_q,wait);

      /* nothing to read */
      return 0;
    }

    /* ready to read */
    return 1;
  }

  /* always ready for write and exceptions */
  return 1;
}
/*****************************************************************/
static char
checkParams(Rti860 *p)
{
  /***** 
    There is an enumeration type with limited range for channel.
  *****/ 
  if( p->channel<-16 || p->channel>6 ) return 0;

  /*****
    There is an enumeration type with limited range for trigger type.
  *****/    
  if( p->trigger>5 ) return 0;

  /*****
    Ticks must be positive or 16bit negative but not 0,-1,-2;
  *****/
  if( p->tics<=-65536 ) return 0;
  if( p->tics<=0 && p->tics>=-1 ) return 0;

  /***** 
    We allow all irqs which can be jumpered according table 2.4 (page
    2-8) of the rti860 manual.
  *****/
  if( p->irq!=0 && p->irq!=3 && p->irq!=5 && p->irq!=7
      && p->irq!=10 && p->irq!=11 && p->irq!=12 && p->irq != 15)
    return 0;

  if( p->sleep_time<1 ) return 0;
  return 1;
}
/*****************************************************************/
static int 
rti860_ioctl(struct inode *inode, struct file *file,
	     unsigned int cmd, unsigned long arg)
{
  unsigned int minor = MINOR(inode->i_rdev);
  Rti860 tmprti860;
  Rti860 *ptr = (Rti860 *)arg;
  int error;

  switch( cmd ) {
  case RTI860_SET_PAR:
    /* device busy ? */
    if( rti860[INDEX(minor)].flags & RTI860_F_BUSY ) return (-EBUSY);

    /* check memory */
    error = verify_area( VERIFY_READ, ptr, sizeof(Rti860) );
    if (error) return (error);

    /* copy data */
    memcpy_fromfs( (void *)&tmprti860, ptr, sizeof(Rti860) );

    /***** 
      We probably must correct the tics. However we do this only for
      the internal clock. Doing it for the external clock is rather
      useless, because the divider tends to so small then that division
      by 2 or 4 will invalidate it.
      We do this before checking parameters because the parameters must
      come out correct afterwards.
    *****/
    if( tmprti860.channel >= 0 && tmprti860.tics>0) {
      switch( tmprti860.channel ) {
      case Channel1_16:
	tmprti860.tics /= 4;
	break;
      case Channel1_8:
      case Channel9_16:      
	tmprti860.tics /= 2;
	break;
      case Channel1_4:
      case Channel5_8:
      case Channel9_12:
      case Channel13_16:
	break;
      default:
	return (-EINVAL);
      }
    }

    /* check validity of parameters */
    if( !checkParams(&tmprti860) ) return -EINVAL;
    
    /* set data */
    rti860[INDEX(minor)].flags = 
      (rti860[INDEX(minor)].flags & ~RTI860_F_USER) | 
      (tmprti860.flags & RTI860_F_USER);
    rti860[INDEX(minor)].channel	= tmprti860.channel;
    rti860[INDEX(minor)].trigger        = tmprti860.trigger;
    rti860[INDEX(minor)].level          = tmprti860.level;
    rti860[INDEX(minor)].tics           = tmprti860.tics;
    rti860[INDEX(minor)].irq		= tmprti860.irq;
    rti860[INDEX(minor)].timeout	= tmprti860.timeout;
    rti860[INDEX(minor)].sleep_time = tmprti860.sleep_time;      
    return 0;  



  case RTI860_GET_PAR: /* get actual parameters */
    /* check memory */

    error = verify_area(VERIFY_WRITE, ptr, sizeof(Rti860));
    if( error ) return error;

    tmprti860 = rti860[INDEX(minor)];

    /***** need to correct the outgoing ticks? */
    if( tmprti860.channel >= 0 && tmprti860.tics>0) {
      switch( tmprti860.channel ) {
      case Channel1_16:
	tmprti860.tics *= 4;
	break;
      case Channel1_8:
      case Channel9_16:      
	tmprti860.tics *= 2;
	break;
      default:
	break;
      }
    }
    /* copy data */
    memcpy_tofs(ptr, (void *)(&tmprti860), sizeof(Rti860));
    
    return 0;

  case RTI860_START:
    /* device busy ? */
    if( (rti860[INDEX(minor)].flags & RTI860_F_BUSY) == 0 ) {
      if( rti860[INDEX(minor)].irq > 0 ) {
	/* we need an interrupt, get it */
	rti860_alloc_irq(minor);
      }
      /* set device busy */
      rti860[INDEX(minor)].flags |= RTI860_F_BUSY;
    }

    /* reset and start the hardware */
    rti860_start(minor);

    /* oK */
    return 0;
    
  case RTI860_RESET:
    /* hardware reset */
    rti860_init(minor);
    
    /* set device not busy */
    rti860[INDEX(minor)].flags &= ~RTI860_F_BUSY;

    /* oK */
    return 0;

  default:
    return (-EINVAL);
  }
}

/****************************************************************************
 * open the device
 ****************************************************************************/
static int rti860_open(struct inode *inode, struct file *file)
{
  unsigned int minor = MINOR(inode->i_rdev);

  /* check minor device number */
  if( minor >= RTI860_NO )
    return (-ENODEV);

  /* check if hardware present */
  if( !(rti860[INDEX(minor)].flags & RTI860_F_PRESENT) )
    return (-ENODEV);

  /* open for write ? */
  if( (file->f_flags & O_ACCMODE) != O_RDONLY )
    return (-EROFS);

  /* already opened ? */
  if( rti860[INDEX(minor)].flags & RTI860_F_OPEN )
    return (-EBUSY);

  /* set device open */
  rti860[INDEX(minor)].flags |= RTI860_F_OPEN;

  /* oK */
  return 0;
}

/****************************************************************************
 * close the device
 ****************************************************************************/
static void rti860_release(struct inode *inode, struct file *file)
{
  unsigned int minor = MINOR(inode->i_rdev);

  /* hardware reset */
  rti860_init(minor);

  /* clear irq */
  rti860_free_irq(minor);

  /* set device not busy */
  rti860[INDEX(minor)].flags &= ~RTI860_F_BUSY;

  /* set device closed */
  rti860[INDEX(minor)].flags &= ~RTI860_F_OPEN;
}

static struct file_operations rti860_fops = {
        NULL,		/* rti860_lseek */
        rti860_read,
        NULL,		/* rti860_write */
        NULL,		/* rti860_readdir */
        rti860_select,
        rti860_ioctl,
        NULL,		/* rti860_mmap */
        rti860_open,
        rti860_release
};

/****************************************************************************
 * initialize the device
 ****************************************************************************/
int init_module( void )
{
  unsigned long address;
  unsigned int minor, num=1;

  if( register_chrdev(RTI860_MAJOR, "rti860", &rti860_fops) )
  {
    printk(KERN_ERR
	   "unable to get major %d for rti860 device\n", 
	   RTI860_MAJOR);
    return -EIO;
  }

  /* find the rti860 */
  for( minor=0,address=0x100 ; address<=0x3E0 ; address+=0x20,minor++ ) {

    /* check if rti860 is present at address */
    if( (inw(address) & 0xF000) == 0x2000 ) {

      printk(KERN_INFO "rti860 found at 0x%lX\n", address );

      INDEX(minor) = num++;

      /*
       * Set present flag and base address
       */
      rti860[INDEX(minor)].flags |= RTI860_F_PRESENT;
      rti860[INDEX(minor)].base = address;

      /* maximal number of rti860's reached */
      if( num >= RTI860_MAX ) break;
    }
  }

  /* calculate delay loops */
  loops_per_msec = loops_per_sec/1000;

  printk(KERN_INFO "rti860 device driver installed.\n");

  return 0;  

}

/****************************************************************************
 * kill the device
 ****************************************************************************/
void cleanup_module(void) 
{
  int i;

  unregister_chrdev(RTI860_MAJOR, "rti860");
  for(i=0; i<RTI860_NO; i++) {
    if( INDEX(i)<RTI860_MAX ) {
      rti860_free_irq(i);
      if( timer[INDEX(i)].prev || timer[INDEX(i)].next) {
	del_timer( &(timer[INDEX(i)]) );
      }
    }
  }
  printk(KERN_INFO "rti860: module removed.\n");
}
