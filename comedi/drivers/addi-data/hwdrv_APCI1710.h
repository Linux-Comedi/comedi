




#define COMEDI_SUBD_TTLIO				11  /* Digital Input Output But TTL */
#define COMEDI_SUBD_PWM					12  /* Pulse width Measurement */
#define COMEDI_SUBD_SSI					13  /* Synchronous serial interface */
#define COMEDI_SUBD_TOR					14  /* Tor counter */
#define COMEDI_SUBD_CHRONO              15  /* Chrono meter*/
#define COMEDI_SUBD_PULSEENCODER        16  /* Pulse Encoder INP CPT*/
#define COMEDI_SUBD_INCREMENTALCOUNTER  17  /* Incremental Counter */


   #define INT           int
   #define UINT          unsigned int
   #define BYTE          unsigned char
   #define CHAR          char
   #define LONG          long
   #define ULONG         unsigned long
   #define VOID          void
   #define PINT          int *
   #define PUINT         unsigned int *
   #define PBYTE         unsigned char *
   #define PCHAR         char *
   #define PLONG         long *
   #define PULONG        unsigned long *
   #define DWORD         unsigned long
   #define WORD          unsigned short

   #define      APCI1710_BOARD_NAME			  "apci1710"
   #define      APCI1710_BOARD_VENDOR_ID	  0x10E8
   #define      APCI1710_BOARD_DEVICE_ID	  0x818F
   #define      APCI1710_ADDRESS_RANGE        256
   #define      APCI1710_CONFIG_ADDRESS_RANGE 8
   #define      APCI1710_INCREMENTAL_COUNTER  0x53430000UL
   #define      APCI1710_SSI_COUNTER          0x53490000UL
   #define      APCI1710_TTL_IO               0x544C0000UL
   #define      APCI1710_DIGITAL_IO           0x44490000UL
   #define      APCI1710_82X54_TIMER          0x49430000UL
   #define      APCI1710_CHRONOMETER          0x43480000UL
   #define      APCI1710_PULSE_ENCODER	      0x495A0000UL
   #define      APCI1710_TOR_COUNTER	      0x544F0000UL
   #define      APCI1710_PWM		      0x50570000UL
   #define      APCI1710_ETM		      0x45540000UL
   #define      APCI1710_CDA		      0x43440000UL
   #define      APCI1710_DISABLE              0
   #define      APCI1710_ENABLE               1
   #define      APCI1710_SYNCHRONOUS_MODE     1
   #define      APCI1710_ASYNCHRONOUS_MODE    0


//MODULE INFO STRUCTURE

comedi_lrange range_apci1710_ttl={ 4, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2),
		BIP_RANGE(1)		
	}
};


comedi_lrange range_apci1710_ssi={ 4, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2),
		BIP_RANGE(1)		
	}
};

comedi_lrange range_apci1710_inccpt={ 4, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2),
		BIP_RANGE(1)		
	}
};
