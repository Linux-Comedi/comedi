// Helper types to take care of the fact that the DSP card memory
//   is 16 bits, but aligned on a 32 bit PCI boundary
typedef union {
  u8 as_u8;
  u16 as_u16;
  u32 as_u32;
} u_val_t;

typedef union {
  s8 as_s8;
  s16 as_s16;
  s32 as_s32;
} s_val_t;

static inline u16 get_u16(volatile u_val_t val) 
{
  u_val_t tmp;
  tmp.as_u32 = le32_to_cpu(val.as_u32);
  return tmp.as_u16;
} 

static inline void set_u16(volatile u_val_t *p, u16 val) 
{
  u_val_t tmp;
  tmp.as_u16 = val;
  p->as_u32 = cpu_to_le32(tmp.as_u32);
} 

static inline s16 get_s16(volatile s_val_t val) 
{
  s_val_t tmp;
  tmp.as_s32 = le32_to_cpu(val.as_s32);
  return tmp.as_s16;
} 

static inline void set_s16(volatile s_val_t *p, s16 val) 
{
  s_val_t tmp;
  tmp.as_s16 = val;
  p->as_s32 = cpu_to_le32(tmp.as_s32);
} 

// The raw data is stored in a format which facilitates rapid
// processing by the JR3 DSP chip. The raw_channel structure shows the
// format for a single channel of data. Each channel takes four,
// two-byte words. 
//
// Raw_time is an unsigned integer which shows the value of the JR3
// DSP's internal clock at the time the sample was received. The clock
// runs at 1/10 the JR3 DSP cycle time. JR3's slowest DSP runs at 10
// Mhz. At 10 Mhz raw_time would therefore clock at 1 Mhz.
// 
// Raw_data is the raw data received directly from the sensor. The
// sensor data stream is capable of representing 16 different
// channels. Channel 0 shows the excitation voltage at the sensor. It
// is used to regulate the voltage over various cable lengths.
// Channels 1-6 contain the coupled force data Fx through Mz. Channel
// 7 contains the sensor's calibration data. The use of channels 8-15
// varies with different sensors.
typedef struct raw_channel
{
  u_val_t raw_time;
  s_val_t raw_data;
  s_val_t reserved[2];
} raw_channel_t;

// The force_array structure shows the layout for the decoupled and
// filtered force data.
typedef struct force_array
{
  s_val_t fx;
  s_val_t fy;
  s_val_t fz;
  s_val_t mx;
  s_val_t my;
  s_val_t mz;
  s_val_t v1;
  s_val_t v2;
} force_array_t;

// The six_axis_array structure shows the layout for the offsets and
// the full scales.
typedef struct six_axis_array
{
  s_val_t fx;
  s_val_t fy;
  s_val_t fz;
  s_val_t mx;
  s_val_t my;
  s_val_t mz;
} six_axis_array_t;

// VECT_BITS
// Indicates which axis are to be used when computing the vectors. A vector
// is composed by 3 components and its "magnitude" is placed in V1 and V2.
// V1 defaults to a force vector and V2 defaults to a moment vector.
// Setting changeV1 or changeV2 will change that vector to be the opposite of
// its default.

// *** Check this norby ***
// This is badly defined at JR3 Manual. Correct definition follows:
typedef struct vect_bits
{
 unsigned fx : 1;
 unsigned fy : 1;
 unsigned fz : 1;
 unsigned mx : 1;
 unsigned my : 1;
 unsigned mz : 1;
 unsigned changeV1 : 1;
 unsigned changeV2 : 1;
 unsigned reserved : 8;
} vect_bits;

// WARNINGS
// Bit pattern for the warning word: xx_near_sat means that a near saturation
// has been reached or exceeded.
typedef struct warning_bits
{
 unsigned fx_near_sat : 1;
 unsigned fy_near_sat : 1;
 unsigned fz_near_sat : 1;
 unsigned mx_near_sat : 1;
 unsigned my_near_sat : 1;
 unsigned mz_near_sat : 1;
 unsigned reserved : 10;
} warning_bits;

// ERROR_BITS
// Bit pattern for the error word:
// 1. xx_sat means that a near saturation has been reached or exceeded.
// 2. memory_error indicates RAM memory error during power up.
// 3. sensor_change indicates that the sensor plugged in (different from the
//    original one) has passed CRC check. The user must reset this bit.
// 4. system_busyindicates system busy: transf. change, new full scale or new
//    sennsor plugged in.
// 5. cal_crc_bad means that it was a problem transmiting the calibration data
//    stored inside the sensor. If this bit does not come to zero 2s after the
//    sensor has been plugged in, there is a problem with the sensor's calibra-
//    tion data.
// 6. watch_dog2 indicates that sensor data and clock are being received.
// 7. watch_dog indicates that data line seems to be acting correctly.
// If either watch dog barks, the sensor data is not beig receive correctly.
#if 0
typedef struct error_bits
{
 unsigned fx_sat : 1;
 unsigned fy_sat : 1;
 unsigned fz_sat : 1;
 unsigned mx_sat : 1;
 unsigned my_sat : 1;
 unsigned mz_sat : 1;
 unsigned reserved : 4;
 unsigned memory_error : 1;
 unsigned sensor_change : 1;
 unsigned system_busy : 1;
 unsigned cal_crc_bad : 1;
 unsigned watch_dog2 : 1;
 unsigned watch_dog : 1;
} error_bits;
#endif

// THRESH_STRUCT
// This structure shows the layout for a single threshold packet inside of a
// load envelope. Each load envelope can contain several threshold structures.
// 1. data_address contains the address of the data for that threshold. This
//    includes filtered, unfiltered, raw, rate, counters, error and warning data
// 2. threshold is the is the value at which, if data is above or below, the
//    bits will be set ... (pag.24).
// 3. bit_pattern contains the bits that will be set if the threshold value is
//    met or exceeded.
typedef struct thresh_struct
{
 s32 data_address;
 s32 threshold;
 s32 bit_pattern;
} thresh_struct;

// LE_STRUCT
// Layout of a load enveloped packet. Four thresholds are showed ... for more
// see manual (pag.25)
// 1. latch_bits is a bit pattern that show which bits the user wants to latch.
//    The latched bits will not be reset once the threshold which set them is
//    no longer true. In that case the user must reset them using the reset_bit
//    command.
// 2. number_of_xx_thresholds specify how many GE/LE threshold there are.
typedef struct {
 s32 latch_bits;
 s32 number_of_ge_thresholds;
 s32 number_of_le_thresholds;
 struct thresh_struct thresholds[4];
 s32 reserved;
} le_struct_t;

// LINK_TYPES
// Link types is an enumerated value showing the different possible transform
// link types.
// 0 - end transform packet
// 1 - translate along X axis (TX)
// 2 - translate along Y axis (TY)
// 3 - translate along Z axis (TZ)
// 4 - rotate about X axis (RX)
// 5 - rotate about Y axis (RY)
// 6 - rotate about Z axis (RZ)
// 7 - negate all axes (NEG)
typedef enum link_types
{
 end_x_form,
 tx,
 ty,
 tz,
 rx,
 ry,
 rz,
 neg
} link_types;

// TRANSFORM
// Structure used to describe a transform.
typedef struct {
  struct {
    u_val_t link_type;
    s_val_t link_amount;
  } link[8];
} intern_transform_t;

// JR3 force/torque sensor data definition. For more information see sensor and
// hardware manuals.

typedef struct force_sensor_data
{
  // Raw_channels is the area used to store the raw data coming from
  // the sensor.

  raw_channel_t raw_channels[16]; 			/* offset 0x0000 */ 

  // Copyright is a null terminated ASCII string containing the JR3
  // copyright notice.

  u_val_t copyright[0x0018];				/* offset 0x0040 */
  s_val_t reserved1[0x0008];				/* offset 0x0058 */

  // Shunts contains the sensor shunt readings. Some JR3 sensors have
  //  the ability to have their gains adjusted. This allows the
  //  hardware full scales to be adjusted to potentially allow
  //  better resolution or dynamic range. For sensors that have
  //  this ability, the gain of each sensor channel is measured at
  //  the time of calibration using a shunt resistor. The shunt
  //  resistor is placed across one arm of the resistor bridge, and
  //  the resulting change in the output of that channel is
  //  measured. This measurement is called the shunt reading, and
  //  is recorded here. If the user has changed the gain of the //
  // sensor, and made new shunt measurements, those shunt
  //  measurements can be placed here. The JR3 DSP will then scale
  //  the calibration matrix such so that the gains are again
  //  proper for the indicated shunt readings. If shunts is 0, then
  //  the sensor cannot have its gain changed. For details on
  //  changing the sensor gain, and making shunts readings, please
  //  see the sensor manual. To make these values take effect the
  //  user must call either command (5) use transform # (pg. 33) or
  //  command (10) set new full scales (pg. 38).

  six_axis_array_t shunts; 				/* offset 0x0060 */
  s32 reserved2[2]; 					/* offset 0x0066 */

  // Default_FS contains the full scale that is used if the user does
  // not set a full scale.

  six_axis_array_t default_FS;  			/* offset 0x0068 */
  s_val_t reserved3;  					/* offset 0x006e */

  // Load_envelope_num is the load envelope number that is currently
  // in use. This value is set by the user after one of the load
  // envelopes has been initialized.

  s_val_t load_envelope_num;   			/* offset 0x006f */
  
  // Min_full_scale is the recommend minimum full scale.
  //
  // These values in conjunction with max_full_scale (pg. 9) helps
  // determine the appropriate value for setting the full scales. The
  // software allows the user to set the sensor full scale to an
  // arbitrary value. But setting the full scales has some hazards. If
  // the full scale is set too low, the data will saturate
  // prematurely, and dynamic range will be lost. If the full scale is
  // set too high, then resolution is lost as the data is shifted to
  // the right and the least significant bits are lost. Therefore the
  // maximum full scale is the maximum value at which no resolution is
  // lost, and the minimum full scale is the value at which the data
  // will not saturate prematurely. These values are calculated
  // whenever a new coordinate transformation is calculated. It is
  // possible for the recommended maximum to be less than the
  // recommended minimum. This comes about primarily when using
  // coordinate translations. If this is the case, it means that any
  // full scale selection will be a compromise between dynamic range
  // and resolution. It is usually recommended to compromise in favor
  // of resolution which means that the recommend maximum full scale
  // should be chosen.
  //
  // WARNING: Be sure that the full scale is no less than 0.4% of the
  // recommended minimum full scale. Full scales below this value will
  // cause erroneous results.

  six_axis_array_t min_full_scale; 			/* offset 0x0070 */
  s_val_t reserved4; 					/* offset 0x0076 */

  // Transform_num is the transform number that is currently in use.
  // This value is set by the JR3 DSP after the user has used command
  // (5) use transform # (pg. 33).

  s_val_t transform_num; 				/* offset 0x0077 */

  // Max_full_scale is the recommended maximum full scale. See
  // min_full_scale (pg. 9) for more details.

  six_axis_array_t max_full_scale; 			/* offset 0x0078 */
  s_val_t reserved5;					/* offset 0x007e */

  // Peak_address is the address of the data which will be monitored
  // by the peak routine. This value is set by the user. The peak
  // routine will monitor any 8 contiguous addresses for peak values.
  // (ex. to watch filter3 data for peaks, set this value to 0x00a8).

  s_val_t peak_address; 				/* offset 0x007f */

  // Full_scale is the sensor full scales which are currently in use.
  // Decoupled and filtered data is scaled so that +/- 16384 is equal
  // to the full scales. The engineering units used are indicated by
  // the units value discussed on page 16. The full scales for Fx, Fy,
  // Fz, Mx, My and Mz can be written by the user prior to calling
  // command (10) set new full scales (pg. 38). The full scales for V1
  // and V2 are set whenever the full scales are changed or when the
  // axes used to calculate the vectors are changed. The full scale of
  // V1 and V2 will always be equal to the largest full scale of the
  // axes used for each vector respectively.

  force_array_t full_scale;  				/* offset 0x0080 */

  // Offsets contains the sensor offsets. These values are subtracted from 
  // the sensor data to obtain the decoupled data. The offsets are set a 
  // few seconds (< 10) after the calibration data has been received.
  // They are set so that the output data will be zero. These values
  // can be written as well as read. The JR3 DSP will use the values
  // written here within 2 ms of being written. To set future
  // decoupled data to zero, add these values to the current decoupled
  // data values and place the sum here. The JR3 DSP will change these
  // values when a new transform is applied. So if the offsets are
  // such that FX is 5 and all other values are zero, after rotating
  // about Z by 90 degrees, FY would be 5 and all others would be zero.

  six_axis_array_t offsets; 				/* offset 0x0088 */  
  
  // Offset_num is the number of the offset currently in use. This
  // value is set by the JR3 DSP after the user has executed the use
  // offset # command (pg. 34). It can vary between 0 and 15.

  s_val_t offset_num; 					/* offset 0x008e */

  // Vect_axes is a bit map showing which of the axes are being used
  // in the vector calculations. This value is set by the JR3 DSP
  // after the user has executed the set vector axes command (pg. 37).

  u_val_t vect_axes; 					/* offset 0x008f */

  // Filter0 is the decoupled, unfiltered data from the JR3 sensor.
  // This data has had the offsets removed.
  //
  // These force_arrays hold the filtered data. The decoupled data is
  // passed through cascaded low pass filters. Each succeeding filter
  // has a cutoff frequency of 1/4 of the preceding filter. The cutoff
  // frequency of filter1 is 1/16 of the sample rate from the sensor.
  // For a typical sensor with a sample rate of 8 kHz, the cutoff
  // frequency of filter1 would be 500 Hz. The following filters would
  // cutoff at 125 Hz, 31.25 Hz, 7.813 Hz, 1.953 Hz and 0.4883 Hz.

  struct force_array filter[7];				/* offset 0x0090, 
							   offset 0x0098, 
							   offset 0x00a0, 
							   offset 0x00a8, 
							   offset 0x00b0, 
							   offset 0x00b8 , 
							   offset 0x00c0 */

  // Rate_data is the calculated rate data. It is a first derivative
  // calculation. It is calculated at a frequency specified by the
  // variable rate_divisor (pg. 12). The data on which the rate is
  // calculated is specified by the variable rate_address (pg. 12).

  force_array_t rate_data; 				/* offset 0x00c8 */
  
  // Minimum_data & maximum_data are the minimum and maximum (peak)
  // data values. The JR3 DSP can monitor any 8 contiguous data items
  // for minimums and maximums at full sensor bandwidth. This area is
  // only updated at user request. This is done so that the user does
  // not miss any peaks. To read the data, use either the read peaks
  // command (pg. 40), or the read and reset peaks command (pg. 39).
  // The address of the data to watch for peaks is stored in the
  // variable peak_address (pg. 10). Peak data is lost when executing
  // a coordinate transformation or a full scale change. Peak data is
  // also lost when plugging in a new sensor.

  force_array_t minimum_data; 				/* offset 0x00d0 */
  force_array_t maximum_data; 				/* offset 0x00d8 */

  // Near_sat_value & sat_value contain the value used to determine if
  // the raw sensor is saturated. Because of decoupling and offset
  // removal, it is difficult to tell from the processed data if the
  // sensor is saturated. These values, in conjunction with the error
  // and warning words (pg. 14), provide this critical information.
  // These two values may be set by the host processor. These values
  // are positive signed values, since the saturation logic uses the
  // absolute values of the raw data. The near_sat_value defaults to
  // approximately 80% of the ADC's full scale, which is 26214, while
  // sat_value defaults to the ADC's full scale: 
  // 
  //   sat_value = 32768 - 2^(16 - ADC bits)


  s_val_t near_sat_value; 				/* offset 0x00e0 */
  s_val_t sat_value;  					/* offset 0x00e1 */

  // Rate_address, rate_divisor & rate_count contain the data used to
  // control the calculations of the rates. Rate_address is the
  // address of the data used for the rate calculation. The JR3 DSP
  // will calculate rates for any 8 contiguous values (ex. to
  // calculate rates for filter3 data set rate_address to 0x00a8).
  // Rate_divisor is how often the rate is calculated. If rate_divisor
  // is 1, the rates are calculated at full sensor bandwidth. If
  // rate_divisor is 200, rates are calculated every 200 samples.
  // Rate_divisor can be any value between 1 and 65536. Set
  // rate_divisor to 0 to calculate rates every 65536 samples.
  // Rate_count starts at zero and counts until it equals
  // rate_divisor, at which point the rates are calculated, and
  // rate_count is reset to 0. When setting a new rate divisor, it is
  // a good idea to set rate_count to one less than rate divisor. This
  // will minimize the time necessary to start the rate calculations.

  s_val_t rate_address;  				/* offset 0x00e2 */
  u_val_t rate_divisor;  				/* offset 0x00e3 */
  u_val_t rate_count;  					/* offset 0x00e4 */

  // Command_word2 through command_word0 are the locations used to
  // send commands to the JR3 DSP. Their usage varies with the command
  // and is detailed later in the Command Definitions section (pg.
  // 29). In general the user places values into various memory
  // locations, and then places the command word into command_word0.
  // The JR3 DSP will process the command and place a 0 into
  // command_word0 to indicate successful completion. Alternatively
  // the JR3 DSP will place a negative number into command_word0 to
  // indicate an error condition. Please note the command locations
  // are numbered backwards. (I.E. command_word2 comes before
  // command_word1).

  s_val_t command_word2;  				/* offset 0x00e5 */
  s_val_t command_word1;  				/* offset 0x00e6 */
  s_val_t command_word0;  				/* offset 0x00e7 */

  // Count1 through count6 are unsigned counters which are incremented
  // every time the matching filters are calculated. Filter1 is
  // calculated at the sensor data bandwidth. So this counter would
  // increment at 8 kHz for a typical sensor. The rest of the counters
  // are incremented at 1/4 the interval of the counter immediately
  // preceding it, so they would count at 2 kHz, 500 Hz, 125 Hz etc.
  // These counters can be used to wait for data. Each time the
  // counter changes, the corresponding data set can be sampled, and
  // this will insure that the user gets each sample, once, and only
  // once.

  u_val_t count1;  					/* offset 0x00e8 */
  u_val_t count2;  					/* offset 0x00e9 */
  u_val_t count3;  					/* offset 0x00ea */
  u_val_t count4;  					/* offset 0x00eb */
  u_val_t count5;  					/* offset 0x00ec */
  u_val_t count6;  					/* offset 0x00ed */

  // Error_count is a running count of data reception errors. If this
  // counter is changing rapidly, it probably indicates a bad sensor
  // cable connection or other hardware problem. In most installations
  // error_count should not change at all. But it is possible in an
  // extremely noisy environment to experience occasional errors even
  // without a hardware problem. If the sensor is well grounded, this
  // is probably unavoidable in these environments. On the occasions
  // where this counter counts a bad sample, that sample is ignored.

  u_val_t error_count;	  				/* offset 0x00ee */


  // Count_x is a counter which is incremented every time the JR3 DSP
  // searches its job queues and finds nothing to do. It indicates the
  // amount of idle time the JR3 DSP has available. It can also be
  // used to determine if the JR3 DSP is alive. See the Performance
  // Issues section on pg. 49 for more details.

  u_val_t count_x;	  				/* offset 0x00ef */

  // Warnings & errors contain the warning and error bits
  // respectively. The format of these two words is discussed on page
  // 21 under the headings warnings_bits and error_bits.

  u_val_t warnings;	  				/* offset 0x00f0 */
  u_val_t errors;	  				/* offset 0x00f1 */

  // Threshold_bits is a word containing the bits that are set by the
  // load envelopes. See load_envelopes (pg. 17) and thresh_struct
  // (pg. 23) for more details.
  
  s_val_t threshold_bits;	  			/* offset 0x00f2 */

  // Last_crc is the value that shows the actual calculated CRC. CRC
  // is short for cyclic redundancy code. It should be zero. See the
  // description for cal_crc_bad (pg. 21) for more information.
  
  s_val_t last_CRC;	  				/* offset 0x00f3 */

  // EEProm_ver_no contains the version number of the sensor EEProm.
  // EEProm version numbers can vary between 0 and 255.
  // Software_ver_no contains the software version number. Version
  // 3.02 would be stored as 302.

  s_val_t eeprom_ver_no;	  			/* offset 0x00f4 */
  s_val_t software_ver_no;	  			/* offset 0x00f5 */

  // Software_day & software_year are the release date of the software
  // the JR3 DSP is currently running. Day is the day of the year,
  // with January 1 being 1, and December 31, being 365 for non leap
  // years.

  s_val_t software_day;	  				/* offset 0x00f6 */
  s_val_t software_year;	  			/* offset 0x00f7 */

  // Serial_no & model_no are the two values which uniquely identify a
  // sensor. This model number does not directly correspond to the JR3
  // model number, but it will provide a unique identifier for
  // different sensor configurations.

  u_val_t serial_no;	  				/* offset 0x00f8 */
  u_val_t model_no;	  				/* offset 0x00f9 */

  // Cal_day & cal_year are the sensor calibration date. Day is the
  // day of the year, with January 1 being 1, and December 31, being
  // 366 for leap years.

  s_val_t cal_day;	  				/* offset 0x00fa */
  s_val_t cal_year;	  				/* offset 0x00fb */

  // Units is an enumerated read only value defining the engineering
  // units used in the sensor full scale. The meanings of particular
  // values are discussed in the section detailing the force_units
  // structure on page 22. The engineering units are setto customer
  // specifications during sensor manufacture and cannot be changed by
  // writing to Units.
  //
  // Bits contains the number of bits of resolution of the ADC
  // currently in use.
  //
  // Channels is a bit field showing which channels the current sensor
  // is capable of sending. If bit 0 is active, this sensor can send
  // channel 0, if bit 13 is active, this sensor can send channel 13,
  // etc. This bit can be active, even if the sensor is not currently
  // sending this channel. Some sensors are configurable as to which
  // channels to send, and this field only contains information on the
  // channels available to send, not on the current configuration. To
  // find which channels are currently being sent, monitor the
  // Raw_time fields (pg. 19) in the raw_channels array (pg. 7). If
  // the time is changing periodically, then that channel is being
  // received.

  u_val_t units;	  				/* offset 0x00fc */
  s_val_t bits;	  					/* offset 0x00fd */
  s_val_t channels;  					/* offset 0x00fe */

  // Thickness specifies the overall thickness of the sensor from
  // flange to flange. The engineering units for this value are
  // contained in units (pg. 16). The sensor calibration is relative
  // to the center of the sensor. This value allows easy coordinate
  // transformation from the center of the sensor to either flange.

  s_val_t thickness;  					/* offset 0x00ff */

  // Load_envelopes is a table containing the load envelope
  // descriptions. There are 16 possible load envelope slots in the
  // table. The slots are on 16 word boundaries and are numbered 0-15.
  // Each load envelope needs to start at the beginning of a slot but
  // need not be fully contained in that slot. That is to say that a
  // single load envelope can be larger than a single slot. The
  // software has been tested and ran satisfactorily with 50
  // thresholds active. A single load envelope this large would take
  // up 5 of the 16 slots. The load envelope data is laid out in an
  // order that is most efficient for the JR3 DSP. The structure is
  // detailed later in the section showing the definition of the
  // le_struct structure (pg. 23).

  le_struct_t load_envelopes[0x10];			/* offset 0x0100 */
  
  // Transforms is a table containing the transform descriptions.
  // There are 16 possible transform slots in the table. The slots are
  // on 16 word boundaries and are numbered 0-15. Each transform needs
  // to start at the beginning of a slot but need not be fully
  // contained in that slot. That is to say that a single transform
  // can be larger than a single slot. A transform is 2 * no of links
  // + 1 words in length. So a single slot can contain a transform
  // with 7 links. Two slots can contain a transform that is 15 links.
  // The layout is detailed later in the section showing the
  // definition of the transform structure (pg. 26).

  intern_transform_t transforms[0x10];			/* offset 0x0200 */
} jr3_channel_t;

typedef struct {
  struct {
    u_val_t program_low[0x4000];	// 0x00000 - 0x10000
    jr3_channel_t data;			// 0x10000 - 0x10c00
    char pad2[0x30000 - 0x00c00];	// 0x10c00 - 0x40000
    u_val_t program_high[0x8000];	// 0x40000 - 0x60000
    u32 reset;				// 0x60000 - 0x60004
    char pad3[0x20000 - 0x00004];	// 0x60004 - 0x80000
  } channel[4];
} jr3_t;

