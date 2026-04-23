/*
    comedi/drivers/comedi_test.c

    Generates fake waveform signals that can be read through
    the command interface.  It does _not_ read from any board;
    it just generates deterministic waveforms.
    Useful for various testing purposes.

    Copyright (C) 2002 Joachim Wuttke <Joachim.Wuttke@icn.siemens.de>
    Copyright (C) 2002 Frank Mori Hess <fmhess@users.sourceforge.net>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@schleef.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

************************************************************************/
/*
Driver: comedi_test
Description: generates fake waveforms
Author: Joachim Wuttke <Joachim.Wuttke@icn.siemens.de>, Frank Mori Hess
  <fmhess@users.sourceforge.net>, ds
Devices:
Status: works
Updated: Sat, 16 Mar 2002 17:34:48 -0800

This driver is mainly for testing purposes, but can also be used to
generate sample waveforms on systems that don't have data acquisition
hardware.

Configuration options:
  [0] - Amplitude in microvolts for fake waveforms (default 1 volt)
  [1] - Period in microseconds for fake waveforms (default 0.1 sec)

Generates a sawtooth wave on channel 0, square wave on channel 1, additional
waveforms could be added to other channels (currently they return flatline
zero volts).

*/

#include <linux/comedidev.h>

#include <asm/div64.h>

/* Board descriptions */
typedef struct waveform_board_struct {
	const char *name;
	int ai_chans;
	int ai_bits;
	int have_dio;
} waveform_board;

#define N_CHANS 8

static const waveform_board waveform_boards[] = {
	{
		.name		= "comedi_test",
		.ai_chans	= N_CHANS,
		.ai_bits	= 16,
		.have_dio	= 0,
	},
};

#define thisboard ((const waveform_board *)dev->board_ptr)

/* Data unique to this driver */
typedef struct {
	comedi_device *dev;		// parent comedi device
	struct timer_list ai_timer;	// timer for AI commands
	u64 ai_convert_time;		// time of next AI conversion in usec
	unsigned int wf_amplitude;	// waveform amplitude in microvolts
	u32 wf_period;			// waveform period in microseconds
	u32 wf_current;			// current time in waveform period
	unsigned int ai_scan_period;	// AI scan period in microseconds
	unsigned int ai_convert_period;	// AI conversion period in microseconds
	struct timer_list ao_timer;	// timer for AO commands
	u64 ao_last_scan_time;		// time of previous AO scan in microsecs
	unsigned int ao_scan_period;	// AO scan period in microseconds
	bool ai_timer_enable:1;		// should AI timer be running?
	bool ao_timer_enable:1;		// should AO timer be running?
	sampl_t ao_loopbacks[N_CHANS];
} waveform_private;
#define devpriv ((waveform_private *)dev->private)

static int waveform_attach(comedi_device * dev, comedi_devconfig * it);
static int waveform_detach(comedi_device * dev);
static comedi_driver driver_waveform = {
	.driver_name	= "comedi_test",
	.module		= THIS_MODULE,
	.attach		= waveform_attach,
	.detach		= waveform_detach,
	.board_name	= &waveform_boards[0].name,
	.offset		= sizeof(waveform_board),
	.num_names	= ARRAY_SIZE(waveform_boards),
};

COMEDI_INITCLEANUP(driver_waveform);

static int waveform_ai_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd);
static int waveform_ai_cmd(comedi_device * dev, comedi_subdevice * s);
static int waveform_ai_cancel(comedi_device * dev, comedi_subdevice * s);
static int waveform_ai_insn_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int waveform_ao_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd);
static int waveform_ao_cmd(comedi_device * dev, comedi_subdevice * s);
static int waveform_ao_cancel(comedi_device * dev, comedi_subdevice * s);
static int waveform_ao_insn_write(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static sampl_t fake_sawtooth(comedi_device * dev, unsigned int range,
	unsigned int current_time);
static sampl_t fake_squarewave(comedi_device * dev, unsigned int range,
	unsigned int current_time);
static sampl_t fake_flatline(comedi_device * dev, unsigned int range,
	unsigned int current_time);
static sampl_t fake_waveform(comedi_device * dev, unsigned int channel,
	unsigned int range, unsigned int current_time);

// fake analog input ranges
static const comedi_lrange waveform_ai_ranges = {
	2,
	{
		BIP_RANGE(10),
		BIP_RANGE(5),
	},
};

/*
   This is the background routine used to generate arbitrary data.
   It should run in the background; therefore it is scheduled by
   a timer mechanism.
*/
static void waveform_ai_timer(struct timer_list *t)
{
	waveform_private *priv = timer_container_of(priv, t, ai_timer);
	comedi_device *dev = priv->dev;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	u64 now;
	unsigned int nsamples;
	unsigned int time_increment;

	now = ktime_to_us(ktime_get());
	nsamples = comedi_nsamples_left(s, UINT_MAX);

	while (nsamples && devpriv->ai_convert_time < now) {
		unsigned int chanspec = cmd->chanlist[async->cur_chan];
		sampl_t sample =
			fake_waveform(dev, CR_CHAN(chanspec),
				CR_RANGE(chanspec), devpriv->wf_current);
		u64 wf_time64;

		if (comedi_buf_write_samples(s, &sample, 1) == 0)
			goto overrun;
		time_increment = devpriv->ai_convert_period;
		if (async->scan_progress == 0) {
			/* done last conversion in scan, so add dead time */
			time_increment +=
				devpriv->ai_scan_period -
				(devpriv->ai_convert_period *
				 cmd->scan_end_arg);
		}
		wf_time64 = (u64)devpriv->wf_current + time_increment;
		if (wf_time64 < devpriv->wf_period) {
			devpriv->wf_current = wf_time64;
		} else {
			div_u64_rem(wf_time64, devpriv->wf_period,
				&devpriv->wf_current);
		}
		devpriv->ai_convert_time += time_increment;
		nsamples--;
	}

	if (cmd->stop_src == TRIG_COUNT && async->scans_done >= cmd->stop_arg) {
		async->events |= COMEDI_CB_EOA;
	} else {
		unsigned long flags;

		if (devpriv->ai_convert_time > now)
			time_increment = devpriv->ai_convert_time - now;
		else
			time_increment = 1;
		comedi_spin_lock_irqsave(&dev->spinlock, flags);
		if (devpriv->ai_timer_enable) {
			mod_timer(&devpriv->ai_timer,
				  jiffies + usecs_to_jiffies(time_increment));
		}
		comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
	}

overrun:
	comedi_handle_events(dev, s);
}

/*
 * This is the background routine to handle AO commands, scheduled by
 * a timer mechanism.
 */
static void waveform_ao_timer(struct timer_list *t)
{
	waveform_private *priv = timer_container_of(priv, t, ao_timer);
	comedi_device *dev = priv->dev;
	comedi_subdevice *s = dev->write_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	u64 now;
	u64 scans_since;
	unsigned int scans_avail = 0;

	/* determine number of scan periods since last time */
	now = ktime_to_us(ktime_get());
	scans_since = div_u64(now - devpriv->ao_last_scan_time,
			devpriv->ao_scan_period);
	if (scans_since) {
		unsigned int i;

		/* determine scans in buffer, limit to scans to do this time */
		scans_avail = comedi_nscans_left(s, 0);
		if (scans_avail > scans_since)
			scans_avail = scans_since;
		if (scans_avail) {
			/* skip all but the last scan to save processing time */
			if (scans_avail > 1) {
				unsigned int skip_bytes, nbytes;

				skip_bytes =
				comedi_samples_to_bytes(s, cmd->scan_end_arg *
							   (scans_avail - 1));
				nbytes = comedi_buf_read_alloc(s, skip_bytes);
				comedi_buf_read_free(s, nbytes);
				comedi_inc_scan_progress(s, nbytes);
				if (nbytes < skip_bytes) {
					/* unexpected underrun! (cancelled?) */
					async->events |= COMEDI_CB_OVERFLOW;
					goto underrun;
				}
			}
			/* output the last scan */
			for (i = 0; i < cmd->scan_end_arg; i++) {
				unsigned int chan = CR_CHAN(cmd->chanlist[i]);
				sampl_t *pd;

				pd = &devpriv->ao_loopbacks[chan];
				if (!comedi_buf_read_samples(s, pd, 1)) {
					/* unexpected underrun! (cancelled?) */
					async->events |= COMEDI_CB_OVERFLOW;
					goto underrun;
				}
			}
			/* advance time of last scan */
			devpriv->ao_last_scan_time +=
				(u64)scans_avail * devpriv->ao_scan_period;
		}
	}
	if (cmd->stop_src == TRIG_COUNT && async->scans_done >= cmd->stop_arg) {
		async->events |= COMEDI_CB_EOA;
	} else if (scans_avail < scans_since) {
		async->events |= COMEDI_CB_OVERFLOW;
	} else {
		unsigned int time_inc = devpriv->ao_last_scan_time +
					devpriv->ao_scan_period - now;
		unsigned long flags;

		comedi_spin_lock_irqsave(&dev->spinlock, flags);
		if (devpriv->ao_timer_enable) {
			mod_timer(&devpriv->ao_timer,
				  jiffies + usecs_to_jiffies(time_inc));
		}
		comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
	}

underrun:
	comedi_handle_events(dev, s);
}

static int waveform_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	int amplitude = it->options[0];
	int period = it->options[1];

	printk("comedi%d: comedi_test: ", dev->minor);

	dev->board_name = thisboard->name;

	if (alloc_private(dev, sizeof(waveform_private)) < 0) {
		printk(KERN_CONT "Allocation error\n");
		return -ENOMEM;
	}

	devpriv->dev = dev;

	// set default amplitude and period
	if (amplitude <= 0)
		amplitude = 1000000;	// 1 volt
	if (period <= 0)
		period = 100000;	// 0.1 sec

	devpriv->wf_amplitude = amplitude;
	devpriv->wf_period = period;

	printk(KERN_CONT "%u microvolt, %u microsecond waveform ",
		devpriv->wf_amplitude, devpriv->wf_period);
	dev->n_subdevices = 2;
	if (alloc_subdevices(dev, dev->n_subdevices) < 0) {
		printk(KERN_CONT "Allocation error\n");
		return -ENOMEM;
	}

	s = dev->subdevices + 0;
	dev->read_subdev = s;
	/* analog input subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ;
	s->n_chan = thisboard->ai_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &waveform_ai_ranges;
	s->len_chanlist = s->n_chan * 2;
	s->insn_read = waveform_ai_insn_read;
	s->do_cmd = waveform_ai_cmd;
	s->do_cmdtest = waveform_ai_cmdtest;
	s->cancel = waveform_ai_cancel;

	s = dev->subdevices + 1;
	dev->write_subdev = s;
	/* analog output subdevice (loopback) */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE | SDF_GROUND | SDF_CMD_WRITE;
	s->n_chan = thisboard->ai_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &waveform_ai_ranges;
	s->len_chanlist = s->n_chan;
	s->insn_write = waveform_ao_insn_write;
	s->insn_read = waveform_ai_insn_read;	/* do same as AI insn_read */
	s->do_cmd = waveform_ao_cmd;
	s->do_cmdtest = waveform_ao_cmdtest;
	s->cancel = waveform_ao_cancel;
	{
		/* Our default loopback value is just a 0V flatline */
		int i;
		for (i = 0; i < s->n_chan; i++)
			devpriv->ao_loopbacks[i] = s->maxdata / 2;
	}

	timer_setup(&devpriv->ai_timer, waveform_ai_timer, 0);
	timer_setup(&devpriv->ao_timer, waveform_ao_timer, 0);

	printk(KERN_CONT "attached\n");

	return 1;
}

static int waveform_detach(comedi_device * dev)
{
	printk("comedi%d: comedi_test: remove\n", dev->minor);

	if (dev->private) {
		waveform_ai_cancel(dev, dev->read_subdev);
	}

	return 0;
}

static int waveform_ai_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	unsigned int tmp, limit;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_TIMER;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW | TRIG_TIMER;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2a: make sure trigger sources are unique */

	if (cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->scan_begin_src != TRIG_TIMER) {
		err++;
	}
	if (cmd->convert_src != TRIG_NOW && cmd->convert_src != TRIG_TIMER)
		err++;
	if (cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	/* step 2b: and mutually compatible */

	if (cmd->scan_begin_src == TRIG_FOLLOW && cmd->convert_src == TRIG_NOW)
		err++;	/* scan period would be 0 */

	if (err)
		return 2;

	/* step 3: make sure arguments are trivially compatible */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}
	if (cmd->convert_src == TRIG_NOW) {
		if (cmd->convert_arg != 0) {
			cmd->convert_arg = 0;
			err++;
		}
	} else {
		/* cmd->convert_src == TRIG_TIMER */
		if (cmd->scan_begin_src == TRIG_FOLLOW) {
			if (cmd->convert_arg < NSEC_PER_USEC) {
				cmd->convert_arg = NSEC_PER_USEC;
			}
		}
	}
	if (cmd->scan_begin_src == TRIG_FOLLOW) {
		if (cmd->scan_begin_arg != 0) {
			cmd->scan_begin_arg = 0;
			err++;
		}
	} else {
		/* cmd->scan_begin_src == TRIG_TIMER */
		if (cmd->scan_begin_arg < NSEC_PER_USEC) {
			cmd->scan_begin_arg = NSEC_PER_USEC;
			err++;
		}
	}
	if (cmd->scan_begin_src == TRIG_TIMER &&
		cmd->convert_src == TRIG_TIMER &&
		cmd->scan_begin_arg < cmd->convert_arg * cmd->chanlist_len) {
		cmd->scan_begin_arg = cmd->convert_arg * cmd->chanlist_len;
		err++;
	}
	// XXX these checks are generic and should go in core if not there already
	if (!cmd->chanlist_len) {
		cmd->chanlist_len = 1;
		err++;
	}
	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	if (cmd->stop_src == TRIG_COUNT) {
		if (!cmd->stop_arg) {
			cmd->stop_arg = 1;
			err++;
		}
	} else {		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	if (cmd->convert_src == TRIG_TIMER) {
		/* round convert_arg to nearest microsecond */
		tmp = cmd->convert_arg;
		tmp = min(tmp,
			rounddown(UINT_MAX, (unsigned int)NSEC_PER_USEC));
		tmp = NSEC_PER_USEC * DIV_ROUND_CLOSEST(tmp, NSEC_PER_USEC);
		if (cmd->scan_begin_src == TRIG_TIMER) {
			/* limit convert_arg to keep scan_begin_arg in range */
			limit = UINT_MAX / cmd->scan_end_arg;
			limit = rounddown(limit, (unsigned int)NSEC_PER_USEC);
			tmp = min(tmp, limit);
		}
		if (tmp != cmd->convert_arg) {
			cmd->convert_arg = tmp;
			err++;
		}
	}
	if (cmd->scan_begin_src == TRIG_TIMER) {
		/* round scan_begin_arg to nearest microsecond */
		tmp = cmd->scan_begin_arg;
		tmp = min(tmp, rounddown(UINT_MAX,
					(unsigned int)NSEC_PER_USEC));
		tmp = NSEC_PER_USEC * DIV_ROUND_CLOSEST(tmp, NSEC_PER_USEC);
		if (cmd->convert_src == TRIG_TIMER) {
			/* but ensure scan_begin_arg is large enough */
			tmp = max(tmp, cmd->convert_arg * cmd->scan_end_arg);
		}
		if (tmp != cmd->scan_begin_arg) {
			cmd->scan_begin_arg = tmp;
			err++;
		}
	}

	if (err)
		return 4;

	return 0;
}

static int waveform_ai_cmd(comedi_device * dev, comedi_subdevice * s)
{
	comedi_cmd *cmd = &s->async->cmd;
	unsigned int first_convert_time;
	u64 wf_current;
	unsigned long flags;

	if (cmd->flags & CMDF_PRIORITY) {
		comedi_error(dev,
			"commands at RT priority not supported in this driver");
		return -1;
	}

	if (cmd->convert_src == TRIG_NOW)
		devpriv->ai_convert_period = 0;
	else /* cmd->convert_src == TRIG_TIMER */
		devpriv->ai_convert_period = cmd->convert_arg / NSEC_PER_USEC;

	if (cmd->scan_begin_src == TRIG_FOLLOW) {
		devpriv->ai_scan_period =
			devpriv->ai_convert_period * cmd->scan_end_arg;
	} else {
		/* cmd->scan_begin_src == TRIG_TIMER */
		devpriv->ai_scan_period = cmd->scan_begin_arg / NSEC_PER_USEC;
	}

	/*
	 * Simulate first conversion to occur at convert period after
	 * conversion timer starts.  If scan_begin_src is TRIG_FOLLOW, assume
	 * the conversion timer starts immediately.  If scan_begin_src is
	 * TRIG_TIMER, assume the conversion timer starts after the scan
	 * period.
	 */
	first_convert_time = devpriv->ai_convert_period;
	if (cmd->scan_begin_src == TRIG_TIMER)
		first_convert_time += devpriv->ai_scan_period;
	devpriv->ai_convert_time = ktime_to_us(ktime_get()) +
				   first_convert_time;

	/* Determine time within waveform period at time of conversion. */
	wf_current = devpriv->ai_convert_time;
	div_u64_rem(devpriv->ai_convert_time, devpriv->wf_period,
			&devpriv->wf_current);

	/*
	 * Schedule timer to expire just after first conversion time.
	 * Seem to need an extra jiffy here, otherwise timer expires slightly
	 * early!
	 */
	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	devpriv->ai_timer_enable = true;
	devpriv->ai_timer.expires =
		jiffies + usecs_to_jiffies(devpriv->ai_convert_period) + 1;
	add_timer(&devpriv->ai_timer);
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
	return 0;
}

static int waveform_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long flags;

	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	devpriv->ai_timer_enable = false;
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
	if (in_softirq()) {
		/*
		 * Assume we were called from the timer routing itself
		 * (via comedi_handle_events()).
		 */
		timer_delete(&devpriv->ai_timer);
	} else {
		timer_delete_sync(&devpriv->ai_timer);
	}
	return 0;
}

static sampl_t fake_sawtooth(comedi_device * dev, unsigned int range_index,
	unsigned int current_time)
{
	comedi_subdevice *s = dev->read_subdev;
	unsigned int offset = s->maxdata / 2;
	u64 value;
	const comedi_krange *krange = &s->range_table->range[range_index];
	u64 binary_amplitude;

	binary_amplitude = s->maxdata;
	binary_amplitude *= devpriv->wf_amplitude;
	binary_amplitude = div_u64(binary_amplitude, krange->max - krange->min);

	value = current_time;
	value *= binary_amplitude * 2;
	value = div_u64(value, devpriv->wf_period);
	value -= binary_amplitude;	// get rid of sawtooth's dc offset

	return offset + value;
}
static sampl_t fake_squarewave(comedi_device * dev, unsigned int range_index,
	unsigned int current_time)
{
	comedi_subdevice *s = dev->read_subdev;
	unsigned int offset = s->maxdata / 2;
	u64 value;
	const comedi_krange *krange = &s->range_table->range[range_index];

	value = s->maxdata;
	value *= devpriv->wf_amplitude;
	value = div_u64(value, krange->max - krange->min);

	if (current_time < devpriv->wf_period / 2)
		value *= -1;

	return offset + value;
}

static sampl_t fake_flatline(comedi_device * dev, unsigned int range_index,
	unsigned int current_time)
{
	return dev->read_subdev->maxdata / 2;
}

// generates a different waveform depending on what channel is read
static sampl_t fake_waveform(comedi_device * dev, unsigned int channel,
	unsigned int range, unsigned int current_time)
{
	enum {
		SAWTOOTH_CHAN,
		SQUARE_CHAN,
	};
	switch (channel) {
	case SAWTOOTH_CHAN:
		return fake_sawtooth(dev, range, current_time);
		break;
	case SQUARE_CHAN:
		return fake_squarewave(dev, range, current_time);
		break;
	default:
		break;
	}

	return fake_flatline(dev, range, current_time);
}

static int waveform_ai_insn_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i, chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_loopbacks[chan];

	return insn->n;
}

static int waveform_ao_inttrig_start(comedi_device * dev, comedi_subdevice * s,
	unsigned int trig_num)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned long flags;

	if (trig_num != cmd->start_arg)
		return -EINVAL;

	async->inttrig = NULL;

	devpriv->ao_last_scan_time = ktime_to_us(ktime_get());
	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	devpriv->ao_timer_enable = true;
	devpriv->ao_timer.expires =
		jiffies + usecs_to_jiffies(devpriv->ao_scan_period);
	add_timer(&devpriv->ao_timer);
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);

	return 1;
}

static int waveform_ao_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	unsigned int tmp;

	/* Step 1 : check if triggers are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_INT;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* Step 2a : make sure trigger sources are unique */

	if (cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	/* Step 2b : and mutually compatible */

	if (err)
		return 2;

	/* Step 3: check if arguments are trivially valid */

	if (cmd->scan_begin_arg < NSEC_PER_USEC) {
		cmd->scan_begin_arg = NSEC_PER_USEC;
		err++;
	}

	if (cmd->convert_arg != 0) {
		cmd->convert_arg = 0;
		err++;
	}

	if (cmd->chanlist_len < 1) {
		cmd->chanlist_len = 1;
		err++;
	}
	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	if (cmd->stop_src == TRIG_COUNT) {
		if (!cmd->stop_arg) {
			cmd->stop_arg = 1;
			err++;
		}
	} else {		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	/* round scan_begin_arg to nearest microsecond */
	tmp = cmd->scan_begin_arg;
	tmp = min(tmp, rounddown(UINT_MAX, (unsigned int)NSEC_PER_USEC));
	tmp = NSEC_PER_USEC * DIV_ROUND_CLOSEST(tmp, NSEC_PER_USEC);
	if (cmd->scan_begin_arg != tmp) {
		cmd->scan_begin_arg = tmp;
		err++;
	}

	if (err)
		return 4;

	return 0;
}

static int waveform_ao_cmd(comedi_device * dev, comedi_subdevice * s)
{
	comedi_cmd *cmd = &s->async->cmd;

	if (cmd->flags & CMDF_PRIORITY) {
		comedi_error(dev,
			"commands at RT priority not supported in this driver");
		return -1;
	}

	devpriv->ao_scan_period = cmd->scan_begin_arg / NSEC_PER_USEC;
	s->async->inttrig = waveform_ao_inttrig_start;
	return 0;
}

static int waveform_ao_cancel(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long flags;

	s->async->inttrig = NULL;
	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	devpriv->ao_timer_enable = false;
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
	if (in_softirq()) {
		/* Assume we were called from the timer routine itself. */
		timer_delete(&devpriv->ao_timer);
	} else {
		timer_delete_sync(&devpriv->ao_timer);
	}
	return 0;
}

static int waveform_ao_insn_write(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i, chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		devpriv->ao_loopbacks[chan] = data[i];

	return insn->n;
}
