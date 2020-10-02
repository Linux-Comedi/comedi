/*
    module/proc.c
    /proc interface for comedi

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@schleef.org>

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

*/

/*
	This is some serious bloatware.

	Taken from Dave A.'s PCL-711 driver, 'cuz I thought it
	was cool.
*/

#define __NO_VERSION__
#include <linux/comedidev.h>
#include <linux/proc_fs.h>

extern comedi_driver *comedi_drivers;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
/* Old version */

int comedi_read_procmem(char *buf, char **start, off_t offset, int len,
	int *eof, void *data)
{
	int i;
	int devices_q = 0;
	int l = 0;
	comedi_driver *driv;

	l += sprintf(buf + l,
		"comedi version " COMEDI_RELEASE "\n"
		"format string: %s\n",
		"\"%2d: %-20s %-20s %4d\",i,driver_name,board_name,n_subdevices");

	for (i = 0; i < COMEDI_NUM_BOARD_MINORS; i++) {
		comedi_device *dev = comedi_get_device_by_minor(i);

		if (dev == NULL) continue;

		if (dev->attached) {
			devices_q = 1;
			l += sprintf(buf + l, "%2d: %-20s %-20s %4d\n",
				i,
				dev->driver->driver_name,
				dev->board_name, dev->n_subdevices);
		}
	}
	if (!devices_q) {
		l += sprintf(buf + l, "no devices\n");
	}

	for (driv = comedi_drivers; driv; driv = driv->next) {
		l += sprintf(buf + l, "%s:\n", driv->driver_name);
		for (i = 0; i < driv->num_names; i++) {
			l += sprintf(buf + l, " %s\n",
				*(char **)((char *)driv->board_name +
					i * driv->offset));
		}
		if (!driv->num_names) {
			l += sprintf(buf + l, " %s\n", driv->driver_name);
		}
	}

	return l;
}

void comedi_proc_init(void)
{
	struct proc_dir_entry *comedi_proc;

	comedi_proc = create_proc_entry("comedi", S_IFREG | S_IRUGO, 0);
	if (comedi_proc)
		comedi_proc->read_proc = comedi_read_procmem;
}

#else
/* New version, using proc_create() and the seq_file interface. */
#include <linux/seq_file.h>

static int comedi_read(struct seq_file *m, void *v)
{
	int i;
	int devices_q = 0;
	comedi_driver *driv;

	seq_printf(m,
		     "comedi version " COMEDI_RELEASE "\n"
		     "format string: %s\n",
		     "\"%2d: %-20s %-20s %4d\", i, "
		     "driver_name, board_name, n_subdevices");

	for (i = 0; i < COMEDI_NUM_BOARD_MINORS; i++) {
		comedi_device *dev = comedi_get_device_by_minor(i);

		if (dev == NULL)
			continue;

		if (dev->attached) {
			devices_q = 1;
			seq_printf(m, "%2d: %-20s %-20s %4d\n",
				   i, dev->driver->driver_name,
				   dev->board_name, dev->n_subdevices);
		}
	}
	if (!devices_q)
		seq_puts(m, "no devices\n");

	for (driv = comedi_drivers; driv; driv = driv->next) {
		seq_printf(m, "%s:\n", driv->driver_name);
		for (i = 0; i < driv->num_names; i++)
			seq_printf(m, " %s\n",
				   *(char **)((char *)driv->board_name +
					      i * driv->offset));

		if (!driv->num_names)
			seq_printf(m, " %s\n", driv->driver_name);
	}

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,18,0)
/* proc_create_single() is not available */

/*
 * seq_file wrappers for procfile show routines.
 */
static int comedi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, comedi_read, NULL);
}

static const struct file_operations comedi_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= comedi_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void comedi_proc_init(void)
{
	proc_create("comedi", S_IFREG | S_IRUGO, NULL, &comedi_proc_fops);
}

#else
/* proc_create_single() is available */

void comedi_proc_init(void)
{
	proc_create_single("comedi", S_IFREG | S_IRUGO, NULL, comedi_read);
}

#endif
#endif

void comedi_proc_cleanup(void)
{
	remove_proc_entry("comedi", 0);
}
