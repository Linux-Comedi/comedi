/*
    comedi_config/comedi_config.c
    configuration program for comedi kernel module

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>

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

#define CC_VERSION	"0.6.13"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <comedi.h>

int quiet=0,verbose=0,script=0;

void do_script(void);

void do_help(int i)
{
	fputs(
"comedi_config version " CC_VERSION "\n"
"usage:  comedi_config [-[vVq]] <device file> <driver> <option1>,<option2>,...\n"
"where <optionN> are integers (or blank) whose interpretation depends on\n"
"the driver.  In general, however, option1 refers to the I/O port address\n"
"and option2 refers to IRQ number to be used by the driver\n"
		,stderr);
	exit(i);
}

int main(int argc,char *argv[])
{
	comedi_devconfig it;
	int fd;
	int c,i,num,k;
	char *opts;
	char *fn,*driver;
	struct stat statbuf;
	int ret;
	int remove=0;
	
	while(1){
		c=getopt(argc,argv,"rvVq");
		if(c==-1)break;
		switch(c){
		case 'v':
			verbose=1;
			break;
		case 'V':
			fputs("comedi_config version " CC_VERSION "\n",stderr);
			exit(0);
			break;
		case 'q':
			quiet=1;
			break;
		case 'a':
			script=1;
			break;
		case 'r':
			remove=1;
			break;
		default:
			do_help(1);
		}
	}

	if(script){
		if( (argc-optind)>0 ){
			do_help(1);
		}else{
			do_script();
		}
	}
	
	if((argc-optind)!=2 && (argc-optind)!=3){
		do_help(1);
	}
		
	fn=argv[optind];
	
	driver=argv[optind+1];
	strncpy(it.board_name,driver,COMEDI_NAMELEN-1);

	for(i=0;i<COMEDI_NDEVCONFOPTS;i++)it.options[i]=0;

	if((argc-optind)==3){
		opts=argv[optind+2];
		i=0;
		while(*opts){
			if((*opts)==','){
				i++;
				opts++;
				if(i>=COMEDI_NDEVCONFOPTS)
					do_help(1);
				continue;
			}
			if(sscanf(opts,"%i%n",&num,&k)>0){
				it.options[i]=num;
				opts+=k;
				continue;
			}
			do_help(1);
		}
	}
	
	ret=stat(fn,&statbuf);
	if(ret<0){
		perror(fn);
		exit(1);
	}
#if 0
	/* this appears to be broken */
	if(  !(S_ISCHR(statbuf.st_mode)) ||
	     major(statbuf.st_dev)!=COMEDI_MAJOR){
		if(!quiet)
			fprintf(stderr,"warning: %s might not be a comedi device\n",fn);
	}
#endif
	
	fd=open(fn,O_RDWR);
	if(fd<0){
		perror(fn);
		exit(1);
	}
	
	/* add: sanity check for device */
	
	if(verbose){
		printf("configuring driver=%s ",it.board_name);
		for(i=0;i<COMEDI_NDEVCONFOPTS;i++)printf("%d,",it.options[i]);
		printf("\n");
	}
	if(ioctl(fd,COMEDI_DEVCONFIG,remove?NULL:&it)<0){
		perror("Configure failed!");
		exit(1);
	}
	exit(0);
}

void do_script(void)
{
	printf("comedi_config: -a option not supported (yet).\n");
	exit(0);
}

