

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "../include/comedi.h"

char s[100];
char name[100];

int get_flags(char *p);

#define skip_white(a)	{while(*(a) && isspace(*(a)))(a)++;}

void help(void)
{
	fprintf(stderr, "Use the source, dumbass\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	char *p;
	double a, b;
	int i = 0, j = 0, ln = 0;
	int mode = 0;
	int flags = 0;
	int n;

	if (argc != 2)
		help();
	if (!strcmp(argv[1], "include")) {
		mode = 1;
	} else if (!strcmp(argv[1], "source")) {
		mode = 2;
	} else
		help();

	if(mode==1){
	}else{
		printf("#ifdef RANGE_C\n");
		printf("comedi_krange comedi_kranges[]={\n");
	}
	while (1) {
		fgets(s, 100, stdin);
		ln++;
		if (feof(stdin))
			break;

		s[strlen(s) - 1] = 0;
		p = s;
		skip_white(p);
		if(p==s){
			if (mode == 1) {
				if (i)
					printf("%d)\n", j);
				printf("#define %s __RANGE(%d,", p, i);
			} else {
				printf("/* %s */\n", p);
			}
			j = 0;
		}else{
			if (isdigit(*p) || (*p) == '-') {
				sscanf(p, "%lf %lf%n", &a, &b , &n);
				p+=n;

				flags=get_flags(p);
				if(flags<0){
					printf("\n\n#error while running scripts/mk_range, line %d, unknown flag\n", ln);
					return 1;
				}

				if (mode == 2) {
					printf("\t{%d,%d,%d},\n", (int)(1e6*a), (int)(1e6*b),flags);
				}
				i++;
				j++;
			} else {
				printf("\n\n#error while running scripts/mk_range, line %d\n", ln);
				return 1;
			}
		}
	}
	if(mode==1){
		printf("%d)\n", j);
	}else{
		printf("};\n");
		printf("unsigned int comedi_max_range=%d;\n",i);
		printf("#endif\n");
	}
	return 0;
}

int get_flags(char *p)
{
	int flags=0;

	while(*p){
		skip_white(p);
		if(!strncmp(p,"ext",3)){
			flags|=RF_EXTERNAL;
			p+=3;
			continue;
		}
		if(!strncmp(p,"mA",2)){
			flags|=UNIT_mA;
			p+=2;
			continue;
		}
		if(!strncmp(p,"volt",4)){
			flags|=UNIT_volt;
			p+=4;
			continue;
		}
		if(!strncmp(p,"none",4)){
			flags|=UNIT_none;
			p+=4;
			continue;
		}
		if(!strncmp(p,"unitless",8)){
			flags|=UNIT_none;
			p+=8;
			continue;
		}
		return -1;
	}
	return flags;
}

