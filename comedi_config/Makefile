
CFLAGS=-Wall -O2 -I../include
#LDFLAGS=-L../lib/ -lcomedi
LDFLAGS=

OBJS=comedi_config.o

comedi_config: $(OBJS)
	$(CC) -o comedi_config $(OBJS) $(LDFLAGS)

clean:
	rm -f comedi_config *.o
