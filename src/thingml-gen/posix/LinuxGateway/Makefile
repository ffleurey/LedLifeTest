CC = cc
LIBS = -lpthread 
CFLAGS = -O2 -w
SRCS = LinuxGateway.c StatsdLogger.c LinuxGateway_cfg.c Timer.c runtime.c
OBJS = LinuxGateway.o StatsdLogger.o LinuxGateway_cfg.o Timer.o runtime.o

all : LinuxGateway

.c.o :
	${CC} ${CFLAGS} -c $< 

.cpp.o :
	${CC} ${CFLAGS} -c $< 

LinuxGateway : $(OBJS)
	$(CC) -o $@ $(OBJS) $(LIBS) -lm

clean:
	rm -f *.o *~ LinuxGateway
