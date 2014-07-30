#change to correct file
#OBJECTS = main.o 
TARGET = main
SRC = $(TARGET).c
#put libs below
LIBS = 
OBJECTS = $(SRC) $(LIBS)
 
CC = msp430-gcc
#change to correct ic here
CFLAGS =-Os -Wall -g -mmcu=msp430g2252
 
all : $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o main.elf
 
%.o : %.c 
	$(CC) $(CFLAGS) -c $<

 
clean:
	rm -fr  main.elf
 
erase:
	mspdebug rf2500 "erase"
 
upload:
	mspdebug rf2500 "prog main.elf"

tilib-upload:

 	mspdebug-new tilib "prog main.elf"

size:
	msp430-size main.elf
