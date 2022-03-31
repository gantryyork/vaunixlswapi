all : LSWtest LSWhid.o

# How we create LSWhid.o
LSWhid.o : LSWhid.c LSWhid.h
	gcc -o LSWhid.o -c LSWhid.c -w

# The final output is one or more linked object files
LSWtest : LSWtest.c LSWhid.o LSWhid.h
	gcc -o LSWtest LSWtest.c LSWhid.o -lm -lpthread -lusb

clean :
	rm -f LSWhid.o
	rm -f LSWtest
