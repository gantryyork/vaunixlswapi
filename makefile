all : LSWtest LSWhid.o

# How we create LSWhid.o
LSWhid.o : LSWhid.c LSWhid.h
	gcc -o LSWhid.o -c LSWhid.c

# The final output is one or more linked object files
LSWtest : LSWtest.c LSWhid.o LSWhid.h
	gcc -o LSWtest -lm -lpthread -lusbl LSWtest.c LSWhid.o

clean :
	rm -f LSWhid.o
	rm -f LSWtest
