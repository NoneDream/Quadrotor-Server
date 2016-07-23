OBJ = server.o
LINK = -lpthread -lwiringPi

server:$(OBJ)
	gcc -o server $(OBJ) $(LINK)

server.o:server.h

.PHONY:clean
clean:
	rm server $(OBJ)

