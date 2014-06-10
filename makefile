#  ***  MACROS
#.SUFFIXES: .cpp

CC=g++
CFLAGS=-std=c++11 -I"usr/include" 

OBJECTS= main.o SDL_Helper.o
LIBS= \
      -L"usr/lib" -lSDL -lSDL_image\
	-lopencv_calib3d -lopencv_contrib -lopencv_core\
	-lopencv_features2d -lopencv_flann -lopencv_highgui\
	-lopencv_imgproc -lopencv_legacy -lopencv_ml\
	-lopencv_objdetect -lopencv_photo -lopencv_stitching\
	-lopencv_ts -lopencv_video -lopencv_videostab

all:  clarify  run

clarify: $(OBJECTS)

	$(CC) -o clarify $(OBJECTS) $(LIBS) 

%.o : %.cpp 
	$(CC) $(CFLAGS) -c $<

clean:
	-rm -f *.o
run:
	./astro_guider &
