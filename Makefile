#----------------------------------------------------------------------------------------
# Name: Makefile
# Author(s): Wimroy D'souza, Hrushikesh Kulkarni
# Comments: make a 'src' folder. It will compile all the files from the 'src' folder,
#			all the compiled object files will be placed in the obj folder
# Excecutable: The executable will be created in the folder containing the makefile
#
# NOTES: Don't forget to add OpenCV, FFMpeg in the LD_LIBRARY_PATH, in your .cshrc	
#----------------------------------------------------------------------------------------

CC = g++
CFLAGS = -c

WFLAGS = -Wall  -Warray-bounds

OPTS = -O1
OFLAGS = -g

#LFLAGS = -Lgui -lcpptk -ltcl8.5 -ltk8.5
LFLAGS = -llapack -lm

# ENTER YOUR EXECUTABLE NAME HERE 
EXECUTABLE = calculateparameters

#Opencv Includes Go here
OPENCV_PATH = /usr/local/OpenCV
#OPENCV_LIBS = -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ts -lopencv_nonfree -lopencv_video -lfftw3f -lopencv_gpu
OPENCV_LIBS = -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ts -lopencv_video -lfftw3f
#OPENCV_LIBS = -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_highgui -lopencv_imgproc -lopencv_video 
OPENCV_INCPATH =  -I$(OPENCV_PATH)/include -I./include
#-I$(OPENCV_PATH)/include/opencv/
OPENCV_LIBPATH = -L$(OPENCV_PATH)/lib

#Boost Libs Go Here
BOOST_LIBS = -lboost_system  -lboost_filesystem  -lboost_program_options 

#Armadillo Libs go here
ARMADILLO_PATH = /s/chopin/a/grad/jindal/armadillo-4.650.4
ARMADILLO_LIBS = -larmadillo
ARMADILLO_INCPATH = -I$(ARMADILLO_PATH)/usr/include -I./include
ARMADILLO_LIBPATH = -L$(ARMADILLO_PATH)/usr/lib64


CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))


SUBMIT_DIR = $(shell whoami)
BACKUP_DIR = $(shell date "+%b_%d_%Y_%I_%M")
BACKUP_REPO	= ./Backups
BACKUP_PATH = $(BACKUP_REPO)/$(BACKUP_DIR)


all: $(EXECUTABLE)


$(EXECUTABLE): $(OBJ_FILES)
	$(CC) $(NFLAGS) $(PFLAGS) $(WFLAGS) $(OPTS) $(LFLAGS)  $(OPENCV_LIBPATH) $(BOOST_LIBS) $(ARMADILLO_LIBS) $(ARMADILLO_INCPATH) $(ARMADILLO_LIBPATH) $^ -o $@ $(LFLAGS) $(OPENCV_LIBS) $(OPENCV_INCPATH)

#$(EXECUTABLE): $(OBJ_FILES)
#	$(CC) $(WFLAGS) $(OPTS) $(LFLAGS) $(OPENCV_LIBS) $(OPENCV_INCPATH) $(OPENCV_LIBPATH) $(BOOST_LIBS) $^ -o $@   $(LFLAGS)
	
	
obj/%.o: src/%.cpp
	mkdir -p ./obj
	$(CC) $(CFLAGS) $(WFLAGS) $(OPTS) $(OFLAGS) $(OPENCV_LIBS) $(OPENCV_INCPATH) $(OPENCV_LIBPATH) $(BOOST_LIBS) $(ARMADILLO_LIBS) $(ARMADILLO_INCPATH) $(ARMADILLO_LIBPATH) -c -o $@ $<  $(LFLAGS)

#obj/%.o: src/%.cpp
#	mkdir -p ./obj
#	$(CC) $(CFLAGS) $(WFLAGS) $(OPTS) $(OFLAGS) $(OPENCV_LIBS) $(OPENCV_INCPATH) $(OPENCV_LIBPATH) $(BOOST_LIBS) -c -o $@ $<  $(LFLAGS)

clean:
	rm -f $(OBJ_FILES)
	rm -f *.out
	rm -f *~
	rm -f $(EXECUTABLE) 
	
	
#Create a Backup directory with <Month>_<Date>_<Year>_<Hr>_<Min>_<Sec>.tar
backup: 
	mkdir -p $(BACKUP_REPO)
	mkdir -p $(BACKUP_PATH)
	mkdir -p $(BACKUP_PATH)/src
	cp -r ./src/*.h ./$(BACKUP_PATH)/src
	cp -r ./src/*.cpp ./$(BACKUP_PATH)/src
	cp Makefile $(BACKUP_PATH)/
	cp TestScript.sh $(BACKUP_PATH)/
	tar -zcvf $(BACKUP_REPO)/$(BACKUP_DIR).tar $(BACKUP_PATH)/
	rm -rf $(BACKUP_PATH)
