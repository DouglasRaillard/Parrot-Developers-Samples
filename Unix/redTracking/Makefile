EXEC_NAME=BebopPilotingNewAPI #output filename
CC=g++
SDK_DIR=../../../../out
IDIR=./
CFLAGS= -std=c++11 -Wno-write-strings -g3 -I$(IDIR) -I$(SDK_DIR)/Unix-base/staging/usr/include -I/usr/include/opencv


OBJDIR=obj
LDIR = $(SDK_DIR)/Unix-base/staging/usr/lib

EXTERNAL_LIB=-lncurses -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so /usr/lib/x86_64-linux-gnu/libopencv_contrib.so /usr/lib/x86_64-linux-gnu/libopencv_core.so /usr/lib/x86_64-linux-gnu/libopencv_features2d.so /usr/lib/x86_64-linux-gnu/libopencv_flann.so /usr/lib/x86_64-linux-gnu/libopencv_gpu.so /usr/lib/x86_64-linux-gnu/libopencv_highgui.so /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so /usr/lib/x86_64-linux-gnu/libopencv_legacy.so /usr/lib/x86_64-linux-gnu/libopencv_ml.so /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so /usr/lib/x86_64-linux-gnu/libopencv_ocl.so /usr/lib/x86_64-linux-gnu/libopencv_photo.so /usr/lib/x86_64-linux-gnu/libopencv_stitching.so /usr/lib/x86_64-linux-gnu/libopencv_superres.so /usr/lib/x86_64-linux-gnu/libopencv_ts.so /usr/lib/x86_64-linux-gnu/libopencv_video.so /usr/lib/x86_64-linux-gnu/libopencv_videostab.so

LIBS=-L$(LDIR) -larsal -larcontroller -lardiscovery $(EXTERNAL_LIB) -Wl,-rpath=$(LDIR)
LIBS_DBG=-L$(LDIR) -larsal_dbg -larcontroller_dbg -lardiscovery_dbg $(EXTERNAL_LIB) -Wl,-rpath=$(LDIR)

_DEPS = BebopPiloting.h ihm.h LoggingFieldNames.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = BebopPiloting.o ihm.o redtracking.o Logging.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))

FIFO = video_fifo.h264

all: $(EXEC_NAME)
	@[ -p $(FIFO) ] || mkfifo $(FIFO)

$(OBJDIR)/%.o: %.c $(DEPS)
	@ [ -d $(OBJDIR) ] || mkdir $(OBJDIR)
	@ $(CC) -c -o $@ $< $(CFLAGS)

$(EXEC_NAME): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	@ rm -f $(OBJDIR)/*.o *~ core $(INCDIR)/*~
	@ rm -rf $(OBJDIR)
	@ rm -r $(EXEC_NAME)
	@ rm -f $(FIFO)

run:
	LD_LIBRARY_PATH=$(LDIR) ./$(EXEC_NAME)

valgrind:
	LD_LIBRARY_PATH=$(LDIR) valgrind ./$(EXEC_NAME)
