EXEC_NAME=JumpingSumoPiloting #output filename

SDK_DIR=../../../../out
IDIR=./
CC=gcc
CFLAGS=-I$(IDIR) -I $(SDK_DIR)/Unix-base/staging/usr/include

OBJDIR=obj
LDIR = $(SDK_DIR)/Unix-base/staging/usr/lib

EXTERNAL_LIB=-lncurses

LIBS=-L$(SDK_DIR)/Unix-base/staging/usr/lib -larsal -larcommands -larnetwork -larnetworkal -lardiscovery $(EXTERNAL_LIB)
LIBS_DBG=-L$(SDK_DIR)/Unix-base/staging/usr/lib -larsal_dbg -larcommands_dbg -larnetwork_dbg -larnetworkal_dbg -lardiscovery_dbg $(EXTERNAL_LIB)

_DEPS = JumpingSumoPiloting.h ihm.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = JumpingSumoPiloting.o ihm.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))

$(OBJDIR)/%.o: %.c $(DEPS)
	@ [ -d $(OBJDIR) ] || mkdir $(OBJDIR)
	@ $(CC) -c -o $@ $< $(CFLAGS)

$(EXEC_NAME): $(OBJ)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	@ rm -f $(OBJDIR)/*.o *~ core $(INCDIR)/*~
	@ rm -rf $(OBJDIR)
	@ rm -r $(EXEC_NAME)

run:
	LD_LIBRARY_PATH=$(LDIR) ./$(EXEC_NAME)
