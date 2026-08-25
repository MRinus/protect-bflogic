CC      ?= gcc
CFLAGS  ?= -O2 -fPIC -Wall
LDLIBS  := -lm

# Path to a checked-out BruteFIR source tree (github.com/atorger/brutefir).
# Only src/bfmod.h is needed at build time -- nothing is linked against
# BruteFIR itself, the module is loaded into BruteFIR's own process at
# runtime via modules_path.
BRUTEFIR_SRC ?=

TARGET := protect.bflogic
SRC    := src/bflogic_protect.c

.PHONY: all clean check-brutefir-src

all: check-brutefir-src $(TARGET)

check-brutefir-src:
	@if [ -z "$(BRUTEFIR_SRC)" ]; then \
		echo "BRUTEFIR_SRC is not set."; \
		echo "Point it at a BruteFIR source checkout, e.g.:"; \
		echo "  make BRUTEFIR_SRC=/path/to/brutefir"; \
		exit 1; \
	fi
	@if [ ! -f "$(BRUTEFIR_SRC)/src/bfmod.h" ]; then \
		echo "$(BRUTEFIR_SRC)/src/bfmod.h not found -- is BRUTEFIR_SRC correct?"; \
		exit 1; \
	fi

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -I$(BRUTEFIR_SRC)/src -shared -o $@ $< $(LDLIBS)

clean:
	rm -f $(TARGET)
