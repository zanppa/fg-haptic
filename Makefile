# Makefile to build the SDL tests

srcdir  = .

# Parameters tosupport different library versions
EXE	= 

PKGS	= sdl2 SDL2_net

# Compiler parameters etc
CC      = gcc
CFLAGS  = -g -O2 -Wall -std=c99 $(foreach pkg,$(PKGS),$(shell pkg-config --cflags $(pkg)))
DEFS	= -D_POSIX_C_SOURCE
LIBS	= -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lm $(foreach pkg,$(PKGS),$(shell pkg-config --libs $(pkg)))

TARGETS = \
	fg-haptic$(EXE) \
	test-haptic$(EXE) \
	js-send$(EXE)

all: $(TARGETS)

fg-haptic$(EXE): $(srcdir)/fg-haptic.c
	$(CC) -o $@ $? $(DEFS) $(CFLAGS) $(LIBS)

test-haptic$(EXE): $(srcdir)/tools/test-haptic.c
	$(CC) -o $@ $? $(DEFS) $(CFLAGS) $(LIBS)

js-send$(EXE): $(srcdir)/tools/js-send.c
	$(CC) -o $@ $? $(DEFS) $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS)

distclean: clean
	rm -f Makefile
	rm -f config.status config.cache config.log
	rm -rf $(srcdir)/autom4te*
