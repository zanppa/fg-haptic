#!/bin/sh

x86_64-w64-mingw32-gcc \
	fg-haptic.c \
	-I"../SDL2/SDL2-2.0.10/x86_64-w64-mingw32/include" \
	-I"../SDL2/SDL2-2.0.10/x86_64-w64-mingw32/include/SDL2" \
	-I"../SDL2/SDL2_net-2.0.1/x86_64-w64-mingw32/include" \
	-L"../SDL2/SDL2-2.0.10/x86_64-w64-mingw32/lib" \
	-L"../SDL2/SDL2_net-2.0.1/x86_64-w64-mingw32/lib" \
	-lmingw32 \
	-lSDL2main \
	-lSDL2 \
	-lSDL2_net \
	-mconsole \
	-o bin/fg-haptic.exe

i686-w64-mingw32-gcc \
	fg-haptic.c \
	-I"../SDL2/SDL2-2.0.10/i686-w64-mingw32/include" \
	-I"../SDL2/SDL2-2.0.10/i686-w64-mingw32/include/SDL2" \
	-I"../SDL2/SDL2_net-2.0.1/i686-w64-mingw32/include" \
	-L"../SDL2/SDL2-2.0.10/i686-w64-mingw32/lib" \
	-L"../SDL2/SDL2_net-2.0.1/i686-w64-mingw32/lib" \
	-lmingw32 \
	-lSDL2main \
	-lSDL2 \
	-lSDL2_net \
	-mconsole \
	-mwindows \
	-o bin/fg-haptic-w32.exe
