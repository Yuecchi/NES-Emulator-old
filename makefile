CC 		= gcc
C_FILES = $(wildcard src/*.c)
O_FILES = $(C_FILES:src/%.c=build/%.o)
OUT		= bin/nes.exe

.DEFAULT: all

all: $(O_FILES)
	$(CC) $^ -o $(OUT)

build/%.o: src/%.c
	$(CC) -c $^ -o $@