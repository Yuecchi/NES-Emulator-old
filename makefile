CC 		= gcc
C_FILES = $(wildcard src/*.c)
O_FILES = $(C_FILES:src/%.c=build/%.o)
OUT		= bin/nes.exe

.DEFAULT: all

all: $(O_FILES)
	$(CC) $^ -o $(OUT)

cpu_test:
	$(CC) test/cpu_test.c src/memory_map.c -o test/test.exe

build/%.o: src/%.c
	$(CC) -c $^ -o $@