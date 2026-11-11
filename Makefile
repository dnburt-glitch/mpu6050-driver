CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror
SAN     := -fsanitize=address,undefined -fno-omit-frame-pointer -g
INC     := -Iinclude

.PHONY: all test test-sanitize linux-example clean

all: test

build/test_mpu6050: src/mpu6050.c tests/mock_i2c.c tests/test_mpu6050.c include/mpu6050.h tests/mock_i2c.h
	@mkdir -p build
	$(CC) $(CFLAGS) -D_DEFAULT_SOURCE $(INC) -Itests src/mpu6050.c tests/mock_i2c.c tests/test_mpu6050.c -lm -o $@

test: build/test_mpu6050
	./build/test_mpu6050

test-sanitize:
	@mkdir -p build
	$(CC) $(CFLAGS) -D_DEFAULT_SOURCE $(SAN) $(INC) -Itests src/mpu6050.c tests/mock_i2c.c tests/test_mpu6050.c -lm -o build/test_san
	./build/test_san

# Raspberry Pi / any Linux board with i2c-dev: reads the sensor and prints samples.
linux-example: build/mpu6050_linux

build/mpu6050_linux: src/mpu6050.c ports/linux/mpu6050_port_linux.c examples/linux_main.c
	@mkdir -p build
	$(CC) $(CFLAGS) -D_DEFAULT_SOURCE $(INC) -Iports/linux $^ -lm -o $@

clean:
	rm -rf build
