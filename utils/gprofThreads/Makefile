CTC_DIR ?= /home/robocup/2010/ctc-robocup-1.6.0
ifeq "$(origin CC)" "default"
  CC = $(CTC_DIR)/cross/geode/bin/i586-linux-gcc
endif
CFLAGS ?= --sysroot $(CTC_DIR)/staging/geode-linux
LDFLAGS ?= -shared -fPIC -lpthread -ldl --sysroot $(CTC_DIR)/staging/geode-linux
gprofThreads.so: gprof-helper.o
	$(CC) $(LDFLAGS) $^ -o $@
