# make ROBOT=sniper

CTC_DIR:=/home/robocup/2010/ctc-robocup-1.6.13/
CC=$(CTC_DIR)/cross/geode/bin/i586-linux-gcc
CFLAGS=-fPIC -g -Wall --sysroot=$(CTC_DIR)/staging/geode-linux/ -Werror
LDLIBS=-ldl -lc
LDFLAGS=--sysroot=$(CTC_DIR)/staging/geode-linux/

run: libioctltrap.so.1.0 naoqi.patch
	rsync -a --partial $^ nao@$(ROBOT).local:
	ssh nao@$(ROBOT).local "grep ioctl /usr/bin/naoqi > /dev/null || su -c 'patch -p0' < naoqi.patch"
	ssh nao@$(ROBOT).local /etc/init.d/naoqi stop
	ssh nao@$(ROBOT).local naoqi --ioctltrap &
	sleep 10
	ssh nao@$(ROBOT).local tail -f /tmp/ioctltrap.txt -n 20
libioctltrap.so: libioctltrap.so.1
	ln -s $< $@
libioctltrap.so.1: libioctltrap.so.1.0
	ln -s $< $@
libioctltrap.so.1.0: ioctltrap.o
	$(CC) -shared -Wl,-soname,libioctltrap.so.1 -o $@ $(LDLIBS) $< $(LDFLAGS)
