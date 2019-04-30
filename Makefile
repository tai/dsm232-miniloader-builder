
TARGET = spi.bin

RSAKEY = rsakey.pem
RSAPUB = $(RSAKEY:.pem=.pub)

# NOTE: copy miniloader binary bundled in D-Link archive
ML_BIN = miniloader.bin
CL_BIN = coreloader.bin
CL_SIG = $(CL_BIN:.bin=.sig)

NV_TXT = nvram.txt
NV_BIN = $(NV_TXT:.txt=.bin)

SP_YML = second-param.yaml
SP_BIN = $(SP_YML:.yaml=.bin)
SP_SHA = $(SP_BIN:.bin=.sha)
SP_SIG = $(SP_BIN:.bin=.sig)

APP_BIN = app.bin
APP_SIG = app.sig

BINS = $(CL_BIN) $(SP_BIN) $(SP_SHA) $(SP_SIG) $(NV_BIN) $(APP_SIG)
OBJS = $(CL_BIN).o $(SP_BIN).o $(SP_SIG).o $(NV_BIN).o \
	$(APP_BIN).o $(APP_SIG).o

CROSS = arm-none-linux-gnueabi-
OBJDUMP = $(CROSS)objdump

######################################################################

%.mod: %.pem
	openssl rsa -in $< -noout -modulus | \
	perl -pe 's/^Modulus=//; s/(..)/chr(hex($$1))/ge' | \
	dd bs=256 count=1 > $@

%.pub: %.pem
	openssl rsa -in $< -pubout > $@

%.pub-der: %.pem
	openssl rsa -in $< -pubout -outform DER > $@

%.bin: %.yaml
	dsm232-secondparam $< > $@

%.sig: %.bin
	openssl sha1 -sha256 --sign $(RSAKEY) $< > $@

%.sha: %.bin
	openssl sha1 -sha256 -binary $< > $@

%.bin.o: %.bin
	ld -r -b binary -o $@ $<

%.sig.o: %.sig
	ld -r -b binary -o $@ $<

%.sha.o: %.sha
	ld -r -b binary -o $@ $<

######################################################################

help:
	@echo "Usage: make (spi|check|sp|dis|clean)"

spi: $(TARGET)

sp: $(TARGET)
	@echo === second-param-0 @ 0x0204 ===
	bbe -s -b $$((0x0204)):252 $< | dsm232-secondparam -d /dev/stdin
	@echo === second-param-1 @ 0xb500 ===
	bbe -s -b $$((0xb500)):256 $< | dsm232-secondparam -d /dev/stdin

dis: $(TARGET)
	$(OBJDUMP) -marm -bbinary -D $<

check: $(TARGET) $(RSAPUB)
# verify second-param-1
	bbe -s -b $$((0xb500)):256 $(TARGET) > sp-tmp.bin
	bbe -s -b $$((0xb600)):256 $(TARGET) > sp-tmp.sig
	openssl sha1 -sha256 \
	-signature sp-tmp.sig -verify $(RSAPUB) sp-tmp.bin
	rm sp-tmp.*
# verify SBL
	bbe -s -b $$((0xb800)):$$((0x300)) $(TARGET) > app-tmp.bin
	bbe -s -b $$((0xbb00)):$$((0x100)) $(TARGET) > app-tmp.sig
	openssl sha1 -sha256 \
	-signature app-tmp.sig -verify $(RSAPUB) app-tmp.bin
	rm app-tmp.*

clean:
	$(RM) $(TARGET) $(BINS) *.pub *.sig *.sha *.o *.old *.bak *~

$(TARGET): $(OBJS) image.lds
	ld -b binary -o $@ -T image.lds $(OBJS)

# NOTE: Range 0x204-0xb500 seems to be protected with secure boot
$(CL_BIN): $(ML_BIN)
	dd if=$< of=$@ bs=$$((0xb500)) count=1

# NOTE: Need CRC32 for whole 0x2000 block
$(NV_BIN): $(NV_TXT)
	uboot-nvram -s 0x2000 < $< > $@

# NOTE: Need signature for whole 256B block
$(SP_SIG): $(SP_BIN)
	dd if=/dev/zero of=part1_temp.bin bs=256 count=1
	dd if=$< of=part1_temp.bin conv=notrunc
	openssl sha1 -sha256 -sign $(RSAKEY) < part1_temp.bin > $@
	rm -f part1_temp.bin

# NOTE: Need signature for the whole SBL block
$(APP_SIG): $(APP_BIN)
	dd if=/dev/zero of=app_temp.bin bs=$$((0x300)) count=1
	dd if=$< of=app_temp.bin conv=notrunc
	openssl sha1 -sha256 -sign $(RSAKEY) < app_temp.bin > $@
	rm -f app_temp.bin

#
# NOTE:
# This came from tool/script in D-Link GPL archive, but it seems
# there is no way to let miniloader use only SHA256 for verification.
# So this rule is probably needless.
# 
$(SP_SHA): $(SP_BIN)
	dd if=/dev/zero of=part1_temp.bin bs=256 count=1
	dd if=$< of=part1_temp.bin conv=notrunc
	openssl sha1 -sha256 part1_temp.bin > $@
	rm -f part1_temp.bin

######################################################################

clear-sp0: $(TARGET)
	dd if=/dev/zero bs=256 count=1 | \
	dd of=$< obs=1 conv=notrunc seek=$((0x200))

clear-sp0-sig: $(TARGET)
	dd if=/dev/zero bs=256 count=1 | \
	dd of=$< obs=1 conv=notrunc seek=$((0x300))

clear-sp1: $(TARGET)
	dd if=/dev/zero bs=256 count=1 | \
	dd of=$< obs=1 conv=notrunc seek=$((0xb500))

clear-sp1-sig: $(TARGET)
	dd if=/dev/zero bs=256 count=1 | \
	dd of=$< obs=1 conv=notrunc seek=$((0xb600))

clear-app: $(TARGET)
	dd if=/dev/zero bs=$$((0x300)) count=1 | \
	dd of=$< obs=1 conv=notrunc seek=$((0xb800))

clear-app-sig: $(TARGET)
	dd if=/dev/zero bs=256 count=1 | \
	dd of=$< obs=1 conv=notrunc seek=$((0xbb00))
