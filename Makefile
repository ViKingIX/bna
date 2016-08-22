#
# Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
# Copyright (c) 2014 QLogic Corp.
# All rights reserved
# www.qlogic.com
#
# See LICENSE.bna for copyright and licensing details.
#

#GCC_4_1_SP3     := $(shell                                                                      \
#                        GCC_MAJOR=`echo|gcc -dM -E -|grep __GNUC__|cut -d' ' -f3`;              \
#                        GCC_MINOR=`echo|gcc -dM -E -|grep __GNUC_MINOR__|cut -d' ' -f3`;        \
#                        GCC_PATCHLEVEL=`echo|gcc -dM -E -|grep __GNUC_PATCHLEVEL_|cut -d' ' -f3`;\
#                        if [ $$GCC_MAJOR == 4 -a $$GCC_MINOR == 1 ]; then                       \
#                                if [ $$GCC_PATCHLEVEL == 1 -o $$GCC_PATCHLEVEL == 2 ]; then     \
#                                        echo 1;                                                 \
#                                fi                                                              \
#                        fi)

#IS_SUSE_SP3     := $(shell                                                                              \
#                        if [ -f /etc/SuSE-release ]; then                                               \
#                                sles_ver=`cat /etc/SuSE-release |grep "VERSION"|awk '{print $$3}'`;     \
#                                patch_lvl=`cat /etc/SuSE-release |grep "PATCHLEVEL" | awk '{print $$3}'`;\
#                                if [ $$sles_ver == 10 ]; then                                           \
#                                        if [ $$patch_lvl == 2 -o $$patch_lvl == 3 ]; then               \
#                                                echo 1;                                                 \
#                                        fi                                                              \
#                                fi                                                                      \
#                        fi)

BASE_OBJECTS := bnad.o bnad_compat.o bnad_ethtool.o \
		bnad_ioctl.o bnad_aen.o \
		bnad_ioctl_common.o bnad_diag_lb_common.o

BASE_NO_IOCTL := bnad_debugfs.o

cur_kernel := $(shell uname -r)
sub_level := $(subst ., ,$(cur_kernel))
sub_level := $(subst -, ,$(sub_level))
sub_level_1 := $(word 1,$(sub_level))
sub_level_3 := $(word 3,$(sub_level))
debugfs_en_level := 31
debugfs_en := $(shell if [ $(sub_level_3) -ge $(debugfs_en_level) ] || \
		[ $(sub_level_1) -ge 3 ]; then echo 1; fi)

ifeq ($(debugfs_en), 1)
	BASE_OBJECTS += $(BASE_NO_IOCTL)
endif
		
KERNEL_DIR := /lib/modules/$(shell uname -r)/build

BNA_SRC := $(PWD)
include $(BNA_SRC)/bna_linux.make

OBJECT_FILES += $(BASE_OBJECTS) $(BNA_OBJECTS) $(CNA_OBJECTS)

EXTRA_CFLAGS += -I$(BNA_SRC) -I$(BNA_SRC)/include -I$(BNA_SRC)/include/hal \
		-I$(BNA_SRC)/cna/include -I$(BNA_SRC)/cna/include/linux \
		-I$(BNA_SRC)/include/cee -I$(BNA_SRC)/include/cna/pstats \
		-I$(BNA_SRC)/include/cs -I$(BNA_SRC)/include/ioctl  \
		-DCNA_ASSERT_PRINTK_ONLY -DBNAD_MAKE_LRO -DBNAD_GRO_ENABLED

ifeq ($(IS_SUSE_SP3), 1)
        ifeq ($(GCC_4_1_SP3), 1)
                EXTRA_CFLAGS +=
        else
                EXTRA_CFLAGS += -O2
        endif
else
        EXTRA_CFLAGS += -O2
endif

obj-m = bna.o 

bna-objs := $(OBJECT_FILES)

clean_files := \( -name "*.o" -o -name "*.ko" -o -name "*.tgz" -o -name "*.mod.o" \
		-o -name "*.mod.c" -o -name ".*.cmd" -o -name ".*.flags" \
		-o -name " *.rpm" -o -name  ".tmp_versions" \
		-o -name "Module.symvers" \)

all build: 
	$(MAKE) -C $(KERNEL_DIR) M=`pwd` CONFIG_DEBUG_INFO=

cscope:
	@rm -rf cscope.*
	@find . -name "*.[ch]" >cscope.files
	@ctags -L cscope.files
	@cscope -b

clean:
	@find . $(clean_files) | xargs rm -rf
