# SPDX-License-Identifier: GPL-2.0-or-later

ifeq ($(CONFIG_MACH_SC594_SOM),y)
zreladdr-y      += 0xA0008000
params_phys-y   := 0xA0000100
else
zreladdr-y	+= 0x80008000
params_phys-y	:= 0x80000100
endif
