# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)-or-later

ifeq ($(CONFIG_MACH_SC594_SOM),y)
zreladdr-y      += 0xA2008000
params_phys-y   := 0xA2000100
else
zreladdr-y	+= 0xA0008000
params_phys-y	:= 0xA0000100
endif
