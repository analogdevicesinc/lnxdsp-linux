ifeq ($(CONFIG_MACH_SC594_SOM_EZKIT),y)
zreladdr-y      += 0xA0008000
params_phys-y   := 0xA0000100
else
zreladdr-y	+= 0x80008000
params_phys-y	:= 0x80000100
endif
