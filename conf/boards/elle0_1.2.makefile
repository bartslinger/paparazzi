# Hey Emacs, this is a -*- makefile -*-
#
# elle0_1.2.makefile
#
# http://wiki.paparazziuav.org/wiki/Elle0
#

BOARD=elle0
BOARD_VERSION=1.2
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/elle0.ld

# -----------------------------------------------------------------------

# default flash mode is via usb dfu bootloader (luftboot)
# other possibilities: DFU-UTIL, SWD, JTAG_BMP, STLINK, SERIAL
FLASH_MODE ?= DFU-UTIL

HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8004000
DFU_ADDR = 0x8004000
DFU_PRODUCT = Elle0
endif


#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 3
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1


#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART2
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART5

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
GPS_BAUD ?= B38400


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
