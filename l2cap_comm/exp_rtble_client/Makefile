# Configure nrf52dk as default board
BOARD ?= nrf52840dk
WERROR=0
# load the default test environment
include ../Makefile.tests_common


# RIOT modules used in for this test
USEMODULE += xtimer
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += bluetil_ad
USEMODULE += printf_float


APPLICATION = periodic_client

# Get the shared NimBLE and test configuration from the backend server
include $(RIOTBASE)/l2cap_comm/exp_rtble_server/nimble.inc.mk

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DLOG_LEVEL=LOG_ALL
CFLAGS += -DDEBUG_ASSERT_VERBOSE
CFLAGS += -DSDBG=1

CFLAGS += -DRTBLE=1
USEMODULE += rt_ble


include $(RIOTBASE)/Makefile.include
