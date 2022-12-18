# Configure nrf52dk as default board
BOARD ?= nrf52840dk
WERROR=0

# load the default test environment
include ../Makefile.tests_common

# RIOT modules used in for this test
USEMODULE += xtimer
USEMODULE += bluetil_ad
USEMODULE += blex

APPLICATION = periodic_server

# Get the shared NimBLE and test configuration from the backend server
include $(RIOTBASE)/l2cap_comm/exp_blex_server/nimble.inc.mk

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DLOG_LEVEL=LOG_ALL
CFLAGS += -DDEBUG_ASSERT_VERBOSE
CFLAGS += -DSDBG=1
CFLAGS += -DGET_SCH_ANCHOR=1

##CHANGES
CFLAGS += -DBLEX=1


include $(RIOTBASE)/Makefile.include
