PKG_NAME=edbg
PKG_URL=https://github.com/ataradov/edbg
PKG_VERSION=e2cd361327253808f130359e2f23fa50ef44dc63
PKG_LICENSE=BSD-3-Clause

# manually set some RIOT env vars, so this Makefile can be called stand-alone
RIOTBASE ?= $(CURDIR)/../../..
RIOTTOOLS ?= $(CURDIR)/..

PKG_SOURCE_DIR = $(CURDIR)/bin
PKG_BUILD_OUT_OF_SOURCE = 0

include $(RIOTBASE)/pkg/pkg.mk

all: $(CURDIR)/edbg

$(CURDIR)/edbg:
# Start edbg build in a clean environment, so variables set by RIOT's build process
# for cross compiling a specific target platform are reset and edbg can
# be built cleanly for the native platform.
	env -i PATH="$(PATH)" TERM="$(TERM)" "$(MAKE)" -C "$(PKG_BUILD_DIR)"
	mv $(PKG_BUILD_DIR)/edbg .

clean::
	rm -f $(CURDIR)/edbg
