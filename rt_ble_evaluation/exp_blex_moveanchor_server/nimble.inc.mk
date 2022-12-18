# For nRF51-based targets, we need to reduce buffer sizes to make this test fit
# into RAM
# Note: as the CPU variable is not set at this point, we manually 'whitelist'
#       all supported nrf51-boards here
BOARDS_NRF51 := airfy-beacon calliope-mini microbit nrf51dk nrf51dongle \
                nrf6310 yunjia-nrf51822
ifneq (,$(filter $(BOARDS_NRF51),$(BOARD)))
  APP_MTU ?= 250
  MSYS_CNT ?= 6
endif

# Set the tests default configuration
APP_MTU ?= 4940
APP_BUF_CHUNKSIZE ?= 247   # must be full divider of APP_MTU
APP_BUF_NUM ?= 6
APP_NODENAME ?= \"nimble_tests\"
APP_CID ?= 0x0235

# Apply configuration values
CFLAGS += -DAPP_MTU=$(APP_MTU)
CFLAGS += -DAPP_BUF_CHUNKSIZE=$(APP_BUF_CHUNKSIZE)
CFLAGS += -DAPP_BUF_NUM=$(APP_BUF_NUM)
CFLAGS += -DAPP_NODENAME=$(APP_NODENAME)
CFLAGS += -DAPP_CID=$(APP_CID)

# configure NimBLE
USEPKG += nimble
MSYS_CNT ?= 40
CFLAGS += -DMYNEWT_VAL_BLE_L2CAP_COC_MAX_NUM=8
CFLAGS += -DMYNEWT_VAL_BLE_MAX_CONNECTIONS=8
CFLAGS += -DMYNEWT_VAL_BLE_GAP_MAX_PENDING_CONN_PARAM_UPDATE=8
CFLAGS += -DMYNEWT_VAL_BLE_L2CAP_COC_MPS=247
CFLAGS += -DMYNEWT_VAL_MSYS_1_BLOCK_COUNT=$(MSYS_CNT)
CFLAGS += -DMYNEWT_VAL_MSYS_1_BLOCK_SIZE=298
CFLAGS += -DMYNEWT_VAL_BLE_LL_CFG_FEAT_DATA_LEN_EXT=1
CFLAGS += -DMYNEWT_VAL_BLE_LL_CFG_FEAT_LL_ENHANCED_CONN_UPDATE=1

INCLUDES += -I$(RIOTBASE)/tests/nimble_l2cap_server/include
CFLAGS += -DTXEND_OUTPUT=1
##CHANGES
CFLAGS += -DREPORT_INTERVAL=500
CFLAGS += -DPKT_LOSS_RATE=0
CFLAGS += -DNODE_NUM=1
CFLAGS += -DC_LEN=0
CFLAGS += -DP_LEN=100
CFLAGS += -DLATENCY=200
CFLAGS += -DPERCENTILE=95
CFLAGS += -DCONTINUOUS_LOSS=0
CFLAGS += -DCONF_LEVEL=6
