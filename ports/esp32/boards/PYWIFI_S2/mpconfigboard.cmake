set(IDF_TARGET esp32s2)

# 配置定义支持图片解码库,y支持，否则不支持
set(MICROPY_PORT_PICLIB y)

set(SDKCONFIG_DEFAULTS
    boards/PYWIFI_S2/sdkconfig.board
		boards/sdkconfig.spiram_sx
)

if(NOT MICROPY_FROZEN_MANIFEST)
    set(MICROPY_FROZEN_MANIFEST ${MICROPY_PORT_DIR}/boards/manifest.py)
endif()
