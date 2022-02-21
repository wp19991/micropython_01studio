# Set location of base MicroPython directory.
if(NOT MICROPY_DIR)
    get_filename_component(MICROPY_DIR ${PROJECT_DIR}/../.. ABSOLUTE)
endif()

# Include core source components.
include(${MICROPY_DIR}/py/py.cmake)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    include(${MICROPY_DIR}/py/usermod.cmake)
    include(${MICROPY_DIR}/extmod/extmod.cmake)
endif()

set(MICROPY_QSTRDEFS_PORT
    ${PROJECT_DIR}/qstrdefsport.h
)

if(MICROPY_PORT_PICLIB STREQUAL "y")
	#add_compile_definitions("MICROPY_PORT_PICLIB")
	#message("\n compile MICROPY_PY_PICLIB \n")
	
	set(MICROPY_INC_PICTURE
    ${PROJECT_DIR}/picture
	)
	set(MICROPY_SOURCE_PICTURE
    ${PROJECT_DIR}/picture/bmp.c
	${PROJECT_DIR}/picture/piclib.c
	${PROJECT_DIR}/picture/tjpgd.c
	)

endif()

# cam
if(MICROPY_PORT_CAMLIB STREQUAL "y")
	set(CAM_SRCS_DIR
	${PROJECT_DIR}/esp32-camera
	)
	set(CAM_SRCS
		${CAM_SRCS_DIR}/driver/esp_camera.c
		${CAM_SRCS_DIR}/driver/cam_hal.c
		${CAM_SRCS_DIR}/driver/sccb.c
		${CAM_SRCS_DIR}/driver/sensor.c
		${CAM_SRCS_DIR}/sensors/ov2640.c
		${CAM_SRCS_DIR}/sensors/ov3660.c
		${CAM_SRCS_DIR}/sensors/ov5640.c
		${CAM_SRCS_DIR}/sensors/ov7725.c
		${CAM_SRCS_DIR}/sensors/ov7670.c
		${CAM_SRCS_DIR}/sensors/nt99141.c
		${CAM_SRCS_DIR}/sensors/gc0308.c
		${CAM_SRCS_DIR}/sensors/gc2145.c
		${CAM_SRCS_DIR}/sensors/gc032a.c
		${CAM_SRCS_DIR}/conversions/yuv.c
		${CAM_SRCS_DIR}/conversions/to_jpg.cpp
		${CAM_SRCS_DIR}/conversions/to_bmp.c
		${CAM_SRCS_DIR}/conversions/jpge.cpp
		${CAM_SRCS_DIR}/conversions/esp_jpg_decode.c
		)

	set(CAM_INC
		${CAM_SRCS_DIR}/conversions/include
		${CAM_SRCS_DIR}/conversions/private_include
		${CAM_SRCS_DIR}/driver/include
		${CAM_SRCS_DIR}/driver/private_include
		${CAM_SRCS_DIR}/sensors/private_include
		${CAM_SRCS_DIR}/target/private_include
		)
		
	if(IDF_TARGET STREQUAL "esp32")
		list(APPEND CAM_SRCS ${CAM_SRCS_DIR}/target/esp32/ll_cam.c)
		list(APPEND CAM_SRCS ${CAM_SRCS_DIR}/target/xclk.c)
	elseif(IDF_TARGET STREQUAL "esp32s2")
		list(APPEND CAM_SRCS ${CAM_SRCS_DIR}/target/esp32s2/ll_cam.c)
		list(APPEND CAM_SRCS ${CAM_SRCS_DIR}/target/xclk.c)
		list(APPEND CAM_SRCS ${CAM_SRCS_DIR}/target/esp32s2/tjpgd.c)
		list(APPEND CAM_INC ${CAM_SRCS_DIR}/target/esp32s2/private_include)
	elseif(IDF_TARGET STREQUAL "esp32s3")
		list(APPEND CAM_SRCS ${CAM_SRCS_DIR}/target/esp32s3/ll_cam.c)
	endif()

endif()
#NES
if(MICROPY_PORT_NESEMU STREQUAL "y")
	set(NESEMU_FILE_DIR
		${PROJECT_DIR}/esp32-nesemu/components
		)
	set(NESEMU_INC
		${NESEMU_FILE_DIR}/nofrendo
		${NESEMU_FILE_DIR}/nofrendo/cpu
		${NESEMU_FILE_DIR}/nofrendo/libsnss
		${NESEMU_FILE_DIR}/nofrendo/nes
		${NESEMU_FILE_DIR}/nofrendo/sndhrdw
		${NESEMU_FILE_DIR}/nofrendo-esp32
		)
	file (GLOB_RECURSE NESEMU_SRCS "${NESEMU_FILE_DIR}/*.c") 
	# list(REMOVE_ITEM NESEMU_SRCS "${NESEMU_FILE_DIR}/main/main.c")
endif()

#uvc cam
if(MICROPY_PORT_USB_CAM STREQUAL "y")
set(UVC_CAM_SRCS
		${PROJECT_DIR}/uvc-stream/uvc_stream.c
		${PROJECT_DIR}/uvc-stream/usb_uvc_port.c
		${PROJECT_DIR}/uvc-stream/usb/hcd.c
	)
set(UVC_CAM_INC
		${PROJECT_DIR}/uvc-stream/include
		${PROJECT_DIR}/uvc-stream/usb/private_include
	)
#jpeg-9a lib
set(JPEG9A_DIR
    ${PROJECT_DIR}/uvc-stream/decoder_ijg
	)
set(JPEG9A_SOURCE_LIB
	${JPEG9A_DIR}/jpeg-9a/jaricom.c
	${JPEG9A_DIR}/jpeg-9a/jcomapi.c
	${JPEG9A_DIR}/jpeg-9a/jutils.c
	${JPEG9A_DIR}/jpeg-9a/jerror.c
	${JPEG9A_DIR}/jpeg-9a/jmemmgr.c
	${JPEG9A_DIR}/jpeg-9a/jdapimin.c
	${JPEG9A_DIR}/jpeg-9a/jdapistd.c
	${JPEG9A_DIR}/jpeg-9a/jdarith.c
	${JPEG9A_DIR}/jpeg-9a/jdmaster.c
	${JPEG9A_DIR}/jpeg-9a/jdinput.c
	${JPEG9A_DIR}/jpeg-9a/jdmarker.c
	${JPEG9A_DIR}/jpeg-9a/jdhuff.c
	${JPEG9A_DIR}/jpeg-9a/jdmainct.c
	${JPEG9A_DIR}/jpeg-9a/jdcoefct.c
	${JPEG9A_DIR}/jpeg-9a/jddctmgr.c
	${JPEG9A_DIR}/jpeg-9a/jdpostct.c
	${JPEG9A_DIR}/jpeg-9a/jdsample.c
	${JPEG9A_DIR}/jpeg-9a/jdcolor.c
	${JPEG9A_DIR}/jpeg-9a/jquant2.c
	${JPEG9A_DIR}/jpeg-9a/jquant1.c
	${JPEG9A_DIR}/jpeg-9a/jdmerge.c
	${JPEG9A_DIR}/jpeg-9a/jmemnobs.c
	${JPEG9A_DIR}/jpeg-9a/wrppm.c
	${JPEG9A_DIR}/jpeg-9a/jidctint.c
	${JPEG9A_DIR}/jpeg-9a/jidctflt.c
	${JPEG9A_DIR}/jpeg-9a/jidctfst.c
	${JPEG9A_DIR}/jpegd2.c
	)
	set(JPEG9A_SOURCE_INC
	${JPEG9A_DIR}/include
	)
endif()
#web stream
if(MICROPY_PORT_WEB_STREAM STREQUAL "y")
	set(WEB_SRCS
			${PROJECT_DIR}/http_stream.c
		)
		
	set(WWW_EMBED_FILES
			${PROJECT_DIR}/www/index_uvc.html.gz
		)
endif()
#-----------------------------------------------------------

set(MICROPY_SOURCE_SHARED
    ${MICROPY_DIR}/shared/readline/readline.c
    ${MICROPY_DIR}/shared/netutils/netutils.c
    ${MICROPY_DIR}/shared/timeutils/timeutils.c
    ${MICROPY_DIR}/shared/runtime/interrupt_char.c
    ${MICROPY_DIR}/shared/runtime/stdout_helpers.c
    ${MICROPY_DIR}/shared/runtime/sys_stdio_mphal.c
    ${MICROPY_DIR}/shared/runtime/pyexec.c
)

set(MICROPY_SOURCE_LIB
    ${MICROPY_DIR}/lib/littlefs/lfs1.c
    ${MICROPY_DIR}/lib/littlefs/lfs1_util.c
    ${MICROPY_DIR}/lib/littlefs/lfs2.c
    ${MICROPY_DIR}/lib/littlefs/lfs2_util.c
    ${MICROPY_DIR}/lib/mbedtls_errors/mp_mbedtls_errors.c
    ${MICROPY_DIR}/lib/oofatfs/ff.c
    ${MICROPY_DIR}/lib/oofatfs/ffunicode.c
)
if(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND MICROPY_SOURCE_LIB ${MICROPY_DIR}/shared/runtime/gchelper_generic.c)
endif()

set(MICROPY_SOURCE_DRIVERS
    ${MICROPY_DIR}/drivers/bus/softspi.c
    ${MICROPY_DIR}/drivers/dht/dht.c
)

set(MICROPY_SOURCE_PORT
    ${PROJECT_DIR}/main.c
    ${PROJECT_DIR}/uart.c
    ${PROJECT_DIR}/usb.c
    ${PROJECT_DIR}/usb_serial_jtag.c
    ${PROJECT_DIR}/gccollect.c
    ${PROJECT_DIR}/mphalport.c
    ${PROJECT_DIR}/fatfs_port.c
    ${PROJECT_DIR}/help.c
    ${PROJECT_DIR}/modutime.c
    ${PROJECT_DIR}/moduos.c
    ${PROJECT_DIR}/machine_bitstream.c
    ${PROJECT_DIR}/machine_timer.c
    ${PROJECT_DIR}/machine_pin.c
    ${PROJECT_DIR}/machine_touchpad.c
    ${PROJECT_DIR}/machine_adc.c
    ${PROJECT_DIR}/machine_dac.c
    ${PROJECT_DIR}/machine_i2c.c
    ${PROJECT_DIR}/machine_i2s.c
    ${PROJECT_DIR}/machine_uart.c
    ${PROJECT_DIR}/modmachine.c
    ${PROJECT_DIR}/modnetwork.c
    ${PROJECT_DIR}/network_lan.c
    ${PROJECT_DIR}/network_ppp.c
    ${PROJECT_DIR}/network_wlan.c
    ${PROJECT_DIR}/mpnimbleport.c
    ${PROJECT_DIR}/modsocket.c
    ${PROJECT_DIR}/modesp.c
    ${PROJECT_DIR}/esp32_nvs.c
    ${PROJECT_DIR}/esp32_partition.c
    ${PROJECT_DIR}/esp32_rmt.c
    ${PROJECT_DIR}/esp32_ulp.c
    ${PROJECT_DIR}/modesp32.c
    ${PROJECT_DIR}/machine_hw_spi.c
    ${PROJECT_DIR}/machine_wdt.c
    ${PROJECT_DIR}/mpthreadport.c
    ${PROJECT_DIR}/machine_rtc.c
    ${PROJECT_DIR}/machine_sdcard.c
	${PROJECT_DIR}/modtftlcd.c
	${PROJECT_DIR}/ILI9341.c
	${PROJECT_DIR}/ST7789.c
	${PROJECT_DIR}/ST7735.c
	${PROJECT_DIR}/modtouch.c
	${PROJECT_DIR}/xpt2046.c
	${PROJECT_DIR}/modgui.c
	${PROJECT_DIR}/gui_button.c
	${PROJECT_DIR}/modsensor.c
	${PROJECT_DIR}/modov2640.c
	${PROJECT_DIR}/modespusb.c
	${PROJECT_DIR}/modusbcam.c
	${PROJECT_DIR}/lcd_spibus.c
	${PROJECT_DIR}/modgame.c
	${PROJECT_DIR}/modnes.c
	${PROJECT_DIR}/psxcontroller.c
)

set(MICROPY_SOURCE_QSTR
    ${MICROPY_SOURCE_PY}
    ${MICROPY_SOURCE_EXTMOD}
    ${MICROPY_SOURCE_USERMOD}
    ${MICROPY_SOURCE_SHARED}
    ${MICROPY_SOURCE_LIB}
    ${MICROPY_SOURCE_PORT}
    ${MICROPY_SOURCE_BOARD}
)

set(IDF_COMPONENTS
	esp_http_server
    app_update
    bootloader_support
    bt
    driver
	esp_adc_cal
    esp_common
    esp_eth
    esp_event
    esp_ringbuf
    esp_rom
    esp_wifi
    freertos
    heap
    log
    lwip
    mbedtls
    mdns
    newlib
    nvs_flash
    sdmmc
    soc
    spi_flash
    tcpip_adapter
    ulp
    vfs
    xtensa
)

if(IDF_VERSION_MINOR GREATER_EQUAL 1 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND IDF_COMPONENTS esp_netif)
endif()

if(IDF_VERSION_MINOR GREATER_EQUAL 2 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND IDF_COMPONENTS esp_system)
    list(APPEND IDF_COMPONENTS esp_timer)
endif()

if(IDF_VERSION_MINOR GREATER_EQUAL 3 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    list(APPEND IDF_COMPONENTS esp_hw_support)
    list(APPEND IDF_COMPONENTS esp_pm)
    list(APPEND IDF_COMPONENTS hal)
endif()

if(IDF_TARGET STREQUAL "esp32")
    list(APPEND IDF_COMPONENTS esp32)
elseif(IDF_TARGET STREQUAL "esp32c3")
    list(APPEND IDF_COMPONENTS esp32c3)
    list(APPEND IDF_COMPONENTS riscv)
	list(APPEND IDF_COMPONENTS usb)
elseif(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND IDF_COMPONENTS esp32s2)
    list(APPEND IDF_COMPONENTS tinyusb)
	list(APPEND IDF_COMPONENTS usb)
elseif(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND IDF_COMPONENTS esp32s3)
    list(APPEND IDF_COMPONENTS tinyusb)
	# list(APPEND IDF_COMPONENTS usb)
endif()

# Register the main IDF component.
idf_component_register(
    SRCS
        ${MICROPY_SOURCE_PY}
        ${MICROPY_SOURCE_EXTMOD}
        ${MICROPY_SOURCE_SHARED}
        ${MICROPY_SOURCE_LIB}
        ${MICROPY_SOURCE_DRIVERS}
        ${MICROPY_SOURCE_PORT}
		${MICROPY_SOURCE_BOARD}
		${MICROPY_SOURCE_PICTURE}
		${CAM_SRCS}
		${WEB_SRCS}
		${UVC_CAM_SRCS}
		${JPEG9A_SOURCE_LIB}
		${NESEMU_SRCS}
    INCLUDE_DIRS
        ${MICROPY_INC_CORE}
        ${MICROPY_INC_USERMOD}
        ${MICROPY_PORT_DIR}
        ${MICROPY_BOARD_DIR}
        ${CMAKE_BINARY_DIR}
		${MICROPY_INC_PICTURE}
		${CAM_INC}
		${WEB_SRCS_INC}
		${UVC_CAM_INC}
		${JPEG9A_SOURCE_INC}
		${NESEMU_INC}
    REQUIRES
        ${IDF_COMPONENTS}
	EMBED_FILES
		${WWW_EMBED_FILES}
)

# Set the MicroPython target as the current (main) IDF component target.
set(MICROPY_TARGET ${COMPONENT_TARGET})

# Define mpy-cross flags, for use with frozen code.
set(MICROPY_CROSS_FLAGS -march=xtensawin)

# Set compile options for this port.
target_compile_definitions(${MICROPY_TARGET} PUBLIC
    ${MICROPY_DEF_CORE}
    MICROPY_ESP_IDF_4=1
    MICROPY_VFS_FAT=1
    MICROPY_VFS_LFS2=1
    FFCONF_H=\"${MICROPY_OOFATFS_DIR}/ffconf.h\"
    LFS1_NO_MALLOC LFS1_NO_DEBUG LFS1_NO_WARN LFS1_NO_ERROR LFS1_NO_ASSERT
    LFS2_NO_MALLOC LFS2_NO_DEBUG LFS2_NO_WARN LFS2_NO_ERROR LFS2_NO_ASSERT
)

# Disable some warnings to keep the build output clean.
target_compile_options(${MICROPY_TARGET} PUBLIC
    -Wno-clobbered
    -Wno-deprecated-declarations
    -Wno-missing-field-initializers
	-Wno-stringop-truncation
	-Wno-implicit-fallthrough
)

# Add additional extmod and usermod components.
target_link_libraries(${MICROPY_TARGET} micropy_extmod_btree)
target_link_libraries(${MICROPY_TARGET} usermod)


# Collect all of the include directories and compile definitions for the IDF components.
foreach(comp ${IDF_COMPONENTS})
    micropy_gather_target_properties(__idf_${comp})
endforeach()

if(IDF_VERSION_MINOR GREATER_EQUAL 2 OR IDF_VERSION_MAJOR GREATER_EQUAL 5)
    # These paths cannot currently be found by the IDF_COMPONENTS search loop above,
    # so add them explicitly.
    list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/soc/soc/${IDF_TARGET}/include)
    list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/soc/soc/include)
    if(IDF_VERSION_MINOR GREATER_EQUAL 3)
	list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/tinyusb/additions/include)
	list(APPEND MICROPY_CPP_INC_EXTRA ${IDF_PATH}/components/tinyusb/tinyusb/src)
    endif()
endif()

# Include the main MicroPython cmake rules.
include(${MICROPY_DIR}/py/mkrules.cmake)
