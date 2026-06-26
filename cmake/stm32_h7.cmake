add_compile_definitions(STM32H7)

if (CMSIS_CORE STREQUAL "CM4")
    add_compile_definitions(CORE_CM4 VIRTIO_SLAVE_ONLY)
    add_compile_options(-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
    add_link_options(-mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)
elseif(CMSIS_CORE STREQUAL "CM7")
    add_compile_definitions(CORE_CM7 VIRTIO_MASTER_ONLY)
    add_compile_options(-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard)
    add_link_options(-mcpu=cortex-m7 --specs=nano.specs  -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb)
endif()

add_compile_definitions(
    STM32H755xx
    USE_HAL_DRIVER
    METAL_INTERNAL
    NO_ATOMIC_64_SUPPORT
    RPMSG_BUFFER_SIZE=512
    METAL_MAX_DEVICE_REGIONS=2
)