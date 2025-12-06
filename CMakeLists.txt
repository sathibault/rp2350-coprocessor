if(CONFIG_RP2350_COPROCESSOR)
    add_library(rp2350_coprocessor INTERFACE)
    target_sources(rp2350_coprocessor INTERFACE
        src/rp2350_fifo.c
    )
    target_include_directories(rp2350_coprocessor INTERFACE
        include
    )
    target_link_libraries(rp2350_coprocessor INTERFACE
        zephyr::kernel
        zephyr::drivers
    )

endif()
