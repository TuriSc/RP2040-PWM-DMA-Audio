if (NOT TARGET pwm_dma_audio)
    add_library(pwm_dma_audio INTERFACE)

    target_sources(pwm_dma_audio INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/audio.c
    )

    target_include_directories(pwm_dma_audio INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(pwm_dma_audio INTERFACE
        pico_stdlib
        hardware_dma
        hardware_pwm
    )
endif()