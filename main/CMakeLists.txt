idf_component_register(SRCS "main.c" "led_strip_rmt_ws2812.c" "led_strip_encoder.c" "FrontPanel.c" "SCI.c" "CircularBuffer.c" "at24c.c" "MotionSensor.c" "ADC.c" "DeepSleep.c"
                    INCLUDE_DIRS "." "include"
                    PRIV_REQUIRES "driver"
                    REQUIRES "esp_adc" "nvs_flash" "bt" 

                    )
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=unused-but-set-variable )
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-label -Wno-sizeof-pointer-memaccess)