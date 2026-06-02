# PyDrone-cpp/main/CMakeLists.txt
idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash esp_adc esp_timer driver esp_netif esp_event freertos)