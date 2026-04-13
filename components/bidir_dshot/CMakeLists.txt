idf_component_register(
    SRCS "bidir_dshot.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_driver_gpio esp_driver_rmt esp_hw_support hal soc esp_rom
)
