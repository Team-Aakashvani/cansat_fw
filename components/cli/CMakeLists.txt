idf_component_register(SRCS "src/console.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES nvs_flash config_mgr comms)
