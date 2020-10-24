idf_component_register(SRCS "ms5611.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    PRIV_REQUIRES freertos
                    )