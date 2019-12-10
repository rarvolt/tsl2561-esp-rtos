set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_SRCS
        "src/tsl2561.c"
        "src/tsl2561_lux.c")
set(COMPONENT_REQUIRES
        freertos
        esp8266)

register_component()
