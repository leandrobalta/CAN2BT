idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver mcp2515 nvs_flash
)
