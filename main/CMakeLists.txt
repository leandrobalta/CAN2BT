idf_component_register(
    SRCS "CAN2BT.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt mcp2515 driver
)

set(CMAKE_CXX_STANDARD 17)
