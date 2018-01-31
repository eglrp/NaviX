add_library(amcl_sensors STATIC
    amcl_sensor.cpp
    amcl_odom.cpp
    amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)
