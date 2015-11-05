if(DEFINED TARGET_LIKE_GCC)
  file(GLOB_RECURSE objs "gcc/*.o")
endif()

add_library(nxp-rf-driver ${objs})
set_target_properties(nxp-rf-driver PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(nxp-rf-driver
    mbed-client-c
    nanostack-libservice
    sal-stack-nanostack-eventloop
    nanostack-randlib
)
