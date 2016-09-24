SET(LIBSERIAL_SRC serial_port.cpp)
ADD_LIBRARY(serial_port SHARED ${LIBSERIAL_SRC})
ADD_LIBRARY(serial_port_static STATIC ${LIBSERIAL_SRC})
SET_TARGET_PROPERTIES(serial_port_static PROPERTIES OUTPUT_NAME "serial_port")
SET_TARGET_PROPERTIES(serial_port PROPERTIES VERSION 1.0 SOVERSION 1)
INSTALL(TARGETS serial_port serial_port_static
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
INSTALL(FILES serial_port.h DESTINATION include/serial_port)
