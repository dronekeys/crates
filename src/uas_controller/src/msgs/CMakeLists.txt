find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# List of custom messages
set(msgs environment.proto)

# Generate the message source and header files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Compile the message library
add_library(custom_msgs ${PROTO_SRCS})

# Link the message library
target_link_libraries(custom_msgs ${PROTOBUF_LIBRARY})
