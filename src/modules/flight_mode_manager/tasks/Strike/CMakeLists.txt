
px4_add_library(FlightTaskStrike
	FlightTaskStrike.cpp
)

target_link_libraries(FlightTaskStrike PUBLIC FlightTask)
target_include_directories(FlightTaskStrike PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
