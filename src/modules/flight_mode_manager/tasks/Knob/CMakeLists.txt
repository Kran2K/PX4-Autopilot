px4_add_library(FlightTaskKnob
	FlightTaskKnob.cpp
)

target_link_libraries(FlightTaskKnob PUBLIC FlightTask)
target_include_directories(FlightTaskKnob PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
