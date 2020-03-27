add_definitions(-DSQLITE_THREADSAFE=1)

add_library(${PROJECT_NAME} SHARED
	camera.cpp
	utils.cpp
	matcher.cpp
	frame.cpp
	database.cpp
	initializer.cpp
	track.cpp
	mappoint.cpp
	map.cpp
	optimizer.cpp
	reconstruction.cpp
	triangulator.cpp
	config.cpp
	pnpSolver.cpp
	sqlite3.c
		)

target_link_libraries(${PROJECT_NAME} ${THIRD_PARTY_LIBS})
