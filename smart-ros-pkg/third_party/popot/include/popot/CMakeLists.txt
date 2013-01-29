# Generate the list of source files
# using GLOB does not RECURSE
file(
	GLOB
	source_files
	*
)

file(
	GLOB
	headers_files
	*.h
)

# Define our library 
add_library(
	popot
	SHARED
	${source_files}
)

# Installation targets
install (TARGETS popot
	 DESTINATION ${LIB_INSTALL_DIR})
install (FILES ${headers_files} 
	 DESTINATION ${INCLUDE_INSTALL_DIR})
