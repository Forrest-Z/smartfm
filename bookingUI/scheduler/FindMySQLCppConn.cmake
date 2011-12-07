# Try to find the MySQL cpp connector: MySQLCppConn
#
# Once run this will define:
# MySQLCppConn_FOUND       = system has MySQLCppConn lib
# MySQLCppConn_LIBRARIES   = full path to the libraries
# MySQLCppConn_INCLUDE_DIR      = where to find headers


FIND_PATH(MySQLCppConn_INCLUDE_DIR
    NAMES "connection.h" "driver.h" "statement.h"
    PATH_SUFFIXES "cppconn"
)

FIND_LIBRARY(MySQLCppConn_LIBRARY NAMES mysqlcppconn)

IF( MySQLCppConn_INCLUDE_DIR AND MySQLCppConn_LIBRARY )
    SET(MySQLCppConn_FOUND true)
ENDIF()


##====================================================
## Print message
##----------------------------------------------------
IF(NOT MySQLCppConn_FOUND)
    # make FIND_PACKAGE friendly
    IF(NOT MySQLCppConn_FIND_QUIETLY)
        IF(MySQLCppConn_FIND_REQUIRED)
            MESSAGE(FATAL_ERROR "MySQLCppConn required but some headers or libs not found.")
        ELSE(MySQLCppConn_FIND_REQUIRED)
            MESSAGE(STATUS "WARNING: MySQLCppConn was not found.")
        ENDIF(MySQLCppConn_FIND_REQUIRED)
    ENDIF(NOT MySQLCppConn_FIND_QUIETLY)
ENDIF(NOT MySQLCppConn_FOUND)