g++ readPtsAndMatch.cpp dbgstream.cpp `pkg-config --cflags --libs opencv` -I/usr/include/mysql -lmysqlpp -fopenmp -o readPtsAndMatchStd 

