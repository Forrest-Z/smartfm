#include <stdio.h>
#include <stdarg.h>

#include <string>
#include <cassert>

#include "DebugLogger.h"

using namespace std;


DebugLogger::DebugLogger() :
    logfile(NULL), console_stream(stdout), verbosity_lvl(-1)
{

}

void DebugLogger::setLogFile(const string & filename)
{
    assert(logfile == NULL);
    logfile = fopen(filename.c_str(), "w");
}

void DebugLogger::setLogFile(FILE *file)
{
    assert(logfile == NULL);
    logfile = file;
}

void DebugLogger::setConsoleStream(FILE *stream)
{
    console_stream = stream;
}

void DebugLogger::setVerbosityLevel(int lvl)
{
    verbosity_lvl = lvl;
}

void DebugLogger::copyLoggingSettings(DebugLogger & logger)
{
    setLogFile(logger.logfile);
    setConsoleStream(logger.console_stream);
    setVerbosityLevel(logger.verbosity_lvl);
}

void DebugLogger::close()
{
    fclose(logfile);
}

void DebugLogger::msglog(int lvl, const char *fmt, ...)
{
    va_list ap1, ap2;
    va_start(ap1,fmt);
    va_copy(ap2,ap1);
    if( logfile!=NULL ) {
        vfprintf(logfile, fmt, ap1);
        fflush(logfile);
    }
    if( lvl<=verbosity_lvl ) {
        vfprintf(console_stream, fmt, ap2);
        fflush(console_stream);
    }
    va_end(ap1);
    va_end(ap2);
}
