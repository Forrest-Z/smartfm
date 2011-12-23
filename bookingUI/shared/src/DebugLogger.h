#ifndef __DEBUG_LOGGER_H__
#define __DEBUG_LOGGER_H__

#include <string>
#include <stdio.h>

/// Logging to console and files made easy.
class DebugLogger
{
private:
    FILE *logfile;
    FILE *console_stream;
    int verbosity_lvl;

public:
    DebugLogger();

    void setLogFile(const std::string & filename);
    void setLogFile(FILE *file);
    void setConsoleStream(FILE *stream);
    void setVerbosityLevel(int lvl);
    void copyLoggingSettings(DebugLogger & logger);

    void msglog(int lvl, const char *fmt, ...);

    void close();
};

#define MSGLOG(lvl, fmt, ...) \
    msglog(lvl, "%s#%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);

#define ERROR(fmt, ...) \
    msglog(-1, "ERROR %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);

#define MSGLOG_(logger, lvl, fmt, ...) \
    logger.msglog(lvl, "%s#%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);

#define ERROR_(logger, fmt, ...) \
    logger.msglog(-1, "ERROR %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__);

#endif /* __DEBUG_LOGGER_H__ */
