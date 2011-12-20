#ifndef __DEBUG_MACROS_H__
#define __DEBUG_MACROS_H__

#include <stdio.h>

// SchedulerUI displays what is printed on stderr
#ifndef DEBUG_STREAM
#define DEBUG_STREAM stderr
#endif

#ifndef DEBUG_LOGFILE
#define DEBUG_LOGFILE 0
#warning DEBUG_LOGFILE not set. Not logging.
#endif

#ifndef DEBUG_VERBOSITY_VAR
#define DEBUG_VERBOSITY_VAR -1
#warning DEBUG_VERBOSITY_VAR not set. MSG will be quiet.
#endif

#define LOG(fmt, ...) do { \
        if( DEBUG_LOGFILE ) { \
            fprintf(DEBUG_LOGFILE, "%s#%d, time %u, " fmt "\n", \
                    __func__, __LINE__, (unsigned)time(NULL), ##__VA_ARGS__); \
            fflush(DEBUG_LOGFILE); \
        } \
    } while(0)

#define MSG(lvl, fmt, ...) do { \
        if (DEBUG_VERBOSITY_VAR >= lvl) { \
            fprintf(DEBUG_STREAM, "%s#%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            fflush(DEBUG_STREAM); \
            }\
    } while(0)

#define MSGLOG(lvl, fmt, ...) do { MSG(lvl, fmt,##__VA_ARGS__); LOG(fmt,##__VA_ARGS__); } while(0)

#define ERROR(fmt, ...) do { \
        fprintf(DEBUG_STREAM, "ERROR %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        fflush(DEBUG_STREAM); \
        LOG("\nERROR: " fmt "\n", ##__VA_ARGS__); \
    } while(0)


#endif /* __DEBUG_MACROS_H__ */
