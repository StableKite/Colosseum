#ifndef MavLinkCom_HighPriorityThread_hpp
#define MavLinkCom_HighPriorityThread_hpp

#include <thread>
#include <string>
#include <chrono>

namespace mavlink_utils
{

class CurrentThread
{
public:
    // make the current thread run with maximum priority.
    static bool setMaximumPriority();

    // set a nice name on the current thread which aids in debugging.
    static bool setThreadName(const std::string& name);

    // cross-platform sleep in milliseconds
    static void sleep_for_ms(unsigned int ms);
};
}

#endif