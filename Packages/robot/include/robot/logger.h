/**
 * @file logger.h
 * @brief Header file for the logger class.
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <mutex>

class Logger {
public:
    static const std::string log_level_debug;
    static const std::string log_level_info;
    static const std::string log_level_error;

    // Returns a reference to the singleton Logger object
    static Logger& Instance();

    // Logs a single message at the given log level
    void Log(const std::string &in_message,
             const std::string &in_log_level);

    // Logs a vector of messages at the given log level
    void Log(const std::vector <std::string> &in_messages,
             const std::string &in_log_level);

protected:
    // Static variable for the one-and-only instance
    static Logger* p_instance;

    // Constant for the filename
    static const char* const log_file_name;

    // Data member for the output stream
    std::ofstream output_stream_;

    // Embedded class to make sure the single Logger
    // instance gets deleted on program shutdown.
    friend class Cleanup;
    class Cleanup
    {
    public:
        ~Cleanup();
    };

    // Logs message. The thread should own a lock on sMutex
    // before calling this function.
    void LogHelper(const std::string &in_message,
                   const std::string &in_log_level);

private:
    Logger();
    virtual ~Logger();
    Logger(const Logger&);
    Logger& operator=(const Logger&);
    static std::mutex s_mutex;
};

