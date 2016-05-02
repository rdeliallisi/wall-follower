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

/**
 * @brief Defines the Logger class
 * which will log what happens in different situations.
 *
 * @details There are three levels of logs: debug, info, error. 
 * These will help checking what happens if we encounter a problem or if 
 * there are any warnings or of there is no problem.
 */

class Logger {
public:
    /**
     * @brief There are more levels of the log:
     */
    /**
     * @brief the debug level
     */
    static const std::string log_level_debug;
    /**
     * @brief the info level
     */
    static const std::string log_level_info;
    /**
     * @brief the error level
     */
    static const std::string log_level_error;
    /**
     * @brief Returns a reference to the singleton Logger object
     */
    static Logger& Instance();
     /**
     * @brief Logs a single message at the given log level
     * @param in_message is the received message
     * @param in_log_level is the level of the log
     */
    void Log(const std::string &in_message,
             const std::string &in_log_level);
     /**
     * @brief Logs a vector of messages at the given log level
     * @param in_messages is the vector with the received messages
     * @param in_log_level is the level of the log
     */
    void Log(const std::vector <std::string> &in_messages,
             const std::string &in_log_level);

protected:
    /**
     * @brief Static variable for the one-and-only instance
     */
    static Logger* p_instance;
    /**
     * @brief Constant for the filename
     */
    static const char* const log_file_name;
     /**
     * @brief Data member for the output stream
     */ 
    std::ofstream output_stream_;
      /**
     * @brief Embedded class to make sure the single Logger
     *        instance gets deleted on program shutdown.
     */ 
    friend class Cleanup;

    /**
     * @brief Defines the Cleanup class that is responsible for removing
     * unnecessary data after the logger is of no use anymore
     * @details Deletes the files and all the material created by the
     * logger
     */
    class Cleanup
    {
    public:
     /**
     * @brief Default destructor
     */ 
        ~Cleanup();
    };
      /**
     * @brief Logs message. The thread should own a lock on sMutex
     *        before calling this function.
     * @param in_message Is the received message
     * @param in_log_level Is the level of the log
     */ 
    void LogHelper(const std::string &in_message,
                   const std::string &in_log_level);

private:
     /**
     * @brief Default constructor
     */ 
    Logger();
    /**
     * @brief Default destructor
     */ 
    virtual ~Logger();
    /**
     * @brief Constructor with arguments
     */ 
    Logger(const Logger&);
    /**
     * @brief Overloading the "=" operator
     */ 
    Logger& operator=(const Logger&);
    /**
     * @brief Mutex for multithreading
     */ 
    static std::mutex s_mutex;
};

