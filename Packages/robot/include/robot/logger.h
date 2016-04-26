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
    /**
     * @brief Returns a reference to the singleton Logger object
     */
    static Logger& Instance();
     /**
     * @brief Logs a single message at the given log level
     */
    void Log(const std::string &in_message,
             const std::string &in_log_level);
     /**
     * @brief Logs a vector of messages at the given log level
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
    class Cleanup
    {
    public:
        ~Cleanup();
    };
      /**
     * @brief Logs message. The thread should own a lock on sMutex
     *        before calling this function.
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

