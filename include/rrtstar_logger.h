#ifndef RRTSTAR_LOGGER_H
#define RRTSTAR_LOGGER_H

#include <string>
#include <iostream>
#include <fstream>
#include <mutex>
#include <ctime>
#include <iomanip>

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

class Logger {
private:
    static std::ofstream log_file;
    static std::mutex log_mutex;
    static LogLevel current_level;
    
    static std::string getTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
        return ss.str();
    }
    
    static std::string levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::CRITICAL: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }
    
public:
    static void init(const std::string& filename, LogLevel level = LogLevel::INFO) {
        std::lock_guard<std::mutex> lock(log_mutex);
        current_level = level;
        log_file.open(filename, std::ios::app);
        if (!log_file.is_open()) {
            std::cerr << "Failed to open log file: " << filename << std::endl;
        }
    }
    
    static void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(log_mutex);
        current_level = level;
    }
    
    template<typename... Args>
    static void log(LogLevel level, Args... args) {
        if (level < current_level) return;
        
        std::lock_guard<std::mutex> lock(log_mutex);
        std::stringstream message;
        message << "[" << getTimestamp() << "] [" << levelToString(level) << "] ";
        (message << ... << args);
        
        if (log_file.is_open()) {
            log_file << message.str() << std::endl;
        }
        std::cout << message.str() << std::endl;
    }
    
    template<typename... Args>
    static void debug(Args... args) {
        log(LogLevel::DEBUG, args...);
    }
    
    template<typename... Args>
    static void info(Args... args) {
        log(LogLevel::INFO, args...);
    }
    
    template<typename... Args>
    static void warning(Args... args) {
        log(LogLevel::WARNING, args...);
    }
    
    template<typename... Args>
    static void error(Args... args) {
        log(LogLevel::ERROR, args...);
    }
    
    template<typename... Args>
    static void critical(Args... args) {
        log(LogLevel::CRITICAL, args...);
    }
    
    static void close() {
        std::lock_guard<std::mutex> lock(log_mutex);
        if (log_file.is_open()) {
            log_file.close();
        }
    }
};

std::ofstream Logger::log_file;
std::mutex Logger::log_mutex;
LogLevel Logger::current_level = LogLevel::INFO;

#endif // RRTSTAR_LOGGER_H