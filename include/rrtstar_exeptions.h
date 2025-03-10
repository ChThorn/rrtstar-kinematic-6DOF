#ifndef RRTSTAR_EXCEPTIONS_H
#define RRTSTAR_EXCEPTIONS_H

#include <stdexcept>
#include <string>
#include <chrono>

// Base exception class for all RRTStar exceptions
class RRTStarException : public std::runtime_error {
public:
    explicit RRTStarException(const std::string& message) 
        : std::runtime_error("RRTStar Error: " + message) {}
};

// Specific exception types
class PlanningTimeoutException : public RRTStarException {
public:
    explicit PlanningTimeoutException(const std::string& message = "Planning timeout exceeded") 
        : RRTStarException(message) {}
};

class InvalidConfigurationException : public RRTStarException {
public:
    explicit InvalidConfigurationException(const std::string& message = "Invalid robot configuration") 
        : RRTStarException(message) {}
};

class CollisionException : public RRTStarException {
public:
    explicit CollisionException(const std::string& message = "Collision detected") 
        : RRTStarException(message) {}
};

class IKFailureException : public RRTStarException {
public:
    explicit IKFailureException(const std::string& message = "Inverse kinematics failed") 
        : RRTStarException(message) {}
};

class NoPathFoundException : public RRTStarException {
public:
    explicit NoPathFoundException(const std::string& message = "No valid path found") 
        : RRTStarException(message) {}
};

class OptimizationFailureException : public RRTStarException {
public:
    explicit OptimizationFailureException(const std::string& message = "Path optimization failed") 
        : RRTStarException(message) {}
};

#endif // RRTSTAR_EXCEPTIONS_H