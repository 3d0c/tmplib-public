#ifndef LOG_H
#define LOG_H

#include <fstream>
#include <iostream>
#include "DFDefinitions.h"
#include "DFPhysicalParams.h"

class Log
{
public:
    Log();
    virtual ~Log();

    void Init(std::ostream *plog);

    void Set(Log& log, std::ofstream::openmode _Mode = std::ofstream::out);
    void Set(std::ostream *plog, std::ofstream::openmode _Mode = std::ofstream::out);
    void Set(const std::string &fn, std::ofstream::openmode _Mode = std::ofstream::out);

    void Close();

    void operator=(const Log &a);

    template<typename T>
    Log& operator<<(const T& st)
    {
        std::cout << st;
        return *this;
    }

    size_t GetMemoryUsage() const;

private:
    std::ostream *p_log_out;
    bool log_owner;
};

#endif  // !LOG_H
