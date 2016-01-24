#include "Log.h"

Log::Log()
    : p_log_out(0), 
    log_owner(false)
{
    Init(0);
}

Log::~Log()
{
    Close();
}

void Log::Init(std::ostream *plog)
{
    Close();
    p_log_out = plog;
    log_owner = false;
}

void Log::Set(std::ostream *plog, std::ofstream::openmode _Mode)
{
    (void)_Mode;
    Close();
    p_log_out = plog;
}

void Log::Set(Log& log, std::ofstream::openmode _Mode)
{
    (void)_Mode;
    Close();
    p_log_out = log.p_log_out;
    log_owner = false;
}

void Log::Set(const std::string &fn, std::ofstream::openmode _Mode)
{
    Close();
    p_log_out = new std::ofstream;
    dynamic_cast<std::ofstream*>(p_log_out)->open(fn.c_str(), _Mode);
    log_owner = p_log_out != 0;
}

void Log::operator=(const Log &a)
{
    Close();
    p_log_out = a.p_log_out;
    //log_owner = a.log_owner;
}

void Log::Close()
{
    if (log_owner && p_log_out) {
        dynamic_cast<std::ofstream*>(p_log_out)->close();
        delete p_log_out;
    }
    p_log_out = 0;
    log_owner = false;
}

size_t Log::GetMemoryUsage() const
{
    return sizeof(*this);
}
