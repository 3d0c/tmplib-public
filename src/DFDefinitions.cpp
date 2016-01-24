#include "DFDefinitions.h"
#include <sstream>
#include <ctime>

const std::string DF::print_error(DF::Error_t err)
{
    std::string ret;
    switch(err)
    {
    case err_ok:
        ret = "ok";
        break;

    case err_file_io_problem:
        ret = "err_file_io_problem";
        break;

    case err_file_structure_problem:
        ret = "err_file_structure_problem";
        break;

    case err_not_enough_elements:
        ret = "err_not_enough_elements";
        break;

    case err_empty_param_name:
        ret = "err_empty_param_name";
        break;

    case err_null_normal_detected:
        ret = "err_null_normal_detected";
        break;

    case err_cant_calc_projection:
        ret = "err_cant_calc_projection";
        break;

    case err_cant_find_nearest:
        ret = "err_cant_find_nearest";
        break;

    case err_illegal_type:
        ret = "err_illegal_type";
        break;

    case err_cant_generate_allocation_map:
        ret = "err_cant_generate_allocation_map";
        break;

    case err_cant_generate_incidence_struct:
        ret = "err_cant_generate_incidence_struct";
        break;

    case err_cant_recalc_normals:
        ret = "err_cant_recalc_normals";
        break;

    case err_binary_import_error:
        ret = "err_binary_input_error";
        break;

    case err_binary_export_error:
        ret = "err_binary_export_error";
        break;

    case err_time_elapsed:
        ret = "err_time_elapsed";
        break;

    case err_exception_unknown:
        ret = "err_exception_unknown";
        break;

    case err_bad_mannequin_pointer:
        ret = "err_bad_mannequin_pointer";
        break;

    case err_unknown:
    default:
        ret = "ERROR: unknown error";
        break;
    }

    return ret;
} 

DF::StringVector DF::split(const std::string &s)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (ss >> item)
        elems.push_back(item);
    return elems;
}

DF::StringVector DF::split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }	

    return elems;
}

std::string DF::GetCurrentTimeString(bool maximize_compatibility)
{
    time_t t = time(0);
    struct tm *now;
    now = localtime(&t);
    std::ostringstream ss;
    if (maximize_compatibility) {
        ss  << (1900 + now->tm_year)
            << (now->tm_mon<9?"0":"") << (now->tm_mon + 1)
            << (now->tm_mday<10?"0":"") << now->tm_mday << '_'
            << (now->tm_hour<10?"0":"") << now->tm_hour
            << (now->tm_min<10?"0":"") << now->tm_min
            << (now->tm_sec<10?"0":"") << now->tm_sec;
    } else {
        ss << (now->tm_mday<10?"0":"") << now->tm_mday << '.' << (now->tm_mon<9?"0":"") << (now->tm_mon + 1) << '.' << (1900 + now->tm_year) << ' ';
        ss << (now->tm_hour<10?"0":"") << now->tm_hour << ':' << (now->tm_min<10?"0":"") << now->tm_min << ':' << (now->tm_sec<10?"0":"") << now->tm_sec;
    }
    return ss.str();
}

std::string DF::ClipExtension(const std::string &fullpath)
{
    size_t last_slash_i = fullpath.find_last_of("/\\");
    if (last_slash_i == std::string::npos)
        last_slash_i = 0;
    size_t last_point_i = fullpath.find_last_of(".");
    if (last_slash_i > last_point_i)
        return fullpath;
    return fullpath.substr(0, last_point_i);
}

std::string DF::GetPath(const std::string &fullpath)
{
    size_t last_slash_i = fullpath.find_last_of("/\\");
    if (last_slash_i == std::string::npos)
        last_slash_i = 0;
    return fullpath.substr(0, last_slash_i);
}

std::string DF::GetFilename(const std::string &fullpath)
{
    size_t last_slash_i = fullpath.find_last_of("/\\");
    if (last_slash_i == std::string::npos)
        last_slash_i = 0;
    return fullpath.substr(last_slash_i + 1);
}

std::string DF::GetExtension(const std::string &fullpath)
{
    size_t last_slash_i = fullpath.find_last_of("/\\");
    if (last_slash_i == std::string::npos)
        last_slash_i = 0;
    size_t last_point_i = fullpath.find_last_of(".");
    if (last_slash_i > last_point_i)
        return std::string(".");
    return fullpath.substr(last_point_i);
}

bool DF::FileExists(const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

