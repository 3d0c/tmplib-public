#include "DFCache.h"

std::string GetDressFromCache(const DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams morph_params, DFMannequin::MorphParams& cached_morph_params, const std::string name, bool & bCached, std::ofstream& log)
{
    bCached = false;
    std::string ret(name);
    std::string path = name;
    DFInitialParams initial_params;
    log << "[" << DF::GetCurrentTimeString() << "] -----------------> Trying to onen XML \"" << std::string("/mnt/storage/dressformer/models/cache/table_size_cloth.xml") << "\"\n";
    log << "[" << DF::GetCurrentTimeString() << "] -----------------> initial_params.LoadXML() returns " << initial_params.LoadXML(std::string("/mnt/storage/dressformer/models/cache/table_size_cloth.xml")) << " code\n";
    log << "[" << DF::GetCurrentTimeString() << "] -----------------> mannequin_ID = \"" << mannequin_ID << "\"\n";
    if (initial_params.GetInitialParams().morphparams_storage[mannequin_ID].size() == 0)
        return ret;

    DFMannequin::MorphParams min_parameter_difference = initial_params.GetInitialParams().morphparams_storage[mannequin_ID].front() - morph_params;
    DFMannequin::MorphParams cur_parameter_difference;
    MorphParamsDeque::iterator it_min_morph_param = initial_params.GetInitialParams().morphparams_storage[mannequin_ID].begin();
    for (MorphParamsDeque::iterator it_cur_morph_param = it_min_morph_param; it_cur_morph_param != initial_params.GetInitialParams().morphparams_storage[mannequin_ID].end(); it_cur_morph_param++)
    {
        cur_parameter_difference = *it_cur_morph_param - morph_params;
        if (cur_parameter_difference < min_parameter_difference)
        {
            min_parameter_difference = cur_parameter_difference;
            it_min_morph_param = it_cur_morph_param;
        }
    }

    ret = initial_params.GetInitialParams().path_to_cache_clothes + std::string("/") + DF::ClipExtension(DF::GetFilename(path)) + std::string("-");
    static const std::string param_names[10] = {"chest", "underchest", "waist", "hips", "height"};
    for (const std::string* p_param_name = param_names; p_param_name->length(); p_param_name++)
        ret += std::to_string(static_cast<unsigned>((*it_min_morph_param)[*p_param_name])) + "_";
//     for (MorphParams::const_iterator cit_cur_param = it_min_morph_param->cbegin(); cit_cur_param != it_min_morph_param->cend(); cit_cur_param++)
//         ret += std::to_string(static_cast<unsigned>(cit_cur_param->second)) + "_";
    ret.erase(ret.end()-1);
    ret += DF::GetExtension(path);

    std::string cur_params_string;
    for (const std::string* p_param_name = param_names; p_param_name->length(); p_param_name++)
        cur_params_string += *p_param_name + "=" + std::to_string(morph_params[*p_param_name]) + " ";
    cur_params_string.erase(cur_params_string.end()-1);
    log << "[" << DF::GetCurrentTimeString() << "] -----------------> Current morph params: \"" << cur_params_string << "\", Characteristic(min_parameter_difference) = " << Characteristic(min_parameter_difference) << "\n";
    log << "[" << DF::GetCurrentTimeString() << "] -----------------> Checking possible cached file name \"" << ret << "\": ";

    if (!DF::FileExists(ret)) {
        ret = name;
        log << "NOT ";
        bCached = false;
    }
    else {
        bCached = true;
        cached_morph_params = *it_min_morph_param;
    }
    log << "found\n";

    return ret;
}

