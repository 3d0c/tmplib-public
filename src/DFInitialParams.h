#ifndef DFINITIALPARAMS_H
#define DFINITIALPARAMS_H

#include "DFMannequin.h"

struct ParamMorphtarget
{
    std::string filename;
    DFMannequin::MorphParamType type;
    DF::FloatingPointType val;
};

typedef std::deque<ParamMorphtarget> MorphtargetsDeque;
typedef std::map<DFMannequin::MannequinIDType, MorphtargetsDeque> MorphtargetsDequeStorage;
typedef std::deque<DFMannequin::MorphParams> MorphParamsDeque;
typedef std::map<DFMannequin::MannequinIDType, MorphParamsDeque> MorphParamsStorage;

struct TInitialParameters
{
    TInitialParameters()
        : time_limit_dress(0), time_limit_first_frame(0), time_limit_per_frame(0), time_limit_total(0), count_iteration(0)
    {};

    void Clear()
    {
        path_to_clothes.clear();
        path_to_cache_clothes.clear();
        path_to_mannequins.clear();
        path_to_animation.clear();
        time_limit_dress = 0.f;
        time_limit_first_frame = 0.f;
        time_limit_per_frame = 0.f;
        time_limit_total = 0.f;
        count_iteration = 0;
        morphtargets_storage.clear();
    }

    std::string	path_to_clothes;
    std::string	path_to_cache_clothes;
    std::string	path_to_mannequins;
    std::string	path_to_animation;
    float		time_limit_dress;
    float		time_limit_first_frame;
    float		time_limit_per_frame;
    float		time_limit_total;
    size_t		count_iteration;
    MorphtargetsDequeStorage morphtargets_storage;
    MorphParamsStorage morphparams_storage;
    std::vector<std::string> gressess_filenames;
    std::map<DF::Index, std::string> output_filenames;
};

class DFInitialParams
{
public:
    DFInitialParams(void);
    virtual ~DFInitialParams(void);
    int LoadXML(std::string path);
    void SaveXML(std::string path);
    void Clear();
    TInitialParameters & GetInitialParams();

private:
    TInitialParameters initial_params;
};

#endif // !DFINITIALPARAMS_H
