#ifndef DFCACHE_H
#define DFCACHE_H

#include "DFInitialParams.h"

std::string GetDressFromCache(const DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams morph_params, DFMannequin::MorphParams& cached_morph_params, const std::string name, bool & bCached, std::ofstream& log);


#endif // !DFCACHE_H
