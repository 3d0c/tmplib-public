#include "DFFitting.h"
#include "DFPoint.h"
#include <iostream>
#include <climits>
#include <cmath>
#include <string.h>
#include <thread>
#include <algorithm>

// #define VERLET_INTEGRATION
const size_t SmallCount = 1024;
const size_t SmallMaxCount = SmallCount*5;

DFFitting::DFFitting()
{
    fitting_deque_.resize(20);
    fitting_deque_.resize(1);
    fitting_deque_names_.push_back("mannequin");
    force_rate_ = 1.5;
    mannequins_storage_pointer_ = 0;
	m_frameTime = 0.033f;
}

DFFitting::~DFFitting()
{
}

DF::Error_t DFFitting::FrameFirst(DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams &morph_params, DFMannequin::MorphParamsArray &cached_morph_params_array, const double dress_time_limit, const double time_limit_total)
{
    if (fitting_deque_.size() == 0) return DF::err_not_enough_elements;
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;

    // log << "[" << DF::GetCurrentTimeString() << "] FrameFirst() started\n";
    clock_t start = clock();
    clock_t dress_time_limit_clocks = static_cast<clock_t>(dress_time_limit * CLOCKS_PER_SEC);
//    clock_t time_limit_total_clocks = static_cast<clock_t>(time_limit_total * CLOCKS_PER_SEC);

    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
    DF::Error_t err = DF::err_ok;
    if (morph_params.size() > 0)
    {
        err = mannequin.Morph(fitting_deque_[0], morph_params);
        fitting_deque_[0].Clear(DFMesh::CLEAR_GROUPS);
        fitting_deque_[0].AddStandartGroup("morph");
        fitting_deque_[0].SetPrecession(4);
    }

    MeshArray cached_morphs(std::max(cached_morph_params_array.size(), fitting_deque_.size()));
    for (size_t i = 0; i < cached_morph_params_array.size() && err == DF::err_ok; i++)
        if (err == DF::err_ok && cached_morph_params_array[i].size() > 0)
            err = mannequin.Morph(cached_morphs[i], cached_morph_params_array[i]);

    for (size_t cur_dress_i = 1; cur_dress_i < fitting_deque_.size() && err == DF::err_ok /*&& time_limit_total_clocks - clock() + start > 0*/; cur_dress_i++)
    {
		DFMassSpringObject& curMesh = fitting_deque_[cur_dress_i];
        std::string fn = std::string("new_dress")+(char(cur_dress_i+'0'));
        log << "[" << DF::GetCurrentTimeString() << "] Putting on started\n";
        clock_t start1 = clock();
        curMesh.GenerateCollisionalLinks();
        if (err == DF::err_ok) err = FrameDressFirst(mannequin_ID, cur_dress_i, static_cast<clock_t>(dress_time_limit_clocks * CalcTimeRate(curMesh.vv_.size()/3)), cached_morphs[cur_dress_i - 1]);
        clock_t stop1 = clock();
        log << "[" << DF::GetCurrentTimeString() 
            << "] Putting on"
            << " (base_time_limit_for_dress=" << dress_time_limit << " sec"
            << ", base_time_limit_total=" << time_limit_total << " sec"
            << ", v:" << curMesh.vv_.size() / 3
            << ", v/s:" << (static_cast<double>(curMesh.vv_.size() / 3) * CLOCKS_PER_SEC / static_cast<double>(stop1 - start1)) // считаем количество точек в секунду
            << ", f:" << curMesh.vf_.size() 
            << ", f/s:" << (static_cast<double>(curMesh.vf_.size()) * CLOCKS_PER_SEC / static_cast<double>(stop1 - start1)) // считаем количество полигонов в секунду
            << ") finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop1 - start1) / CLOCKS_PER_SEC) << " sec)\n";
        if (err == DF::err_ok) err = curMesh.GenerateAllocationMap();
    }

    for (size_t cur_dress_i = 1; cur_dress_i < fitting_deque_.size() && err == DF::err_ok; cur_dress_i++)
    {
        if (err == DF::err_ok) err = fitting_deque_[cur_dress_i].RecalcNormals();
    }
    clock_t stop = clock();
    // log << "[" << DF::GetCurrentTimeString() << "] FrameFirst() finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    // комментим для экономии времени
//     size_t memory_usage = GetMemoryUsage();
//     log << "[" << DF::GetCurrentTimeString() << "] Memory usage: total: " << memory_usage << " Bytes (" << static_cast<double>(memory_usage) / 1048576. << " MBytes); ";
//     memory_usage = mannequin.GetMemoryUsage();
//     log << "base mannequin + morph-targets + morph: " << memory_usage << " Bytes (" << static_cast<double>(memory_usage) / 1048576. << " MBytes)\n";
    return err;
}

DF::Error_t DFFitting::FitAnimated(DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams &morph_params, DFMannequin::MorphParamsArray &cached_morph_params_array, std::string output_path, size_t iterations_count, size_t n, const double time_limit_first_frame, const double time_limit_next_frame, const double time_limit_total)
{
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;

    log << "[" << DF::GetCurrentTimeString() << "] Animated fitting started\n";
    clock_t start = clock();

    DF::Error_t err = FrameFirst(mannequin_ID, morph_params, cached_morph_params_array, time_limit_first_frame, time_limit_first_frame);
    if (err == DF::err_time_elapsed)
        err = DF::err_ok;
    if (err != DF::err_ok)
        return err;
    if (fitting_deque_.size() < 2)
        return DF::err_not_enough_elements;

    for (size_t cur_dress_i = 0; cur_dress_i < fitting_deque_.size() && err == DF::err_ok; cur_dress_i++)
    {
        /* fitting_deque_[cur_dress_i].ExportBIN(output_path + fitting_deque_names_[cur_dress_i] + std::string(".bin"), OUT_VERTICES|OUT_TEXTURES|OUT_NORMALS|OUT_FACES, true);
        */
        fitting_deque_[cur_dress_i].GenerateAllocationMap();
        fitting_deque_[cur_dress_i].RecalcNormals();
    }

    clock_t time_limit_total_clocks = 0;
    for (size_t cur_dress_i = 1; cur_dress_i < fitting_deque_.size(); cur_dress_i++)
        time_limit_total_clocks += static_cast<clock_t>(time_limit_total / (fitting_deque_.size() - 1) * CalcTimeRate(fitting_deque_[cur_dress_i].vv_.size() / 3) * CLOCKS_PER_SEC);

    float step = force_rate_/(iterations_count/10);
    for (size_t k = 1; k <= iterations_count; k++)
    {
        clock_t cur_time_clocks = clock() - start;
        if (time_limit_total_clocks - cur_time_clocks <= 0)
            break;
        FrameNext( step, n, k, /*time_limit_total_clocks,*/ time_limit_next_frame, output_path );
    }

    for (size_t cur_dress_i = 0; cur_dress_i < fitting_deque_.size() && err == DF::err_ok; cur_dress_i++)
        fitting_deque_[cur_dress_i].RecalcNormals();

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Animated fitting finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
//     пока исключаем для экономии времени
//     size_t memory_usage = GetMemoryUsage();
//     log << "[" << DF::GetCurrentTimeString() << "] Memory usage: total: " << memory_usage << " Bytes (" << static_cast<double>(memory_usage) / 1048576. << " MBytes); ";
//     memory_usage = mannequins_[mannequin_ID].GetMemoryUsage();
//     log << "base mannequin + morph-targets + morph: " << memory_usage << " Bytes (" << static_cast<double>(memory_usage) / 1048576. << " MBytes)\n";
    return err;
}

DF::Error_t DFFitting::FrameNext( const float step, size_t n, size_t k, /*clock_t time_limit_clocks,*/ const double time_limit_next_frame, std::string output_path )
{
    clock_t start = clock();
    // log << "[" << DF::GetCurrentTimeString() << "] FrameNext() started\n";
	DF::Error_t err = DF::err_ok;
    std::string file_name_suffix;
    if (n>0 && k%n==0) {
	    log << "k= " << k << " --------------------------- \n";
        // запись кадра
        size_t kk = k / n;
        file_name_suffix.push_back('_');
        file_name_suffix.push_back('0'+ kk/100%10);
        file_name_suffix.push_back('0'+ kk%100/10);
        file_name_suffix.push_back('0'+ kk%10);
        fitting_deque_[0].ExportBIN(output_path + fitting_deque_names_[0] + file_name_suffix + std::string(".bin"), OUT_VERTICES, true);
    }
    if (force_rate_-step >= 0.2)
        force_rate_ -= step;
    else
        force_rate_ = 0.2;
    for (size_t cur_dress_i = 1; cur_dress_i < fitting_deque_.size() && err == DF::err_ok; cur_dress_i++) {
		if (err == DF::err_ok) err = FrameDressNext(cur_dress_i, static_cast<clock_t>(time_limit_next_frame * CalcTimeRate(fitting_deque_[cur_dress_i].vv_.size() / 3) * CLOCKS_PER_SEC));
		//!!!fitting_deque_[cur_dress_i].SmoothMesh(DFMesh::SMOOTH_POST, static_cast<DF::FloatingPointType>(1));
        if (err == DF::err_time_elapsed)
            err = DF::err_ok;
        if (n>0 && k%n==0) {
            fitting_deque_[cur_dress_i].ExportBIN(output_path + fitting_deque_names_[cur_dress_i] + file_name_suffix + std::string(".bin"), OUT_VERTICES, true);
        }
    }
    clock_t stop = clock();
	m_frameTime = static_cast<double>(stop - start) / CLOCKS_PER_SEC;
    // log << "[" << DF::GetCurrentTimeString() << "] FrameNext() finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";

	if( m_frameTime < 0.3 )
		m_frameTime = 0.3;
	if( m_frameTime > 0.9 )
		m_frameTime = 0.9;
	
	m_frameTime = 0.5;

	return err;
}

DF::Error_t DFFitting::FitAnimatedPhys(DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams &morph_params, std::string output_path)
{
    if (fitting_deque_.size() == 0) return DF::err_not_enough_elements;
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;

    log << "[" << DF::GetCurrentTimeString() << "] Animated fitting started\n";
    clock_t start = clock();

    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
    DF::Error_t err = DF::err_ok;
    err = mannequin.Morph(fitting_deque_[0], morph_params);

    //	mannequin_.morph.GenerateAllocationMap();
    //	mannequin_.morph.RecalcNormals();
    fitting_deque_[0] = mannequin.morph_targets()[DFMannequin::base_name].begin()->second;

    // TODO: адаптация под старую библиотечную логику+++
    err = DF::err_ok;

    for (size_t cur_dress_i = 0; cur_dress_i < fitting_deque_.size(); cur_dress_i++)
        fitting_deque_[cur_dress_i].SetTimeStep(static_cast<DF::FloatingPointType>(0.01));

    // сохраняем стартовые модели
//     mannequin_.base_mannequin.RecalcUVWs();
    mannequin.morph_targets()[DFMannequin::base_name].begin()->second.ExportBIN(output_path + fitting_deque_names_[0] + std::string(".bin"));
    for (size_t cur_dress_i = 1; cur_dress_i < fitting_deque_.size() && err == DF::err_ok; cur_dress_i++)
    {
// 		MoveDressToStartPosition(cur_dress_i);
        DFMesh cached_morphs;
        SimpleMoveDressToStartPosition(mannequin_ID, cur_dress_i, cached_morphs);
//         fitting_deque_[cur_dress_i].RecalcUVWs();
        fitting_deque_[cur_dress_i].ExportBIN(output_path + fitting_deque_names_[cur_dress_i] + std::string(".bin"));
        fitting_deque_[cur_dress_i].GenerateAllocationMap();
        fitting_deque_[cur_dress_i].RecalcNormals();
    }

    // анимация морфирования манекена вместе с одеждой
    size_t iterations_count = 100;
    for (size_t k = 1; k <= iterations_count; k++)
    {
        log <<'[' << DF::GetCurrentTimeString() << "] k = " << k << "\n";
        err = mannequin.ApproximateToMorph(fitting_deque_[0], static_cast<DF::FloatingPointType>(k) / iterations_count);
        fitting_deque_[0].GenerateAllocationMap();

        std::string file_name_suffix("_");
        file_name_suffix.push_back('0'+ k/100%10);
        file_name_suffix.push_back('0'+ k%100/10);
        file_name_suffix.push_back('0'+ k%10);

//         for (size_t cur_dress_i = 1; cur_dress_i < fitting_deque_.size() && err == DF::err_ok; cur_dress_i++)
//         {
//             if (err == DF::err_ok) err = fitting_deque_[cur_dress_i].Step(fitting_deque_, cur_dress_i);
            fitting_deque_[0].ExportBIN(output_path + fitting_deque_names_[0] + file_name_suffix + std::string(".bin"), OUT_VERTICES);
//             fitting_deque_[cur_dress_i].ExportBinaryFile(output_path + dresses_file_name + char(cur_dress_i/10+'0') + char(cur_dress_i%10+'0') + file_name_suffix + std::string(".bin"), OUT_VERTICES);
//         }
    }

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Animated fitting finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    size_t memory_usage = GetMemoryUsage();
    log << "[" << DF::GetCurrentTimeString() << "] Memory usage: total: " << memory_usage << " Bytes (" << static_cast<double>(memory_usage) / 1048576. << " MBytes); ";
    memory_usage = mannequin.GetMemoryUsage();
    log << "base mannequin + morph-targets + morph: " << memory_usage << " Bytes (" << static_cast<double>(memory_usage) / 1048576. << " MBytes)\n";
    return err;
}

void DFFitting::Release()
{
    DF::ReleaseSTLContainer(fitting_deque_);
    log << "[" << DF::GetCurrentTimeString() << "] Releasing fitting resources\n";
    log.Close();
}

DF::Error_t DFFitting::AddDress(const std::string &fn)
{
    fitting_deque_.resize(fitting_deque_.size() + 1);
#   if defined _DEBUG || defined _ALL_LOGS
    fitting_deque_.back().log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding dress (" << fn << ") started";
    clock_t start = clock();
    DFMesh::FileType file_type = DFMesh::FILETYPE_UNKNOWN;
    DF::Error_t err = fitting_deque_.back().ImportFile(fn, file_type);
    switch (file_type)
    {
    case DFMesh::FILETYPE_WAVEFRONT: log << " (Wavefront OBJ type detected)\n"; break;
    case DFMesh::FILETYPE_DFBINARY: log << " (DFBinary type detected)\n"; break;
    default: log << " (unknown type detected)\n"; break;
    }
    if (err == DF::err_ok) err = fitting_deque_.back().GenerateAllocationMap();
    if (err == DF::err_ok) err = fitting_deque_.back().MergeCoincidentVertices();
    if (err == DF::err_ok) err = fitting_deque_.back().RecalcNormals();
    if (err == DF::err_ok) err = fitting_deque_.back().GenerateIncidenceStruct(true, false);
    if (err == DF::err_ok) err = fitting_deque_.back().GenerateCollisionalLinks();

//     if (err == DF::err_ok) {
//         int loops_counter = 0;
//         for (size_t ivec = 0; ivec < fitting_deque_.back().collision_links_.size(); ivec++)
//         {
//             for (DF::IncedenceVector::const_iterator irec = fitting_deque_.back().collision_links_[ivec].cbegin(); irec != fitting_deque_.back().collision_links_[ivec].cend(); irec++)
//             {
//                 int iii = irec->index;
//                 for (DF::IncedenceVector::const_iterator linkedrec = fitting_deque_.back().collision_links_[irec->index / 3].cbegin(); linkedrec != fitting_deque_.back().collision_links_[irec->index / 3].cend(); linkedrec++)
//                 {
//                     if (linkedrec->index == ivec*3)
//                     {
//                         loops_counter++;
//                     }
//                 }
//             }
//         }
//         log << "! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! loops_counter = " << loops_counter/2 << "/" << fitting_deque_.back().vv_.size() / 3 << " (" << 100*(loops_counter/2)/(fitting_deque_.back().vv_.size()/3) << "%)\n";
//     }

    if (err == DF::err_ok) fitting_deque_.back().CalcInternalParams();
    if (err == DF::err_ok)
    {
        std::string xmlfn = DF::ClipExtension(fn) + std::string(".xml");
        DF::Error_t err = fitting_deque_.back().LoadPhysParams(xmlfn);
        if (err == DF::err_ok) {
            log << "[" << DF::GetCurrentTimeString() << "] Loaded physical params: ";
            for (size_t i = 0; i < fitting_deque_.back().physical_params_.size(); i++)
                log << i << ": " << fitting_deque_.back().physical_params_[i] << " ";
            log << '\n';
        } else {
            log << "[" << DF::GetCurrentTimeString() << "] Can't open physical description file (" << xmlfn << ")(" << DF::print_error(err) << ")\n";
        }
    }
    fitting_deque_names_.push_back("dress");
    fitting_deque_names_.back().push_back(char((fitting_deque_names_.size()-1)/10+'0'));
    fitting_deque_names_.back().push_back(char((fitting_deque_names_.size()-1)%10+'0'));
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding dress finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFFitting::AddDress(void *objData)
{
    fitting_deque_.resize(fitting_deque_.size() + 1);
#   if defined _DEBUG || defined _ALL_LOGS
    fitting_deque_.back().log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding dress started\n";
    clock_t start = clock();
    DF::Error_t err = fitting_deque_.back().ParseWavefrontBuffer(objData);
    if (err == DF::err_ok) err = fitting_deque_.back().GenerateAllocationMap();
    if (err == DF::err_ok) err = fitting_deque_.back().MergeCoincidentVertices();
    if (err == DF::err_ok) err = fitting_deque_.back().RecalcNormals();
    if (err == DF::err_ok) err = fitting_deque_.back().GenerateIncidenceStruct(true, false);
    if (err == DF::err_ok) err = fitting_deque_.back().GenerateCollisionalLinks();
    if (err == DF::err_ok) fitting_deque_.back().CalcInternalParams();
    fitting_deque_names_.push_back("dress");
    fitting_deque_names_.back().push_back(char((fitting_deque_names_.size()-1)/10+'0'));
    fitting_deque_names_.back().push_back(char((fitting_deque_names_.size()-1)%10+'0'));
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding dress finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

void DFFitting::RemoveDress()
{
	if( fitting_deque_.size() > 1 ) {
		// снимаем только одежду, а манекен fitting_deque_[0] не трогаем
		fitting_deque_.back().Release();
		fitting_deque_.resize(fitting_deque_.size() - 1);
		fitting_deque_names_.back().clear();
		fitting_deque_names_.back().resize(fitting_deque_names_.size() - 1);
	}
}

DF::Error_t DFFitting::AddMorph(const std::string &fn)
{
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph (" << fn << ") started\n";
    clock_t start = clock();
    DF::Error_t err = fitting_deque_[0].ImportOBJ(fn);
    if (err == DF::err_ok) err = fitting_deque_[0].GenerateAllocationMap();
    if (err == DF::err_ok) err = fitting_deque_[0].RecalcNormals();
    if (err == DF::err_ok) err = fitting_deque_[0].GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) fitting_deque_[0].CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFFitting::AddMorph(void *objData)
{
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph started\n";
    clock_t start = clock();
    DF::Error_t err = fitting_deque_[0].ParseWavefrontBuffer(objData);
    if (err == DF::err_ok) err = fitting_deque_[0].GenerateAllocationMap();
    if (err == DF::err_ok) err = fitting_deque_[0].RecalcNormals();
    if (err == DF::err_ok) err = fitting_deque_[0].GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) fitting_deque_[0].CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DFMassSpringObject& DFFitting::GetFittingElement(DF::Index i)
{
    if (fitting_deque_.size()) {
        if (i < 0) i = 0;
        if (i >= fitting_deque_.size()) i = fitting_deque_.size() - 1;
    } else {
        i = 0;
    }

    return fitting_deque_.at(i);
}

DF::Error_t DFFitting::FindStartPoint(DF::Index &start_point_index, const size_t dress_index, DF::StateVector point_status) const
{
    start_point_index = static_cast<DF::Index>(-1);
    const DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
    DF::Index start_point_index_up = static_cast<DF::Index>(-1);
//    DF::Index start_point_index_down = static_cast<DF::Index>(-1);
    DF::FloatingPointType start_up = -300;
//    DF::FloatingPointType start_down = 300;
    for (size_t i = 0; i < cur_mesh.vv_.size(); i += 3)
    {
        if (point_status[i / 3] > 0)
            continue;
        if (cur_mesh.vv_[i+1] > start_up/* && abs(cur_mesh.vv_[i]) < 1*/) {
            start_up = cur_mesh.vv_[i+1];
            start_point_index_up = i;
        }
//        if (cur_mesh.vv_[i+1] < start_down) {
//            start_down = cur_mesh.vv_[i+1];
//            start_point_index_down = i;
//        }
    }
//     start_point_index = (start_down + start_up) / 2. < 100 ? start_point_index_down : start_point_index_up;
    start_point_index = start_point_index_up;
    return start_point_index == static_cast<DF::Index>(-1) ? DF::err_cant_find_nearest : DF::err_ok;
}

DF::Error_t DFFitting::FindStartPoints(DF::IndexVector &start_point_index_vector, const size_t dress_index, DF::StateVector point_status) const
{
    start_point_index_vector.resize(4, static_cast<DF::Index>(-1));
    DF::PointsVector heights(4, static_cast<DF::FloatingPointType>(-100));
    const DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
    for (size_t i = 0; i < cur_mesh.vv_.size(); i += 3)
    {
        if (point_status[i / 3] > 0)
            continue;
        if (heights[0] > cur_mesh.vv_[i+1] && abs(cur_mesh.vn_[i]) < 1e-1 && cur_mesh.vn_[i+2] > 0) {  // vy=max, nx=0, nz>0
            heights[0] = cur_mesh.vv_[i+1];
            start_point_index_vector[0] = i;
        }
        if (heights[1] > cur_mesh.vv_[i+1] && abs(cur_mesh.vn_[i]) < 1e-1 && cur_mesh.vn_[i+2] < 0) {  // vy=max, nx=0, nz<0
            heights[1] = cur_mesh.vv_[i+1];
            start_point_index_vector[1] = i;
        }
        if (heights[2] > cur_mesh.vv_[i+1] && cur_mesh.vn_[i] < 0 && abs(cur_mesh.vn_[i+2]) < 1e-1) {  // vy=max, nx<0, nz=0
            heights[2] = cur_mesh.vv_[i+1];
            start_point_index_vector[2] = i;
        }
        if (heights[3] > cur_mesh.vv_[i+1] && cur_mesh.vn_[i] > 0 && abs(cur_mesh.vn_[i+2]) < 1e-1) {  // vy=max, nx>0, nz=0
            heights[3] = cur_mesh.vv_[i+1];
            start_point_index_vector[3] = i;
        }
    }
    bool found = false;
    for (size_t i = 0; i < start_point_index_vector.size() && !found; i++)
        if (start_point_index_vector[i] != static_cast<DF::Index>(-1))
            found = true;
    return found ? DF::err_ok : DF::err_cant_find_nearest;
}

DF::Error_t DFFitting::FrameDressFirst(DFMannequin::MannequinIDType mannequin_ID, const size_t dress_index, const clock_t time_limit_clocks, const DFMesh& cached_morph)
{
//    clock_t start = clock();
    DF::Error_t ret = DF::err_ok;
    DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
    ret = SimpleMoveDressToStartPosition(mannequin_ID, dress_index, cached_morph);
	if( ret != DF::err_ok )
		// не нашли точку входа
		return ret;

	cur_mesh.count_ = cur_mesh.vv_.size() / 3;
    cur_mesh.maxCount_ = cur_mesh.count_*5;
    if (cur_mesh.indexs_ == 0)
        cur_mesh.indexs_ = new DF::Index[cur_mesh.maxCount_];
    if (cur_mesh.newIndexs_ == 0)
        cur_mesh.newIndexs_ = new DF::Index[cur_mesh.maxCount_];
	if( cur_mesh.m_points == 0 )
		cur_mesh.m_points = new DFPoint[cur_mesh.count_];
    for (size_t cur_point_index = 0; cur_point_index < cur_mesh.count_; cur_point_index++)
		cur_mesh.indexs_[cur_point_index] = cur_point_index;

	size_t newCount = 0;
	DF::FloatingPointType tempForceRate = force_rate_;
	force_rate_ = 0;//koefG; // при первичном одевании сила тяжести не нужна, но необходима для запуска расчёта
	m_frameTime = 0.1;
	FrameMultiThread(50, dress_index, cur_mesh.indexs_, cur_mesh.count_, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, time_limit_clocks, DF::INITIAL);
	force_rate_ = tempForceRate;

	/*
	ProcessDress(dress_index, 5, cur_mesh.indexs_, cur_mesh.count_, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, DF::INITIAL);
    for (int i = 0; i < 200; i++) {
//  		log << "i = " << i << ", count = " << cur_mesh.count_ << ", dt=" << time_limit_clocks - clock() + start << "\n";
		ProcessDress(dress_index, 3, cur_mesh.indexs_, cur_mesh.count_, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, DF::INITIAL);
		ProcessDress(dress_index, 5, cur_mesh.indexs_, cur_mesh.count_, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, DF::INITIAL);
//        if (newCount == 0)
//            break;
		{std::sort(&cur_mesh.newIndexs_[0], &cur_mesh.newIndexs_[newCount]); DF::Index *end = std::unique(&cur_mesh.newIndexs_[0], &cur_mesh.newIndexs_[newCount]); newCount = end - &cur_mesh.newIndexs_[0];}
		log << "SubIter=" << i << " count=" << cur_mesh.count_ << ", newCount=" << newCount << '\n';
		memcpy( cur_mesh.indexs_, cur_mesh.newIndexs_, newCount*sizeof(DF::Index) ); cur_mesh.count_ = newCount; newCount = 0;
    }
	*/
	// восстанавливаем count_
	cur_mesh.count_ = cur_mesh.vv_.size() / 3;
	
	ProcessFloor( dress_index );
    return ret;
}

void DFFitting::Jolt(size_t dress_index)
{
    DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
    DF::PhysicalParam &cur_param = cur_mesh.physical_params_[0];
    for (size_t cur_point_index_3 = 0; cur_point_index_3 < cur_mesh.vv_.size(); cur_point_index_3 += 3)
    {
		DFMath::Vector3 vec_mv(cur_mesh.vn_[cur_point_index_3], cur_mesh.vn_[cur_point_index_3 + 1], cur_mesh.vn_[cur_point_index_3 + 2]);
		vec_mv *= (cur_param.shift() * static_cast<DF::FloatingPointType>(7.0));
        for (size_t i = 0; i < dress_index; i++)
		    cur_mesh.ShakeOutPoint(cur_point_index_3, vec_mv, fitting_deque_[i]/*ingot_*/);
    }
//         DF::Index nearest_point_index = 0;
//         DFMath::Vector3 point(cur_mesh.vv_[cur_point_index_3], cur_mesh.vv_[cur_point_index_3 + 1], cur_mesh.vv_[cur_point_index_3 + 2]);
//         DFMath::Vector3 vec_mv_temp(cur_mesh.vn_[cur_point_index_3], cur_mesh.vn_[cur_point_index_3 + 1], cur_mesh.vn_[cur_point_index_3 + 2]);
//         DFMath::Vector3 vec_mv(static_cast<DF::FloatingPointType>(0));
//         DF::FloatingPointType min_rad_2 = static_cast<DF::FloatingPointType>(-1);
//         DF::Error_t err;
//         do
//         {
//             if ((err = mannequin_.base_mannequin.SimpleFindNearestInEnvironmentWithMinMargin(nearest_point_index, point, min_rad_2, 10000, 2)) != DF::err_ok)
//                 break;
//             vec_mv.set(mannequin_.base_mannequin.vn_[nearest_point_index], mannequin_.base_mannequin.vn_[nearest_point_index + 1], mannequin_.base_mannequin.vn_[nearest_point_index + 2]);
//             min_rad_2 = DFMath::length2(point - DFMath::Vector3(mannequin_.base_mannequin.vv_[nearest_point_index], mannequin_.base_mannequin.vv_[nearest_point_index + 1], mannequin_.base_mannequin.vv_[nearest_point_index + 2]));
//         } while (DFMath::ScalarProduct(vec_mv, vec_mv_temp) < 0.9);
//         if (err != DF::err_ok)
//             vec_mv.set(vec_mv_temp);
//         vec_mv *= (cur_mesh.shift_ * static_cast<DF::FloatingPointType>(29.9));
//         cur_mesh.ShakeOutPoint3(cur_point_index_3, vec_mv, fitting_deque_.begin(), fitting_deque_.begin() + dress_index);
//         cur_mesh.ShakeOutPoint4(cur_point_index_3, vec_mv, ingot);
}

struct ParamProcDT
{
	size_t m_dress_index;
	int m_k;
	const DF::Index* m_indexs;
	size_t m_count;
	DF::FittingMode m_fitting_mode;
	ParamProcDT();
	ParamProcDT( size_t dress_index, int k, size_t count, DF::FittingMode fitting_mode );
};

inline
ParamProcDT::ParamProcDT() :
	m_dress_index( 0 ),
	m_k( 0 ),
	m_indexs( 0 ),
	m_count( 0 ),
	m_fitting_mode( DF::FittingMode::INITIAL )
{
}

inline
ParamProcDT::ParamProcDT( size_t dress_index, int k, size_t count, DF::FittingMode fitting_mode ) :
	m_dress_index( dress_index ),
	m_k( k ),
	m_indexs( 0 ),
	m_count( count ),
	m_fitting_mode( fitting_mode )
{
}

void ProcDT1( DFFitting* p, ParamProcDT* param )
{
	std::chrono::milliseconds sleepTime( 0 );
    std::this_thread::sleep_for( sleepTime );
	p->ProcessDressStep1( param->m_dress_index, param->m_k, param->m_indexs, param->m_count, param->m_fitting_mode );
}

void ProcDT2( DFFitting* p, ParamProcDT* param )
{
	std::chrono::milliseconds sleepTime( 0 );
    std::this_thread::sleep_for( sleepTime );
	p->ProcessDressStep2( param->m_dress_index, param->m_k, param->m_indexs, param->m_count, param->m_fitting_mode );
}

void DFFitting::ProcessDressMultiThread(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount, DF::FittingMode fitting_mode)
// 1. разбиваем весь буфер indexs на части
// 2. каждую часть считаем в отдельном потоке
{
    if( count < 500 )
        // 500  точек - это уже случайные колебания
        return;
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
	if( fitting_mode == DF::INITIAL && maxCount > 0 ) {
		// используется для первичного одевания
		cur_mesh.m_vv0 = cur_mesh.vv_; // сохраняем текущие позиции точек
	}
	const size_t MaxCountThread = 16+1; // максимальное количество потоков - для буфера под них
    size_t CountThread = 16+1;//__min(16+1,MaxCountThread); // на 1 больше, т.к. необходимо ещё посчитать остаток
    if( CountThread > MaxCountThread )
        CountThread = MaxCountThread;
	if( count < 4096 )
		CountThread = 2;
	size_t sizeBuffer = count / (CountThread-1);
	// создаём потоки
	std::thread* threads[MaxCountThread];
	ParamProcDT* params[MaxCountThread];
	// расчёт первого шага - суммарные силы
	for( size_t i=0; i<CountThread; i++ ) {
		// инициализируем параметры функции потока
		size_t size = (i == CountThread-1) ? (count%(CountThread-1)): sizeBuffer;
		ParamProcDT*& par = params[i];
		par = new ParamProcDT( dress_index, k, size, fitting_mode );
		par->m_indexs = indexs+i*sizeBuffer;
		threads[i] = new std::thread( ProcDT1, this, par );
	}
	// ждём завершения всех потоков
	for( size_t i=0; i<CountThread; i++ )
		threads[i]->join();
	for( size_t i=0; i<CountThread; i++ ) {
		delete params[i];
		delete threads[i];
	}
	// расчёт второго шага - перемещение точек
	for( size_t i=0; i<CountThread; i++ ) {
		// инициализируем параметры функции потока
		size_t size = (i == CountThread-1) ? (count%(CountThread-1)): sizeBuffer;
		ParamProcDT*& par = params[i];
		par = new ParamProcDT( dress_index, k, size, fitting_mode );
		par->m_indexs = indexs+i*sizeBuffer;
		threads[i] = new std::thread( ProcDT2, this, par );
	}
	// ждём завершения всех потоков
	for( size_t i=0; i<CountThread; i++ )
		threads[i]->join();
	for( size_t i=0; i<CountThread; i++ ) {
		delete params[i];
		delete threads[i];
	}

//    if( maxCount > 0 ) {
        ProcessDressStep25(dress_index);
//    }

	if( fitting_mode == DF::INITIAL && maxCount > 0 ) {
		// используется для первичного одевания
		// обычно добавляется очень мало новых точек, поэтому при распараллеливании выигрыша по скорости не получается!
		// добавление в расчёт переместившихся точек
		ProcessDressStep3( dress_index, indexs, count, newIndexs, newCount, maxCount );
	}
}

DF::Error_t DFFitting::ProcessDress(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount, DF::FittingMode fitting_mode)
{
    DF::Error_t err = DF::err_ok;
    if( count < 500 )
        // 500  точек - это уже случайные колебания
        return err;
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
	if( fitting_mode == DF::INITIAL && maxCount > 0 ) {
		// используется для первичного одевания
		cur_mesh.m_vv0 = cur_mesh.vv_; // сохраняем текущие позиции точек
	}
	ProcessDressStep1( dress_index, k, indexs, count, fitting_mode );
	ProcessDressStep2( dress_index, k, indexs, count, fitting_mode );
        ProcessDressStep25(dress_index);
	if( fitting_mode == DF::INITIAL && maxCount > 0 ) {
		// используется для первичного одевания
		ProcessDressStep3( dress_index, indexs, count, newIndexs, newCount, maxCount );
	}
	
    return err;
}

void DFFitting::ProcessDressStep1(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::FittingMode fitting_mode)
{
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
    const DF::PhysicalParam& cur_param = cur_mesh.physical_params_[0];
    const DF::FloatingPointType& cur_force_rate = force_rate_;

    for (size_t i = 0; i < count; i++) {
		size_t index = indexs[i];
		DFPoint& point = cur_mesh.m_points[ index ];
		point.SetIndex( index ); // привязываем точку к конкретному индексу
		if (k == 4) {
			// добавляем силу тяжести и прочие внешние силы
			point.AddForce( DFMath::Vector3(0., cur_param.freefall_acelleration()*cur_force_rate, 0.) );
		}
		point.AddForcesFromLinks( cur_mesh, cur_force_rate, fitting_mode );

    }
}

void DFFitting::ProcessDressStep2(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::FittingMode fitting_mode)
{
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
    const DF::PhysicalParam& cur_param = cur_mesh.physical_params_[0];
    const DF::FloatingPointType& shift = cur_param.shift();
    const DF::FloatingPointType cos_lim1 = static_cast<DF::FloatingPointType>(0.0);
    const DF::FloatingPointType cos_lim2 = static_cast<DF::FloatingPointType>(0.33);
    const DF::FloatingPointType dt = m_frameTime;
	int countCalcPoints = 0;
    for (size_t i = 0; i < count; i++) {
		DFPoint& point = cur_mesh.m_points[ indexs[i] ];
		if( fitting_mode == DF::ITERATIONAL && !point.IsMove( dt, static_cast<DF::FloatingPointType>( 0.01 ) ) )
			continue;

		countCalcPoints ++;

		if( k == 5 ) {
//				cur_mesh.MovePointWithFriction(indexs[i] * 3, cur_param, point.GetForce(), fitting_deque_, dress_index, cos_lim1);
			point.MovePoint( cur_mesh, dt, cur_param, fitting_deque_, dress_index, cos_lim1, DFPoint::Neutral, true );
			// перемещаем точку над поверхностью
			DFMath::Vector3 vec_mv( point.GetNormal( cur_mesh ) );
            switch (fitting_mode)
            {
            case DF::INITIAL:
			case DF::ITERATIONAL:
                {
					DFMath::Vector3 point2( point.GetVector3( cur_mesh ) );
                    DFPoint pointPush( point ); // создаём копию точки, чтобы не изменять текущую силу m_force во время выдавливания
					vec_mv *= static_cast<DF::FloatingPointType>(0.5);
					pointPush.SetForce( vec_mv );
					DF::FloatingPointType len1 = pointPush.MovePoint( cur_mesh, 1/*dt*/, cur_param, fitting_deque_, dress_index, cos_lim2, DFPoint::Up, true );
					pointPush.SetForce( -vec_mv );
					DF::FloatingPointType len2 = pointPush.MovePoint( cur_mesh, 1/*dt*/, cur_param, fitting_deque_, dress_index, cos_lim2, DFPoint::Down, true );
					if( len2 > len1 ) {
						// данный код блокирует возникновение ряда артефактов, например, спадание(прорыв) с руки длинных рукавов
                        // обратный ход больше прямого - это неправильно
						point.SetVector3( cur_mesh, point2 );
                    }
					
                }
                break;
//			case DF::ITERATIONAL:
				//vec_mv *= static_cast<DF::FloatingPointType>(shift);
				//vec_mv *= shift - static_cast<DF::FloatingPointType>(0.05);
//				break;
            default:
                break;
            }
			
            // если требуется - выталкиваем
			// данный код выталкивания существенно сглаживает ткань, например, без него уже на втором слое виден эффект "пластинчатости"
			DFMath::Vector3 pointNew( point.GetVector3( cur_mesh ) );
            DF::FloatingPointType scalar_correction = 0;
            DFMath::Vector3 vect_correction(DF::zero);
            DF::Index nearest_point_index = 0;
            DFMath::Vector3 nearest_point(DF::zero);
            for (int dress_i = static_cast<int>(dress_index) - 1; dress_i >= 0; dress_i--) {
                if (fitting_deque_[dress_i].SimpleFindNearestArea(nearest_point_index, nearest_point, vect_correction, pointNew, 1) == DF::err_ok) 
                {
                    DFMath::Vector3 radius_vect(nearest_point - pointNew);
                    scalar_correction = DFMath::ScalarProduct(vect_correction, radius_vect) + shift;
                    if (scalar_correction > 0 && DFMath::length2(radius_vect) > (shift+0.1)*(shift+0.1) && DFMath::ScalarProduct(vec_mv, vect_correction) > 0) {
                        pointNew += vect_correction * scalar_correction * static_cast<DF::FloatingPointType>(0.5);
                        break;
                    }
                } 
            }
			point.SetVector3( cur_mesh, pointNew );
			
		}
		else {
			if( k == 4 ) {
//				cur_mesh.MovePointWithFriction(indexs[i] * 3, cur_param, point.GetForce(), fitting_deque_, dress_index, cos_lim1);
				point.MovePoint( cur_mesh, dt, cur_param, fitting_deque_, dress_index, cos_lim1, DFPoint::Neutral, true );
			}
			else {
				// перемещаем точку без сопротивления поверхности
				point.MovePoint( cur_mesh, dt, cur_param, fitting_deque_, dress_index, cos_lim1, DFPoint::Neutral, false );
			}
		}
    }
//	std::cout << "count= " << count << " countCalcPoints= " << countCalcPoints << "\n";
}


void DFFitting::ProcessDressStep25(size_t dress_index)
{
//!!!    return;
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
    DF::PhysicalParam &cur_param = cur_mesh.physical_params_[0];
    for (size_t i = 0; i < cur_mesh.vv_.size() / 3; i++) {
        DF::IncedenceVector::const_iterator cbegin = cur_mesh.collision_links_[i].cbegin();
        DF::IncedenceVector::const_iterator cend = cur_mesh.collision_links_[i].cend();
        if (cbegin == cend)
            continue;
        DFPoint& cur_point = cur_mesh.m_points[i];
        DFMath::Vector3 cur_normal = cur_point.GetNormal(cur_mesh);
        for (DF::IncedenceVector::const_iterator cur_linked_point_index_cit = cbegin; cur_linked_point_index_cit != cend; cur_linked_point_index_cit++) {
            DFPoint& linked_point = cur_mesh.m_points[cur_linked_point_index_cit->index / 3];
            DFMath::Vector3 vec = linked_point.GetVector3(cur_mesh) - cur_point.GetVector3(cur_mesh);
            DF::FloatingPointType scalar_correction = DFMath::ScalarProduct(vec, cur_normal);
            if ( scalar_correction < 0 && DFMath::length2(vec) < cur_mesh.max_dist_2_ * 0.25 ) {//const
                linked_point.AddForce( cur_normal * 0.05f );
            }
        }
    }
}

void DFFitting::ProcessDressStep3(size_t dress_index, const DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount)
{
    (void)indexs;
    (void)count;
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
    // проверяем переместившиеся точки, добавляем их и связанные с ними
    DF::FloatingPointType D2 = static_cast<DF::FloatingPointType>(0.05);    // пороговая сумма перемещений по координатам
    DF::FloatingPointType R1 = static_cast<DF::FloatingPointType>(0.15);   // пороговое изменение длин ребер
    size_t points_count = cur_mesh.vv_.size() / 3;
    DF::PointsVector &vv1 = cur_mesh.vv_;
    for (size_t cur_point_i = 0, cur_point_i_3 = 0; cur_point_i < points_count; cur_point_i++, cur_point_i_3 += 3)
    {
        DF::FloatingPointType cur_characteristic = fabs(vv1[cur_point_i_3]-cur_mesh.m_vv0[cur_point_i_3]) + fabs(vv1[cur_point_i_3+1]-cur_mesh.m_vv0[cur_point_i_3+1]) + fabs(vv1[cur_point_i_3+2]-cur_mesh.m_vv0[cur_point_i_3+2]);
        if (cur_characteristic > D2) {
            if (newCount+2 >= maxCount)
                break;
            newIndexs[newCount++] = cur_point_i;

            DFMath::Vector3 cur_point(cur_mesh.vv_[cur_point_i * 3], cur_mesh.vv_[cur_point_i * 3 + 1], cur_mesh.vv_[cur_point_i * 3 + 2]);
            DF::IncedenceVector &cur_incedence_vector = cur_mesh.incidence_[cur_point_i];
            for (size_t linked_point_i = 0; linked_point_i < cur_incedence_vector.size(); linked_point_i++) {
                if (newCount+2 >= maxCount)
                    break;
                DF::Index &linked_point_index = cur_incedence_vector[linked_point_i].index;
                DFMath::Vector3 rib(DFMath::Vector3(cur_mesh.vv_[linked_point_index], cur_mesh.vv_[linked_point_index + 1], cur_mesh.vv_[linked_point_index + 2]) - cur_point);
                DF::FloatingPointType L = DFMath::length(rib);
                DF::FloatingPointType delta_rate = L / cur_incedence_vector[linked_point_i].distance_prev;
                if (delta_rate > 1+R1 || delta_rate < 1-R1) {
                    newIndexs[newCount++] = linked_point_index / 3;
                }
            }
        }
    }

}

/*
DF::Error_t DFFitting::FrameNext(size_t dress_index, clock_t time_limit_clocks)
{
    log << "[" << DF::GetCurrentTimeString() << "] FrameNext() started\n";
    clock_t start = clock();

    DF::Error_t err = DF::err_ok;
	DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
#   ifdef VERLET_INTEGRATION
    cur_mesh.StoreState(2);
#   endif

	if( cur_mesh.indexs_ == 0 )
		cur_mesh.indexs_ = new DF::Index[cur_mesh.maxCount_];
    if (cur_mesh.newIndexs_ == 0)
        cur_mesh.newIndexs_ = new DF::Index[cur_mesh.maxCount_];
	size_t newCount = 0;
	int maxIterations = 1; // для анимации необходимо меньше итераций
	for (size_t cur_point_index = 0; cur_point_index < cur_mesh.count_; cur_point_index++)
		cur_mesh.indexs_[cur_point_index] = cur_point_index;
    DF::FloatingPointType characteristic = 1;

	size_t smallCount = 1000;
	if( cur_mesh.pos_ >= cur_mesh.count_ ) {
		cur_mesh.pos_ = 0;
        if (cur_mesh.force_rate_ > 1)
            cur_mesh.force_rate_ -= 0.1;
        else
            cur_mesh.force_rate_ = 1;
    }
	for ( ; cur_mesh.pos_<cur_mesh.count_; cur_mesh.pos_+=smallCount ) {
		if (time_limit_clocks - clock() + start <= 0)
			break;
		DF::Index* inds = cur_mesh.indexs_ + cur_mesh.pos_;
		size_t sCount = std::min<size_t>(smallCount,cur_mesh.count_-cur_mesh.pos_);

//		ProcessDressMultiThread(dress_index, inds, sCount, cur_mesh.newIndexs_, newCount, SmallMaxCount, time_limit_clocks);


		for (int i = 0; i < maxIterations; i++) {
			ProcessDress(dress_index, 4, inds, sCount, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, ITERATIONAL, characteristic);
			// одевание без сопротивления поверхности
			ProcessDress(dress_index, 3, inds, sCount, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, ITERATIONAL, characteristic);
			// одевание с сопротивлением поверхности
			ProcessDress(dress_index, 5, inds, sCount, cur_mesh.newIndexs_, newCount, cur_mesh.maxCount_, ITERATIONAL, characteristic);
			if (newCount == 0)
				break;
#			ifdef VERLET_INTEGRATION
			VerletRemainder(dress_index);
#			endif
			{std::sort(&cur_mesh.newIndexs_[0], &cur_mesh.newIndexs_[newCount]); DF::Index *end = std::unique(&cur_mesh.newIndexs_[0], &cur_mesh.newIndexs_[newCount]); newCount = end - &cur_mesh.newIndexs_[0];}
 			log << "dress_index=" << dress_index << ",\tSubIter=" << i << " pos=" << cur_mesh.pos_ << ",\tcount=" << sCount << " count=" << cur_mesh.count_ << ", newCount=" << newCount << ", force_rate=" << cur_mesh.force_rate_ << '\n';
			size_t sz = std::min<size_t>(newCount,cur_mesh.maxCount_ - cur_mesh.count_);
			if( sz > 0 ) {
				memcpy( cur_mesh.indexs_+cur_mesh.count_, cur_mesh.newIndexs_, sz*sizeof(DF::Index) );
				cur_mesh.count_ += sz;
			}
			newCount = 0;
			if (time_limit_clocks - clock() + start <= 0)
				break;
		}
//		{std::sort(&indexs[pos], &indexs[count]); DF::Index *end = std::unique(&indexs[pos], &indexs[count]); count = end - &indexs[0];}
		{std::sort(&cur_mesh.indexs_[0], &cur_mesh.indexs_[cur_mesh.count_]); DF::Index *end = std::unique(&cur_mesh.indexs_[0], &cur_mesh.indexs_[cur_mesh.count_]); cur_mesh.count_ = end - &cur_mesh.indexs_[0];}

	}
	ProcessFloor( dress_index );

	cur_mesh.GenerateAllocationMap();

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] FrameNext() finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}
*/

DF::Error_t DFFitting::FrameDressNext(size_t dress_index, clock_t time_limit_clocks)
{
    clock_t start = clock();
    DF::Error_t err = DF::err_ok;
	DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
	for (size_t cur_point_index = 0; cur_point_index < cur_mesh.count_; cur_point_index++)
		cur_mesh.indexs_[cur_point_index] = cur_point_index;

	if (time_limit_clocks - clock() + start <= 0)
		return err;


	size_t newCount = 0;
	int maxIterations = 3;
	FrameMultiThread(1, dress_index, cur_mesh.indexs_, cur_mesh.count_, cur_mesh.newIndexs_, newCount, SmallMaxCount, time_limit_clocks, DF::ITERATIONAL);

	ProcessFloor( dress_index );

	cur_mesh.GenerateAllocationMap();

    return err;
}

//#define MT_FALSE

void DFFitting::FrameMultiThread(int maxIterations, size_t dress_index, DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount, clock_t time_limit_clocks, DF::FittingMode fitting_mode)
{
    clock_t start = clock();
	// одевание под действием силы тяжести
	newCount = 0;
	if( force_rate_ > 0 ) {
#ifdef MT_FALSE
		ProcessDress(dress_index, 4, indexs, count, newIndexs, newCount, maxCount, fitting_mode);
#else
		ProcessDressMultiThread(dress_index, 4, indexs, count, newIndexs, newCount, 0, fitting_mode);
#endif
	}
	else {
#ifdef MT_FALSE
		ProcessDress(dress_index, 5, indexs, count, newIndexs, newCount, maxCount, fitting_mode);
#else
		ProcessDressMultiThread(dress_index, 5, indexs, count, newIndexs, newCount, 0, fitting_mode);
#endif
	}
	for (int i = 0; i < maxIterations; i++) {
		// одевание без сопротивления поверхности
#ifdef MT_FALSE
		ProcessDress(dress_index, 3, indexs, count, newIndexs, newCount, maxCount, fitting_mode);
#else
		ProcessDressMultiThread(dress_index, 3, indexs, count, newIndexs, newCount, 0, fitting_mode);
#endif
		
		// одевание с сопротивлением поверхности
#ifdef MT_FALSE
		ProcessDress(dress_index, 5, indexs, count, newIndexs, newCount, maxCount, fitting_mode);
#else
		ProcessDressMultiThread(dress_index, 5, indexs, count, newIndexs, newCount, maxCount, fitting_mode);
#endif
//		if (newCount == 0)
//			break;
//		AddRibsToCalc( dress_index, newIndexs, newCount, maxCount );
		
		if( fitting_mode == DF::INITIAL ) {
			// используется для первичного одевания
			{std::sort(&newIndexs[0], &newIndexs[newCount]); DF::Index *end = std::unique(&newIndexs[0], &newIndexs[newCount]); newCount = end - &newIndexs[0];}
//	 		log << "dress_index=" << dress_index << ",\tSubIter=" << i << " count=" << count << ", newCount=" << newCount << " time=" << (time_limit_clocks - clock() + start) << '\n';
			memcpy( indexs, newIndexs, newCount*sizeof(DF::Index) ); count = newCount; newCount = 0;
		}
		
		if (time_limit_clocks - clock() + start <= 0)
			break;
	}
	
}

void DFFitting::VerletRemainder(const size_t dress_index)
{
    DFMassSpringObject &cur_object = fitting_deque_[dress_index];
    DF::PointsVector &cur_vv = cur_object.vv_;
    size_t size_vv = cur_vv.size();
    for (size_t i_3 = 0; i_3 < size_vv; i_3++)
    {
        DF::FloatingPointType different = 0;
        cur_object.GetDifferent(0, i_3, different);
        cur_vv[i_3] += different;
    }
}

void DFFitting::AddRibsToCalc( size_t dress_index, DF::Index* newIndexs, size_t& newCount, size_t maxCount )
{
    DFMassSpringObject& cur_mesh = fitting_deque_[dress_index];
	// считаем все рёбра
	int debugCountRibStretched = 0;
	int debugCountRibShortly = 0;
	size_t countt = cur_mesh.vv_.size() / 3;
//	const DF::FloatingPointType R2 = 0.25;
	const DF::FloatingPointType R2 = 0.5;
	for (size_t cur_point_index = 0; cur_point_index < countt; cur_point_index++) {
		DFMath::Vector3 cur_point(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
		DFMath::Vector3 cur_normal(cur_mesh.vn_[cur_point_index * 3], cur_mesh.vn_[cur_point_index * 3 + 1], cur_mesh.vn_[cur_point_index * 3 + 2]);
		const DF::IncedenceVector &cur_incedence_vector = cur_mesh.incidence_[cur_point_index];
// 		bool flPull = false;
		for (size_t linked_point_i = 0; linked_point_i < cur_incedence_vector.size(); linked_point_i++) {
			DF::Index linked_point_index = cur_incedence_vector[linked_point_i].index;
			DFMath::Vector3 linked_point(cur_mesh.vv_[linked_point_index], cur_mesh.vv_[linked_point_index + 1], cur_mesh.vv_[linked_point_index + 2]);
			DF::FloatingPointType L0 = cur_incedence_vector[linked_point_i].distance;
			DFMath::Vector3 rib(linked_point - cur_point);
			DF::FloatingPointType L = DFMath::length(rib);
// 			DF::FloatingPointType stiffness_coefficient = 0;
			if( L > (1+R2)*L0 ) {
				// ребро слишком сильно растянуто
				debugCountRibStretched ++;
				if( newCount+2 < maxCount)
					newIndexs[newCount++] = linked_point_index/3;
			}
			else
			if( L < L0*(1-R2) ) {
				// ребро слишком сильно сжато
				debugCountRibShortly ++;
				if( newCount+2 < maxCount)
					newIndexs[newCount++] = linked_point_index/3;
			}
		}
		
	}
// 	log << "ribs " << debugCountRibStretched << "\t" << debugCountRibShortly << "\n";
	// ----------------------------------------------------------------------------
}

void DFFitting::ProcessFloor( size_t dress_index )
// обрезания одежды по полу
{
    DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
	size_t size = cur_mesh.vv_.size();
    for (size_t cur_point_index_3_1 = 1; cur_point_index_3_1 < size; cur_point_index_3_1 += 3) {
//        if (cur_mesh.vv_[cur_point_index_3_1-1] < -200)
//            cur_mesh.vv_[cur_point_index_3_1-1] = -200;
        if (cur_mesh.vv_[cur_point_index_3_1] < 0)
            cur_mesh.vv_[cur_point_index_3_1] = 0.0;
//        if (cur_mesh.vv_[cur_point_index_3_1+1] < -200)
//            cur_mesh.vv_[cur_point_index_3_1+1] = -200;
//        if (cur_mesh.vv_[cur_point_index_3_1-1] > 200)
//            cur_mesh.vv_[cur_point_index_3_1-1] = 200;
//        if (cur_mesh.vv_[cur_point_index_3_1] > 200)
//            cur_mesh.vv_[cur_point_index_3_1] = 200;
//        if (cur_mesh.vv_[cur_point_index_3_1+1] > 200)
//            cur_mesh.vv_[cur_point_index_3_1+1] = 200;
    }
}
/*
void DFFitting::InitForce( const DFMesh& cur_mesh )
{
	std::cout << "InitForce Begin\n";
//	if( force_.size() != cur_mesh.vv_.size() / 3 ) {
		// инициализируем начальные значения сил
        force_.resize( 0 );
        force_.resize( cur_mesh.vv_.size() / 3, DF::zero );
//	}
	std::cout << "InitForce End\n";
}
*/
DF::Error_t DFFitting::MoveDressToStartPosition(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index)
{
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
    DF::Error_t err = DF::err_ok;
    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
    DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
    DF::PointsVector source_vv(cur_mesh.vv_);

    DF::StateVector point_status(cur_mesh.vv_.size() / 3, 0);
    DF::IndexVector fixed_points(cur_mesh.vv_.size() / 3, INT_MAX);
    DF::Vector3Vector forces(cur_mesh.vv_.size() / 3);
    size_t fixed_point_actual_size = 0;
    size_t fixed_points_size_last = 1;
    size_t fixed_points_size_curr = 0;

    DFMesh& base_mannequin = mannequin.morph_targets()[DFMannequin::base_name].begin()->second;

    while (fixed_point_actual_size < point_status.size()) {
        DF::Index start_point_index = static_cast<DF::Index>(-1);
        DF::Index nearest_point_index = static_cast<DF::Index>(-1);
        DFMath::Vector3 vect_correction(0.);
        err = FindStartPoint(start_point_index, dress_index, point_status);
        if (err != DF::err_ok) {
            DF::print_error(err);
            return err;
        }

        DF::FloatingPointType &vx = cur_mesh.vv_[start_point_index];
        DF::FloatingPointType &vy = cur_mesh.vv_[start_point_index+1];
        DF::FloatingPointType &vz = cur_mesh.vv_[start_point_index+2];

        // вычисление правильной позиции для стартовой точки
        err = base_mannequin.SimpleFindNearestInEnvironment(nearest_point_index, DFMath::Vector3(vx, vy, vz), 5);

        if (err != DF::err_ok) {
            DF::print_error(err);
            return err;
        }

//         vx += fitting_deque_[0].vv_[nearest_point_index]   - mannequin_.base_mannequin.vv_[nearest_point_index];
        vy += fitting_deque_[0].vv_[nearest_point_index+1] - base_mannequin.vv_[nearest_point_index+1];
//         vz += fitting_deque_[0].vv_[nearest_point_index+2] - mannequin_.base_mannequin.vv_[nearest_point_index+2];

//         for (size_t i = 0; i < dress_index; i++)
//         {
//             if (fitting_deque_[i].DetectPenetration(scalar_correction, vect_correction, DFMath::Vector3(vx, vy, vz), 10000))
//             {
//                 vect_correction *= scalar_correction + shift;
//                 vx += vect_correction[0];
//                 vy += vect_correction[1];
//                 vz += vect_correction[2];
//             }
//         }
		/*
		size_t size = cur_mesh.vv_.size();
		for (size_t cur_point_index_3_1 = 1; cur_point_index_3_1 < size; cur_point_index_3_1 += 3) {
			cur_mesh.vv_[cur_point_index_3_1] += vy;
		}
		*/
		
        point_status[start_point_index / 3] = 2;
        fixed_points[fixed_point_actual_size] = start_point_index / 3;
        forces.resize(0);
        DF::Index forces_counter = 0;
        DFMath::Vector3 point1(0.), point1n(0.), point2(0.), delta_vector(0.);
        DF::Index counter = 0;
        DF::IndexVector layers;

        // проход по графу (поиск в ширину), вычисление корректной позиции для всех точек
        while ((fixed_point_actual_size < point_status.size() && fixed_points_size_last != fixed_points_size_curr) || counter < layers.size() + 10)
        {
            // сформируем слой
            fixed_points_size_last = fixed_points_size_curr;
            fixed_points_size_curr = fixed_point_actual_size;
            if (fixed_points_size_last != fixed_points_size_curr)
                layers.push_back(fixed_points_size_curr);
            for (size_t fixed_points_i = fixed_points_size_last; fixed_points_i < fixed_points_size_curr; fixed_points_i++)
            {
                DF::Index &cur_point_index = fixed_points[fixed_points_i];
                for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
                {
                    DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
                    if (point_status[cur_connected_point_index / 3] == 0) {  // ещё не пройдена
                        point_status[cur_connected_point_index / 3] = 1;
                        fixed_points[fixed_point_actual_size++] = cur_connected_point_index / 3;
                    }
                }
            }

            // вычисление перемещений (1)
            forces.resize(0);
            forces.resize(fixed_point_actual_size, DFMath::Vector3(0.));
            for (size_t fixed_points_i = fixed_points_size_curr; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
            {
                DF::Index &cur_point_index = fixed_points[fixed_points_i];
                DFMath::Vector3 &cur_force = forces[fixed_points_i];
                forces_counter = 0;
                for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
                {
                    DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
                    if (point_status[cur_connected_point_index / 3] < 2)  // не из предыдущих слоёв
                        continue;
                    cur_force += DFMath::Vector3(cur_mesh.vv_[cur_connected_point_index] - source_vv[cur_connected_point_index], 
                        cur_mesh.vv_[cur_connected_point_index + 1] - source_vv[cur_connected_point_index + 1], 
                        cur_mesh.vv_[cur_connected_point_index + 2] - source_vv[cur_connected_point_index + 2]);
                    forces_counter++;
                }
                // перемещение
                if (forces_counter) {
                    cur_force /= static_cast<DF::FloatingPointType>(forces_counter);
//                     DF::FloatingPointType cur_force_length = DFMath::length(cur_force);
//                     cur_force += DFMath::Vector3(0, -1, 0);
//                     cur_force *= cur_force_length / DFMath::length(cur_force);
                    point1.set(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
                    point1 += cur_force;
					/*
                    for (int dress_i = dress_index - 1; dress_i >= 0; dress_i--)
                    {
                        scalar_correction = 0;
                        if (!fitting_deque_[dress_i].DetectPenetration(scalar_correction, vect_correction, point1, cur_mesh.max_dist_2_)) {
                            points_to_move[cur_point_index] = 1;
                            continue;
                        }
                        scalar_correction += shift;
                        if (scalar_correction > 0) {
                            point1 += vect_correction * scalar_correction;
                            points_to_move[cur_point_index] = 0;
                        } else {
                            points_to_move[cur_point_index] = 1;
                        }
                        break;
                    }
					*/
                    cur_mesh.vv_[cur_point_index * 3] = point1[0];
                    cur_mesh.vv_[cur_point_index * 3 + 1] = point1[1];
                    cur_mesh.vv_[cur_point_index * 3 + 2] = point1[2];
                }
            }

            for (size_t fixed_points_i = fixed_points_size_curr; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
                point_status[fixed_points[fixed_points_i]] = 2;

//             size_t first_point = layers.size() >= 10 ? *(layers.end()-10) : 0;
// //             size_t first_point = fixed_points_size_last;
//             first_point = 0;
//             DF::FloatingPointType arc_length_max = static_cast<DF::FloatingPointType>(0);
//             DF::FloatingPointType arc_length_max_prev = static_cast<DF::FloatingPointType>(0);
//             DF::FloatingPointType arc_length_avr = static_cast<DF::FloatingPointType>(0);
//             size_t avr_length_counter = 0;
//             DF::FloatingPointType length_rate = static_cast<DF::FloatingPointType>(0);
//             DF::FloatingPointType barrier_rate = static_cast<DF::FloatingPointType>(0.5);
//             do
//             {
//                 forces.resize(0);
//                 forces.resize(fixed_point_actual_size, DFMath::Vector3(0.));
//                 arc_length_max_prev = arc_length_max;
//                 arc_length_max = static_cast<DF::FloatingPointType>(0);
//                 arc_length_avr = static_cast<DF::FloatingPointType>(0);
//                 avr_length_counter = 0;
//                 for (size_t fixed_points_i = first_point; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
//                 {
//                     DF::Index &cur_point_index = fixed_points[fixed_points_i];
//                     DFMath::Vector3 &cur_force = forces[fixed_points_i];
//                     forces_counter = 0;
//                     for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
//                     {
//                         DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
//                         if (point_status[cur_connected_point_index / 3] < 2)  // не из предыдущих слоёв
//                             continue;
//                         point1.set(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
//                         point2.set(cur_mesh.vv_[cur_connected_point_index], cur_mesh.vv_[cur_connected_point_index + 1], cur_mesh.vv_[cur_connected_point_index + 2]);
//                         delta_vector.set(point2 - point1);
//                         L0 = cur_mesh.incidence_[cur_point_index][incidence_i].distance;
//                         L = DFMath::length(delta_vector);
//                         length_rate = L0 / L;
//                         if (length_rate > barrier_rate)
//                         {
//                             continue;
//                         }
//                         if (arc_length_max < L)
//                             arc_length_max = L;
//                         arc_length_avr += L;
//                         avr_length_counter++;
//                         if (L > 1e-3) {
//                             cur_force += delta_vector * (static_cast<DF::FloatingPointType>(1.) - length_rate);
//                             forces_counter++;
//                         }
//                     }
//                     if (forces_counter)
//                         cur_force /= static_cast<DF::FloatingPointType>(forces_counter);
//                 }
//                 arc_length_avr /= avr_length_counter;
// 
//                 for (size_t fixed_points_i = first_point; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
//                 {
//                     DF::Index &cur_point_index = fixed_points[fixed_points_i];
//                     DFMath::Vector3 &cur_force = forces[fixed_points_i];
//                     point1.set(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
//                     point1 += forces[fixed_points_i];
//                     for (int dress_i = dress_index - 1; dress_i >= 0; dress_i--)
//                     {
//                         scalar_correction = 0;
//                         if (!fitting_deque_[dress_i].DetectPenetration(scalar_correction, vect_correction, point1, cur_mesh.max_dist_2_))
//                             continue;
//                         scalar_correction += shift;
//                         if (scalar_correction > 0)
//                             point1 += vect_correction * scalar_correction;
//                         break;
//                     }
//                     cur_mesh.vv_[cur_point_index * 3] = point1[0];
//                     cur_mesh.vv_[cur_point_index * 3 + 1] = point1[1];
//                     cur_mesh.vv_[cur_point_index * 3 + 2] = point1[2];
//                 }
//                 barrier_rate *= 2;
// //                 log << "avr = " << arc_length_avr << ", max = " << arc_length_max << ", max_prev = " << arc_length_max_prev << ", count = " << avr_length_counter << "\n";
//             } while (barrier_rate <= 1.1);

            counter++;
//             log << fixed_point_actual_size << " ";
//             DFMassSpringSystem temp_mesh = fitting_deque_[dress_index];
//             for (size_t iii = 0; iii < dress_index; iii++)
//                 temp_mesh.Add(fitting_deque_[iii]);
//             temp_mesh.ExportWavefront(std::string("temp_") + char('0'+dress_index) + std::string("_") + char('0'+counter/100) + char('0'+counter/10%10) + char('0'+counter%10) + std::string(".obj"));
        }
		

        //     log << "\n";
    }

    return err;
}

DF::Error_t DFFitting::SimpleMoveDressToStartPosition(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index, const DFMesh& cached_morph)
{
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
    DF::Error_t err = DF::err_ok;
    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
    DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];
    DF::PointsVector source_vv(cur_mesh.vv_);
    DF::StateVector point_status(cur_mesh.vv_.size() / 3, 0);
    DF::IndexVector fixed_points(cur_mesh.vv_.size() / 3, INT_MAX);
    size_t fixed_point_actual_size = 0;
    size_t fixed_points_size_last = 1;
    size_t fixed_points_size_curr = 0;

    const DFMesh& base_mannequin = cached_morph.isExist() ? cached_morph : mannequin.morph_targets()[DFMannequin::base_name].begin()->second;
    while (fixed_point_actual_size < point_status.size())
    {
        DF::Index start_point_index = static_cast<DF::Index>(-1);
        DF::Index nearest_point_index = static_cast<DF::Index>(-1);
        DFMath::Vector3 vect_correction(0.);
        err = FindStartPoint(start_point_index, dress_index, point_status);
        if (err != DF::err_ok) {
            DF::print_error(err);
            return err;
        }

        DF::FloatingPointType &vx = cur_mesh.vv_[start_point_index];
        DF::FloatingPointType &vy = cur_mesh.vv_[start_point_index+1];
        DF::FloatingPointType &vz = cur_mesh.vv_[start_point_index+2];

        // вычисление правильной позиции для стартовой точки
        err = base_mannequin.SimpleFindNearestInEnvironment(nearest_point_index, DFMath::Vector3(vx, vy, vz), 5);
        if (err != DF::err_ok) {
            DF::print_error(err);
            return err;
        }
        DF::FloatingPointType cur_shift_y = fitting_deque_[0].vv_[nearest_point_index+1] - base_mannequin.vv_[nearest_point_index+1];

        point_status[start_point_index / 3] = 2;
        fixed_points[fixed_point_actual_size++] = start_point_index / 3;
        cur_mesh.vv_[start_point_index + 1] += cur_shift_y;
        DFMath::Vector3 point1(0.), point1n(0.), point2(0.), delta_vector(0.);
        DF::Index counter = 0;
        DF::IndexVector layers;

        // проход по графу (поиск в ширину), вычисление корректной позиции для всех точек
        while ((fixed_point_actual_size < point_status.size() && fixed_points_size_last != fixed_points_size_curr) || counter < layers.size() + 10)
        {
            // сформируем слой
            fixed_points_size_last = fixed_points_size_curr;
            fixed_points_size_curr = fixed_point_actual_size;
            if (fixed_points_size_last != fixed_points_size_curr)
                layers.push_back(fixed_points_size_curr);
            for (size_t fixed_points_i = fixed_points_size_last; fixed_points_i < fixed_points_size_curr; fixed_points_i++)
            {
                DF::Index &cur_point_index = fixed_points[fixed_points_i];
                for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
                {
                    DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
                    if (point_status[cur_connected_point_index / 3] == 0) {  // ещё не пройдена
                        point_status[cur_connected_point_index / 3] = 1;
                        fixed_points[fixed_point_actual_size++] = cur_connected_point_index / 3;
                    }
                }
            }

            // перемещение
            for (size_t fixed_points_i = fixed_points_size_curr; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
            {
                cur_mesh.vv_[fixed_points[fixed_points_i] * 3 + 1] += cur_shift_y;
                point_status[fixed_points[fixed_points_i]] = 2;
            }

            counter++;
        }
    }

    return err;
}

DF::Error_t DFFitting::PutOnWithHardArcs(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index)
{
    DF::Error_t err = DF::err_ok;

    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMassSpringObject &cur_mesh = fitting_deque_[dress_index];

    // TODO: переменная толщина одежды
    const DF::FloatingPointType shift = static_cast<DF::FloatingPointType>(0.4);

    // вычисление максимальной дальности поиска
    DF::FloatingPointType max_dist_2 = 0;
    for (size_t i = 0; i < cur_mesh.incidence_.size(); i++)
        for (size_t j = 0; j < cur_mesh.incidence_[i].size(); j++)
            if (max_dist_2 < cur_mesh.incidence_[i][j].distance)
                max_dist_2 = cur_mesh.incidence_[i][j].distance;
//     max_dist_2 *= 2;
    max_dist_2 *= max_dist_2;

//     cur_mesh.GenerateIncidenceStruct();
//     cur_mesh.GenerateSpringsStruct();

    // поиск стартовой точки
    DF::Index start_point_index_up = static_cast<DF::Index>(-1);
    DF::Index start_point_index_down = static_cast<DF::Index>(-1);
    DF::FloatingPointType start_up = -300;
    DF::FloatingPointType start_down = 300;
    for (size_t i = 0; i < cur_mesh.vv_.size(); i += 3)
    {
        if (cur_mesh.vv_[i+1] > start_up) {
            start_up = cur_mesh.vv_[i+1];
            start_point_index_up = i;
        }
        if (cur_mesh.vv_[i+1] < start_down) {
            start_down = cur_mesh.vv_[i+1];
            start_point_index_down = i;
        }
    }
    DF::Index start_point_index = (start_down + start_up) / 2. < 100 ? start_point_index_down : start_point_index_up;

    DF::FloatingPointType &vx = cur_mesh.vv_[start_point_index];
    DF::FloatingPointType &vy = cur_mesh.vv_[start_point_index+1];
    DF::FloatingPointType &vz = cur_mesh.vv_[start_point_index+2];

    DF::PointsVector source_vv(cur_mesh.vv_);

    DF::FloatingPointType scalar_correction = 0;
    DFMath::Vector3 vect_correction(0.);
    // вычисление правильной позиции для стартовой точки
    DF::Index index_nearest = static_cast<DF::Index>(-1);
    DFMesh& base_mannequin = (*mannequins_storage_pointer_)[mannequin_ID].morph_targets()[DFMannequin::base_name].begin()->second;
    if ((err = base_mannequin.SimpleFindNearestInEnvironment(index_nearest, DFMath::Vector3(vx, vy, vz), 5)) != DF::err_ok)
        return err;
    vx += fitting_deque_[0].vv_[index_nearest]   - base_mannequin.vv_[index_nearest];
    vy += fitting_deque_[0].vv_[index_nearest+1] - base_mannequin.vv_[index_nearest+1];
    vz += fitting_deque_[0].vv_[index_nearest+2] - base_mannequin.vv_[index_nearest+2];

    for (size_t i = 0; i < dress_index; i++)
    {
        if (fitting_deque_[i].DetectPenetration(scalar_correction, vect_correction, DFMath::Vector3(vx, vy, vz)))
        {
            vect_correction *= scalar_correction + shift;
            vx += vect_correction[0];
            vy += vect_correction[1];
            vz += vect_correction[2];
        }
    }

//     DFMassSpringObject temp_mesh = fitting_deque_[0];
//     for (size_t iii = 1; iii <= dress_index; iii++)
//         temp_mesh.Add(fitting_deque_[iii]);
//     temp_mesh.ExportWavefront(std::string("temp_") + char('0'+dress_index) + std::string("_000.obj"));

    DF::StateVector point_status(cur_mesh.vv_.size() / 3, 0);
    point_status[start_point_index / 3] = 2;
    DF::IndexVector fixed_points(cur_mesh.vv_.size() / 3, INT_MAX);
    fixed_points[0] = start_point_index / 3;
    DF::IndexVector fixed_points_inverse(cur_mesh.vv_.size() / 3, INT_MAX);
    fixed_points_inverse[start_point_index / 3] = 0;
    DF::Vector3Vector forces(cur_mesh.vv_.size() / 3);
    forces.resize(0);
    DF::Vector3Vector forces_spread(cur_mesh.vv_.size() / 3);
    forces_spread.resize(0);
    DF::StateVector skipping_map(cur_mesh.vv_.size() / 3);
    skipping_map.resize(0);
    size_t fixed_point_actual_size = 1;
    size_t fixed_points_size_last = 1;
    size_t fixed_points_size_curr = 0;
    DF::Index forces_counter = 0;
    DFMath::Vector3 point1(0.), point1n(0.), point2(0.), delta_vector(0.);
    DF::Index counter = 0;
    DF::IndexVector layers;

    // проход по графу (поиск в ширину), вычисление корректной позиции для всех точек
    while ((fixed_point_actual_size < point_status.size() && fixed_points_size_last != fixed_points_size_curr) || counter < layers.size() + 20)
    {
        // сформируем слой
        fixed_points_size_last = fixed_points_size_curr;
        fixed_points_size_curr = fixed_point_actual_size;
        if (fixed_points_size_last != fixed_points_size_curr)
            layers.push_back(fixed_points_size_curr);
        for (size_t fixed_points_i = fixed_points_size_last; fixed_points_i < fixed_points_size_curr; fixed_points_i++)
        {
            DF::Index &cur_point_index = fixed_points[fixed_points_i];
            for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
            {
                DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
                if (point_status[cur_connected_point_index / 3] == 0) {  // ещё не пройдена
                    point_status[cur_connected_point_index / 3] = 1;
                    fixed_points_inverse[cur_connected_point_index / 3] = fixed_point_actual_size;
                    fixed_points[fixed_point_actual_size++] = cur_connected_point_index / 3;
                }
            }
        }

        // пройдем по слоям, вычислим силы натяжения (учитывая только связи с предыдущими слоями)
        DF::FloatingPointType L = 0, L0 = 0;
        forces.resize(0);
        forces.resize(fixed_point_actual_size, DFMath::Vector3(0.));
        // вычисление перемещений (1)
        for (size_t fixed_points_i = fixed_points_size_curr; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
        {
            DF::Index &cur_point_index = fixed_points[fixed_points_i];
            DFMath::Vector3 &cur_force = forces[fixed_points_i];
            forces_counter = 0;
            for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
            {
                DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
                if (point_status[cur_connected_point_index / 3] < 2)  // не из предыдущих слоёв
                    continue;
                cur_force += DFMath::Vector3(cur_mesh.vv_[cur_connected_point_index] - source_vv[cur_connected_point_index], 
                    cur_mesh.vv_[cur_connected_point_index + 1] - source_vv[cur_connected_point_index + 1], 
                    cur_mesh.vv_[cur_connected_point_index + 2] - source_vv[cur_connected_point_index + 2]);
                forces_counter++;
            }
            // перемещение
            if (forces_counter) {
                cur_force /= static_cast<DF::FloatingPointType>(forces_counter);
                point1.set(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
                point1n.set(cur_mesh.vn_[cur_point_index * 3], cur_mesh.vn_[cur_point_index * 3 + 1], cur_mesh.vn_[cur_point_index * 3 + 2]);
                point1 += cur_force;
                for (int dress_i = static_cast<int>(dress_index) - 1; dress_i >= 0; dress_i--)
                {
                    scalar_correction = 0;
                    if (!fitting_deque_[dress_i].DetectPenetrationWithMinRadius(scalar_correction, vect_correction, point1, point1n, max_dist_2))
                        continue;
                    scalar_correction += shift;
//                     if (scalar_correction > 0)
                        point1 += vect_correction * scalar_correction;
                    break;
                }
                cur_mesh.vv_[cur_point_index * 3] = point1[0];
                cur_mesh.vv_[cur_point_index * 3 + 1] = point1[1];
                cur_mesh.vv_[cur_point_index * 3 + 2] = point1[2];
            }
        }

        // подтягивание (2)
        forces.resize(0);
        forces.resize(fixed_point_actual_size, DFMath::Vector3(0.));
        // вычисление сил
        for (size_t fixed_points_i = 0; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
        {
            DF::Index &cur_point_index = fixed_points[fixed_points_i];
            DFMath::Vector3 &cur_force = forces[fixed_points_i];
            forces_counter = 0;
            for (size_t incidence_i = 0; incidence_i < cur_mesh.incidence_[cur_point_index].size(); incidence_i++)
            {
                DF::Index &cur_connected_point_index = cur_mesh.incidence_[cur_point_index][incidence_i].index;
                if (point_status[cur_connected_point_index / 3] < 2)  // не из предыдущих слоёв
                    continue;
                point1.set(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
                point2.set(cur_mesh.vv_[cur_connected_point_index], cur_mesh.vv_[cur_connected_point_index + 1], cur_mesh.vv_[cur_connected_point_index + 2]);
                delta_vector.set(point2 - point1);
                L0 = cur_mesh.incidence_[cur_point_index][incidence_i].distance;
                L = DFMath::length(delta_vector);
                if (L > 1e-3) {
                    cur_force += delta_vector * (static_cast<DF::FloatingPointType>(1.) - L0 / L);
                    forces_counter++;
                }
            }
            if (forces_counter)
                cur_force /= static_cast<DF::FloatingPointType>(forces_counter);
        }
        // распространение сил
        forces_spread.resize(0);
        forces_spread.resize(fixed_point_actual_size, DFMath::Vector3(0.));
//         for (size_t fixed_points_i = 0; fixed_points_i < fixed_points_size_curr; fixed_points_i++)
//         {
//             skipping_map.resize(0);
//             skipping_map.resize(fixed_point_actual_size, INT_MAX);
//             cur_mesh.recurse_spread_force(fixed_points_i, forces[fixed_points_i], forces_spread, skipping_map, fixed_points, fixed_points_inverse, point_status, static_cast<DF::FloatingPointType>(0.1), 1, 2);
//         }
        // приложение сил
        for (size_t fixed_points_i = 0; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
        {
            DF::Index &cur_point_index = fixed_points[fixed_points_i];
//             DFMath::Vector3 &cur_force = forces[fixed_points_i];
            point1.set(cur_mesh.vv_[cur_point_index * 3], cur_mesh.vv_[cur_point_index * 3 + 1], cur_mesh.vv_[cur_point_index * 3 + 2]);
            point1 += forces[fixed_points_i] + forces_spread[fixed_points_i];
            for (int dress_i = static_cast<int>(dress_index) - 1; dress_i >= 0; dress_i--)
            {
                scalar_correction = 0;
                if (!fitting_deque_[dress_i].DetectPenetration(scalar_correction, vect_correction, point1))
                    continue;
                scalar_correction += shift;
                if (scalar_correction > 0)
                    point1 += vect_correction * scalar_correction;
                break;
            }
            cur_mesh.vv_[cur_point_index * 3] = point1[0];
            cur_mesh.vv_[cur_point_index * 3 + 1] = point1[1];
            cur_mesh.vv_[cur_point_index * 3 + 2] = point1[2];
        }

        for (size_t fixed_points_i = fixed_points_size_curr; fixed_points_i < fixed_point_actual_size; fixed_points_i++)
            point_status[fixed_points[fixed_points_i]] = 2;

//         log << fixed_point_actual_size << " ";

        counter++;
//         DFMassSpring temp_mesh = fitting_deque_[0];
//         for (size_t iii = 1; iii <= dress_index; iii++)
//             temp_mesh.Add(fitting_deque_[iii]);
//         temp_mesh.ExportWavefront(std::string("temp_") + char('0'+dress_index) + std::string("_") + char('0'+counter/100) + char('0'+counter/10%10) + char('0'+counter%10) + std::string(".obj"));
    }
//     log << "\n";

//     cur_mesh.GenerateAllocationMap();
    return DF::err_ok;
}

size_t DFFitting::GetMemoryUsage() const
{
    size_t fitting_deque_size = sizeof(fitting_deque_);
    for (ObjectsArray::const_iterator cit = fitting_deque_.cbegin(); cit != fitting_deque_.cend(); cit++)
        fitting_deque_size += cit->GetMemoryUsage();
    return sizeof(*this) + fitting_deque_size + /*ingot_.GetMemoryUsage() +*/ log.GetMemoryUsage();
}

DF::Error_t DFFitting::PushToFittingDeque(DF::Index i, const DFMassSpringObject &mesh)
{
    DFMassSpringObject *pMesh = 0;
    std::string *pName = 0;
    if (i < fitting_deque_.size())
    {
        fitting_deque_[i] = mesh;
        pMesh = &fitting_deque_[i];
        pName = &fitting_deque_names_[i];
    }
    else
    {
        fitting_deque_.push_back(mesh);
        pMesh = &fitting_deque_.back();
        fitting_deque_names_.resize(fitting_deque_names_.size() + 1);
        pName = &fitting_deque_names_.back();
    }
    if (pMesh->GenerateAllocationMap() != DF::err_ok) return DF::err_cant_generate_allocation_map;
    if (pMesh->MergeCoincidentVertices() != DF::err_ok) return DF::err_unknown;
    if (pMesh->RecalcNormals() != DF::err_ok) return DF::err_cant_recalc_normals;
    if (pMesh->GenerateIncidenceStruct(true, false) != DF::err_ok) return DF::err_cant_generate_incidence_struct;
    if (pMesh->GenerateCollisionalLinks() != DF::err_ok) return DF::err_cant_generate_incidence_struct;
    pMesh->CalcInternalParams();
    *pName = "dress";
    pName->push_back(char(i/10+'0'));
    pName->push_back(char(i%10+'0'));
    if (pMesh->groups_.size() == 0)
        pMesh->AddStandartGroup(pName->c_str());
    return DF::err_ok;
}

DF::FloatingPointType DFFitting::CalcTimeRate(const size_t vertices_count)
{
    if (vertices_count > 50000)
        return 3;
    if (vertices_count > 20000)
        return 2;
    return 1;
}

DF::Error_t ExportBIN_OBJ(const DFMassSpringObject &mso, const std::string &file_name_without_extension)
{
    DF::Error_t err = mso.ExportOBJ(file_name_without_extension + std::string(".obj"));
    if (err == DF::err_ok)
        err = mso.ExportBIN(file_name_without_extension + std::string(".bin"), OUT_VERTICES);
    return err;
}

DF::Error_t DFFitting::GenerateDressMorphTargets(DFMannequin::MannequinIDType mannequin_ID, std::string output_path)
{
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
    DFMannequin::MorphParams morph_params;
    DF::Error_t err = DF::err_ok;
    const double time_limit_first_frame = 10.;
    const double time_limit_per_frame = 1.5;
    const double time_limit_total = 180.;
    DFMassSpringObject source_dress(fitting_deque_[1]);
    DFMesh& base_mannequin = mannequin.morph_targets()[DFMannequin::base_name].begin()->second;
    for (DFMannequin::MorphTargetStorage::const_iterator morph_target_group_it = mannequin.morph_targets().cbegin(); morph_target_group_it != mannequin.morph_targets().cend(); morph_target_group_it++)
    {
        for (DFMannequin::MorphTargetsByParam::const_iterator morph_target_it = morph_target_group_it->second.cbegin(); morph_target_it != morph_target_group_it->second.cend(); morph_target_it++)
        {
            fitting_deque_[0] = morph_target_it->second;
            fitting_deque_[0].vf_ = base_mannequin.vf_;
            fitting_deque_[0].RecalcNormals();
            fitting_deque_[0].GenerateAllocationMap();
            fitting_deque_[0].GenerateIncidenceStruct(false, false);
            DFMannequin::MorphParamsArray cached_morph_params;
            if ((err = FitAnimated(mannequin_ID, morph_params, cached_morph_params, output_path, 1000, 0, time_limit_first_frame, time_limit_per_frame, time_limit_total)) != DF::err_ok) {
                log << "Error: " << print_error(err) << '\n';
                return err;
            }
            ExportBIN_OBJ(fitting_deque_[1],
                output_path + 
                std::string("dress") + 
                '_' + 
//                 char('0'+morph_target_group_it->first%10) + 
                morph_target_group_it->first + 
                '_' +
                char('0'+morph_target_it->first / 100) + 
                char('0'+int(morph_target_it->first)%100/10) + 
                char('0'+int(morph_target_it->first)%10) + 
                ',' +
                char('0'+int(morph_target_it->first*10)%10));
            fitting_deque_[1] = source_dress;
        }
    }
    base_mannequin.ExportBIN(output_path + std::string("morph.bin"));
    fitting_deque_[0] = base_mannequin;
    fitting_deque_[0].RecalcNormals();
    fitting_deque_[0].GenerateAllocationMap();
    fitting_deque_[0].GenerateIncidenceStruct(false, false);
    DFMannequin::MorphParamsArray cached_morph_params;
    if ((err = FitAnimated(mannequin_ID, morph_params, cached_morph_params, output_path, 1000, 0, time_limit_first_frame, time_limit_per_frame, time_limit_total)) != DF::err_ok) {
        log << "Error: " << print_error(err) << '\n';
        return err;
    }
    fitting_deque_[1].ExportBIN(output_path + std::string("dress.bin"));
    return DF::err_ok;
}

DF::Error_t DFFitting::GenerateMorphAnimation(DFMannequin::MannequinIDType mannequin_ID, std::string output_path, DFMannequin::MorphParams &morphing_params0, DFMannequin::MorphParams &morphing_params1)
{
    if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
    DFMannequin dress;
    dress.SetLog(log);
    if (dress.AddMorphTargetBIN(output_path + std::string("dress.bin"), DFMannequin::base_name, 0) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_0_080,0.bin"), "0", 80) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_0_130,0.bin"), "0", 130) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_1_060,0.bin"), "1", 60) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_1_090,0.bin"), "1", 90) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_2_084,0.bin"), "2", 84) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_2_110,0.bin"), "2", 110) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_3_155,0.bin"), "3", 155) != DF::err_ok) return DF::err_binary_export_error;
    if (dress.AddMorphTargetBIN(output_path+std::string("dress_3_190,0.bin"), "3", 190) != DF::err_ok) return DF::err_binary_export_error;

//     for (MorphTargetStorage::const_iterator morph_target_group_it = mannequin_.morph_targets.cbegin(); morph_target_group_it != mannequin_.morph_targets.cend(); morph_target_group_it++)
//     {
//         if (morph_target_group_it->first == param)
//             morphing_params[morph_target_group_it->first] = (morph_target_group_it->second.cbegin()->first + morph_target_group_it->second.crbegin()->first) / 2;
//     }
    
    DFMannequin::MorphParams morphing_params(morphing_params0);
    const int max_counter = 100;
    try {
        DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
        mannequin.GetMemoryUsage();
    }
    catch(...) {
        return DF::err_bad_mannequin_pointer;
    }
    DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
    DFMesh morph;
    for (int counter = 1; counter <= max_counter+1; counter++)
    {
        for (DFMannequin::MorphParams::iterator mp_it = morphing_params.begin(); mp_it != morphing_params.end(); mp_it++)
            mp_it->second = (morphing_params1[mp_it->first] - morphing_params0[mp_it->first]) * (counter - 1) / max_counter + morphing_params0[mp_it->first];
        mannequin.Morph(morph, morphing_params);
        morph.ExportBIN(output_path + "morph_"+char('0'+counter % 1000 / 100)+char('0'+counter % 100 / 10)+char('0'+counter % 10)+".bin", OUT_VERTICES);
        dress.Morph(morph, morphing_params);
        morph.ExportBIN(output_path + "dress_"+char('0'+counter % 1000 / 100)+char('0'+counter % 100 / 10)+char('0'+counter % 10)+".bin", OUT_VERTICES);
    }

    return DF::err_ok;
}
