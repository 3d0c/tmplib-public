#include "DFMannequin.h"
#include <ctime>
#include <cmath>

const DFMannequin::MorphParamType DFMannequin::base_name = "base";

DFMannequin::DFMannequin()
{

}

DF::Error_t DFMannequin::AddMorphTargetOBJ(const std::string &fn, const MorphParamType &param_name, const DF::FloatingPointType &val)
{
    //if (param_name.size() == 0) return DF::err_empty_param_name;
#   if defined _DEBUG || defined _ALL_LOGS
    morph_targets_[param_name][val].log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target (" << fn << "/" << param_name << "/" << val << ") started\n";
    clock_t start = clock();
    DF::Error_t err = morph_targets_[param_name][val].ImportOBJ(fn);
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateAllocationMap();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].RecalcNormals();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) morph_targets_[param_name][val].CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFMannequin::AddMorphTargetBIN(const std::string &fn, const MorphParamType &param_name, const DF::FloatingPointType &val)
{
    //if (param_name.size() == 0) return DF::err_empty_param_name;
#   if defined _DEBUG || defined _ALL_LOGS
    morph_targets_[param_name][val].log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target (" << fn << "/" << param_name << "/" << val << ") started\n";
    clock_t start = clock();
    DF::Error_t err = morph_targets_[param_name][val].ImportBIN(fn);
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateAllocationMap();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].RecalcNormals();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) morph_targets_[param_name][val].CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFMannequin::AddMorphTargetOBJ(void *objData, const MorphParamType &param_name, const DF::FloatingPointType &val)
{
    //if (param_name.size() == 0) return DF::err_empty_param_name;
#   if defined _DEBUG || defined _ALL_LOGS
    morph_targets_[param_name][val].log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target (" << param_name << "/" << val << ") started\n";
    clock_t start = clock();
    DF::Error_t err = morph_targets_[param_name][val].ParseWavefrontBuffer(objData);
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateAllocationMap();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].RecalcNormals();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) morph_targets_[param_name][val].CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

DF::Error_t DFMannequin::AddMorphTargetBIN(void *objData, const MorphParamType &param_name, const DF::FloatingPointType &val)
{
    //if (param_name.size() == 0) return DF::err_empty_param_name;
#   if defined _DEBUG || defined _ALL_LOGS
    morph_targets_[param_name][val].log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target (" << param_name << "/" << val << ") started\n";
    clock_t start = clock();
    DF::Error_t err = morph_targets_[param_name][val].ImportBinaryArray(objData);
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateAllocationMap();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].RecalcNormals();
    if (err == DF::err_ok) err = morph_targets_[param_name][val].GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) morph_targets_[param_name][val].CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

size_t DFMannequin::GetMemoryUsage() const
{
    MorphTargetStorage morph_targets;
    return sizeof(*this) + DF::GetMemoryUsage(morph_targets) + log.GetMemoryUsage();
}

void DFMannequin::Release()
{
    DF::ReleaseSTLContainer(morph_targets_);
    log.Close();
}

inline void CalcMorphDifference(DF::PointsVector& v3, const DF::PointsVector& v0, const DF::PointsVector& v1, const DF::PointsVector& v2, const DF::FloatingPointType &k)
{
    for (DF::Index i = 0; i < v3.size(); i++) v3[i] += v1[i] + k * (v2[i] - v1[i]) - v0[i];
}

DF::Error_t DFMannequin::Morph(DFMesh& morph, MorphParams &morph_params, bool prepare_environment_data)
{
    if (!isExist())
        return DF::err_not_enough_elements;
    log << "[" << DF::GetCurrentTimeString() << "] Morphing object ( ";
    for (MorphTargetStorage::iterator it_morphtarget_class = morph_targets_.begin(); it_morphtarget_class != morph_targets_.end(); it_morphtarget_class++)
        log << it_morphtarget_class->first << "=" << morph_params[it_morphtarget_class->first] << " ";
    log << ") started\n";
    clock_t start = clock();

    morph = morph_targets_[DFMannequin::base_name].begin()->second;
    morph.Clear(DFMesh::NULL_VERTICES);
    const DF::PointsVector& vv0 = morph_targets_[DFMannequin::base_name].begin()->second.vv_;
//     const DFMesh::PointsVector& vn0 = base_mannequin.vn();
//     const DFMesh::PointsVector& vt0 = base_mannequin.vt();
    DF::PointsVector& vv3 = morph.vv_;
//     DFMesh::PointsVector& vn3 = morph->vn();
//     DFMesh::PointsVector& vt3 = morph->vt();
    for (MorphTargetStorage::iterator it_morphtarget_class = morph_targets_.begin(); it_morphtarget_class != morph_targets_.end(); it_morphtarget_class++)
    {
        // поиск двух морфтаргетов, локализующих позицию требуемого морфа
        if (it_morphtarget_class->second.size() < 2) continue;
        MorphTargetsByParam::iterator it_morphtarget_1 = it_morphtarget_class->second.lower_bound(morph_params[it_morphtarget_class->first]);
        if (it_morphtarget_1 == it_morphtarget_class->second.begin()) it_morphtarget_1 ++;
        if (it_morphtarget_1 == it_morphtarget_class->second.end()) it_morphtarget_1 --;
        MorphTargetsByParam::iterator it_morphtarget_0 = it_morphtarget_1;
        it_morphtarget_0 --;

        const DF::PointsVector& vv1 = it_morphtarget_0->second.vv_;
//         const DFMesh::PointsVector& vn1 = it_morphtarget_0->second.vn();
//         const DFMesh::PointsVector& vt1 = it_morphtarget_0->second.vt();
        const DF::PointsVector& vv2 = it_morphtarget_1->second.vv_;
//         const DFMesh::PointsVector& vn2 = it_morphtarget_1->second.vn();
//         const DFMesh::PointsVector& vt2 = it_morphtarget_1->second.vt();

        DF::FloatingPointType k = (morph_params[it_morphtarget_class->first] - it_morphtarget_0->first) / (it_morphtarget_1->first - it_morphtarget_0->first);
//         // TODO: remove this patch
//         const DF::FloatingPointType alpha = 0.25;
//         k = pow(2 - alpha, k) + alpha - 1;

        CalcMorphDifference(vv3, vv0, vv1, vv2, k);
//         CalcMorphDifference(vn3, vn0, vn1, vn2, k);
//         CalcMorphDifference(vt3, vt0, vt1, vt2, k);
    }

    for (DF::Index i = 0; i < vv3.size(); i++) vv3[i] += vv0[i];
//     for (DF::Index i = 0; i < vn3.size(); i++) vn3[i] += vn0[i];
//     for (DF::Index i = 0; i < vt3.size(); i++) vt3[i] += vt0[i];
    if (prepare_environment_data)
    {
        morph.vt_ = morph_targets_[DFMannequin::base_name].begin()->second.vt_;
        morph.RecalcNormals();
        morph.GenerateAllocationMap();
//+++        morph->GenerateNormalsField(20, 20, 20);
        morph.GenerateIncidenceStruct(false, false);
    }

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Morphing object finished with status: " << DF::print_error(DF::err_ok) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";

    return DF::err_ok;
}

DF::Error_t DFMannequin::ApproximateToMorph(DFMesh &approximated_mannequin, const DF::FloatingPointType rate)
{
    (void)approximated_mannequin;
    (void)rate;
//     size_t mannequin_vv_size = base_mannequin.vv_.size();
//     approximated_mannequin.vv_.resize(mannequin_vv_size);
//     for (size_t i = 0; i < mannequin_vv_size; i++)
//         approximated_mannequin.vv_[i] = base_mannequin.vv_[i] * (1 - rate) + morph.vv_[i] * rate;
    return DF::err_ok;
}

DF::Error_t DFMannequin::InitMannequin()
{
    DF::Error_t err = DF::err_ok;
    if (err == DF::err_ok) err = morph_targets_[DFMannequin::base_name].begin()->second.GenerateAllocationMap();
    if (err == DF::err_ok) err = morph_targets_[DFMannequin::base_name].begin()->second.RecalcNormals();
    return err;
}

DF::Error_t DFMannequin::AddBaseMannequin(const DFMesh &mesh) { 
    DFMesh &base_mannequin = morph_targets_[base_name][0];
    base_mannequin = mesh;
    if (base_mannequin.GenerateAllocationMap() != DF::err_ok) return DF::err_cant_generate_allocation_map;
    if (base_mannequin.RecalcNormals() != DF::err_ok) return DF::err_cant_recalc_normals;
	if (base_mannequin.GenerateIncidenceStruct(false, false) != DF::err_ok) return DF::err_cant_generate_incidence_struct;
    base_mannequin.CalcInternalParams();
    if (base_mannequin.groups_.size() == 0)
        base_mannequin.AddStandartGroup("Mannequin");
    return DF::err_ok;
}

DF::Error_t DFMannequin::AddMorphTarget(const std::string &fn, const MorphParamType &param_name, const DF::FloatingPointType &val)
{
    //if (param_name.size() == 0) return DF::err_empty_param_name;
    DFMesh &cur_mesh = morph_targets_[param_name][val];
#   if defined _DEBUG || defined _ALL_LOGS
    cur_mesh.log = log;
#   endif
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target (\"" << fn << "\" / \"" << param_name << "\" / \"" << val << "\") started";
    clock_t start = clock();
    DFMesh::FileType file_type = DFMesh::FILETYPE_UNKNOWN;
    DF::Error_t err = cur_mesh.ImportFile(fn, file_type);
    switch (file_type)
    {
    case DFMesh::FILETYPE_WAVEFRONT: log << " (Wavefront OBJ type detected)\n"; break;
    case DFMesh::FILETYPE_DFBINARY: log << " (DFBinary type detected)\n"; break;
    default: log << " (unknown type detected)\n"; break;
    }
    if (err == DF::err_ok) err = cur_mesh.GenerateAllocationMap();
    if (err == DF::err_ok) err = cur_mesh.RecalcNormals();
    if (err == DF::err_ok) err = cur_mesh.GenerateIncidenceStruct(false, false);
    if (err == DF::err_ok) cur_mesh.CalcInternalParams();
    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Adding morph target finished with status: " << DF::print_error(err) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";
    return err;
}

bool DFMannequin::isExist()
{
    return morph_targets_.size() && morph_targets_.cbegin()->second.size()
        && morph_targets_.cbegin()->second.cbegin()->second.vv_.size() && morph_targets_.cbegin()->second.cbegin()->second.vf_.size();
}

size_t DFMannequin::Count()
{
    size_t ret = 0;
    if (isExist())
        ret++;
    ret += morph_targets_.size();
    return ret;
}

DFMannequin::MorphParams operator-(DFMannequin::MorphParams &morph_params1, DFMannequin::MorphParams& morph_params2)
{
    DFMannequin::MorphParams ret;
    for (DFMannequin::MorphParams::const_iterator cur_pare = morph_params1.cbegin(); cur_pare != morph_params1.cend(); cur_pare++) {
        DF::FloatingPointType cur_diff = cur_pare->second - morph_params2[cur_pare->first];
        ret[cur_pare->first] = cur_diff < 0 ? cur_diff : cur_diff * 10;
    }
    return ret;
}

DF::FloatingPointType Characteristic(const DFMannequin::MorphParams &morph_params)
{
    DF::FloatingPointType sum = 0;
    for (DFMannequin::MorphParams::const_iterator cur_pare = morph_params.cbegin(); cur_pare != morph_params.cend(); cur_pare++)
        sum += fabs(cur_pare->second);
    return sum;
}

bool operator<(const DFMannequin::MorphParams &morph_params1, const DFMannequin::MorphParams& morph_params2)
{
    return Characteristic(morph_params1) < Characteristic(morph_params2);
}
