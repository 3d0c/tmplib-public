#ifndef DFMANNEQUIN_H
#define DFMANNEQUIN_H

#include "DFMassSpringObject.h"
#include <map>

namespace DFMannequinDefinitions {
}

class DFMannequin
{
public:
    typedef std::string MorphParamType;   // ��� �������������� ��������� ������������
    typedef std::string MannequinIDType;  // ��� �������������� ��������
    static const MorphParamType base_name;
    typedef std::map<DF::FloatingPointType, DFMesh> MorphTargetsByParam;       // ��� ������ ������������ ������ ���� (� ���������� ���������� ������������)
    typedef std::map<MorphParamType, MorphTargetsByParam> MorphTargetStorage;  // ��� ������ ���� ������������ ��������
    typedef std::map<MorphParamType, DF::FloatingPointType> MorphParams;       // ��� ������ ���������� ������������
    typedef std::vector<MorphParams> MorphParamsArray;
    typedef std::map<MannequinIDType, DFMannequin> MannequinStorage;           // ��� ������ ���������

    DFMannequin();

    DF::Error_t AddBaseMannequin(const DFMesh &mesh);
    DF::Error_t AddMorphTarget(const std::string &fn, const MorphParamType &param_identifier, const DF::FloatingPointType &value);

    DF::Error_t AddMorphTargetOBJ(const std::string &fn, const MorphParamType &param_identifier, const DF::FloatingPointType &value);
    DF::Error_t AddMorphTargetOBJ(void *objData, const MorphParamType &param_identifier, const DF::FloatingPointType &value);

    DF::Error_t AddMorphTargetBIN(const std::string &fn, const MorphParamType &param_identifier, const DF::FloatingPointType &value);
    DF::Error_t AddMorphTargetBIN(void *objData, const MorphParamType &param_identifier, const DF::FloatingPointType &value);

    size_t GetMemoryUsage() const;
    void Release();

    DF::Error_t InitMannequin();
    // ��������� �������
    DF::Error_t Morph(DFMesh& morph, MorphParams &morph_params, bool prepare_environment_data = true);
    // ��������� approximated_mannequin ��� ������� ����� base_mannequin(rate=0) � morph(rate=1)
    DF::Error_t ApproximateToMorph(DFMesh &approximated_mannequin, const DF::FloatingPointType rate);
    // ��������� ������� �������� (���������� ��������)
    bool isExist();
    size_t Count();
    template<class T>
    void SetLog(T plog, std::ofstream::openmode _Mode = std::ofstream::out) { log.Set(plog, _Mode); }
    MorphTargetStorage& morph_targets() { return morph_targets_; }

private:
    MorphTargetStorage morph_targets_;

    Log log;
};

DFMannequin::MorphParams operator-(DFMannequin::MorphParams &morph_params1, DFMannequin::MorphParams& morph_params2);
DF::FloatingPointType Characteristic(const DFMannequin::MorphParams &morph_params);
bool operator<(const DFMannequin::MorphParams &morph_params1, const DFMannequin::MorphParams& morph_params2);


#endif // !DFMANNEQUIN_H
