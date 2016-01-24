#ifndef DFDRESS_H
#define DFDRESS_H

#include "DFMesh.h"
#include "DFPhysicalParams.h"
#include <deque>

class DFMassSpringObject;
class DFPoint;
typedef std::vector<DFMassSpringObject> ObjectsArray;
typedef std::deque<DF::PointsVector> StateDeque;

static const DF::FloatingPointType conversion_of_distance = 100;
static const DFMath::Vector3 free_fall_acceleration(DF::zero, static_cast<DF::FloatingPointType>(-9.8*conversion_of_distance), DF::zero);

class DFMassSpringObject : public DFMesh
{
    friend class DFFitting;
    friend class DFPoint;
public:
    DFMassSpringObject();
    DFMassSpringObject(const DFMesh&);
    void Init();
    virtual ~DFMassSpringObject();

    DF::Error_t LoadPhysParams(const std::string &fn);

    bool ShakeOutPoint(DF::Index point_index_3, DFMath::Vector3 vec, const DFMassSpringObject &ingot);
    bool ShakeOutPoint(DF::Index point_index_3, DFMath::Vector3 vec, const DFMassSpringObject &ingot, bool out);
    DF::FloatingPointType MovePointFull(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const DFMassSpringObject &ingot, DF::FloatingPointType maxLen );
//    DF::FloatingPointType MovePoint(DF::Index point_index_3, DFMath::Vector3 vec, ObjectArray::const_iterator mesh_deque_begin, ObjectArray::const_iterator mesh_deque_end, DF::FloatingPointType cos_lim);
    DF::FloatingPointType MovePoint(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const ObjectsArray& meshs, size_t dress_index, DF::FloatingPointType cos_lim, bool surface);
    DF::FloatingPointType MovePointWithFriction(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const ObjectsArray& meshs, size_t dress_index, DF::FloatingPointType cos_lim);
    void ShakeOutPoint0(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 vec, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end);
    void ShakeOutPoint2(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 vec, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end);
    void ShakeOutPoint3(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 vec, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end);
    void ShakeOutPoint4(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 vec, const DFMassSpringObject &ingot);
    bool Detect(DFMath::Vector3 specified_point, const DF::PhysicalParam &phys_param, DFMath::Vector3 vec, DF::FloatingPointType cos_lim) const;
    bool NearSurface( DFMath::Vector3& point, DFMath::Vector3 direction, bool in ) const;

    DetectStatus DecomposeMovement(DFMath::Vector3 specified_point, const DF::PhysicalParam &phys_param, const DFMath::Vector3 &vec, DFMath::Vector3 &tangential_vec, DFMath::Vector3 &normal_vec, DF::FloatingPointType cos_lim) const;

    // стягивает меш по упругим рёбрам
//     DF::Error_t Tighten_old(DF::FloatingPointType stiffness_coefficient);
    DF::Error_t Tighten(DF::FloatingPointType stiffness_coefficient, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end, DF::FloatingPointType max_dist_2, DF::FloatingPointType shift, size_t iteration_num);
    DF::Error_t Tighten(DF::FloatingPointType stiffness_coefficient, const DFMassSpringObject &mesh_deque_begin, DF::FloatingPointType max_dist_2, DF::FloatingPointType shift, size_t iteration_num);

    // разрешает коллизию
    // коллизией считается попадание элементов меша "внутрь" мешей mesh_deque
    DF::Error_t SolveCollizion(ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end, const DF::FloatingPointType shift);

    void SetTimeStep(const DF::FloatingPointType new_time_step);
    DF::FloatingPointType GetTimeStep() const;

    DF::Error_t StoreState(const size_t n);
    DF::Error_t GetDifferent(const DF::Index state_num, const DF::Index i_3, DF::FloatingPointType &speed);

    size_t GetMemoryUsage() const;
    void Release();

    const DF::PhysicalParamsVector& physical_params() { return physical_params_; }

private:
    DF::FloatingPointType time_step_;
    StateDeque stored_states_;
    DF::PhysicalParamsVector physical_params_;

    DF::Index* indexs_;
    DF::Index* newIndexs_;
    DF::Index maxCount_;
    DF::Index pos_;
    DF::Index count_;
	DFPoint* m_points;
	DF::PointsVector m_vv0;

};

#endif // !DFDRESS_H

