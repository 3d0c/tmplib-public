#ifndef DFFITTING_H
#define DFFITTING_H

#include "DFMesh.h"
#include "DFMassSpringObject.h"
#include "DFMannequin.h"
#include <ctime>

class DFFitting
{
public:
    enum SolvingCollizionMethod
    {
        DO_NOT_SOLVE,
        PUSH_OUT_FALLEN,
        DELETE_FALLEN
    };

    DFFitting();
    ~DFFitting();

//     template<class T> DF::Error_t AddBaseMannequinOBJ(MannequinIDType mannequin_ID, const T &mt) {
//         if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
//         try {
//             DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
//             mannequin.GetMemoryUsage();
//         }
//         catch(...) {
//             return DF::err_bad_mannequin_pointer;
//         }
//         return (*mannequins_storage_pointer_)[mannequin_ID].AddBaseMannequinOBJ(mt);
//     }
//     template<class T> DF::Error_t AddBaseMannequinBIN(MannequinIDType mannequin_ID, const T &mt) {
//         if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
//         try {
//             DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
//             mannequin.GetMemoryUsage();
//         }
//         catch(...) {
//             return DF::err_bad_mannequin_pointer;
//         }
//         return (*mannequins_storage_pointer_)[mannequin_ID].AddBaseMannequinBIN(mt);
//     }
//     template<class T> DF::Error_t AddMorphTargetOBJ(MannequinIDType mannequin_ID, T mt, const MorphParamType &param_identifier, const DF::FloatingPointType &value) {
//         if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
//         try {
//             DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
//             mannequin.GetMemoryUsage();
//         }
//         catch(...) {
//             return DF::err_bad_mannequin_pointer;
//         }
//         return (*mannequins_storage_pointer_)[mannequin_ID].AddMorphTargetOBJ(mt, param_identifier, value);
//     }
//     template<class T> DF::Error_t AddMorphTargetBIN(MannequinIDType mannequin_ID, T mt, const MorphParamType &param_identifier, const DF::FloatingPointType &value) {
//         if (mannequins_storage_pointer_ == 0) return DF::err_bad_mannequin_pointer;
//         try {
//             DFMannequin &mannequin = (*mannequins_storage_pointer_)[mannequin_ID];
//             mannequin.GetMemoryUsage();
//         }
//         catch(...) {
//             return DF::err_bad_mannequin_pointer;
//         }
//         return (*mannequins_storage_pointer_)[mannequin_ID].AddMorphTargetBIN(mt, param_identifier, value);
//     }
    void SetMannequinStorage(DFMannequin::MannequinStorage& mannequin_storage) { mannequins_storage_pointer_ = &mannequin_storage; }
    DF::Error_t AddDress(const std::string &fn);
    DF::Error_t AddDress(void *objData);
	void RemoveDress(); // снимает последнюю в очереди модель одежды

    DF::Error_t AddMorph(const std::string &fn);
    DF::Error_t AddMorph(void *objData);

    DF::Error_t PushToFittingDeque(DF::Index i, const DFMassSpringObject &mesh);

    // 0 - морф
    // нумерация одежды с 1
    DFMassSpringObject& GetFittingElement(DF::Index i);
    const ObjectsArray& GetFittingDeque() const { return fitting_deque_; };
    DF::Index GetDressCount() { return fitting_deque_.size() - 1; }

//     DFMannequin &mannequin() { return mannequin_; }
//     DressList &dress_list() { return dress_list_; }
//     DFMesh &morph() { return dress_list_[0]; }

    // первичное одевание
    DF::Error_t FrameFirst(DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams &morph_params, DFMannequin::MorphParamsArray &cached_morph_params_array, const double dress_time_limit, const double time_limit_total);
    // первичное одевание отдельной одежды
    DF::Error_t FrameDressFirst(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index, const clock_t time_limit, const DFMesh& cached_morph);
	// одевание следующего кадра
	DF::Error_t FrameNext( const float step, size_t n, size_t k, /*clock_t time_limit_clocks,*/ const double time_limit_next_frame, std::string output_path );
	// одевание отдельной одежды в следующем кадре
	DF::Error_t FrameDressNext(size_t dress_index, clock_t time_limit_clocks);
    // производит акт примерки c анимацией
    DF::Error_t FitAnimated(DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams &morph_params, DFMannequin::MorphParamsArray &cached_morph_params_array, std::string output_path, size_t iterations_count, size_t n, const double time_limit_first_frame, const double time_limit_next_frame, const double time_limit_total);
    // производит акт примерки c анимацией и физикой
    DF::Error_t FitAnimatedPhys(DFMannequin::MannequinIDType mannequin_ID, DFMannequin::MorphParams &morph_params, std::string output_path);
    // генерирует последовательность морфтаргетов для 1-го предмета одежды из очереди одевания
    DF::Error_t GenerateDressMorphTargets(DFMannequin::MannequinIDType mannequin_ID, std::string output_path);
    // генерирует анимацию с морфированием для 1-го предмета одежды из очереди одевания
    DF::Error_t GenerateMorphAnimation(DFMannequin::MannequinIDType mannequin_ID, std::string output_path, DFMannequin::MorphParams &morphing_params0, DFMannequin::MorphParams &morphing_params1);
    // высвобождает ресурсы, задействованные в акте примерки
    void Release();
    // очищает все внутренние данные
    DF::Error_t Clear();

    void ProcessDressStep1(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::FittingMode fitting_mode);
    void ProcessDressStep2(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::FittingMode fitting_mode);
    void ProcessDressStep25(size_t dress_index);
    void ProcessDressStep3(size_t dress_index, const DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount);

    size_t GetMemoryUsage() const;

    template<class T>
    void SetLogIerarhy(T a, std::ofstream::openmode _Mode = std::ofstream::app|std::ofstream::out)
    {
        log.Set(a, _Mode);
#       if defined _DEBUG || defined _ALL_LOGS
        for (ObjectsArray::iterator it = fitting_deque_.begin(); it != fitting_deque_.end(); it++) it->log.Set(a, _Mode);
#       endif
    }

private:
    // одеть по жёстким ребрам (по топологии графа)
    DF::Error_t PutOnWithHardArcs(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index);

    DF::Error_t MoveDressToStartPosition(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index);
    DF::Error_t SimpleMoveDressToStartPosition(DFMannequin::MannequinIDType mannequin_ID, size_t dress_index, const DFMesh& cached_morph);

	void FrameMultiThread(int maxIterations, size_t dress_index, DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount, clock_t time_limit_clocks, DF::FittingMode fitting_mode);
	void ProcessDressMultiThread(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount, DF::FittingMode fitting_mode);
	DF::Error_t ProcessDress(size_t dress_index, int k, const DF::Index* indexs, size_t count, DF::Index* newIndexs, size_t& newCount, size_t maxCount, DF::FittingMode fitting_mode);

	// добавление рёбер для расчётаs
    void AddRibsToCalc( size_t dress_index, DF::Index* newIndexs, size_t& newCount, size_t maxCount );
	void ProcessFloor( size_t dress_index ); // обрезания одежды по полу

    DF::Error_t FindStartPoint(DF::Index &start_point_index, const size_t dress_index, DF::StateVector point_status) const;
    DF::Error_t FindStartPoints(DF::IndexVector &start_point_index_vector, const size_t dress_index, DF::StateVector point_status) const;
    void Jolt(size_t dress_index);
    void DropDown(size_t dress_index);
    DF::FloatingPointType CalcMaxFindDist(const DFMesh &cur_mesh) const;
//	void InitForce( const DFMesh& cur_mesh );
    // добавить остаток интеграла Верле
    void VerletRemainder(const size_t dress_index);

    DF::FloatingPointType CalcTimeRate(const size_t vertices_count);

//     void output_ingot(const DFMesh &mesh, int k);

    // указатель на внешний относительно класса набор манекенов
    DFMannequin::MannequinStorage *mannequins_storage_pointer_;
    // 0 - морф, далее - предметы одежды в порядке одевания
    ObjectsArray fitting_deque_;
    // имена выходных файлов очереди одежды
    DF::StringVector fitting_deque_names_;
    DF::FloatingPointType force_rate_;
	float m_frameTime; // промежуток времени с окончания предыдущего кадра

    Log log;
};

#endif // !DFFITTING_H
