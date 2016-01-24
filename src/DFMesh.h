#ifndef DFMESH_H
#define DFMESH_H

#include "DFDefinitions.h"
#include "3d_algebra.h"
#include "Log.h"
#include <algorithm>
#include "DFBinary.h"

class DFMesh;
typedef std::vector<DFMesh> MeshArray;

class DFMesh
{
    friend class DFMannequin;
    friend class DFFitting;
    friend class DFPoint;
public:
    DF::FloatingPointType min_dist_2_;
    DF::FloatingPointType max_dist_2_;
    DF::FloatingPointType max_find_distance_;
    DF::FloatingPointType max_find_distance_2_;

    enum CopyMode
    {
        DO_NOT_COPY,
        RESIZE_ONLY,
        COPY_COORDS,
        COPY_FACES,
        COPY_ALL
    };

    enum ClearMode
    {
        DO_NOT_CLEAR,
        NULL_POINTS_CLEAR_FACES,
        NULL_POINTS,
        NULL_VERTICES,
        CLEAR_FACES_GROUPS,
        CLEAR_GROUPS,
        CLOSE_LOG,
        CLEAR_INTERNAL,
        CLEAR_ALL
    };

    enum DetectStatus
    {
        NOT_DETECTED,          // �� ������������� ������ ��������
        CLASH_DETECTED,        // ������������� ������������ � �����������
        PENETRATION_DETECTED,  // ������������� ������������� �������
        PENETRATION_SOLVED,    // ������������� ������������ ������������� �������
        SLIDING_DETECTED       // ������������� ���������� �� �����������
    };

    enum BinaryType
    {
        VERTICES,
        NORMALS,
        TEXTURES,
        FACES,
        STANDART_GROUP
    };

    enum FileType
    {
        FILETYPE_UNKNOWN,
        FILETYPE_WAVEFRONT,
        FILETYPE_DFBINARY
    };

    enum SmoothType
    {
        SMOOTH_POST,
        SMOOTH_ACTUAL
    };

    DFMesh();
    DFMesh(const DFMesh * const init_mesh, CopyMode copy_mode);
    virtual ~DFMesh();

    void Init(const DFMesh * const init_mesh, CopyMode copy_mode);
    void Clear(ClearMode clear_mode);

    DF::Error_t ImportFile(const std::string &fn, FileType &file_type, bool bPack = false);

    DF::Error_t ImportOBJ(const std::string &fn, bool bPack = false);
    DF::Error_t ExportOBJ(const std::string &fn, int out_mode = OUT_VERTICES|OUT_TEXTURES|OUT_NORMALS|OUT_FACES, bool bPack = false, int compression_level = -1) const;

    DF::Error_t ImportBIN(const std::string &fn, bool bPack = false);
    DF::Error_t ExportBIN(const std::string &fn, int out_mode = OUT_VERTICES|OUT_TEXTURES|OUT_NORMALS|OUT_FACES, bool reform_vertices_according_textures = false, bool bPack = false, int compression_level = -1) const;
    DF::Error_t ImportBinaryArray(void * const pBinData, bool bPack = false);
    DF::Error_t ExportBinaryArray(void **ppBinData, size_t &bufSize, int out_mode, bool reform_vertices_according_textures = false, bool bPack = false, int compression_level = -1) const;

    DF::Error_t ImportBinaryCoords(DF::PointsVector &coord_vector, BinaryType input_type);
    DF::Error_t ImportBinaryFaces(DF::FacesVector &coord_vector);
    DF::Error_t ExportBinaryCoords(DF::PointsVector &coord_vector, BinaryType output_type) const;
    DF::Error_t ExportBinaryFaces(DF::FacesVector &coord_vector);

    DF::Error_t ParseWavefrontBuffer(void *objData, size_t objSize = 0, bool bPack = false);
    DF::Error_t ConvertToWavefrontBuffer(void **objData, size_t &objSize, bool out_normals = true, bool out_textures = true, bool bPack = false, int compression_level = -1) const;

    DF::Error_t GenerateAllocationMap() const;

    // ������������ � ��������� normal_field, mass_centers �� ������������ ������� vertex_allocation_map_, ��������� ������������ ������� �������
    DF::Error_t GenerateNormalsField();
    DF::Error_t GetNormalFieldVector(DFMath::Vector3 &found_vector, const DFMath::Vector3 &specified_point) const;

    DF::Index GetGroupByPoint(DF::Index ind_3) const;

    // ����������� ��������
    // ������� ������� incidence_
    DF::Error_t SmoothNormals(const size_t iterations_num, DF::PointsVector& pv);

    // ����������� ��������� ����
    // ������� ������� vn_ (��������)
    // k - ���������� ����������� (0,5 �� ���������)
    DF::Error_t SmoothMesh(DFMesh::SmoothType smooth_method, const DF::FloatingPointType k = static_cast<DF::FloatingPointType>(0.5));

    DF::Error_t Add(const DFMesh &m, bool add_name = true);

    void MoveY(DF::FloatingPointType dy);
	void MoveZ(float);
	void Move(float, int);
	// �������� ������� �� ��� x �� ���� angles[0], �� ��� y �� ���� angles[1], �� ��� z �� ���� angles[2]
	void RotateXYZ(const DFMath::Vector3 & angles);
	//
	DF::Error_t Tighten1(DF::FloatingPointType stiffness_coefficient, size_t iteration_num/*, float*/);
    // ������� ��������� ����� � ����
    // bMerge_normals_UVs - ����� �� ��������������� ���������� ���������� � ������� (���������� ��������� �� ����������� �������������)
    // ������������ vv_, vt_, vn_, vf_, vertex_allocation_map_, groups_
    // �� ������������ incidence_, collision_links_ (����� ���� MergePoints() ��������� �������� ���� ��������)
    void MergePoints(DF::Index i, DF::Index j, bool bMerge_normals_UVs = false);
	// ���������� �������
	void AddVertex(const DF::FloatingPointType &);
	// �������� �������
	void InsertVertex(size_t num, const DF::FloatingPointType &);
	// ������� ������� 
	// ��� �������� �������, �����: 
	// 1. ���� ��������� ������������ �� ����, �������� ������� � vf_, � ������� �����������
	// 2. ���� ��������� groups_ �� ����, �������� �������
	// 3. ���� ��������� �������� �� ����, �����������,
	void DelVertex(size_t num, bool b_vf = true, size_t z_num = 0, bool tt = false);
	// �������� ������ �� ���� ���������� �������
	DF::FloatingPointType & GetVertex(size_t);
	// ���������� �������� ��� ���������� �������
	void SetVertex(size_t, DF::FloatingPointType);
	// ���������� ���������� ���������
	void AddCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w = 0);
	// ���������� ���������� ����������
	void SetCoordTexture(const size_t &, DF::FloatingPointType);
	// �������� ���������� ����������
	DF::FloatingPointType GetCoordTexture(const size_t &);
	// ����������� ���� ���������� ���������
	void SumCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w = 0);
	void DivisionCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w = 1);
	void MultiplyCoordTexture(const DF::FloatingPointType & u, const DF::FloatingPointType & v, const DF::FloatingPointType & w = 1);
	// ���������� ����������
	void AddFaces(const DF::FacesVector  &);
	// ���������� ���������
	void AddFace(const DF::faceItemVector  &);
	// �������� ���������
	void InsertFace(size_t num, const DF::faceItemVector &);
	// ����� ������ ����������� �� ������� ������.
	bool FindNumFace(std::vector<size_t> & num_vert, size_t & num_face);
	// ��������, ��� ��������� � ����������� num_face, ������� old_v �� new_v
	//void ChangeVertInFace(const size_t & num_face, const size_t & old_v, const size_t & new_v);
	// ���������� ����� �����, ��������� �� �� �������� ������� �����������
	bool NumVertexWithMinimumDistanceFromGivenVertex(DFMath::Vector3 & vec, double & min_dist, size_t & num_min_point);
	//
	void SetFace(size_t, const DF::faceItemVector &);
	// �������� ���������
	DF::faceItemVector & GetFace(const size_t & num);
	// ���������� ������
	void AddGroup(const DF::GroupIdentifier &);
	// ���������� ������
	void SetGroup(size_t, DF::GroupIdentifier &);
	// ���������� �������� ������� �������������
	void AddIncidence(size_t, DF::IncedenceRecord &);
	//
	DF::IncedenceVector GetIncidence(size_t);
	//
	size_t CountIncidence();
	// ���������� ���-�� ����� � ����
	size_t CountRibs();
	// ���������� ���-�� ������
	size_t CountVertex();
	// ���������� ���-�� ���������� ���������
	size_t CountCoordTexture();
	// ���������� ���-�� ������������ (������)
	size_t CountFaces();
	// ���������� ���-�� �����
	size_t CountGroups();
	// �������� ������
	DF::GroupIdentifier & GetGroup(const size_t num);
	bool GetGroup(const std::string & name1, DF::GroupIdentifier & group);
    // ���������� ����� ������ �� ������� ����� (����������� �� 3)
    // (������������� �� ������������� vv_ �� �������)
    size_t GetGroupIndex3(const size_t point_num);
	// ����������� �����
	void UnionGroup();
	//
	DF::FloatingPointType GetNormalize(size_t);
	//
	void SetNormalize(size_t, DF::FloatingPointType);
    void AddStandartGroup(const char *group_name);

    // ������������� �������
    DF::Error_t RecalcNormals();
    // ������������� ���������� ���������� ��� ������������ ��������
    DF::Error_t RecalcUVWs();
    // ������������� ���������� ���������� ��� ������������ ��������, �������� ������� (�� ������ �������� �� ����� - ������ ���������)
    DF::Error_t RecalcUVWsExtended();

    // ����������� ������������� ����� � ���
    bool DetectPenetration(DF::FloatingPointType &scalar_correction, DFMath::Vector3 &vec, const DFMath::Vector3 &specified_point) const;
    bool DetectPenetrationNew(DF::FloatingPointType &scalar_correction, DFMath::Vector3 &vec, const DFMath::Vector3 &specified_point) const;
    bool DetectPenetrationWithMinRadius(DF::FloatingPointType &scalar_correction, DFMath::Vector3 &vec, const DFMath::Vector3 &specified_point, const DFMath::Vector3 &specified_normal, const DF::FloatingPointType max_dist_2) const;

    // ���������� ������� ����� ��������� ����� ����� ����� ����
//     DF::Error_t SimpleFindNearestInEnvironment_old(DF::Index &nearest_point_index, const DFMath::Vector3 &specified_point, const DF::FloatingPointType max_dist) const;
    DF::Error_t SimpleFindNearestInEnvironment(DF::Index &nearest_point_index, const DFMath::Vector3 &specified_point, size_t max_counter) const;
    DF::Error_t SimpleFindNearestInEnvironmentWithMinMargin(DF::Index &nearest_point_index, const DFMath::Vector3 &specified_point, const DF::FloatingPointType min_dist_2, const DF::FloatingPointType max_dist_2, size_t max_counter) const;

    // ���� ��� ����� ����������� � ��� �� �����������
    DF::Error_t FindIdentical(DF::Index &identical_point_index, const DF::Index& specified_point_index) const;

    // ���������� ������� ����� ��������� ������� �� 3-� ����� ����� ����� ����
    DF::Error_t SimpleFindNearestArea(DF::Index &nearest_point_index_3, DFMath::Vector3 &projection_point, DFMath::Vector3 &projection_normal, const DFMath::Vector3 &specified_point, size_t max_counter) const;
    DF::Error_t SimpleFindNearestAreaOriented(DF::Index &nearest_point_index_3, DFMath::Vector3 &projection_point, DFMath::Vector3 &projection_normal, const DFMath::Vector3 &specified_point, const DFMath::Vector3 &specified_normal, const DF::FloatingPointType max_dist_2) const;
    DF::Error_t SimpleFindNearestAreaWithMinMargin(DF::Index &nearest_point_index_3, DFMath::Vector3 &projection_point, DFMath::Vector3 &projection_normal, const DFMath::Vector3 &specified_point, size_t max_counter) const;

    // �������� ������� ������������� ��� ����
    // generate_stiffness_ribs - ������������ �������������� ����� (���� ���������)
    // generate_nexus_ribs - ������������ �������������� ����� (����� �������� ��������) (�������������� ��������� ������� GenerateAllocationMap())
	// b_evenly = true ������� ����� ����� ����� = len_rib
    DF::Error_t GenerateIncidenceStruct(bool generate_stiffness_ribs, bool generate_nexus_ribs, bool b_evenly = false, float len_rib = 0);

    // generate_attached_ribs - ������������ ����� �������� (���������� ������������� ������� ���������) (��������� ������� �������� � AllocationMap)
    DF::Error_t GenerateCollisionalLinks();

    // ������� ����������� ������������ �����
    // (�������������� ��������� ������� GenerateAllocationMap())
    DF::Error_t MergeCoincidentVertices();

	// ������� ����� ������� ( �������� num_del_v �� num_new_v � ������� �������������. ��� ������ ���� num_del_v �������� �� 1)
	void DelIncedenceVector(size_t num_del_v, size_t num_new_v);
	// ���������� ���-�� �����, ������� ��������
	size_t CountRibsLongerValue(DF::FloatingPointType len_rib);

    // ��������� �� ��������� ����� ����� ����� ����, ���������� ��������� � ���� ����� ���������
    // (���������� �������� ��������� ����� �������� �� �������������� ����� ������� �������)
    // (���������� ��������� vf_, �� ������� �������� vv_, vn_, vt_, incidence_, vertex_allocation_map_)
    DF::Error_t GenerateContinuityMap(DF::IndexVector &vv_colors, DF::IndexVector &vf_colors, DF::Index &colors_num) const;

    // �������� �� ����� ���� ������� �� start_point_index, "������������" ����� � ����� �������� ������ �����
    // (�������� ColourInVertices(), ColourInFaces() ���������� ��������� vf_)
    DF::Error_t ColourInVerticesFaces(const size_t start_point_index, DF::IndexVector &vv_colors, DF::IndexVector &vf_colors, DF::Index &colors_count) const;

    // ������� ������� �������������
    void CalcInternalParams();

    size_t GetMemoryUsage() const;
    void Release();

    void SetPrecession(const unsigned new_precision) { precision = new_precision; }
    template<class T>
    void SetLog(T plog, std::ofstream::openmode _Mode = std::ofstream::out) { log.Set(plog, _Mode); }
    inline const size_t& incedences_count() {return incedences_count_;}
    inline const size_t& collisions_links_count(){return collisions_links_count_;}
    bool isExist() const;

	DFMath::Vector3 Min();
	DFMath::Vector3 Max();

    // ���������� true, ���� ����� i ������� ������ � ������ j
    bool Connected(DF::Index i, DF::Index j);
    // ���������� true, ���� ����� i ������� ������ ���� rib_type � ������ j
    bool Connected(DF::Index i, DF::Index j, DF::RibType rib_type);
    // ���������� ���������� ����� ������� � ����� �����
    int GraphDistance(DF::Index i, DF::Index j);

protected:

    DF::FloatingPointType cube_section_size_x;  // ������ ��������������� ������� ���� ��������� ������������
    DF::FloatingPointType cube_section_size_y;  // ������ ��������������� ������� ���� ��������� ������������
    DF::FloatingPointType cube_section_size_z;  // ������ ��������������� ������� ���� ��������� ������������
    DF::FloatingPointType cube_origin_shift_x;  // ����� ������ ��������� ��������� ���� �� ��������������� ����������
    DF::FloatingPointType cube_origin_shift_y;  // ����� ������ ��������� ��������� ���� �� ��������������� ����������
    DF::FloatingPointType cube_origin_shift_z;  // ����� ������ ��������� ��������� ���� �� ��������������� ����������

    DF::PointsVector vv_;
    DF::PointsVector vn_;
    DF::PointsVector vt_;
    DF::FacesVector  vf_;
    DF::GroupsVector groups_;

    unsigned precision;

    mutable Log log;

    size_t incedences_count_;
    DF::IncedenceVector2D incidence_;
    size_t collisions_links_count_;
    DF::IncedenceVector2D collision_links_;
    mutable DF::IndexVector4D vertex_allocation_map_;  // 3� ������ ������ ����� �������� �������� �� ������������� �� ����� (������ ������� �� 3)
    DF::Vector3Vector3D normal_field_;  // ���� �������� (������� ������� �� �����) (�������������� vertex_allocation_map_)
//     DF::Vector3Vector3D mass_centers_;  // ������ ���� ����� (�������������� vertex_allocation_map_)

    // �������� �� ����� ���� ������� �� start_point_index, "������������" ����� �������� ������ �����
    // (���������� ��������� vv_, incidence_)
    DF::Error_t ColourInVertices(size_t start_point_index, DF::IndexVector &vv_colors, DF::Index &colors_num) const;

    // "������������" ����� �������� ��������� ����� vv_colors
    DF::Error_t ColourInFaces(const DF::IndexVector &vv_colors, DF::IndexVector &vf_colors) const;

    DF::Error_t ReformMeshByGraphLevels(DF::IndexVector &vf_colors);

    void recurse_spread_force(const DF::Index &cur_point_index, const DFMath::Vector3 &cur_force, DF::Vector3Vector &forces_spread, DF::StateVector &skipping_map, const DF::IndexVector &fixed_points, const DF::IndexVector &fixed_points_inverse, const DF::StateVector &point_status, DF::FloatingPointType attenuation_coefficient, const size_t cur_level, const size_t max_level) const;
};

#endif // !DFMESH_H
