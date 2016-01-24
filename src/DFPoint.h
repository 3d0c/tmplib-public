#ifndef DFPOINT_H
#define DFPOINT_H

#include "3d_algebra.h"
#include "DFMassSpringObject.h"

const float Mass = 0.05; // масса точки в граммах

class DFPoint
{
protected:
	size_t m_index;	// индекс точки в DFMesh::vv_
	DFMath::Vector3 m_force; // суммарная действующая сила, она же - ускорение
    size_t m_forces_counter; // счётчик сложенных сил
public:
	enum Movement { Neutral, Down, Up};
	DFPoint();
	DFPoint( size_t index );
	DFPoint( size_t index, const DFMath::Vector3& force );
	DFPoint( const DFPoint& p );

	DFMath::Vector3 GetVector3( const DFMesh& clothing ) const;
	DFMath::Vector3 GetVector3( const DFMesh& clothing, size_t index_3 ) const;
	void SetVector3( DFMesh& clothing, const DFMath::Vector3& point );

    DFMath::Vector3 GetNormal( DFMesh& clothing );
    DFMath::Vector3 GetNormal( DFMesh& clothing, size_t index_3 );

    inline DFMath::Vector3 GetForce() const { return m_force; }
    template <typename T>
    inline void SetForce(const T& a) { m_force.set(a); }

    inline size_t GetForcesCounter() { return m_forces_counter; }
    inline void SetForcesCounter(size_t fc) { m_forces_counter = fc; }
    inline void IncrementForcesCounter() { m_forces_counter++; }

	size_t GetIndex() const { return m_index; }
	void SetIndex( size_t index ) { m_index = index; }

	bool IsMove( float dt, DF::FloatingPointType h ); // если сила превышает порог, то точку разрешается переместить и её необходимо пересчитать

	void AddForce( const DFMath::Vector3& force ); // добавляет силу
	void AddForcesFromLinks( DFMassSpringObject& clothing, DF::FloatingPointType force_rate, DF::FittingMode fitting_mode ); // добавляет силы из связей с соседними точками

	DFMath::Vector3 CalcV( float dt ); // считаем скорость
	DFMath::Vector3 CalcS( float dt ); // считаем перемещение

	bool DetectCollision( DFPoint& detectPoint, const DFMesh& clothing, const DF::PhysicalParam &phys_param, DFMath::Vector3 specified_point, DF::FloatingPointType cos_lim ) const; // проверяет, возможно ли переместить точку без столновения с поверхностью

	DF::FloatingPointType MovePoint( DFMassSpringObject& clothing, float dt, const DF::PhysicalParam &phys_param, ObjectsArray& meshs, size_t dress_index, DF::FloatingPointType cos_lim, DFPoint::Movement movement, bool surface ); // перемещает точку
	void MovePoint( DFMesh& clothing ); // перемещает точку, без учёта внешних влияний
	bool MovePointWithFriction( DFMesh& clothing, float dt ); // перемещает точку с учётом трения

	DFPoint& operator = ( const DFPoint& p );
};

inline
DFPoint::DFPoint() :
	m_index( -1 ),
	m_force( 0, 0, 0 ),
	m_forces_counter( 0 )
{
}

inline
DFPoint::DFPoint( size_t index ) :
	m_index( index ),
	m_force( 0, 0, 0 ),
	m_forces_counter( 0 )
{
}

inline
DFPoint::DFPoint( size_t index, const DFMath::Vector3& force ) :
	m_index( index ),
	m_force( force ),
	m_forces_counter( 0 )
{
}

inline
DFPoint::DFPoint( const DFPoint& p ) :
	m_index( p.m_index ),
	m_force( p.m_force ),
	m_forces_counter( p.m_forces_counter )
{
}

inline
DFMath::Vector3 DFPoint::GetVector3( const DFMesh& clothing ) const
{
	return DFMath::Vector3(clothing.vv_[m_index*3], clothing.vv_[m_index*3+1], clothing.vv_[m_index*3+2]);
}

inline
DFMath::Vector3 DFPoint::GetVector3( const DFMesh& clothing, size_t index_3 ) const
{
	return DFMath::Vector3(clothing.vv_[index_3], clothing.vv_[index_3+1], clothing.vv_[index_3+2]);
}

inline
DFMath::Vector3 DFPoint::GetNormal( DFMesh& clothing )
{
	return DFMath::Vector3(clothing.vn_[m_index*3], clothing.vn_[m_index*3+1], clothing.vn_[m_index*3+2]);
}

inline
DFMath::Vector3 DFPoint::GetNormal( DFMesh& clothing, size_t index_3 )
{
	return DFMath::Vector3(clothing.vn_[index_3], clothing.vn_[index_3+1], clothing.vn_[index_3+2]);
}

inline
void DFPoint::SetVector3( DFMesh& clothing, const DFMath::Vector3& point )
{
	clothing.vv_[m_index*3] = point[0];
	clothing.vv_[m_index*3+1] = point[1];
	clothing.vv_[m_index*3+2] = point[2];
}

inline
DFPoint& DFPoint::operator = ( const DFPoint& p )
{
	m_index = p.m_index;
	m_force = p.m_force;
	m_forces_counter = p.m_forces_counter;
    return *this;
}

inline
DFMath::Vector3 DFPoint::CalcV( float dt )
{
	return m_force * dt;
}

inline
DFMath::Vector3 DFPoint::CalcS( float dt )
{
	return CalcV( dt ) * dt;
}

#endif
