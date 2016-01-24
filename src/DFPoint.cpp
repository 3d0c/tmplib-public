#include "DFPoint.h"

#include <iostream>
#include <stdio.h>

bool DFPoint::IsMove( float dt, DF::FloatingPointType h )
{
	if( DFMath::length( CalcS( dt ) ) > h )
//	if( DFMath::length(m_force) > h )
		return true;
	return false;
}

void DFPoint::AddForce( const DFMath::Vector3& force )
{
	m_force += Mass*force;
}

void DFPoint::AddForcesFromLinks( DFMassSpringObject& clothing, DF::FloatingPointType force_rate, DF::FittingMode fitting_mode )
{
    (void)force_rate;
    (void)fitting_mode;
    size_t countForces = 0;
	const DF::PhysicalParam &cur_param = clothing.physical_params_[0];
    DFMath::Vector3 cur_point( GetVector3(clothing) );
    DF::IncedenceVector &cur_incedence_vector = clothing.incidence_[m_index];
	double sumLY = 0;
	DFMath::Vector3 force = GetForce();
	DFMath::Vector3 newForce( DF::zero );
    for (size_t linked_point_i = 0; linked_point_i < cur_incedence_vector.size(); linked_point_i++)
    {
		DF::IncedenceRecord& ir = cur_incedence_vector[linked_point_i];
        DF::FloatingPointType L0 = ir.distance;
        DFMath::Vector3 rib(GetVector3(clothing, ir.index) - cur_point);
        DF::FloatingPointType L = DFMath::length(rib);
        ir.distance_prev = L; // результат используется в ProcessDressStep3
        DF::FloatingPointType stiffness_coefficient = 0;
        switch (ir.type) {
        case DF::structural: stiffness_coefficient = cur_param.stretch_structural(); break;
        case DF::stiffness:  stiffness_coefficient = cur_param.stretch_stiffness();  break;
        }

        countForces++;
        if (L == 0.)
            continue;

		if( L > L0 )
			// суммируем растяжение по вертикали
			sumLY += rib[1];
		
		DF::FloatingPointType x = static_cast<DF::FloatingPointType>(1.0)-L0/L;
        if ( fitting_mode != DF::INITIAL && L > L0 && rib[1] > L0 ) {
            // компенсация силы тяжести - ускоренное стягивание вверх
			x *= 1.5;
		}
		newForce += rib * (x * stiffness_coefficient);
    }
	m_force += newForce/(DF::FloatingPointType)countForces;
	sumLY /= countForces;
	if( fitting_mode != DF::INITIAL && sumLY > 0 ) {
		// компенсация силы тяжести - ускоренное стягивание вверх
//		std::cout << "sumLY= " << sumLY << " f= " << abs(force[1]) << "\n";
		AddForce(  DFMath::Vector3( 0, abs(force[1]), 0 ) );
	}
}

bool DFPoint::DetectCollision( DFPoint& detectPoint, const DFMesh& clothing, const DF::PhysicalParam &phys_param, DFMath::Vector3 specified_point, DF::FloatingPointType cos_lim ) const
{
    DF::Index nearest_point_index = static_cast<DF::Index>(-1);
    DF::FloatingPointType scalar_correction = 0;
    DFMath::Vector3 nearest_point(0.);
    DFMath::Vector3 nearest_normal(0.);
    specified_point += m_force;
    bool found = clothing.SimpleFindNearestArea(nearest_point_index, nearest_point, nearest_normal, specified_point, 1) == DF::err_ok;
    if (found) {
        DFMath::Vector3 radius_vect(nearest_point - specified_point);
        scalar_correction = DFMath::ScalarProduct(nearest_normal, radius_vect);
		if( cos_lim != 0.45 )
			scalar_correction += phys_param.shift();
        if (scalar_correction < 0) {
			// точка над поверхностью достаточно далеко
            return false;
        }
        else {
			// точка под поверхностью или слишком близко
            DF::FloatingPointType vec_norm_projection = DFMath::ScalarProduct(nearest_normal, m_force) / DFMath::length(m_force);
            if (vec_norm_projection > cos_lim) {
				// точка движется наружу
				detectPoint.SetIndex( nearest_point_index/3 );
                return false;
            }
            else {
				// точка движется внутрь
                return true;
            }
        }
    }
    return false;  // поверхность далеко за пределами области поиска
}

DF::FloatingPointType DFPoint::MovePoint( DFMassSpringObject& clothing, float dt, const DF::PhysicalParam &phys_param, ObjectsArray& meshs, size_t dress_index, DF::FloatingPointType cos_lim, DFPoint::Movement movement, bool surface )
{
	DFMath::Vector3 s = CalcS( dt );
    DF::FloatingPointType move_vec_length = DFMath::length( s );
    const DF::FloatingPointType& shift = phys_param.shift();
    int parts = static_cast<int>(move_vec_length / shift) + 1;
    DFMath::Vector3 part_vec(DF::zero);
    static const int max_move_steps = 5;
    if( parts > max_move_steps ) {
        parts = max_move_steps;
        part_vec = s * (static_cast<DF::FloatingPointType>(5) * shift / move_vec_length);
    } else {
        part_vec = s / static_cast<DF::FloatingPointType>(parts);
    }
    DFMath::Vector3 point( GetVector3(clothing) );
    DFMath::Vector3 point0( point );
	if( surface ) {
		
		if( movement == Up ) {
			DFPoint detectPoint;
			detectPoint.SetIndex( -1 );
			DetectCollision( detectPoint, clothing, phys_param, point, 0.45 );
			if( detectPoint.GetIndex() != static_cast<DF::Index>(-1) ) {
//				std::cout << "ind= " << detectPoint.GetIndex() << "f= " << DFMath::length( detectPoint.GetForce() ) << "\n";
				DFPoint& dPoint = clothing.m_points[ detectPoint.GetIndex() ];
				dPoint.AddForce( part_vec );
//				std::cout << "len= " << DFMath::length( dPoint.GetVector3(clothing)-point ) << "f= " << DFMath::length( detectPoint.GetForce() ) << "\n";
				//detectPoint.MovePoint( meshs[k] );
			}
		}
		

		SetForce( part_vec );
		bool penetration_detected = false;
		for( int i = 0; i < parts; i++ ) {
			penetration_detected = false;
			for( int k = static_cast<int>(dress_index)-1; k >= 0 && !penetration_detected; k-- ) {
				DFPoint detectPoint;
				penetration_detected = DetectCollision( detectPoint, meshs[k], phys_param, point, cos_lim );
				/*
				if( k != 0 && detectPoint.GetIndex() != static_cast<DF::Index>(-1) ) {
					//std::cout << "ind= " << detectPoint.GetIndex() << "f= " << DFMath::length( detectPoint.GetForce() ) << "\n";
					DFPoint& dPoint = meshs[k].m_points[ detectPoint.GetIndex() ];
					dPoint.AddForce( part_vec );
					//detectPoint.MovePoint( meshs[k] );
				}
				*/
				if( penetration_detected ) {
					// если столкнулись с ближайшим слоем, то дальше не проверяем - выходим
					break;
				}
			}

			if( penetration_detected ) {
				point += part_vec/4;
				break;
			}
			else
				point += part_vec;
		}


	}
	else
		point += s;
	if( movement == Up || movement == Down )
		SetForce( DF::zero );
	SetVector3( clothing, point );
    return DFMath::length(point - point0);
}

void DFPoint::MovePoint( DFMesh& clothing )
{
    SetVector3( clothing, GetVector3(clothing)+m_force );
}

bool DFPoint::MovePointWithFriction( DFMesh& clothing, float dt )
{
    (void)clothing;
	return false;
}
