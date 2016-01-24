#include "DFMassSpringObject.h"
#include "DFPoint.h"
#include "tinyxml.h"
#include <ctime>

bool DFMassSpringObject::ShakeOutPoint(DF::Index point_index_3, DFMath::Vector3 move_vec, const DFMassSpringObject &ingot)
{
	bool rt = false;
//     int parts = 1;//static_cast<int>(DFMath::length(move_vec) / shift_) + 1;
//     DFMath::Vector3 part_vec = move_vec;

	const int SizePoints = 25;
	DFMath::Vector3 points[SizePoints];
// 	int levels[SizePoints] = {-1};
	int countPoints = 0;

    DFMath::Vector3 tangential_component(static_cast<DF::FloatingPointType>(0));
    DFMath::Vector3 normal_component(static_cast<DF::FloatingPointType>(0));
    bool detect_status;
	DFMath::Vector3 last_penetration;
//     for (ObjectsArray::const_iterator mesh_deque_cur = mesh_deque_end - 1; mesh_deque_cur >= mesh_deque_begin; mesh_deque_cur--) {
		detect_status = false;
		DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);

//		DFMath::Vector3 point0 = point;
		detect_status = ingot.NearSurface( point, -move_vec, false );
		if( detect_status ) {
			// точка переместилась до препятствия
			rt = true;
			points[countPoints] = point;
			if( countPoints+1 < SizePoints )
				countPoints ++;
		}
		/*
		//else {
			detect_status = ingot.NearSurface( point, move_vec, false );
			if( detect_status  ) {
				// точка переместилась до препятствия
				rt = true;
				points[countPoints] = point;
				if( countPoints+1 < SizePoints )
					countPoints ++;
			}
		//}
		*/
// 	}
	if( countPoints ) {
		last_penetration.set(points[countPoints-1]);
		
		
	}
	if( rt ) {
		// только если точка переместилась
		vv_[point_index_3]   = last_penetration[0];
		vv_[point_index_3+1] = last_penetration[1];
		vv_[point_index_3+2] = last_penetration[2];
	}
	return rt;
}

bool DFMassSpringObject::ShakeOutPoint(DF::Index point_index_3, DFMath::Vector3 move_vec, const DFMassSpringObject &ingot, bool )
{
	bool rt = false;

	const int SizePoints = 25;
	DFMath::Vector3 points[SizePoints];
//	int levels[SizePoints] = {-1};
	int countPoints = 0;

    DFMath::Vector3 tangential_component(DF::zero);
    DFMath::Vector3 normal_component(DF::zero);
    bool detect_status;
    DFMath::Vector3 last_penetration(DF::zero);
	detect_status = false;
	DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);

//	DFMath::Vector3 point0 = point;
	detect_status = ingot.NearSurface( point, -move_vec, false );
	if( detect_status  ) {
		// точка переместилась до препятствия изнутри
		rt = true;
		points[countPoints] = point;
		if( countPoints+1 < SizePoints )
			countPoints ++;
	}
	//else {
	
		detect_status = ingot.NearSurface( point, move_vec, true );
		if( detect_status  ) {
			// точка переместилась до препятствия снаружи
			rt = true;
			points[countPoints] = point;
			if( countPoints+1 < SizePoints )
				countPoints ++;
		}

	//}
	if( countPoints ) {
		DFMath::Vector3 p = points[0];
		for( int i=1; i<countPoints; i++ )
			p += points[i];
		p /= static_cast<DF::FloatingPointType>(countPoints);
		last_penetration.set(p);
//		last_penetration.set(points[countPoints-1]);
		
		
	}
	if( rt ) {
		// только если точка переместилась
		vv_[point_index_3]   = last_penetration[0];
		vv_[point_index_3+1] = last_penetration[1];
		vv_[point_index_3+2] = last_penetration[2];
	}
	return rt;
}

void DFMassSpringObject::ShakeOutPoint0(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end)
{
//     DF::FloatingPointType factual_movement_forward = MovePoint(point_index_3, move_vec * (shift_ * static_cast<DF::FloatingPointType>(29.9)), mesh_deque_begin, mesh_deque_end, static_cast<DF::FloatingPointType>(0.1));
//     DF::FloatingPointType factual_movement_backward = MovePoint(point_index_3, move_vec * (-factual_movement_forward), mesh_deque_begin, mesh_deque_end, static_cast<DF::FloatingPointType>(-0.3));

    int parts = static_cast<int>(DFMath::length(move_vec) / phys_param.shift()) + 1;
    DFMath::Vector3 part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
//     DFMath::Vector3 cur_normal(vn_[point_index_3], vn_[point_index_3+1], vn_[point_index_3+2]);
//     point -= cur_normal * 3;
    DFMath::Vector3 point0(point);
    DFMath::Vector3 last_penetration(point);
    DFMath::Vector3 tangential_component(static_cast<DF::FloatingPointType>(0));
    DFMath::Vector3 normal_component(static_cast<DF::FloatingPointType>(0));
    DetectStatus detect_status = PENETRATION_DETECTED;
    DetectStatus detect_status_last = NOT_DETECTED;
    for (int i = 0; i < parts; i++)
    {
        detect_status_last = detect_status;
        detect_status = NOT_DETECTED;
//         for (ObjectsArray::const_iterator mesh_deque_cur = mesh_deque_end - 1; mesh_deque_cur >= mesh_deque_begin && detect_status != CLASH_DETECTED; mesh_deque_cur--)
        for (ObjectsArray::const_iterator mesh_deque_cur = mesh_deque_end - 1; mesh_deque_cur >= mesh_deque_begin && detect_status == NOT_DETECTED; mesh_deque_cur--)
        {
            detect_status = mesh_deque_cur->DecomposeMovement(point, phys_param, part_vec, tangential_component, normal_component, static_cast<DF::FloatingPointType>(0.1));
#           ifdef _DEBUG
            if (mesh_deque_cur == mesh_deque_begin)
                break;
#           endif
        }
        point += normal_component + tangential_component;
        if (detect_status != PENETRATION_DETECTED && detect_status_last == PENETRATION_DETECTED)
        {
            last_penetration.set(point);
        }
    }
    vv_[point_index_3]   = last_penetration[0];
    vv_[point_index_3+1] = last_penetration[1];
    vv_[point_index_3+2] = last_penetration[2];
//     if (mesh_deque_end - 1 != mesh_deque_begin)
//     {
//         vv_[point_index_3]   = point[0];
//         vv_[point_index_3+1] = point[1];
//         vv_[point_index_3+2] = point[2];
//     }

//     return DFMath::length(point - point0);
}

void DFMassSpringObject::ShakeOutPoint2(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end)
{
    //     DF::FloatingPointType factual_movement_forward = MovePoint(point_index_3, move_vec * (shift_ * static_cast<DF::FloatingPointType>(29.9)), mesh_deque_begin, mesh_deque_end, static_cast<DF::FloatingPointType>(0.1));
    //     DF::FloatingPointType factual_movement_backward = MovePoint(point_index_3, move_vec * (-factual_movement_forward), mesh_deque_begin, mesh_deque_end, static_cast<DF::FloatingPointType>(-0.3));

    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
    DF::Vector3Vector penetration_points;
    penetration_points.push_back(point);
    point += move_vec;
    move_vec *= -1;
    int parts = static_cast<int>(DFMath::length(move_vec) / phys_param.shift()) + 1;
    DFMath::Vector3 part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    DFMath::Vector3 tangential_component(static_cast<DF::FloatingPointType>(0));
    DFMath::Vector3 normal_component(static_cast<DF::FloatingPointType>(0));
    DetectStatus detect_status = NOT_DETECTED;
    for (int i = 0; i < parts; i++)
    {
        for (ObjectsArray::const_iterator mesh_deque_cur = mesh_deque_end - 1; mesh_deque_cur >= mesh_deque_begin && detect_status == NOT_DETECTED; mesh_deque_cur--)
        {
            detect_status = mesh_deque_cur->DecomposeMovement(point, phys_param, part_vec, tangential_component, normal_component, static_cast<DF::FloatingPointType>(0.1));
#           ifdef _DEBUG
            if (mesh_deque_cur == mesh_deque_begin)
                break;
#           endif
        }
        if (DFMath::length(tangential_component) > 2 || DFMath::length(normal_component) > 2)
            log << point_index_3 << "   " << DFMath::length(tangential_component) << "   " << DFMath::length(normal_component) << "\n";
        point += normal_component + tangential_component;
        if (detect_status == CLASH_DETECTED)
        {
            penetration_points.push_back(point);
        }
    }
    //     if (penetration_points.size() > 1)
    //     {
    //         vv_[point_index_3]   = penetration_points[1][0];
    //         vv_[point_index_3+1] = penetration_points[1][1];
    //         vv_[point_index_3+2] = penetration_points[1][2];
    //     }
    //     else
    {
        vv_[point_index_3]   = penetration_points.back()[0];
        vv_[point_index_3+1] = penetration_points.back()[1];
        vv_[point_index_3+2] = penetration_points.back()[2];
        //         log << parts << "\t" << penetration_points.size() << "\n";
    }

    //     return DFMath::length(point - point0);
}

size_t GetLastGroupOuterIndex(const std::vector<ObjectsArray::const_iterator> &mesh_iterators_vector, const DF::Vector3Vector &mesh_points_vector, const DF::FloatingPointType max_delta_2)
{
//     if (mesh_iterators_vector.size() != mesh_points_vector.size())
//         return mesh_iterators_vector.begin();
    int ret_index = static_cast<int>(mesh_iterators_vector.size()) - 1;
    for (; ret_index > 0; ret_index--)
    {
        if (DFMath::length2(mesh_points_vector[ret_index] - mesh_points_vector[ret_index-1]) > max_delta_2)
            break;
    }
    return ret_index;
}

void DFMassSpringObject::ShakeOutPoint3(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end)
{
//     DF::FloatingPointType factual_movement_forward = MovePoint(point_index_3, move_vec * (shift_ * static_cast<DF::FloatingPointType>(29.9)), mesh_deque_begin, mesh_deque_end, static_cast<DF::FloatingPointType>(0.1));
//     DF::FloatingPointType factual_movement_backward = MovePoint(point_index_3, move_vec * (-factual_movement_forward), mesh_deque_begin, mesh_deque_end, static_cast<DF::FloatingPointType>(-0.3));

    DF::Vector3Vector last_penetration;
    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
    DFMath::Vector3 point0(point);
    last_penetration.push_back(point);
    // TODO: некорректный поиск столкновения: последним столкновением останется внутренний, а не внешний слой
    std::vector<ObjectsArray::const_iterator> mesh_deque_iterators_list;
    mesh_deque_iterators_list.push_back(mesh_deque_begin);
    DFMath::Vector3 point_new(point);
    point += move_vec;
//     last_penetration.push_back(point);
    move_vec *= -1;
    int parts = static_cast<int>(DFMath::length(move_vec) / phys_param.shift()) + 1;
    DFMath::Vector3 part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    DFMath::Vector3 tangential_component(static_cast<DF::FloatingPointType>(0));
    DFMath::Vector3 normal_component(static_cast<DF::FloatingPointType>(0));
    DetectStatus detect_status = CLASH_DETECTED;
    DetectStatus detect_status_prev;
    ObjectsArray::const_iterator mesh_deque_cur = mesh_deque_begin;
    for (int i = 0; i < parts * 2; i++)
    {
        detect_status_prev = detect_status;
        detect_status = NOT_DETECTED;
        for (mesh_deque_cur = mesh_deque_end - 1; mesh_deque_cur >= mesh_deque_begin && detect_status != CLASH_DETECTED && detect_status != SLIDING_DETECTED; mesh_deque_cur--)
        {
            detect_status = mesh_deque_cur->DecomposeMovement(point, phys_param, part_vec, tangential_component, normal_component, static_cast<DF::FloatingPointType>(0.1));
#           ifdef _DEBUG
            if (mesh_deque_cur == mesh_deque_begin)
                break;
#           endif
        }
        point_new = point + normal_component + tangential_component;
//         point += normal_component + tangential_component;
        if ((detect_status == CLASH_DETECTED/* || detect_status == SLIDING_DETECTED*/) && detect_status_prev != CLASH_DETECTED/* && detect_status_last != SLIDING_DETECTED*//* &&
            DFMath::length2(normal_component) / DFMath::length2(tangential_component) > static_cast<DF::FloatingPointType>(0.0001)*/
//             && mesh_deque_iterators_list.back() < mesh_deque_cur
            )
        {
            if (i >= parts)
            {
                if (DFMath::length2(point0 - point_new) < DFMath::length2(last_penetration.back() - point0)/* && GetLastGroupOuterIndex(mesh_deque_iterators_list, last_penetration, DF::sqr(static_cast<DF::FloatingPointType>(shift_ + 1e-2))) <= mesh_deque_cur - mesh_deque_begin*/)
                {
                    last_penetration.push_back(point0);
                    mesh_deque_iterators_list.push_back(mesh_deque_end);
                }
                break;
            }
            last_penetration.push_back(point_new);
            mesh_deque_iterators_list.push_back(mesh_deque_cur);
        }
        point += part_vec;
    }

    // фильтрация результата
    size_t i = last_penetration.size() - 1;
//     ObjectsArray::const_iterator mesh_deque_iter_max = mesh_deque_iterators_list[i];
//     for (; (i > 0) && (DFMath::length(last_penetration[i] - last_penetration[i-1]) <= shift_ + 1e-2); i--)
//     {
//         
//     }
//     size_t qwe = GetLastGroupOuterIndex(mesh_deque_iterators_list, last_penetration, DF::sqr(static_cast<DF::FloatingPointType>(shift_ + 1e-2)));
    vv_[point_index_3]   = last_penetration[i][0];
    vv_[point_index_3+1] = last_penetration[i][1];
    vv_[point_index_3+2] = last_penetration[i][2];
//     vv_[point_index_3]   = last_penetration[qwe][0];
//     vv_[point_index_3+1] = last_penetration[qwe][1];
//     vv_[point_index_3+2] = last_penetration[qwe][2];
//     vv_[point_index_3]   = point[0];
//     vv_[point_index_3+1] = point[1];
//     vv_[point_index_3+2] = point[2];

//     return DFMath::length(point - point0);
}

void DFMassSpringObject::ShakeOutPoint4(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const DFMassSpringObject &ingot)
{
    const int additional_internal_steps = 20;
    int parts = static_cast<int>(DFMath::length(move_vec) / phys_param.shift()) + 1;
    DFMath::Vector3 part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
    point -= part_vec;
//     DFMath::Vector3 cur_normal(vn_[point_index_3], vn_[point_index_3+1], vn_[point_index_3+2]);
//     point -= cur_normal * 3;
    DFMath::Vector3 point0(point);
    DFMath::Vector3 tangential_component(static_cast<DF::FloatingPointType>(0));
    DFMath::Vector3 normal_component(static_cast<DF::FloatingPointType>(0));
    DetectStatus detect_status = NOT_DETECTED;
    DetectStatus detect_status_last = NOT_DETECTED;
    bool one_penetration_solved = false;
    // исправляем коллизию, если она имеется изначально
    do
    {
        detect_status = ingot.DecomposeMovement(point, phys_param, part_vec, tangential_component, normal_component, static_cast<DF::FloatingPointType>(0.1));
        point += normal_component + tangential_component;
        if (detect_status != PENETRATION_DETECTED && detect_status != PENETRATION_SOLVED)
            break;
        one_penetration_solved = true;
    } while (true);
    DFMath::Vector3 last_penetration(point);

    // если коллизии не было обнаружено, то двигаемся вглубь
    bool internal_layer_found = false;
    if (!one_penetration_solved)
    {
        for (int i = 0; i < additional_internal_steps; i++)
        {
            detect_status = ingot.DecomposeMovement(point, phys_param, -part_vec, tangential_component, normal_component, static_cast<DF::FloatingPointType>(0.1));
            if (detect_status == SLIDING_DETECTED || detect_status == CLASH_DETECTED)
            {
                internal_layer_found = true;
                break;
            }
            point += normal_component + tangential_component;
        }
    }

    // если внутри слоя не нашлось, то двигаемся наружу до первого исправления коллизии
    point.set(last_penetration);
    if (!internal_layer_found)
    {
        detect_status = PENETRATION_DETECTED;
        detect_status_last = NOT_DETECTED;
        for (int i = 0; i < parts; i++)
        {
            detect_status_last = detect_status;
            detect_status = ingot.DecomposeMovement(point, phys_param, part_vec, tangential_component, normal_component, static_cast<DF::FloatingPointType>(0.1));
            if (detect_status == CLASH_DETECTED)
                break;
            point += normal_component + tangential_component;
            if (detect_status == PENETRATION_SOLVED && detect_status_last == PENETRATION_DETECTED)  // точка вышла наружу некоей поверхности, и проникновение более не детектируется
            {
                last_penetration.set(point);
                break;
            }
        }
    }
    vv_[point_index_3]   = last_penetration[0];
    vv_[point_index_3+1] = last_penetration[1];
    vv_[point_index_3+2] = last_penetration[2];
//     vv_[point_index_3]   = point[0];
//     vv_[point_index_3+1] = point[1];
//     vv_[point_index_3+2] = point[2];

}

DFMassSpringObject::DetectStatus DFMassSpringObject::DecomposeMovement(DFMath::Vector3 specified_point, const DF::PhysicalParam &phys_param, const DFMath::Vector3 &vec, DFMath::Vector3 &tangential_vec, DFMath::Vector3 &normal_vec, DF::FloatingPointType cos_lim) const
{
    DF::Index nearest_point_index = static_cast<DF::Index>(-1);
    tangential_vec.set(vec);
    normal_vec.set(static_cast<DF::FloatingPointType>(0));
    DFMath::FloatingPointType vec_length = DFMath::length(vec);
    if (vec_length < 1e-4)
        return NOT_DETECTED;
    DF::FloatingPointType scalar_correction = 0;
//     DFMath::Vector3 vect_correction(0.);
    DFMath::Vector3 nearest_point(0.);
    DFMath::Vector3 nearest_normal(0.);
    specified_point += vec;
    bool found = SimpleFindNearestArea(nearest_point_index, nearest_point, nearest_normal, specified_point, 1) == DF::err_ok;
    if (found)
    {
        DFMath::Vector3 radius_vect(nearest_point - specified_point);
        scalar_correction = DFMath::ScalarProduct(nearest_normal, radius_vect);
        scalar_correction += phys_param.shift();
        DF::FloatingPointType Vn = DFMath::ScalarProduct(nearest_normal, vec);
        normal_vec.set(nearest_normal);
        normal_vec *= Vn;
        tangential_vec -= normal_vec;
        if (scalar_correction < 0)  // точка над поверхностью достаточно далеко
        {
            return NOT_DETECTED;
        }
        else  // точка под поверхностью или слишком близко
        {
            if (Vn / vec_length > cos_lim)  // точка движется наружу
            {
                if (scalar_correction < phys_param.shift())  // выходит
                {
                    normal_vec += nearest_normal * scalar_correction;
                    return PENETRATION_SOLVED;
                }
                else  // еще глубоко
                {
                    return PENETRATION_DETECTED;
                }
            }
            else  // точка движется внутрь
            {
                normal_vec += nearest_normal * scalar_correction;
                DFMath::Vector3 tangential_vec_temp(tangential_vec);
                if (Vn > 1e-4)
                    tangential_vec_temp *= (Vn + scalar_correction) / Vn;
                else
                    tangential_vec_temp.set(static_cast<DF::FloatingPointType>(0));
                tangential_vec -= tangential_vec_temp;
                DF::FloatingPointType F_tr = scalar_correction * phys_param.friction();
                DF::FloatingPointType Vt = DFMath::length(tangential_vec);
                if (F_tr < Vt)  // сила трения меньше силы
                {
                    if (Vt > 1e-4)
                        tangential_vec *= (Vt - F_tr) / Vt;
                    tangential_vec += tangential_vec_temp;
                    return SLIDING_DETECTED;
                }
                else  // сила трения больше силы, гасит
                {
                    tangential_vec.set(tangential_vec_temp);
                    return CLASH_DETECTED;
                }
            }
        }
    }
    return NOT_DETECTED;  // поверхность далеко за пределами области поиска
}

DF::FloatingPointType DFMassSpringObject::MovePointFull(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const DFMassSpringObject &ingot, DF::FloatingPointType /*maxLen*/)
{
    int parts = static_cast<int>(DFMath::length(move_vec) / phys_param.shift()) + 1;
    DFMath::Vector3 part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
    DFMath::Vector3 point0(point);
	bool penetration_detected = false;
	for (int i = 0; i < parts; i++)
	{
		penetration_detected = false;
		penetration_detected = ingot.Detect(point, phys_param, part_vec, /*0.3*/0.0);
		if (penetration_detected)
			break;
		else
			point += part_vec;
	}
//	if( DFMath::length(point - point0) <= maxLen ) {
		vv_[point_index_3] = point[0];
		vv_[point_index_3+1] = point[1];
		vv_[point_index_3+2] = point[2];
//	}
	return DFMath::length(point - point0);
}

DF::FloatingPointType DFMassSpringObject::MovePoint(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const ObjectsArray& meshs, size_t dress_index, DF::FloatingPointType cos_lim, bool surface)
{
    DF::FloatingPointType move_vec_length = DFMath::length(move_vec);
    const DF::FloatingPointType& shift = phys_param.shift();
    int parts = static_cast<int>(move_vec_length / shift) + 1;
    DFMath::Vector3 part_vec(DF::zero);
    static const int max_move_steps = 5;
    if( parts > max_move_steps ) {
        parts = max_move_steps;
        part_vec = move_vec * (static_cast<DF::FloatingPointType>(5) * shift / move_vec_length);
    } else {
        part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    }
    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
    DFMath::Vector3 point0(point);
	if( surface ) {
		bool penetration_detected = false;
		for (int i = 0; i < parts; i++)
		{
			penetration_detected = false;
//			for (ObjectsArray::const_iterator curMesh = meshs.begin() + dress_index-1; curMesh >= meshs.begin() && !penetration_detected; curMesh--) {
			for (int k = static_cast<int>(dress_index)-1; k >= 0 && !penetration_detected; k--) {
				const DFMassSpringObject& curMesh = meshs[k];
				penetration_detected = curMesh.Detect(point, phys_param, part_vec, cos_lim);
			}
			if (penetration_detected) {
//				point += part_vec*2/3;
				point += part_vec/2;
				break;
			}
			else
				point += part_vec;
		}
	}
	else
		point += move_vec;
    vv_[point_index_3] = point[0];
    vv_[point_index_3+1] = point[1];
    vv_[point_index_3+2] = point[2];
    return DFMath::length(point - point0);
}

DF::FloatingPointType DFMassSpringObject::MovePointWithFriction(DF::Index point_index_3, const DF::PhysicalParam &phys_param, DFMath::Vector3 move_vec, const ObjectsArray& meshs, size_t dress_index, DF::FloatingPointType cos_lim)
{
    DF::FloatingPointType move_vec_len = DFMath::length(move_vec);
    int parts = static_cast<int>(move_vec_len / phys_param.shift()) + 1;
    static const int max_move_steps = 5;
    if( parts > max_move_steps )
        parts = max_move_steps;
    DFMath::Vector3 part_vec = move_vec / static_cast<DF::FloatingPointType>(parts);
    DFMath::Vector3 point(vv_[point_index_3], vv_[point_index_3+1], vv_[point_index_3+2]);
    DFMath::Vector3 point0(point);
    DFMath::Vector3 tangential_component(static_cast<DF::FloatingPointType>(0));
    DFMath::Vector3 normal_component(static_cast<DF::FloatingPointType>(0));
	bool penetration_detected = false;

    for (int i = 0; i < parts; i++)
	{
		penetration_detected = false;
        for (int k = static_cast<int>(dress_index)-1; k >= 0 && !penetration_detected; k--) {
            const DFMassSpringObject& curMesh = meshs[k];
            DFMesh::DetectStatus status = curMesh.DecomposeMovement(point, phys_param, part_vec, tangential_component, normal_component, cos_lim);
            penetration_detected = status == CLASH_DETECTED || status == SLIDING_DETECTED;  // SLIDING_DETECTED ???
            point += tangential_component + normal_component;
        }
        if (penetration_detected)
            break;
	}
    DF::FloatingPointType fact_move_len = DFMath::length(point - point0);
    if (fact_move_len <= move_vec_len + 1) {
        vv_[point_index_3] = point[0];
        vv_[point_index_3+1] = point[1];
        vv_[point_index_3+2] = point[2];
    }
    else
    {
        point = (point - point0) / fact_move_len;  // нормализуем
        vv_[point_index_3] += point[0];
        vv_[point_index_3+1] += point[1];
        vv_[point_index_3+2] += point[2];
        log << "WARNING: (fact_move_len > move_vec_len + 1), move_vec_len = " << move_vec_len << ", fact_move_len = " << fact_move_len << '\n';
    }
    return fact_move_len;
}

bool DFMassSpringObject::Detect(DFMath::Vector3 specified_point, const DF::PhysicalParam &phys_param, DFMath::Vector3 vec, DF::FloatingPointType cos_lim) const
{
    DF::Index nearest_point_index = static_cast<DF::Index>(-1);
    DF::FloatingPointType scalar_correction = 0;
    DFMath::Vector3 nearest_point(0.);
    DFMath::Vector3 nearest_normal(0.);
    specified_point += vec;
    bool found = SimpleFindNearestArea(nearest_point_index, nearest_point, nearest_normal, specified_point, 1) == DF::err_ok;
    if (found) {
        DFMath::Vector3 radius_vect(nearest_point - specified_point);
        scalar_correction = DFMath::ScalarProduct(nearest_normal, radius_vect);
        scalar_correction += phys_param.shift();
        if (scalar_correction < 0) {
			// точка над поверхностью достаточно далеко
            return false;
        }
        else {
			// точка под поверхностью или слишком близко
            DF::FloatingPointType vec_norm_projection = DFMath::ScalarProduct(nearest_normal, vec) / DFMath::length(vec);
            if (vec_norm_projection > cos_lim) {
				// точка движется наружу
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

bool DFMassSpringObject::NearSurface( DFMath::Vector3& point, DFMath::Vector3 direction, bool in ) const
// true - если точка пытается пройти сквозь препятствие по направлению вектора direction
{
	// нашли препятствие - перемещаемся к нему как можно ближе
	const int count = 10;
	direction /= count;
	DF::FloatingPointType minDist = 1000000;
	bool rt = false;
	
	DFMath::Vector3 pointTemp = point;
	for( int i=0; i<count; i++ ) {
		DF::Index nearest_point_index = static_cast<DF::Index>(-1);
		DF::Error_t err = SimpleFindNearestInEnvironment( nearest_point_index, pointTemp+direction, 4 );
		if( err == DF::err_ok ) {
			DFMath::Vector3 nearPoint(vv_[nearest_point_index], vv_[nearest_point_index + 1], vv_[nearest_point_index + 2]);
			DF::FloatingPointType dist = DFMath::length2( nearPoint-pointTemp ); 
			if( minDist >= dist ) {
				minDist = dist;
				rt |= true;
				pointTemp += direction;
				point = pointTemp;
			}
		}
		//if( in )
			pointTemp += direction;
	}
	if( in )
		pointTemp -= direction*4;
	return rt;
}

DFMassSpringObject::DFMassSpringObject(const DFMesh& init_mesh)
    : DFMesh(&init_mesh, DFMesh::COPY_ALL)
{
    Init();
}

DFMassSpringObject::DFMassSpringObject()
{
    Init();
}

DFMassSpringObject::~DFMassSpringObject()
{
    delete[] indexs_; indexs_ = 0;
    delete[] newIndexs_; newIndexs_ = 0;
	delete[] m_points; m_points = 0;
}

void DFMassSpringObject::Init()
{
    physical_params_.resize(1);
    time_step_ = static_cast<DF::FloatingPointType>(0.1);

    indexs_ = 0;
    newIndexs_ = 0;
    maxCount_ = 0;
    pos_ = 0;
    count_ = 0;
	m_points = 0;
}

DF::Error_t DFMassSpringObject::Tighten(DF::FloatingPointType stiffness_coefficient, ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end, DF::FloatingPointType /*max_dist_2*/, DF::FloatingPointType shift, size_t iteration_num)
{
    if (incidence_.size() == 0)
        return DF::err_not_enough_elements;

    log << "[" << DF::GetCurrentTimeString() << "] Tighten started\n";
    clock_t start = clock();

    DF::FloatingPointType cur_length;
    DF::FloatingPointType spring_coeff;
    DFMath::Vector3 point1, point2, arc;
    std::vector<DFMath::Vector3> forces(vv_.size() / 3);
    std::vector<size_t> counter(vv_.size() / 3);
    DF::FloatingPointType scalar_correction = 0;
    DFMath::Vector3 vect_correction(0.);

    for (size_t i1 = 0; i1 < iteration_num; i1++)
    {
        for (size_t i2 = 0; i2 < 10; i2++)
        {
            // TODO: условие завершения
            forces.resize(0);
            forces.resize(vv_.size() / 3, DFMath::Vector3(0.));

            for (size_t i_cur_point = 0; i_cur_point < incidence_.size(); i_cur_point++)
            {
                DF::IncedenceVector &cur_incedence_vector = incidence_[i_cur_point];
                for (DF::IncedenceVector::iterator it_cur_linked = cur_incedence_vector.begin(); it_cur_linked < cur_incedence_vector.end(); it_cur_linked++)
                {
                    point1.set(vv_[i_cur_point * 3], vv_[i_cur_point * 3 + 1], vv_[i_cur_point * 3 + 2]);
                    point2.set(vv_[it_cur_linked->index], vv_[it_cur_linked->index + 1], vv_[it_cur_linked->index + 2]);
                    arc.set(point2 - point1);
                    cur_length = DFMath::length(arc);
                    if (cur_length < 1e-3)
                        continue;
                    spring_coeff = (cur_length - it_cur_linked->distance * stiffness_coefficient) / cur_length;
                    forces[i_cur_point] += arc * spring_coeff;
                    counter[i_cur_point]++;
                }
            }

            for (size_t i = 0; i < forces.size(); i++)
            {
                if (counter[i] == 0) continue;
                forces[i] /= static_cast<DF::FloatingPointType>(counter[i]);
                vv_[i * 3] += forces[i][0];
                vv_[i * 3 + 1] += forces[i][1];
                vv_[i * 3 + 2] += forces[i][2];
            }
        }
        for (size_t i = 0; i < forces.size(); i++)
        {
            point1.set(vv_[i * 3], vv_[i * 3 + 1], vv_[i * 3 + 2]);
            for (ObjectsArray::const_iterator dress_it = mesh_deque_end - 1; dress_it >= mesh_deque_begin; dress_it--)
            {
                scalar_correction = 0;
                if (dress_it->DetectPenetration(scalar_correction, vect_correction, point1))
                {
                    scalar_correction += shift;
                    if (scalar_correction > 0)
                        point1 += vect_correction * scalar_correction;
                    break;
                }
#               ifdef _DEBUG
                if (dress_it == mesh_deque_begin)
                    break;
#               endif
            }
            vv_[i * 3] = point1[0];
            vv_[i * 3 + 1] = point1[1];
            vv_[i * 3 + 2] = point1[2];
            if (vv_[i * 3 + 1] < 0)
                vv_[i * 3 + 1] = 0;
        }
    }

    clock_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Tighten finished with status: " << DF::print_error(DF::err_ok) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";

    return DF::err_ok;
}

DF::Error_t DFMassSpringObject::Tighten(DF::FloatingPointType stiffness_coefficient, const DFMassSpringObject &mesh, DF::FloatingPointType max_dist_2, DF::FloatingPointType shift, size_t iteration_num)
{
    ObjectsArray mesh_deque;
    mesh_deque.push_back(mesh);
    return Tighten(stiffness_coefficient, mesh_deque.begin(), mesh_deque.begin() + 1, max_dist_2, shift, iteration_num);
}

void DFMassSpringObject::Release()
{
    DFMesh::Release();
    DF::ReleaseSTLContainer(stored_states_);
}

size_t DFMassSpringObject::GetMemoryUsage() const
{
    size_t prev_states_size = DF::GetMemoryUsage(stored_states_);
    return DFMesh::GetMemoryUsage() + prev_states_size;
}

void DFMassSpringObject::SetTimeStep(const DF::FloatingPointType new_time_step)
{
    time_step_ = new_time_step;
}

DF::FloatingPointType DFMassSpringObject::GetTimeStep() const
{
    return time_step_;
}

DF::Error_t DFMassSpringObject::LoadPhysParams(const std::string &fn)
{
    TiXmlDocument DocXML(fn.c_str());
    if (!DocXML.LoadFile()) 
        return DF::err_file_io_problem;

    if (TiXmlElement *pMaterialsParent = DocXML.FirstChildElement("materials")) 
    {
        physical_params_.resize(0);
        for (TiXmlElement *pCurMaterial = pMaterialsParent->FirstChildElement("material"); pCurMaterial; pCurMaterial = pCurMaterial->NextSiblingElement())
        {
            DF::ExternalPhysicalParam cur_param;
            cur_param.name = std::string(pCurMaterial->Attribute("name"));
            for (TiXmlElement *pCurProperty = pCurMaterial->FirstChildElement("property"); pCurProperty; pCurProperty = pCurProperty->NextSiblingElement())
            {
                std::string param_name(pCurProperty->Attribute("name"));
                double param_val = 0;
                pCurProperty->Attribute("value", &param_val);
                if (param_name == "StretchWeft")
                    cur_param.stretch_weft = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "StretchWarp")
                    cur_param.stretch_warp = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "Shear")
                    cur_param.stretch_shear = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "Bending")
                    cur_param.bending = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "Buckling")
                    cur_param.buckling = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "InternalDamping")
                    cur_param.internal_damping = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "Density")
                    cur_param.density = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "Friction")
                    cur_param.friction = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "AirPressure")
                    cur_param.air_pressure = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "FreefallAcelleration")
                    cur_param.freefall_acelleration = static_cast<DF::FloatingPointType>(param_val);
                else if (param_name == "Thickness")
                    cur_param.shift = static_cast<DF::FloatingPointType>(param_val);
            }
            physical_params_.push_back(cur_param);
        }
    }
    if (physical_params_.size() == 0)
        physical_params_.resize(1);
    return DF::err_ok;
}

DF::Error_t DFMassSpringObject::StoreState(const DF::Index n)
{
    while (stored_states_.size() >= n)
        stored_states_.erase(stored_states_.begin());
    while (stored_states_.size() < n)
        stored_states_.push_back(vv_);
    return DF::err_ok;
}

DF::Error_t DFMassSpringObject::GetDifferent(const DF::Index state_num, const DF::Index i_3, DF::FloatingPointType &different)
{
    if (i_3 >= vv_.size())
        return DF::err_not_enough_elements;
    different = stored_states_[state_num + 1][i_3] - stored_states_[state_num][i_3];
    return DF::err_ok;
}

DF::Error_t DFMassSpringObject::SolveCollizion(ObjectsArray::const_iterator mesh_deque_begin, ObjectsArray::const_iterator mesh_deque_end, const DF::FloatingPointType shift)
{
    if (mesh_deque_end <= mesh_deque_begin) {
        log << '[' << DF::GetCurrentTimeString() << "] SolveCollizion(): " << DF::print_error(DF::err_not_enough_elements) << " (mesh_deque_end <= mesh_deque_begin)\n";
        return DF::err_not_enough_elements;
    }

    log << "[" << DF::GetCurrentTimeString() << "] Pushing out started\n";
    time_t start = clock();

    for (size_t mesh2_v_i = 0; mesh2_v_i < vv_.size(); mesh2_v_i += 3)
    {
        DFMath::Vector3 vect_dress_point(vv_[mesh2_v_i], vv_[mesh2_v_i + 1], vv_[mesh2_v_i + 2]);
        DFMath::Vector3 vect_dress_normal(vn_[mesh2_v_i], vn_[mesh2_v_i + 1], vn_[mesh2_v_i + 2]);
        DFMath::Vector3 nearest_point(DF::zero);
        DFMath::Vector3 nearest_normal(DF::zero);
        DF::FloatingPointType scalar_correction = DF::zero;
        DF::FloatingPointType len = DF::zero;
        DF::Index nearest_point_index = static_cast<DF::Index>(-1);
        DFMath::Vector3 vect_dress_body(DF::zero);

        for (ObjectsArray::const_iterator mesh_it = mesh_deque_begin; mesh_it < mesh_deque_end; mesh_it++)
        {
//             DF::FloatingPointType factor = 1;
//             do
//             {
                if (mesh_it->SimpleFindNearestArea(nearest_point_index, nearest_point, nearest_normal, vect_dress_point, 2) != DF::err_ok) 
                    break;
                vect_dress_body.set(nearest_point - vect_dress_point);
                scalar_correction = DFMath::ScalarProduct(nearest_normal, vect_dress_body);
                len = DFMath::length(vect_dress_body);
//                 if (len < 1e-2 || abs(DFMath::ScalarProduct(vect_dress_normal, vect_dress_body) / len) <= cos_delta_angle || abs(scalar_correction / len) < cos_delta_angle)
                if (len > 1.33 * shift)
                    break;
                scalar_correction += shift;
                if (scalar_correction > 0)
                    vect_dress_point += nearest_normal * scalar_correction;
//                 vect_dress_point += nearest_normal * (scalar_correction * factor);
//                 factor *= static_cast<DF::FloatingPointType>(0.5);  // h/2
//             } while (factor > 1e-2);
        }

        vv_[mesh2_v_i]     = vect_dress_point[0];
        vv_[mesh2_v_i + 1] = vect_dress_point[1];
        vv_[mesh2_v_i + 2] = vect_dress_point[2];
    }

    time_t stop = clock();
    log << "[" << DF::GetCurrentTimeString() << "] Pushing out finished with status: " << DF::print_error(DF::err_ok) << " (" << (static_cast<double>(stop - start) / CLOCKS_PER_SEC) << " sec)\n";

    return DF::err_ok;
}

