#include "DFPhysicalParams.h"
#include <ostream>

std::ostream& DF::operator<< (std::ostream &out, const DF::PhysicalParam &phys_param)
{
    out << "{";
    out << "name=" << phys_param.name() << ", ";
    out << "stretch_structural=" << phys_param.stretch_structural() << ", ";
    out << "stretch_stiffness=" << phys_param.stretch_stiffness() << ", ";
    out << "bending=" << phys_param.bending() << ", ";
    out << "buckling=" << phys_param.buckling() << ", ";
    out << "internal_damping=" << phys_param.internal_damping() << ", ";
    out << "density=" << phys_param.density() << ", ";
    out << "friction=" << phys_param.friction() << ", ";
    out << "air_pressure=" << phys_param.air_pressure() << ", ";
    out << "freefall_acelleration=" << phys_param.freefall_acelleration() << ", ";
    out << "shift=" << phys_param.shift() << "";
    out << "}";
    return out;
}
