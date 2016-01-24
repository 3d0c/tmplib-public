#ifndef DFPHYSICALPARAMS_H
#define DFPHYSICALPARAMS_H

#include "DFDefinitions.h"

const float koefG = 9.8f;
const float koefFilterShift = 1.5f;

namespace DF {
    struct ExternalPhysicalParam
    {
        std::string name;                         // им€ материала
        FloatingPointType stretch_weft;           // упругость горизонтальна€ (сопротивление горизонтальным деформаци€м)
        FloatingPointType stretch_warp;           // упругость вертикальна€ (сопротивление вертикальным деформаци€м)
        FloatingPointType stretch_shear;          // упругость диагональна€ (сопротивление диагональным деформаци€м)
        FloatingPointType bending;                // сгибаемость (степень сопротивлени€ сгибанию) (0 - м€гка€ (шЄлк), 100 - жестка€ (кожа))
        FloatingPointType buckling;               // гибкость (0 - низка€ сгибаемость (шерсть), 100 - высока€ сгибаемость (шЄлк))
        FloatingPointType internal_damping;       // затухание 
        FloatingPointType density;                // плотность (соотношение веса к площади) (0 - т€жела€, 99 - лЄгка€)
        FloatingPointType friction;               // трение
        FloatingPointType air_pressure;           // давление воздуха
        FloatingPointType freefall_acelleration;  // ускорение свободного падени€
        FloatingPointType shift;                  // толщина

        ExternalPhysicalParam() :
            stretch_weft(static_cast<FloatingPointType>(50)),
            stretch_warp(static_cast<FloatingPointType>(50)),
            stretch_shear(static_cast<FloatingPointType>(50)),
            bending(static_cast<FloatingPointType>(50)),
            buckling(static_cast<FloatingPointType>(50)),
            internal_damping(zero),
            density(static_cast<FloatingPointType>(50)),
            friction(static_cast<DF::FloatingPointType>(1.)),
            air_pressure(zero),
            freefall_acelleration(static_cast<FloatingPointType>(9.8)),
            shift(static_cast<FloatingPointType>(0.3))
        { }
    };

    class PhysicalParam
    {
    public:
        PhysicalParam() :
            name_("default"),
            stretch_structural_(static_cast<FloatingPointType>(1)),
            stretch_stiffness_(static_cast<FloatingPointType>(1)),
            bending_(zero),
            buckling_(zero),
            internal_damping_(zero),
            density_(static_cast<FloatingPointType>(1)),
            friction_(static_cast<DF::FloatingPointType>(1.)),
            air_pressure_(zero),
            freefall_acelleration_(static_cast<FloatingPointType>(-koefG)),
            shift_(static_cast<FloatingPointType>(0.3))
        { }

        PhysicalParam(const PhysicalParam& param) :
            name_(param.name_),
            stretch_structural_(param.stretch_structural_),
            stretch_stiffness_(param.stretch_stiffness_),
            bending_(param.bending_),
            buckling_(param.buckling_),
            internal_damping_(param.internal_damping_),
            density_(param.density_),
            friction_(param.friction_),
            air_pressure_(param.air_pressure_),
            freefall_acelleration_(param.freefall_acelleration_),
            shift_(param.shift_)
        { }

        PhysicalParam(const ExternalPhysicalParam& external_param)
            : name_(external_param.name)
        {
            stretch_structural_ = external_param.stretch_weft / 50;
            Bound(stretch_structural_, zero, static_cast<FloatingPointType>(2));
            stretch_stiffness_ = external_param.stretch_weft / 50;
            Bound(stretch_stiffness_, zero, static_cast<FloatingPointType>(2));
            bending_ = external_param.bending;
            Bound(bending_, zero, static_cast<FloatingPointType>(100));
            buckling_ = external_param.buckling;
            Bound(buckling_, zero, static_cast<FloatingPointType>(100));
            internal_damping_ = external_param.internal_damping;
            Bound(internal_damping_, zero, static_cast<FloatingPointType>(99));
            density_ = external_param.density;
            Bound(density_, zero, static_cast<FloatingPointType>(100));
            friction_ = external_param.friction;
            Bound(friction_, zero, static_cast<FloatingPointType>(100));
            air_pressure_ = external_param.air_pressure;
            Bound(air_pressure_, zero, static_cast<FloatingPointType>(100));
            freefall_acelleration_ = external_param.freefall_acelleration * static_cast<FloatingPointType>(-koefG / 9.8);
            Bound(freefall_acelleration_, static_cast<FloatingPointType>(-100), static_cast<FloatingPointType>(100));
            shift_ = external_param.shift;
            Bound(shift_, zero, static_cast<FloatingPointType>(10));
        }

        inline const std::string& name() const { return name_; }                           
        inline const FloatingPointType& stretch_structural() const { return stretch_structural_; };
        inline const FloatingPointType& stretch_stiffness() const { return stretch_stiffness_; };
        inline const FloatingPointType& bending() const { return bending_; };
        inline const FloatingPointType& buckling() const { return buckling_; };
        inline const FloatingPointType& internal_damping() const { return internal_damping_; };
        inline const FloatingPointType& density() const { return density_; };
        inline const FloatingPointType& friction() const { return friction_; };
        inline const FloatingPointType& air_pressure() const { return air_pressure_; };
        inline const FloatingPointType& freefall_acelleration() const { return freefall_acelleration_; };
        inline const FloatingPointType shift() const { return shift_*koefFilterShift; };

        friend std::ostream& operator<< (std::ostream &out, const PhysicalParam &phys_param);

    private:
        std::string name_;                           
        FloatingPointType stretch_structural_;
        FloatingPointType stretch_stiffness_;
        FloatingPointType bending_;
        FloatingPointType buckling_;
        FloatingPointType internal_damping_;
        FloatingPointType density_;
        FloatingPointType friction_;
        FloatingPointType air_pressure_;
        FloatingPointType freefall_acelleration_;
        FloatingPointType shift_;
    };

    typedef std::vector<PhysicalParam> PhysicalParamsVector;

    std::ostream& operator<< (std::ostream &out, const DF::PhysicalParam &phys_param);
}

#endif  // !DFPHYSICALPARAMS_H
