//------------------------------------------------------------------------------
// Реализация 3х мерной алгебры
//
// В классе реализованы необходимые конструкторы, операции заполнения,
// операции индексации с проверкой диапазона и без,
// поэлементные операции: +, +=, -, -=, унарный -,
// операции масштабирования: *, *=, /, /=,
// логические операции: ==, !=.
//------------------------------------------------------------------------------

#ifndef _3D_ALGEBRA_H
#define _3D_ALGEBRA_H

#include <stdexcept>
#include <cmath>

namespace DFMath
{
    typedef float FloatingPointType;
    const FloatingPointType epsilon = static_cast<FloatingPointType>(1e-5);

    // класс - 3-компонентный вектор
    template<class Atom>
    class Triple {
    public:
        Triple() {}
        Triple(const Triple &t) { set(t); }
        Triple(const Atom &a) { set(a); }
        Triple(const Atom &d1 , const Atom &d2 , const Atom &d3) { set(d1, d2, d3); }
        Triple(const Atom* const p) { set(p); }

        void set(const Triple &t) { v[0] = t.v[0]; v[1] = t.v[1]; v[2] = t.v[2]; }
        void set(const Atom &a) { v[0] = a; v[1] = a; v[2] = a; }
        void set(const Atom &d1, const Atom &d2, const Atom &d3) { v[0] = d1; v[1] = d2; v[2] = d3; }
        void set(const Atom * const p) { v[0] = p[0]; v[1] = p[1]; v[2] = p[2]; }

        void operator=(const Atom &a) { set(a); }
        void operator=(const Triple &t) { set(t); }
        void operator=(const Atom * const p) { set(p); }

        const Atom& operator[](const size_t i) const { return v[i]; }
        Atom& operator[](const size_t i) { return v[i]; }

        const Atom& at(const size_t i) const
        {
            if (i < 3) return v[i];
            else throw std::out_of_range ("Vector3");
        }
        Atom& at(const size_t i)
        {
            if (i < 3) return v[i];
            else throw std::out_of_range ("Vector3");
        }

        FloatingPointType getFloatingPoint(const size_t i) const { return FloatingPointType(v[i]); }

        void operator+=(const Triple &t)
        {
            v[0] += t.v[0];
            v[1] += t.v[1];
            v[2] += t.v[2];
        }
        Triple operator+(const Triple &t) const
        {
            Triple res(*this);
            res += t;
            return res;
        }

        void operator-=(const Triple &t)
        {
            v[0] -= t.v[0];
            v[1] -= t.v[1];
            v[2] -= t.v[2];
        }
        Triple operator-(const Triple &t) const
        {
            Triple res(*this);
            res -= t;
            return res;
        }

        Triple operator-(void) const
        {
            Triple res(*this * -1);
            return res;
        }

        void operator*=(const Atom &a)
        {
            v[0] *= a;
            v[1] *= a;
            v[2] *= a;
        }
        Triple operator*(const Atom &a) const
        {
            Triple ret(*this);
            ret *= a;
            return ret;
        }
        friend Triple operator*(const Atom &a, const Triple &t)
        {
            Triple ret(t);
            ret *= a;
            return ret;
        }

        void operator/=(const Atom &a)
        {
            if (a != 0)
            {
                v[0] /= a;
                v[1] /= a;
                v[2] /= a;
            }
        }
        Triple operator/(const Atom &a) const
        {
            Triple ret(*this);
            ret /= a;
            return ret;
        }

        bool operator==(const Triple &t) const
        {
            return v[0] == t.v[0] && 
                v[1] == t.v[1] && 
                v[2] == t.v[2];
        }
        bool operator!=(const Triple &t) const
        {
            return !((*this) == t);
        }

        bool operator<(const Triple &t) const
        {
            return (v[1] != t.v[1]) ? v[1] < t.v[1] : ((v[0] != t.v[0]) ? v[0] < t.v[0] : v[2] < t.v[2]);
        }

    private:
        Atom v[3];
    };

    typedef Triple<FloatingPointType> Vector3;

    // возвращает длину
    inline FloatingPointType length(const Vector3 &a)
    {
        return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    }

    // возвращает квадрат длины
    inline FloatingPointType length2(const Vector3 &a)
    {
        return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
    }

    // возвращает расстояние городских кварталов
    inline FloatingPointType TaxicabLength(const Vector3 &a)
    {
        return fabs(a[0]) + fabs(a[1]) + fabs(a[2]);
    }

    inline FloatingPointType ScalarProduct(const Vector3 &a, const Vector3 &b)
    {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    inline Vector3 CrossProduct(const Vector3 &a, const Vector3 &b)
    {
        return Vector3(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
            );
    }

    inline Vector3 normalize(const Vector3 &a)
    {
        FloatingPointType len_2 = length2(a);
        if (len_2 < epsilon)
            return Vector3(static_cast<FloatingPointType>(0));
        return a / sqrt(len_2);
    }
}

#endif // !_3D_ALGEBRA_H
