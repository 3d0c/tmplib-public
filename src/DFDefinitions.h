#ifndef DFBASEDEFINITIONS_H
#define DFBASEDEFINITIONS_H

#define USE_FIXED_POINT_ARITHMETIC

#include <vector>
#include <string>
#include <map>
#include <ctype.h>
#include "3d_algebra.h"
#include <deque>

namespace DF
{
    enum FittingMode
    {
        INITIAL,
        ITERATIONAL
    };

    typedef size_t Index;
    typedef DFMath::FloatingPointType FloatingPointType;
    const FloatingPointType zero = static_cast<FloatingPointType>(0);

    template<typename T>
    inline void Bound(T& val, const T& val_min, const T& val_max)
    {
        if (val < val_min)
            val = val_min;
        if (val > val_max)
            val = val_max;
    }

    enum Error_t
    {
        err_ok = 0,
        err_unknown,
        err_file_io_problem,
        err_file_structure_problem,
        err_not_enough_elements,
        err_empty_param_name,
        err_null_normal_detected,
        err_cant_calc_projection,
        err_cant_find_nearest,
        err_illegal_type,
        err_cant_generate_allocation_map,
        err_cant_generate_incidence_struct,
        err_cant_recalc_normals,
        err_binary_import_error,
        err_binary_export_error,
        err_time_elapsed,
        err_exception_unknown,
        err_bad_mannequin_pointer
    };

    struct FaceItem
    {
        FaceItem () : v(static_cast<DF::Index>(-1)), t(static_cast<DF::Index>(-1)), n(static_cast<DF::Index>(-1)) { }
		FaceItem (DF::Index _v, DF::Index _t, DF::Index _n) : v(_v), t(_t), n(_n) { }
        DF::Index v;
        DF::Index t;
        DF::Index n;
    };

    struct Linkage
    {
        DF::Index face_index;
        DF::Index point_order;
        FaceItem face;
    };
    
    typedef std::vector<Linkage> LinkageVector;

    struct GroupIdentifier
    {
        std::string name;                    // им€ группы
        DF::Index f_first;                   // первый полигон группы
        DF::Index f_last;                    // последний полигон группы
        DF::Index v_first;                   // перва€ вершина группы
        DF::Index v_last;                    // последн€€ вершина группы
        DF::Index t_first;                   // перва€ текстурна€ координата группы
        DF::Index t_last;                    // последн€€ текстурна€ координата группы
        DF::Index n_first;                   // первый вектор нормали группы
        DF::Index n_last;                    // последний вектор нормали группы
        unsigned char type;
    };

    typedef std::vector<GroupIdentifier> GroupsVector;

//     class Spring
//     {
//     public:
//         Spring() : i_(static_cast<DF::Index>(-1)), j_(static_cast<DF::Index>(-1)) {}
//         Spring(DF::Index i, DF::Index j) : i_(i), j_(j) { if (i_ > j_) std::swap(i_, j_); }
//         void operator=(const Spring &a) { i_ = a.i_; j_ = a.j_; }
//         bool operator<(const Spring &a) const { return (i_ != a.i_) ? i_ < a.i_ : j_ < a.j_; }
// 
//         DF::Index i_;
//         DF::Index j_;
//     };

//     typedef std::map<DFMath::Vector3, DF::Index> WeightStruct;
//     typedef std::map<Spring, DF::FloatingPointType> Springs;

    typedef std::map<FloatingPointType, Index> SortedStruct;

    typedef std::vector<Index> IndexVector;
    typedef std::vector<IndexVector> IndexVector2D;
    typedef std::vector<IndexVector2D> IndexVector3D;
    typedef std::vector<IndexVector3D> IndexVector4D;
    typedef std::vector<std::string> StringVector;

    enum RibType
    {
        structural,
        stiffness,
        collisional
    };

    struct IncedenceRecord
    {
        Index index;  // индекс, умноженный на 3
        FloatingPointType distance;
        FloatingPointType distance_prev;
        RibType type;
        bool operator==(const IncedenceRecord &a) const { return index == a.index; }
        bool operator==(const Index &i) const { return index == i; }
        bool operator<(const IncedenceRecord &a) const { return index < a.index; }
		IncedenceRecord() : index( 0 ), distance( 0 ), distance_prev( 0 ), type( structural ) {}
    };
    typedef std::vector<IncedenceRecord> IncedenceVector;
    typedef std::vector<IncedenceVector> IncedenceVector2D;

    typedef int State;
    typedef std::vector<State> StateVector;

    typedef std::vector<FaceItem> faceItemVector;
    typedef std::vector<DF::FloatingPointType> PointsVector;
    typedef std::vector<faceItemVector> FacesVector;

    typedef std::vector<DFMath::Vector3> Vector3Vector;
    typedef std::vector<Vector3Vector> Vector3Vector2D;
    typedef std::vector<Vector3Vector2D> Vector3Vector3D;
    typedef std::vector<Vector3Vector3D> Vector3Vector4D;

    const std::string print_error(Error_t err);

    StringVector split(const std::string &s);
    StringVector split(const std::string &s, char delim);

    template <typename T>
    inline const T parse(unsigned char **pp_ch)
    {
        static const T exp_10[16] = {
            static_cast<T>(1), 
            static_cast<T>(0.1), 
            static_cast<T>(0.01), 
            static_cast<T>(0.001), 
            static_cast<T>(0.0001), 
            static_cast<T>(0.00001), 
            static_cast<T>(0.000001), 
            static_cast<T>(0.0000001), 
            static_cast<T>(0.00000001), 
            static_cast<T>(0.000000001), 
            static_cast<T>(0.0000000001),
            static_cast<T>(0.00000000001),
            static_cast<T>(0.000000000001),
            static_cast<T>(0.0000000000001),
            static_cast<T>(0.00000000000001),
            static_cast<T>(0.000000000000001)
        };

        if (iscntrl(**pp_ch)) return 0;

        T res = 0;
        T sign = 1;
        if (**pp_ch == '-') { sign = static_cast<T>(-1); (*pp_ch)++; }
        while (isdigit(**pp_ch))
        {
            res = res * 10 + (**pp_ch - '0');
            (*pp_ch)++;
        }
        if (**pp_ch != '.') return sign * res;
        (*pp_ch)++;
        unsigned fract_counter = 0;
        while (isdigit(**pp_ch))
        {
            res = res * 10 + (**pp_ch - '0');
            (*pp_ch)++;
            fract_counter++;
        }

        return sign * res * exp_10[fract_counter];
    }

    inline void bufcpy(char **dest, const char *src)
    {
        while (*src) *(*dest)++ = *src++;
    }

    template<class T>
    inline void bufput(char **dest, T val, unsigned precision)
    {
        static const T exp10[16] = {
            static_cast<T>(1), 
            static_cast<T>(10), 
            static_cast<T>(100), 
            static_cast<T>(1000), 
            static_cast<T>(10000), 
            static_cast<T>(100000), 
            static_cast<T>(1000000), 
            static_cast<T>(10000000), 
            static_cast<T>(100000000), 
            static_cast<T>(1000000000), 
            static_cast<T>(10000000000),
            static_cast<T>(100000000000),
            static_cast<T>(1000000000000),
            static_cast<T>(10000000000000),
            static_cast<T>(100000000000000),
            static_cast<T>(1000000000000000)
        };

        if (val < 0) {
            val *= -1;
            *(*dest)++ = '-';
        }
        if (val > 1e+4) {
            bufput(dest, static_cast<unsigned>(0), 0);
            return;
        }
        unsigned integer = static_cast<unsigned>(val);
        unsigned fraction = static_cast<unsigned>((val - integer) * exp10[precision]);
        bufput(dest, integer, 0);
        *(*dest)++ = '.';
        bufput(dest, fraction, precision);
    }

    template<>
    inline void bufput(char **dest, unsigned val, unsigned n)
    {
        unsigned e = 10;
        for (unsigned i = 1; val >= e || i < n; i++) e *= 10;
        do 
        {
            val %= e;
            e /= 10;
            *(*dest)++ = static_cast<char>(val / e + '0');
        } while (e > 1);
    }

    inline int isblank(int ch) { return ch == ' ' || ch == '\t'; }

    template<class T>
    inline void swap2(T &a, T &b)
    {
        T t = a;
        a = b;
        b = t;
    }

    template<class I, class T>
    inline void sort2(I &ai, I &bi, T &a, T &b)
    {
        if (a > b) { swap2(a, b); swap2(ai, bi); }
    }

    template<class I, class T>
    inline void sort3(I &ai, I &bi, I&ci, T &a, T &b, T&c)
    {
        if (a > b) { swap2(a, b); swap2(ai, bi); }
        if (b > c) { swap2(b, c); swap2(bi, ci); }
        if (a > b) { swap2(a, b); swap2(ai, bi); }
    }

    template<class I, class T>
    inline void sort3(I &ai, I &bi, I&ci, T &a, T &b, T&c, bool (*comp)(T&, T&))
    {
        if (comp(a, b)) { swap2(a, b); swap2(ai, bi); }
        if (comp(b, c)) { swap2(b, c); swap2(bi, ci); }
        if (comp(a, b)) { swap2(a, b); swap2(ai, bi); }
    }

    std::string GetCurrentTimeString(bool maximize_compatibility = false);

    template<class T>
    inline T sqr(const T a)
    {
        return a * a;
    }

    template<class T>
    inline T sqr(const T* const a)
    {
        return *a * *a;
    }

    template<typename T>
    inline size_t GetMemoryUsage(const T &)
    {
        return sizeof(T);
    }

    template<typename T>
    inline size_t GetMemoryUsage(const std::vector<T> &cntnr)
    {
        size_t res = sizeof(cntnr);
        for (typename std::vector<T>::const_iterator it = cntnr.begin(); it != cntnr.end(); it++)
        {
            res += GetMemoryUsage(*it);
        }
        res += (cntnr.capacity() - cntnr.size()) * sizeof(T);
        return res;
    }

    template<typename T>
    inline size_t GetMemoryUsage(const std::deque<T> &cntnr)
    {
        size_t res = sizeof(cntnr);
        for (typename std::deque<T>::const_iterator it = cntnr.begin(); it != cntnr.end(); it++)
        {
            res += GetMemoryUsage(*it);
        }
        return res;
    }

    template<typename T1, typename T2>
    inline size_t GetMemoryUsage(const typename std::map<T1,T2> &cntnr)
    {
        size_t res = sizeof(cntnr);
        for (typename std::map<T1,T2>::const_iterator it = cntnr.begin(); it != cntnr.end(); it++)
        {
            res += GetMemoryUsage(*it);
        }
        return res;
    }

    template<class T>
    inline T sign(const T a)
    {
        return a < 0 ? static_cast<T>(-1) : static_cast<T>(1);
    }

    template <typename T>
    void ReleaseSTLContainer( T & t )
    {
        T tmp;
        t.swap( tmp );
    }

    std::string ClipExtension(const std::string &fullpath);
    std::string GetPath(const std::string &fullpath);
    std::string GetFilename(const std::string &fullpath);
    std::string GetExtension(const std::string &fullpath);
    bool FileExists(const std::string& name);
}

#endif // !DFBASEDEFINITIONS_H


