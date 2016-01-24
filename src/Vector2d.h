//
#pragma once
//
//
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
//
//
template < typename T >
struct _TVector2d
{
	T XCoord, YCoord;

	_TVector2d() : XCoord(0), YCoord(0) {};

	_TVector2d(T _XCoord, T _YCoord) : XCoord(_XCoord), YCoord(_YCoord) {};

	_TVector2d( const _TVector2d & vec )
	{
		XCoord	= vec.XCoord;
		YCoord	= vec.YCoord;
	};

	_TVector2d & operator= ( const _TVector2d & vec )
	{
		XCoord	= vec.XCoord;
		YCoord	= vec.YCoord;

		return (*this);
	};

	_TVector2d operator+ (const _TVector2d& _SecVector)
	{
		return _TVector2d(XCoord + _SecVector.XCoord, YCoord + _SecVector.YCoord);
	};
	_TVector2d operator- (const _TVector2d& _SecVector)
	{
		return _TVector2d(_SecVector.XCoord - XCoord, _SecVector.YCoord - YCoord);
	}
	T operator* (const _TVector2d& _SecVector)
	{
		return	XCoord * _SecVector.XCoord + YCoord * _SecVector.YCoord;
	}
	_TVector2d operator* (const T _fScale)
	{
		return	_TVector2d(XCoord * _fScale, YCoord * _fScale);
	}
	_TVector2d operator*= (const _TVector2d& _SecVector)
	{
		return	(*this) = (*this) + _SecVector;
	}
	_TVector2d operator/ (const _TVector2d& _SecVector)
	{
		return _TVector2d(XCoord / _SecVector.XCoord, YCoord / _SecVector.YCoord);
	}
	_TVector2d operator+= (const _TVector2d& _SecVector)
	{
		return (*this) = (*this) + _SecVector;
	}
	_TVector2d operator-= (_TVector2d& _SecVector)
	{
		return (*this) = _SecVector - (*this);
	}
	bool operator== (const _TVector2d& _SecVector)
	{
		return (XCoord == _SecVector.XCoord && YCoord == _SecVector.YCoord);
	}
	bool operator!= (const _TVector2d& _SecVector)
	{
		return !((*this) == _SecVector);
	}
	int operator< (const _TVector2d& _SecVector)
	{
        return ((XCoord < _SecVector.XCoord) || ((XCoord == _SecVector.XCoord) && (YCoord < _SecVector.YCoord)));
	}
	int operator> (const _TVector2d& _SecVector)
	{
        return ((XCoord > _SecVector.XCoord) || ((XCoord == _SecVector.XCoord) && (YCoord > _SecVector.YCoord)));
	}
};

template<typename T>
_TVector2d<T> _RotateVect(const T & Radian, const _TVector2d<T> & Vector)
{
	_TVector2d<T> Vect;
	Vect.XCoord = cos(Radian) * Vector.XCoord - sin(Radian) * Vector.YCoord;
	Vect.YCoord = sin(Radian) * Vector.XCoord + cos(Radian) * Vector.YCoord;
	return Vect;
}
