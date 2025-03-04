#pragma once

#pragma message("Compiling math_macro.h - this should happen just once per project.\n")

#define TRUE    1
#define FALSE   0

#ifdef MATH_DOUBLE_PRECISION
typedef double m_real;
#else
typedef float m_real;
#endif

#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#define ABS(x)   ( ((x)>0.0) ? (x) :(-1.0*(x)) )
#define ACOS(x)  ( ((x)>1.0) ? (0) : ( ((x)<-1.0) ? (M_PI) : (acos(x)) ) )
#define ASIN(x)  ( ((x)>1.0) ? (M_PI/2.0) : ( ((x)<-1.0) ? (-M_PI/2.0) : (asin(x)) ) )
#define SQR(x)   ( (x)*(x) )
#define SHIFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define ROUND(x) ((int)floor((x)+0.5))
#define TO_RADIAN(degree) ((degree) * (M_PI / 180.0f))
#define TO_DEGREE(radian) ((radian)/(M_PI/180.f))

/*
template<class T>
inline const T SQR(const T a) {return a*a;}

template<class T>
inline const T MAX(const T &a, const T &b)
{return b > a ? (b) : (a);}

inline float MAX(const double &a, const float &b)
{return b > a ? (b) : float(a);}

inline float MAX(const float &a, const double &b)
{return b > a ? float(b) : (a);}

template<class T>
inline const T MIN(const T &a, const T &b)
{return b < a ? (b) : (a);}

inline float MIN(const double &a, const float &b)
{return b < a ? (b) : float(a);}

inline float MIN(const float &a, const double &b)
{return b < a ? float(b) : (a);}

template<class T>
inline const T SIGN(const T &a, const T &b)
{return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);}

inline float SIGN(const float &a, const double &b)
{return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);}

inline float SIGN(const double &a, const float &b)
{return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);}
*/
template<class T>
inline void SWAP(T &a, T &b)
{T dum=a; a=b; b=dum;}

template <class T>
inline const T CUBIC(const T x){return x*x*x;}
/*
inline m_real CUBIC(m_real x)
{
	return x*x*x;
}*/


inline m_real MAX3(m_real a,m_real b, m_real c) {
	if(a>b)
	{
		if(a>c) return a;
		else return c;
	}
	else
	{
		if(c>b) return c;
		else return b;
	}
}

inline m_real MIN3(m_real a,m_real b, m_real c) {
	if(a<b)
	{
		if(a<c) return a;
		else return c;
	}
	else
	{
		if(c<b) return c;
		else return b;
	}
}

inline m_real CLAMP(m_real a, m_real i1, m_real i2)
{
	if(a<i1) return i1;
	if(a>i2) return i2;
	return a;
}

inline int CLAMP(int a, int i1, int i2)
{
	if(a<i1) return i1;
	if(a>i2) return i2;
	return a;
}

inline bool isSimilar(m_real a, m_real b)
{
	return ABS(a-b)<0.0001;
}
