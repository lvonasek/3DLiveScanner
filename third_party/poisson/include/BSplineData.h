/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef BSPLINE_DATA_INCLUDED
#define BSPLINE_DATA_INCLUDED

#include "BinaryNode.h"
#include "PPolynomial.h"
#include "Array.h"

enum BoundaryType
{
	BOUNDARY_FREE ,
	BOUNDARY_DIRICHLET ,
	BOUNDARY_NEUMANN ,
	BOUNDARY_COUNT
};
template< BoundaryType BType > inline bool HasPartitionOfUnity( void ){ return BType!=BOUNDARY_DIRICHLET; }

// This class represents a function that is a linear combination of B-spline elements.
// The coeff member indicating how much of each element is present.
// [WARNING] The ordering of B-spline elements is in the opposite order from that returned by Polynomial::BSplineComponent
template< int Degree >
struct BSplineElementCoefficients
{
	int coeffs[Degree+1];
	BSplineElementCoefficients( void ){ memset( coeffs , 0 , sizeof( coeffs ) ); }
	int& operator[]( int idx ){ return coeffs[idx]; }
	const int& operator[]( int idx ) const { return coeffs[idx]; }
};

// This class represents a function on the the interval, partitioned into "res" blocks.
// On each block, the function is a degree-Degree polynomial, represented by the coefficients
// in the associated BSplineElementCoefficients.
// [NOTE] This representation of a function is agnostic to the type of boundary conditions (though the constructor is not).
template< int Degree >
struct BSplineElements : public std::vector< BSplineElementCoefficients< Degree > >
{
	static const bool _Primal = (Degree&1)==1;
	static const int _Off = (Degree+1)/2;
	static int _ReflectLeft ( int offset , int res );
	static int _ReflectRight( int offset , int res );
	static int _RotateLeft  ( int offset , int res );
	static int _RotateRight ( int offset , int res );
	template< bool Left > void _addPeriodic( int offset , bool negate );
public:
	// Coefficients are ordered as "/" "-" "\"
	// [WARNING] This is the opposite of the order in Polynomial::BSplineComponent
	int denominator;

	BSplineElements( void ) { denominator = 1; }
	BSplineElements( int res , int offset , BoundaryType bType );

	void upSample( BSplineElements& high ) const;
	template< unsigned int D >
	void differentiate( BSplineElements< Degree-D >& d ) const;

	void print( FILE* fp=stdout ) const
	{
		for( int i=0 ; i<std::vector< BSplineElementCoefficients< Degree > >::size() ; i++ )
		{
			printf( "%d]" , i );
			for( int j=0 ; j<=Degree ; j++ ) printf( " %d" , (*this)[i][j] );
			printf( " (%d)\n" , denominator );
		}
	}
	Polynomial< Degree > polynomial( int idx ) const
	{
		int res = (int)std::vector< BSplineElementCoefficients< Degree > >::size();
		Polynomial< Degree > P;
		if( idx>=0 && idx<res ) for( int d=0 ; d<=Degree ; d++ ) P += Polynomial< Degree >::BSplineComponent( Degree-d ).scale( 1./res ).shift( (idx+0.)/res ) * ( (*this)[idx][d] );
		return P / denominator;
	}
};
template< int Degree , int DDegree > struct Differentiator                   { static void Differentiate( const BSplineElements< Degree >& bse , BSplineElements< DDegree >& dbse ); };
template< int Degree >               struct Differentiator< Degree , Degree >{ static void Differentiate( const BSplineElements< Degree >& bse , BSplineElements<  Degree >& dbse ); };
#define BSPLINE_SET_BOUNDS( name , s , e ) \
	static const int name ## Start = (s); \
	static const int name ## End   = (e); \
	static const int name ## Size  = (e)-(s)+1

// Assumes that x is non-negative
#define _FLOOR_OF_HALF( x ) (   (x)    >>1 )
#define  _CEIL_OF_HALF( x ) ( ( (x)+1 )>>1 )
// Done with the assumption
#define FLOOR_OF_HALF( x ) ( (x)<0 ? -  _CEIL_OF_HALF( -(x) ) : _FLOOR_OF_HALF( x ) )
#define  CEIL_OF_HALF( x ) ( (x)<0 ? - _FLOOR_OF_HALF( -(x) ) :  _CEIL_OF_HALF( x ) )
#define SMALLEST_INTEGER_LARGER_THAN_HALF( x ) (  CEIL_OF_HALF( (x)+1 ) )
#define LARGEST_INTEGER_SMALLER_THAN_HALF( x ) ( FLOOR_OF_HALF( (x)-1 ) )
#define SMALLEST_INTEGER_LARGER_THAN_OR_EQUAL_TO_HALF( x ) (  CEIL_OF_HALF( x ) )
#define LARGEST_INTEGER_SMALLER_THAN_OR_EQUAL_TO_HALF( x ) ( FLOOR_OF_HALF( x ) )

template< int Degree >
struct BSplineSupportSizes
{
	inline static int Nodes( int depth ){ return ( 1<<depth ) + ( Degree&1 ); }
	inline static bool OutOfBounds( int depth , int offset ){ return offset>=0 || offset<Nodes(depth); }
	// An index is interiorly supported if its support is in the range [0,1<<depth)
	inline static void InteriorSupportedSpan( int depth , int& begin , int& end ){ begin = -SupportStart , end = (1<<depth)-SupportEnd; }

	// If the degree is even, we use a dual basis and functions are centered at the center of the interval
	// It the degree is odd, we use a primal basis and functions are centered at the left end of the interval
	// The function at index I is supported in:
	//	Support( I ) = [ I - (Degree+1-Inset)/2 , I + (Degree+1+Inset)/2 ]
	// [NOTE] The value of ( Degree + 1 +/- Inset ) is always even
	static const int Inset = (Degree&1) ? 0 : 1;
	BSPLINE_SET_BOUNDS(      Support , -( (Degree+1)/2 ) , Degree/2           );
	BSPLINE_SET_BOUNDS( ChildSupport ,    2*SupportStart , 2*(SupportEnd+1)-1 );
	BSPLINE_SET_BOUNDS(       Corner ,    SupportStart+1 , SupportEnd         );
	BSPLINE_SET_BOUNDS(  ChildCorner ,  2*SupportStart+1 , 2*SupportEnd + 1   );

	// Setting I=0, we are looking for the smallest/largest integers J such that:
	//		Support( 0 ) CONTAINS Support( J )
	// <=>	[-(Degree+1-Inset) , (Degree+1+Inset) ] CONTAINS [ J-(Degree+1-Inset)/2 , J+(Degree+1+Inset)/2 ]
	// Which is the same as the smallest/largest integers J such that:
	//		J - (Degree+1-Inset)/2 >= -(Degree+1-Inset)	| J + (Degree+1+Inset)/2 <= (Degree+1+Inset)
	// <=>	J >= -(Degree+1-Inset)/2					| J <= (Degree+1+Inset)/2
	BSPLINE_SET_BOUNDS( UpSample , - ( Degree + 1 - Inset ) / 2 , ( Degree + 1 + Inset ) /2 );

	// Setting I=0/1, we are looking for the smallest/largest integers J such that:
	//		Support( J ) CONTAINS Support( 0/1 )
	// <=>	[ 2*J - (Degree+1-Inset) , 2*J + (Degree+1+Inset) ] CONTAINS [ 0/1 - (Degree+1-Inset)/2 , 0/1 + (Degree+1+Inset)/2 ]
	// Which is the same as the smallest/largest integers J such that:
	//		2*J + (Degree+1+Inset) >= 0/1 + (Degree+1+Inset)/2	| 2*J - (Degree+1-Inset) <= 0/1 - (Degree+1-Inset)/2
	// <=>	2*J >= 0/1 - (Degree+1+Inset)/2						| 2*J <= 0/1 + (Degree+1-Inset)/2
	BSPLINE_SET_BOUNDS( DownSample0 , SMALLEST_INTEGER_LARGER_THAN_OR_EQUAL_TO_HALF( 0 - ( Degree + 1 + Inset ) / 2 ) , LARGEST_INTEGER_SMALLER_THAN_OR_EQUAL_TO_HALF( 0 + ( Degree + 1 - Inset ) / 2 ) );
	BSPLINE_SET_BOUNDS( DownSample1 , SMALLEST_INTEGER_LARGER_THAN_OR_EQUAL_TO_HALF( 1 - ( Degree + 1 + Inset ) / 2 ) , LARGEST_INTEGER_SMALLER_THAN_OR_EQUAL_TO_HALF( 1 + ( Degree + 1 - Inset ) / 2 ) );
	static const int DownSampleStart[], DownSampleSize[];
};
template< int Degree > const int BSplineSupportSizes< Degree >::DownSampleStart[] = { DownSample0Start , DownSample1Start };
template< int Degree > const int BSplineSupportSizes< Degree >::DownSampleSize [] = { DownSample0Size  , DownSample1Size  };


// Given a B-Spline of degree Degree1 at position i, this gives the offsets of the B-splines of degree Degree2 that just overlap with it.
template< int Degree1 , int Degree2 >
struct BSplineOverlapSizes
{
	typedef BSplineSupportSizes< Degree1 > EData1;
	typedef BSplineSupportSizes< Degree2 > EData2;
	BSPLINE_SET_BOUNDS(             Overlap , EData1::     SupportStart - EData2::SupportEnd , EData1::     SupportEnd - EData2::SupportStart );
	BSPLINE_SET_BOUNDS(        ChildOverlap , EData1::ChildSupportStart - EData2::SupportEnd , EData1::ChildSupportEnd - EData2::SupportStart );
	BSPLINE_SET_BOUNDS(      OverlapSupport ,      OverlapStart + EData2::SupportStart ,      OverlapEnd + EData2::SupportEnd );
	BSPLINE_SET_BOUNDS( ChildOverlapSupport , ChildOverlapStart + EData2::SupportStart , ChildOverlapEnd + EData2::SupportEnd );

	// Setting I=0/1, we are looking for the smallest/largest integers J such that:
	//		Support( 2*J ) * 2 INTERSECTION Support( 0/1 ) NON-EMPTY
	// <=>	[ 2*J - (Degree2+1-Inset2) , 2*J + (Degree2+1+Inset2) ] INTERSECTION [ 0/1 - (Degree1+1-Inset1)/2 , 0/1 + (Degree1+1+Inset1)/2 ] NON-EMPTY
	// Which is the same as the smallest/largest integers J such that:
	//		0/1 - (Degree1+1-Inset1)/2 < 2*J + (Degree2+1+Inset2)			| 0/1 + (Degree1+1+Inset1)/2 > 2*J - (Degree2+1-Inset2)	
	// <=>	2*J > 0/1 - ( 2*Degree2 + Degree1 + 3 + 2*Inset2 - Inset1 ) / 2	| 2*J < 0/1 + ( 2*Degree2 + Degree1 + 3 - 2*Inset2 + Inset1 ) / 2
	BSPLINE_SET_BOUNDS( ParentOverlap0 , SMALLEST_INTEGER_LARGER_THAN_HALF( 0 - ( 2*Degree2 + Degree1 + 3 + 2*EData2::Inset - EData1::Inset ) / 2 ) , LARGEST_INTEGER_SMALLER_THAN_HALF( 0 + ( 2*Degree2 + Degree1 + 3 - 2*EData2::Inset + EData1::Inset ) / 2 ) );
	BSPLINE_SET_BOUNDS( ParentOverlap1 , SMALLEST_INTEGER_LARGER_THAN_HALF( 1 - ( 2*Degree2 + Degree1 + 3 + 2*EData2::Inset - EData1::Inset ) / 2 ) , LARGEST_INTEGER_SMALLER_THAN_HALF( 1 + ( 2*Degree2 + Degree1 + 3 - 2*EData2::Inset + EData1::Inset ) / 2 ) );
	static const int ParentOverlapStart[] , ParentOverlapEnd[];
};
template< int Degree1 , int Degree2 > const int BSplineOverlapSizes< Degree1 , Degree2 >::ParentOverlapStart[] = { ParentOverlap0Start , ParentOverlap1Start };
template< int Degree1 , int Degree2 > const int BSplineOverlapSizes< Degree1 , Degree2 >::ParentOverlapEnd  [] = { ParentOverlap0End   , ParentOverlap1End   };

template< int Degree , BoundaryType BType >
class BSplineEvaluationData
{
public:
	static const int Pad = (BType==BOUNDARY_FREE ) ? BSplineSupportSizes< Degree >::SupportEnd : ( (Degree&1) && BType==BOUNDARY_DIRICHLET ) ? -1 : 0;
	inline static int Begin( int depth ){ return -Pad; }
	inline static int End  ( int depth ){ return (1<<depth) + (Degree&1) + Pad; }
	inline static bool OutOfBounds( int depth , int offset ){ return offset<Begin(depth) || offset>=End(depth); }

	static const int OffsetStart = -BSplineSupportSizes< Degree >::SupportStart , OffsetStop = BSplineSupportSizes< Degree >::SupportEnd + ( Degree&1 ) , IndexSize = OffsetStart + OffsetStop + 1 + 2 * Pad;
	static int OffsetToIndex( int depth , int offset )
	{
		int dim = BSplineSupportSizes< Degree >::Nodes( depth );
		if     ( offset<OffsetStart )     return Pad + offset;
		else if( offset>=dim-OffsetStop ) return Pad + OffsetStart + 1 + offset - ( dim-OffsetStop );
		else                              return Pad + OffsetStart;
	}
	static inline int IndexToOffset( int depth , int idx ){ return ( idx-Pad<=OffsetStart ? idx - Pad : ( BSplineSupportSizes< Degree >::Nodes(depth) + Pad - IndexSize + idx ) ); }

	BSplineEvaluationData( void );

	// [NOTE] The offset represents the node position, not the index of the function
	static double Value( int depth , int off , double s , bool derivative );

	// Note that this struct stores the components in left-to-right order
	struct BSplineComponents
	{
	protected:
		Polynomial< Degree > _polys[Degree+1];
	public:
		BSplineComponents( void ){ ; }
		BSplineComponents( int depth , int offset );
		const Polynomial< Degree >& operator[] ( int idx ) const { return _polys[idx]; }
		BSplineComponents derivative( void ) const;
	};
	struct BSplineUpSamplingCoefficients
	{
	protected:
		int _coefficients[ BSplineSupportSizes< Degree >::UpSampleSize ];
	public:
		BSplineUpSamplingCoefficients( void ){ ; }
		BSplineUpSamplingCoefficients( int depth , int offset );
		double operator[] ( int idx ){ return (double)_coefficients[idx] / (1<<Degree); }
	};

	struct CenterEvaluator
	{
		struct Evaluator
		{
		protected:
			friend BSplineEvaluationData;
			int _depth;
			double _ccValues[2][IndexSize][BSplineSupportSizes< Degree >::SupportSize];
		public:
			Evaluator( void ){ _depth = 0 ; memset( _ccValues , 0 , sizeof(_ccValues) ); }
			double value( int fIdx , int cIdx , bool d ) const;
			int depth( void ) const { return _depth; }
		};
		struct ChildEvaluator
		{
		protected:
			friend BSplineEvaluationData;
			int _parentDepth;
			double _pcValues[2][IndexSize][BSplineSupportSizes< Degree >::ChildSupportSize];
		public:
			ChildEvaluator( void ){ _parentDepth = 0 ; memset( _pcValues , 0 , sizeof(_pcValues) ); }
			double value( int fIdx , int cIdx , bool d ) const;
		};
	};
	static void SetCenterEvaluator( typename CenterEvaluator::Evaluator& evaluator , int depth );
	static void SetChildCenterEvaluator( typename CenterEvaluator::ChildEvaluator& evaluator , int parentDepth );

	struct CornerEvaluator
	{
		struct Evaluator
		{
		protected:
			friend BSplineEvaluationData;
			int _depth;
			double _ccValues[2][IndexSize][BSplineSupportSizes< Degree >::CornerSize];
		public:
			Evaluator( void ){ _depth = 0 ; memset( _ccValues , 0 , sizeof( _ccValues ) ); }
			double value( int fIdx , int cIdx , bool d ) const;
			int depth( void ) const { return _depth; }
		};
		struct ChildEvaluator
		{
		protected:
			friend BSplineEvaluationData;
			int _parentDepth;
			double _pcValues[2][IndexSize][BSplineSupportSizes< Degree >::ChildCornerSize];
		public:
			ChildEvaluator( void ){ _parentDepth = 0 ; memset( _pcValues , 0 , sizeof( _pcValues ) ); }
			double value( int fIdx , int cIdx , bool d ) const;
		};
	};
	static void SetCornerEvaluator( typename CornerEvaluator::Evaluator& evaluator , int depth );
	static void SetChildCornerEvaluator( typename CornerEvaluator::ChildEvaluator& evaluator , int parentDepth );

	struct Evaluator
	{
		typename CenterEvaluator::Evaluator centerEvaluator;
		typename CornerEvaluator::Evaluator cornerEvaluator;
		double centerValue( int fIdx , int cIdx , bool d ) const { return centerEvaluator.value( fIdx , cIdx , d ); }
		double cornerValue( int fIdx , int cIdx , bool d ) const { return cornerEvaluator.value( fIdx , cIdx , d ); }
	};
	static void SetEvaluator( Evaluator& evaluator , int depth ){ SetCenterEvaluator( evaluator.centerEvaluator , depth ) , SetCornerEvaluator( evaluator.cornerEvaluator , depth ); }

	struct ChildEvaluator
	{
		typename CenterEvaluator::ChildEvaluator centerEvaluator;
		typename CornerEvaluator::ChildEvaluator cornerEvaluator;
		double centerValue( int fIdx , int cIdx , bool d ) const { return centerEvaluator.value( fIdx , cIdx , d ); }
		double cornerValue( int fIdx , int cIdx , bool d ) const { return cornerEvaluator.value( fIdx , cIdx , d ); }
	};
	static void SetChildEvaluator( ChildEvaluator& evaluator , int depth ){ SetChildCenterEvaluator( evaluator.centerEvaluator , depth ) , SetChildCornerEvaluator( evaluator.cornerEvaluator , depth ); }

	struct UpSampleEvaluator
	{
	protected:
		friend BSplineEvaluationData;
		int _lowDepth;
		double _pcValues[IndexSize][BSplineSupportSizes< Degree >::UpSampleSize];
	public:
		UpSampleEvaluator( void ){ _lowDepth = 0 ; memset( _pcValues , 0 , sizeof( _pcValues ) ); }
		double value( int pIdx , int cIdx ) const;
	};
	static void SetUpSampleEvaluator( UpSampleEvaluator& evaluator , int lowDepth );
};

template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
class BSplineIntegrationData
{
public:
	static const int OffsetStart = - BSplineOverlapSizes< Degree1 , Degree2 >::OverlapSupportStart , OffsetStop = BSplineOverlapSizes< Degree1 , Degree2 >::OverlapSupportEnd + ( Degree1&1 ) , IndexSize = OffsetStart + OffsetStop + 1 + 2 * BSplineEvaluationData< Degree1 , BType1 >::Pad;
	static int OffsetToIndex( int depth , int offset )
	{
		int dim = BSplineSupportSizes< Degree1 >::Nodes( depth );
		if     ( offset<OffsetStart )     return BSplineEvaluationData< Degree1 , BType1 >::Pad + offset;
		else if( offset>=dim-OffsetStop ) return BSplineEvaluationData< Degree1 , BType1 >::Pad + OffsetStart + 1 + offset - ( dim-OffsetStop );
		else                              return BSplineEvaluationData< Degree1 , BType1 >::Pad + OffsetStart;
	}
	static inline int IndexToOffset( int depth , int idx ){ return ( idx-BSplineEvaluationData< Degree1 , BType1 >::Pad<=OffsetStart ? idx-BSplineEvaluationData< Degree1 , BType1 >::Pad : ( BSplineSupportSizes< Degree1 >::Nodes(depth) + BSplineEvaluationData< Degree1 , BType1 >::Pad - IndexSize + idx ) ); }

	template< unsigned int D1 , unsigned int D2 > static double Dot( int depth1 , int off1 , int depth2 , int off2 );
	// An index is interiorly overlapped if the support of its overlapping neighbors is in the range [0,1<<depth)
	inline static void InteriorOverlappedSpan( int depth , int& begin , int& end ){ begin = -BSplineOverlapSizes< Degree1 , Degree2 >::OverlapStart-BSplineSupportSizes< Degree2 >::SupportStart , end = (1<<depth)-BSplineOverlapSizes< Degree1 , Degree2 >::OverlapEnd-BSplineSupportSizes< Degree2 >::SupportEnd; }

	struct FunctionIntegrator
	{
		template< unsigned int D1 , unsigned int D2 >
		struct Integrator
		{
		protected:
			friend BSplineIntegrationData;
			int _depth;
			double _ccIntegrals[D1+1][D2+1][IndexSize][BSplineOverlapSizes< Degree1 , Degree2 >::OverlapSize];
		public:
			Integrator( void ){ _depth = 0 ; memset(_ccIntegrals, 0, sizeof(_ccIntegrals)); }
			double dot( int fIdx1 , int fidx2 , int d1 , int d2 ) const;
			int depth( void ) const { return _depth; }
		};
		template< unsigned int D1 , unsigned int D2 >
		struct ChildIntegrator
		{
		protected:
			friend BSplineIntegrationData;
			int _parentDepth;
			double _pcIntegrals[D1+1][D2+1][IndexSize][BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapSize];
		public:
			ChildIntegrator( void ){ _parentDepth = 0 ; memset( _pcIntegrals , 0 , sizeof( _pcIntegrals ) ); }
			double dot( int fIdx1 , int fidx2 , int d1 , int d2 ) const;
			int childDepth( void ) const { return _parentDepth+1; }
		};
	};
	// D1 and D2 indicate the number of derivatives that should be taken
	template< unsigned int D1 , unsigned int D2 >
	static void SetIntegrator( typename FunctionIntegrator::template Integrator< D1 , D2 >& integrator , int depth );
	template< unsigned int D1 , unsigned int D2 >
	static void SetChildIntegrator( typename FunctionIntegrator::template ChildIntegrator< D1 , D2 >& integrator , int parentDepth );

protected:
	// _D1 and _D2 indicate the total number of derivatives the integrator will be storing
	template< unsigned int D1 , unsigned int D2 , unsigned int _D1 , unsigned int _D2 >
	struct _IntegratorSetter
	{
		static void Set( typename FunctionIntegrator::template      Integrator< _D1 , _D2 >& integrator , int depth );
		static void Set( typename FunctionIntegrator::template ChildIntegrator< _D1 , _D2 >& integrator , int depth );
	};

	template< unsigned int D1 , unsigned int D2 , unsigned int _D1 , unsigned int _D2 , class Integrator >
	struct IntegratorSetter
	{
		static void Set2D( Integrator& integrator , int depth );
		static void Set1D( Integrator& integrator , int depth );
	};
	template< unsigned int D1 , unsigned int _D1 , unsigned int _D2 , class Integrator >
	struct IntegratorSetter< D1 , 0 , _D1 , _D2 , Integrator >
	{
		static void Set2D( Integrator& integrator , int  depth );
		static void Set1D( Integrator& integrator , int  depth );
	};
	template< unsigned int D2 , unsigned int _D1 , unsigned int _D2 , class Integrator >
	struct IntegratorSetter< 0 , D2 , _D1 , _D2 , Integrator >
	{
		static void Set2D( Integrator& integrator , int  depth );
		static void Set1D( Integrator& integrator , int  depth );
	};
	template< unsigned int _D1 , unsigned int _D2 , class Integrator >
	struct IntegratorSetter< 0 , 0 , _D1 , _D2 , Integrator >
	{
		static void Set2D( Integrator& integrator , int  depth );
		static void Set1D( Integrator& integrator , int  depth );
	};
};
#undef BSPLINE_SET_BOUNDS
#undef _FLOOR_OF_HALF
#undef  _CEIL_OF_HALF
#undef FLOOR_OF_HALF
#undef  CEIL_OF_HALF
#undef SMALLEST_INTEGER_LARGER_THAN_HALF
#undef LARGEST_INTEGER_SMALLER_THAN_HALF
#undef SMALLEST_INTEGER_LARGER_THAN_OR_EQUAL_TO_HALF
#undef LARGEST_INTEGER_SMALLER_THAN_OR_EQUAL_TO_HALF

template< int Degree , BoundaryType BType >
struct BSplineData
{
	inline static int TotalFunctionCount( int depth ){ return depth<0 ? 0 : (1<<(depth+1)) - 1 + (depth+1) * ( (Degree&1) + 2 * BSplineEvaluationData< Degree , BType >::Pad ); }
	inline static int FunctionIndex( int depth , int offset ){ return TotalFunctionCount( depth-1 ) + offset + BSplineEvaluationData< Degree , BType >::Pad; }
	inline static void FactorFunctionIndex( int idx , int& depth , int& offset )
	{
		int dim;
		depth = 0;
		while( idx>=( dim = BSplineEvaluationData< Degree , BType >::End( depth ) - BSplineEvaluationData< Degree , BType >::Begin( depth ) ) ) idx -= dim , depth++;
		offset = idx - BSplineEvaluationData< Degree , BType >::Pad;
	}
	inline static void FunctionSpan( int depth , int& fStart , int& fEnd ){ fStart = TotalFunctionCount( depth-1 ) , fEnd = TotalFunctionCount( depth ); }
	inline static int RemapOffset( int depth , int idx , bool& reflect );

	size_t functionCount;
	Pointer( typename BSplineEvaluationData< Degree , BType >::BSplineComponents )  baseBSplines;
	Pointer( typename BSplineEvaluationData< Degree , BType >::BSplineComponents ) dBaseBSplines;

	BSplineData( int maxDepth );
	~BSplineData( void );
};

template< int Degree1 , int Degree2 > void SetBSplineElementIntegrals( double integrals[Degree1+1][Degree2+1] );



///////////////////////////
// BSplineEvaluationData //
///////////////////////////
template< int Degree , BoundaryType BType >
double BSplineEvaluationData< Degree , BType >::Value( int depth , int off , double s , bool derivative )
{
	if( s<0 || s>1 ) return 0.;

	int res = 1<<depth;
	if( OutOfBounds( depth , off ) ) return 0;

	BSplineComponents components = BSplineComponents( depth , off );

	// [NOTE] This is an ugly way to ensure that when s=1 we evaluate using a B-Spline component within the valid range.
	int ii = std::max< int >( 0 , std::min< int >( res-1 , (int)floor( s * res ) ) ) - off;

	if( ii<BSplineSupportSizes< Degree >::SupportStart || ii>BSplineSupportSizes< Degree >::SupportEnd ) return 0;
	if( derivative ) return components[ ii-BSplineSupportSizes< Degree >::SupportStart ].derivative()(s);
	else             return components[ ii-BSplineSupportSizes< Degree >::SupportStart ](s);
}
template< int Degree , BoundaryType BType >
void BSplineEvaluationData< Degree , BType >::SetCenterEvaluator( typename CenterEvaluator::Evaluator& evaluator , int depth )
{
	evaluator._depth = depth;
	int res = 1<<depth;
	for( int i=0 ; i<IndexSize ; i++ ) for( int j=BSplineSupportSizes< Degree >::SupportStart ; j<=BSplineSupportSizes< Degree >::SupportEnd ; j++ )
		{
			int ii = IndexToOffset( depth , i );
			double s = 0.5 + ii + j;
			for( int d1=0 ; d1<2 ; d1++ ) evaluator._ccValues[d1][i][j-BSplineSupportSizes< Degree >::SupportStart] = Value( depth , ii , s/res , d1!=0 );
		}
}
template< int Degree , BoundaryType BType >
void BSplineEvaluationData< Degree , BType >::SetChildCenterEvaluator( typename CenterEvaluator::ChildEvaluator& evaluator , int parentDepth )
{
	evaluator._parentDepth = parentDepth;
	int res = 1<<(parentDepth+1);
	for( int i=0 ; i<IndexSize ; i++ ) for( int j=BSplineSupportSizes< Degree >::ChildSupportStart ; j<=BSplineSupportSizes< Degree >::ChildSupportEnd ; j++ )
		{
			int ii = IndexToOffset( parentDepth , i );
			double s = 0.5 + 2*ii + j;
			for( int d1=0 ; d1<2 ; d1++ ) evaluator._pcValues[d1][i][j-BSplineSupportSizes< Degree >::ChildSupportStart] = Value( parentDepth , ii , s/res , d1!=0 );
		}
}
template< int Degree , BoundaryType BType >
double BSplineEvaluationData< Degree , BType >::CenterEvaluator::Evaluator::value( int fIdx , int cIdx , bool d ) const
{
	int dd = cIdx-fIdx , res = 1<<(_depth);
	if( cIdx<0 || cIdx>=res || OutOfBounds( _depth , fIdx ) || dd<BSplineSupportSizes< Degree >::SupportStart || dd>BSplineSupportSizes< Degree >::SupportEnd ) return 0;
	return _ccValues[d?1:0][ OffsetToIndex( _depth , fIdx ) ][dd-BSplineSupportSizes< Degree >::SupportStart];
}
template< int Degree , BoundaryType BType >
double BSplineEvaluationData< Degree , BType >::CenterEvaluator::ChildEvaluator::value( int fIdx , int cIdx , bool d ) const
{
	int dd = cIdx-2*fIdx , res = 1<<(_parentDepth+1);
	if( cIdx<0 || cIdx>=res || OutOfBounds( _parentDepth , fIdx ) || dd<BSplineSupportSizes< Degree >::ChildSupportStart || dd>BSplineSupportSizes< Degree >::ChildSupportEnd ) return 0;
	return _pcValues[d?1:0][ OffsetToIndex( _parentDepth , fIdx ) ][dd-BSplineSupportSizes< Degree >::ChildSupportStart];
}
template< int Degree , BoundaryType BType >
void BSplineEvaluationData< Degree , BType >::SetCornerEvaluator( typename CornerEvaluator::Evaluator& evaluator , int depth )
{
	evaluator._depth = depth;
	int res = 1<<depth;
	for( int i=0 ; i<IndexSize ; i++ ) for( int j=BSplineSupportSizes< Degree >::CornerStart ; j<=BSplineSupportSizes< Degree >::CornerEnd ; j++ )
		{
			int ii = IndexToOffset( depth , i );
			double s = ii + j;
			for( int d1=0 ; d1<2 ; d1++ ) evaluator._ccValues[d1][i][j-BSplineSupportSizes< Degree >::CornerStart] = Value( depth , ii , s/res , d1!=0 );
		}
}
template< int Degree , BoundaryType BType >
void BSplineEvaluationData< Degree , BType >::SetChildCornerEvaluator( typename CornerEvaluator::ChildEvaluator& evaluator , int parentDepth )
{
	evaluator._parentDepth = parentDepth;
	int res = 1<<(parentDepth+1);
	for( int i=0 ; i<IndexSize ; i++ ) for( int j=BSplineSupportSizes< Degree >::ChildCornerStart ; j<=BSplineSupportSizes< Degree >::ChildCornerEnd ; j++ )
		{
			int ii = IndexToOffset( parentDepth , i );
			double s = 2*ii + j;
			for( int d1=0 ; d1<2 ; d1++ ) evaluator._pcValues[d1][i][j-BSplineSupportSizes< Degree >::ChildCornerStart] = Value( parentDepth , ii , s/res , d1!=0 );
		}
}
template< int Degree , BoundaryType BType >
void BSplineEvaluationData< Degree , BType >::SetUpSampleEvaluator( UpSampleEvaluator& evaluator , int lowDepth )
{
	evaluator._lowDepth = lowDepth;
	for( int i=0 ; i<IndexSize ; i++ )
	{
		int ii = IndexToOffset( lowDepth , i );
		BSplineUpSamplingCoefficients b( lowDepth , ii );
		for( int j=0 ; j<BSplineSupportSizes< Degree >::UpSampleSize ; j++ ) evaluator._pcValues[i][j] = b[j];
	}
}
template< int Degree , BoundaryType BType >
double BSplineEvaluationData< Degree , BType >::CornerEvaluator::Evaluator::value( int fIdx , int cIdx , bool d ) const
{
	int dd = cIdx-fIdx , res = ( 1<<_depth ) + 1;
	if( cIdx<0 || cIdx>=res || OutOfBounds( _depth , fIdx ) || dd<BSplineSupportSizes< Degree >::CornerStart || dd>BSplineSupportSizes< Degree >::CornerEnd ) return 0;
	return _ccValues[d?1:0][ OffsetToIndex( _depth , fIdx ) ][dd-BSplineSupportSizes< Degree >::CornerStart];
}
template< int Degree , BoundaryType BType >
double BSplineEvaluationData< Degree , BType >::CornerEvaluator::ChildEvaluator::value( int fIdx , int cIdx , bool d ) const
{
	int dd = cIdx-2*fIdx , res = ( 1<<(_parentDepth+1) ) + 1;
	if( cIdx<0 || cIdx>=res || OutOfBounds( _parentDepth , fIdx ) || dd<BSplineSupportSizes< Degree >::ChildCornerStart || dd>BSplineSupportSizes< Degree >::ChildCornerEnd ) return 0;
	return _pcValues[d?1:0][ OffsetToIndex( _parentDepth , fIdx ) ][dd-BSplineSupportSizes< Degree >::ChildCornerStart];
}
template< int Degree , BoundaryType BType >
double BSplineEvaluationData< Degree , BType >::UpSampleEvaluator::value( int pIdx , int cIdx ) const
{
	int dd = cIdx-2*pIdx;
	if( OutOfBounds( _lowDepth+1 , cIdx ) || OutOfBounds( _lowDepth , pIdx ) || dd<BSplineSupportSizes< Degree >::UpSampleStart || dd>BSplineSupportSizes< Degree >::UpSampleEnd ) return 0;
	return _pcValues[ OffsetToIndex( _lowDepth , pIdx ) ][dd-BSplineSupportSizes< Degree >::UpSampleStart];
}

//////////////////////////////////////////////
// BSplineEvaluationData::BSplineComponents //
//////////////////////////////////////////////
template< int Degree , BoundaryType BType >
BSplineEvaluationData< Degree , BType >::BSplineComponents::BSplineComponents( int depth , int offset )
{
	int res = 1<<depth;
	BSplineElements< Degree > elements( res , offset , BType );

	// The first index is the position, the second is the element type
	Polynomial< Degree > components[Degree+1][Degree+1];
	// Generate the elements that can appear in the base function corresponding to the base function at (depth,offset) = (0,0)
	for( int d=0 ; d<=Degree ; d++ ) for( int dd=0 ; dd<=Degree ; dd++ ) components[d][dd] = Polynomial< Degree >::BSplineComponent( Degree-dd ).shift( -( (Degree+1)/2 ) + d );

	// Now adjust to the desired depth and offset
	double width = 1. / res;
	for( int d=0 ; d<=Degree ; d++ ) for( int dd=0 ; dd<=Degree ; dd++ ) components[d][dd] = components[d][dd].scale( width ).shift( width*offset );

	// Now write in the polynomials
	for( int d=0 ; d<=Degree ; d++ )
	{
		int idx = offset + BSplineSupportSizes< Degree >::SupportStart + d;
		_polys[d] = Polynomial< Degree >();

		if( idx>=0 && idx<res ) for( int dd=0 ; dd<=Degree ; dd++ ) _polys[d] += components[d][dd] * ( ( double )( elements[idx][dd] ) ) / elements.denominator;
	}
}

template< int Degree , BoundaryType BType >
typename BSplineEvaluationData< Degree , BType >::BSplineComponents BSplineEvaluationData< Degree , BType >::BSplineComponents::derivative( void ) const
{
	BSplineComponents b = (*this);
	for( int d=0 ; d<=Degree ; d++ ) b._polys[d] = b._polys[d].derivative();
	return b;
}

//////////////////////////////////////////////////////////
// BSplineEvaluationData::BSplineUpSamplingCoefficients //
//////////////////////////////////////////////////////////
template< int Degree , BoundaryType BType >
BSplineEvaluationData< Degree , BType >::BSplineUpSamplingCoefficients::BSplineUpSamplingCoefficients( int depth , int offset )
{
	// [ 1/8 1/2 3/4 1/2 1/8]
	// [ 1 , 1 ] ->  [ 3/4 , 1/2 , 1/8 ] + [ 1/8 , 1/2 , 3/4 ] = [ 7/8 , 1 , 7/8 ]
	int dim = BSplineSupportSizes< Degree >::Nodes(depth) , _dim = BSplineSupportSizes< Degree >::Nodes(depth+1);
	bool reflect;
	offset = BSplineData< Degree , BType >::RemapOffset( depth , offset , reflect );
	int multiplier = ( BType==BOUNDARY_DIRICHLET && reflect ) ? -1 : 1;
	bool useReflected = ( BType!=BOUNDARY_FREE ) && ( BSplineSupportSizes< Degree >::Inset || ( offset % ( dim-1 ) ) );
	int b[ BSplineSupportSizes< Degree >::UpSampleSize ];
	Polynomial< Degree+1 >::BinomialCoefficients( b );

	// Clear the values
	memset( _coefficients , 0 , sizeof(int) * BSplineSupportSizes< Degree >::UpSampleSize );

	// Get the array of coefficients, relative to the origin
	int* coefficients = _coefficients - ( 2*offset + BSplineSupportSizes< Degree >::UpSampleStart );
	for( int i=BSplineSupportSizes< Degree >::UpSampleStart ; i<=BSplineSupportSizes< Degree >::UpSampleEnd ; i++ )
	{
		int _offset = 2*offset+i;
		_offset = BSplineData< Degree , BType >::RemapOffset( depth+1 , _offset , reflect );
		if( useReflected || !reflect )
		{
			int _multiplier = multiplier * ( ( BType==BOUNDARY_DIRICHLET && reflect ) ? -1 : 1 );
			coefficients[ _offset ] += b[ i-BSplineSupportSizes< Degree >::UpSampleStart ] * _multiplier;
		}
		// If we are not inset and we are at the boundary, use the reflection as well
		if( BType!=BOUNDARY_FREE && !BSplineSupportSizes< Degree >::Inset && ( offset % (dim-1) ) && !( _offset % (_dim-1) ) )
		{
			_offset = BSplineData< Degree , BType >::RemapOffset( depth+1 , _offset , reflect );
			int _multiplier = multiplier * ( ( BType==BOUNDARY_DIRICHLET && reflect ) ? -1 : 1 );
			if( BType==BOUNDARY_DIRICHLET ) _multiplier *= -1;
			coefficients[ _offset ] += b[ i-BSplineSupportSizes< Degree >::UpSampleStart ] * _multiplier;
		}
	}
}

////////////////////////////
// BSplineIntegrationData //
////////////////////////////
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 >
double BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::Dot( int depth1 ,  int off1 , int depth2 , int off2 )
{
	if( D1>Degree1 ) fprintf( stderr , "[ERROR] BSplineIntegrationData::Dot: taking more derivatives than the degree: %d > %d\n" , D1 , Degree1 ) , exit( 0 );
	if( D2>Degree2 ) fprintf( stderr , "[ERROR] BSplineIntegrationData::Dot: taking more derivatives than the degree: %d > %d\n" , D2 , Degree2 ) , exit( 0 );
	const int _Degree1 = ( Degree1>=D1 ) ? Degree1 - D1 : 0 , _Degree2 = ( Degree2>=D2 ) ? Degree2 - D2 : 0;
	int sums[ Degree1+1 ][ Degree2+1 ];

	int depth = std::max< int >( depth1 , depth2 );

	BSplineElements< Degree1 > b1( 1<<depth1 , off1 , BType1 );
	BSplineElements< Degree2 > b2( 1<<depth2 , off2 , BType2 );

	{
		BSplineElements< Degree1 > b;
		while( depth1<depth ) b=b1 , b.upSample( b1 ) , depth1++;
	}
	{
		BSplineElements< Degree2 > b;
		while( depth2<depth ) b=b2 , b.upSample( b2 ) , depth2++;
	}

	BSplineElements< Degree1-D1 > db1;
	BSplineElements< Degree2-D2 > db2;
	b1.template differentiate< D1 >( db1 ) , b2.template differentiate< D2 >( db2 );

	int start1=-1 , end1=-1 , start2=-1 , end2=-1;
	for( int i=0 ; i<int( b1.size() ) ; i++ )
	{
		for( int j=0 ; j<=Degree1 ; j++ )
		{
			if( b1[i][j] && start1==-1 ) start1 = i;
			if( b1[i][j] ) end1 = i+1;
		}
		for( int j=0 ; j<=Degree2 ; j++ )
		{
			if( b2[i][j] && start2==-1 ) start2 = i;
			if( b2[i][j] ) end2 = i+1;
		}
	}
	if( start1==end1 || start2==end2 || start1>=end2 || start2>=end1 ) return 0.;
	int start = std::max< int >( start1 , start2 ) , end = std::min< int >( end1 , end2 );
	memset( sums , 0 , sizeof( sums ) );

	// Iterate over the support
	for( int i=start ; i<end ; i++ )
		// Iterate over all pairs of elements within a node
		for( int j=0 ; j<=_Degree1 ; j++ ) for( int k=0 ; k<=_Degree2 ; k++ )
				// Accumulate the product of the coefficients
				sums[j][k] += db1[i][j] * db2[i][k];

	double _dot = 0;
	{
		double integrals[ _Degree1+1 ][ _Degree2+1 ];
		SetBSplineElementIntegrals< _Degree1 , _Degree2 >( integrals );
		for( int j=0 ; j<=_Degree1 ; j++ ) for( int k=0 ; k<=_Degree2 ; k++ ) _dot += integrals[j][k] * sums[j][k];
	}

	_dot /= b1.denominator;
	_dot /= b2.denominator;
	return ( !D1 && !D2 ) ? _dot / (1<<depth) : _dot * ( 1<<( depth*(D1+D2-1) ) );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 , unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< D1 , D2 , _D1 , _D2 , Integrator >::Set2D( Integrator& integrator , int depth )
{
	IntegratorSetter< D1-1 , D2 , _D1 , _D2 , Integrator >::Set2D( integrator , depth );
	IntegratorSetter< D1   , D2 , _D1 , _D2 , Integrator >::Set1D( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 , unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< D1 , D2 , _D1 , _D2 , Integrator >::Set1D( Integrator& integrator , int depth )
{
	IntegratorSetter< D1 , D2-1 , _D1 , _D2 , Integrator >::Set1D( integrator , depth );
	_IntegratorSetter< D1 , D2 , _D1 , _D2 >::Set( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D2 , unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< 0 , D2 , _D1 , _D2 , Integrator >::Set2D( Integrator& integrator , int depth )
{
	IntegratorSetter< 0 , D2 , _D1 , _D2 , Integrator >::Set1D( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D2 , unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< 0 , D2 , _D1 , _D2 , Integrator >::Set1D( Integrator& integrator , int depth )
{
	IntegratorSetter< 0 , D2-1 , _D1 , _D2 , Integrator >::Set1D( integrator , depth );
	_IntegratorSetter< 0 , D2 , _D1 , _D2 >::Set( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< D1 , 0 , _D1 , _D2 , Integrator >::Set2D( Integrator& integrator , int depth )
{
	IntegratorSetter< D1-1 , 0 , _D1 , _D2 , Integrator >::Set2D( integrator , depth );
	IntegratorSetter< D1   , 0 , _D1 , _D2 , Integrator >::Set1D( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< D1 , 0 , _D1 , _D2 , Integrator >::Set1D( Integrator& integrator , int depth )
{
	_IntegratorSetter< D1 , 0 , _D1 , _D2 >::Set( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< 0 , 0 , _D1 , _D2 , Integrator >::Set2D( Integrator& integrator , int depth )
{
	IntegratorSetter< 0 , 0 , _D1 , _D2 , Integrator >::Set1D( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int _D1 , unsigned int _D2 , class Integrator >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::IntegratorSetter< 0 , 0 , _D1 , _D2 , Integrator >::Set1D( Integrator& integrator , int depth )
{
	_IntegratorSetter< 0 , 0 , _D1 , _D2 >::Set( integrator , depth );
}

template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 , unsigned int _D1 , unsigned int _D2 >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::_IntegratorSetter< D1 , D2 , _D1 , _D2 >::Set( typename FunctionIntegrator::template Integrator< _D1 , _D2 >& integrator , int depth )
{
	for( int i=0 ; i<IndexSize ; i++ ) for( int j=BSplineOverlapSizes< Degree1 , Degree2 >::OverlapStart ; j<=BSplineOverlapSizes< Degree1 , Degree2 >::OverlapEnd ; j++ )
		{
			int ii = IndexToOffset( depth , i );
			integrator._ccIntegrals[D1][D2][i][j-BSplineOverlapSizes< Degree1 , Degree2 >::OverlapStart] = Dot< D1 , D2 >( depth , ii , depth , ii+j );
		}
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 , unsigned int _D1 , unsigned int _D2 >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::_IntegratorSetter< D1 , D2 , _D1 , _D2 >::Set( typename FunctionIntegrator::template ChildIntegrator< _D1 , _D2 >& integrator , int pDepth )
{
	for( int i=0 ; i<IndexSize ; i++ ) for( int j=BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapStart ; j<=BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapEnd ; j++ )
		{
			int ii = IndexToOffset( pDepth , i );
			integrator._pcIntegrals[D1][D2][i][j-BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapStart] = Dot< D1 , D2 >( pDepth , ii , pDepth+1 , 2*ii+j );
		}
}

template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::SetIntegrator( typename FunctionIntegrator::template Integrator< D1 , D2 >& integrator , int depth )
{
	integrator._depth = depth;
	IntegratorSetter< D1 , D2 , D1 , D2 , typename FunctionIntegrator::template Integrator< D1 , D2 > >::Set2D( integrator , depth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 >
void BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::SetChildIntegrator( typename FunctionIntegrator::template ChildIntegrator< D1 , D2 >& integrator , int parentDepth )
{
	integrator._parentDepth = parentDepth;
	IntegratorSetter< D1 , D2 , D1 , D2 , typename FunctionIntegrator::template ChildIntegrator< D1 , D2 > >::Set2D( integrator , parentDepth );
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 >
double BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::FunctionIntegrator::Integrator< D1 , D2 >::dot( int off1 , int off2 , int d1 , int d2 ) const
{
	int d = off2-off1;
	if( BSplineEvaluationData< Degree1 , BType1 >::OutOfBounds( _depth , off1 ) || BSplineEvaluationData< Degree2 , BType2 >::OutOfBounds( _depth , off2 ) || d<BSplineOverlapSizes< Degree1 , Degree2 >::OverlapStart || d>BSplineOverlapSizes< Degree1 , Degree2 >::OverlapEnd ) return 0;
	return _ccIntegrals[d1][d2][ OffsetToIndex( _depth , off1 ) ][d-BSplineOverlapSizes< Degree1 , Degree2 >::OverlapStart];
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< unsigned int D1 , unsigned int D2 >
double BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::FunctionIntegrator::ChildIntegrator< D1 , D2 >::dot( int off1 , int off2 , int d1 , int d2 ) const
{
	int d = off2-2*off1;
	if( BSplineEvaluationData< Degree1 , BType1 >::OutOfBounds( _parentDepth , off1 ) || BSplineEvaluationData< Degree2 , BType2 >::OutOfBounds( _parentDepth+1 , off2 ) || d<BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapStart || d>BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapEnd ) return 0;
	return _pcIntegrals[d1][d2][ OffsetToIndex( _parentDepth , off1 ) ][d-BSplineOverlapSizes< Degree1 , Degree2 >::ChildOverlapStart];
}

/////////////////
// BSplineData //
/////////////////
#define MODULO( A , B ) ( (A)<0 ? ( (B)-((-(A))%(B)) ) % (B) : (A) % (B) )
template< int Degree , BoundaryType BType >
int BSplineData< Degree , BType >::RemapOffset( int depth , int offset , bool& reflect )
{
	const int I = ( Degree&1 ) ? 0 : 1;
	if( BType==BOUNDARY_FREE ){ reflect = false ; return offset; }
	int dim = BSplineEvaluationData< Degree , BOUNDARY_NEUMANN >::End( depth ) - BSplineEvaluationData< Degree , BOUNDARY_NEUMANN >::Begin( depth );
	offset = MODULO( offset , 2*(dim-1+I) );
	reflect = offset>=dim;
	if( reflect ) return 2*(dim-1+I) - (offset+I);
	else          return offset;
}
#undef MODULO

template< int Degree , BoundaryType BType >
BSplineData< Degree , BType >::BSplineData( int maxDepth )
{
	functionCount = TotalFunctionCount( maxDepth );
	baseBSplines = NewPointer< typename BSplineEvaluationData< Degree , BType >::BSplineComponents >( functionCount );
	dBaseBSplines = NewPointer< typename BSplineEvaluationData< Degree , BType >::BSplineComponents >( functionCount );

	for( size_t i=0 ; i<functionCount ; i++ )
	{
		int d , off;
		FactorFunctionIndex( (int)i , d , off );
		baseBSplines[i] = typename BSplineEvaluationData< Degree , BType >::BSplineComponents( d , off );
		dBaseBSplines[i] = baseBSplines[i].derivative();
	}
}
template< int Degree , BoundaryType BType >
BSplineData< Degree , BType >::~BSplineData( void )
{
	FreePointer(  baseBSplines );
	FreePointer( dBaseBSplines );
}

/////////////////////
// BSplineElements //
/////////////////////
template< int Degree >
BSplineElements< Degree >::BSplineElements( int res , int offset , BoundaryType bType )
{
	denominator = 1;
	std::vector< BSplineElementCoefficients< Degree > >::resize( res , BSplineElementCoefficients< Degree >() );

	// If we have primal dirichlet constraints, the boundary functions are necessarily zero
	if( _Primal && bType==BOUNDARY_DIRICHLET && !(offset%res) ) return;

	// Construct the B-Spline
	for( int i=0 ; i<=Degree ; i++ )
	{
		int idx = -_Off + offset + i;
		if( idx>=0 && idx<res ) (*this)[idx][i] = 1;
	}
	if( bType!=BOUNDARY_FREE )
	{
		// Fold in the periodic instances (which cancels the negation)
		_addPeriodic< true >( _RotateLeft ( offset , res ) , false ) , _addPeriodic< false >( _RotateRight( offset , res ) , false );

		// Recursively fold in the boundaries
		if( _Primal && !(offset%res) ) return;

		// Fold in the reflected instance (which may require negation)
		_addPeriodic< true >( _ReflectLeft( offset , res ) , bType==BOUNDARY_DIRICHLET ) , _addPeriodic< false >( _ReflectRight( offset , res ) , bType==BOUNDARY_DIRICHLET );
	}
}
template< int Degree > int BSplineElements< Degree >::_ReflectLeft ( int offset , int res ){ return (Degree&1) ?      -offset :      -1-offset; }
template< int Degree > int BSplineElements< Degree >::_ReflectRight( int offset , int res ){ return (Degree&1) ? 2*res-offset : 2*res-1-offset; }
template< int Degree > int BSplineElements< Degree >::_RotateLeft  ( int offset , int res ){ return offset-2*res; }
template< int Degree > int BSplineElements< Degree >::_RotateRight ( int offset , int res ){ return offset+2*res; }

template< int Degree >
template< bool Left >
void BSplineElements< Degree >::_addPeriodic( int offset , bool negate )
{
	int res = int( std::vector< BSplineElementCoefficients< Degree > >::size() );
	bool set = false;
	// Add in the corresponding B-spline elements (possibly negated)
	for( int i=0 ; i<=Degree ; i++ )
	{
		int idx = -_Off + offset + i;
		if( idx>=0 && idx<res ) (*this)[idx][i] += negate ? -1 : 1 , set = true;
	}
	// If there is a change for additional overlap, give it a go
	if( set ) _addPeriodic< Left >( Left ? _RotateLeft( offset , res ) : _RotateRight( offset , res ) , negate );
}
template< int Degree >
void BSplineElements< Degree >::upSample( BSplineElements< Degree >& high ) const
{
	int bCoefficients[ BSplineSupportSizes< Degree >::UpSampleSize ];
	Polynomial< Degree+1 >::BinomialCoefficients( bCoefficients );

	high.resize( std::vector< BSplineElementCoefficients< Degree > >::size()*2 );
	high.assign( high.size() , BSplineElementCoefficients< Degree >() );
	// [NOTE] We have flipped the order of the B-spline elements
	for( int i=0 ; i<int(std::vector< BSplineElementCoefficients< Degree > >::size()) ; i++ ) for( int j=0 ; j<=Degree ; j++ )
		{
			// At index I , B-spline element J corresponds to a B-spline centered at:
			//		I - SupportStart - J
			int idx = i - BSplineSupportSizes< Degree >::SupportStart - j;
			for( int k=BSplineSupportSizes< Degree >::UpSampleStart ; k<=BSplineSupportSizes< Degree >::UpSampleEnd ; k++ )
			{
				// Index idx at the coarser resolution gets up-sampled into indices:
				//		2*idx + [UpSampleStart,UpSampleEnd]
				// at the finer resolution
				int _idx = 2*idx + k;
				// Compute the index of the B-spline element relative to 2*i and 2*i+1
				int _j1 = -_idx + 2*i - BSplineSupportSizes< Degree >::SupportStart , _j2 = -_idx + 2*i + 1 - BSplineSupportSizes< Degree >::SupportStart;
				if( _j1>=0 && _j1<=Degree ) high[2*i+0][_j1] += (*this)[i][j] * bCoefficients[k-BSplineSupportSizes< Degree >::UpSampleStart];
				if( _j2>=0 && _j2<=Degree ) high[2*i+1][_j2] += (*this)[i][j] * bCoefficients[k-BSplineSupportSizes< Degree >::UpSampleStart];
			}
		}
	high.denominator = denominator<<Degree;
}

template< int Degree >
template< unsigned int D >
void BSplineElements< Degree >::differentiate( BSplineElements< Degree-D >& d ) const{ Differentiator< Degree , Degree-D >::Differentiate( *this , d ); }

template< int Degree , int DDegree >
void Differentiator< Degree , DDegree >::Differentiate( const BSplineElements< Degree >& bse , BSplineElements< DDegree >& dbse )
{
	BSplineElements< Degree-1 > _dbse;
	_dbse.resize( bse.size() );
	_dbse.assign( _dbse.size() , BSplineElementCoefficients< Degree-1 >() );
	for( int i=0 ; i<(int)bse.size() ; i++ ) for( int j=0 ; j<=Degree ; j++ )
		{
			if( j-1>=0 )   _dbse[i][j-1] -= bse[i][j];
			if( j<Degree ) _dbse[i][j  ] += bse[i][j];
		}
	_dbse.denominator = bse.denominator;
	return Differentiator< Degree-1 , DDegree >::Differentiate( _dbse , dbse );
}

template< int Degree >
void Differentiator< Degree , Degree >::Differentiate( const BSplineElements< Degree >& bse , BSplineElements< Degree >& dbse ){ dbse = bse; }

// If we were really good, we would implement this integral table to store
// rational values to improve precision...
template< int Degree1 , int Degree2 >
void SetBSplineElementIntegrals( double integrals[Degree1+1][Degree2+1] )
{
	for( int i=0 ; i<=Degree1 ; i++ )
	{
		Polynomial< Degree1 > p1 = Polynomial< Degree1 >::BSplineComponent( Degree1-i );
		for( int j=0 ; j<=Degree2 ; j++ )
		{
			Polynomial< Degree2 > p2 = Polynomial< Degree2 >::BSplineComponent( Degree2-j );
			integrals[i][j] = ( p1 * p2 ).integral( 0 , 1 );
		}
	}
}
#endif // BSPLINE_DATA_INCLUDED