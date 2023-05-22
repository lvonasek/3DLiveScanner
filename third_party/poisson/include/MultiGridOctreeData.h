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
// [COMMENTS]
// -- Throughout the code, should make a distinction between indices and offsets
// -- Make an instance of _evaluate that samples the finite-elements correctly (specifically, to handle the boundaries)
// -- Make functions like depthAndOffset parity dependent (ideally all "depth"s should be relative to the B-Slpline resolution
// -- Make all points relative to the unit-cube, regardless of degree parity
// -- It's possible that for odd degrees, the iso-surfacing will fail because the leaves in the SortedTreeNodes do not form a partition of space
// -- [MAYBE] Treat normal field as a sum of delta functions, rather than a smoothed signal (again, so that high degrees aren't forced to generate smooth reconstructions)
// -- [MAYBE] Make the degree of the B-Spline with which the normals are splatted independent of the degree of the FEM system. (This way, higher degree systems aren't forced to generate smoother normal fields.)
// -- [MAYBE] Remove the isValidFEM/isValidSpace functions since the octree supports all degrees/boundary types (up to the max degree for which finalizedBrooded... was called)

// [TODO]
// -- Currently, the implementation assumes that the boundary constraints are the same for vector fields and scalar fields
// -- Modify the setting of the flags so that only the subset of the broods that are needed

#ifndef MULTI_GRID_OCTREE_DATA_INCLUDED
#define MULTI_GRID_OCTREE_DATA_INCLUDED

#define FAST_SET_UP				// If enabled, kernel density estimation is done aglomeratively

#define POINT_DATA_RES 0		// Specifies the resolution of the subgrid storing points with each voxel (0==1 but is faster)

#define DATA_DEGREE 1			// The order of the B-Spline used to splat in data for color interpolation
#define WEIGHT_DEGREE 2			// The order of the B-Spline used to splat in the weights for density estimation
#define NORMAL_DEGREE 2			// The order of the B-Spline used to splat int the normals for constructing the Laplacian constraints

#include <unordered_map>
#include "BSplineData.h"
#include "PointStream.h"
#include "Geometry.h"
#include "Octree.h"
#include "SparseMatrix.h"

int get_thread_num( void ){ return 0; }

#define DERIVATIVES( Degree ) ( ( Degree>1 ) ? 2 : ( Degree==1 ? 1 : 0 ) )

#ifdef FAST_SET_UP
#include <functional>
#endif // FAST_SET_UP
#include <cmath>
#include "PointStream.h"
#include "Octree.h"
#include "MAT.h"

class TreeNodeData
{
public:
	enum
	{
		SPACE_FLAG = 1 ,
		FEM_FLAG = 2 ,
		GHOST_FLAG = 1<<7
	};
	int nodeIndex;
	char flags;

	void setGhostFlag( bool f ){ if( f ) flags |= GHOST_FLAG ; else flags &= ~GHOST_FLAG; }
	bool getGhostFlag( void ) const { return ( flags & GHOST_FLAG )!=0; }
	TreeNodeData( void );
	~TreeNodeData( void );
};

class VertexData
{
	typedef OctNode< TreeNodeData > TreeOctNode;
public:
	static const int VERTEX_COORDINATE_SHIFT = ( sizeof( long long ) * 8 ) / 3;
	static long long   EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth , int index[DIMENSION] );
	static long long   EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth );
	static long long   FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth,int index[DIMENSION] );
	static long long   FaceIndex( const TreeOctNode* node , int fIndex , int maxDepth );
	static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth , int index[DIMENSION] );
	static long long CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth );
	static long long CenterIndex( const TreeOctNode* node , int maxDepth , int index[DIMENSION] );
	static long long CenterIndex( const TreeOctNode* node , int maxDepth );
	static long long CornerIndex( int depth , const int offSet[DIMENSION] , int cIndex , int maxDepth , int index[DIMENSION] );
	static long long CenterIndex( int depth , const int offSet[DIMENSION] , int maxDepth , int index[DIMENSION] );
	static long long CornerIndexKey( const int index[DIMENSION] );
};

// This class stores the octree nodes, sorted by depth and then by z-slice.
// To support primal representations, the initializer takes a function that
// determines if a node should be included/indexed in the sorted list.
// [NOTE] Indexing of nodes is _GLOBAL_
class SortedTreeNodes
{
	typedef OctNode< TreeNodeData > TreeOctNode;
protected:
	Pointer( Pointer( int ) ) _sliceStart;
	int _levels;
public:
	Pointer( TreeOctNode* ) treeNodes;
	int begin( int depth ) const{ return _sliceStart[depth][0]; }
	int   end( int depth ) const{ return _sliceStart[depth][(size_t)1<<depth]; }
	int begin( int depth , int slice ) const{ return _sliceStart[depth][slice  ]  ; }
	int   end( int depth , int slice ) const{ if(depth<0||depth>=_levels||slice<0||slice>=(1<<depth)) printf( "uh oh\n" ) ; return _sliceStart[depth][slice+1]; }
	int size( void ) const { return _sliceStart[_levels-1][(size_t)1<<(_levels-1)]; }
	int size( int depth ) const { if(depth<0||depth>=_levels) printf( "uhoh\n" ); return _sliceStart[depth][(size_t)1<<depth] - _sliceStart[depth][0]; }
	int size( int depth , int slice ) const { return _sliceStart[depth][slice+1] - _sliceStart[depth][slice]; }
	int levels( void ) const { return _levels; }

	SortedTreeNodes( void );
	~SortedTreeNodes( void );
	void set( TreeOctNode& root , std::vector< int >* map );
	void set( TreeOctNode& root );

	template< int Indices >
	struct  _Indices
	{
		int idx[Indices];
		_Indices( void ){ memset( idx , -1 , sizeof( int ) * Indices ); }
		int& operator[] ( int i ) { return idx[i]; }
		const int& operator[] ( int i ) const { return idx[i]; }
	};
	typedef _Indices< Square::CORNERS > SquareCornerIndices;
	typedef _Indices< Square::EDGES > SquareEdgeIndices;
	typedef _Indices< Square::FACES > SquareFaceIndices;

	struct SliceTableData
	{
		Pointer( SquareCornerIndices ) cTable;
		Pointer( SquareEdgeIndices   ) eTable;
		Pointer( SquareFaceIndices   ) fTable;
		int cCount , eCount , fCount , nodeOffset , nodeCount;
		SliceTableData( void ){ fCount = eCount = cCount = 0 , cTable = NullPointer( SquareCornerIndices ) , eTable = NullPointer( SquareEdgeIndices ) , fTable = NullPointer( SquareFaceIndices ) , _cMap = _eMap = _fMap = NullPointer( int ); }
		~SliceTableData( void ){ clear(); }
		void clear( void ){ DeletePointer( cTable ) ; DeletePointer( eTable ) ; DeletePointer( fTable ) ; DeletePointer( _cMap ) ; DeletePointer( _eMap ) ; DeletePointer( _fMap ) ; fCount = eCount = cCount = 0; }
		SquareCornerIndices& cornerIndices( const TreeOctNode* node );
		SquareCornerIndices& cornerIndices( int idx );
		const SquareCornerIndices& cornerIndices( const TreeOctNode* node ) const;
		const SquareCornerIndices& cornerIndices( int idx ) const;
		SquareEdgeIndices& edgeIndices( const TreeOctNode* node );
		SquareEdgeIndices& edgeIndices( int idx );
		const SquareEdgeIndices& edgeIndices( const TreeOctNode* node ) const;
		const SquareEdgeIndices& edgeIndices( int idx ) const;
		SquareFaceIndices& faceIndices( const TreeOctNode* node );
		SquareFaceIndices& faceIndices( int idx );
		const SquareFaceIndices& faceIndices( const TreeOctNode* node ) const;
		const SquareFaceIndices& faceIndices( int idx ) const;
	protected:
		Pointer( int ) _cMap;
		Pointer( int ) _eMap;
		Pointer( int ) _fMap;
		friend class SortedTreeNodes;
	};
	struct XSliceTableData
	{
		Pointer( SquareCornerIndices ) eTable;
		Pointer( SquareEdgeIndices ) fTable;
		int fCount , eCount , nodeOffset , nodeCount;
		XSliceTableData( void ){ fCount = eCount = 0 , eTable = NullPointer( SquareCornerIndices ) , fTable = NullPointer( SquareEdgeIndices ) , _eMap = _fMap = NullPointer( int ); }
		~XSliceTableData( void ){ clear(); }
		void clear( void ) { DeletePointer( fTable ) ; DeletePointer( eTable ) ; DeletePointer( _eMap ) ; DeletePointer( _fMap ) ; fCount = eCount = 0; }
		SquareCornerIndices& edgeIndices( const TreeOctNode* node );
		SquareCornerIndices& edgeIndices( int idx );
		const SquareCornerIndices& edgeIndices( const TreeOctNode* node ) const;
		const SquareCornerIndices& edgeIndices( int idx ) const;
		SquareEdgeIndices& faceIndices( const TreeOctNode* node );
		SquareEdgeIndices& faceIndices( int idx );
		const SquareEdgeIndices& faceIndices( const TreeOctNode* node ) const;
		const SquareEdgeIndices& faceIndices( int idx ) const;
	protected:
		Pointer( int ) _eMap;
		Pointer( int ) _fMap;
		friend class SortedTreeNodes;
	};
	void setSliceTableData (  SliceTableData& sData , int depth , int offset , int threads ) const;
	void setXSliceTableData( XSliceTableData& sData , int depth , int offset , int threads ) const;
};

template< int Degree >
struct PointSupportKey : public OctNode< TreeNodeData >::NeighborKey< BSplineSupportSizes< Degree >::SupportEnd , -BSplineSupportSizes< Degree >::SupportStart >
{
	static const int LeftRadius  =  BSplineSupportSizes< Degree >::SupportEnd;
	static const int RightRadius = -BSplineSupportSizes< Degree >::SupportStart;
	static const int Size = LeftRadius + RightRadius + 1;
};
template< int Degree >
struct ConstPointSupportKey : public OctNode< TreeNodeData >::ConstNeighborKey< BSplineSupportSizes< Degree >::SupportEnd , -BSplineSupportSizes< Degree >::SupportStart >
{
	static const int LeftRadius  =  BSplineSupportSizes< Degree >::SupportEnd;
	static const int RightRadius = -BSplineSupportSizes< Degree >::SupportStart;
	static const int Size = LeftRadius + RightRadius + 1;
};

template< class Real , bool HasGradients >
struct SinglePointData
{
	Point3D< Real > position;
	Real weight;
	Real value , _value;
	SinglePointData  operator +  ( const SinglePointData& p ) const { return SinglePointData( position + p.position , value + p.value , weight + p.weight ); }
	SinglePointData& operator += ( const SinglePointData& p ){ position += p.position ; weight += p.weight , value += p.value ; return *this; }
	SinglePointData  operator *  ( Real s ) const { return SinglePointData( position*s , weight*s , value*s ); }
	SinglePointData& operator *= ( Real s ){ position *= s , weight *= s , value *= s ; return *this; }
	SinglePointData  operator /  ( Real s ) const { return SinglePointData( position/s , weight/s , value/s ); }
	SinglePointData& operator /= ( Real s ){ position /= s , weight /= s , value /= s ; return *this; }
	SinglePointData( void ) : position( Point3D< Real >() ) , weight(0) , value(0) , _value(0) { ; }
	SinglePointData( Point3D< Real > p , Real v , Real w ) { position = p , value = v , weight = w , _value = (Real)0; }
};
template< class Real >
struct SinglePointData< Real , true > : public SinglePointData< Real , false >
{
	using SinglePointData< Real , false >::position;
	using SinglePointData< Real , false >::weight;
	using SinglePointData< Real , false >::value;
	using SinglePointData< Real , false >::_value;
	Point3D< Real > gradient , _gradient;
	SinglePointData  operator +  ( const SinglePointData& p ) const { return SinglePointData( position + p.position , weight + p.weight , value + p.value , gradient + p.gradient ); }
	SinglePointData& operator += ( const SinglePointData& p ){ position += p.position , weight += p.weight , value += p.value , gradient += p.gradient ; return *this; }
	SinglePointData  operator *  ( Real s ) const { return SinglePointData( position*s , weight*s , value*s , gradient*s ); }
	SinglePointData& operator *= ( Real s ){ position *= s , weight *= s , value *= s , gradient *= s ; return *this; }
	SinglePointData  operator /  ( Real s ) const { return SinglePointData( position/s , weight/s , value/s , gradient/s ); }
	SinglePointData& operator /= ( Real s ){ position /= s , weight /= s , value /= s , gradient /= s ; return *this; }
	SinglePointData( void ) : SinglePointData< Real , false >() , gradient( Point3D< Real >() ) , _gradient( Point3D< Real >() ) { ; }
	SinglePointData( Point3D< Real > p , Real v , Point3D< Real > g , Real w ) : SinglePointData< Real , false >( p , v , w ) { gradient = g , _gradient = Point3D< Real >(); }
};

#if POINT_DATA_RES
template< class Real , bool HasGradients >
struct PointData
{
	static const int RES = POINT_DATA_RES;
	static const int SAMPLES = RES * RES * RES;

	SinglePointData< Real , HasGradients > points[SAMPLES];
	SinglePointData< Real , HasGradients >& operator[] ( int idx ) { return points[idx]; }
	const SinglePointData< Real , HasGradients >& operator[] ( int idx ) const { return points[idx]; }

	static void SetIndices( Point3D< Real > p , Point3D< Real > c , Real w , int x[3] )
	{
		for( int d=0 ; d<3 ; d++ ) x[d] = std::max< int >( 0 , std::min< int >( RES-1 , int( floor( ( p[d]-( c[d]-w/2 ) ) / w * RES ) ) ) );
	}

	void addPoint( SinglePointData< Real , HasGradients > p , Point3D< Real > center , Real width  )
	{
		int x[3];
		SetIndices( p.position , center , width , x );
		points[ x[0]+x[1]*RES+x[2]*RES*RES ] += p;
	}

	PointData  operator +  ( const PointData& p ) const { PointData _p ; for( int c=0 ; c<SAMPLES ;  c++ ) _p.points[c] = points[c] + _p.points[c] ; return _p; }
	PointData& operator += ( const PointData& p ){ for( int c=0 ; c<SAMPLES ; c++ ) points[c] += p.points[c] ; return *this; }
	PointData  operator *  ( Real s ) const { PointData _p ; for( int c=0 ; c<SAMPLES ;  c++ ) _p.points[c] = points[c] * s ; return _p; }
	PointData& operator *= ( Real s ){ for( int c=0 ; c<SAMPLES ; c++ ) points[c] *= s ; return *this; }
	PointData  operator /  ( Real s ) const { PointData _p ; for( int c=0 ; c<SAMPLES ;  c++ ) _p.points[c] = points[c] / s ; return _p; }
	PointData& operator /= ( Real s ){ for( int c=0 ; c<SAMPLES ; c++ ) points[c] /= s ; return *this; }
};
#else // !POINT_DATA_RES
template< class Real , bool HasGradients > using PointData = SinglePointData< Real , HasGradients >;
#endif // POINT_DATA_RES

template< class Data , int Degree >
struct SparseNodeData
{
	size_t size( void ) const { return _data.size(); }
	const Data& operator[] ( int idx ) const { return _data[idx]; }
	Data& operator[] ( int idx ) { return _data[idx]; }
	void reserve( size_t sz ){ if( sz>_indices.size() ) _indices.resize( sz , -1 ); }
	Data* operator()( const OctNode< TreeNodeData >* node ){ return ( node->nodeData.nodeIndex<0 || node->nodeData.nodeIndex>=(int)_indices.size() || _indices[ node->nodeData.nodeIndex ]<0 ) ? NULL : &_data[ _indices[ node->nodeData.nodeIndex ] ]; }
	const Data* operator()( const OctNode< TreeNodeData >* node ) const { return ( node->nodeData.nodeIndex<0 || node->nodeData.nodeIndex>=(int)_indices.size() || _indices[ node->nodeData.nodeIndex ]<0 ) ? NULL : &_data[ _indices[ node->nodeData.nodeIndex ] ]; }
	Data& operator[]( const OctNode< TreeNodeData >* node )
	{
		if( node->nodeData.nodeIndex>=(int)_indices.size() ) _indices.resize( node->nodeData.nodeIndex+1 , -1 );
		if( _indices[ node->nodeData.nodeIndex ]==-1 )
		{
			_indices[ node->nodeData.nodeIndex ] = (int)_data.size();
			_data.push_back( Data() );
		}
		return _data[ _indices[ node->nodeData.nodeIndex ] ];
	}
	void remapIndices( const std::vector< int >& map )
	{
		std::vector< int > temp = _indices;
		_indices.resize( map.size() );
		for( size_t i=0 ; i<map.size() ; i++ )
			if( map[i]<(int)temp.size() ) _indices[i] = temp[ map[i] ];
			else                          _indices[i] = -1;
	}
	template< class _Data , int _Degree > friend struct SparseNodeData;
	template< class _Data , int _Degree >
	void init( const SparseNodeData< _Data , _Degree >& snd ){ _indices = snd._indices , _data.resize( snd._data.size() ); }
	void remove( const OctNode< TreeNodeData >* node ){ if( node->nodeData.nodeIndex<(int)_indices.size() && node->nodeData.nodeIndex>=0 ) _indices[ node->nodeData.nodeIndex ] = -1; }
protected:
	std::vector< int > _indices;
	std::vector< Data > _data;
};
template< class Data , int Degree >
struct DenseNodeData
{
	DenseNodeData( void ){ _data = NullPointer( Data ) ; _sz = 0; }
	DenseNodeData( size_t sz ){ _sz = sz ; if( sz ) _data = NewPointer< Data >( sz ) ; else _data = NullPointer( Data ); }
	DenseNodeData( const DenseNodeData&  d ) : DenseNodeData() { _resize( d._sz ) ; if( _sz ) memcpy( _data , d._data , sizeof(Data) * _sz ); }
	DenseNodeData(       DenseNodeData&& d ){ _data = d._data , _sz = d._sz ; d._data = NullPointer( Data ) , d._sz = 0; }
	DenseNodeData& operator = ( const DenseNodeData&  d ){ _resize( d._sz ) ; if( _sz ) memcpy( _data , d._data , sizeof(Data) * _sz ) ; return *this; }
	DenseNodeData& operator = (       DenseNodeData&& d ){ size_t __sz = _sz ; Pointer( Data ) __data = _data ; _data = d._data , _sz = d._sz ; d._data = __data , d._sz = __sz ; return *this; }
	~DenseNodeData( void ){ DeletePointer( _data ) ; _sz = 0; }

	Data& operator[] ( int idx ) { return _data[idx]; }
	const Data& operator[] ( int idx ) const { return _data[idx]; }
	size_t size( void ) const { return _sz; }
	Data& operator[]( const OctNode< TreeNodeData >* node ) { return _data[ node->nodeData.nodeIndex ]; }
	Data* operator()( const OctNode< TreeNodeData >* node ) { return ( node==NULL || node->nodeData.nodeIndex>=(int)_sz ) ? NULL : &_data[ node->nodeData.nodeIndex ]; }
	const Data* operator()( const OctNode< TreeNodeData >* node ) const { return ( node==NULL || node->nodeData.nodeIndex>=(int)_sz ) ? NULL : &_data[ node->nodeData.nodeIndex ]; }
	int index( const OctNode< TreeNodeData >* node ) const { return ( !node || node->nodeData.nodeIndex<0 || node->nodeData.nodeIndex>=_sz ) ? -1 : node->nodeData.nodeIndex; }
protected:
	size_t _sz;
	void _resize( size_t sz ){ DeletePointer( _data ) ; if( sz ) _data = NewPointer< Data >( sz ) ; else _data = NullPointer( Data ) ; _sz = sz; }
	Pointer( Data ) _data;
};

// This is may be necessary in case the memory usage is larger than what fits on the stack
template< class C , int N > struct Stencil
{
	Stencil( void ){ _values = NewPointer< C >( N * N * N ); }
	~Stencil( void ){ DeletePointer( _values ); }
	C& operator()( int i , int j , int k ){ return _values[ i*N*N + j*N + k ]; }
	const C& operator()( int i , int j , int k ) const { return _values[ i*N*N + j*N + k ]; }
protected:
	Pointer( C ) _values;
};

template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
class SystemCoefficients
{
	static const int OverlapSize  = BSplineOverlapSizes< Degree1 , Degree2 >::OverlapSize;
	static const int OverlapStart = BSplineOverlapSizes< Degree1 , Degree2 >::OverlapStart;
	static const int OverlapEnd   = BSplineOverlapSizes< Degree1 , Degree2 >::OverlapEnd;
public:
	typedef typename BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::FunctionIntegrator::template      Integrator< DERIVATIVES( Degree1 ) , DERIVATIVES( Degree2 ) >      Integrator;
	typedef typename BSplineIntegrationData< Degree1 , BType1 , Degree2 , BType2 >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( Degree1 ) , DERIVATIVES( Degree2 ) > ChildIntegrator;

	// The FEMSystemFunctor is a class that takes an object of type Integrator/ChildIntegrator, as well as a pair of indices of octree nodes
	// and returns the corresponding system coefficient.
	template< class _FEMSystemFunctor > static void SetCentralSystemStencil ( const _FEMSystemFunctor& F , const      Integrator& integrator , Stencil< double , OverlapSize >& stencil           );
	template< class _FEMSystemFunctor > static void SetCentralSystemStencils( const _FEMSystemFunctor& F , const ChildIntegrator& integrator , Stencil< double , OverlapSize >  stencils[2][2][2] );
	template< bool Reverse , class _FEMSystemFunctor > static void SetCentralConstraintStencil ( const _FEMSystemFunctor& F , const      Integrator& integrator , Stencil<          double   , OverlapSize >& stencil           );
	template< bool Reverse , class _FEMSystemFunctor > static void SetCentralConstraintStencils( const _FEMSystemFunctor& F , const ChildIntegrator& integrator , Stencil<          double   , OverlapSize >  stencils[2][2][2] );
	template< bool Reverse , class _FEMSystemFunctor > static void SetCentralConstraintStencil ( const _FEMSystemFunctor& F , const      Integrator& integrator , Stencil< Point3D< double > , OverlapSize >& stencil           );
	template< bool Reverse , class _FEMSystemFunctor > static void SetCentralConstraintStencils( const _FEMSystemFunctor& F , const ChildIntegrator& integrator , Stencil< Point3D< double > , OverlapSize >  stencils[2][2][2] );
};

template< int FEMDegree , BoundaryType BType >
struct FEMSystemFunctor
{
	double massWeight , lapWeight , biLapWeight;
	FEMSystemFunctor( double mWeight=0 , double lWeight=0 , double bWeight=0 ) : massWeight( mWeight ) , lapWeight( lWeight ) , biLapWeight( bWeight ) { ; }
	double integrate( const typename SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::     Integrator& integrator , const int off1[] , const int off2[] ) const { return _integrate( integrator , off1 , off2 ); }
	double integrate( const typename SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::ChildIntegrator& integrator , const int off1[] , const int off2[] ) const { return _integrate( integrator , off1 , off2 ); }
	bool vanishesOnConstants( void ) const { return massWeight==0; }
protected:
	template< class I > double _integrate( const I& integrator , const int off1[] , const int off2[] ) const;
};

template< int VFDegree , BoundaryType VFBType , int FEMDegree , BoundaryType FEMBType >
struct FEMVFConstraintFunctor
{
	double lapWeight , biLapWeight;
	FEMVFConstraintFunctor( double lWeight=0 , double bWeight=0 ) : lapWeight( lWeight ) , biLapWeight( bWeight ) { ; }
	template< bool Reverse >
	Point3D< double > integrate( const typename SystemCoefficients< Reverse ? FEMDegree : VFDegree , Reverse ? FEMBType : VFBType , Reverse ? VFDegree : FEMDegree , Reverse ? VFBType : FEMBType >::     Integrator& integrator , const int off1[] , const int off2[] ) const { return _integrate< Reverse >( integrator , off1 , off2 ); }
	template< bool Reverse >
	Point3D< double > integrate( const typename SystemCoefficients< Reverse ? FEMDegree : VFDegree , Reverse ? FEMBType : VFBType , Reverse ? VFDegree : FEMDegree , Reverse ? VFBType : FEMBType >::ChildIntegrator& integrator , const int off1[] , const int off2[] ) const { return _integrate< Reverse >( integrator , off1 , off2 ); }
protected:
	template< bool Reverse , class I > Point3D< double > _integrate( const I& integrator , const int off1[] , const int off[2] ) const;
};

inline void SetGhostFlag( OctNode< TreeNodeData >* node , bool flag ){ if( node && node->parent ) node->parent->nodeData.setGhostFlag( flag ); }
inline bool GetGhostFlag( const OctNode< TreeNodeData >* node ){ return node==NULL || node->parent==NULL || node->parent->nodeData.getGhostFlag( ); }
inline bool IsActiveNode( const OctNode< TreeNodeData >* node ){ return !GetGhostFlag( node ); }

template< class Real >
class Octree
{
	typedef OctNode< TreeNodeData > TreeOctNode;
	static int _NodeCount;
	static void _NodeInitializer( TreeOctNode& node ){ node.nodeData.nodeIndex = _NodeCount++; }
public:
#if 0
	struct LocalDepth
	{
		LocalDepth( int d=0 ) : _d(d) { ; }
		operator int&()       { return _d; }
		operator int () const { return _d; }
	protected:
		int _d;
	};
	struct LocalOffset
	{
		LocalOffset( const int* off=NULL ){ if( off ) memcpy( _off , off , sizeof(_off) ) ; else memset( _off , 0 , sizeof( _off ) ); }
		operator        int*()       { return _off; }
		operator const  int*() const { return _off; }
	protected:
		int _off[3];
	};
#else
	typedef int LocalDepth;
	typedef int LocalOffset[3];
#endif

	static void ResetNodeCount( void ){ _NodeCount = 0 ; }
	static int NodeCount( void ){ return _NodeCount; }
	template< int FEMDegree , BoundaryType BType > void functionIndex( const TreeOctNode* node , int idx[3] ) const;

	struct PointSample{ const TreeOctNode* node ; ProjectiveData< OrientedPoint3D< Real > , Real > sample; };

	typedef typename TreeOctNode::     NeighborKey< 1 , 1 >      AdjacenctNodeKey;
	typedef typename TreeOctNode::ConstNeighborKey< 1 , 1 > ConstAdjacenctNodeKey;

	template< int FEMDegree , BoundaryType BType > bool isValidFEMNode( const TreeOctNode* node ) const;
	bool isValidSpaceNode( const TreeOctNode* node ) const;
	TreeOctNode* leaf( Point3D< Real > p );
	const TreeOctNode* leaf( Point3D< Real > p ) const;

	template< bool HasGradients >
	struct InterpolationInfo
	{
		SparseNodeData< PointData< Real , HasGradients > , 0 > iData;
		Real valueWeight , gradientWeight;
		InterpolationInfo( const class Octree< Real >& tree , const std::vector< PointSample >& samples , Real pointValue , int adaptiveExponent , Real v , Real g ) : valueWeight(v) , gradientWeight(g)
		{ iData = tree._densifyInterpolationInfo< HasGradients >( samples , pointValue , adaptiveExponent ); }
		PointData< Real , HasGradients >* operator()( const OctNode< TreeNodeData >* node ){ return iData(node); }
		const PointData< Real , HasGradients >* operator()( const OctNode< TreeNodeData >* node ) const { return iData(node); }
	};

	template< int DensityDegree > struct DensityEstimator : public SparseNodeData< Real , DensityDegree >
	{
		DensityEstimator( int kernelDepth ) : _kernelDepth( kernelDepth ){ ; }
		int kernelDepth( void ) const { return _kernelDepth; }
	protected:
		int _kernelDepth;
	};
protected:
	bool _isValidSpaceNode( const TreeOctNode* node ) const { return !GetGhostFlag( node ) && ( node->nodeData.flags & TreeNodeData::SPACE_FLAG ); }
	bool _isValidFEMNode( const TreeOctNode* node ) const { return !GetGhostFlag( node ) && ( node->nodeData.flags & TreeNodeData::FEM_FLAG ); }

	TreeOctNode* _tree;
	TreeOctNode* _spaceRoot;
	SortedTreeNodes _sNodes;
	LocalDepth _fullDepth , _maxDepth;

	static bool _InBounds( Point3D< Real > p );

	int _depthOffset;
	int _localToGlobal( LocalDepth d ) const { return d + _depthOffset; }
	LocalDepth _localDepth( const TreeOctNode* node ) const { return node->depth() - _depthOffset; }
	LocalDepth _localMaxDepth( const TreeOctNode* tree ) const { return tree->maxDepth() - _depthOffset; }
	int _localInset( LocalDepth d ) const { return _depthOffset<=1 ? 0 : 1<<( d + _depthOffset - 1 ); }
	void _localDepthAndOffset( const TreeOctNode* node , LocalDepth& d , LocalOffset& off ) const
	{
		node->depthAndOffset( d , off ) ; d -= _depthOffset;
		int inset = _localInset( d );
		off[0] -= inset , off[1] -= inset , off[2] -= inset;
	}
	template< int FEMDegree , BoundaryType BType > static int _BSplineBegin( LocalDepth depth ){ return BSplineEvaluationData< FEMDegree , BType >::Begin( depth ); }
	template< int FEMDegree , BoundaryType BType > static int _BSplineEnd  ( LocalDepth depth ){ return BSplineEvaluationData< FEMDegree , BType >::End  ( depth ); }
	template< int FEMDegree , BoundaryType BType >
	bool _outOfBounds( const TreeOctNode* node ) const
	{
		if( !node ) return true;
		LocalDepth d ; LocalOffset off;
		_localDepthAndOffset( node , d , off );
		return d<0 || BSplineEvaluationData< FEMDegree , BType >::OutOfBounds( d , off[0] ) || BSplineEvaluationData< FEMDegree , BType >::OutOfBounds( d , off[1] ) || BSplineEvaluationData< FEMDegree , BType >::OutOfBounds( d , off[2] );
	}
	int _sNodesBegin( LocalDepth d ) const { return _sNodes.begin( _localToGlobal( d ) ); }
	int _sNodesEnd  ( LocalDepth d ) const { return _sNodes.end  ( _localToGlobal( d ) ); }
	int _sNodesSize ( LocalDepth d ) const { return _sNodes.size ( _localToGlobal( d ) ); }
	int _sNodesBegin( LocalDepth d , int slice ) const { return _sNodes.begin( _localToGlobal( d ) , slice + _localInset( d ) ); }
	int _sNodesEnd  ( LocalDepth d , int slice ) const { return _sNodes.end  ( _localToGlobal( d ) , slice + _localInset( d ) ); }
	int _sNodesSize ( LocalDepth d , int slice ) const { return _sNodes.size ( _localToGlobal( d ) , slice + _localInset( d ) ); }

	template< int FEMDegree > static bool _IsInteriorlySupported( LocalDepth depth , const LocalOffset off )
	{
		if( depth>=0 )
		{
			int begin , end;
			BSplineSupportSizes< FEMDegree >::InteriorSupportedSpan( depth , begin , end );
			return ( off[0]>=begin && off[0]<end && off[1]>=begin && off[1]<end && off[2]>=begin && off[2]<end );
		}
		else return false;
	}
	template< int FEMDegree > bool _isInteriorlySupported( const TreeOctNode* node ) const
	{
		if( !node ) return false;
		LocalDepth d ; LocalOffset off;
		_localDepthAndOffset( node , d , off );
		return _IsInteriorlySupported< FEMDegree >( d , off );
	}
	template< int FEMDegree1 , int FEMDegree2 > static bool _IsInteriorlyOverlapped( LocalDepth depth , const LocalOffset off )
	{
		if( depth>=0 )
		{
			int begin , end;
			BSplineIntegrationData< FEMDegree1 , BOUNDARY_NEUMANN , FEMDegree2 , BOUNDARY_NEUMANN >::InteriorOverlappedSpan( depth , begin , end );
			return ( off[0]>=begin && off[0]<end && off[1]>=begin && off[1]<end && off[2]>=begin && off[2]<end );
		}
		else return false;
	}
	template< int FEMDegree1 , int FEMDegree2 > bool _isInteriorlyOverlapped( const TreeOctNode* node ) const
	{
		if( !node ) return false;
		LocalDepth d ; LocalOffset off;
		_localDepthAndOffset( node , d , off );
		return _IsInteriorlyOverlapped< FEMDegree1 , FEMDegree2 >( d , off );
	}
	void _startAndWidth( const TreeOctNode* node , Point3D< Real >& start , Real& width ) const
	{
		LocalDepth d ; LocalOffset off;
		_localDepthAndOffset( node , d , off );
		if( d>=0 ) width = Real( 1.0 / (1<<  d ) );
		else       width = Real( 1.0 * (1<<(-d)) );
		for( int dd=0 ; dd<DIMENSION ; dd++ ) start[dd] = Real( off[dd] ) * width;
	}
	void _centerAndWidth( const TreeOctNode* node , Point3D< Real >& center , Real& width ) const
	{
		int d , off[3];
		_localDepthAndOffset( node , d , off );
		width = Real( 1.0 / (1<<d) );
		for( int dd=0 ; dd<DIMENSION ; dd++ ) center[dd] = Real( off[dd] + 0.5 ) * width;
	}
	int _childIndex( const TreeOctNode* node , Point3D< Real > p ) const
	{
		Point3D< Real > c ; Real w;
		_centerAndWidth( node , c , w );
		return ( p[0]<c[0] ? 0 : 1 ) | ( p[1]<c[1] ? 0 : 2 ) | ( p[2]<c[2] ? 0 : 4 );
	}

	template< int Degree , BoundaryType BType > void _setFullDepth( TreeOctNode* node , LocalDepth depth ) const;
	template< int Degree , BoundaryType BType > void _setFullDepth( LocalDepth depth );

	template< int LeftRadius , int RightRadius >
	static typename TreeOctNode::ConstNeighbors< LeftRadius + RightRadius + 1 >& _neighbors( TreeOctNode::ConstNeighborKey< LeftRadius , RightRadius >& key , const TreeOctNode* node ){ return key.neighbors[ node->depth() ]; }
	template< int LeftRadius , int RightRadius >
	static typename TreeOctNode::Neighbors< LeftRadius + RightRadius + 1 >& _neighbors( TreeOctNode::NeighborKey< LeftRadius , RightRadius >& key , const TreeOctNode* node ){ return key.neighbors[ node->depth() ]; }
	template< int LeftRadius , int RightRadius >
	static const typename TreeOctNode::template Neighbors< LeftRadius + RightRadius + 1 >& _neighbors( const typename TreeOctNode::template NeighborKey< LeftRadius , RightRadius >& key , const TreeOctNode* node ){ return key.neighbors[ node->depth() ]; }
	template< int LeftRadius , int RightRadius >
	static const typename TreeOctNode::template ConstNeighbors< LeftRadius + RightRadius + 1 >& _neighbors( const typename TreeOctNode::template ConstNeighborKey< LeftRadius , RightRadius >& key , const TreeOctNode* node ){ return key.neighbors[ node->depth() ]; }

public:
	LocalDepth depth( const TreeOctNode* node ) const { return _localDepth( node ); }
	void depthAndOffset( const TreeOctNode* node , LocalDepth& depth , LocalOffset& offset ) const { _localDepthAndOffset( node , depth , offset ); }

	int nodesBegin( LocalDepth d ) const { return _sNodes.begin( _localToGlobal( d ) ); }
	int nodesEnd  ( LocalDepth d ) const { return _sNodes.end  ( _localToGlobal( d ) ); }
	int nodesSize ( LocalDepth d ) const { return _sNodes.size ( _localToGlobal( d ) ); }
	int nodesBegin( LocalDepth d , int slice ) const { return _sNodes.begin( _localToGlobal( d ) , slice + _localInset( d ) ); }
	int nodesEnd  ( LocalDepth d , int slice ) const { return _sNodes.end  ( _localToGlobal( d ) , slice + _localInset( d ) ); }
	int nodesSize ( LocalDepth d , int slice ) const { return _sNodes.size ( _localToGlobal( d ) , slice + _localInset( d ) ); }
	const TreeOctNode* node( int idx ) const { return _sNodes.treeNodes[idx]; }
protected:

	////////////////////////////////////
	// System construction code       //
	// MultiGridOctreeData.System.inl //
	////////////////////////////////////
	template< int FEMDegree >
	void _setMultiColorIndices( int start , int end , std::vector< std::vector< int > >& indices ) const;
	struct _SolverStats
	{
		double bNorm2 , inRNorm2 , outRNorm2;
	};
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	int _solveSystemGS( const FEMSystemFunctor& F , const BSplineData< FEMDegree , BType >& bsData , InterpolationInfo< HasGradients >* interpolationInfo , LocalDepth depth , DenseNodeData< Real , FEMDegree >& solution , DenseNodeData< Real , FEMDegree >& constraints , DenseNodeData< Real , FEMDegree >& metSolutionConstraints , int iters , bool coarseToFine , _SolverStats& stats , bool computeNorms );
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	int _solveSystemCG( const FEMSystemFunctor& F , const BSplineData< FEMDegree , BType >& bsData , InterpolationInfo< HasGradients >* interpolationInfo , LocalDepth depth , DenseNodeData< Real , FEMDegree >& solution , DenseNodeData< Real , FEMDegree >& constraints , DenseNodeData< Real , FEMDegree >& metSolutionConstraints , int iters , bool coarseToFine , _SolverStats& stats , bool computeNorms , double accuracy );
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	int _setMatrixRow( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& neighbors , Pointer( MatrixEntry< Real > ) row , int offset , const typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& integrator , const Stencil< double , BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& stencil , const BSplineData< FEMDegree , BType >& bsData ) const;
	template< int FEMDegree , BoundaryType BType >
	int _getMatrixRowSize( const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& neighbors ) const;

	template< int FEMDegree1 , int FEMDegree2 > static void _SetParentOverlapBounds( const TreeOctNode* node , int& startX , int& endX , int& startY , int& endY , int& startZ , int& endZ );
	// Updates the constraints @(depth) based on the solution coefficients @(depth-1)

	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	void _updateConstraintsFromCoarser( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& neighbors , const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& pNeighbors , TreeOctNode* node , DenseNodeData< Real , FEMDegree >& constraints , const DenseNodeData< Real , FEMDegree >& metSolution , const typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& childIntegrator , const Stencil< double , BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& stencil , const BSplineData< FEMDegree , BType >& bsData ) const;

	// evaluate the points @(depth) using coefficients @(depth-1)
	template< int FEMDegree , BoundaryType BType , bool HasGradients >
	void _setPointValuesFromCoarser( InterpolationInfo< HasGradients >& interpolationInfo , LocalDepth highDepth , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& upSampledCoefficients );

	// Updates the cumulative integral constraints @(depth-1) based on the change in solution coefficients @(depth)
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor >
	void _updateCumulativeIntegralConstraintsFromFiner( const FEMSystemFunctor& F ,
		const BSplineData< FEMDegree , BType >& bsData , LocalDepth highDepth , const DenseNodeData< Real , FEMDegree >& fineSolution , DenseNodeData< Real , FEMDegree >& cumulativeConstraints ) const;
	// Updates the cumulative interpolation constraints @(depth-1) based on the change in solution coefficient @(depth)
	template< int FEMDegree , BoundaryType BType , bool HasGradients >
	void _updateCumulativeInterpolationConstraintsFromFiner( const InterpolationInfo< HasGradients >& interpolationInfo ,
		const BSplineData< FEMDegree , BType >& bsData , LocalDepth highDepth , const DenseNodeData< Real , FEMDegree >& fineSolution , DenseNodeData< Real , FEMDegree >& cumulativeConstraints ) const;

	template< int FEMDegree , BoundaryType BType >
	Real _coarserFunctionValue( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& upSampledCoefficients ) const;
	template< int FEMDegree , BoundaryType BType >
	Point3D< Real > _coarserFunctionGradient( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& upSampledCoefficients ) const;
	template< int FEMDegree , BoundaryType BType >
	Real   _finerFunctionValue( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& coefficients ) const;
	template< int FEMDegree , BoundaryType BType >
	Point3D< Real >   _finerFunctionGradient( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& coefficients ) const;
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	int _getSliceMatrixAndUpdateConstraints( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , SparseMatrix< Real >& matrix , DenseNodeData< Real , FEMDegree >& constraints , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& integrator , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& childIntegrator , const BSplineData< FEMDegree , BType >& bsData , LocalDepth depth , int slice , const DenseNodeData< Real , FEMDegree >& metSolution , bool coarseToFine );
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	int _getMatrixAndUpdateConstraints( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , SparseMatrix< Real >& matrix , DenseNodeData< Real , FEMDegree >& constraints , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& integrator , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& childIntegrator , const BSplineData< FEMDegree , BType >& bsData , LocalDepth depth , const DenseNodeData< Real , FEMDegree >& metSolution , bool coarseToFine );

	// Down samples constraints @(depth) to constraints @(depth-1)
	template< class C , int FEMDegree , BoundaryType BType > void _downSample( LocalDepth highDepth , DenseNodeData< C , FEMDegree >& constraints ) const;
	// Up samples coefficients @(depth-1) to coefficients @(depth)
	template< class C , int FEMDegree , BoundaryType BType > void _upSample( LocalDepth highDepth , DenseNodeData< C , FEMDegree >& coefficients ) const;
	template< class C , int FEMDegree , BoundaryType BType > static void _UpSample( LocalDepth highDepth , ConstPointer( C ) lowCoefficients , Pointer( C ) highCoefficients , int threads );
public:
	template< class C , int FEMDegree , BoundaryType BType > DenseNodeData< C , FEMDegree > coarseCoefficients( const  DenseNodeData< C , FEMDegree >& coefficients ) const;
	template< class C , int FEMDegree , BoundaryType BType > DenseNodeData< C , FEMDegree > coarseCoefficients( const SparseNodeData< C , FEMDegree >& coefficients ) const;
protected:

	/////////////////////////////////////////////
	// Code for splatting point-sample data    //
	// MultiGridOctreeData.WeightedSamples.inl //
	/////////////////////////////////////////////
	template< int WeightDegree >
	void _addWeightContribution( DensityEstimator< WeightDegree >& densityWeights , TreeOctNode* node , Point3D< Real > position , PointSupportKey< WeightDegree >& weightKey , Real weight=Real(1.0) );
	template< int WeightDegree , class PointSupportKey >
	Real _getSamplesPerNode( const DensityEstimator< WeightDegree >& densityWeights , const TreeOctNode* node , Point3D< Real > position , PointSupportKey& weightKey ) const;
	template< int WeightDegree , class PointSupportKey >
	void _getSampleDepthAndWeight( const DensityEstimator< WeightDegree >& densityWeights , const TreeOctNode* node , Point3D< Real > position , PointSupportKey& weightKey , Real& depth , Real& weight ) const;
	template< int WeightDegree , class PointSupportKey >
	void _getSampleDepthAndWeight( const DensityEstimator< WeightDegree >& densityWeights , Point3D< Real > position , PointSupportKey& weightKey , Real& depth , Real& weight ) const;
	template< bool CreateNodes ,                    int DataDegree , class V > void      _splatPointData( TreeOctNode* node ,                                           Point3D< Real > point , V v , SparseNodeData< V , DataDegree >& data ,                                              PointSupportKey< DataDegree >& dataKey                                                   );
	template< bool CreateNodes , int WeightDegree , int DataDegree , class V > Real      _splatPointData( const DensityEstimator< WeightDegree >& densityWeights , Point3D< Real > point , V v , SparseNodeData< V , DataDegree >& data , PointSupportKey< WeightDegree >& weightKey , PointSupportKey< DataDegree >& dataKey , LocalDepth minDepth , LocalDepth maxDepth , int dim=DIMENSION );
	template< bool CreateNodes , int WeightDegree , int DataDegree , class V > Real _multiSplatPointData( const DensityEstimator< WeightDegree >* densityWeights , TreeOctNode* node , Point3D< Real > point , V v , SparseNodeData< V , DataDegree >& data , PointSupportKey< WeightDegree >& weightKey , PointSupportKey< DataDegree >& dataKey , int dim=DIMENSION );
	template< class V , int DataDegree , BoundaryType BType , class Coefficients > V _evaluate( const Coefficients& coefficients , Point3D< Real > p , const BSplineData< DataDegree , BType >& bsData , const ConstPointSupportKey< DataDegree >& dataKey ) const;
public:
	template< class V , int DataDegree , BoundaryType BType > Pointer( V ) voxelEvaluate( const DenseNodeData< V , DataDegree >& coefficients , int& res , Real isoValue=0.f , LocalDepth depth=-1 , bool primal=false );

	template< int NormalDegree >
	struct HasNormalDataFunctor
	{
		const SparseNodeData< Point3D< Real > , NormalDegree >& normalInfo;
		HasNormalDataFunctor( const SparseNodeData< Point3D< Real > , NormalDegree >& ni ) : normalInfo( ni ){ ; }
		bool operator() ( const TreeOctNode* node ) const
		{
			const Point3D< Real >* n = normalInfo( node );
			if( n )
			{
				const Point3D< Real >& normal = *n;
				if( normal[0]!=0 || normal[1]!=0 || normal[2]!=0 ) return true;
			}
			if( node->children ) for( int c=0 ; c<Cube::CORNERS ; c++ ) if( (*this)( node->children + c ) ) return true;
			return false;
		}
	};
	struct TrivialHasDataFunctor{ bool operator() ( const TreeOctNode* node ) const{ return true; } };

	// [NOTE] The input/output for this method is pre-scaled by weight
	template< bool HasGradients > bool _setInterpolationInfoFromChildren( TreeOctNode* node , SparseNodeData< PointData< Real , HasGradients > , 0 >& iInfo ) const;
	template< bool HasGradients > SparseNodeData< PointData< Real , HasGradients > , 0 > _densifyInterpolationInfo( const std::vector< PointSample >& samples , Real pointValue , int adaptiveExponent ) const;

	template< int FEMDegree , BoundaryType BType > void _setValidityFlags( void );
	template< class HasDataFunctor > void _clipTree( const HasDataFunctor& f );

	template< int FEMDegree , BoundaryType BType > SparseNodeData<          Real   , 0 > leafValues   ( const DenseNodeData< Real , FEMDegree >& coefficients ) const;
	template< int FEMDegree , BoundaryType BType > SparseNodeData< Point3D< Real > , 0 > leafGradients( const DenseNodeData< Real , FEMDegree >& coefficients ) const;

	////////////////////////////////////
	// Evaluation Methods             //
	// MultiGridOctreeData.Evaluation //
	////////////////////////////////////
	static const int CHILDREN = Cube::CORNERS;
	template< int FEMDegree , BoundaryType BType >
	struct _Evaluator
	{
		typename BSplineEvaluationData< FEMDegree , BType >::Evaluator evaluator;
		typename BSplineEvaluationData< FEMDegree , BType >::ChildEvaluator childEvaluator;
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > cellStencil;
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > cellStencils  [CHILDREN];
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > edgeStencil             [Cube::EDGES  ];
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > edgeStencils  [CHILDREN][Cube::EDGES  ];
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > faceStencil             [Cube::FACES  ];
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > faceStencils  [CHILDREN][Cube::FACES  ];
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > cornerStencil           [Cube::CORNERS];
		Stencil< double , BSplineSupportSizes< FEMDegree >::SupportSize > cornerStencils[CHILDREN][Cube::CORNERS];

		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dCellStencil;
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dCellStencils  [CHILDREN];
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dEdgeStencil             [Cube::EDGES  ];
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dEdgeStencils  [CHILDREN][Cube::EDGES  ];
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dFaceStencil             [Cube::FACES  ];
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dFaceStencils  [CHILDREN][Cube::FACES  ];
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dCornerStencil           [Cube::CORNERS];
		Stencil< Point3D< double > , BSplineSupportSizes< FEMDegree >::SupportSize > dCornerStencils[CHILDREN][Cube::CORNERS];

		void set( LocalDepth depth );
		_Evaluator( void ){ _bsData = NULL; }
		~_Evaluator( void ){ if( _bsData ) delete _bsData , _bsData = NULL; }
	protected:
		BSplineData< FEMDegree , BType >* _bsData;
		friend Octree;
	};
	template< class V , int FEMDegree , BoundaryType BType >
	V _getCenterValue( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node ,                     const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const;
	template< class V , int FEMDegree , BoundaryType BType >
	V _getCornerValue( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int corner        , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const;
	template< class V , int FEMDegree , BoundaryType BType >
	V _getEdgeValue  ( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int edge          , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const;
	template< class V , int FEMDegree , BoundaryType BType >
	V _getValue      ( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , Point3D< Real > p , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator ) const;

	template< int FEMDegree , BoundaryType BType >
	std::pair< Real , Point3D< Real > > _getCenterValueAndGradient( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node ,                     const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const;
	template< int FEMDegree , BoundaryType BType >
	std::pair< Real , Point3D< Real > > _getCornerValueAndGradient( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int corner        , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const;
	template< int FEMDegree , BoundaryType BType >
	std::pair< Real , Point3D< Real > > _getEdgeValueAndGradient  ( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int edge          , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const;
	template< int FEMDegree , BoundaryType BType >
	std::pair< Real , Point3D< Real > > _getValueAndGradient      ( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , Point3D< Real > p , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator ) const;

public:
	template< int Degree , BoundaryType BType >
	class MultiThreadedEvaluator
	{
		const Octree* _tree;
		int _threads;
		std::vector< ConstPointSupportKey< Degree > > _neighborKeys;
		_Evaluator< Degree , BType > _evaluator;
		const DenseNodeData< Real , Degree >& _coefficients;
		DenseNodeData< Real , Degree > _coarseCoefficients;
	public:
		MultiThreadedEvaluator( const Octree* tree , const DenseNodeData< Real , Degree >& coefficients , int threads=1 );
		Real value( Point3D< Real > p , int thread=0 , const TreeOctNode* node=NULL );
		std::pair< Real , Point3D< Real > > valueAndGradient( Point3D< Real > , int thread=0 , const TreeOctNode* node=NULL );
	};

	////////////////////////////////////////
	// Iso-Surfacing Methods              //
	// MultiGridOctreeData.IsoSurface.inl //
	////////////////////////////////////////
protected:
	struct _IsoEdge
	{
		long long edges[2];
		_IsoEdge( void ){ edges[0] = edges[1] = 0; }
		_IsoEdge( long long v1 , long long v2 ){ edges[0] = v1 , edges[1] = v2; }
		long long& operator[]( int idx ){ return edges[idx]; }
		const long long& operator[]( int idx ) const { return edges[idx]; }
	};
	struct _FaceEdges
	{
		_IsoEdge edges[2];
		int count;
	};
	template< class Vertex >
	struct _SliceValues
	{
		typename SortedTreeNodes::SliceTableData sliceData;
		Pointer( Real ) cornerValues ; Pointer( Point3D< Real > ) cornerGradients ; Pointer( char ) cornerSet;
		Pointer( long long ) edgeKeys ; Pointer( char ) edgeSet;
		Pointer( _FaceEdges ) faceEdges ; Pointer( char ) faceSet;
		Pointer( char ) mcIndices;
		std::unordered_map< long long, std::vector< _IsoEdge > > faceEdgeMap;
		std::unordered_map< long long, std::pair< int, Vertex > > edgeVertexMap;
		std::unordered_map< long long, long long > vertexPairMap;

		_SliceValues( void );
		~_SliceValues( void );
		void reset( bool nonLinearFit );
	protected:
		int _oldCCount , _oldECount , _oldFCount , _oldNCount;
	};
	template< class Vertex >
	struct _XSliceValues
	{
		typename SortedTreeNodes::XSliceTableData xSliceData;
		Pointer( long long ) edgeKeys ; Pointer( char ) edgeSet;
		Pointer( _FaceEdges ) faceEdges ; Pointer( char ) faceSet;
		std::unordered_map< long long, std::vector< _IsoEdge > > faceEdgeMap;
		std::unordered_map< long long, std::pair< int, Vertex > > edgeVertexMap;
		std::unordered_map< long long, long long > vertexPairMap;

		_XSliceValues( void );
		~_XSliceValues( void );
		void reset( void );
	protected:
		int _oldECount , _oldFCount;
	};
	template< class Vertex >
	struct _SlabValues
	{
	protected:
		_XSliceValues< Vertex > _xSliceValues[2];
		_SliceValues< Vertex > _sliceValues[2];
	public:
		_SliceValues< Vertex >& sliceValues( int idx ){ return _sliceValues[idx&1]; }
		const _SliceValues< Vertex >& sliceValues( int idx ) const { return _sliceValues[idx&1]; }
		_XSliceValues< Vertex >& xSliceValues( int idx ){ return _xSliceValues[idx&1]; }
		const _XSliceValues< Vertex >& xSliceValues( int idx ) const { return _xSliceValues[idx&1]; }
	};
	template< class Vertex , int FEMDegree , BoundaryType BType >
	void _setSliceIsoCorners( const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , Real isoValue , LocalDepth depth , int slice ,         std::vector< _SlabValues< Vertex > >& sValues , const _Evaluator< FEMDegree , BType >& evaluator , int threads );
	template< class Vertex , int FEMDegree , BoundaryType BType >
	void _setSliceIsoCorners( const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , Real isoValue , LocalDepth depth , int slice , int z , std::vector< _SlabValues< Vertex > >& sValues , const _Evaluator< FEMDegree , BType >& evaluator , int threads );
	template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
	void _setSliceIsoVertices( Real isoValue , LocalDepth depth , int slice ,         int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< _SlabValues< Vertex > >& sValues , int threads );
	template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
	void _setSliceIsoVertices( Real isoValue , LocalDepth depth , int slice , int z , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< _SlabValues< Vertex > >& sValues , int threads );
	template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
	void _setXSliceIsoVertices( Real isoValue , LocalDepth depth , int slab , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< _SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void _setSliceIsoEdges( LocalDepth depth , int slice ,         std::vector< _SlabValues< Vertex > >& slabValues , int threads );
	template< class Vertex >
	void _setSliceIsoEdges( LocalDepth depth , int slice , int z , std::vector< _SlabValues< Vertex > >& slabValues , int threads );
	template< class Vertex >
	void _setXSliceIsoEdges( LocalDepth depth , int slice , std::vector< _SlabValues< Vertex > >& slabValues , int threads );
	template< class Vertex >
	void _copyFinerSliceIsoEdgeKeys( LocalDepth depth , int slice ,         std::vector< _SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void _copyFinerSliceIsoEdgeKeys( LocalDepth depth , int slice , int z , std::vector< _SlabValues< Vertex > >& sValues , int threads );
	template< class Vertex >
	void _copyFinerXSliceIsoEdgeKeys( LocalDepth depth , int slab , std::vector< _SlabValues< Vertex > >& sValues , int threads );

	template< class Vertex >
	void _setIsoSurface( LocalDepth depth , int offset , const _SliceValues< Vertex >& bValues , const _SliceValues< Vertex >& fValues , const _XSliceValues< Vertex >& xValues , CoredMeshData< Vertex >& mesh , bool polygonMesh , bool addBarycenter , int& vOffset , int threads );

	template< class Vertex >
	static int _addIsoPolygons( CoredMeshData< Vertex >& mesh , std::vector< std::pair< int , Vertex > >& polygon , bool polygonMesh , bool addBarycenter , int& vOffset );

	template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
	bool _getIsoVertex( Real isoValue , ConstPointSupportKey< WeightDegree >& weightKey , ConstPointSupportKey< ColorDegree >& colorKey , const TreeOctNode* node , int edgeIndex , int z , const _SliceValues< Vertex >& sValues , Vertex& vertex );
	template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
	bool _getIsoVertex( Real isoValue , ConstPointSupportKey< WeightDegree >& weightKey , ConstPointSupportKey< ColorDegree >& colorKey , const TreeOctNode* node , int cornerIndex , const _SliceValues< Vertex >& bValues , const _SliceValues< Vertex >& fValues , Vertex& vertex );

	void _init( TreeOctNode* node , LocalDepth maxDepth , bool (*Refine)( LocalDepth d , LocalOffset off ) );

public:
	int threads;

	Octree( void );

	void init( LocalDepth maxDepth , bool (*Refine)( LocalDepth d , LocalOffset off ) );
	template< class Data >
	int init( OrientedPointStream< Real >& pointStream , LocalDepth maxDepth , bool useConfidence , std::vector< PointSample >& samples , std::vector< ProjectiveData< Data , Real > >* sampleData );
	template< int DensityDegree >
	typename Octree::template DensityEstimator< DensityDegree >* setDensityEstimator( const std::vector< PointSample >& samples , LocalDepth splatDepth , Real samplesPerNode );
	template< int NormalDegree , int DensityDegree >
	SparseNodeData< Point3D< Real > , NormalDegree > setNormalField( const std::vector< PointSample >& samples , const DensityEstimator< DensityDegree >& density , Real& pointWeightSum , bool forceNeumann );
	template< int DataDegree , bool CreateNodes , int DensityDegree , class Data >
	SparseNodeData< ProjectiveData< Data , Real > , DataDegree > setDataField( const std::vector< PointSample >& samples , std::vector< ProjectiveData< Data , Real > >& sampleData , const DensityEstimator< DensityDegree >* density );
	template< int MaxDegree , int FEMDegree , BoundaryType FEMBType , class HasDataFunctor > void inalizeForBroodedMultigrid( LocalDepth fullDepth , const HasDataFunctor& F , std::vector< int >* map=NULL );

	// Generate an empty set of constraints
	template< int FEMDegree > DenseNodeData< Real , FEMDegree > initDenseNodeData( void );

	// Add finite-elements constraints (derived from a sparse scalar field)
	template< int FEMDegree , BoundaryType FEMBType , int SFDegree , BoundaryType SFBType , class FEMSFConstraintFunctor > void addFEMConstraints( const FEMSFConstraintFunctor& F , const SparseNodeData< Real , SFDegree >& sfCoefficients , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth )
	{ return _addFEMConstraints< FEMDegree , FEMBType , SFDegree , SFBType , FEMSFConstraintFunctor , const SparseNodeData< Real   , SFDegree > , Real , double >( F , sfCoefficients , constraints , maxDepth ); }
	// Add finite-elements constraints (derived from a dense scalar field)
	template< int FEMDegree , BoundaryType FEMBType , int SFDegree , BoundaryType SFBType , class FEMSFConstraintFunctor > void addFEMConstraints( const FEMSFConstraintFunctor& F , const  DenseNodeData< Real , SFDegree >& sfCoefficients , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth )
	{ return _addFEMConstraints< FEMDegree , FEMBType , SFDegree , SFBType , FEMSFConstraintFunctor , const  DenseNodeData< Real   , SFDegree > , Real , double >( F , sfCoefficients , constraints , maxDepth ); }
	// Add finite-elements constraints (derived from a sparse vector field)
	template< int FEMDegree , BoundaryType FEMBType , int VFDegree , BoundaryType VFBType , class FEMVFConstraintFunctor > void addFEMConstraints( const FEMVFConstraintFunctor& F , const SparseNodeData< Point3D< Real > , VFDegree >& vfCoefficients , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth )
	{ return _addFEMConstraints< FEMDegree , FEMBType , VFDegree , VFBType , FEMVFConstraintFunctor , const SparseNodeData< Point3D< Real > , VFDegree > , Point3D< Real > , Point3D< double > >( F , vfCoefficients , constraints , maxDepth ); }
	// Add finite-elements constraints (derived from a dense vector field)
	template< int FEMDegree , BoundaryType FEMBType , int VFDegree , BoundaryType VFBType , class FEMVFConstraintFunctor > void addFEMConstraints( const FEMVFConstraintFunctor& F , const  DenseNodeData< Point3D< Real > , VFDegree >& vfCoefficients , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth )
	{ return _addFEMConstraints< FEMDegree , FEMBType , VFDegree , VFBType , FEMVFConstraintFunctor , const  DenseNodeData< Point3D< Real > , VFDegree > , Point3D< Real > , Point3D< double > >( F , vfCoefficients , constraints , maxDepth ); }
	// Add interpolation constraints
	template< int FEMDegree , BoundaryType FEMBType , bool HasGradients > void addInterpolationConstraints( const InterpolationInfo< HasGradients >& interpolationInfo , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth );

	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor > double dot( const DotFunctor& F , const SparseNodeData< Real , Degree1 >& coefficients1 , const SparseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , false >( F , (const InterpolationInfo< false >*)NULL , coefficients1 , coefficients2 ); }
	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor > double dot( const DotFunctor& F , const SparseNodeData< Real , Degree1 >& coefficients1 , const DenseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , false >( F , (const InterpolationInfo< false >*)NULL , coefficients1 , coefficients2 ); }
	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor > double dot( const DotFunctor& F , const DenseNodeData< Real , Degree1 >& coefficients1 , const SparseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , false >( F , (const InterpolationInfo< false >*)NULL , coefficients1 , coefficients2 ); }
	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor > double dot( const DotFunctor& F , const DenseNodeData< Real , Degree1 >& coefficients1 , const DenseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , false >( F , (const InterpolationInfo< false >*)NULL , coefficients1 , coefficients2 ); }

	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor , bool HasGradients > double dot( const DotFunctor& F , const InterpolationInfo< HasGradients >* iInfo , const SparseNodeData< Real , Degree1 >& coefficients1 , const SparseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , HasGradients >( F , iInfo , coefficients1 , coefficients2 ); }
	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor , bool HasGradients > double dot( const DotFunctor& F , const InterpolationInfo< HasGradients >* iInfo , const SparseNodeData< Real , Degree1 >& coefficients1 , const DenseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , HasGradients >( F , iInfo , coefficients1 , coefficients2 ); }
	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor , bool HasGradients > double dot( const DotFunctor& F , const InterpolationInfo< HasGradients >* iInfo , const DenseNodeData< Real , Degree1 >& coefficients1 , const SparseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , HasGradients >( F , iInfo , coefficients1 , coefficients2 ); }
	template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 , class DotFunctor , bool HasGradients > double dot( const DotFunctor& F , const InterpolationInfo< HasGradients >* iInfo , const DenseNodeData< Real , Degree1 >& coefficients1 , const DenseNodeData< Real , Degree2 >& coefficients2 ) const
	{ return _dot< Degree1 , BType1 , Degree2 , BType2 , DotFunctor , HasGradients >( F , iInfo , coefficients1 , coefficients2 ); }

	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	void setSystemMatrix( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , LocalDepth depth , SparseMatrix< Real >& matrix ) const;

	// Solve the linear system
	struct SolverInfo
	{
		// How to solve
		LocalDepth cgDepth;
		int iters;
		double cgAccuracy , lowResIterMultiplier;

		SolverInfo( void ) : cgDepth(0) , iters(1), cgAccuracy(0) , lowResIterMultiplier(0) { ; }
	};
	template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
	DenseNodeData< Real , FEMDegree > solveSystem( const FEMSystemFunctor& F , InterpolationInfo< HasGradients >* iData , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxSolveDepth , const SolverInfo& solverInfo );

	template< int FEMDegree , BoundaryType BType , int WeightDegree , int ColorDegree , class Vertex >
	void getMCIsoSurface( const DenseNodeData< Real , FEMDegree >& solution , Real isoValue , CoredMeshData< Vertex >& mesh , bool nonLinearFit=true , bool addBarycenter=false , bool polygonMesh=false );


	const TreeOctNode& tree( void ) const{ return *_tree; }
	size_t leaves( void ) const { return _tree->leaves(); }
	size_t nodes( void ) const { int count = 0 ; for( const TreeOctNode* n=_tree->nextNode() ; n ; n=_tree->nextNode( n ) ) if( IsActiveNode( n ) ) count++ ; return count; }
	size_t ghostNodes( void ) const { int count = 0 ; for( const TreeOctNode* n=_tree->nextNode() ; n ; n=_tree->nextNode( n ) ) if( !IsActiveNode( n ) ) count++ ; return count; }
	inline size_t validSpaceNodes( void ) const { int count = 0 ; for( const TreeOctNode* n=_tree->nextNode() ; n ; n=_tree->nextNode( n ) ) if( isValidSpaceNode( n ) ) count++ ;  return count; }
	inline size_t validSpaceNodes( LocalDepth d ) const { int count = 0 ; for( const TreeOctNode* n=_tree->nextNode() ; n ; n=_tree->nextNode( n ) ) if( _localDepth(n)==d && isValidSpaceNode( n ) ) count++ ; return count; }
	template< int Degree , BoundaryType BType > size_t validFEMNodes( void ) const { int count = 0 ; for( const TreeOctNode* n=_tree->nextNode() ; n ; n=_tree->nextNode( n ) ) if( isValidFEMNode< Degree , BType >( n ) ) count++ ;  return count; }
	template< int Degree , BoundaryType BType > size_t validFEMNodes( LocalDepth d ) const { int count = 0 ; for( const TreeOctNode* n=_tree->nextNode() ; n ; n=_tree->nextNode( n ) ) if( _localDepth(n)==d && isValidFEMNode< Degree , BType >( n ) ) count++ ; return count; }
	LocalDepth depth( void ) const { return _localMaxDepth( _tree ); }
	void resetNodeIndices( void ){ _NodeCount = 0 ; for( TreeOctNode* node=_tree->nextNode() ; node ; node=_tree->nextNode( node ) ) _NodeInitializer( *node ) , node->nodeData.flags=0; }

protected:
	template< class D > static bool _IsZero( const D& d );
	template< class D > static Real _Dot( const D& d1 , const D& d2 );
	template< int FEMDegree , BoundaryType FEMBType , int CDegree , BoundaryType CBType , class FEMConstraintFunctor , class Coefficients , class D , class _D >
	void _addFEMConstraints( const FEMConstraintFunctor& F , const Coefficients& coefficients , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth );
	template< int FEMDegree1 , BoundaryType FEMBType1 , int FEMDegree2 , BoundaryType FEMBType2 , class DotFunctor , bool HasGradients , class Coefficients1 , class Coefficients2 >
	double _dot( const DotFunctor& F , const InterpolationInfo< HasGradients >* iInfo , const Coefficients1& coefficients1 , const Coefficients2& coefficients2 ) const;
};
template< class Real > int Octree< Real >::_NodeCount = 0;


template< class Real > void Reset( void ){ Octree< Real >::ResetNodeCount(); }


#define MEMORY_ALLOCATOR_BLOCK_SIZE 0

const double EPSILON              = 1e-6;

//////////////////
// TreeNodeData //
//////////////////
TreeNodeData::TreeNodeData( void ){ flags = 0; }
TreeNodeData::~TreeNodeData( void ) { }


////////////
// Octree //
////////////
template< class Real > Octree< Real >::Octree( void ) : threads(1)
{
	_tree = TreeOctNode::NewBrood( _NodeInitializer );
	_tree->initChildren( _NodeInitializer ) , _spaceRoot = _tree->children;
	_depthOffset = 1;
}

template< class Real >
template< int FEMDegree , BoundaryType BType >
void Octree< Real >::functionIndex( const TreeOctNode* node , int idx[3] ) const
{
	LocalDepth d ; LocalOffset off;
	_localDepthAndOffset( node , d , off );
	for( int dd=0 ; dd<DIMENSION ; dd++ ) idx[dd] = BSplineData< FEMDegree , BType >::FunctionIndex( d , off[dd] );
}

template< class Real >
OctNode< TreeNodeData >* Octree< Real >::leaf( Point3D< Real > p )
{
	if( !_InBounds( p ) ) return NULL;
	Point3D< Real > center = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
	Real width = Real(1.0);
	TreeOctNode* node = _spaceRoot;
	while( node->children )
	{
		int cIndex = TreeOctNode::CornerIndex( center , p );
		node = node->children + cIndex;
		width /= 2;
		if( cIndex&1 ) center[0] += width/2;
		else           center[0] -= width/2;
		if( cIndex&2 ) center[1] += width/2;
		else           center[1] -= width/2;
		if( cIndex&4 ) center[2] += width/2;
		else           center[2] -= width/2;
	}
	return node;
}
template< class Real >
const OctNode< TreeNodeData >* Octree< Real >::leaf( Point3D< Real > p ) const
{
	if( !_InBounds( p ) ) return NULL;
	Point3D< Real > center = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
	Real width = Real(1.0);
	TreeOctNode* node = _spaceRoot;
	while( node->children )
	{
		int cIndex = TreeOctNode::CornerIndex( center , p );
		node = node->children + cIndex;
		width /= 2;
		if( cIndex&1 ) center[0] += width/2;
		else           center[0] -= width/2;
		if( cIndex&2 ) center[1] += width/2;
		else           center[1] -= width/2;
		if( cIndex&4 ) center[2] += width/2;
		else           center[2] -= width/2;
	}
	return node;
}
template< class Real > bool Octree< Real >::_InBounds( Point3D< Real > p ){ return p[0]>=Real(0.) && p[0]<=Real(1.0) && p[1]>=Real(0.) && p[1]<=Real(1.0) && p[2]>=Real(0.) && p[2]<=Real(1.0); }
template< class Real >
template< int FEMDegree , BoundaryType BType >
bool Octree< Real >::isValidFEMNode( const TreeOctNode* node ) const
{
	if( GetGhostFlag( node ) ) return false;
	LocalDepth d ; LocalOffset off;
	_localDepthAndOffset( node , d , off );
	if( d<0 ) return false;
	return !BSplineEvaluationData< FEMDegree , BType >::OutOfBounds( d , off[0] ) && !BSplineEvaluationData< FEMDegree , BType >::OutOfBounds( d , off[1] ) && !BSplineEvaluationData< FEMDegree , BType >::OutOfBounds( d , off[2] );
}
template< class Real >
bool Octree< Real >::isValidSpaceNode( const TreeOctNode* node ) const
{
	if( !node ) return false;
	LocalDepth d ; LocalOffset off;
	_localDepthAndOffset( node , d , off );
	if( d<0 ) return false;
	int res = 1<<d;
	return off[0]>=0 && off[0]<res && off[1]>=0 && off[1]<res && off[2]>=0 && off[2]<res;
}
template< class Real >
template< int Degree , BoundaryType BType >
void Octree< Real >::_setFullDepth( TreeOctNode* node , LocalDepth depth ) const
{
	bool refine = false;
	LocalDepth d ; LocalOffset off;
	_localDepthAndOffset( node , d , off );
	if( d<depth )
	{
		if( d<0 )
			refine = true;
		else if( BType==BOUNDARY_FREE && !_outOfBounds< Degree , BType >( node ) ) refine = true;
		else if( !BSplineSupportSizes< Degree >::OutOfBounds( d , off[0] ) && !BSplineSupportSizes< Degree >::OutOfBounds( d , off[1] ) && !BSplineSupportSizes< Degree >::OutOfBounds( d , off[2] ) ) refine = true;
	}
	if( refine )
	{
		if( !node->children ) node->initChildren( _NodeInitializer );
		for( int c=0 ; c<Cube::CORNERS ; c++ ) _setFullDepth< Degree , BType >( node->children+c , depth );
	}
}
template< class Real >
template< int Degree , BoundaryType BType >
void Octree< Real >::_setFullDepth( LocalDepth depth )
{
	if( !_tree->children ) _tree->initChildren( _NodeInitializer );
	for( int c=0 ; c<Cube::CORNERS ; c++ ) _setFullDepth< Degree , BType >( _tree->children+c , depth );
}

template< class Real , bool HasGradients >
struct _PointDataAccumulator_
{
#if POINT_DATA_RES
	static inline void _AddToPointData_( PointData< Real , HasGradients >& pData , Point3D< Real > position , Real value , Point3D< Real > gradient , Point3D< Real > center , Real width , Real weight );
#else // !POINT_DATA_RES
	static inline void _AddToPointData_( PointData< Real , HasGradients >& pData , Point3D< Real > position , Real value , Point3D< Real > gradient , Real weight );
#endif // POINT_DATA_RES
};
template< class Real >
struct _PointDataAccumulator_< Real , false >
{
#if POINT_DATA_RES
	static inline void _AddToPointData_( PointData< Real , false >& pData , Point3D< Real > position , Real value , Point3D< Real > gradient , Point3D< Real > center , Real width , Real weight ){ pData.addPoint( SinglePointData< Real , false >( position , value , weight ) , center , width ); }
#else // !POINT_DATA_RES
	static inline void _AddToPointData_( PointData< Real , false >& pData , Point3D< Real > position , Real value , Point3D< Real > gradient , Real weight ){ pData.position += position , pData.value += value , pData.weight += weight; }
#endif // POINT_DATA_RES
};
template< class Real >
struct _PointDataAccumulator_< Real , true >
{
#if POINT_DATA_RES
	static inline void _AddToPointData_( PointData< Real , true >& pData , Point3D< Real > position , Real value , Point3D< Real > gradient , Point3D< Real > center , Real width , Real weight ){ pData.addPoint( SinglePointData< Real , true >( position , value , gradient , weight ) , center , width ); }
#else // !POINT_DATA_RES
	static inline void _AddToPointData_( PointData< Real , true >& pData , Point3D< Real > position , Real value , Point3D< Real > gradient , Real weight ){ pData.position += position , pData.value += value , pData.gradient += gradient , pData.weight += weight; }
#endif // POINT_DATA_RES
};

template< class Real >
void Octree< Real >::_init( TreeOctNode* node , LocalDepth maxDepth , bool (*Refine)( LocalDepth , LocalOffset ) )
{
	if( _localDepth( node )<maxDepth )
	{
		LocalDepth d ; LocalOffset off;
		_localDepthAndOffset( node , d , off );
		if( Refine( d , off ) )
		{
			node->initChildren( _NodeInitializer );
			for( int c=0 ; c<Cube::CORNERS ; c++ ) _init( node->children + c , maxDepth , Refine );
		}
	}
}
template< class Real > void Octree< Real >::init( LocalDepth maxDepth , bool (*Refine)( LocalDepth , LocalOffset ) ){ _init( _spaceRoot , maxDepth , Refine ); }
template< class Real >
template< class Data >
int Octree< Real >::init( OrientedPointStream< Real >& pointStream , LocalDepth maxDepth , bool useConfidence , std::vector< PointSample >& samples , std::vector< ProjectiveData< Data , Real > >* sampleData )
{
	OrientedPointStreamWithData< Real , Data >& pointStreamWithData = ( OrientedPointStreamWithData< Real , Data >& )pointStream;

	// Add the point data
	int outOfBoundPoints = 0 , zeroLengthNormals = 0 , undefinedNormals = 0 , pointCount = 0;
	{
		std::vector< int > nodeToIndexMap;
		Point3D< Real > p , n;
		OrientedPoint3D< Real > _p;
		Data _d;
		while( ( sampleData ? pointStreamWithData.nextPoint( _p , _d ) : pointStream.nextPoint( _p ) ) )
		{
			p = Point3D< Real >(_p.p) , n = Point3D< Real >(_p.n);
			Real len = (Real)Length( n );
			if( !_InBounds(p) ){ outOfBoundPoints++ ; continue; }
			if( !len ){ zeroLengthNormals++ ; continue; }
			if( len!=len ){ undefinedNormals++ ; continue; }
			n /= len;
			Point3D< Real > center = Point3D< Real >( Real(0.5) , Real(0.5) , Real(0.5) );
			Real width = Real(1.0);
			TreeOctNode* temp = _spaceRoot;
			LocalDepth depth = _localDepth( temp );
			while( depth<maxDepth )
			{
				if( !temp->children ) temp->initChildren( _NodeInitializer );
				int cIndex = TreeOctNode::CornerIndex( center , p );
				temp = temp->children + cIndex;
				width /= 2;
				if( cIndex&1 ) center[0] += width/2;
				else           center[0] -= width/2;
				if( cIndex&2 ) center[1] += width/2;
				else           center[1] -= width/2;
				if( cIndex&4 ) center[2] += width/2;
				else           center[2] -= width/2;
				depth++;
			}
			Real weight = (Real)( useConfidence ? len : 1. );
			int nodeIndex = temp->nodeData.nodeIndex;
			if( nodeIndex>=nodeToIndexMap.size() ) nodeToIndexMap.resize( nodeIndex+1 , -1 );
			int idx = nodeToIndexMap[ nodeIndex ];
			if( idx==-1 )
			{
				idx = (int)samples.size();
				nodeToIndexMap[ nodeIndex ] = idx;
				samples.resize( idx+1 ) , samples[idx].node = temp;
				if( sampleData ) sampleData->resize( idx+1 );
			}
			samples[idx].sample += ProjectiveData< OrientedPoint3D< Real > , Real >( OrientedPoint3D< Real >( p * weight , n * weight ) , weight );
			if( sampleData ) (*sampleData)[ idx ] += ProjectiveData< Data , Real >( _d * weight , weight );
			pointCount++;
		}
		pointStream.reset();
	}
	if( outOfBoundPoints  ) fprintf( stderr , "[WARNING] Found out-of-bound points: %d\n" , outOfBoundPoints );
	if( zeroLengthNormals ) fprintf( stderr , "[WARNING] Found zero-length normals: %d\n" , zeroLengthNormals );
	if( undefinedNormals  ) fprintf( stderr , "[WARNING] Found undefined normals: %d\n" , undefinedNormals );

	return pointCount;
}
template< class Real >
template< int DensityDegree >
typename Octree< Real >::template DensityEstimator< DensityDegree >* Octree< Real >::setDensityEstimator( const std::vector< PointSample >& samples , LocalDepth splatDepth , Real samplesPerNode )
{
	LocalDepth maxDepth = _localMaxDepth( _tree );
	splatDepth = std::max< LocalDepth >( 0 , std::min< LocalDepth >( splatDepth , maxDepth ) );
	DensityEstimator< DensityDegree >* _density = new DensityEstimator< DensityDegree >( splatDepth );
	DensityEstimator< DensityDegree >& density = *_density;
	PointSupportKey< DensityDegree > densityKey;
	densityKey.set( _localToGlobal( splatDepth ) );

	std::vector< int > sampleMap( NodeCount() , -1 );
#pragma omp parallel for num_threads( threads )
	for( int i=0 ; i<samples.size() ; i++ ) if( samples[i].sample.weight>0 ) sampleMap[ samples[i].node->nodeData.nodeIndex ] = i;
	std::function< ProjectiveData< OrientedPoint3D< Real > , Real > ( TreeOctNode* ) > SetDensity = [&] ( TreeOctNode* node )
	{
		ProjectiveData< OrientedPoint3D< Real > , Real > sample;
		LocalDepth d = _localDepth( node );
		int idx = node->nodeData.nodeIndex;
		if( node->children )
			for( int c=0 ; c<Cube::CORNERS ; c++ )
			{
				ProjectiveData< OrientedPoint3D< Real > , Real > s = SetDensity( node->children + c );
				if( d<=splatDepth && s.weight>0 )
				{
					Point3D< Real > p = s.data.p / s.weight;
					Real w = s.weight / samplesPerNode;
					_addWeightContribution( density , node , p , densityKey , w );
				}
				sample += s;
			}
		else if( idx<sampleMap.size() && sampleMap[idx]!=-1 )
		{
			sample = samples[ sampleMap[ idx ] ].sample;
			if( d<=splatDepth && sample.weight>0 )
			{
				Point3D< Real > p = sample.data.p / sample.weight;
				Real w = sample.weight / samplesPerNode;
				_addWeightContribution( density , node , p , densityKey , w );
			}
		}
		return sample;
	};
	SetDensity( _spaceRoot );

	return _density;
}
template< class Real >
template< int NormalDegree , int DensityDegree >
SparseNodeData< Point3D< Real > , NormalDegree > Octree< Real >::setNormalField( const std::vector< PointSample >& samples , const DensityEstimator< DensityDegree >& density , Real& pointWeightSum , bool forceNeumann )
{
	LocalDepth maxDepth = _localMaxDepth( _tree );
	PointSupportKey< DensityDegree > densityKey;
	PointSupportKey< NormalDegree > normalKey;
	densityKey.set( _localToGlobal( maxDepth ) ) , normalKey.set( _localToGlobal( maxDepth ) );

	Real weightSum = 0;
	pointWeightSum = 0;
	SparseNodeData< Point3D< Real > , NormalDegree > normalField;
	for( int i=0 ; i<samples.size() ; i++ )
	{
		const ProjectiveData< OrientedPoint3D< Real > , Real >& sample = samples[i].sample;
		if( sample.weight>0 )
		{
			Point3D< Real > p = sample.data.p / sample.weight , n = sample.data.n;
			weightSum += sample.weight;
			if( !_InBounds(p) ){ fprintf( stderr , "[WARNING] Octree:setNormalField: Point sample is out of bounds\n" ) ; continue; }
			pointWeightSum += _splatPointData< true >( density , p , n , normalField , densityKey , normalKey , 0 , maxDepth , 3 );
		}
	}
	pointWeightSum /= weightSum;

	return normalField;
}
template< class Real >
template< int DataDegree , bool CreateNodes , int DensityDegree , class Data >
SparseNodeData< ProjectiveData< Data , Real > , DataDegree > Octree< Real >::setDataField( const std::vector< PointSample >& samples , std::vector< ProjectiveData< Data , Real > >& sampleData , const DensityEstimator< DensityDegree >* density )
{
	LocalDepth maxDepth = _localMaxDepth( _tree );
	PointSupportKey< DensityDegree > densityKey;
	PointSupportKey< DataDegree > dataKey;
	densityKey.set( _localToGlobal( maxDepth ) ) , dataKey.set( _localToGlobal( maxDepth ) );

	SparseNodeData< ProjectiveData< Data , Real > , DataDegree > dataField;
	for( int i=0 ; i<samples.size() ; i++ )
	{
		const ProjectiveData< OrientedPoint3D< Real > , Real >& sample = samples[i].sample;
		const ProjectiveData< Data , Real >& data = sampleData[i];
		Point3D< Real > p = sample.weight==0 ? sample.data.p : sample.data.p / sample.weight;
		if( !_InBounds(p) ){ fprintf( stderr , "[WARNING] Point is out of bounds: %f %f %f <- %f %f %f [%f]\n" , p[0] , p[1] , p[2] , sample.data.p[0] , sample.data.p[1] , sample.data.p[2] , sample.weight ) ; continue; }
		_multiSplatPointData< CreateNodes >( density , (TreeOctNode*)samples[i].node , p , data , dataField , densityKey , dataKey , 2 );
	}
	return dataField;
}
template< class Real >
template< int MaxDegree , int FEMDegree , BoundaryType FEMBType , class HasDataFunctor >
void Octree< Real >::inalizeForBroodedMultigrid( LocalDepth fullDepth , const HasDataFunctor& F , std::vector< int >* map )
{
	if( FEMDegree>MaxDegree ) fprintf( stderr , "[ERROR] MaxDegree must be at least as large as the FEM degree: %d <= %d\n" , FEMDegree , MaxDegree );
	while( _localInset( 0 ) + BSplineEvaluationData< MaxDegree , BOUNDARY_FREE >::Begin( 0 )<0 || _localInset( 0 ) + BSplineEvaluationData< MaxDegree , BOUNDARY_FREE >::End( 0 )>(1<<_depthOffset) )
	{
		TreeOctNode* newSpaceRootParent = TreeOctNode::NewBrood( _NodeInitializer );
		TreeOctNode* oldSpaceRootParent = _spaceRoot->parent;
		int corner = _depthOffset<=1 ? Cube::CORNERS-1 : 0;
		newSpaceRootParent[corner].children = _spaceRoot;
		oldSpaceRootParent->children = newSpaceRootParent;
		for( int c=0 ; c<Cube::CORNERS ; c++ ) _spaceRoot[c].parent = newSpaceRootParent + corner , newSpaceRootParent[c].parent = oldSpaceRootParent;
		_depthOffset++;
	}
	int d=0 , off[] = { 0 , 0 , 0 };
	TreeOctNode::ResetDepthAndOffset( _tree , d , off );
	_maxDepth = _localMaxDepth( _tree );

	// Make the low-resolution part of the tree be complete
	_fullDepth = std::max< LocalDepth >( 0 , std::min< LocalDepth >( _maxDepth , fullDepth ) );
	_setFullDepth< MaxDegree , BOUNDARY_FREE >( _fullDepth );
	// Clear all the flags and make everything that is not low-res a ghost node
	for( TreeOctNode* node=_tree->nextNode() ; node ; node=_tree->nextNode( node ) ) node->nodeData.flags = 0 , SetGhostFlag( node , _localDepth( node )>_fullDepth );

	// Set the ghost nodes for the high-res part of the tree
	_clipTree( F );

	const int OverlapRadius = -BSplineOverlapSizes< MaxDegree , MaxDegree >::OverlapStart;
	typename TreeOctNode::NeighborKey< OverlapRadius , OverlapRadius > neighborKey;
	neighborKey.set( _localToGlobal( _maxDepth-1 ) );

	for( LocalDepth d=_maxDepth-1 ; d>=0 ; d-- )
		for( TreeOctNode* node=_tree->nextNode() ; node ; node=_tree->nextNode( node ) ) if( _localDepth( node )==d && IsActiveNode( node->children ) )
			{
				neighborKey.template getNeighbors< true >( node , _NodeInitializer );
				for( int i=0 ; i<neighborKey.Width ; i++ ) for( int j=0 ; j<neighborKey.Width ; j++ ) for( int k=0 ; k<neighborKey.Width ; k++ ) SetGhostFlag( neighborKey.neighbors[ _localToGlobal(d) ].neighbors[i][j][k] , false );
			}

	_sNodes.set( *_tree , map );
	_setValidityFlags< FEMDegree , FEMBType >();
	for( TreeOctNode* node=_tree->nextNode() ; node ; node=_tree->nextNode( node ) ) if( !IsActiveNode( node ) ) node->nodeData.nodeIndex = -1;
}


template< class Real >
template< int FEMDegree , BoundaryType BType >
void Octree< Real >::_setValidityFlags( void )
{
	for( int i=0 ; i<_sNodes.size() ; i++ )
	{
		const unsigned char MASK = ~( TreeNodeData::SPACE_FLAG | TreeNodeData::FEM_FLAG );
		_sNodes.treeNodes[i]->nodeData.flags &= MASK;
		if( isValidSpaceNode( _sNodes.treeNodes[i] ) ) _sNodes.treeNodes[i]->nodeData.flags |= TreeNodeData::SPACE_FLAG;
		if( isValidFEMNode< FEMDegree , BType >( _sNodes.treeNodes[i] ) ) _sNodes.treeNodes[i]->nodeData.flags |= TreeNodeData::FEM_FLAG;
	}
}

// Trim off the branches of the tree (finer than _fullDepth) that don't contain data
template< class Real >
template< class HasDataFunctor >
void Octree< Real >::_clipTree( const HasDataFunctor& f )
{
	// Because we are doing things in a brooded fashion, if any of the children has data then the whole brood is active
	for( TreeOctNode* temp=_tree->nextNode() ; temp ; temp=_tree->nextNode(temp) ) if( temp->children && _localDepth( temp )>=_fullDepth )
		{
			bool hasData = false;
			for( int c=0 ; c<Cube::CORNERS && !hasData ; c++ ) hasData |= f( temp->children + c );
			for( int c=0 ; c<Cube::CORNERS ; c++ ) SetGhostFlag( temp->children+c , !hasData );
		}
}

template< class Real >
template< bool HasGradients >
bool Octree< Real >::_setInterpolationInfoFromChildren( TreeOctNode* node , SparseNodeData< PointData< Real , HasGradients > , 0 >& interpolationInfo ) const
{
	if( IsActiveNode( node->children ) )
	{
		bool hasChildData = false;
		PointData< Real , HasGradients > pData;
#if POINT_DATA_RES
		Point3D< Real > center;
		Real width;
		_centerAndWidth( node , center , width );
		for( int c=0 ; c<Cube::CORNERS ; c++ )
			if( _setInterpolationInfoFromChildren( node->children + c , interpolationInfo ) )
			{
				const PointData< Real , HasGradients >& _pData = interpolationInfo[ node->children + c ];
				for( int cc=0 ; cc<PointData< Real , HasGradients >::SAMPLES ; cc++ )
				{
					int x[3];
					PointData< Real , HasGradients >::SetIndices( _pData[cc].position / _pData[cc].weight , center , width , x );
					pData[ x[0] + x[1]*PointData< Real , HasGradients >::RES + x[2]*PointData< Real , HasGradients >::RES*PointData< Real , HasGradients >::RES ] += _pData[cc];
				}
				hasChildData = true;
			}
#else // !POINT_DATA_RES
		for( int c=0 ; c<Cube::CORNERS ; c++ )
			if( _setInterpolationInfoFromChildren( node->children + c , interpolationInfo ) )
			{
				pData += interpolationInfo[ node->children + c ];
				hasChildData = true;
			}
#endif // POINT_DATA_RES
		if( hasChildData && IsActiveNode( node ) ) interpolationInfo[ node ] += pData;
		return hasChildData;
	}
	else return interpolationInfo( node )!=NULL;
}
template< class Real >
template< bool HasGradients >
SparseNodeData< PointData< Real , HasGradients > , 0 > Octree< Real >::_densifyInterpolationInfo( const std::vector< PointSample >& samples , Real pointValue , int adaptiveExponent ) const
{
	SparseNodeData< PointData< Real , HasGradients > , 0 > iInfo;
	for( int i=0 ; i<samples.size() ; i++ )
	{
		const TreeOctNode* node = samples[i].node;
		const ProjectiveData< OrientedPoint3D< Real > , Real >& pData = samples[i].sample;
		while( !IsActiveNode( node ) ) node = node->parent;
		if( pData.weight )
		{
#if POINT_DATA_RES
			Point3D< Real > center;
			Real width;
			_centerAndWidth( node , center , width );
			_PointDataAccumulator_< Real , HasGradients >::_AddToPointData_( iInfo[node] , pData.data.p , pointValue * pData.weight , pData.data.n , center , width , pData.weight );
#else // !POINT_DATA_RES
			_PointDataAccumulator_< Real , HasGradients >::_AddToPointData_( iInfo[node] , pData.data.p , pointValue * pData.weight , pData.data.n , pData.weight );
#endif // POINT_DATA_RES
		}
	}

	// Set the interior values
	_setInterpolationInfoFromChildren( _spaceRoot, iInfo );
#pragma omp parallel for
	for( int i=0 ; i<(int)iInfo.size() ; i++ )
#if POINT_DATA_RES
		for( int c=0 ; c<PointData< Real , HasGradients >::SAMPLES ; c++ )
		{
			Real w = iInfo[i][c].weight;
			iInfo[i][c] /= w ; iInfo[i][c].weight = w;
		}
#else // !POINT_DATA_RES
	{
		Real w = iInfo[i].weight;
		iInfo[i] /= w ; iInfo[i].weight = w;
	}
#endif // POINT_DATA_RES
	LocalDepth maxDepth = _localMaxDepth( _tree );

	// Set the average position and scale the weights
	for( const TreeOctNode* node=_tree->nextNode() ; node ; node=_tree->nextNode(node) ) if( IsActiveNode( node ) )
		{
			PointData< Real , HasGradients >* pData = iInfo( node );
			if( pData )
			{
				int e = _localDepth( node ) * adaptiveExponent - ( maxDepth ) * (adaptiveExponent-1);
#if POINT_DATA_RES
				for( int c=0 ; c<PointData< Real , HasGradients >::SAMPLES ; c++ ) if( (*pData)[c].weight )
			{
				if( e<0 ) (*pData)[c].weight /= Real( 1<<(-e) );
				else      (*pData)[c].weight *= Real( 1<<  e  );
			}
#else // !POINT_DATA_RES
				if( e<0 ) pData->weight /= Real( 1<<(-e) );
				else      pData->weight *= Real( 1<<  e  );
#endif // POINT_DATA_RES
			}
		}
	return iInfo;
}
////////////////
// VertexData //
////////////////
long long VertexData::CenterIndex( const TreeOctNode* node , int maxDepth )
{
	int idx[DIMENSION];
	return CenterIndex(node,maxDepth,idx);
}
long long VertexData::CenterIndex(const TreeOctNode* node,int maxDepth,int idx[DIMENSION])
{
	int d , o[3];
	node->depthAndOffset( d , o );
	for( int i=0 ; i<DIMENSION ; i++ ) idx[i] = BinaryNode::CornerIndex( maxDepth+1 , d+1 , o[i]<<1 , 1 );
	return (long long)(idx[0]) | (long long)(idx[1])<<VERTEX_COORDINATE_SHIFT | (long long)(idx[2])<<(2*VERTEX_COORDINATE_SHIFT);
}
long long VertexData::CenterIndex( int depth , const int offSet[DIMENSION] , int maxDepth , int idx[DIMENSION] )
{
	for(int i=0;i<DIMENSION;i++) idx[i]=BinaryNode::CornerIndex( maxDepth+1 , depth+1 , offSet[i]<<1 , 1 );
	return (long long)(idx[0]) | (long long)(idx[1])<<VERTEX_COORDINATE_SHIFT | (long long)(idx[2])<<(2*VERTEX_COORDINATE_SHIFT);
}
long long VertexData::CornerIndex(const TreeOctNode* node,int cIndex,int maxDepth)
{
	int idx[DIMENSION];
	return CornerIndex(node,cIndex,maxDepth,idx);
}
long long VertexData::CornerIndex( const TreeOctNode* node , int cIndex , int maxDepth , int idx[DIMENSION] )
{
	int x[DIMENSION];
	Cube::FactorCornerIndex( cIndex , x[0] , x[1] , x[2] );
	int d , o[3];
	node->depthAndOffset( d , o );
	for( int i=0 ; i<DIMENSION ; i++ ) idx[i] = BinaryNode::CornerIndex( maxDepth+1 , d , o[i] , x[i] );
	return CornerIndexKey( idx );
}
long long VertexData::CornerIndex( int depth , const int offSet[DIMENSION] , int cIndex , int maxDepth , int idx[DIMENSION] )
{
	int x[DIMENSION];
	Cube::FactorCornerIndex( cIndex , x[0] , x[1] , x[2] );
	for( int i=0 ; i<DIMENSION ; i++ ) idx[i] = BinaryNode::CornerIndex( maxDepth+1 , depth , offSet[i] , x[i] );
	return CornerIndexKey( idx );
}
long long VertexData::CornerIndexKey( const int idx[DIMENSION] )
{
	return (long long)(idx[0]) | (long long)(idx[1])<<VERTEX_COORDINATE_SHIFT | (long long)(idx[2])<<(2*VERTEX_COORDINATE_SHIFT);
}
long long VertexData::FaceIndex(const TreeOctNode* node,int fIndex,int maxDepth){
	int idx[DIMENSION];
	return FaceIndex(node,fIndex,maxDepth,idx);
}
long long VertexData::FaceIndex(const TreeOctNode* node,int fIndex,int maxDepth,int idx[DIMENSION])
{
	int dir,offset;
	Cube::FactorFaceIndex(fIndex,dir,offset);
	int d,o[3];
	node->depthAndOffset(d,o);
	for(int i=0;i<DIMENSION;i++){idx[i]=BinaryNode::CornerIndex(maxDepth+1,d+1,o[i]<<1,1);}
	idx[dir]=BinaryNode::CornerIndex(maxDepth+1,d,o[dir],offset);
	return (long long)(idx[0]) | (long long)(idx[1])<<VERTEX_COORDINATE_SHIFT | (long long)(idx[2])<<(2*VERTEX_COORDINATE_SHIFT);
}
long long VertexData::EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth ){ int idx[DIMENSION] ; return EdgeIndex( node , eIndex , maxDepth , idx ); }
long long VertexData::EdgeIndex( const TreeOctNode* node , int eIndex , int maxDepth , int idx[DIMENSION] )
{
	int o , i1 , i2;
	int d , off[3];
	node->depthAndOffset( d ,off );
	Cube::FactorEdgeIndex( eIndex , o , i1 , i2 );
	for( int i=0 ; i<DIMENSION ; i++ ) idx[i] = BinaryNode::CornerIndex( maxDepth+1 , d+1 , off[i]<<1 , 1 );
	switch(o)
	{
		case 0:
			idx[1] = BinaryNode::CornerIndex( maxDepth+1 , d , off[1] , i1 );
			idx[2] = BinaryNode::CornerIndex( maxDepth+1 , d , off[2] , i2 );
			break;
		case 1:
			idx[0] = BinaryNode::CornerIndex( maxDepth+1 , d , off[0] , i1 );
			idx[2] = BinaryNode::CornerIndex( maxDepth+1 , d , off[2] , i2 );
			break;
		case 2:
			idx[0] = BinaryNode::CornerIndex( maxDepth+1 , d , off[0] , i1 );
			idx[1] = BinaryNode::CornerIndex( maxDepth+1 , d , off[1] , i2 );
			break;
	};
	return (long long)(idx[0]) | (long long)(idx[1])<<VERTEX_COORDINATE_SHIFT | (long long)(idx[2])<<(2*VERTEX_COORDINATE_SHIFT);
}


/////////////////////
// SortedTreeNodes //
/////////////////////
SortedTreeNodes::SortedTreeNodes( void )
{
	_sliceStart = NullPointer( Pointer( int ) );
	treeNodes = NullPointer( TreeOctNode* );
	_levels = 0;
}
SortedTreeNodes::~SortedTreeNodes( void )
{
	if( _sliceStart ) for( int d=0 ; d<_levels ; d++ ) FreePointer( _sliceStart[d] );
	FreePointer( _sliceStart );
	DeletePointer( treeNodes );
}
void SortedTreeNodes::set( TreeOctNode& root , std::vector< int >* map )
{
	set( root );

	if( map )
	{
		map->resize( _sliceStart[_levels-1][(size_t)1<<(_levels-1)] );
		for( int i=0 ; i<_sliceStart[_levels-1][(size_t)1<<(_levels-1)] ; i++ ) (*map)[i] = treeNodes[i]->nodeData.nodeIndex;
	}
	for( int i=0 ; i<_sliceStart[_levels-1][(size_t)1<<(_levels-1)] ; i++ ) treeNodes[i]->nodeData.nodeIndex = i;
}
void SortedTreeNodes::set( TreeOctNode& root )
{
	_levels = root.maxDepth()+1;

	if( _sliceStart ) for( int d=0 ; d<_levels ; d++ ) FreePointer( _sliceStart[d] );
	FreePointer( _sliceStart );
	DeletePointer( treeNodes );

	_sliceStart = AllocPointer< Pointer( int ) >( _levels );
	for( int l=0 ; l<_levels ; l++ )
	{
		_sliceStart[l] = AllocPointer< int >( ((size_t)1<<l)+1 );
		memset( _sliceStart[l] , 0 , sizeof(int)*( ((size_t)1<<l)+1 ) );
	}

	// Count the number of nodes in each slice
	for( TreeOctNode* node = root.nextNode() ; node ; node = root.nextNode( node ) ) if( !GetGhostFlag( node ) )
		{
			int d , off[3];
			node->depthAndOffset( d , off );
			_sliceStart[d][ off[2]+1 ]++;
		}

	// Get the start index for each slice
	{
		int levelOffset = 0;
		for( int l=0 ; l<_levels ; l++ )
		{
			_sliceStart[l][0] = levelOffset;
			for( int s=0 ; s<((size_t)1<<l); s++ ) _sliceStart[l][s+1] += _sliceStart[l][s];
			levelOffset = _sliceStart[l][(size_t)1<<l];
		}
	}
	// Allocate memory for the tree nodes
	treeNodes = NewPointer< TreeOctNode* >( _sliceStart[_levels-1][(size_t)1<<(_levels-1)] );

	// Add the tree nodes
	for( TreeOctNode* node=root.nextNode() ; node ; node=root.nextNode( node ) ) if( !GetGhostFlag( node ) )
		{
			int d , off[3];
			node->depthAndOffset( d , off );
			treeNodes[ _sliceStart[d][ off[2] ]++ ] = node;
		}

	// Shift the slice offsets up since we incremented as we added
	for( int l=0 ; l<_levels ; l++ )
	{
		for( int s=(1<<l) ; s>0 ; s-- ) _sliceStart[l][s] = _sliceStart[l][s-1];
		_sliceStart[l][0] = l>0 ? _sliceStart[l-1][(size_t)1<<(l-1)] : 0;
	}
}
SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::SliceTableData::cornerIndices( const TreeOctNode* node ) { return cTable[ node->nodeData.nodeIndex - nodeOffset ]; }
SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::SliceTableData::cornerIndices( int idx ) { return cTable[ idx - nodeOffset ]; }
const SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::SliceTableData::cornerIndices( const TreeOctNode* node ) const { return cTable[ node->nodeData.nodeIndex - nodeOffset ]; }
const SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::SliceTableData::cornerIndices( int idx ) const { return cTable[ idx - nodeOffset ]; }
SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::SliceTableData::edgeIndices( const TreeOctNode* node ) { return eTable[ node->nodeData.nodeIndex - nodeOffset ]; }
SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::SliceTableData::edgeIndices( int idx ) { return eTable[ idx - nodeOffset ]; }
const SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::SliceTableData::edgeIndices( const TreeOctNode* node ) const { return eTable[ node->nodeData.nodeIndex - nodeOffset ]; }
const SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::SliceTableData::edgeIndices( int idx ) const { return eTable[ idx - nodeOffset ]; }
SortedTreeNodes::SquareFaceIndices& SortedTreeNodes::SliceTableData::faceIndices( const TreeOctNode* node ) { return fTable[ node->nodeData.nodeIndex - nodeOffset ]; }
SortedTreeNodes::SquareFaceIndices& SortedTreeNodes::SliceTableData::faceIndices( int idx ) { return fTable[ idx - nodeOffset ]; }
const SortedTreeNodes::SquareFaceIndices& SortedTreeNodes::SliceTableData::faceIndices( const TreeOctNode* node ) const { return fTable[ node->nodeData.nodeIndex - nodeOffset ]; }
const SortedTreeNodes::SquareFaceIndices& SortedTreeNodes::SliceTableData::faceIndices( int idx ) const { return fTable[ idx - nodeOffset ]; }
SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::XSliceTableData::edgeIndices( const TreeOctNode* node ) { return eTable[ node->nodeData.nodeIndex - nodeOffset ]; }
SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::XSliceTableData::edgeIndices( int idx ) { return eTable[ idx - nodeOffset ]; }
const SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::XSliceTableData::edgeIndices( const TreeOctNode* node ) const { return eTable[ node->nodeData.nodeIndex - nodeOffset ]; }
const SortedTreeNodes::SquareCornerIndices& SortedTreeNodes::XSliceTableData::edgeIndices( int idx ) const { return eTable[ idx - nodeOffset ]; }
SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::XSliceTableData::faceIndices( const TreeOctNode* node ) { return fTable[ node->nodeData.nodeIndex - nodeOffset ]; }
SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::XSliceTableData::faceIndices( int idx ) { return fTable[ idx - nodeOffset ]; }
const SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::XSliceTableData::faceIndices( const TreeOctNode* node ) const { return fTable[ node->nodeData.nodeIndex - nodeOffset ]; }
const SortedTreeNodes::SquareEdgeIndices& SortedTreeNodes::XSliceTableData::faceIndices( int idx ) const { return fTable[ idx - nodeOffset ]; }

void SortedTreeNodes::setSliceTableData( SliceTableData& sData , int depth , int offset , int threads ) const
{
	// [NOTE] This is structure is purely for determining adjacency and is independent of the FEM degree
	typedef OctNode< TreeNodeData >::template ConstNeighborKey< 1 , 1 > ConstAdjacenctNodeKey;
	if( offset<0 || offset>((size_t)1<<depth) ) return;
	if( threads<=0 ) threads = 1;
	// The vector of per-depth node spans
	std::pair< int , int > span( _sliceStart[depth][ std::max< int >( 0 , offset-1 ) ] , _sliceStart[depth][ std::min< int >( (size_t)1<<depth , offset+1 ) ] );
	sData.nodeOffset = span.first;
	sData.nodeCount = span.second - span.first;

	DeletePointer( sData._cMap ) ; DeletePointer( sData._eMap ) ; DeletePointer( sData._fMap );
	DeletePointer( sData.cTable ) ; DeletePointer( sData.eTable ) ; DeletePointer( sData.fTable );
	if( sData.nodeCount )
	{
		sData._cMap = NewPointer< int >( sData.nodeCount * Square::CORNERS );
		sData._eMap = NewPointer< int >( sData.nodeCount * Square::EDGES );
		sData._fMap = NewPointer< int >( sData.nodeCount * Square::FACES );
		sData.cTable = NewPointer< typename SortedTreeNodes::SquareCornerIndices >( sData.nodeCount );
		sData.eTable = NewPointer< typename SortedTreeNodes::SquareCornerIndices >( sData.nodeCount );
		sData.fTable = NewPointer< typename SortedTreeNodes::SquareFaceIndices >( sData.nodeCount );
		memset( sData._cMap , 0 , sizeof(int) * sData.nodeCount * Square::CORNERS );
		memset( sData._eMap , 0 , sizeof(int) * sData.nodeCount * Square::EDGES );
		memset( sData._fMap , 0 , sizeof(int) * sData.nodeCount * Square::FACES );
	}
	std::vector< ConstAdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( depth );
#pragma omp parallel for num_threads( threads )
	for( int i=span.first ; i<span.second ; i++ )
	{
		ConstAdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];
		TreeOctNode* node = treeNodes[i];
		const TreeOctNode::ConstNeighbors< 3 >& neighbors = neighborKey.getNeighbors( node );
		int d , off[3];
		node->depthAndOffset( d , off );
		int z;
		if     ( off[2]==offset-1 ) z = 1;
		else if( off[2]==offset   ) z = 0;
		else fprintf( stderr , "[ERROR] Node out of bounds: %d %d\n" , offset , off[2] ) , exit( 0 );
		// Process the corners
		for( int x=0 ; x<2 ; x++ ) for( int y=0 ; y<2 ; y++ )
			{
				int c = Cube::CornerIndex( x , y , z );
				int fc = Square::CornerIndex( x , y );
				bool cornerOwner = true;
				int ac = Cube::AntipodalCornerIndex(c); // The index of the node relative to the corner
				for( int cc=0 ; cc<Cube::CORNERS ; cc++ ) // Iterate over the corner's cells
				{
					int xx , yy , zz;
					Cube::FactorCornerIndex( cc , xx , yy , zz );
					xx += x , yy += y , zz += z;
					if( IsActiveNode( neighbors.neighbors[xx][yy][zz] ) && cc<ac ){ cornerOwner = false ; break; }
				}
				if( cornerOwner )
				{
					int myCount = (i - sData.nodeOffset) * Square::CORNERS + fc;
					sData._cMap[ myCount ] = 1;
					for( int cc=0 ; cc<Cube::CORNERS ; cc++ )
					{
						int xx , yy , zz;
						Cube::FactorCornerIndex( cc , xx , yy , zz );
						int ac = Square::CornerIndex( 1-xx , 1-yy );
						xx += x , yy += y , zz += z;
						if( IsActiveNode( neighbors.neighbors[xx][yy][zz] ) ) sData.cornerIndices( neighbors.neighbors[xx][yy][zz] )[ac] = myCount;
					}
				}
			}
		// Process the edges
		for( int o=0 ; o<2 ; o++ ) for( int y=0 ; y<2 ; y++ )
			{
				int fe = Square::EdgeIndex( o , y );
				bool edgeOwner = true;

				int ac = Square::AntipodalCornerIndex( Square::CornerIndex( y , z ) );
				for( int cc=0 ; cc<Square::CORNERS ; cc++ )
				{
					int ii , jj , xx , yy , zz;
					Square::FactorCornerIndex( cc , ii , jj );
					ii += y , jj += z;
					switch( o )
					{
						case 0: yy = ii , zz = jj , xx = 1 ; break;
						case 1: xx = ii , zz = jj , yy = 1 ; break;
					}
					if( IsActiveNode( neighbors.neighbors[xx][yy][zz] ) && cc<ac ){ edgeOwner = false ; break; }
				}
				if( edgeOwner )
				{
					int myCount = ( i - sData.nodeOffset ) * Square::EDGES + fe;
					sData._eMap[ myCount ] = 1;
					// Set all edge indices
					for( int cc=0 ; cc<Square::CORNERS ; cc++ )
					{
						int ii , jj , aii , ajj , xx , yy , zz;
						Square::FactorCornerIndex( cc , ii , jj );
						Square::FactorCornerIndex( Square::AntipodalCornerIndex( cc ) , aii , ajj );
						ii += y , jj += z;
						switch( o )
						{
							case 0: yy = ii , zz = jj , xx = 1 ; break;
							case 1: xx = ii , zz = jj , yy = 1 ; break;
						}
						if( IsActiveNode( neighbors.neighbors[xx][yy][zz] ) ) sData.edgeIndices( neighbors.neighbors[xx][yy][zz] )[ Square::EdgeIndex( o , aii ) ] = myCount;
					}
				}
			}
		// Process the Faces
		{
			bool faceOwner = !( IsActiveNode( neighbors.neighbors[1][1][2*z] ) && !z );
			if( faceOwner )
			{
				int myCount = ( i - sData.nodeOffset ) * Square::FACES;
				sData._fMap[ myCount ] = 1;
				// Set the face indices
				sData.faceIndices( node )[0] = myCount;
				if( IsActiveNode( neighbors.neighbors[1][1][2*z] ) ) sData.faceIndices( neighbors.neighbors[1][1][2*z] )[0] = myCount;
			}
		}
	}
	int cCount = 0 , eCount = 0 , fCount = 0;

	for( size_t i=0 ; i<sData.nodeCount * Square::CORNERS ; i++ ) if( sData._cMap[i] ) sData._cMap[i] = cCount++;
	for( size_t i=0 ; i<sData.nodeCount * Square::EDGES   ; i++ ) if( sData._eMap[i] ) sData._eMap[i] = eCount++;
	for( size_t i=0 ; i<sData.nodeCount * Square::FACES   ; i++ ) if( sData._fMap[i] ) sData._fMap[i] = fCount++;
#pragma omp parallel for num_threads( threads )
	for( int i=0 ; i<sData.nodeCount ; i++ )
	{
		for( int j=0 ; j<Square::CORNERS ; j++ ) sData.cTable[i][j] = sData._cMap[ sData.cTable[i][j] ];
		for( int j=0 ; j<Square::EDGES   ; j++ ) sData.eTable[i][j] = sData._eMap[ sData.eTable[i][j] ];
		for( int j=0 ; j<Square::FACES   ; j++ ) sData.fTable[i][j] = sData._fMap[ sData.fTable[i][j] ];
	}

	sData.cCount = cCount , sData.eCount = eCount , sData.fCount = fCount;
}
void SortedTreeNodes::setXSliceTableData( XSliceTableData& sData , int depth , int offset , int threads ) const
{
	typedef OctNode< TreeNodeData >::template ConstNeighborKey< 1 , 1 > ConstAdjacenctNodeKey;
	if( offset<0 || offset>=((size_t)1<<depth) ) return;
	if( threads<=0 ) threads = 1;
	// The vector of per-depth node spans
	std::pair< int , int > span( _sliceStart[depth][offset] , _sliceStart[depth][offset+1] );
	sData.nodeOffset = span.first;
	sData.nodeCount = span.second - span.first;

	DeletePointer( sData._eMap ) ; DeletePointer( sData._fMap );
	DeletePointer( sData.eTable ) ; DeletePointer( sData.fTable );
	if( sData.nodeCount )
	{
		sData._eMap = NewPointer< int >( sData.nodeCount * Square::CORNERS );
		sData._fMap = NewPointer< int >( sData.nodeCount * Square::EDGES );
		sData.eTable = NewPointer< typename SortedTreeNodes::SquareCornerIndices >( sData.nodeCount );
		sData.fTable = NewPointer< typename SortedTreeNodes::SquareEdgeIndices >( sData.nodeCount );
		memset( sData._eMap , 0 , sizeof(int) * sData.nodeCount * Square::CORNERS );
		memset( sData._fMap , 0 , sizeof(int) * sData.nodeCount * Square::EDGES );
	}

	std::vector< ConstAdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( depth );
#pragma omp parallel for num_threads( threads )
	for( int i=span.first ; i<span.second ; i++ )
	{
		ConstAdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];
		TreeOctNode* node = treeNodes[i];
		const TreeOctNode::ConstNeighbors<3>& neighbors = neighborKey.getNeighbors( node );
		int d , off[3];
		node->depthAndOffset( d , off );
		// Process the edges
		int o=2;
		for( int x=0 ; x<2 ; x++ ) for( int y=0 ; y<2 ; y++ )
			{
				int fc = Square::CornerIndex( x , y );
				bool edgeOwner = true;

				int ac = Square::AntipodalCornerIndex( Square::CornerIndex( x , y ) );
				for( int cc=0 ; cc<Square::CORNERS ; cc++ )
				{
					int ii , jj , xx , yy , zz;
					Square::FactorCornerIndex( cc , ii , jj );
					ii += x , jj += y;
					xx = ii , yy = jj , zz = 1;
					if( IsActiveNode( neighbors.neighbors[xx][yy][zz] ) && cc<ac ){ edgeOwner = false ; break; }
				}
				if( edgeOwner )
				{
					int myCount = ( i - sData.nodeOffset ) * Square::CORNERS + fc;
					sData._eMap[ myCount ] = 1;

					// Set all edge indices
					for( int cc=0 ; cc<Square::CORNERS ; cc++ )
					{
						int ii , jj , aii , ajj , xx , yy , zz;
						Square::FactorCornerIndex( cc , ii , jj );
						Square::FactorCornerIndex( Square::AntipodalCornerIndex( cc ) , aii , ajj );
						ii += x , jj += y;
						xx = ii , yy = jj , zz = 1;
						if( IsActiveNode( neighbors.neighbors[xx][yy][zz] ) ) sData.edgeIndices( neighbors.neighbors[xx][yy][zz] )[ Square::CornerIndex( aii , ajj ) ] = myCount;
					}
				}
			}
		// Process the faces
		for( int o=0 ; o<2 ; o++ ) for( int y=0 ; y<2 ; y++ )
			{
				bool faceOwner;
				if( o==0 ) faceOwner = !( IsActiveNode( neighbors.neighbors[1][2*y][1] ) && !y );
				else       faceOwner = !( IsActiveNode( neighbors.neighbors[2*y][1][1] ) && !y );
				if( faceOwner )
				{
					int fe = Square::EdgeIndex( o , y );
					int ae = Square::EdgeIndex( o , 1-y );
					int myCount = ( i - sData.nodeOffset ) * Square::EDGES + fe;
					sData._fMap[ myCount ] = 1;
					// Set the face indices
					sData.faceIndices( node )[fe] = myCount;
					if( o==0 && IsActiveNode( neighbors.neighbors[1][2*y][1] ) ) sData.faceIndices( neighbors.neighbors[1][2*y][1] )[ae] = myCount;
					if( o==1 && IsActiveNode( neighbors.neighbors[2*y][1][1] ) ) sData.faceIndices( neighbors.neighbors[2*y][1][1] )[ae] = myCount;
				}
			}
	}
	int eCount = 0 , fCount = 0;

	for( size_t i=0 ; i<sData.nodeCount * Square::CORNERS ; i++ ) if( sData._eMap[i] ) sData._eMap[i] = eCount++;
	for( size_t i=0 ; i<sData.nodeCount * Square::EDGES   ; i++ ) if( sData._fMap[i] ) sData._fMap[i] = fCount++;
#pragma omp parallel for num_threads( threads )
	for( int i=0 ; i<sData.nodeCount ; i++ )
	{
		for( int j=0 ; j<Square::CORNERS ; j++ ) sData.eTable[i][j] = sData._eMap[ sData.eTable[i][j] ];
		for( int j=0 ; j<Square::EDGES   ; j++ ) sData.fTable[i][j] = sData._fMap[ sData.fTable[i][j] ];
	}

	sData.eCount = eCount , sData.fCount = fCount;
}


// evaluate the result of splatting along a plane and then evaluating at a point on the plane.
template< int Degree > double GetScaleValue( void )
{
	double centerValues[Degree+1];
	Polynomial< Degree >::BSplineComponentValues( 0.5 , centerValues );
	double scaleValue = 0;
	for( int i=0 ; i<=Degree ; i++ ) scaleValue += centerValues[i] * centerValues[i];
	return 1./ scaleValue;
}
template< class Real >
template< int WeightDegree >
void Octree< Real >::_addWeightContribution( DensityEstimator< WeightDegree >& densityWeights , TreeOctNode* node , Point3D< Real > position , PointSupportKey< WeightDegree >& weightKey , Real weight )
{
	static const double ScaleValue = GetScaleValue< WeightDegree >();
	double dx[ DIMENSION ][ PointSupportKey< WeightDegree >::Size ];
	typename TreeOctNode::Neighbors< PointSupportKey< WeightDegree >::Size >& neighbors = weightKey.template getNeighbors< true >( node , _NodeInitializer );
	densityWeights.reserve( NodeCount() );
	Point3D< Real > start;
	Real w;
	_startAndWidth( node , start , w );
	for( int dim=0 ; dim<DIMENSION ; dim++ ) Polynomial< WeightDegree >::BSplineComponentValues( ( position[dim]-start[dim] ) / w , dx[dim] );

	weight *= (Real)ScaleValue;

	for( int i=0 ; i<PointSupportKey< WeightDegree >::Size ; i++ ) for( int j=0 ; j<PointSupportKey< WeightDegree >::Size ; j++ )
		{
			double dxdy = dx[0][i] * dx[1][j] * weight;
			TreeOctNode** _neighbors = neighbors.neighbors[i][j];
			for( int k=0 ; k<PointSupportKey< WeightDegree >::Size ; k++ ) if( _neighbors[k] ) densityWeights[ _neighbors[k] ] += Real( dxdy * dx[2][k] );
		}
}

template< class Real >
template< int WeightDegree , class PointSupportKey >
Real Octree< Real >::_getSamplesPerNode( const DensityEstimator< WeightDegree >& densityWeights , const TreeOctNode* node , Point3D< Real > position , PointSupportKey& weightKey ) const
{
	Real weight = 0;
	double dx[ DIMENSION ][ PointSupportKey::Size ];
	const typename PointSupportKey::template Neighbors< PointSupportKey::Size >& neighbors = weightKey.getNeighbors( node );

	Point3D< Real > start;
	Real w;
	_startAndWidth( node , start , w );

	for( int dim=0 ; dim<DIMENSION ; dim++ ) Polynomial< WeightDegree >::BSplineComponentValues( ( position[dim]-start[dim] ) / w , dx[dim] );

	for( int i=0 ; i<PointSupportKey::Size ; i++ ) for( int j=0 ; j<PointSupportKey::Size ; j++ )
		{
			double dxdy = dx[0][i] * dx[1][j];
			for( int k=0 ; k<PointSupportKey::Size ; k++ ) if( neighbors.neighbors[i][j][k] )
				{
					const Real* w = densityWeights( neighbors.neighbors[i][j][k] );
					if( w ) weight += Real( dxdy * dx[2][k] * (*w) );
				}
		}
	return weight;
}
template< class Real >
template< int WeightDegree , class PointSupportKey >
void Octree< Real >::_getSampleDepthAndWeight( const DensityEstimator< WeightDegree >& densityWeights , const TreeOctNode* node , Point3D< Real > position , PointSupportKey& weightKey , Real& depth , Real& weight ) const
{
	const TreeOctNode* temp = node;
	while( _localDepth( temp )>densityWeights.kernelDepth() ) temp = temp->parent;
	weight = _getSamplesPerNode( densityWeights , temp , position , weightKey );
	if( weight>=(Real)1. ) depth = Real( _localDepth( temp ) + log( weight ) / log(double(1<<(DIMENSION-1))) );
	else
	{
		Real oldWeight , newWeight;
		oldWeight = newWeight = weight;
		while( newWeight<(Real)1. && temp->parent )
		{
			temp=temp->parent;
			oldWeight = newWeight;
			newWeight = _getSamplesPerNode( densityWeights , temp , position , weightKey );
		}
		depth = Real( _localDepth( temp ) + log( newWeight ) / log( newWeight / oldWeight ) );
	}
	weight = Real( pow( double(1<<(DIMENSION-1)) , -double(depth) ) );
}
template< class Real >
template< int WeightDegree , class PointSupportKey >
void Octree< Real >::_getSampleDepthAndWeight( const DensityEstimator< WeightDegree >& densityWeights , Point3D< Real > position , PointSupportKey& weightKey , Real& depth , Real& weight ) const
{
	TreeOctNode* temp;
	Point3D< Real > myCenter( (Real)0.5 , (Real)0.5 , (Real)0.5 );
	Real myWidth = Real( 1. );

	// Get the finest node with depth less than or equal to the splat depth that contains the point
	temp = _spaceRoot;
	while( _localDepth( temp )<densityWeights.kernelDepth() )
	{
		if( !IsActiveNode( temp->children ) ) break;// fprintf( stderr , "[ERROR] Octree::GetSampleDepthAndWeight\n" ) , exit( 0 );
		int cIndex = TreeOctNode::CornerIndex( myCenter , position );
		temp = temp->children + cIndex;
		myWidth /= 2;
		if( cIndex&1 ) myCenter[0] += myWidth/2;
		else		   myCenter[0] -= myWidth/2;
		if( cIndex&2 ) myCenter[1] += myWidth/2;
		else		   myCenter[1] -= myWidth/2;
		if( cIndex&4 ) myCenter[2] += myWidth/2;
		else		   myCenter[2] -= myWidth/2;
	}
	return _getSampleDepthAndWeight( densityWeights , temp , position , weightKey , depth , weight );
}

template< class Real >
template< bool CreateNodes , int DataDegree , class V >
void Octree< Real >::_splatPointData( TreeOctNode* node , Point3D< Real > position , V v , SparseNodeData< V , DataDegree >& dataInfo , PointSupportKey< DataDegree >& dataKey )
{
	double dx[ DIMENSION ][ PointSupportKey< DataDegree >::Size ];
	typename TreeOctNode::Neighbors< PointSupportKey< DataDegree >::Size >& neighbors = dataKey.template getNeighbors< CreateNodes >( node , _NodeInitializer );
	Point3D< Real > start;
	Real w;
	_startAndWidth( node , start , w );

	for( int dd=0 ; dd<DIMENSION ; dd++ ) Polynomial< DataDegree >::BSplineComponentValues( ( position[dd]-start[dd] ) / w , dx[dd] );

	for( int i=0 ; i<PointSupportKey< DataDegree >::Size ; i++ ) for( int j=0 ; j<PointSupportKey< DataDegree >::Size ; j++ )
		{
			double dxdy = dx[0][i] * dx[1][j];
			for( int k=0 ; k<PointSupportKey< DataDegree >::Size ; k++ )
				if( IsActiveNode( neighbors.neighbors[i][j][k] ) )
				{
					TreeOctNode* _node = neighbors.neighbors[i][j][k];

					double dxdydz = dxdy * dx[2][k];
					dataInfo[ _node ] += v * (Real)dxdydz;
				}
		}
}
template< class Real >
template< bool CreateNodes , int WeightDegree , int DataDegree , class V >
Real Octree< Real >::_splatPointData( const DensityEstimator< WeightDegree >& densityWeights , Point3D< Real > position , V v , SparseNodeData< V , DataDegree >& dataInfo , PointSupportKey< WeightDegree >& weightKey , PointSupportKey< DataDegree >& dataKey , LocalDepth minDepth , LocalDepth maxDepth , int dim )
{
	double dx;
	V _v;
	TreeOctNode* temp;
	int cnt=0;
	double width;
	Point3D< Real > myCenter( (Real)0.5 , (Real)0.5 , (Real)0.5 );
	Real myWidth = (Real)1.;

	temp = _spaceRoot;
	while( _localDepth( temp )<densityWeights.kernelDepth() )
	{
		if( !IsActiveNode( temp->children ) ) break;
		int cIndex = TreeOctNode::CornerIndex( myCenter , position );
		temp = temp->children + cIndex;
		myWidth /= 2;
		if( cIndex&1 ) myCenter[0] += myWidth/2;
		else		   myCenter[0] -= myWidth/2;
		if( cIndex&2 ) myCenter[1] += myWidth/2;
		else 	  	   myCenter[1] -= myWidth/2;
		if( cIndex&4 ) myCenter[2] += myWidth/2;
		else 		   myCenter[2] -= myWidth/2;
	}
	Real weight , depth;
	_getSampleDepthAndWeight( densityWeights , temp , position , weightKey , depth , weight );

	if( depth<minDepth ) depth = Real(minDepth);
	if( depth>maxDepth ) depth = Real(maxDepth);
	int topDepth = int(ceil(depth));

	dx = 1.0-(topDepth-depth);
	if     ( topDepth<=minDepth ) topDepth = minDepth , dx = 1;
	else if( topDepth> maxDepth ) topDepth = maxDepth , dx = 1;

	while( _localDepth( temp )>topDepth ) temp=temp->parent;
	while( _localDepth( temp )<topDepth )
	{
		if( !temp->children ) temp->initChildren( _NodeInitializer );
		int cIndex = TreeOctNode::CornerIndex( myCenter , position );
		temp = &temp->children[cIndex];
		myWidth/=2;
		if( cIndex&1 ) myCenter[0] += myWidth/2;
		else		   myCenter[0] -= myWidth/2;
		if( cIndex&2 ) myCenter[1] += myWidth/2;
		else		   myCenter[1] -= myWidth/2;
		if( cIndex&4 ) myCenter[2] += myWidth/2;
		else		   myCenter[2] -= myWidth/2;
	}
	width = 1.0 / ( 1<<_localDepth( temp ) );
	_v = v * weight / Real( pow( width , dim ) ) * Real( dx );
	_splatPointData< CreateNodes >( temp , position , _v , dataInfo , dataKey );
	if( fabs(1.0-dx) > EPSILON )
	{
		dx = Real(1.0-dx);
		temp = temp->parent;
		width = 1.0 / ( 1<<_localDepth( temp ) );

		_v = v * weight / Real( pow( width , dim ) ) * Real( dx );
		_splatPointData< CreateNodes >( temp , position , _v , dataInfo , dataKey );
	}
	return weight;
}
template< class Real >
template< bool CreateNodes , int WeightDegree , int DataDegree , class V >
Real Octree< Real >::_multiSplatPointData( const DensityEstimator< WeightDegree >* densityWeights , TreeOctNode* node , Point3D< Real > position , V v , SparseNodeData< V , DataDegree >& dataInfo , PointSupportKey< WeightDegree >& weightKey , PointSupportKey< DataDegree >& dataKey , int dim )
{
	Real _depth , weight;
	if( densityWeights ) _getSampleDepthAndWeight( *densityWeights , position , weightKey , _depth , weight );
	else weight = (Real)1.;
	V _v = v * weight;

	double dx[ DIMENSION ][ PointSupportKey< DataDegree >::Size ];
	dataKey.template getNeighbors< CreateNodes >( node , _NodeInitializer );

	for( TreeOctNode* _node=node ; _localDepth( _node )>=0 ; _node=_node->parent )
	{
		V __v = _v * (Real)pow( 1<<_localDepth( _node ) , dim );
		Point3D< Real > start;
		Real w;
		_startAndWidth( _node , start , w );
		for( int dd=0 ; dd<DIMENSION ; dd++ ) Polynomial< DataDegree >::BSplineComponentValues( ( position[dd]-start[dd] ) / w , dx[dd] );
		typename TreeOctNode::Neighbors< PointSupportKey< DataDegree >::Size >& neighbors = dataKey.neighbors[ _localToGlobal( _localDepth( _node ) ) ];
		for( int i=0 ; i<PointSupportKey< DataDegree >::Size ; i++ ) for( int j=0 ; j<PointSupportKey< DataDegree >::Size ; j++ )
			{
				double dxdy = dx[0][i] * dx[1][j];
				for( int k=0 ; k<PointSupportKey< DataDegree >::Size ; k++ )
					if( IsActiveNode( neighbors.neighbors[i][j][k] ) )
					{
						TreeOctNode* _node = neighbors.neighbors[i][j][k];
						double dxdydz = dxdy * dx[2][k];
						dataInfo[ _node ] += __v * (Real)dxdydz;
					}
			}
	}
	return weight;
}

template< class Real >
template< class V , int DataDegree , BoundaryType BType , class Coefficients >
V Octree< Real >::_evaluate( const Coefficients& coefficients , Point3D< Real > p , const BSplineData< DataDegree , BType >& bsData , const ConstPointSupportKey< DataDegree >& dataKey ) const
{
	V value = V(0);

	for( int d=_localToGlobal( 0 ) ; d<=dataKey.depth() ; d++ )
	{
		double dx[ DIMENSION ][ PointSupportKey< DataDegree >::Size ];
		memset( dx , 0 , sizeof( double ) * DIMENSION * PointSupportKey< DataDegree >::Size );
		{
			const TreeOctNode* n = dataKey.neighbors[d].neighbors[ PointSupportKey< DataDegree >::LeftRadius ][ PointSupportKey< DataDegree >::LeftRadius ][ PointSupportKey< DataDegree >::LeftRadius ];
			if( !n ) fprintf( stderr , "[ERROR] Point is not centered on a node\n" ) , exit( 0 );
			int fIdx[3];
			functionIndex< DataDegree , BType >( n , fIdx );
			int fStart , fEnd;
			BSplineData< DataDegree , BType >::FunctionSpan( _localDepth( n ) , fStart , fEnd );
			for( int dd=0 ; dd<DIMENSION ; dd++ ) for( int i=-PointSupportKey< DataDegree >::LeftRadius ; i<=PointSupportKey< DataDegree >::RightRadius ; i++ )
					if( fIdx[dd]+i>=fStart && fIdx[dd]+i<fEnd ) dx[dd][i] = bsData.baseBSplines[ fIdx[dd]+i ][ -i+PointSupportKey< DataDegree >::RightRadius ]( p[dd] );
		}
		for( int i=0 ; i<PointSupportKey< DataDegree >::Size ; i++ ) for( int j=0 ; j<PointSupportKey< DataDegree >::Size ; j++ ) for( int k=0 ; k<PointSupportKey< DataDegree >::Size ; k++ )
				{
					const TreeOctNode* n = dataKey.neighbors[d].neighbors[i][j][k];
					if( isValidFEMNode< DataDegree , BType >( n ) )
					{
						const V* v = coefficients( n );
						if( v ) value += (*v) * (Real) ( dx[0][i] * dx[1][j] * dx[2][k] );
					}
				}
	}

	return value;
}

template< class Real >
template< class V , int DataDegree , BoundaryType BType >
Pointer( V ) Octree< Real >::voxelEvaluate( const DenseNodeData< V , DataDegree >& coefficients , int& res , Real isoValue , LocalDepth depth , bool primal )
{
	int begin , end , dim;
	if( depth<=0 || depth>_maxDepth ) depth = _maxDepth;

	// Initialize the coefficients at the coarsest level
	Pointer( V ) _coefficients = NullPointer( V );
	{
		LocalDepth d = 0;
		begin = _BSplineBegin< DataDegree , BType >( d ) , end = _BSplineEnd< DataDegree , BType >( d ) , dim = end - begin;
		_coefficients = NewPointer< V >( dim * dim * dim );
		memset( _coefficients , 0 , sizeof( V ) * dim  * dim * dim );
#pragma omp parallel for num_threads( threads )
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( !_outOfBounds< DataDegree , BType >( _sNodes.treeNodes[i] ) )
			{
				LocalDepth _d ; LocalOffset _off;
				_localDepthAndOffset( _sNodes.treeNodes[i] , _d , _off );
				_off[0] -= begin , _off[1] -= begin , _off[2] -= begin;
				_coefficients[ _off[0] + _off[1]*dim + _off[2]*dim*dim ] = coefficients[i];
			}
	}

	// Up-sample and add in the existing coefficients
	for( LocalDepth d=1 ; d<=depth ; d++ )
	{
		begin = _BSplineBegin< DataDegree , BType >( d ) , end = _BSplineEnd< DataDegree , BType >( d ) , dim = end - begin;
		Pointer( V ) __coefficients = NewPointer< V >( dim * dim *dim );
		memset( __coefficients , 0 , sizeof( V ) * dim  * dim * dim );
#pragma omp parallel for num_threads( threads )
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( !_outOfBounds< DataDegree , BType >( _sNodes.treeNodes[i] ) )
			{
				LocalDepth _d ; LocalOffset _off;
				_localDepthAndOffset( _sNodes.treeNodes[i] , _d , _off );
				_off[0] -= begin , _off[1] -= begin , _off[2] -= begin;
				__coefficients[ _off[0] + _off[1]*dim + _off[2]*dim*dim ] = coefficients[i];
			}
		_UpSample< V , DataDegree , BType >( d , ( ConstPointer(V) )_coefficients , __coefficients , threads );
		DeletePointer( _coefficients );
		_coefficients = __coefficients;
	}

	res = 1<<depth;
	if( primal ) res++;
	Pointer( V ) values = NewPointer< V >( res*res*res );
	memset( values , 0 , sizeof(V)*res*res*res );

	if( primal )
	{
		// evaluate at the cell corners
		typename BSplineEvaluationData< DataDegree , BType >::CornerEvaluator::Evaluator evaluator;
		BSplineEvaluationData< DataDegree , BType >::SetCornerEvaluator( evaluator , depth );
#pragma omp parallel for num_threads( threads )
		for( int k=0 ; k<res ; k++ ) for( int j=0 ; j<res ; j++ ) for( int i=0 ; i<res ; i++ )
				{
					V value = values[ i + j*res + k*res*res ];
					for( int kk=-BSplineSupportSizes< DataDegree >::CornerEnd ; kk<=-BSplineSupportSizes< DataDegree >::CornerStart ; kk++ ) if( k+kk>=begin && k+kk<end )
							for( int jj=-BSplineSupportSizes< DataDegree >::CornerEnd ; jj<=-BSplineSupportSizes< DataDegree >::CornerStart ; jj++ ) if( j+jj>=begin && j+jj<end )
								{
									double weight = evaluator.value( k+kk , k , false ) * evaluator.value( j+jj , j , false );
									int idx = (j+jj-begin)*dim + (k+kk-begin)*dim*dim;
									for( int ii=-BSplineSupportSizes< DataDegree >::CornerEnd ; ii<=-BSplineSupportSizes< DataDegree >::CornerStart ; ii++ ) if( i+ii>=begin && i+ii<end )
											value += _coefficients[ i + ii - begin + idx ] * Real( weight * evaluator.value( i + ii , i , false ) );
								}
					values[ i + j*res + k*res*res ] = value;
				}
	}
	else
	{
		// evaluate at the cell centers
		typename BSplineEvaluationData< DataDegree , BType >::CenterEvaluator::Evaluator evaluator;
		BSplineEvaluationData< DataDegree , BType >::SetCenterEvaluator( evaluator , depth );
#pragma omp parallel for num_threads( threads )
		for( int k=0 ; k<res ; k++ ) for( int j=0 ; j<res ; j++ ) for( int i=0 ; i<res ; i++ )
				{
					V& value = values[ i + j*res + k*res*res ];
					for( int kk=-BSplineSupportSizes< DataDegree >::SupportEnd ; kk<=-BSplineSupportSizes< DataDegree >::SupportStart ; kk++ ) if( k+kk>=begin && k+kk<end )
							for( int jj=-BSplineSupportSizes< DataDegree >::SupportEnd ; jj<=-BSplineSupportSizes< DataDegree >::SupportStart ; jj++ ) if( j+jj>=begin && j+jj<end )
								{
									double weight = evaluator.value( k+kk , k , false ) * evaluator.value( j+jj , j , false );
									int idx = (j+jj-begin)*dim + (k+kk-begin)*dim*dim;
									for( int ii=-BSplineSupportSizes< DataDegree >::SupportEnd ; ii<=-BSplineSupportSizes< DataDegree >::SupportStart ; ii++ ) if( i+ii>=begin && i+ii<end )
											value += _coefficients[ i + ii - begin + idx ] * Real( weight * evaluator.value( i+ii , i , false ) );
								}
				}
	}
	DeletePointer( _coefficients );
	for( int i=0 ; i<res*res*res ; i++ ) values[i] -= isoValue;

	return values;
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
SparseNodeData< Real , 0 > Octree< Real >::leafValues( const DenseNodeData< Real , FEMDegree >& coefficients ) const
{
	SparseNodeData< Real , 0 > values;
	DenseNodeData< Real , FEMDegree > _coefficients( _sNodesEnd(_maxDepth-1) );
	memset( &_coefficients[0] , 0 , sizeof(Real)*_sNodesEnd(_maxDepth-1) );
	for( int i=_sNodes.begin( _localToGlobal( 0 ) ) ; i<_sNodesEnd(_maxDepth-1) ; i++ ) _coefficients[i] = coefficients[i];
	for( LocalDepth d=1 ; d<_maxDepth ; d++ ) _upSample( d , _coefficients );
	for( LocalDepth d=_maxDepth ; d>=0 ; d-- )
	{
		_Evaluator< FEMDegree , BType > evaluator;
		evaluator.set( d );
		std::vector< ConstPointSupportKey< FEMDegree > > neighborKeys( std::max< int >( 1 , threads ) );
		for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d ) );
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
			{
				ConstPointSupportKey< FEMDegree >& neighborKey = neighborKeys[ get_thread_num() ];
				TreeOctNode* node = _sNodes.treeNodes[i];
				if( !IsActiveNode( node->children ) )
				{
					neighborKey.getNeighbors( node );
					bool isInterior = _IsInteriorlySupported< FEMDegree >( node->parent );
					values[ node ] = _getCenterValue( neighborKey , node , coefficients , _coefficients , evaluator , isInterior );
				}
			}
	}
	return values;
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
SparseNodeData< Point3D< Real > , 0 > Octree< Real >::leafGradients( const DenseNodeData< Real , FEMDegree >& coefficients ) const
{
	SparseNodeData< Point3D< Real > , 0 > gradients;
	DenseNodeData< Real , FEMDegree > _coefficients( _sNodesEnd(_maxDepth-1 ) );
	memset( &_coefficients[0] , 0 , sizeof(Real)*_sNodesEnd(_maxDepth-1) );
	for( int i=_sNodesBegin(0) ; i<_sNodesEnd(_maxDepth-1) ; i++ ) _coefficients[i] = coefficients[i];
	for( LocalDepth d=1 ; d<_maxDepth ; d++ ) _upSample( d , _coefficients );
	for( LocalDepth d=_maxDepth ; d>=0 ; d-- )
	{
		_Evaluator< FEMDegree , BType > evaluator;
		evaluator.set( d );
		std::vector< ConstPointSupportKey< FEMDegree > > neighborKeys( std::max< int >( 1 , threads ) );
		for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d ) );
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
			{
				ConstPointSupportKey< FEMDegree >& neighborKey = neighborKeys[ get_thread_num() ];
				TreeOctNode* node = _sNodes.treeNodes[i];
				if( !IsActiveNode( node->children ) )
				{
					neighborKey.getNeighbors( node );
					bool isInterior = _IsInteriorlySupported< FEMDegree >( node->parent );
					gradients[ node ] = _getCenterValueAndGradient( neighborKey , node , coefficients , _coefficients , evaluator , isInterior ).second;
				}
			}
	}
	return gradients;
}


template< class Real , int Degree , bool HasGradients >
struct _ConstraintCalculator_
{
	static inline Real _CalculateConstraint_( const PointData< Real , HasGradients >& p , const Polynomial< Degree >& px , const Polynomial< Degree >& py , const Polynomial< Degree >& pz , const Polynomial< Degree >& dpx , const Polynomial< Degree >& dpy , const Polynomial< Degree >& dpz , Real valueWeight , Real gradientWeight );
	static inline Real _CalculateConstraint_( const PointData< Real , HasGradients >& p , const Polynomial< Degree >& px , const Polynomial< Degree >& py , const Polynomial< Degree >& pz , const Polynomial< Degree >& dpx , const Polynomial< Degree >& dpy , const Polynomial< Degree >& dpz );
#if POINT_DATA_RES
	static inline void _CalculateCoarser_( int c , PointData< Real , HasGradients >& p , Real value , Point3D< Real > gradient , Real valueWeight , Real gradientWeight );
#else // !POINT_DATA_RES
	static inline void _CalculateCoarser_( PointData< Real , HasGradients >& p , Real value , Point3D< Real > gradient , Real valueWeight , Real gradientWeight );
#endif // POINT_DATA_RES

};
template< class Real , int Degree >
struct _ConstraintCalculator_< Real , Degree , false >
{
	static inline Real _CalculateConstraint_( const PointData< Real , false >& p , const Polynomial< Degree >& px , const Polynomial< Degree >& py , const Polynomial< Degree >& pz , const Polynomial< Degree >& dpx , const Polynomial< Degree >& dpy , const Polynomial< Degree >& dpz , Real valueWeight , Real gradientWeight )
	{
#if POINT_DATA_RES
		Real constraint = 0;
		for( int c=0 ; c<PointData< Real , false >::SAMPLES ; c++ ) if( p[c].weight )
		{
			const Point3D< Real > q = p[c].position;
			constraint += (Real)( px( q[0] ) * py( q[1] ) * pz( q[2] ) * p[c].weight * p[c].value );
		}
		return constraint * valueWeight;
#else // !POINT_DATA_RES
		const Point3D< Real > q = p.position;
		return (Real)( px( q[0] ) * py( q[1] ) * pz( q[2] ) * p.weight * p.value ) * valueWeight;
#endif // POINT_DATA_RES
	}
	static inline Real _CalculateConstraint_( const PointData< Real , false >& p , const Polynomial< Degree >& px , const Polynomial< Degree >& py , const Polynomial< Degree >& pz , const Polynomial< Degree >& dpx , const Polynomial< Degree >& dpy , const Polynomial< Degree >& dpz )
	{
#if POINT_DATA_RES
		Real constraint = 0;
		for( int c=0 ; c<PointData< Real , false >::SAMPLES ; c++ ) if( p[c].weight )
		{
			const Point3D< Real > q = p[c].position;
			constraint += (Real)( px( q[0] ) * py( q[1] ) * pz( q[2] ) * p[c]._value );
		}
		return constraint;
#else // !POINT_DATA_RES
		const Point3D< Real > q = p.position;
		return (Real)( px( q[0] ) * py( q[1] ) * pz( q[2] ) * p._value );
#endif // POINT_DATA_RES
	}
#if POINT_DATA_RES
	static inline void _CalculateCoarser_( int c , PointData< Real , false >& p , Real value , Point3D< Real > gradient , Real valueWeight , Real gradientWeight ){ p[c]._value = value * valueWeight * p[c].weight; }
#else // !POINT_DATA_RES
	static inline void _CalculateCoarser_( PointData< Real , false >& p , Real value , Point3D< Real > gradient , Real valueWeight , Real gradientWeight ){ p._value = value * valueWeight * p.weight; }
#endif // POINT_DATA_RES
};
template< class Real , int Degree >
struct _ConstraintCalculator_< Real , Degree , true >
{
	static inline Real _CalculateConstraint_( const PointData< Real , true >& p , const Polynomial< Degree >& px , const Polynomial< Degree >& py , const Polynomial< Degree >& pz , const Polynomial< Degree >& dpx , const Polynomial< Degree >& dpy , const Polynomial< Degree >& dpz , Real valueWeight , Real gradientWeight )
	{
#if POINT_DATA_RES
		Real constraint = 0;
		for( int c=0 ; c<PointData< Real , true >::SAMPLES ; c++ ) if( p[c].weight )
		{
			const Point3D< Real > q = p[c].position;
			double _px = px( q[0] ) , _py = py( q[1] ) , _pz = pz( q[2] );
			constraint +=
				(
					(Real)( _px * _py * _pz * p[c].value ) * valueWeight +
					Point3D< Real >::Dot( Point3D< Real >( dpx( q[0] ) * _py * _pz , _px * dpy( q[1] ) * _pz , _px * _py * dpz( q[2] ) ) , p[c].gradient ) * gradientWeight
				) * p[c].weight;
		}
		return constraint;
#else // !POINT_DATA_RES
		const Point3D< Real > q = p.position;
		double _px = px( q[0] ) , _py = py( q[1] ) , _pz = pz( q[2] );
		return
				(
						(Real)( _px * _py * _pz * p.value ) * valueWeight +
						Point3D< Real >::Dot( Point3D< Real >( dpx( q[0] ) * _py * _pz , _px * dpy( q[1] ) * _pz , _px * _py * dpz( q[2] ) ) , p.gradient ) * gradientWeight
				) * p.weight;
#endif // POINT_DATA_RES
	}
	static inline Real _CalculateConstraint_( const PointData< Real , true >& p , const Polynomial< Degree >& px , const Polynomial< Degree >& py , const Polynomial< Degree >& pz , const Polynomial< Degree >& dpx , const Polynomial< Degree >& dpy , const Polynomial< Degree >& dpz )
	{
#if POINT_DATA_RES
		Real constraint = 0;
		for( int c=0 ; c<PointData< Real , true >::SAMPLES ; c++ ) if( p[c].weight )
		{
			const Point3D< Real > q = p[c].position;
			double _px = px( q[0] ) , _py = py( q[1] ) , _pz = pz( q[2] );
			constraint +=
				(Real)( _px * _py * _pz * p[c]._value ) +
				Point3D< Real >::Dot( Point3D< Real >( dpx( q[0] ) * _py * _pz , _px * dpy( q[1] ) * _pz , _px * _py * dpz( q[2] ) ) , p[c]._gradient );
		}
		return constraint;
#else // !POINT_DATA_RES
		const Point3D< Real > q = p.position;
		double _px = px( q[0] ) , _py = py( q[1] ) , _pz = pz( q[2] );
		return
				(Real)( _px * _py * _pz * p._value ) +
				Point3D< Real >::Dot( Point3D< Real >( dpx( q[0] ) * _py * _pz , _px * dpy( q[1] ) * _pz , _px * _py * dpz( q[2] ) ) , p._gradient );
#endif // POINT_DATA_RES
	}
#if POINT_DATA_RES
	static inline void _CalculateCoarser_( int c , PointData< Real , true >& p , Real value , Point3D< Real > gradient , Real valueWeight , Real gradientWeight ){ p[c]._value = value * valueWeight * p[c].weight ; p[c]._gradient = gradient * gradientWeight * p[c].weight; }
#else // !POINT_DATA_RES
	static inline void _CalculateCoarser_( PointData< Real , true >& p , Real value , Point3D< Real > gradient , Real valueWeight , Real gradientWeight ){ p._value = value * valueWeight * p.weight ; p._gradient = gradient * gradientWeight * p.weight; }
#endif // POINT_DATA_RES
};

template< >
template< class I >
double FEMSystemFunctor< 0 , BOUNDARY_FREE >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight;
#undef D_DOT
}
template< >
template< class I >
double FEMSystemFunctor< 0 , BOUNDARY_NEUMANN >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight;
#undef D_DOT
}
template< >
template< class I >
double FEMSystemFunctor< 0 , BOUNDARY_DIRICHLET >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight;
#undef D_DOT
}
template< >
template< class I >
double FEMSystemFunctor< 1 , BOUNDARY_FREE >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 ) , d11[] = D_DOT( 1 , 1 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight
			+
			(
					d11[0] * d00[1] * d00[2] +
					d11[1] * d00[2] * d00[0] +
					d11[2] * d00[0] * d00[1]
			) * lapWeight;
#undef D_DOT
}
template< >
template< class I >
double FEMSystemFunctor< 1 , BOUNDARY_NEUMANN >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 ) , d11[] = D_DOT( 1 , 1 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight
			+
			(
					d11[0] * d00[1] * d00[2] +
					d11[1] * d00[2] * d00[0] +
					d11[2] * d00[0] * d00[1]
			) * lapWeight;
#undef D_DOT
}
template< >
template< class I >
double FEMSystemFunctor< 1 , BOUNDARY_DIRICHLET >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 ) , d11[] = D_DOT( 1 , 1 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight
			+
			(
					d11[0] * d00[1] * d00[2] +
					d11[1] * d00[2] * d00[0] +
					d11[2] * d00[0] * d00[1]
			) * lapWeight;
#undef D_DOT
}

template< int FEMDegree , BoundaryType BType >
template< class I >
double FEMSystemFunctor< FEMDegree , BType >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , D1 , D2 ) , integrator.dot( off1[1] , off2[1] , D1 , D2 ) , integrator.dot( off1[2] , off2[2] , D1 , D2 ) }
	double d00[] = D_DOT( 0 , 0 ) , d02[] = D_DOT( 0 , 2 ) , d20[] = D_DOT( 2 , 0 ) , d22[] = D_DOT( 2 , 2 ) , d11[] = D_DOT( 1 , 1 );
	return
			(
					d00[0] * d00[1] * d00[2]
			) * massWeight
			+
			(
					d11[0] * d00[1] * d00[2] +
					d11[1] * d00[2] * d00[0] +
					d11[2] * d00[0] * d00[1]
			) * lapWeight
			+
			(
					d22[0] * d00[1] * d00[2] +							// Unmixed
					d22[1] * d00[2] * d00[0] +							// Unmixed
					d22[2] * d00[0] * d00[1] +							// Unmixed
					d00[0] * ( d02[1] * d20[2] + d20[1] * d02[2] ) +	//   Mixed
					d00[1] * ( d02[2] * d20[0] + d20[2] * d02[0] ) +	//   Mixed
					d00[2] * ( d02[0] * d20[1] + d20[0] * d02[1] )		//   Mixed
			) * biLapWeight;
#undef D_DOT
}

template< int VFDegree , BoundaryType VFBType , int FEMDegree , BoundaryType FEMBType >
template< bool Reverse , class I >
Point3D< double > FEMVFConstraintFunctor< VFDegree , VFBType , FEMDegree , FEMBType >::_integrate( const I& integrator , const int off1[] , const int off2[] ) const
{
#define D_DOT( D1 , D2 ) { integrator.dot( off1[0] , off2[0] , Reverse ? D2 : D1 , Reverse ? D1 : D2 ) , integrator.dot( off1[1] , off2[1] , Reverse ? D2 : D1 , Reverse ? D1 : D2 ) , integrator.dot( off1[2] , off2[2] , Reverse ? D2 : D1 , Reverse ? D1 : D2 ) }
	if( FEMDegree==0 ) fprintf( stderr , "[ERROR] FEMDegree does not support differentiation: %d\n" , FEMDegree  ) , exit( 0 );
	if( VFDegree==0 || FEMDegree==1 )
	{
		double d00[] = D_DOT( 0 , 0 ) , d01[] = D_DOT( 0 , 1 );
		return
				Point3D< double >
						(
								d01[0] * d00[1] * d00[2] ,
								d01[1] * d00[2] * d00[0] ,
								d01[2] * d00[0] * d00[1]
						) * lapWeight;
	}
	else
	{
		double d00[] = D_DOT( 0 , 0 ) , d10[] = D_DOT( 1 , 0 ) , d01[] = D_DOT( 0 , 1 ) , d02[] = D_DOT( 0 , 2 ) , d12[] = D_DOT( 1 , 2 );
		return
				Point3D< double >
						(
								d01[0] * d00[1] * d00[2] ,
								d01[1] * d00[2] * d00[0] ,
								d01[2] * d00[0] * d00[1]
						) * lapWeight
				+
				Point3D< double >
						(
								d12[0] * d00[1] * d00[2] + d10[0] * ( d00[1] * d02[2] + d02[1] * d00[2] ) ,
								d12[1] * d00[2] * d00[0] + d10[1] * ( d00[2] * d02[0] + d02[2] * d00[0] ) ,
								d12[2] * d00[0] * d00[1] + d10[2] * ( d00[0] * d02[1] + d02[0] * d00[1] )
						) * biLapWeight;
	}
#undef D_DOT
}

template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< bool Reverse , class _FEMSystemFunctor >
void SystemCoefficients< Degree1 , BType1 , Degree2 , BType2 >::SetCentralConstraintStencil( const _FEMSystemFunctor& F , const Integrator& integrator , Stencil< double , OverlapSize >& stencil  )
{
	int center = ( 1<<integrator.depth() )>>1;
	int offset[] = { center , center , center };
	for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
			{
				int _offset[] = { x+center-OverlapEnd , y+center-OverlapEnd , z+center-OverlapEnd };
				stencil( x , y , z ) = F.template integrate< Reverse >( integrator , _offset , offset );
			}
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< bool Reverse , class _FEMSystemFunctor >
void SystemCoefficients< Degree1 , BType1 , Degree2 , BType2 >::SetCentralConstraintStencils( const _FEMSystemFunctor& F , const ChildIntegrator& integrator , Stencil< double , OverlapSize > stencils[2][2][2] )
{
	int center = ( 1<<integrator.childDepth() )>>1;
	// [NOTE] We want the center to be at the first node of the brood
	// Which is not the case when childDepth is 1.
	center = ( center>>1 )<<1;
	for( int i=0 ; i<2 ; i++ ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ )
			{
				int offset[] = { center+i , center+j , center+k };
				for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
						{
							int _offset[] = { x+center/2-OverlapEnd , y+center/2-OverlapEnd , z+center/2-OverlapEnd };
							stencils[i][j][k]( x , y , z ) = F.template integrate< Reverse >( integrator , _offset , offset );
						}
			}
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< bool Reverse , class _FEMSystemFunctor >
void SystemCoefficients< Degree1 , BType1 , Degree2 , BType2 >::SetCentralConstraintStencil( const _FEMSystemFunctor& F , const Integrator& integrator , Stencil< Point3D< double > , OverlapSize >& stencil  )
{
	int center = ( 1<<integrator.depth() )>>1;
	int offset[] = { center , center , center };
	for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
			{
				int _offset[] = { x+center-OverlapEnd , y+center-OverlapEnd , z+center-OverlapEnd };
				stencil( x , y , z ) = F.template integrate< Reverse >( integrator , _offset , offset );
			}
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< bool Reverse , class _FEMSystemFunctor >
void SystemCoefficients< Degree1 , BType1 , Degree2 , BType2 >::SetCentralConstraintStencils( const _FEMSystemFunctor& F , const ChildIntegrator& integrator , Stencil< Point3D< double > , OverlapSize > stencils[2][2][2] )
{
	int center = ( 1<<integrator.childDepth() )>>1;
	// [NOTE] We want the center to be at the first node of the brood
	// Which is not the case when childDepth is 1.
	center = ( center>>1 )<<1;
	for( int i=0 ; i<2 ; i++ ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ )
			{
				int offset[] = { center+i , center+j , center+k };
				for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
						{
							int _offset[] = { x+center/2-OverlapEnd , y+center/2-OverlapEnd , z+center/2-OverlapEnd };
							stencils[i][j][k]( x , y , z ) = F.template integrate< Reverse >( integrator , _offset , offset );
						}
			}
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< class _FEMSystemFunctor >
void SystemCoefficients< Degree1 , BType1 , Degree2 , BType2 >::SetCentralSystemStencil( const _FEMSystemFunctor& F , const Integrator& integrator , Stencil< double , OverlapSize >& stencil )
{
	int center = ( 1<<integrator.depth() )>>1;
	int offset[] = { center , center , center };
	for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
			{
				int _offset[] = { x+center-OverlapEnd , y+center-OverlapEnd , z+center-OverlapEnd };
				stencil( x , y , z ) = F.integrate( integrator , _offset , offset );
			}
}
template< int Degree1 , BoundaryType BType1 , int Degree2 , BoundaryType BType2 >
template< class _FEMSystemFunctor >
void SystemCoefficients< Degree1 , BType1 , Degree2 , BType2 >::SetCentralSystemStencils( const _FEMSystemFunctor& F , const ChildIntegrator& integrator , Stencil< double , OverlapSize > stencils[2][2][2] )
{
	int center = ( 1<<integrator.childDepth() )>>1;
	// [NOTE] We want the center to be at the first node of the brood
	// Which is not the case when childDepth is 1.
	center = ( center>>1 )<<1;
	for( int i=0 ; i<2 ; i++ ) for( int j=0 ; j<2 ; j++ ) for( int k=0 ; k<2 ; k++ )
			{
				int offset[] = { center+i , center+j , center+k };
				for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
						{
							int _offset[] = { x+center/2-OverlapEnd , y+center/2-OverlapEnd , z+center/2-OverlapEnd };
							stencils[i][j][k]( x , y , z ) = F.integrate( integrator , _offset , offset );
						}
			}
}

template< class Real >
template< int FEMDegree >
void Octree< Real >::_setMultiColorIndices( int start , int end , std::vector< std::vector< int > >& indices ) const
{
	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;

	const int modulus = OverlapRadius+1;
	indices.resize( modulus*modulus*modulus );
	int count[modulus*modulus*modulus];
	memset( count , 0 , sizeof(int)*modulus*modulus*modulus );
#pragma omp parallel for num_threads( threads )
	for( int i=start ; i<end ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			// [NOTE] We have to use the global offset so that it's positive
			int d , off[3];
			_sNodes.treeNodes[i]->depthAndOffset( d , off );
			int idx = (modulus*modulus) * ( off[2]%modulus ) + modulus * ( off[1]%modulus ) + ( off[0]%modulus );
#pragma omp atomic
			count[idx]++;
		}

	for( int i=0 ; i<modulus*modulus*modulus ; i++ ) indices[i].reserve( count[i] ) , count[i]=0;
	for( int i=start ; i<end ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			int d , off[3];
			_sNodes.treeNodes[i]->depthAndOffset( d , off );
			int idx = (modulus*modulus) * ( off[2]%modulus ) + modulus * ( off[1]%modulus ) + ( off[0]%modulus );
			indices[idx].push_back( i - start );
		}
}

template< class Real >
template< class C , int FEMDegree , BoundaryType BType >
void Octree< Real >::_downSample( LocalDepth highDepth , DenseNodeData< C , FEMDegree >& constraints ) const
{
	typedef typename TreeOctNode::NeighborKey< -BSplineSupportSizes< FEMDegree >::UpSampleStart , BSplineSupportSizes< FEMDegree >::UpSampleEnd > UpSampleKey;

	LocalDepth lowDepth = highDepth-1;
	if( lowDepth<0 ) return;

	typename BSplineEvaluationData< FEMDegree , BType >::UpSampleEvaluator upSampleEvaluator;
	BSplineEvaluationData< FEMDegree , BType >::SetUpSampleEvaluator( upSampleEvaluator , lowDepth );
	std::vector< UpSampleKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( lowDepth ) );

	Stencil< double , BSplineSupportSizes< FEMDegree >::UpSampleSize > upSampleStencil;
	int lowCenter = ( 1<<lowDepth )>>1;
	for( int i=0 ; i<BSplineSupportSizes< FEMDegree >::UpSampleSize ; i++ ) for( int j=0 ; j<BSplineSupportSizes< FEMDegree >::UpSampleSize ; j++ ) for( int k=0 ; k<BSplineSupportSizes< FEMDegree >::UpSampleSize ; k++ )
				upSampleStencil( i , j , k ) =
						upSampleEvaluator.value( lowCenter , 2*lowCenter + i + BSplineSupportSizes< FEMDegree >::UpSampleStart ) *
						upSampleEvaluator.value( lowCenter , 2*lowCenter + j + BSplineSupportSizes< FEMDegree >::UpSampleStart ) *
						upSampleEvaluator.value( lowCenter , 2*lowCenter + k + BSplineSupportSizes< FEMDegree >::UpSampleStart );

	// Iterate over all (valid) parent nodes
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(lowDepth) ; i<_sNodesEnd(lowDepth) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			TreeOctNode* pNode = _sNodes.treeNodes[i];

			UpSampleKey& neighborKey = neighborKeys[ get_thread_num() ];
			LocalDepth d ; LocalOffset off;
			_localDepthAndOffset( pNode , d , off );

			neighborKey.template getNeighbors< false >( pNode );

			// Get the child neighbors
			typename TreeOctNode::Neighbors< BSplineSupportSizes< FEMDegree >::UpSampleSize > neighbors;
			neighborKey.template getChildNeighbors< false >( 0 , _localToGlobal( d ) , neighbors );

			C& coarseConstraint = constraints[i];

			// Want to make sure test if contained children are interior.
			// This is more conservative because we are test that overlapping children are interior
			bool isInterior = _isInteriorlyOverlapped< FEMDegree , FEMDegree >( pNode );
			if( isInterior )
			{
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::UpSampleSize ; ii++ ) for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::UpSampleSize ; jj++ ) for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::UpSampleSize ; kk++ )
						{
							const TreeOctNode* cNode = neighbors.neighbors[ii][jj][kk];
							if( IsActiveNode( cNode ) ) coarseConstraint += (C)( constraints[ cNode->nodeData.nodeIndex ] * upSampleStencil( ii , jj , kk ) );
						}
			}
			else
			{
				double upSampleValues[3][ BSplineSupportSizes< FEMDegree >::UpSampleSize ];
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::UpSampleSize ; ii++ )
				{
					upSampleValues[0][ii] = upSampleEvaluator.value( off[0] , 2*off[0] + ii + BSplineSupportSizes< FEMDegree >::UpSampleStart );
					upSampleValues[1][ii] = upSampleEvaluator.value( off[1] , 2*off[1] + ii + BSplineSupportSizes< FEMDegree >::UpSampleStart );
					upSampleValues[2][ii] = upSampleEvaluator.value( off[2] , 2*off[2] + ii + BSplineSupportSizes< FEMDegree >::UpSampleStart );
				}
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::UpSampleSize ; ii++ ) for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::UpSampleSize ; jj++ )
					{
						double dxy = upSampleValues[0][ii] * upSampleValues[1][jj];
						for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::UpSampleSize ; kk++ )
						{
							const TreeOctNode* cNode = neighbors.neighbors[ii][jj][kk];
							if( _isValidFEMNode( cNode ) ) coarseConstraint += (C)( constraints[ cNode->nodeData.nodeIndex ] * dxy * upSampleValues[2][kk] );
						}
					}
			}
		}
}
template< class Real >
template< class C , int FEMDegree , BoundaryType BType >
void Octree< Real >::_upSample( LocalDepth highDepth , DenseNodeData< C , FEMDegree >& coefficients ) const
{
	static const int  LeftDownSampleRadius = -( ( BSplineSupportSizes< FEMDegree >::DownSample0Start < BSplineSupportSizes< FEMDegree >::DownSample1Start ) ? BSplineSupportSizes< FEMDegree >::DownSample0Start : BSplineSupportSizes< FEMDegree >::DownSample1Start );
	static const int RightDownSampleRadius =  ( ( BSplineSupportSizes< FEMDegree >::DownSample0End   > BSplineSupportSizes< FEMDegree >::DownSample1End   ) ? BSplineSupportSizes< FEMDegree >::DownSample0End   : BSplineSupportSizes< FEMDegree >::DownSample1End   );
	typedef TreeOctNode::NeighborKey< LeftDownSampleRadius , RightDownSampleRadius > DownSampleKey;

	LocalDepth lowDepth = highDepth-1;
	if( lowDepth<0 ) return;

	typename BSplineEvaluationData< FEMDegree , BType >::UpSampleEvaluator upSampleEvaluator;
	BSplineEvaluationData< FEMDegree , BType >::SetUpSampleEvaluator( upSampleEvaluator , lowDepth );
	std::vector< DownSampleKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( lowDepth ) );

	static const int DownSampleSize = BSplineSupportSizes< FEMDegree >::DownSample0Size > BSplineSupportSizes< FEMDegree >::DownSample1Size ? BSplineSupportSizes< FEMDegree >::DownSample0Size : BSplineSupportSizes< FEMDegree >::DownSample1Size;
	Stencil< double , DownSampleSize > downSampleStencils[ Cube::CORNERS ];
	int lowCenter = ( 1<<lowDepth )>>1;
	for( int c=0 ; c<Cube::CORNERS ; c++ )
	{
		int cx , cy , cz;
		Cube::FactorCornerIndex( c , cx , cy , cz );
		for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ )
			for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; jj++ )
				for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; kk++ )
					downSampleStencils[c]( ii , jj , kk ) =
							upSampleEvaluator.value( lowCenter + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] , 2*lowCenter + cx ) *
							upSampleEvaluator.value( lowCenter + jj + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] , 2*lowCenter + cy ) *
							upSampleEvaluator.value( lowCenter + kk + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] , 2*lowCenter + cz ) ;
	}

	// For Dirichlet constraints, can't get to all children from parents because boundary nodes are invalid
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(highDepth) ; i<_sNodesEnd(highDepth) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			TreeOctNode *cNode = _sNodes.treeNodes[i] , *pNode = cNode->parent;
			int c = (int)( cNode-pNode->children );

			DownSampleKey& neighborKey = neighborKeys[ get_thread_num() ];
			LocalDepth d ; LocalOffset off;
			_localDepthAndOffset( pNode , d , off );
			typename TreeOctNode::Neighbors< LeftDownSampleRadius + RightDownSampleRadius + 1 >& neighbors = neighborKey.template getNeighbors< false >( pNode );

			// Want to make sure test if contained children are interior.
			// This is more conservative because we are test that overlapping children are interior
			bool isInterior = _isInteriorlyOverlapped< FEMDegree , FEMDegree >( pNode );

			C& fineCoefficient = coefficients[ cNode->nodeData.nodeIndex ];

			int cx , cy , cz;
			Cube::FactorCornerIndex( c , cx , cy , cz );

			if( isInterior )
			{
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ ) for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; jj++ )
					{
						int _ii = ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] + LeftDownSampleRadius;
						int _jj = jj + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] + LeftDownSampleRadius;
						for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; kk++ )
						{
							int _kk = kk + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] + LeftDownSampleRadius;
							const TreeOctNode* _pNode = neighbors.neighbors[_ii][_jj][_kk];
							if( _pNode ) fineCoefficient += (C)( coefficients[ _pNode->nodeData.nodeIndex ] * downSampleStencils[c]( ii , jj , kk ) );
						}
					}
			}
			else
			{
				double downSampleValues[3][ BSplineSupportSizes< FEMDegree >::DownSample0Size > BSplineSupportSizes< FEMDegree >::DownSample1Size ? BSplineSupportSizes< FEMDegree >::DownSample0Size : BSplineSupportSizes< FEMDegree >::DownSample1Size ];
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ ) downSampleValues[0][ii] = upSampleEvaluator.value( off[0] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] , 2*off[0] + cx );
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; ii++ ) downSampleValues[1][ii] = upSampleEvaluator.value( off[1] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] , 2*off[1] + cy );
				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; ii++ ) downSampleValues[2][ii] = upSampleEvaluator.value( off[2] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] , 2*off[2] + cz );

				for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ ) for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; jj++ )
					{
						double dxy = downSampleValues[0][ii] * downSampleValues[1][jj];
						int _ii = ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] + LeftDownSampleRadius;
						int _jj = jj + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] + LeftDownSampleRadius;
						for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; kk++ )
						{
							int _kk = kk + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] + LeftDownSampleRadius;
							const TreeOctNode* _pNode = neighbors.neighbors[_ii][_jj][_kk];
							if( _isValidFEMNode( _pNode ) ) fineCoefficient += (C)( coefficients[ _pNode->nodeData.nodeIndex ] * dxy * downSampleValues[2][kk] );
						}
					}
			}
		}
}

template< class Real >
template< class C , int FEMDegree , BoundaryType BType >
void Octree< Real >::_UpSample( LocalDepth highDepth , ConstPointer( C ) lowCoefficients , Pointer( C ) highCoefficients , int threads )
{
	static const int  LeftDownSampleRadius = -( ( BSplineSupportSizes< FEMDegree >::DownSample0Start < BSplineSupportSizes< FEMDegree >::DownSample1Start ) ? BSplineSupportSizes< FEMDegree >::DownSample0Start : BSplineSupportSizes< FEMDegree >::DownSample1Start );
	static const int RightDownSampleRadius =  ( ( BSplineSupportSizes< FEMDegree >::DownSample0End   > BSplineSupportSizes< FEMDegree >::DownSample1End   ) ? BSplineSupportSizes< FEMDegree >::DownSample0End   : BSplineSupportSizes< FEMDegree >::DownSample1End   );
	typedef TreeOctNode::NeighborKey< LeftDownSampleRadius , RightDownSampleRadius > DownSampleKey;

	LocalDepth lowDepth = highDepth - 1;
	if( lowDepth<0 ) return;

	typename BSplineEvaluationData< FEMDegree , BType >::UpSampleEvaluator upSampleEvaluator;
	BSplineEvaluationData< FEMDegree , BType >::SetUpSampleEvaluator( upSampleEvaluator , lowDepth );
	std::vector< DownSampleKey > neighborKeys( std::max< int >( 1 , threads ) );

	static const int DownSampleSize = BSplineSupportSizes< FEMDegree >::DownSample0Size > BSplineSupportSizes< FEMDegree >::DownSample1Size ? BSplineSupportSizes< FEMDegree >::DownSample0Size : BSplineSupportSizes< FEMDegree >::DownSample1Size;
	Stencil< double , DownSampleSize > downSampleStencils[ Cube::CORNERS ];
	int lowCenter = ( 1<<lowDepth )>>1;
	for( int c=0 ; c<Cube::CORNERS ; c++ )
	{
		int cx , cy , cz;
		Cube::FactorCornerIndex( c , cx , cy , cz );
		static const int DownSampleSize = BSplineSupportSizes< FEMDegree >::DownSample0Size > BSplineSupportSizes< FEMDegree >::DownSample1Size ? BSplineSupportSizes< FEMDegree >::DownSample0Size : BSplineSupportSizes< FEMDegree >::DownSample1Size;
		for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ )
			for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; jj++ )
				for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; kk++ )
					downSampleStencils[c]( ii , jj , kk ) =
							upSampleEvaluator.value( lowCenter + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] , 2*lowCenter + cx ) *
							upSampleEvaluator.value( lowCenter + jj + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] , 2*lowCenter + cy ) *
							upSampleEvaluator.value( lowCenter + kk + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] , 2*lowCenter + cz ) ;
	}
	int  lowBegin = _BSplineBegin< FEMDegree , BType >(  lowDepth ) ,  lowEnd = _BSplineEnd< FEMDegree , BType >(  lowDepth );
	int highBegin = _BSplineBegin< FEMDegree , BType >( highDepth ) , highEnd = _BSplineEnd< FEMDegree , BType >( highDepth );
	int lowDim = lowEnd - lowBegin , highDim = highEnd - highBegin;
	// Iterate over all child nodes. (This is required since there can be child nodes whose parent is inactive.)
#pragma omp parallel for num_threads( threads )
	for( int k=0 ; k<highDim ; k++ ) for( int j=0 ; j<highDim ; j++ ) for( int i=0 ; i<highDim ; i++ )
			{
				DownSampleKey& neighborKey = neighborKeys[ get_thread_num() ];
				LocalOffset off , _off;
				off[0] = i + highBegin , off[1] = j + highBegin , off[2] = k + highBegin;
				int highIdx = i + j * highDim  + k * highDim * highDim;
				_off[0] = off[0]>>1 , _off[1] = off[1]>>1 , _off[2] = off[2]>>1;

				// Want to make sure test if contained children are interior.
				// This is more conservative because we are test that overlapping children are interior
				bool isInterior = _IsInteriorlyOverlapped< FEMDegree , FEMDegree >( lowDepth , _off );
				int cx = off[0]&1 , cy = off[1]&1 , cz = off[2]&1;
				int c = Cube::CornerIndex( cx , cy , cz );

				C& highCoefficient = highCoefficients[ highIdx ];

				if( isInterior )
				{
					for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ ) for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; jj++ )
						{
							int _i = _off[0] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] - lowBegin;
							int _j = _off[1] + jj + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] - lowBegin;
							for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; kk++ )
							{
								int _k = _off[2] + kk + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] - lowBegin;
								highCoefficient += (C)( lowCoefficients[ _i + _j*lowDim  + _k*lowDim*lowDim ] * downSampleStencils[c]( ii , jj , kk ) );
							}
						}
				}
				else
				{
					double downSampleValues[3][ BSplineSupportSizes< FEMDegree >::DownSample0Size > BSplineSupportSizes< FEMDegree >::DownSample1Size ? BSplineSupportSizes< FEMDegree >::DownSample0Size : BSplineSupportSizes< FEMDegree >::DownSample1Size ];

					for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ ) downSampleValues[0][ii] = upSampleEvaluator.value( _off[0] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] , off[0] );
					for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; ii++ ) downSampleValues[1][ii] = upSampleEvaluator.value( _off[1] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] , off[1] );
					for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; ii++ ) downSampleValues[2][ii] = upSampleEvaluator.value( _off[2] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] , off[2] );

					for( int ii=0 ; ii<BSplineSupportSizes< FEMDegree >::DownSampleSize[cx] ; ii++ ) for( int jj=0 ; jj<BSplineSupportSizes< FEMDegree >::DownSampleSize[cy] ; jj++ )
						{
							double dxy = downSampleValues[0][ii] * downSampleValues[1][jj];
							int _i = _off[0] + ii + BSplineSupportSizes< FEMDegree >::DownSampleStart[cx] - lowBegin;
							int _j = _off[1] + jj + BSplineSupportSizes< FEMDegree >::DownSampleStart[cy] - lowBegin;
							if( _i>=0 && _i<lowDim && _j>=0 && _j<lowDim )
								for( int kk=0 ; kk<BSplineSupportSizes< FEMDegree >::DownSampleSize[cz] ; kk++ )
								{
									int _k = _off[2] + kk + BSplineSupportSizes< FEMDegree >::DownSampleStart[cz] - lowBegin;
									if( _k>=0 && _k<lowDim ) highCoefficient += (C)( lowCoefficients[ _i + _j*lowDim  + _k*lowDim*lowDim ] * dxy * downSampleValues[2][kk] );
								}
						}
				}
			}
}

template< class Real >
template< class C , int FEMDegree , BoundaryType BType >
DenseNodeData< C , FEMDegree > Octree< Real >::coarseCoefficients( const DenseNodeData< C , FEMDegree >& coefficients ) const
{
	DenseNodeData< Real , FEMDegree > coarseCoefficients( _sNodesEnd(_maxDepth-1) );
	memset( &coarseCoefficients[0] , 0 , sizeof(Real)*_sNodesEnd(_maxDepth-1) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(0) ; i<_sNodesEnd(_maxDepth-1) ; i++ ) coarseCoefficients[i] = coefficients[i];
	for( LocalDepth d=1 ; d<_maxDepth ; d++ ) _upSample< C , FEMDegree , BType >( d , coarseCoefficients );
	return coarseCoefficients;
}
template< class Real >
template< class C , int FEMDegree , BoundaryType BType >
DenseNodeData< C , FEMDegree > Octree< Real >::coarseCoefficients( const SparseNodeData< C , FEMDegree >& coefficients ) const
{
	DenseNodeData< Real , FEMDegree > coarseCoefficients( _sNodesEnd(_maxDepth-1) );
	memset( &coarseCoefficients[0] , 0 , sizeof(Real)*_sNodesEnd(_maxDepth-1) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(0) ; i<_sNodesEnd(_maxDepth-1) ; i++ )
	{
		const C* c = coefficients( _sNodes.treeNodes[i] );
		if( c ) coarseCoefficients[i] = *c;
	}
	for( LocalDepth d=1 ; d<_maxDepth ; d++ ) _upSample< C , FEMDegree , BType >( d , coarseCoefficients );
	return coarseCoefficients;
}

template< class Real >
template< int FEMDegree , BoundaryType BType >
Real Octree< Real >::_coarserFunctionValue( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* pointNode , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& upSampledCoefficients ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int  LeftPointSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;

	double pointValue = 0;
	LocalDepth depth = _localDepth( pointNode );
	if( depth<0 ) return (Real)0.;

	// Iterate over all basis functions that overlap the point at the coarser resolution
	{
		const typename TreeOctNode::Neighbors< SupportSize >& neighbors = neighborKey.neighbors[ _localToGlobal( depth-1 ) ];
		LocalDepth _d ; LocalOffset _off;
		_localDepthAndOffset( pointNode->parent , _d , _off );
		int fStart , fEnd;
		BSplineData< FEMDegree , BType >::FunctionSpan( _d , fStart , fEnd );

		double pointValues[ DIMENSION ][SupportSize];
		memset( pointValues , 0 , sizeof(double) * DIMENSION * SupportSize );

		for( int dd=0 ; dd<DIMENSION ; dd++ ) for( int i=-LeftPointSupportRadius ; i<=RightPointSupportRadius ; i++ )
			{
				int fIdx = BSplineData< FEMDegree , BType >::FunctionIndex( _d , _off[dd]+i );
				if( fIdx>=fStart && fIdx<fEnd ) pointValues[dd][i+LeftPointSupportRadius] = bsData.baseBSplines[ fIdx ][LeftSupportRadius-i]( p[dd] );
			}

		for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
			{
				double xyValue = pointValues[0][j] * pointValues[1][k];
				double _pointValue = 0;
				for( int l=0 ; l<SupportSize ; l++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[j][k][l];
					if( _isValidFEMNode( _node ) ) _pointValue += pointValues[2][l] * double( upSampledCoefficients[_node->nodeData.nodeIndex] );
				}
				pointValue += _pointValue * xyValue;
			}
	}
	return Real( pointValue );
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
Point3D< Real > Octree< Real >::_coarserFunctionGradient( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* pointNode , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& upSampledCoefficients ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int  LeftPointSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;

	Point3D< double > pointGradient;
	LocalDepth depth = _localDepth( pointNode );
	if( depth<=0 ) return Real(0.);

	// Iterate over all basis functions that overlap the point at the coarser resolution
	{
		const typename TreeOctNode::Neighbors< SupportSize >& neighbors = neighborKey.neighbors[ _localToGlobal( depth-1 ) ];
		LocalDepth _d ; LocalOffset _off;
		_localDepthAndOffset( pointNode->parent , _d , _off );
		int fStart , fEnd;
		BSplineData< FEMDegree , BType >::FunctionSpan( _d , fStart , fEnd );

		double _pointValues[ DIMENSION ][SupportSize] , dPointValues[ DIMENSION ][SupportSize];
		memset( _pointValues , 0 , sizeof(double) * DIMENSION * SupportSize );
		memset( dPointValues , 0 , sizeof(double) * DIMENSION * SupportSize );

		for( int dd=0 ; dd<DIMENSION ; dd++ ) for( int i=-LeftPointSupportRadius ; i<=RightPointSupportRadius ; i++ )
			{
				int fIdx = BSplineData< FEMDegree , BType >::FunctionIndex( _d , _off[dd]+i );
				if( fIdx>=fStart && fIdx<fEnd )
				{
					_pointValues[dd][i+LeftPointSupportRadius] = bsData.baseBSplines[ fIdx ][LeftSupportRadius-i]( p[dd] );
					dPointValues[dd][i+LeftPointSupportRadius] = bsData.dBaseBSplines[ fIdx ][LeftSupportRadius-i]( p[dd] );
				}
			}

		for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
			{
				double _x_yValue = _pointValues[0][j] * _pointValues[1][k];
				double dx_yValue = dPointValues[0][j] * _pointValues[1][k];
				double _xdyValue = _pointValues[0][j] * dPointValues[1][k];
				double __pointValue = 0 , _dPointValue = 0;
				for( int l=0 ; l<SupportSize ; l++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[j][k][l];
					if( _isValidFEMNode( _node ) )
					{
						__pointValue += _pointValues[2][l] * double( upSampledCoefficients[_node->nodeData.nodeIndex] );
						_dPointValue += dPointValues[2][l] * double( upSampledCoefficients[_node->nodeData.nodeIndex] );
					}
				}

				pointGradient += Point3D< double >( __pointValue * dx_yValue , __pointValue * _xdyValue , _dPointValue * _x_yValue );
			}
	}
	return Point3D< Real >( pointGradient );
}

template< class Real >
template< int FEMDegree , BoundaryType BType >
Real Octree< Real >::_finerFunctionValue( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* pointNode , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& finerCoefficients ) const
{
	typename TreeOctNode::Neighbors< BSplineSupportSizes< FEMDegree >::SupportSize > childNeighbors;
	static const int  LeftPointSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int  LeftSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;

	double pointValue = 0;
	LocalDepth depth = _localDepth( pointNode );
	neighborKey.template getChildNeighbors< false >( _childIndex( pointNode , p ) , _localToGlobal( depth ) , childNeighbors );
	for( int j=-LeftPointSupportRadius ; j<=RightPointSupportRadius ; j++ )
		for( int k=-LeftPointSupportRadius ; k<=RightPointSupportRadius ; k++ )
			for( int l=-LeftPointSupportRadius ; l<=RightPointSupportRadius ; l++ )
			{
				const TreeOctNode* _node = childNeighbors.neighbors[j+LeftPointSupportRadius][k+LeftPointSupportRadius][l+LeftPointSupportRadius];
				if( _isValidFEMNode( _node ) )
				{
					int fIdx[3];
					functionIndex< FEMDegree , BType >( _node , fIdx );
					pointValue +=
							bsData.baseBSplines[ fIdx[0] ][LeftSupportRadius-j]( p[0] ) *
							bsData.baseBSplines[ fIdx[1] ][LeftSupportRadius-k]( p[1] ) *
							bsData.baseBSplines[ fIdx[2] ][LeftSupportRadius-l]( p[2] ) *
							double( finerCoefficients[ _node->nodeData.nodeIndex ] );
				}
			}
	return Real( pointValue );
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
Point3D< Real > Octree< Real >::_finerFunctionGradient( Point3D< Real > p , const PointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* pointNode , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& finerCoefficients ) const
{
	typename TreeOctNode::Neighbors< BSplineSupportSizes< FEMDegree >::SupportSize > childNeighbors;
	static const int  LeftPointSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int  LeftSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;

	Point3D< double > pointGradient = 0;
	LocalDepth depth = _localDepth( pointNode );
	neighborKey.template getChildNeighbors< false >( _childIndex( pointNode , p ) , _localToGlobal( depth ) , childNeighbors );
	for( int j=-LeftPointSupportRadius ; j<=RightPointSupportRadius ; j++ )
		for( int k=-LeftPointSupportRadius ; k<=RightPointSupportRadius ; k++ )
			for( int l=-LeftPointSupportRadius ; l<=RightPointSupportRadius ; l++ )
			{
				const TreeOctNode* _node = childNeighbors.neighbors[j+LeftPointSupportRadius][k+LeftPointSupportRadius][l+LeftPointSupportRadius];
				if( _isValidFEMNode( _node ) )
				{
					int fIdx[3];
					functionIndex< FEMDegree , BType >( _node , fIdx );
					double  x = bsData. baseBSplines[ fIdx[0] ][LeftSupportRadius-j]( p[0] ) ,  y = bsData. baseBSplines[ fIdx[1] ][LeftSupportRadius-k]( p[1] ) ,  z = bsData. baseBSplines[ fIdx[2] ][LeftSupportRadius-l]( p[2] );
					double dx = bsData.dBaseBSplines[ fIdx[0] ][LeftSupportRadius-j]( p[0] ) , dy = bsData.dBaseBSplines[ fIdx[1] ][LeftSupportRadius-k]( p[1] ) , dz = bsData.dBaseBSplines[ fIdx[2] ][LeftSupportRadius-l]( p[2] );
					pointGradient += Point3D< double >( dx * y * z , x * dy * z , x * y * dz ) * (double)( finerCoefficients[ _node->nodeData.nodeIndex ] );
				}
			}
	return Point3D< Real >( pointGradient );
}

template< class Real >
template< int FEMDegree , BoundaryType BType , bool HasGradients >
void Octree< Real >::_setPointValuesFromCoarser( InterpolationInfo< HasGradients >& interpolationInfo , LocalDepth highDepth , const BSplineData< FEMDegree , BType >& bsData , const DenseNodeData< Real , FEMDegree >& upSampledCoefficients )
{
	LocalDepth lowDepth = highDepth-1;
	if( lowDepth<0 ) return;
	std::vector< PointSupportKey< FEMDegree > > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( lowDepth ) );

#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(highDepth) ; i<_sNodesEnd(highDepth) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			PointSupportKey< FEMDegree >& neighborKey = neighborKeys[ get_thread_num() ];
			PointData< Real , HasGradients >* pData = interpolationInfo( _sNodes.treeNodes[i] );
			if( pData )
			{
				neighborKey.template getNeighbors< false >( _sNodes.treeNodes[i]->parent );
#if POINT_DATA_RES
				for( int c=0 ; c<PointData< Real , HasGradients >::SAMPLES ; c++ ) if( (*pData)[c].weight )
				_ConstraintCalculator_< Real , FEMDegree , HasGradients >::_CalculateCoarser_
				(
					c , *pData ,
					_coarserFunctionValue( (*pData)[c].position , neighborKey , _sNodes.treeNodes[i] , bsData , upSampledCoefficients ) ,
					HasGradients ? _coarserFunctionGradient( (*pData)[c].position , neighborKey , _sNodes.treeNodes[i] , bsData , upSampledCoefficients ) : Point3D< Real >() ,
					interpolationInfo.valueWeight , interpolationInfo.gradientWeight
				);
#else // !POINT_DATA_RES
				_ConstraintCalculator_< Real , FEMDegree , HasGradients >::_CalculateCoarser_
						(
								*pData ,
								_coarserFunctionValue( pData->position , neighborKey , _sNodes.treeNodes[i] , bsData , upSampledCoefficients ) ,
								HasGradients ? _coarserFunctionGradient( pData->position , neighborKey , _sNodes.treeNodes[i] , bsData , upSampledCoefficients ) : Point3D< Real >() ,
								interpolationInfo.valueWeight , interpolationInfo.gradientWeight
						);
#endif // POINT_DATA_RES
			}
		}
}

template< class Real >
template< int FEMDegree , BoundaryType BType , bool HasGradients >
void Octree< Real >::_updateCumulativeInterpolationConstraintsFromFiner( const InterpolationInfo< HasGradients >& interpolationInfo , const BSplineData< FEMDegree , BType >& bsData , LocalDepth highDepth , const DenseNodeData< Real , FEMDegree >& finerCoefficients , DenseNodeData< Real , FEMDegree >& coarserConstraints ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftPointSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int  LeftSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;

	// Note: We can't iterate over the finer point nodes as the point weights might be
	// scaled incorrectly, due to the adaptive exponent. So instead, we will iterate
	// over the coarser nodes and evaluate the finer solution at the associated points.
	LocalDepth  lowDepth = highDepth-1;
	if( lowDepth<0 ) return;
	size_t start = _sNodesBegin(lowDepth) , end = _sNodesEnd(lowDepth);
	std::vector< PointSupportKey< FEMDegree > > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( lowDepth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(lowDepth) ; i<_sNodesEnd(lowDepth) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			PointSupportKey< FEMDegree >& neighborKey = neighborKeys[ get_thread_num() ];
			const PointData< Real , HasGradients >* pData = interpolationInfo( _sNodes.treeNodes[i] );
			if( pData )
			{
				typename TreeOctNode::Neighbors< SupportSize >& neighbors = neighborKey.template getNeighbors< false >( _sNodes.treeNodes[i] );
				// evaluate the solution @( depth ) at the current point @( depth-1 )
#if POINT_DATA_RES
				for( int c=0 ; c<PointData< Real , HasGradients >::SAMPLES ; c++ ) if( (*pData)[c].weight )
#endif // POINT_DATA_RES
				{
#if POINT_DATA_RES
					Real finerPointDValue = _finerFunctionValue( (*pData)[c].position , neighborKey , _sNodes.treeNodes[i] , bsData , finerCoefficients ) * interpolationInfo.valueWeight * (*pData)[c].weight;
				Point3D< Real > finerPointDGradient = HasGradients ? _finerFunctionGradient( (*pData)[c].position , neighborKey , _sNodes.treeNodes[i] , bsData , finerCoefficients ) * interpolationInfo.gradientWeight * (*pData)[c].weight : Point3D< Real >();
				Point3D< Real > p = (*pData)[c].position;
#else // !POINT_DATA_RES
					Real finerPointDValue = _finerFunctionValue( pData->position , neighborKey , _sNodes.treeNodes[i] , bsData , finerCoefficients ) * interpolationInfo.valueWeight * pData->weight;
					Point3D< Real > finerPointDGradient = HasGradients ? _finerFunctionGradient( pData->position , neighborKey , _sNodes.treeNodes[i] , bsData , finerCoefficients ) * interpolationInfo.gradientWeight * pData->weight : Point3D< Real >();
					Point3D< Real > p = pData->position;
#endif // POINT_DATA_RES
					// Update constraints for all nodes @( depth-1 ) that overlap the point
					int idx[3];
					functionIndex< FEMDegree , BType >( _sNodes.treeNodes[i] , idx );
					for( int x=-LeftPointSupportRadius ; x<=RightPointSupportRadius ; x++ ) for( int y=-LeftPointSupportRadius ; y<=RightPointSupportRadius ; y++ ) for( int z=-LeftPointSupportRadius ; z<=RightPointSupportRadius ; z++ )
							{
								const TreeOctNode* _node = neighbors.neighbors[x+LeftPointSupportRadius][y+LeftPointSupportRadius][z+LeftPointSupportRadius];
								if( _isValidFEMNode( _node ) )
								{
									double px = bsData.baseBSplines[idx[0]+x][LeftSupportRadius-x]( p[0] ) , py = bsData.baseBSplines[idx[1]+y][LeftSupportRadius-y]( p[1] ) , pz = bsData.baseBSplines[idx[2]+z][LeftSupportRadius-z]( p[2] );
#pragma omp atomic
									coarserConstraints[ _node->nodeData.nodeIndex ] += (Real)( px * py * pz * finerPointDValue );
									if( HasGradients )
									{
										double dpx = bsData.dBaseBSplines[idx[0]+x][LeftSupportRadius-x]( p[0] ) , dpy = bsData.dBaseBSplines[idx[1]+y][LeftSupportRadius-y]( p[1] ) , dpz = bsData.dBaseBSplines[idx[2]+z][LeftSupportRadius-z]( p[2] );
#pragma omp atomic
										coarserConstraints[ _node->nodeData.nodeIndex ] += Point3D< Real >::Dot( finerPointDGradient , Point3D< Real >( dpx * py * pz , px * dpy * pz , px * py * dpz ) );
									}
								}
							}
				}
			}
		}
}

template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
int Octree< Real >::_setMatrixRow( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& neighbors , Pointer( MatrixEntry< Real > ) row , int offset , const typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& integrator , const Stencil< double , BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& stencil , const BSplineData< FEMDegree , BType >& bsData ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;
	static const int OverlapSize   =   BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;
	static const int LeftSupportRadius  = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int LeftPointSupportRadius  = BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;

	bool hasYZPoints[SupportSize] , hasZPoints[SupportSize][SupportSize];
	Real diagonal = 0;
	// Given a node:
	// -- for each node in its support:
	// ---- if the supporting node contains a point:
	// ------ evaluate the x, y, and z B-splines of the nodes supporting the point
	// splineValues \in [-LeftSupportRadius,RightSupportRadius] x [-LeftSupportRadius,RightSupportRadius] x [-LeftSupportRadius,RightSupportRadius] x [0,Dimension) x [-LeftPointSupportRadius,RightPointSupportRadius]
#if POINT_DATA_RES
	Real _splineValues[PointData< Real , HasGradients >::SAMPLES][SupportSize][SupportSize][SupportSize][DIMENSION][SupportSize];
	Real wSplineValues[PointData< Real , HasGradients >::SAMPLES][SupportSize][SupportSize][SupportSize][DIMENSION][SupportSize];
	Real dSplineValues[PointData< Real , HasGradients >::SAMPLES][SupportSize][SupportSize][SupportSize][DIMENSION][SupportSize];
	memset( _splineValues , 0 , sizeof( Real ) * PointData< Real , HasGradients >::SAMPLES * SupportSize * SupportSize * SupportSize * DIMENSION *SupportSize );
	memset( wSplineValues , 0 , sizeof( Real ) * PointData< Real , HasGradients >::SAMPLES * SupportSize * SupportSize * SupportSize * DIMENSION *SupportSize );
	memset( dSplineValues , 0 , sizeof( Real ) * PointData< Real , HasGradients >::SAMPLES * SupportSize * SupportSize * SupportSize * DIMENSION *SupportSize );
#else // !POINT_DATA_RES
	Real _splineValues[SupportSize][SupportSize][SupportSize][DIMENSION][SupportSize];
	Real wSplineValues[SupportSize][SupportSize][SupportSize][DIMENSION][SupportSize];
	Real dSplineValues[SupportSize][SupportSize][SupportSize][DIMENSION][SupportSize];
	memset( _splineValues , 0 , sizeof( Real ) * SupportSize * SupportSize * SupportSize * DIMENSION *SupportSize );
	memset( wSplineValues , 0 , sizeof( Real ) * SupportSize * SupportSize * SupportSize * DIMENSION *SupportSize );
	memset( dSplineValues , 0 , sizeof( Real ) * SupportSize * SupportSize * SupportSize * DIMENSION *SupportSize );
#endif // NEW_POINT_DATA

	int count = 0;
	const TreeOctNode* node = neighbors.neighbors[OverlapRadius][OverlapRadius][OverlapRadius];
	LocalDepth d ; LocalOffset off;
	_localDepthAndOffset( node , d , off );
	int fStart , fEnd;
	BSplineData< FEMDegree , BType >::FunctionSpan( d , fStart , fEnd );
	bool isInterior = _isInteriorlyOverlapped< FEMDegree , FEMDegree >( node );

	if( interpolationInfo )
	{
		// Iterate over all neighboring nodes that may have a constraining point
		// -- For each one, compute the values of the spline functions supported on the point
		for( int j=0 ; j<SupportSize ; j++ )
		{
			hasYZPoints[j] = false;
			for( int k=0 ; k<SupportSize ; k++ ) hasZPoints[j][k] = false;
		}
		for( int j=-LeftSupportRadius , jj=0 ; j<=RightSupportRadius ; j++ , jj++ )
			for( int k=-LeftSupportRadius , kk=0 ; k<=RightSupportRadius ; k++ , kk++ )
				for( int l=-LeftSupportRadius , ll=0 ; l<=RightSupportRadius ; l++ , ll++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[OverlapRadius+j][OverlapRadius+k][OverlapRadius+l];
					if( _isValidSpaceNode( _node ) && (*interpolationInfo)( _node ) )
					{
						int pOff[] = { off[0]+j , off[1]+k , off[2]+l };
						hasYZPoints[jj] = hasZPoints[jj][kk] = true;
						const PointData< Real , HasGradients >& pData = *( (*interpolationInfo)( _node ) );

#if POINT_DATA_RES
						for( int c=0 ; c<PointData< Real , HasGradients >::SAMPLES ; c++ ) if( pData[c].weight )
#endif // POINT_DATA_RES
						{
#if POINT_DATA_RES
							Real (*__splineValues)[SupportSize] = _splineValues[c][jj][kk][ll];
							Real (*_wSplineValues)[SupportSize] = wSplineValues[c][jj][kk][ll];
							Real (*_dSplineValues)[SupportSize] = dSplineValues[c][jj][kk][ll];
							Real weight = pData[c].weight;
							Point3D< Real > p = pData[c].position;
#else // !POINT_DATA_RES
							Real (*__splineValues)[SupportSize] = _splineValues[jj][kk][ll];
							Real (*_wSplineValues)[SupportSize] = wSplineValues[jj][kk][ll];
							Real (*_dSplineValues)[SupportSize] = dSplineValues[jj][kk][ll];
							Real weight = pData.weight;
							Point3D< Real > p = pData.position;
#endif // POINT_DATA_RES

							// evaluate the point p at all the nodes whose functions have it in their support
							for( int s=-LeftPointSupportRadius ; s<=RightPointSupportRadius ; s++ ) for( int dd=0 ; dd<DIMENSION ; dd++ )
								{
									int fIdx = BSplineData< FEMDegree , BType >::FunctionIndex( d , pOff[dd]+s );
									if( fIdx>=fStart && fIdx<fEnd )
									{
										_wSplineValues[dd][ s+LeftPointSupportRadius ] = __splineValues[dd][ s+LeftPointSupportRadius ] = Real( bsData.baseBSplines[ fIdx ][ -s+LeftSupportRadius ]( p[dd] ) );
										if( HasGradients ) _dSplineValues[dd][ s+LeftPointSupportRadius ] = Real( bsData.dBaseBSplines[ fIdx ][ -s+LeftSupportRadius ]( p[dd] ) );
									}
								}
							// The value of the function of the node that we started with
							Real value = __splineValues[0][-j+LeftPointSupportRadius] * __splineValues[1][-k+LeftPointSupportRadius] * __splineValues[2][-l+LeftPointSupportRadius];
							Real weightedValue = value * interpolationInfo->valueWeight * weight;
							Point3D< Real > weightedGradient;
							if( HasGradients )
							{
								Point3D< Real > gradient
										(
												_dSplineValues[0][-j+LeftPointSupportRadius] * __splineValues[1][-k+LeftPointSupportRadius] * __splineValues[2][-l+LeftPointSupportRadius] ,
												__splineValues[0][-j+LeftPointSupportRadius] * _dSplineValues[1][-k+LeftPointSupportRadius] * __splineValues[2][-l+LeftPointSupportRadius] ,
												__splineValues[0][-j+LeftPointSupportRadius] * __splineValues[1][-k+LeftPointSupportRadius] * _dSplineValues[2][-l+LeftPointSupportRadius]
										);
								weightedGradient = gradient * interpolationInfo->gradientWeight * weight;
								diagonal += value * weightedValue + Point3D< Real >::Dot( gradient , weightedGradient );
							}
							else diagonal += value * weightedValue;

							// Pre-multiply the x-coordinate values so that when we evaluate at one of the neighboring basis functions
							// we get the product of the values of the center base function and the base function of the neighboring node
							if( HasGradients ) for( int s=0 ; s<SupportSize ; s++ ) _wSplineValues[0][s] *= weightedValue , _dSplineValues[0][s] *= weightedGradient[0] , _dSplineValues[1][s] *= weightedGradient[1] , _dSplineValues[2][s] *= weightedGradient[2];
							else               for( int s=0 ; s<SupportSize ; s++ ) _wSplineValues[0][s] *= weightedValue;
						}
					}
				}
	}

	Real pointValues[OverlapSize][OverlapSize][OverlapSize];
	if( interpolationInfo )
	{
		memset( pointValues , 0 , sizeof(Real) * OverlapSize * OverlapSize * OverlapSize );
		// Iterate over all supported neighbors that could have a point constraint
		for( int i=-LeftSupportRadius ; i<=RightSupportRadius ; i++ ) if( hasYZPoints[i+LeftSupportRadius] )
				for( int j=-LeftSupportRadius ; j<=RightSupportRadius ; j++ ) if( hasZPoints[i+LeftSupportRadius][j+LeftSupportRadius] )
						for( int k=-LeftSupportRadius ; k<=RightSupportRadius ; k++ )
						{
							const TreeOctNode* _node = neighbors.neighbors[i+OverlapRadius][j+OverlapRadius][k+OverlapRadius];
							if( _isValidSpaceNode( _node ) && (*interpolationInfo)( _node ) )
							{
								const PointData< Real , HasGradients >& pData = *( (*interpolationInfo)( _node ) );
#if POINT_DATA_RES
								for( int c=0 ; c<PointData< Real , HasGradients >::SAMPLES ; c++ ) if( pData[c].weight )
#endif // POINT_DATA_RES
								{
#if POINT_DATA_RES
									Real (*__splineValues)[SupportSize] = _splineValues[c][i+LeftSupportRadius][j+LeftSupportRadius][k+LeftSupportRadius];
							Real (*_wSplineValues)[SupportSize] = wSplineValues[c][i+LeftSupportRadius][j+LeftSupportRadius][k+LeftSupportRadius];
							Real (*_dSplineValues)[SupportSize] = dSplineValues[c][i+LeftSupportRadius][j+LeftSupportRadius][k+LeftSupportRadius];
#else // !POINT_DATA_RES
									Real (*__splineValues)[SupportSize] = _splineValues[i+LeftSupportRadius][j+LeftSupportRadius][k+LeftSupportRadius];
									Real (*_wSplineValues)[SupportSize] = wSplineValues[i+LeftSupportRadius][j+LeftSupportRadius][k+LeftSupportRadius];
									Real (*_dSplineValues)[SupportSize] = dSplineValues[i+LeftSupportRadius][j+LeftSupportRadius][k+LeftSupportRadius];
#endif // POINT_DATA_RES
									// Iterate over all neighbors whose support contains the point and accumulate the mutual integral
									for( int ii=-LeftPointSupportRadius ; ii<=RightPointSupportRadius ; ii++ )
										for( int jj=-LeftPointSupportRadius ; jj<=RightPointSupportRadius ; jj++ )
											if( HasGradients )
											{
												Real partialW_SplineValue = _wSplineValues[0][ii+LeftPointSupportRadius ] * __splineValues[1][jj+LeftPointSupportRadius ];
												Real partial__SplineValue = __splineValues[0][ii+LeftPointSupportRadius ] * __splineValues[1][jj+LeftPointSupportRadius ];
												Real partialD0SplineValue = _dSplineValues[0][ii+LeftPointSupportRadius ] * __splineValues[1][jj+LeftPointSupportRadius ];
												Real partialD1SplineValue = __splineValues[0][ii+LeftPointSupportRadius ] * _dSplineValues[1][jj+LeftPointSupportRadius ];
												Real* _pointValues = pointValues[i+ii+OverlapRadius][j+jj+OverlapRadius] + k + OverlapRadius;
												Real* ___splineValues = __splineValues[2] + LeftPointSupportRadius;
												Real* __dSplineValues = _dSplineValues[2] + LeftPointSupportRadius;
												TreeOctNode* const * _neighbors = neighbors.neighbors[i+ii+OverlapRadius][j+jj+OverlapRadius] + k + OverlapRadius;
												for( int kk=-LeftPointSupportRadius ; kk<=RightPointSupportRadius ; kk++ ) if( _isValidFEMNode( _neighbors[kk] ) )
														_pointValues[kk] +=
																partialW_SplineValue * ___splineValues[kk] + partialD0SplineValue * ___splineValues[kk] + partialD1SplineValue * ___splineValues[kk] + partial__SplineValue * __dSplineValues[kk];
											}
											else
											{
												Real partialWSplineValue = _wSplineValues[0][ii+LeftPointSupportRadius ] * __splineValues[1][jj+LeftPointSupportRadius ];
												Real* _pointValues = pointValues[i+ii+OverlapRadius][j+jj+OverlapRadius] + k + OverlapRadius;
												Real* ___splineValues = __splineValues[2] + LeftPointSupportRadius;
												TreeOctNode* const * _neighbors = neighbors.neighbors[i+ii+OverlapRadius][j+jj+OverlapRadius] + k + OverlapRadius;
												for( int kk=-LeftPointSupportRadius ; kk<=RightPointSupportRadius ; kk++ ) if( _isValidFEMNode( _neighbors[kk] ) )
														_pointValues[kk] += partialWSplineValue * ___splineValues[kk];
											}
								}
							}
						}
	}
	pointValues[OverlapRadius][OverlapRadius][OverlapRadius] = diagonal;
	int nodeIndex = neighbors.neighbors[OverlapRadius][OverlapRadius][OverlapRadius]->nodeData.nodeIndex;
	if( isInterior ) // General case, so try to make fast
	{
		const TreeOctNode* const * _nodes = &neighbors.neighbors[0][0][0];
		const double* _stencil = &stencil( 0 , 0 , 0 );
		Real* _values = &pointValues[0][0][0];
		const static int CenterIndex = OverlapSize*OverlapSize*OverlapRadius + OverlapSize*OverlapRadius + OverlapRadius;
		if( interpolationInfo ) for( int i=0 ; i<OverlapSize*OverlapSize*OverlapSize ; i++ ) _values[i] = Real( _stencil[i] + _values[i] );
		else                    for( int i=0 ; i<OverlapSize*OverlapSize*OverlapSize ; i++ ) _values[i] = Real( _stencil[i] );

		row[count++] = MatrixEntry< Real >( nodeIndex-offset , _values[CenterIndex] );
		for( int i=0 ; i<OverlapSize*OverlapSize*OverlapSize ; i++ ) if( i!=CenterIndex && _isValidFEMNode( _nodes[i] ) )
				row[count++] = MatrixEntry< Real >( _nodes[i]->nodeData.nodeIndex-offset , _values[i] );
	}
	else
	{
		LocalDepth d ; LocalOffset off;
		_localDepthAndOffset( node , d , off );
		Real temp = (Real)F.integrate( integrator , off , off );
		if( interpolationInfo ) temp += pointValues[OverlapRadius][OverlapRadius][OverlapRadius];
		row[count++] = MatrixEntry< Real >( nodeIndex-offset , temp );
		for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
					if( (x!=OverlapRadius || y!=OverlapRadius || z!=OverlapRadius) && _isValidFEMNode( neighbors.neighbors[x][y][z] ) )
					{
						const TreeOctNode* _node = neighbors.neighbors[x][y][z];
						LocalDepth _d ; LocalOffset _off;
						_localDepthAndOffset( _node , _d , _off );
						Real temp = (Real)F.integrate( integrator , _off , off );
						if( interpolationInfo ) temp += pointValues[x][y][z];
						row[count++] = MatrixEntry< Real >( _node->nodeData.nodeIndex-offset , temp );
					}
	}
	return count;
}

template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
int Octree< Real >::_getMatrixAndUpdateConstraints( const FEMSystemFunctor& F , const InterpolationInfo<  HasGradients >* interpolationInfo , SparseMatrix< Real >& matrix , DenseNodeData< Real , FEMDegree >& constraints , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& integrator , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& childIntegrator , const BSplineData< FEMDegree , BType >& bsData , LocalDepth depth , const DenseNodeData< Real , FEMDegree >& metSolution , bool coarseToFine )
{
	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;
	static const int OverlapSize   =   BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;

	size_t start = _sNodesBegin(depth) , end = _sNodesEnd(depth) , range = end-start;
	Stencil< double , OverlapSize > stencil , stencils[2][2][2];
	SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::SetCentralSystemStencil ( F ,      integrator , stencil  );
	SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::SetCentralSystemStencils( F , childIntegrator , stencils );
	matrix.Resize( (int)range );
	std::vector< AdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=0 ; i<(int)range ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i+start] ) )
		{
			AdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* node = _sNodes.treeNodes[i+start];
			// Get the matrix row size
			typename TreeOctNode::Neighbors< OverlapSize > neighbors;
			neighborKey.template getNeighbors< false , OverlapRadius , OverlapRadius >( node , neighbors );
			int count = _getMatrixRowSize< FEMDegree , BType >( neighbors );
			// Allocate memory for the row
			matrix.SetRowSize( i , count );

			// Set the row entries
			matrix.rowSizes[i] = _setMatrixRow( F , interpolationInfo , neighbors , matrix[i] , (int)start , integrator , stencil , bsData );
			if( coarseToFine && depth>0 )
			{
				// Offset the constraints using the solution from lower resolutions.
				int x , y , z;
				Cube::FactorCornerIndex( (int)( node - node->parent->children ) , x , y , z );
				typename TreeOctNode::Neighbors< OverlapSize > pNeighbors;
				neighborKey.template getNeighbors< false , OverlapRadius , OverlapRadius >( node->parent , pNeighbors );
				_updateConstraintsFromCoarser( F , interpolationInfo , neighbors , pNeighbors , node , constraints , metSolution , childIntegrator , stencils[x][y][z] , bsData );
			}
		}
	return 1;
}

template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
int Octree< Real >::_getSliceMatrixAndUpdateConstraints( const FEMSystemFunctor& F , const InterpolationInfo< HasGradients >* interpolationInfo , SparseMatrix< Real >& matrix , DenseNodeData< Real , FEMDegree >& constraints , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& integrator , typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& childIntegrator , const BSplineData< FEMDegree , BType >& bsData , LocalDepth depth , int slice , const DenseNodeData< Real , FEMDegree >& metSolution , bool coarseToFine )
{
	static const int OverlapSize   =  BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;
	static const int OverlapRadius = -BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;

	int nStart = _sNodesBegin( depth , slice ) , nEnd = _sNodesEnd( depth , slice );
	size_t range = nEnd - nStart;
	Stencil< double , OverlapSize > stencil , stencils[2][2][2];
	SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::SetCentralSystemStencil ( F ,      integrator , stencil  );
	SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::SetCentralSystemStencils( F , childIntegrator , stencils );

	matrix.Resize( (int)range );
	std::vector< AdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=0 ; i<(int)range ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i+nStart] ) )
		{
			AdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* node = _sNodes.treeNodes[i+nStart];
			// Get the matrix row size
			typename TreeOctNode::Neighbors< OverlapSize > neighbors;
			neighborKey.template getNeighbors< false , OverlapRadius , OverlapRadius >( node , neighbors );
			int count = _getMatrixRowSize< FEMDegree , BType >( neighbors );

			// Allocate memory for the row
			matrix.SetRowSize( i , count );

			// Set the row entries
			matrix.rowSizes[i] = _setMatrixRow( F , interpolationInfo , neighbors , matrix[i] , _sNodesBegin( depth , slice ) , integrator , stencil , bsData );

			if( coarseToFine && depth>0 )
			{
				// Offset the constraints using the solution from lower resolutions.
				int x , y , z;
				Cube::FactorCornerIndex( (int)( node - node->parent->children ) , x , y , z );
				typename TreeOctNode::Neighbors< OverlapSize > pNeighbors;
				neighborKey.template getNeighbors< false, OverlapRadius , OverlapRadius >( node->parent , pNeighbors );
				_updateConstraintsFromCoarser( F , interpolationInfo , neighbors , pNeighbors , node , constraints , metSolution , childIntegrator , stencils[x][y][z] , bsData );
			}
		}
	return 1;
}

#ifndef MOD
#define MOD( a , b ) ( (a)>0 ? (a) % (b) : ( (b) - ( -(a) % (b) ) ) % (b) )
#endif // MOD
template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
int Octree< Real >::_solveSystemGS( const FEMSystemFunctor& F , const BSplineData< FEMDegree , BType >& bsData , InterpolationInfo< HasGradients >* interpolationInfo , LocalDepth depth , DenseNodeData< Real , FEMDegree >& solution , DenseNodeData< Real , FEMDegree >& constraints , DenseNodeData< Real , FEMDegree >& metSolutionConstraints , int iters , bool coarseToFine , _SolverStats& stats , bool computeNorms )
{
	const int OverlapRadius = -BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;
	typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template      Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >      integrator;
	typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) > childIntegrator;
	BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::SetIntegrator( integrator , depth );
	if( depth>0 ) BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::SetChildIntegrator( childIntegrator , depth-1 );

	DenseNodeData< Real , FEMDegree >& metSolution    = metSolutionConstraints;	// This stores the up-sampled solution up to depth-2
	DenseNodeData< Real , FEMDegree >& metConstraints = metSolutionConstraints; // This stores the down-sampled constraints up to depth

	int sliceBegin = _BSplineBegin< FEMDegree , BType >( depth ) , sliceEnd = _BSplineEnd< FEMDegree , BType >( depth );

	if( coarseToFine )
	{
		if( depth>0 )
		{
			// Up-sample the cumulative change in solution @(depth-2) into the cumulative change in solution @(depth-1)
			if( depth-2>=0 ) _upSample< Real , FEMDegree , BType >( depth-1 , metSolution );
			// Add in the change in solution @(depth-1)
#pragma omp parallel for num_threads( threads )
			for( int i=_sNodesBegin(depth-1) ; i<_sNodesEnd(depth-1) ; i++ ) metSolution[i] += solution[i];
			// evaluate the points @(depth) using the cumulative change in solution @(depth-1)
			if( interpolationInfo )
			{
				_setPointValuesFromCoarser( *interpolationInfo , depth , bsData , metSolution );
			}
		}
	}
	else if( depth<_maxDepth ) for( int i=_sNodesBegin(depth) ; i<_sNodesEnd(depth) ; i++ ) constraints[i] -= metConstraints[i];
	double bNorm = 0 , inRNorm = 0 , outRNorm = 0;
	if( depth>=0 )
	{
		// Add padding space if we are computing residuals
		int frontOffset = computeNorms ? OverlapRadius : 0 , backOffset = computeNorms ? OverlapRadius : 0;
		// Set the number of in-memory slices required for a temporally blocked solver
		int solveSlices = std::max< int >( 0 , std::min< int >( OverlapRadius*iters - (OverlapRadius-1) , sliceEnd-sliceBegin ) ) , matrixSlices = std::max< int >( 1 , std::min< int >( solveSlices+frontOffset+backOffset , sliceEnd-sliceBegin ) );
		// The list of matrices for each in-memory slices
		std::vector< SparseMatrix< Real > > _M( matrixSlices );
		// The list of multi-colored indices  for each in-memory slice
		std::vector< std::vector< std::vector< int > > > __mcIndices( solveSlices );

		int dir = coarseToFine ? -1 : 1 , start = coarseToFine ? sliceEnd-1 : sliceBegin , end = coarseToFine ? sliceBegin-1 : sliceEnd;
		for( int frontSlice=start-frontOffset*dir , backSlice = frontSlice-OverlapRadius*(iters-1)*dir ; backSlice!=end+backOffset*dir ; frontSlice+=dir , backSlice+=dir )
		{
			double t;
			if( frontSlice+frontOffset*dir>=sliceBegin && frontSlice+frontOffset*dir<sliceEnd )
			{
				int s = frontSlice+frontOffset*dir , _s = MOD( s , matrixSlices );

				// Compute the system matrix
				_getSliceMatrixAndUpdateConstraints( F , interpolationInfo , _M[_s] , constraints , integrator , childIntegrator , bsData , depth , s , metSolution , coarseToFine );

				// Compute residuals
				if( computeNorms )
				{
					ConstPointer( Real ) B = GetPointer( &constraints[0] + _sNodesBegin( depth ) , _sNodesSize( depth ) ) + ( _sNodesBegin( depth , s ) - _sNodesBegin( depth ) );
					Pointer( Real ) X = GetPointer( &solution[0] + _sNodesBegin( depth ) , _sNodesSize( depth ) ) + ( _sNodesBegin( depth , s ) - _sNodesBegin( depth ) );
#pragma omp parallel for num_threads( threads ) reduction( + : bNorm , inRNorm )
					for( int j=0 ; j<_M[_s].rows ; j++ )
					{
						Real temp = Real(0);
						ConstPointer( MatrixEntry< Real > ) start = _M[_s][j];
						ConstPointer( MatrixEntry< Real > ) end = start + _M[_s].rowSizes[j];
						ConstPointer( MatrixEntry< Real > ) e;
						for( e=start ; e!=end ; e++ ) temp += X[ e->N ] * e->Value;
						bNorm += B[j]*B[j];
						inRNorm += (temp-B[j]) * (temp-B[j]);
					}
				}
			}

			// Compute the multicolor indices
			if( iters && frontSlice>=sliceBegin && frontSlice<sliceEnd )
			{
				int s = frontSlice , _s = MOD( s , matrixSlices ) , __s = MOD( s , solveSlices );
				for( int i=0 ; i<int( __mcIndices[__s].size() ) ; i++ ) __mcIndices[__s][i].clear();
				_setMultiColorIndices< FEMDegree >( _sNodesBegin( depth , s ) , _sNodesEnd( depth , s ) , __mcIndices[__s] );
			}
			// Advance through the in-memory slices, taking an appropriately sized stride
			for( int slice=frontSlice ; slice*dir>=backSlice*dir ; slice-=OverlapRadius*dir )
				if( slice>=sliceBegin && slice<sliceEnd )
				{
					int s = slice , _s = MOD( s , matrixSlices ) , __s = MOD( s , solveSlices );
					// Do the GS solver
					ConstPointer( Real ) B = GetPointer( &constraints[0] + _sNodesBegin( depth)  , _sNodesSize( depth ) ) + ( _sNodesBegin( depth , s ) - _sNodesBegin( depth ) );
					Pointer( Real ) X = GetPointer( &solution[0] + _sNodesBegin( depth ) , _sNodesSize( depth ) ) + ( _sNodesBegin( depth , s ) - _sNodesBegin( depth ) );
					SparseMatrix< Real >::SolveGS( __mcIndices[__s] , _M[_s] , B , X , !coarseToFine , threads );
				}

			// Compute residuals
			if( computeNorms && backSlice-backOffset*dir>=sliceBegin && backSlice-backOffset*dir<sliceEnd )
			{
				int s = backSlice-backOffset*dir , _s = MOD( s , matrixSlices );
				ConstPointer( Real ) B = GetPointer( &constraints[0] + _sNodesBegin( depth ) , _sNodesSize( depth ) ) + ( _sNodesBegin( depth , s ) - _sNodesBegin( depth ) );
				Pointer( Real ) X = GetPointer( &solution[0] + _sNodesBegin( depth ) , _sNodesSize( depth ) ) + ( _sNodesBegin( depth , s ) - _sNodesBegin( depth ) );
#pragma omp parallel for num_threads( threads ) reduction( + : outRNorm )
				for( int j=0 ; j<_M[_s].rows ; j++ )
				{
					Real temp = Real(0);
					ConstPointer( MatrixEntry< Real > ) start = _M[_s][j];
					ConstPointer( MatrixEntry< Real > ) end = start + _M[_s].rowSizes[j];
					ConstPointer( MatrixEntry< Real > ) e;
					for( e=start ; e!=end ; e++ ) temp += X[ e->N ] * e->Value;
					outRNorm += (temp-B[j]) * (temp-B[j]);
				}
			}
		}
	}
	if( computeNorms ) stats.bNorm2 = bNorm , stats.inRNorm2 = inRNorm , stats.outRNorm2 = outRNorm;

	if( !coarseToFine && depth>0 )
	{
		// Explicitly compute the restriction of the met solution onto the coarser nodes
		// and down-sample the previous accumulation
		{
			_updateCumulativeIntegralConstraintsFromFiner( F , bsData , depth , solution , metConstraints );
			if( interpolationInfo ) _updateCumulativeInterpolationConstraintsFromFiner( *interpolationInfo , bsData , depth , solution , metConstraints );
			if( depth<_maxDepth ) _downSample< Real , FEMDegree , BType >( depth , metConstraints );
		}
	}
	return iters;
}
#undef MOD

template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
int Octree< Real >::_solveSystemCG( const FEMSystemFunctor& F , const BSplineData< FEMDegree , BType >& bsData , InterpolationInfo< HasGradients >* interpolationInfo , LocalDepth depth , DenseNodeData< Real , FEMDegree >& solution , DenseNodeData< Real , FEMDegree >& constraints , DenseNodeData< Real , FEMDegree >& metSolutionConstraints , int iters , bool coarseToFine , _SolverStats& stats , bool computeNorms , double accuracy )
{
	typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template      Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >      integrator;
	typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) > childIntegrator;
	BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::SetIntegrator( integrator , depth );
	if( depth>0 ) BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::SetChildIntegrator( childIntegrator , depth-1 );

	DenseNodeData< Real , FEMDegree >& metSolution    = metSolutionConstraints;	// This stores the up-sampled solution up to depth-2
	DenseNodeData< Real , FEMDegree >& metConstraints = metSolutionConstraints; // This stores the down-sampled constraints up to depth

	int iter = 0;
	Pointer( Real ) X = GetPointer( &   solution[0] + _sNodesBegin(depth) , _sNodesSize(depth) );
	Pointer( Real ) B = GetPointer( &constraints[0] + _sNodesBegin(depth) , _sNodesSize(depth) );
	SparseMatrix< Real > M;

	if( coarseToFine )
	{
		if( depth>0 )
		{
			// Up-sample the cumulative change in solution @(depth-2) into the cumulative change in solution @(depth-1)
			if( depth-2>=0 ) _upSample< Real , FEMDegree , BType >( depth-1 , metSolution );
			// Add in the change in solution @(depth-1)
#pragma omp parallel for num_threads( threads )
			for( int i=_sNodesBegin(depth-1) ; i<_sNodesEnd(depth-1) ; i++ ) metSolution[i] += solution[i];
			// evaluate the points @(depth) using the cumulative change in solution @(depth-1)
			if( interpolationInfo )
			{
				_setPointValuesFromCoarser( *interpolationInfo , depth , bsData , metSolution );
			}
		}
	}
	else if( depth<_maxDepth ) for( int i=_sNodesBegin(depth) ; i<_sNodesEnd(depth) ; i++ ) constraints[i] -= metConstraints[i];

	// Get the system matrix (and adjust the right-hand-side based on the coarser solution if prolonging)
	_getMatrixAndUpdateConstraints( F , interpolationInfo , M , constraints , integrator , childIntegrator , bsData , depth , metSolution , coarseToFine );

	// Solve the linear system
	accuracy = Real( accuracy / 100000 ) * M.rows;
	int dim = _BSplineEnd< FEMDegree , BType >( depth ) - _BSplineBegin< FEMDegree , BType >( depth );
	int nonZeroRows = 0;
	for( int i=0 ; i<M.rows ; i++ ) if( M.rowSizes[i] ) nonZeroRows++;
	bool addDCTerm = ( nonZeroRows==dim*dim*dim && ( !interpolationInfo || !interpolationInfo->valueWeight ) && HasPartitionOfUnity< BType >() && F.vanishesOnConstants() );
	double bNorm = 0 , inRNorm = 0 , outRNorm = 0;
	if( computeNorms )
	{
#pragma omp parallel for num_threads( threads ) reduction( + : bNorm , inRNorm )
		for( int j=0 ; j<M.rows ; j++ )
		{
			Real temp = Real(0);
			ConstPointer( MatrixEntry< Real > ) start = M[j];
			ConstPointer( MatrixEntry< Real > ) end = start + M.rowSizes[j];
			ConstPointer( MatrixEntry< Real > ) e;
			for( e=start ; e!=end ; e++ ) temp += X[ e->N ] * e->Value;
			bNorm += B[j] * B[j];
			inRNorm += ( temp-B[j] ) * ( temp-B[j] );
		}
	}

	iters = std::min< int >( nonZeroRows , iters );
	if( iters ) iter += SparseMatrix< Real >::SolveCG( M , ( ConstPointer( Real ) )B , iters , X , Real( accuracy ) , 0 , addDCTerm , false , threads );

	if( computeNorms )
	{
#pragma omp parallel for num_threads( threads ) reduction( + : outRNorm )
		for( int j=0 ; j<M.rows ; j++ )
		{
			Real temp = Real(0);
			ConstPointer( MatrixEntry< Real > ) start = M[j];
			ConstPointer( MatrixEntry< Real > ) end = start + M.rowSizes[j];
			ConstPointer( MatrixEntry< Real > ) e;
			for( e=start ; e!=end ; e++ ) temp += X[ e->N ] * e->Value;
			outRNorm += ( temp-B[j] ) * ( temp-B[j] );
		}
		stats.bNorm2 = bNorm , stats.inRNorm2 = inRNorm , stats.outRNorm2 = outRNorm;
	}

	// Copy the old solution into the buffer, write in the new solution, compute the change, and update the met solution
	if( !coarseToFine && depth>0 )
	{
		// Explicitly compute the restriction of the met solution onto the coarser nodes
		// and down-sample the previous accumulation
		{
			_updateCumulativeIntegralConstraintsFromFiner( F , bsData , depth , solution , metConstraints );
			if( interpolationInfo ) _updateCumulativeInterpolationConstraintsFromFiner( *interpolationInfo , bsData , depth , solution , metConstraints );
			if( depth>_maxDepth ) _downSample< Real , FEMDegree , BType >( depth , metConstraints );
		}
	}
	return iter;
}

template< class Real >
template< int FEMDegree , BoundaryType BType >
int Octree< Real >::_getMatrixRowSize( const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& neighbors ) const
{
	static const int OverlapSize   =   BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;
	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;

	int count = 0;
	int nodeIndex = neighbors.neighbors[OverlapRadius][OverlapRadius][OverlapRadius]->nodeData.nodeIndex;
	const TreeOctNode* const * _nodes = &neighbors.neighbors[0][0][0];
	for( int i=0 ; i<OverlapSize*OverlapSize*OverlapSize ; i++ ) if( _isValidFEMNode( _nodes[i] ) ) count++;
	return count;
}


template< class Real >
template< int FEMDegree1 , int FEMDegree2 >
void Octree< Real >::_SetParentOverlapBounds( const TreeOctNode* node , int& startX , int& endX , int& startY , int& endY , int& startZ , int& endZ )
{
	const int OverlapStart = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapStart;

	if( node->parent )
	{
		int x , y , z , c = (int)( node - node->parent->children );
		Cube::FactorCornerIndex( c , x , y , z );
		startX = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::ParentOverlapStart[x]-OverlapStart , endX = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::ParentOverlapEnd[x]-OverlapStart+1;
		startY = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::ParentOverlapStart[y]-OverlapStart , endY = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::ParentOverlapEnd[y]-OverlapStart+1;
		startZ = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::ParentOverlapStart[z]-OverlapStart , endZ = BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::ParentOverlapEnd[z]-OverlapStart+1;
	}
}

// It is assumed that at this point, the evaluationg of the current depth's points, using the coarser resolution solution
// has already happened
template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
void Octree< Real >::_updateConstraintsFromCoarser( const FEMSystemFunctor& F , const InterpolationInfo<  HasGradients >* interpolationInfo , const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& neighbors , const typename TreeOctNode::Neighbors< BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& pNeighbors , TreeOctNode* node , DenseNodeData< Real , FEMDegree >& constraints , const DenseNodeData< Real , FEMDegree >& metSolution , const typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) >& childIntegrator , const Stencil< double , BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize >& lapStencil , const BSplineData< FEMDegree , BType >& bsData ) const
{
	static const int LeftSupportRadius  = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;

	if( _localDepth( node )<=0 ) return;
	// This is a conservative estimate as we only need to make sure that the parent nodes don't overlap the child (not the parent itself)
	bool isInterior = _isInteriorlyOverlapped< FEMDegree , FEMDegree >( node->parent );
	LocalDepth d ; LocalOffset off;
	_localDepthAndOffset( node , d , off );

	// Offset the constraints using the solution from lower resolutions.
	int startX , endX , startY , endY , startZ , endZ;
	_SetParentOverlapBounds< FEMDegree , FEMDegree >( node , startX , endX , startY , endY , startZ , endZ );

	for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
				if( _isValidFEMNode( pNeighbors.neighbors[x][y][z] ) )
				{
					const TreeOctNode* _node = pNeighbors.neighbors[x][y][z];
					Real _solution = metSolution[ _node->nodeData.nodeIndex ];
					{
						if( isInterior ) constraints[ node->nodeData.nodeIndex ] -= Real( lapStencil( x , y , z ) * _solution );
						else
						{
							LocalDepth _d ; LocalOffset _off;
							_localDepthAndOffset( _node , _d , _off );
							constraints[ node->nodeData.nodeIndex ] -= (Real)F.integrate( childIntegrator , _off , off ) * _solution;
						}
					}
				}

	if( interpolationInfo )
	{
		double constraint = 0;
		int fIdx[3];
		functionIndex< FEMDegree , BType >( node , fIdx );
		// evaluate the current node's basis function at adjacent points
		for( int x=-LeftSupportRadius ; x<=RightSupportRadius ; x++ ) for( int y=-LeftSupportRadius ; y<=RightSupportRadius ; y++ ) for( int z=-LeftSupportRadius ; z<=RightSupportRadius ; z++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[x+OverlapRadius][y+OverlapRadius][z+OverlapRadius];
					if( _isValidSpaceNode( _node ) && (*interpolationInfo)( _node ) )
					{
						const PointData< Real , HasGradients >& pData = *( (*interpolationInfo)( _node ) );
						constraint += _ConstraintCalculator_< Real , FEMDegree , HasGradients >::_CalculateConstraint_
								(
										pData ,
										bsData. baseBSplines[ fIdx[0] ][x+LeftSupportRadius] ,
										bsData. baseBSplines[ fIdx[1] ][y+LeftSupportRadius] ,
										bsData. baseBSplines[ fIdx[2] ][z+LeftSupportRadius] ,
										bsData.dBaseBSplines[ fIdx[0] ][x+LeftSupportRadius] ,
										bsData.dBaseBSplines[ fIdx[1] ][y+LeftSupportRadius] ,
										bsData.dBaseBSplines[ fIdx[2] ][z+LeftSupportRadius]
								);
					}
				}
		constraints[ node->nodeData.nodeIndex ] -= Real( constraint );
	}
}

// Given the solution @( depth ) add to the met constraints @( depth-1 )
template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor >
void Octree< Real >::_updateCumulativeIntegralConstraintsFromFiner( const FEMSystemFunctor& F , const BSplineData< FEMDegree , BType >& bsData , LocalDepth highDepth , const DenseNodeData< Real , FEMDegree >& fineSolution , DenseNodeData< Real , FEMDegree >& coarseConstraints ) const
{
	typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template ChildIntegrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) > childIntegrator;
	BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::SetChildIntegrator( childIntegrator , highDepth-1 );

	static const int OverlapSize   =   BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;
	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;
	typedef typename TreeOctNode::NeighborKey< -BSplineSupportSizes< FEMDegree >::SupportStart , BSplineSupportSizes< FEMDegree >::SupportEnd >SupportKey;

	if( highDepth<=0 ) return;
	// Get the stencil describing the Laplacian relating coefficients @(depth) with coefficients @(depth-1)
	Stencil< double , OverlapSize > stencils[2][2][2];
	SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::SetCentralSystemStencils( F , childIntegrator , stencils );
	size_t start = _sNodesBegin( highDepth) , end = _sNodesEnd(highDepth) , range = end-start;
	int lStart = _sNodesBegin(highDepth-1);

	// Iterate over the nodes @( depth )
	std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( highDepth )-1 );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(highDepth) ; i<_sNodesEnd(highDepth) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* node = _sNodes.treeNodes[i];

			// Offset the coarser constraints using the solution from the current resolutions.
			int x , y , z , c;
			c = int( node - node->parent->children );
			Cube::FactorCornerIndex( c , x , y , z );
			{
				typename TreeOctNode::Neighbors< OverlapSize > pNeighbors;
				neighborKey.template getNeighbors< false , OverlapRadius , OverlapRadius >( node->parent , pNeighbors );
				const Stencil< double , OverlapSize >& stencil = stencils[x][y][z];

				bool isInterior = _isInteriorlyOverlapped< FEMDegree , FEMDegree >( node->parent );
				LocalDepth d ; LocalOffset off;
				_localDepthAndOffset( node , d , off );

				// Offset the constraints using the solution from finer resolutions.
				int startX , endX , startY , endY , startZ , endZ;
				_SetParentOverlapBounds< FEMDegree , FEMDegree >( node , startX , endX , startY  , endY , startZ , endZ );

				Real solution = fineSolution[ node->nodeData.nodeIndex ];
				for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
							if( _isValidFEMNode( pNeighbors.neighbors[x][y][z] ) )
							{
								const TreeOctNode* _node = pNeighbors.neighbors[x][y][z];
								if( isInterior )
#pragma omp atomic
									coarseConstraints[ _node->nodeData.nodeIndex ] += Real( stencil( x , y , z ) * solution );
								else
								{
									LocalDepth _d ; LocalOffset _off;
									_localDepthAndOffset( _node , _d , _off );
#pragma omp atomic
									coarseConstraints[ _node->nodeData.nodeIndex ] += Real( F.integrate( childIntegrator , _off , off ) * solution );
								}
							}
			}
		}
}

template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
void Octree< Real >::setSystemMatrix( const FEMSystemFunctor& F , const InterpolationInfo<  HasGradients >* interpolationInfo , LocalDepth depth , SparseMatrix< Real >& matrix ) const
{
	if( depth<0 || depth>_maxDepth ) fprintf( stderr , "[ERROR] System depth out of bounds: %d <= %d <= %d\n" , 0 , depth , _maxDepth ) , exit( 0 );
	typename BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::FunctionIntegrator::template Integrator< DERIVATIVES( FEMDegree ) , DERIVATIVES( FEMDegree ) > integrator;
	BSplineIntegrationData< FEMDegree , BType , FEMDegree , BType >::SetIntegrator( integrator , depth );
	BSplineData< FEMDegree , BType > bsData( depth );

	static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;
	static const int OverlapSize   =   BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;

	Stencil< double , OverlapSize > stencil;
	SystemCoefficients< FEMDegree , BType , FEMDegree , BType >::SetCentralSystemStencil ( F , integrator , stencil );

	matrix.Resize( _sNodesSize(depth) );
	std::vector< AdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth) ; i<_sNodesEnd( depth ) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
		{
			int ii = i - _sNodesBegin(depth);
			AdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];

			typename TreeOctNode::Neighbors< OverlapSize > neighbors;
			neighborKey.template getNeighbors< false , OverlapRadius , OverlapRadius >( _sNodes.treeNodes[i] , neighbors );

			matrix.SetRowSize( ii , _getMatrixRowSize< FEMDegree , BType >( neighbors ) );
			matrix.rowSizes[ii] = _setMatrixRow( F , interpolationInfo , neighbors , matrix[ii] , _sNodesBegin(depth) , integrator , stencil , bsData );
		}
}

template< class Real >
template< int FEMDegree , BoundaryType BType , class FEMSystemFunctor , bool HasGradients >
DenseNodeData< Real , FEMDegree > Octree< Real >::solveSystem( const FEMSystemFunctor& F , InterpolationInfo< HasGradients >* interpolationInfo , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxSolveDepth , const typename Octree< Real >::SolverInfo& solverInfo )
{
	BSplineData< FEMDegree , BType > bsData( maxSolveDepth );

	maxSolveDepth = std::min< LocalDepth >( maxSolveDepth , _maxDepth );
	int iter = 0;
	const int _iters = std::max< int >( 0 , solverInfo.iters );

	DenseNodeData< Real , FEMDegree > solution( _sNodesEnd( _maxDepth ) );
	memset( &solution[0] , 0 , sizeof(Real) * _sNodesEnd( _maxDepth ) );

	DenseNodeData< Real , FEMDegree > metSolution( _sNodesEnd( _maxDepth-1 ) );
	memset( &metSolution[0] , 0 , sizeof(Real)*_sNodesEnd( _maxDepth-1 ) );
	for( LocalDepth d=0 ; d<=maxSolveDepth ; d++ )
	{
		int iters = (int)ceil( _iters * pow( solverInfo.lowResIterMultiplier , maxSolveDepth-d ) );
		_SolverStats sStats;
		if( !d ) iter = _solveSystemCG( F , bsData , interpolationInfo , d , solution , constraints , metSolution , _sNodesSize(d) , true , sStats , false , 0 );
		else
		{
			if( d>solverInfo.cgDepth ) iter = _solveSystemGS( F , bsData , interpolationInfo , d , solution , constraints , metSolution , iters , true , sStats , false );
			else                       iter = _solveSystemCG( F , bsData , interpolationInfo , d , solution , constraints , metSolution , iters , true , sStats , false , solverInfo.cgAccuracy );
		}
		int femNodes = 0;
#pragma omp parallel for reduction( + : femNodes )
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) ) femNodes++;
	}
	return solution;
}

template< class Real >
template< int FEMDegree >
DenseNodeData< Real , FEMDegree > Octree< Real >::initDenseNodeData( void )
{
	DenseNodeData< Real , FEMDegree > constraints( _sNodes.size() );
	memset( &constraints[0] , 0 , sizeof(Real)*_sNodes.size() );
	return constraints;
}
template< > template< > float  Octree< float  >::_Dot( const float & r1 , const float & r2 ){ return r1*r2; }
template< > template< > double Octree< double >::_Dot( const double& r1 , const double& r2 ){ return r1*r2; }
template< > template< > float  Octree< float  >::_Dot( const Point3D< float  >& p1 , const Point3D< float  >& p2 ){ return Point3D< float  >::Dot( p1 , p2 ); }
template< > template< > double Octree< double >::_Dot( const Point3D< double >& p1 , const Point3D< double >& p2 ){ return Point3D< double >::Dot( p1 , p2 ); }
template< > template< > bool Octree< float  >::_IsZero( const float & r ){ return r==0; }
template< > template< > bool Octree< double >::_IsZero( const double& r ){ return r==0; }
template< > template< > bool Octree< float  >::_IsZero( const Point3D< float  >& p ){ return p[0]==0 && p[1]==0 && p[2]==0; }
template< > template< > bool Octree< double >::_IsZero( const Point3D< double >& p ){ return p[0]==0 && p[1]==0 && p[2]==0; }
template< class Real >
template< int FEMDegree , BoundaryType FEMBType , int CDegree , BoundaryType CBType , class FEMConstraintFunctor , class Coefficients , class D , class _D >
void Octree< Real >::_addFEMConstraints( const FEMConstraintFunctor& F , const Coefficients& coefficients , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth )
{
	typedef typename TreeOctNode::NeighborKey< -BSplineSupportSizes< FEMDegree >::SupportStart , BSplineSupportSizes< FEMDegree >::SupportEnd > SupportKey;
	const int      CFEMOverlapSize   =  BSplineOverlapSizes< CDegree , FEMDegree >::OverlapSize;
	const int  LeftCFEMOverlapRadius = -BSplineOverlapSizes< CDegree , FEMDegree >::OverlapStart;
	const int RightCFEMOverlapRadius =  BSplineOverlapSizes< CDegree , FEMDegree >::OverlapEnd;
	const int  LeftFEMCOverlapRadius = -BSplineOverlapSizes< FEMDegree , CDegree >::OverlapStart;
	const int RightFEMCOverlapRadius =  BSplineOverlapSizes< FEMDegree , CDegree >::OverlapEnd;

	// To set the constraints, we iterate over the
	// splatted normals and compute the dot-product of the
	// divergence of the normal field with all the basis functions.
	// Within the same depth: set directly as a gather
	// Coarser depths
	maxDepth = std::min< LocalDepth >( maxDepth , _maxDepth );
	DenseNodeData< Real , FEMDegree >* __constraints = new DenseNodeData< Real , FEMDegree >( _sNodesEnd(maxDepth-1) );
	DenseNodeData< Real , FEMDegree >& _constraints = *__constraints;
	memset( &_constraints[0] , 0 , sizeof(Real)*( _sNodesEnd(maxDepth-1) ) );

	for( LocalDepth d=maxDepth ; d>=0 ; d-- )
	{
		Stencil< _D , CFEMOverlapSize > stencil , stencils[2][2][2];
		typename SystemCoefficients< CDegree , CBType , FEMDegree , FEMBType >::     Integrator      integrator;
		typename SystemCoefficients< FEMDegree , FEMBType , CDegree , CBType >::ChildIntegrator childIntegrator;
		BSplineIntegrationData< CDegree , CBType , FEMDegree , FEMBType >::SetIntegrator( integrator , d );
		if( d>0 ) BSplineIntegrationData< FEMDegree , FEMBType , CDegree , CBType >::SetChildIntegrator( childIntegrator , d-1 );
		SystemCoefficients< CDegree , CBType , FEMDegree , FEMBType >::template SetCentralConstraintStencil < false >( F,      integrator , stencil  );
		SystemCoefficients< FEMDegree , FEMBType , CDegree , CBType >::template SetCentralConstraintStencils< true  >( F, childIntegrator , stencils );

		std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
		for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d ) );

#pragma omp parallel for num_threads( threads )
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ )
		{
			SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* node = _sNodes.treeNodes[i];
			int startX=0 , endX=CFEMOverlapSize , startY=0 , endY=CFEMOverlapSize , startZ=0 , endZ=CFEMOverlapSize;
			typename TreeOctNode::Neighbors< CFEMOverlapSize > neighbors;
			neighborKey.template getNeighbors< false , LeftFEMCOverlapRadius , RightFEMCOverlapRadius >( node , neighbors );
			bool isInterior = _isInteriorlyOverlapped< FEMDegree , CDegree >( node ) , isInterior2 = _isInteriorlyOverlapped< CDegree , FEMDegree >( node->parent );

			LocalDepth d ; LocalOffset off;
			_localDepthAndOffset( node , d , off );
			// Set constraints from current depth
			// Gather the constraints from the vector-field at _node into the constraint stored with node
			if( _isValidFEMNode( node ) )
			{
				for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
						{
							const TreeOctNode* _node = neighbors.neighbors[x][y][z];
							if( isValidFEMNode< CDegree , CBType >( _node ) )
							{
								const D* d = coefficients( _node );
								if( d )
								{
									if( isInterior )
										constraints[i] += _Dot( (D)stencil( x , y , z ) , *d );
									else
									{
										LocalDepth _d ; LocalOffset _off;
										_localDepthAndOffset( _node , _d , _off );
										constraints[i] += _Dot( *d , (D)F.template integrate< false >( integrator , _off , off ) );
									}
								}
							}
						}
				_SetParentOverlapBounds< CDegree , FEMDegree >( node , startX , endX , startY , endY , startZ , endZ );
			}
			if( !isValidFEMNode< CDegree , CBType >( node ) ) continue;
			const D* _data = coefficients( node );
			if( !_data ) continue;
			const D& data = *_data;
			if( _IsZero( data ) ) continue;

			// Set the _constraints for the parents
			if( d>0 )
			{
				int cx , cy , cz;
				Cube::FactorCornerIndex( (int)( node - node->parent->children ) , cx , cy ,cz );
				const Stencil< _D , CFEMOverlapSize >& _stencil = stencils[cx][cy][cz];

				neighborKey.template getNeighbors< false , LeftCFEMOverlapRadius , RightCFEMOverlapRadius >( node->parent , neighbors );

				for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
						{
							TreeOctNode* _node = neighbors.neighbors[x][y][z];
							if( _node && ( isInterior2 || _isValidFEMNode( _node ) ) )
							{
								TreeOctNode* _node = neighbors.neighbors[x][y][z];
								Real c;
								if( isInterior2 ) c = _Dot( (D)_stencil( x , y , z ) , data );
								else
								{
									LocalDepth _d ; LocalOffset _off;
									_localDepthAndOffset( _node , _d , _off );
									c = _Dot( data , (D)F.template integrate< true >( childIntegrator , _off , off ) );
								}
#pragma omp atomic
								_constraints[ _node->nodeData.nodeIndex ] += c;
							}
						}
			}
		}
	}

	// Fine-to-coarse down-sampling of constraints
	for( LocalDepth d=maxDepth-1 ; d>0 ; d-- ) _downSample< Real , FEMDegree , FEMBType >( d , _constraints );

	// Add the accumulated constraints from all finer depths
#pragma omp parallel for num_threads( threads )
	for( int i=0 ; i<_sNodesEnd(maxDepth-1) ; i++ ) constraints[i] += _constraints[i];

	delete __constraints;

	DenseNodeData< D , CDegree > _coefficients( _sNodesEnd(maxDepth-1) );
	memset( &_coefficients[0] , 0 , sizeof(D) * _sNodesEnd(maxDepth-1) );
	for( LocalDepth d=maxDepth-1 ; d>=0 ; d-- )
	{
#pragma omp parallel for num_threads( threads )
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( isValidFEMNode< CDegree , CBType >( _sNodes.treeNodes[i] ) )
			{
				const D* d = coefficients( _sNodes.treeNodes[i] );
				if( d )	_coefficients[i] += *d;
			}
	}

	// Coarse-to-fine up-sampling of coefficients
	for( LocalDepth d=1 ; d<maxDepth ; d++ ) _upSample< D , CDegree , CBType >( d , _coefficients );

	// Compute the contribution from all coarser depths
	for( LocalDepth d=1 ; d<=maxDepth ; d++ )
	{
		size_t start = _sNodesBegin( d ) , end = _sNodesEnd( d ) , range = end - start;
		Stencil< _D , CFEMOverlapSize > stencils[2][2][2];
		typename SystemCoefficients< CDegree , CBType , FEMDegree , FEMBType >::ChildIntegrator childIntegrator;
		BSplineIntegrationData< CDegree , CBType , FEMDegree , FEMBType >::SetChildIntegrator( childIntegrator , d-1 );
		SystemCoefficients< CDegree , CBType , FEMDegree , FEMBType >::template SetCentralConstraintStencils< false >( F , childIntegrator , stencils );
		std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
		for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d-1 ) );
#pragma omp parallel for num_threads( threads )
		for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
			{
				SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
				TreeOctNode* node = _sNodes.treeNodes[i];
				int startX , endX , startY , endY , startZ , endZ;
				_SetParentOverlapBounds< FEMDegree , CDegree >( node , startX , endX , startY , endY , startZ , endZ );
				typename TreeOctNode::Neighbors< CFEMOverlapSize > pNeighbors;
				neighborKey.template getNeighbors< false , LeftFEMCOverlapRadius , RightFEMCOverlapRadius >( node->parent , pNeighbors );

				bool isInterior = _isInteriorlyOverlapped< FEMDegree , CDegree >( node->parent );
				int cx , cy , cz;
				if( d>0 )
				{
					int c = int( node - node->parent->children );
					Cube::FactorCornerIndex( c , cx , cy , cz );
				}
				else cx = cy = cz = 0;
				Stencil< _D , CFEMOverlapSize >& _stencil = stencils[cx][cy][cz];

				Real constraint = Real(0);
				LocalDepth d ; LocalOffset off;
				_localDepthAndOffset( node , d , off );
				for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
						{
							TreeOctNode* _node = pNeighbors.neighbors[x][y][z];
							if( isValidFEMNode< CDegree , CBType >( _node ) )
							{
								if( isInterior ) constraint += _Dot( _coefficients[ _node->nodeData.nodeIndex ] , (D)_stencil( x , y , z ) );
								else
								{
									LocalDepth _d ; LocalOffset _off;
									_localDepthAndOffset ( _node , _d , _off );
									constraint += _Dot( _coefficients[ _node->nodeData.nodeIndex ] , (D)F.template integrate< false >( childIntegrator , _off , off ) );
								}
							}
						}
				constraints[i] += constraint;
			}
	}
}

template< class Real >
template< int FEMDegree , BoundaryType BType , bool HasGradients >
void Octree< Real >::addInterpolationConstraints( const InterpolationInfo< HasGradients >& interpolationInfo , DenseNodeData< Real , FEMDegree >& constraints , LocalDepth maxDepth )
{
	typedef typename TreeOctNode::NeighborKey< -BSplineSupportSizes< FEMDegree >::SupportStart , BSplineSupportSizes< FEMDegree >::SupportEnd > SupportKey;
	maxDepth = std::min< LocalDepth >( maxDepth , _maxDepth );
	{
		static const int OverlapSize = BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapSize;
		static const int LeftSupportRadius  = -BSplineSupportSizes< FEMDegree >::SupportStart;
		static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
		static const int OverlapRadius = - BSplineOverlapSizes< FEMDegree , FEMDegree >::OverlapStart;
		BSplineData< FEMDegree , BType > bsData( _maxDepth );
		for( int d=0 ; d<=maxDepth ; d++ )
		{
			std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
			for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( maxDepth ) );

#pragma omp parallel for num_threads( threads )
			for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ ) if( _isValidFEMNode( _sNodes.treeNodes[i] ) )
				{
					TreeOctNode* node = _sNodes.treeNodes[i];
					SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
					typename TreeOctNode::Neighbors< OverlapSize > neighbors;
					neighborKey.template getNeighbors< false , OverlapRadius , OverlapRadius >( node , neighbors );

					double constraint = 0;
					int fIdx[3];
					functionIndex< FEMDegree , BType >( node , fIdx );
					// evaluate the current node's basis function at adjacent points
					for( int x=-LeftSupportRadius ; x<=RightSupportRadius ; x++ ) for( int y=-LeftSupportRadius ; y<=RightSupportRadius ; y++ ) for( int z=-LeftSupportRadius ; z<=RightSupportRadius ; z++ )
							{
								const TreeOctNode* _node = neighbors.neighbors[x+OverlapRadius][y+OverlapRadius][z+OverlapRadius];
								if( _isValidSpaceNode( _node ) && interpolationInfo( _node ) )
								{
									const PointData< Real , HasGradients >& pData = *( interpolationInfo( _node ) );
									constraint += _ConstraintCalculator_< Real , FEMDegree , HasGradients >::_CalculateConstraint_
											(
													pData ,
													bsData. baseBSplines[ fIdx[0] ][x+LeftSupportRadius] ,
													bsData. baseBSplines[ fIdx[1] ][y+LeftSupportRadius] ,
													bsData. baseBSplines[ fIdx[2] ][z+LeftSupportRadius] ,
													bsData.dBaseBSplines[ fIdx[0] ][x+LeftSupportRadius] ,
													bsData.dBaseBSplines[ fIdx[1] ][y+LeftSupportRadius] ,
													bsData.dBaseBSplines[ fIdx[2] ][z+LeftSupportRadius] ,
													interpolationInfo.valueWeight , interpolationInfo.gradientWeight
											);
								}
							}
					constraints[ node->nodeData.nodeIndex ] += (Real)constraint;
				}
		}
	}
}
template< class Real >
template< int FEMDegree1 , BoundaryType FEMBType1 , int FEMDegree2 , BoundaryType FEMBType2 , class DotFunctor , bool HasGradients , class Coefficients1 , class Coefficients2 >
double Octree< Real >::_dot( const DotFunctor& F , const InterpolationInfo< HasGradients >* iInfo , const Coefficients1& coefficients1 , const Coefficients2& coefficients2 ) const
{
	double dot = 0;

	// Calculate the contribution from @(depth,depth)
	{
		typedef typename TreeOctNode::ConstNeighborKey< -BSplineSupportSizes< FEMDegree1 >::SupportStart , BSplineSupportSizes< FEMDegree1 >::SupportEnd > SupportKey;
		const int      OverlapSize   =  BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapSize;
		const int  LeftOverlapRadius = -BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapStart;
		const int RightOverlapRadius =  BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapEnd;

		for( LocalDepth d=0 ; d<=_maxDepth ; d++ )
		{
			Stencil< double , OverlapSize > stencil;
			typename SystemCoefficients< FEMDegree1 , FEMBType1 , FEMDegree2 , FEMBType2 >::Integrator integrator;
			BSplineIntegrationData< FEMDegree1 , FEMBType1 , FEMDegree2 , FEMBType2 >::SetIntegrator( integrator , d );
			SystemCoefficients< FEMDegree1 , FEMBType1 , FEMDegree2 , FEMBType2 >::template SetCentralConstraintStencil< false , DotFunctor >( F , integrator , stencil );

			std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
			for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d ) );

#pragma omp parallel for num_threads( threads ) reduction( + : dot )
			for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ )
			{
				const TreeOctNode* node = _sNodes.treeNodes[i];
				const Real* _data1;
				if( isValidFEMNode< FEMDegree1 , FEMBType1 >( node ) && ( _data1=coefficients1(node) ) )
				{
					SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
					typename TreeOctNode::ConstNeighbors< OverlapSize > neighbors;
					neighborKey.template getNeighbors< LeftOverlapRadius , RightOverlapRadius >( node , neighbors );
					bool isInterior = _isInteriorlyOverlapped< FEMDegree1 , FEMDegree2 >( node );

					LocalDepth d ; LocalOffset off;
					_localDepthAndOffset( node , d , off );

					for( int x=0 ; x<OverlapSize ; x++ ) for( int y=0 ; y<OverlapSize ; y++ ) for( int z=0 ; z<OverlapSize ; z++ )
							{
								const TreeOctNode* _node = neighbors.neighbors[x][y][z];
								const Real* _data2;
								if( isValidFEMNode< FEMDegree2 , FEMBType2 >( _node ) && ( _data2=coefficients2( _node ) ) )
								{
									if( isInterior ) dot += (*_data1) * (*_data2 ) * stencil( x , y , z );
									else
									{
										LocalDepth _d ; LocalOffset _off;
										_localDepthAndOffset( _node , _d , _off );
										dot += (*_data1) * (*_data2) * F.template integrate< false >( integrator , off , _off );
									}
								}
							}
				}
			}
		}
	}
	// Calculate the contribution from @(<depth,depth)
	{
		typedef typename TreeOctNode::ConstNeighborKey< -BSplineSupportSizes< FEMDegree1 >::SupportStart , BSplineSupportSizes< FEMDegree1 >::SupportEnd > SupportKey;
		const int      OverlapSize   =  BSplineOverlapSizes< FEMDegree2 , FEMDegree1 >::OverlapSize;
		const int  LeftOverlapRadius = -BSplineOverlapSizes< FEMDegree2 , FEMDegree1 >::OverlapStart;
		const int RightOverlapRadius =  BSplineOverlapSizes< FEMDegree2 , FEMDegree1 >::OverlapEnd;

		DenseNodeData< Real , FEMDegree1 > cumulative1( _sNodesEnd( _maxDepth-1 ) );
		if( _maxDepth>0 ) memset( &cumulative1[0] , 0 , sizeof(Real) * _sNodesEnd( _maxDepth-1 ) );

		for( LocalDepth d=1 ; d<=_maxDepth ; d++ )
		{
			// Update the cumulative coefficients with the coefficients @(depth-1)
#pragma omp parallel for
			for( int i=_sNodesBegin(d-1) ; i<_sNodesEnd(d-1) ; i++ )
			{
				const Real* _data1 = coefficients1( _sNodes.treeNodes[i] );
				if( _data1 ) cumulative1[i] += *_data1;
			}

			Stencil< double , OverlapSize > stencils[2][2][2];
			typename SystemCoefficients< FEMDegree1 , FEMBType1 , FEMDegree2 , FEMBType2 >::ChildIntegrator childIntegrator;
			BSplineIntegrationData< FEMDegree1 , FEMBType1 , FEMDegree2 , FEMBType2 >::SetChildIntegrator( childIntegrator , d-1 );
			SystemCoefficients< FEMDegree1 , FEMBType1 , FEMDegree2 , FEMBType2 >::template SetCentralConstraintStencils< false >( F, childIntegrator , stencils );

			std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
			for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d-1 ) );

#pragma omp parallel for num_threads( threads ) reduction( + : dot )
			for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ )
			{
				const TreeOctNode* node = _sNodes.treeNodes[i];
				const Real* _data2;
				if( isValidFEMNode< FEMDegree2 , FEMBType2 >( node ) && ( _data2=coefficients2( node ) ) )
				{
					SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
					bool isInterior = _isInteriorlyOverlapped< FEMDegree1 , FEMDegree2 >( node->parent );

					LocalDepth d ; LocalOffset off;
					_localDepthAndOffset( node , d , off );

					int cx , cy , cz;
					Cube::FactorCornerIndex( (int)( node - node->parent->children ) , cx , cy ,cz );
					const Stencil< double , OverlapSize >& _stencil = stencils[cx][cy][cz];
					typename TreeOctNode::ConstNeighbors< OverlapSize > neighbors;
					neighborKey.template getNeighbors< LeftOverlapRadius , RightOverlapRadius >( node->parent , neighbors );

					int startX , endX , startY , endY , startZ , endZ;
					_SetParentOverlapBounds< FEMDegree2 , FEMDegree1 >( node , startX , endX , startY , endY , startZ , endZ );
					for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
							{
								const TreeOctNode* _node = neighbors.neighbors[x][y][z];
								const Real* _data1;
								if( isValidFEMNode< FEMDegree1 , FEMBType1 >( _node ) && ( _data1=cumulative1(_node) ) )
								{
									if( isInterior ) dot += (*_data1) * (*_data2) * _stencil( x , y , z );
									else
									{
										LocalDepth _d ; LocalOffset _off;
										_localDepthAndOffset( _node , _d , _off );
										dot += (*_data1) * (*_data2) * F.template integrate< false >( childIntegrator , _off , off );
									}
								}
							}
				}
			}
			// Up sample the cumulative coefficients for the next level
			if( d<_maxDepth ) _upSample< Real , FEMDegree1 , FEMBType1 >( d , cumulative1 );
		}
	}

	// Calculate the contribution from @(>depth,depth)
	{
		typedef typename TreeOctNode::ConstNeighborKey< -BSplineSupportSizes< FEMDegree2 >::SupportStart , BSplineSupportSizes< FEMDegree2 >::SupportEnd > SupportKey;
		const int      OverlapSize   =  BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapSize;
		const int  LeftOverlapRadius = -BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapStart;
		const int RightOverlapRadius =  BSplineOverlapSizes< FEMDegree1 , FEMDegree2 >::OverlapEnd;

		DenseNodeData< Real , FEMDegree2 > cumulative2( _sNodesEnd( _maxDepth-1 ) );
		if( _maxDepth>0 ) memset( &cumulative2[0] , 0 , sizeof(Real) * _sNodesEnd( _maxDepth-1 ) );

		for( LocalDepth d=_maxDepth ; d>0 ; d-- )
		{
			Stencil< double , OverlapSize > stencils[2][2][2];
			typename SystemCoefficients< FEMDegree2 , FEMBType2 , FEMDegree1 , FEMBType1 >::ChildIntegrator childIntegrator;
			BSplineIntegrationData< FEMDegree2 , FEMBType2 , FEMDegree1 , FEMBType1 >::SetChildIntegrator( childIntegrator , d-1 );
			SystemCoefficients< FEMDegree2 , FEMBType2 , FEMDegree1 , FEMBType1 >::template SetCentralConstraintStencils< true >( F , childIntegrator , stencils );

			std::vector< SupportKey > neighborKeys( std::max< int >( 1 , threads ) );
			for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( d-1 ) );

			// Update the cumulative constraints @(depth-1) from @(depth)
#pragma omp parallel for num_threads( threads )
			for( int i=_sNodesBegin(d) ; i<_sNodesEnd(d) ; i++ )
			{
				const TreeOctNode* node = _sNodes.treeNodes[i];
				const Real* _data1;
				if( isValidFEMNode< FEMDegree1 , FEMBType1 >( node ) && ( _data1=coefficients1( node ) ) )
				{
					SupportKey& neighborKey = neighborKeys[ get_thread_num() ];
					bool isInterior = _isInteriorlyOverlapped< FEMDegree2 , FEMDegree1 >( node->parent );

					LocalDepth d ; LocalOffset off;
					_localDepthAndOffset( node , d , off );

					int cx , cy , cz;
					Cube::FactorCornerIndex( (int)( node - node->parent->children ) , cx , cy ,cz );
					const Stencil< double , OverlapSize >& _stencil = stencils[cx][cy][cz];
					typename TreeOctNode::ConstNeighbors< OverlapSize > neighbors;
					neighborKey.template getNeighbors< LeftOverlapRadius , RightOverlapRadius >( node->parent , neighbors );

					int startX , endX , startY , endY , startZ , endZ;
					_SetParentOverlapBounds< FEMDegree1 , FEMDegree2 >( node , startX , endX , startY , endY , startZ , endZ );

					for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
							{
								const TreeOctNode* _node = neighbors.neighbors[x][y][z];
								if( isValidFEMNode< FEMDegree2 , FEMBType2 >( _node ) )
								{
									Real _dot;
									if( isInterior ) _dot = (*_data1) * _stencil( x , y , z );
									else
									{
										LocalDepth _d ; LocalOffset _off;
										_localDepthAndOffset( _node , _d , _off );
										_dot = (*_data1) * F.template integrate< true >( childIntegrator , _off , off );
									}
#pragma omp atomic
									cumulative2[ _node->nodeData.nodeIndex ] += _dot;
								}
							}
				}
			}
			// Update the dot-product using the cumulative constraints @(depth-1)
#pragma omp parallel for num_threads( threads ) reduction( + : dot )
			for( int i=_sNodesBegin(d-1) ; i<_sNodesEnd(d-1) ; i++ )
			{
				const TreeOctNode* node = _sNodes.treeNodes[i];
				const Real* _data2;
				if( isValidFEMNode< FEMDegree2 , FEMBType2 >( node ) && ( _data2=coefficients2( node ) ) ) dot += cumulative2[ node->nodeData.nodeIndex ] * (*_data2);
			}

			// Down-sample the cumulative constraints from @(depth-1) to @(depth-2) for the next pass
			if( d-1>0 ) _downSample< Real , FEMDegree2 , FEMBType2 >( d-1 , cumulative2 );
		}
	}

	if( iInfo )
	{
		MultiThreadedEvaluator< FEMDegree1 , FEMBType1 > mt1( this , coefficients1 , threads );
		MultiThreadedEvaluator< FEMDegree2 , FEMBType2 > mt2( this , coefficients2 , threads );

#pragma omp parallel for num_threads( threads ) reduction( + : dot )
		for( int i=_sNodesBegin(0) ; i<_sNodesEnd(_maxDepth) ; i++ )
		{
			if( _isValidSpaceNode( _sNodes.treeNodes[i] ) && !_isValidSpaceNode( _sNodes.treeNodes[i]->children ) && (*iInfo)( _sNodes.treeNodes[i] ) )
			{

				const PointData< Real , HasGradients >& pData = *( (*iInfo)( _sNodes.treeNodes[i] ) );
#if POINT_DATA_RES
				for( int c=0 ; c<PointData< Real , false >::SAMPLES ; c++ ) if( pData[c].weight )
				{
					Point3D< Real > p = pData[c].position;
					Real w = pData[c].weight;
					if( HasGradients )
					{
						std::pair< Real , Point3D< Real > > v1 = mt1.valueAndGradient( p , get_thread_num() );
						std::pair< Real , Point3D< Real > > v2 = mt2.valueAndGradient( p , get_thread_num() );
						dot += v1.first * v2.first * w * iInfo->valueWeight + Point3D< Real >::Dot( v1.second , v2.second ) * w * iInfo->gradientWeight;
					}
					else dot += mt1.value( p , get_thread_num() ) * mt2.value( p , get_thread_num() ) * w * iInfo->valueWeight;
				}
#else // !POINT_DATA_RES
				Point3D< Real > p = pData.position;
				Real w = pData.weight;
				if( HasGradients )
				{
					std::pair< Real , Point3D< Real > > v1 = mt1.valueAndGradient( p , get_thread_num() );
					std::pair< Real , Point3D< Real > > v2 = mt2.valueAndGradient( p , get_thread_num() );
					dot += v1.first * v2.first * w * iInfo->valueWeight + Point3D< Real >::Dot( v1.second , v2.second ) * w * iInfo->gradientWeight;
				}
				else dot += mt1.value( p , get_thread_num() ) * mt2.value( p , get_thread_num() ) * w * iInfo->valueWeight;
#endif // POINT_DATA_RES
			}
		}
	}

	return dot;
}


template< class Real >
template< class Vertex >
Octree< Real >::_SliceValues< Vertex >::_SliceValues( void )
{
	_oldCCount = _oldECount = _oldFCount = _oldNCount = 0;
	cornerValues = NullPointer( Real ) ; cornerGradients = NullPointer( Point3D< Real > ) ; cornerSet = NullPointer( char );
	edgeKeys = NullPointer( long long ) ; edgeSet = NullPointer( char );
	faceEdges = NullPointer( _FaceEdges ) ; faceSet = NullPointer( char );
	mcIndices = NullPointer( char );
}
template< class Real >
template< class Vertex >
Octree< Real >::_SliceValues< Vertex >::~_SliceValues( void )
{
	_oldCCount = _oldECount = _oldFCount = _oldNCount = 0;
	FreePointer( cornerValues ) ; FreePointer( cornerGradients ) ; FreePointer( cornerSet );
	FreePointer( edgeKeys ) ; FreePointer( edgeSet );
	FreePointer( faceEdges ) ; FreePointer( faceSet );
	FreePointer( mcIndices );
}
template< class Real >
template< class Vertex >
void Octree< Real >::_SliceValues< Vertex >::reset( bool nonLinearFit )
{
	faceEdgeMap.clear() , edgeVertexMap.clear() , vertexPairMap.clear();

	if( _oldNCount<sliceData.nodeCount )
	{
		_oldNCount = sliceData.nodeCount;
		FreePointer( mcIndices );
		if( sliceData.nodeCount>0 ) mcIndices = AllocPointer< char >( _oldNCount );
	}
	if( _oldCCount<sliceData.cCount )
	{
		_oldCCount = sliceData.cCount;
		FreePointer( cornerValues ) ; FreePointer( cornerGradients ) ; FreePointer( cornerSet );
		if( sliceData.cCount>0 )
		{
			cornerValues = AllocPointer< Real >( _oldCCount );
			if( nonLinearFit ) cornerGradients = AllocPointer< Point3D< Real > >( _oldCCount );
			cornerSet = AllocPointer< char >( _oldCCount );
		}
	}
	if( _oldECount<sliceData.eCount )
	{
		_oldECount = sliceData.eCount;
		FreePointer( edgeKeys ) ; FreePointer( edgeSet );
		edgeKeys = AllocPointer< long long >( _oldECount );
		edgeSet = AllocPointer< char >( _oldECount );
	}
	if( _oldFCount<sliceData.fCount )
	{
		_oldFCount = sliceData.fCount;
		FreePointer( faceEdges ) ; FreePointer( faceSet );
		faceEdges = AllocPointer< _FaceEdges >( _oldFCount );
		faceSet = AllocPointer< char >( _oldFCount );
	}

	if( sliceData.cCount>0 ) memset( cornerSet , 0 , sizeof( char ) * sliceData.cCount );
	if( sliceData.eCount>0 ) memset(   edgeSet , 0 , sizeof( char ) * sliceData.eCount );
	if( sliceData.fCount>0 ) memset(   faceSet , 0 , sizeof( char ) * sliceData.fCount );
}
template< class Real >
template< class Vertex >
Octree< Real >::_XSliceValues< Vertex >::_XSliceValues( void )
{
	_oldECount = _oldFCount = 0;
	edgeKeys = NullPointer( long long ) ; edgeSet = NullPointer( char );
	faceEdges = NullPointer( _FaceEdges ) ; faceSet = NullPointer( char );
}
template< class Real >
template< class Vertex >
Octree< Real >::_XSliceValues< Vertex >::~_XSliceValues( void )
{
	_oldECount = _oldFCount = 0;
	FreePointer( edgeKeys ) ; FreePointer( edgeSet );
	FreePointer( faceEdges ) ; FreePointer( faceSet );
}
template< class Real >
template< class Vertex >
void Octree< Real >::_XSliceValues< Vertex >::reset( void )
{
	faceEdgeMap.clear() , edgeVertexMap.clear() , vertexPairMap.clear();

	if( _oldECount<xSliceData.eCount )
	{
		_oldECount = xSliceData.eCount;
		FreePointer( edgeKeys ) ; FreePointer( edgeSet );
		edgeKeys = AllocPointer< long long >( _oldECount );
		edgeSet = AllocPointer< char >( _oldECount );
	}
	if( _oldFCount<xSliceData.fCount )
	{
		_oldFCount = xSliceData.fCount;
		FreePointer( faceEdges ) ; FreePointer( faceSet );
		faceEdges = AllocPointer< _FaceEdges >( _oldFCount );
		faceSet = AllocPointer< char >( _oldFCount );
	}
	if( xSliceData.eCount>0 ) memset( edgeSet , 0 , sizeof( char ) * xSliceData.eCount );
	if( xSliceData.fCount>0 ) memset( faceSet , 0 , sizeof( char ) * xSliceData.fCount );
}

template< class Real >
template< int FEMDegree , BoundaryType BType , int WeightDegree , int ColorDegree , class Vertex >
void Octree< Real >::getMCIsoSurface( const DenseNodeData< Real , FEMDegree >& solution , Real isoValue , CoredMeshData< Vertex >& mesh , bool nonLinearFit , bool addBarycenter , bool polygonMesh )
{
	if( FEMDegree==1 && nonLinearFit ) fprintf( stderr , "[WARNING] First order B-Splines do not support non-linear interpolation\n" ) , nonLinearFit = false;

	DenseNodeData< Real , FEMDegree > coarseSolution( _sNodesEnd(_maxDepth-1) );
	memset( &coarseSolution[0] , 0 , sizeof(Real)*_sNodesEnd( _maxDepth-1) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(0) ; i<_sNodesEnd(_maxDepth-1) ; i++ ) coarseSolution[i] = solution[i];
	for( LocalDepth d=1 ; d<_maxDepth ; d++ ) _upSample< Real , FEMDegree , BType >( d , coarseSolution );

	std::vector< _Evaluator< FEMDegree , BType > > evaluators( _maxDepth+1 );
	for( LocalDepth d=0 ; d<=_maxDepth ; d++ ) evaluators[d].set( d );

	int vertexOffset = 0;

	std::vector< _SlabValues< Vertex > > slabValues( _maxDepth+1 );

	// Initialize the back slice
	for( LocalDepth d=_maxDepth ; d>=0 ; d-- )
	{
		_sNodes.setSliceTableData ( slabValues[d]. sliceValues(0). sliceData , _localToGlobal( d ) , 0 + _localInset( d ) , threads );
		_sNodes.setSliceTableData ( slabValues[d]. sliceValues(1). sliceData , _localToGlobal( d ) , 1 + _localInset( d ) , threads );
		_sNodes.setXSliceTableData( slabValues[d].xSliceValues(0).xSliceData , _localToGlobal( d ) , 0 + _localInset( d ) , threads );
		slabValues[d].sliceValues (0).reset( nonLinearFit );
		slabValues[d].sliceValues (1).reset( nonLinearFit );
		slabValues[d].xSliceValues(0).reset( );
	}
	for( LocalDepth d=_maxDepth ; d>=0 ; d-- )
	{
		// Copy edges from finer
		if( d<_maxDepth ) _copyFinerSliceIsoEdgeKeys( d , 0 , slabValues , threads );
		_setSliceIsoCorners( solution , coarseSolution , isoValue , d , 0 , slabValues , evaluators[d] , threads );
		_setSliceIsoVertices< WeightDegree , ColorDegree , BType >( isoValue , d , 0 , vertexOffset , mesh , slabValues , threads );
		_setSliceIsoEdges( d , 0 , slabValues , threads );
	}

	// Iterate over the slices at the finest level
	for( int slice=0 ; slice<( 1<<_maxDepth ) ; slice++ )
	{
		// Process at all depths that contain this slice
		LocalDepth d ; int o;
		for( d=_maxDepth , o=slice+1 ; d>=0 ; d-- , o>>=1 )
		{
			// Copy edges from finer (required to ensure we correctly track edge cancellations)
			if( d<_maxDepth )
			{
				_copyFinerSliceIsoEdgeKeys( d , o , slabValues , threads );
				_copyFinerXSliceIsoEdgeKeys( d , o-1 , slabValues , threads );
			}

			// Set the slice values/vertices
			_setSliceIsoCorners( solution , coarseSolution , isoValue , d , o , slabValues , evaluators[d] , threads );
			_setSliceIsoVertices< WeightDegree , ColorDegree , BType >( isoValue , d , o , vertexOffset , mesh , slabValues , threads );
			_setSliceIsoEdges( d , o , slabValues , threads );

			// Set the cross-slice edges
			_setXSliceIsoVertices< WeightDegree , ColorDegree , BType >( isoValue , d , o-1 , vertexOffset , mesh , slabValues , threads );
			_setXSliceIsoEdges( d , o-1 , slabValues , threads );

			// Add the triangles
			_setIsoSurface( d , o-1 , slabValues[d].sliceValues(o-1) , slabValues[d].sliceValues(o) , slabValues[d].xSliceValues(o-1) , mesh , polygonMesh , addBarycenter , vertexOffset , threads );

			if( o&1 ) break;
		}

		for( d=_maxDepth , o=slice+1 ; d>=0 ; d-- , o>>=1 )
		{
			// Initialize for the next pass
			if( o<(1<<(d+1)) )
			{
				_sNodes.setSliceTableData( slabValues[d].sliceValues(o+1).sliceData , _localToGlobal( d ) , o+1 + _localInset( d ) , threads );
				_sNodes.setXSliceTableData( slabValues[d].xSliceValues(o).xSliceData , _localToGlobal( d ) , o + _localInset( d ) , threads );
				slabValues[d].sliceValues(o+1).reset( nonLinearFit );
				slabValues[d].xSliceValues(o).reset();
			}
			if( o&1 ) break;
		}
	}
}


template< class Real >
template< class Vertex , int FEMDegree , BoundaryType BType >
void Octree< Real >::_setSliceIsoCorners( const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , Real isoValue , LocalDepth depth , int slice , std::vector< _SlabValues< Vertex > >& slabValues , const _Evaluator< FEMDegree , BType >& evaluator , int threads )
{
	if( slice>0          ) _setSliceIsoCorners( solution , coarseSolution , isoValue , depth , slice , 1 , slabValues , evaluator , threads );
	if( slice<(1<<depth) ) _setSliceIsoCorners( solution , coarseSolution , isoValue , depth , slice , 0 , slabValues , evaluator , threads );
}
template< class Real >
template< class Vertex , int FEMDegree , BoundaryType BType >
void Octree< Real >::_setSliceIsoCorners( const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , Real isoValue , LocalDepth depth , int slice , int z , std::vector< _SlabValues< Vertex > >& slabValues , const struct _Evaluator< FEMDegree , BType >& evaluator , int threads )
{
	typename Octree::template _SliceValues< Vertex >& sValues = slabValues[depth].sliceValues( slice );
	std::vector< ConstPointSupportKey< FEMDegree > > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,slice-z) ; i<_sNodesEnd(depth,slice-z) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			Real squareValues[ Square::CORNERS ];
			ConstPointSupportKey< FEMDegree >& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* leaf = _sNodes.treeNodes[i];
			if( !IsActiveNode( leaf->children ) )
			{
				const typename SortedTreeNodes::SquareCornerIndices& cIndices = sValues.sliceData.cornerIndices( leaf );

				bool isInterior = _isInteriorlySupported< FEMDegree >( leaf->parent );
				neighborKey.getNeighbors( leaf );

				for( int x=0 ; x<2 ; x++ ) for( int y=0 ; y<2 ; y++ )
					{
						int cc = Cube::CornerIndex( x , y , z );
						int fc = Square::CornerIndex( x , y );
						int vIndex = cIndices[fc];
						if( !sValues.cornerSet[vIndex] )
						{
							if( sValues.cornerGradients )
							{
								std::pair< Real , Point3D< Real > > p = _getCornerValueAndGradient( neighborKey , leaf , cc , solution , coarseSolution , evaluator , isInterior );
								sValues.cornerValues[vIndex] = p.first , sValues.cornerGradients[vIndex] = p.second;
							}
							else sValues.cornerValues[vIndex] = _getCornerValue( neighborKey , leaf , cc , solution , coarseSolution , evaluator , isInterior );
							sValues.cornerSet[vIndex] = 1;
						}
						squareValues[fc] = sValues.cornerValues[ vIndex ];
						TreeOctNode* node = leaf;
						LocalDepth _depth = depth;
						int _slice = slice;
						while( _isValidSpaceNode( node->parent ) && (node-node->parent->children)==cc )
						{
							node = node->parent , _depth-- , _slice >>= 1;
							typename Octree::template _SliceValues< Vertex >& _sValues = slabValues[_depth].sliceValues( _slice );
							const typename SortedTreeNodes::SquareCornerIndices& _cIndices = _sValues.sliceData.cornerIndices( node );
							int _vIndex = _cIndices[fc];
							_sValues.cornerValues[_vIndex] = sValues.cornerValues[vIndex];
							if( _sValues.cornerGradients ) _sValues.cornerGradients[_vIndex] = sValues.cornerGradients[vIndex];
							_sValues.cornerSet[_vIndex] = 1;
						}
					}
				sValues.mcIndices[ i - sValues.sliceData.nodeOffset ] = MarchingSquares::GetIndex( squareValues , isoValue );
			}
		}
}

template< class Real >
template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
void Octree< Real >::_setSliceIsoVertices( Real isoValue , LocalDepth depth , int slice , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	if( slice>0          ) _setSliceIsoVertices< WeightDegree , ColorDegree , BType >( isoValue , depth , slice , 1 , vOffset , mesh , slabValues , threads );
	if( slice<(1<<depth) ) _setSliceIsoVertices< WeightDegree , ColorDegree , BType >( isoValue , depth , slice , 0 , vOffset , mesh , slabValues , threads );
}
template< class Real >
template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
void Octree< Real >::_setSliceIsoVertices( Real isoValue , LocalDepth depth , int slice , int z , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	typename Octree::template _SliceValues< Vertex >& sValues = slabValues[depth].sliceValues( slice );
	// [WARNING] In the case Degree=2, these two keys are the same, so we don't have to maintain them separately.
	std::vector< ConstAdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	std::vector< ConstPointSupportKey< WeightDegree > > weightKeys( std::max< int >( 1 , threads ) );
	std::vector< ConstPointSupportKey< ColorDegree > > colorKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) ) , weightKeys[i].set( _localToGlobal( depth ) ) , colorKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,slice-z) ; i<_sNodesEnd(depth,slice-z) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			ConstAdjacenctNodeKey& neighborKey =  neighborKeys[ get_thread_num() ];
			ConstPointSupportKey< WeightDegree >& weightKey = weightKeys[ get_thread_num() ];
			ConstPointSupportKey< ColorDegree >& colorKey = colorKeys[ get_thread_num() ];
			TreeOctNode* leaf = _sNodes.treeNodes[i];
			if( !IsActiveNode( leaf->children ) )
			{
				int idx = i - sValues.sliceData.nodeOffset;
				const typename SortedTreeNodes::SquareEdgeIndices& eIndices = sValues.sliceData.edgeIndices( leaf );
				if( MarchingSquares::HasRoots( sValues.mcIndices[idx] ) )
				{
					neighborKey.getNeighbors( leaf );
					for( int e=0 ; e<Square::EDGES ; e++ )
						if( MarchingSquares::HasEdgeRoots( sValues.mcIndices[idx] , e ) )
						{
							int vIndex = eIndices[e];
							if( !sValues.edgeSet[vIndex] )
							{
								Vertex vertex;
								int o , y;
								Square::FactorEdgeIndex( e , o , y );
								long long key = VertexData::EdgeIndex( leaf , Cube::EdgeIndex( o , y , z ) , _localToGlobal(_maxDepth) );
								_getIsoVertex< WeightDegree , ColorDegree , BType >( isoValue , weightKey , colorKey , leaf , e , z , sValues , vertex );
								bool stillOwner = false;
								std::pair< int , Vertex > hashed_vertex;
#pragma omp critical (add_point_access)
								{
									if( !sValues.edgeSet[vIndex] )
									{
										mesh.addOutOfCorePoint( vertex );
										sValues.edgeSet[ vIndex ] = 1;
										sValues.edgeKeys[ vIndex ] = key;
										sValues.edgeVertexMap[key] = hashed_vertex = std::pair< int , Vertex >( vOffset , vertex );
										vOffset++;
										stillOwner = true;
									}
								}
								if( stillOwner )
								{
									// We only need to pass the iso-vertex down if the edge it lies on is adjacent to a coarser leaf
									bool isNeeded;
									switch( o )
									{
										case 0: isNeeded = ( !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][2*y][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][2*y][2*z] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][1][2*z] ) ) ; break;
										case 1: isNeeded = ( !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[2*y][1][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[2*y][1][2*z] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][1][2*z] ) ) ; break;
									}
									if( isNeeded )
									{
										int f[2];
										Cube::FacesAdjacentToEdge( Cube::EdgeIndex( o , y , z ) , f[0] , f[1] );
										for( int k=0 ; k<2 ; k++ )
										{
											TreeOctNode* node = leaf;
											LocalDepth _depth = depth;
											int _slice = slice;
											bool _isNeeded = isNeeded;
											while( _isNeeded && _isValidSpaceNode( node->parent ) && Cube::IsFaceCorner( (int)(node-node->parent->children) , f[k] ) )
											{
												node = node->parent , _depth-- , _slice >>= 1;
												typename Octree::template _SliceValues< Vertex >& _sValues = slabValues[_depth].sliceValues( _slice );
#pragma omp critical (add_coarser_point_access)
												_sValues.edgeVertexMap[key] = hashed_vertex;
												switch( o )
												{
													case 0: _isNeeded = ( !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][2*y][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][2*y][2*z] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][1][2*z] ) ) ; break;
													case 1: _isNeeded = ( !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[2*y][1][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[2*y][1][2*z] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][1][2*z] ) ) ; break;
												}
											}
										}
									}
								}
							}
						}
				}
			}
		}
}
template< class Real >
template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
void Octree< Real >::_setXSliceIsoVertices( Real isoValue , LocalDepth depth , int slab , int& vOffset , CoredMeshData< Vertex >& mesh , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	typename Octree::template  _SliceValues< Vertex >& bValues = slabValues[depth].sliceValues ( slab   );
	typename Octree::template  _SliceValues< Vertex >& fValues = slabValues[depth].sliceValues ( slab+1 );
	typename Octree::template _XSliceValues< Vertex >& xValues = slabValues[depth].xSliceValues( slab   );

	// [WARNING] In the case Degree=2, these two keys are the same, so we don't have to maintain them separately.
	std::vector< ConstAdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	std::vector< ConstPointSupportKey< WeightDegree > > weightKeys( std::max< int >( 1 , threads ) );
	std::vector< ConstPointSupportKey< ColorDegree > > colorKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) ) , weightKeys[i].set( _localToGlobal( depth ) ) , colorKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,slab) ; i<_sNodesEnd(depth,slab) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			ConstAdjacenctNodeKey& neighborKey =  neighborKeys[ get_thread_num() ];
			ConstPointSupportKey< WeightDegree >& weightKey = weightKeys[ get_thread_num() ];
			ConstPointSupportKey< ColorDegree >& colorKey = colorKeys[ get_thread_num() ];
			TreeOctNode* leaf = _sNodes.treeNodes[i];
			if( !IsActiveNode( leaf->children ) )
			{
				unsigned char mcIndex = ( bValues.mcIndices[ i - bValues.sliceData.nodeOffset ] ) | ( fValues.mcIndices[ i - fValues.sliceData.nodeOffset ] )<<4;
				const typename SortedTreeNodes::SquareCornerIndices& eIndices = xValues.xSliceData.edgeIndices( leaf );
				if( MarchingCubes::HasRoots( mcIndex ) )
				{
					neighborKey.getNeighbors( leaf );
					for( int x=0 ; x<2 ; x++ ) for( int y=0 ; y<2 ; y++ )
						{
							int c = Square::CornerIndex( x , y );
							int e = Cube::EdgeIndex( 2 , x , y );
							if( MarchingCubes::HasEdgeRoots( mcIndex , e ) )
							{
								int vIndex = eIndices[c];
								if( !xValues.edgeSet[vIndex] )
								{
									Vertex vertex;
									long long key = VertexData::EdgeIndex( leaf , e , _localToGlobal(_maxDepth) );
									_getIsoVertex< WeightDegree , ColorDegree , BType >( isoValue , weightKey , colorKey , leaf , c , bValues , fValues , vertex );
									bool stillOwner = false;
									std::pair< int , Vertex > hashed_vertex;
#pragma omp critical (add_x_point_access)
									{
										if( !xValues.edgeSet[vIndex] )
										{
											mesh.addOutOfCorePoint( vertex );
											xValues.edgeSet[ vIndex ] = 1;
											xValues.edgeKeys[ vIndex ] = key;
											xValues.edgeVertexMap[key] = hashed_vertex = std::pair< int , Vertex >( vOffset , vertex );
											stillOwner = true;
											vOffset++;
										}
									}
									if( stillOwner )
									{
										// We only need to pass the iso-vertex down if the edge it lies on is adjacent to a coarser leaf
										bool isNeeded = ( !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[2*x][1][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[2*x][2*y][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][2*y][1] ) );
										if( isNeeded )
										{
											int f[2];
											Cube::FacesAdjacentToEdge( e , f[0] , f[1] );
											for( int k=0 ; k<2 ; k++ )
											{
												TreeOctNode* node = leaf;
												LocalDepth _depth = depth;
												int _slab = slab;
												bool _isNeeded = isNeeded;
												while( _isNeeded && _isValidSpaceNode( node->parent ) && Cube::IsFaceCorner( (int)(node-node->parent->children) , f[k] ) )
												{
													node = node->parent , _depth-- , _slab >>= 1;
													typename Octree::template _XSliceValues< Vertex >& _xValues = slabValues[_depth].xSliceValues( _slab );
#pragma omp critical (add_x_coarser_point_access)
													_xValues.edgeVertexMap[key] = hashed_vertex;
													_isNeeded = ( !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[2*x][1][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[2*x][2*y][1] ) || !_isValidSpaceNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][2*y][1] ) );
												}
											}
										}
									}
								}
							}
						}
				}
			}
		}
}
template< class Real >
template< class Vertex >
void Octree< Real >::_copyFinerSliceIsoEdgeKeys( LocalDepth depth , int slice , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	if( slice>0          ) _copyFinerSliceIsoEdgeKeys( depth , slice , 1 , slabValues , threads );
	if( slice<(1<<depth) ) _copyFinerSliceIsoEdgeKeys( depth , slice , 0 , slabValues , threads );
}
template< class Real >
template< class Vertex >
void Octree< Real >::_copyFinerSliceIsoEdgeKeys( LocalDepth depth , int slice , int z , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	_SliceValues< Vertex >& pSliceValues = slabValues[depth  ].sliceValues(slice   );
	_SliceValues< Vertex >& cSliceValues = slabValues[depth+1].sliceValues(slice<<1);
	typename SortedTreeNodes::SliceTableData& pSliceData = pSliceValues.sliceData;
	typename SortedTreeNodes::SliceTableData& cSliceData = cSliceValues.sliceData;
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,slice-z) ; i<_sNodesEnd(depth,slice-z) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
			if( IsActiveNode( _sNodes.treeNodes[i]->children ) )
			{
				typename SortedTreeNodes::SquareEdgeIndices& pIndices = pSliceData.edgeIndices( i );
				// Copy the edges that overlap the coarser edges
				for( int orientation=0 ; orientation<2 ; orientation++ ) for( int y=0 ; y<2 ; y++ )
					{
						int fe = Square::EdgeIndex( orientation , y );
						int pIndex = pIndices[fe];
						if( !pSliceValues.edgeSet[ pIndex ] )
						{
							int ce = Cube::EdgeIndex( orientation , y , z );
							int c1 , c2;
							switch( orientation )
							{
								case 0: c1 = Cube::CornerIndex( 0 , y , z ) , c2 = Cube::CornerIndex( 1 , y , z ) ; break;
								case 1: c1 = Cube::CornerIndex( y , 0 , z ) , c2 = Cube::CornerIndex( y , 1 , z ) ; break;
							}
							// [SANITY CHECK]
//					if( _isValidSpaceNode( _sNodes.treeNodes[i]->children + c1 )!=_isValidSpaceNode( _sNodes.treeNodes[i]->children + c2 ) ) fprintf( stderr , "[WARNING] Finer edges should both be valid or invalid\n" ) , exit( 0 );
							if( !_isValidSpaceNode( _sNodes.treeNodes[i]->children + c1 ) || !_isValidSpaceNode( _sNodes.treeNodes[i]->children + c2 ) ) continue;

							int cIndex1 = cSliceData.edgeIndices( _sNodes.treeNodes[i]->children + c1 )[fe];
							int cIndex2 = cSliceData.edgeIndices( _sNodes.treeNodes[i]->children + c2 )[fe];
							if( cSliceValues.edgeSet[cIndex1] != cSliceValues.edgeSet[cIndex2] )
							{
								long long key;
								if( cSliceValues.edgeSet[cIndex1] ) key = cSliceValues.edgeKeys[cIndex1];
								else                                key = cSliceValues.edgeKeys[cIndex2];
								std::pair< int , Vertex > vPair = cSliceValues.edgeVertexMap.find( key )->second;
#pragma omp critical ( copy_finer_edge_keys )
								pSliceValues.edgeVertexMap[key] = vPair;
								pSliceValues.edgeKeys[pIndex] = key;
								pSliceValues.edgeSet[pIndex] = 1;
							}
							else if( cSliceValues.edgeSet[cIndex1] && cSliceValues.edgeSet[cIndex2] )
							{
								long long key1 = cSliceValues.edgeKeys[cIndex1] , key2 = cSliceValues.edgeKeys[cIndex2];
#pragma omp critical ( set_edge_pairs )
								pSliceValues.vertexPairMap[ key1 ] = key2 ,	pSliceValues.vertexPairMap[ key2 ] = key1;

								const TreeOctNode* node = _sNodes.treeNodes[i];
								LocalDepth _depth = depth;
								int _slice = slice;
								while( _isValidSpaceNode( node->parent ) && Cube::IsEdgeCorner( (int)( node - node->parent->children ) , ce ) )
								{
									node = node->parent , _depth-- , _slice >>= 1;
									_SliceValues< Vertex >& _pSliceValues = slabValues[_depth].sliceValues(_slice);
#pragma omp critical ( set_edge_pairs )
									_pSliceValues.vertexPairMap[ key1 ] = key2 , _pSliceValues.vertexPairMap[ key2 ] = key1;
								}
							}
						}
					}
			}
}
template< class Real >
template< class Vertex >
void Octree< Real >::_copyFinerXSliceIsoEdgeKeys( LocalDepth depth , int slab , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	_XSliceValues< Vertex >& pSliceValues  = slabValues[depth  ].xSliceValues(slab);
	_XSliceValues< Vertex >& cSliceValues0 = slabValues[depth+1].xSliceValues( (slab<<1)|0 );
	_XSliceValues< Vertex >& cSliceValues1 = slabValues[depth+1].xSliceValues( (slab<<1)|1 );
	typename SortedTreeNodes::XSliceTableData& pSliceData  = pSliceValues.xSliceData;
	typename SortedTreeNodes::XSliceTableData& cSliceData0 = cSliceValues0.xSliceData;
	typename SortedTreeNodes::XSliceTableData& cSliceData1 = cSliceValues1.xSliceData;
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,slab) ; i<_sNodesEnd(depth,slab) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
			if( IsActiveNode( _sNodes.treeNodes[i]->children ) )
			{
				typename SortedTreeNodes::SquareCornerIndices& pIndices = pSliceData.edgeIndices( i );
				for( int x=0 ; x<2 ; x++ ) for( int y=0 ; y<2 ; y++ )
					{
						int fc = Square::CornerIndex( x , y );
						int pIndex = pIndices[fc];
						if( !pSliceValues.edgeSet[pIndex] )
						{
							int c0 = Cube::CornerIndex( x , y , 0 ) , c1 = Cube::CornerIndex( x , y , 1 );

							// [SANITY CHECK]
//					if( _isValidSpaceNode( _sNodes.treeNodes[i]->children + c0 )!=_isValidSpaceNode( _sNodes.treeNodes[i]->children + c1 ) ) fprintf( stderr , "[ERROR] Finer edges should both be valid or invalid\n" ) , exit( 0 );
							if( !_isValidSpaceNode( _sNodes.treeNodes[i]->children + c0 ) || !_isValidSpaceNode( _sNodes.treeNodes[i]->children + c1 ) ) continue;

							int cIndex0 = cSliceData0.edgeIndices( _sNodes.treeNodes[i]->children + c0 )[fc];
							int cIndex1 = cSliceData1.edgeIndices( _sNodes.treeNodes[i]->children + c1 )[fc];
							if( cSliceValues0.edgeSet[cIndex0] != cSliceValues1.edgeSet[cIndex1] )
							{
								long long key;
								std::pair< int , Vertex > vPair;
								if( cSliceValues0.edgeSet[cIndex0] ) key = cSliceValues0.edgeKeys[cIndex0] , vPair = cSliceValues0.edgeVertexMap.find( key )->second;
								else                                 key = cSliceValues1.edgeKeys[cIndex1] , vPair = cSliceValues1.edgeVertexMap.find( key )->second;
#pragma omp critical ( copy_finer_x_edge_keys )
								pSliceValues.edgeVertexMap[key] = vPair;
								pSliceValues.edgeKeys[ pIndex ] = key;
								pSliceValues.edgeSet[ pIndex ] = 1;
							}
							else if( cSliceValues0.edgeSet[cIndex0] && cSliceValues1.edgeSet[cIndex1] )
							{
								long long key0 = cSliceValues0.edgeKeys[cIndex0] , key1 = cSliceValues1.edgeKeys[cIndex1];
#pragma omp critical ( set_x_edge_pairs )
								pSliceValues.vertexPairMap[ key0 ] = key1 , pSliceValues.vertexPairMap[ key1 ] = key0;
								const TreeOctNode* node = _sNodes.treeNodes[i];
								LocalDepth _depth = depth;
								int _slab = slab , ce = Cube::CornerIndex( 2 , x , y );
								while( _isValidSpaceNode( node->parent ) && Cube::IsEdgeCorner( (int)( node - node->parent->children ) , ce ) )
								{
									node = node->parent , _depth-- , _slab>>= 1;
									_SliceValues< Vertex >& _pSliceValues = slabValues[_depth].sliceValues(_slab);
#pragma omp critical ( set_x_edge_pairs )
									_pSliceValues.vertexPairMap[ key0 ] = key1 , _pSliceValues.vertexPairMap[ key1 ] = key0;
								}
							}
						}
					}
			}
}
template< class Real >
template< class Vertex >
void Octree< Real >::_setSliceIsoEdges( LocalDepth depth , int slice , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	if( slice>0          ) _setSliceIsoEdges( depth , slice , 1 , slabValues , threads );
	if( slice<(1<<depth) ) _setSliceIsoEdges( depth , slice , 0 , slabValues , threads );
}
template< class Real >
template< class Vertex >
void Octree< Real >::_setSliceIsoEdges( LocalDepth depth , int slice , int z , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	typename Octree::template _SliceValues< Vertex >& sValues = slabValues[depth].sliceValues( slice );
	std::vector< ConstAdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth, slice-z) ; i<_sNodesEnd(depth,slice-z) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			int isoEdges[ 2 * MarchingSquares::MAX_EDGES ];
			ConstAdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* leaf = _sNodes.treeNodes[i];
			if( !IsActiveNode( leaf->children ) )
			{
				int idx = i - sValues.sliceData.nodeOffset;
				const typename SortedTreeNodes::SquareEdgeIndices& eIndices = sValues.sliceData.edgeIndices( leaf );
				const typename SortedTreeNodes::SquareFaceIndices& fIndices = sValues.sliceData.faceIndices( leaf );
				unsigned char mcIndex = sValues.mcIndices[idx];
				if( !sValues.faceSet[ fIndices[0] ] )
				{
					neighborKey.getNeighbors( leaf );
					if( !IsActiveNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][1][2*z] ) || !IsActiveNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[1][1][2*z]->children ) )
					{
						_FaceEdges fe;
						fe.count = MarchingSquares::AddEdgeIndices( mcIndex , isoEdges );
						for( int j=0 ; j<fe.count ; j++ ) for( int k=0 ; k<2 ; k++ )
							{
								if( !sValues.edgeSet[ eIndices[ isoEdges[2*j+k] ] ] ) fprintf( stderr , "[ERROR] Edge not set 1: %d / %d\n" , slice , 1<<depth ) , exit( 0 );
								fe.edges[j][k] = sValues.edgeKeys[ eIndices[ isoEdges[2*j+k] ] ];
							}
						sValues.faceSet[ fIndices[0] ] = 1;
						sValues.faceEdges[ fIndices[0] ] = fe;

						TreeOctNode* node = leaf;
						LocalDepth _depth = depth;
						int _slice = slice , f = Cube::FaceIndex( 2 , z );
						std::vector< _IsoEdge > edges;
						edges.resize( fe.count );
						for( int j=0 ; j<fe.count ; j++ ) edges[j] = fe.edges[j];
						while( _isValidSpaceNode( node->parent ) && Cube::IsFaceCorner( (int)(node-node->parent->children) , f ) )
						{
							node = node->parent , _depth-- , _slice >>= 1;
							if( IsActiveNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][1][2*z] ) && IsActiveNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[1][1][2*z]->children ) ) break;
							long long key = VertexData::FaceIndex( node , f , _localToGlobal(_maxDepth) );
#pragma omp critical( add_iso_edge_access )
							{
								typename Octree::template _SliceValues< Vertex >& _sValues = slabValues[_depth].sliceValues( _slice );
								typename std::unordered_map< long long, std::vector< _IsoEdge > >::iterator iter = _sValues.faceEdgeMap.find(key);
								if( iter==_sValues.faceEdgeMap.end() ) _sValues.faceEdgeMap[key] = edges;
								else for( int j=0 ; j<fe.count ; j++ ) iter->second.push_back( fe.edges[j] );
							}
						}
					}
				}
			}
		}
}
template< class Real >
template< class Vertex >
void Octree< Real >::_setXSliceIsoEdges( LocalDepth depth , int slab , std::vector< _SlabValues< Vertex > >& slabValues , int threads )
{
	typename Octree::template  _SliceValues< Vertex >& bValues = slabValues[depth].sliceValues ( slab   );
	typename Octree::template  _SliceValues< Vertex >& fValues = slabValues[depth].sliceValues ( slab+1 );
	typename Octree::template _XSliceValues< Vertex >& xValues = slabValues[depth].xSliceValues( slab   );

	std::vector< ConstAdjacenctNodeKey > neighborKeys( std::max< int >( 1 , threads ) );
	for( size_t i=0 ; i<neighborKeys.size() ; i++ ) neighborKeys[i].set( _localToGlobal( depth ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,slab) ; i<_sNodesEnd(depth,slab) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			int isoEdges[ 2 * MarchingSquares::MAX_EDGES ];
			ConstAdjacenctNodeKey& neighborKey = neighborKeys[ get_thread_num() ];
			TreeOctNode* leaf = _sNodes.treeNodes[i];
			if( !IsActiveNode( leaf->children ) )
			{
				const typename SortedTreeNodes::SquareCornerIndices& cIndices = xValues.xSliceData.edgeIndices( leaf );
				const typename SortedTreeNodes::SquareEdgeIndices& eIndices = xValues.xSliceData.faceIndices( leaf );
				unsigned char mcIndex = ( bValues.mcIndices[ i - bValues.sliceData.nodeOffset ] ) | ( fValues.mcIndices[ i - fValues.sliceData.nodeOffset ]<<4 );
				{
					neighborKey.getNeighbors( leaf );
					for( int o=0 ; o<2 ; o++ ) for( int x=0 ; x<2 ; x++ )
						{
							int e = Square::EdgeIndex( o , x );
							int f = Cube::FaceIndex( 1-o , x );
							unsigned char _mcIndex = MarchingCubes::GetFaceIndex( mcIndex , f );
							int xx = o==1 ? 2*x : 1 , yy = o==0 ? 2*x : 1 , zz = 1;
							if(	!xValues.faceSet[ eIndices[e] ] && ( !IsActiveNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[xx][yy][zz] ) || !IsActiveNode( neighborKey.neighbors[ _localToGlobal( depth ) ].neighbors[xx][yy][zz]->children ) ) )
							{
								_FaceEdges fe;
								fe.count = MarchingSquares::AddEdgeIndices( _mcIndex , isoEdges );
								for( int j=0 ; j<fe.count ; j++ ) for( int k=0 ; k<2 ; k++ )
									{
										int _o , _x;
										Square::FactorEdgeIndex( isoEdges[2*j+k] , _o , _x );
										if( _o==1 ) // Cross-edge
										{
											int idx = o==0 ? cIndices[ Square::CornerIndex(_x,x) ] : cIndices[ Square::CornerIndex(x,_x) ];
											if( !xValues.edgeSet[ idx ] ) fprintf( stderr , "[ERROR] Edge not set 3: %d / %d\n" , slab , 1<<depth ) , exit( 0 );
											fe.edges[j][k] = xValues.edgeKeys[ idx ];
										}
										else
										{
											const typename Octree::template _SliceValues< Vertex >& sValues = (_x==0) ? bValues : fValues;
											int idx = sValues.sliceData.edgeIndices(i)[ Square::EdgeIndex(o,x) ];
											if( !sValues.edgeSet[ idx ] ) fprintf( stderr , "[ERROR] Edge not set 5: %d / %d\n" , slab , 1<<depth ) , exit( 0 );
											fe.edges[j][k] = sValues.edgeKeys[ idx ];
										}
									}
								xValues.faceSet[ eIndices[e] ] = 1;
								xValues.faceEdges[ eIndices[e] ] = fe;

								TreeOctNode* node = leaf;
								LocalDepth _depth = depth;
								int _slab = slab;
								std::vector< _IsoEdge > edges;
								edges.resize( fe.count );
								for( int j=0 ; j<fe.count ; j++ ) edges[j] = fe.edges[j];
								while( _isValidSpaceNode( node->parent ) && Cube::IsFaceCorner( (int)(node-node->parent->children) , f ) )
								{
									node = node->parent , _depth-- , _slab >>= 1;
									if( IsActiveNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[xx][yy][zz] ) && IsActiveNode( neighborKey.neighbors[ _localToGlobal( _depth ) ].neighbors[xx][yy][zz]->children ) ) break;
									long long key = VertexData::FaceIndex( node , f , _localToGlobal(_maxDepth) );
#pragma omp critical( add_x_iso_edge_access )
									{
										typename Octree::template _XSliceValues< Vertex >& _xValues = slabValues[_depth].xSliceValues( _slab );
										typename std::unordered_map< long long, std::vector< _IsoEdge > >::iterator iter = _xValues.faceEdgeMap.find(key);
										if( iter==_xValues.faceEdgeMap.end() ) _xValues.faceEdgeMap[key] = edges;
										else for( int j=0 ; j<fe.count ; j++ ) iter->second.push_back( fe.edges[j] );
									}
								}
							}
						}
				}
			}
		}
}
template< class Real >
template< class Vertex >
void Octree< Real >::_setIsoSurface( LocalDepth depth , int offset , const _SliceValues< Vertex >& bValues , const _SliceValues< Vertex >& fValues , const _XSliceValues< Vertex >& xValues , CoredMeshData< Vertex >& mesh , bool polygonMesh , bool addBarycenter , int& vOffset , int threads )
{
	std::vector< std::pair< int , Vertex > > polygon;
	std::vector< std::vector< _IsoEdge > > edgess( std::max< int >( 1 , threads ) );
#pragma omp parallel for num_threads( threads )
	for( int i=_sNodesBegin(depth,offset) ; i<_sNodesEnd(depth,offset) ; i++ ) if( _isValidSpaceNode( _sNodes.treeNodes[i] ) )
		{
			std::vector< _IsoEdge >& edges = edgess[ get_thread_num() ];
			TreeOctNode* leaf = _sNodes.treeNodes[i];
			int res = 1<<depth;
			LocalDepth d ; LocalOffset off;
			_localDepthAndOffset( leaf , d , off );
			bool inBounds = off[0]>=0 && off[0]<res && off[1]>=0 && off[1]<res && off[2]>=0 && off[2]<res;
			if( inBounds && !IsActiveNode( leaf->children ) )
			{
				edges.clear();
				unsigned char mcIndex = ( bValues.mcIndices[ i - bValues.sliceData.nodeOffset ] ) | ( fValues.mcIndices[ i - fValues.sliceData.nodeOffset ]<<4 );
				// [WARNING] Just because the node looks empty doesn't mean it doesn't get eges from finer neighbors
				{
					// Gather the edges from the faces (with the correct orientation)
					for( int f=0 ; f<Cube::FACES ; f++ )
					{
						int d , o;
						Cube::FactorFaceIndex( f , d , o );
						int flip = d==1 ? 1 : 0; // To account for the fact that the section in y flips the orientation
						if( o ) flip = 1-flip;
						flip = 1-flip; // To get the right orientation
						if( d==2 )
						{
							const _SliceValues< Vertex >& sValues = (o==0) ? bValues : fValues;
							int fIdx = sValues.sliceData.faceIndices(i)[0];
							if( sValues.faceSet[fIdx] )
							{
								const _FaceEdges& fe = sValues.faceEdges[ fIdx ];
								for( int j=0 ; j<fe.count ; j++ ) edges.push_back( _IsoEdge( fe.edges[j][flip] , fe.edges[j][1-flip] ) );
							}
							else
							{
								long long key = VertexData::FaceIndex( leaf , f , _localToGlobal(_maxDepth) );
								typename std::unordered_map< long long, std::vector< _IsoEdge > >::const_iterator iter = sValues.faceEdgeMap.find(key);
								if( iter!=sValues.faceEdgeMap.end() )
								{
									const std::vector< _IsoEdge >& _edges = iter->second;
									for( size_t j=0 ; j<_edges.size() ; j++ ) edges.push_back( _IsoEdge( _edges[j][flip] , _edges[j][1-flip] ) );
								}
								else fprintf( stderr , "[ERROR] Invalid faces: %d  %d %d\n" , i , d , o ) , exit( 0 );
							}
						}
						else
						{
							int fIdx = xValues.xSliceData.faceIndices(i)[ Square::EdgeIndex( 1-d , o ) ];
							if( xValues.faceSet[fIdx] )
							{
								const _FaceEdges& fe = xValues.faceEdges[ fIdx ];
								for( int j=0 ; j<fe.count ; j++ ) edges.push_back( _IsoEdge( fe.edges[j][flip] , fe.edges[j][1-flip] ) );
							}
							else
							{
								long long key = VertexData::FaceIndex( leaf , f , _localToGlobal(_maxDepth) );
								typename std::unordered_map< long long , std::vector< _IsoEdge > >::const_iterator iter = xValues.faceEdgeMap.find(key);
								if( iter!=xValues.faceEdgeMap.end() )
								{
									const std::vector< _IsoEdge >& _edges = iter->second;
									for( size_t j=0 ; j<_edges.size() ; j++ ) edges.push_back( _IsoEdge( _edges[j][flip] , _edges[j][1-flip] ) );
								}
								else fprintf( stderr , "[ERROR] Invalid faces: %d  %d %d\n" , i , d , o ) , exit( 0 );
							}
						}
					}
					// Get the edge loops
					std::vector< std::vector< long long  > > loops;
					while( edges.size() )
					{
						loops.resize( loops.size()+1 );
						_IsoEdge edge = edges.back();
						edges.pop_back();
						long long start = edge[0] , current = edge[1];
						while( current!=start )
						{
							int idx;
							for( idx=0 ; idx<(int)edges.size() ; idx++ ) if( edges[idx][0]==current ) break;
							if( idx==edges.size() )
							{
								typename std::unordered_map< long long, long long >::const_iterator iter;
								if     ( (iter=bValues.vertexPairMap.find(current))!=bValues.vertexPairMap.end() ) loops.back().push_back( current ) , current = iter->second;
								else if( (iter=fValues.vertexPairMap.find(current))!=fValues.vertexPairMap.end() ) loops.back().push_back( current ) , current = iter->second;
								else if( (iter=xValues.vertexPairMap.find(current))!=xValues.vertexPairMap.end() ) loops.back().push_back( current ) , current = iter->second;
								else
								{
									LocalDepth d ; LocalOffset off;
									_localDepthAndOffset( leaf , d , off );
									fprintf( stderr , "[ERROR] Failed to close loop [%d: %d %d %d] | (%d): %lld\n" , d-1 , off[0] , off[1] , off[2] , i , current );
									exit( 0 );
								}
							}
							else
							{
								loops.back().push_back( current );
								current = edges[idx][1];
								edges[idx] = edges.back() , edges.pop_back();
							}
						}
						loops.back().push_back( start );
					}
					// Add the loops to the mesh
					for( size_t j=0 ; j<loops.size() ; j++ )
					{
						std::vector< std::pair< int , Vertex > > polygon( loops[j].size() );
						for( size_t k=0 ; k<loops[j].size() ; k++ )
						{
							long long key = loops[j][k];
							typename std::unordered_map< long long, std::pair< int, Vertex > >::const_iterator iter;
							if     ( ( iter=bValues.edgeVertexMap.find( key ) )!=bValues.edgeVertexMap.end() ) polygon[k] = iter->second;
							else if( ( iter=fValues.edgeVertexMap.find( key ) )!=fValues.edgeVertexMap.end() ) polygon[k] = iter->second;
							else if( ( iter=xValues.edgeVertexMap.find( key ) )!=xValues.edgeVertexMap.end() ) polygon[k] = iter->second;
							else fprintf( stderr , "[ERROR] Couldn't find vertex in edge map\n" ) , exit( 0 );
						}
						_addIsoPolygons( mesh , polygon , polygonMesh , addBarycenter , vOffset );
					}
				}
			}
		}
}

template< class Real > void SetIsoVertex(              PlyVertex< float  >& vertex , Point3D< Real > color , Real value ){ ; }

template< class Real >
template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
bool Octree< Real >::_getIsoVertex( Real isoValue , ConstPointSupportKey< WeightDegree >& weightKey , ConstPointSupportKey< ColorDegree >& colorKey , const TreeOctNode* node , int edgeIndex , int z , const _SliceValues< Vertex >& sValues , Vertex& vertex )
{
	Point3D< Real > position;
	int c0 , c1;
	Square::EdgeCorners( edgeIndex , c0 , c1 );

	bool nonLinearFit = sValues.cornerGradients!=NullPointer( Point3D< Real > );
	const typename SortedTreeNodes::SquareCornerIndices& idx = sValues.sliceData.cornerIndices( node );
	Real x0 = sValues.cornerValues[idx[c0]] , x1 = sValues.cornerValues[idx[c1]];
	Point3D< Real > s;
	Real start , width;
	_startAndWidth( node , s , width );
	int o , y;
	Square::FactorEdgeIndex( edgeIndex , o , y );
	start = s[o];
	switch( o )
	{
		case 0:
			position[1] = s[1] + width*y;
			position[2] = s[2] + width*z;
			break;
		case 1:
			position[0] = s[0] + width*y;
			position[2] = s[2] + width*z;
			break;
	}

	double averageRoot;
	bool rootFound = false;
	if( nonLinearFit )
	{
		double dx0 = sValues.cornerGradients[idx[c0]][o] * width , dx1 = sValues.cornerGradients[idx[c1]][o] * width;

		// The scaling will turn the Hermite Spline into a quadratic
		double scl = (x1-x0) / ( (dx1+dx0 ) / 2 );
		dx0 *= scl , dx1 *= scl;

		// Hermite Spline
		Polynomial< 2 > P;
		P.coefficients[0] = x0;
		P.coefficients[1] = dx0;
		P.coefficients[2] = 3*(x1-x0)-dx1-2*dx0;

		double roots[2];
		int rCount = 0 , rootCount = P.getSolutions( isoValue , roots , 0 );
		averageRoot = 0;
		for( int i=0 ; i<rootCount ; i++ ) if( roots[i]>=0 && roots[i]<=1 ) averageRoot += roots[i] , rCount++;
		if( rCount ) rootFound = true;
		averageRoot /= rCount;
	}
	if( !rootFound )
	{
		// We have a linear function L, with L(0) = x0 and L(1) = x1
		// => L(t) = x0 + t * (x1-x0)
		// => L(t) = isoValue <=> t = ( isoValue - x0 ) / ( x1 - x0 )
		if( x0==x1 ) fprintf( stderr , "[ERROR] Not a zero-crossing root: %g %g\n" , x0 , x1 ) , exit( 0 );
		averageRoot = ( isoValue - x0 ) / ( x1 - x0 );
	}
	if( averageRoot<0 || averageRoot>1 )
	{
		fprintf( stderr , "[WARNING] Bad average root: %f\n" , averageRoot );
		fprintf( stderr , "\t(%f %f) (%f)\n" , x0 , x1 , isoValue );
		if( averageRoot<0 ) averageRoot = 0;
		if( averageRoot>1 ) averageRoot = 1;
	}
	position[o] = Real( start + width*averageRoot );
	vertex.point = position;
	Point3D< Real > color;
	Real depth(0);
	SetIsoVertex( vertex , color , depth );
	return true;
}
template< class Real >
template< int WeightDegree , int ColorDegree , BoundaryType BType , class Vertex >
bool Octree< Real >::_getIsoVertex( Real isoValue , ConstPointSupportKey< WeightDegree >& weightKey , ConstPointSupportKey< ColorDegree >& colorKey , const TreeOctNode* node , int cornerIndex , const _SliceValues< Vertex >& bValues , const _SliceValues< Vertex >& fValues , Vertex& vertex )
{
	Point3D< Real > position;

	bool nonLinearFit = bValues.cornerGradients!=NullPointer( Point3D< Real > ) && fValues.cornerGradients!=NullPointer( Point3D< Real > );
	const typename SortedTreeNodes::SquareCornerIndices& idx0 = bValues.sliceData.cornerIndices( node );
	const typename SortedTreeNodes::SquareCornerIndices& idx1 = fValues.sliceData.cornerIndices( node );
	Real x0 = bValues.cornerValues[ idx0[cornerIndex] ] , x1 = fValues.cornerValues[ idx1[cornerIndex] ];
	Point3D< Real > s;
	Real start , width;
	_startAndWidth( node , s , width );
	start = s[2];
	int x , y;
	Square::FactorCornerIndex( cornerIndex , x , y );


	position[0] = s[0] + width*x;
	position[1] = s[1] + width*y;

	double averageRoot;

	bool rootFound = false;
	if( nonLinearFit )
	{
		double dx0 = bValues.cornerGradients[ idx0[cornerIndex] ][2] * width , dx1 = fValues.cornerGradients[ idx1[cornerIndex] ][2] * width;
		// The scaling will turn the Hermite Spline into a quadratic
		double scl = (x1-x0) / ( (dx1+dx0 ) / 2 );
		dx0 *= scl , dx1 *= scl;

		// Hermite Spline
		Polynomial< 2 > P;
		P.coefficients[0] = x0;
		P.coefficients[1] = dx0;
		P.coefficients[2] = 3*(x1-x0)-dx1-2*dx0;

		double roots[2];
		int rCount = 0 , rootCount = P.getSolutions( isoValue , roots , 0 );
		averageRoot = 0;
		for( int i=0 ; i<rootCount ; i++ ) if( roots[i]>=0 && roots[i]<=1 ) averageRoot += roots[i] , rCount++;
		if( rCount ) rootFound = true;
		averageRoot /= rCount;
	}
	if( !rootFound )
	{
		// We have a linear function L, with L(0) = x0 and L(1) = x1
		// => L(t) = x0 + t * (x1-x0)
		// => L(t) = isoValue <=> t = ( isoValue - x0 ) / ( x1 - x0 )
		if( x0==x1 ) fprintf( stderr , "[ERROR] Not a zero-crossing root: %g %g\n" , x0 , x1 ) , exit( 0 );
		averageRoot = ( isoValue - x0 ) / ( x1 - x0 );
	}
	if( averageRoot<0 || averageRoot>1 )
	{
		fprintf( stderr , "[WARNING] Bad average root: %f\n" , averageRoot );
		fprintf( stderr , "\t(%f %f) (%f)\n" , x0 , x1 , isoValue );
		if( averageRoot<0 ) averageRoot = 0;
		if( averageRoot>1 ) averageRoot = 1;
	}
	position[2] = Real( start + width*averageRoot );
	vertex.point = position;
	Point3D< Real > color;
	Real depth(0);
	SetIsoVertex( vertex , color , depth );
	return true;
}

template< class Real >
template< class Vertex >
int Octree< Real >::_addIsoPolygons( CoredMeshData< Vertex >& mesh , std::vector< std::pair< int , Vertex > >& polygon , bool polygonMesh , bool addBarycenter , int& vOffset )
{
	if( polygonMesh )
	{
		std::vector< int > vertices( polygon.size() );
		for( int i=0 ; i<(int)polygon.size() ; i++ ) vertices[i] = polygon[polygon.size()-1-i].first;
		mesh.addPolygon_s( vertices );
		return 1;
	}
	if( polygon.size()>3 )
	{
		bool isCoplanar = false;
		std::vector< int > triangle( 3 );

		if( addBarycenter )
			for( int i=0 ; i<(int)polygon.size() ; i++ )
				for( int j=0 ; j<i ; j++ )
					if( (i+1)%polygon.size()!=j && (j+1)%polygon.size()!=i )
					{
						Vertex v1 = polygon[i].second , v2 = polygon[j].second;
						for( int k=0 ; k<3 ; k++ ) if( v1.point[k]==v2.point[k] ) isCoplanar = true;
					}
		if( isCoplanar )
		{
			Vertex c;
			typename Vertex::Wrapper _c;
			_c *= 0;
			for( int i=0 ; i<(int)polygon.size() ; i++ ) _c += typename Vertex::Wrapper( polygon[i].second );
			_c /= Real( polygon.size() );
			c = Vertex( _c );
			int cIdx;
#pragma omp critical (add_barycenter_point_access)
			{
				cIdx = mesh.addOutOfCorePoint( c );
				vOffset++;
			}
			for( int i=0 ; i<(int)polygon.size() ; i++ )
			{
				triangle[0] = polygon[ i                  ].first;
				triangle[1] = cIdx;
				triangle[2] = polygon[(i+1)%polygon.size()].first;
				mesh.addPolygon_s( triangle );
			}
			return (int)polygon.size();
		}
		else
		{
			MinimalAreaTriangulation< Real > MAT;
			std::vector< Point3D< Real > > vertices;
			std::vector< TriangleIndex > triangles;
			vertices.resize( polygon.size() );
			// Add the points
			for( int i=0 ; i<(int)polygon.size() ; i++ ) vertices[i] = polygon[i].second.point;
			MAT.GetTriangulation( vertices , triangles );
			for( int i=0 ; i<(int)triangles.size() ; i++ )
			{
				for( int j=0 ; j<3 ; j++ ) triangle[2-j] = polygon[ triangles[i].idx[j] ].first;
				mesh.addPolygon_s( triangle );
			}
		}
	}
	else if( polygon.size()==3 )
	{
		std::vector< int > vertices( 3 );
		for( int i=0 ; i<3 ; i++ ) vertices[2-i] = polygon[i].first;
		mesh.addPolygon_s( vertices );
	}
	return (int)polygon.size()-2;
}


template< class Real >
template< int FEMDegree , BoundaryType BType>
void Octree< Real >::_Evaluator< FEMDegree , BType >::set( LocalDepth depth )
{
	static const int  LeftPointSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;

	BSplineEvaluationData< FEMDegree , BType >::SetEvaluator( evaluator , depth );
	if( depth>0 ) BSplineEvaluationData< FEMDegree , BType >::SetChildEvaluator( childEvaluator , depth-1 );
	int center = ( 1<<depth )>>1;

	// First set the stencils for the current depth
	for( int x=-LeftPointSupportRadius ; x<=RightPointSupportRadius ; x++ ) for( int y=-LeftPointSupportRadius ; y<=RightPointSupportRadius ; y++ ) for( int z=-LeftPointSupportRadius ; z<=RightPointSupportRadius ; z++ )
			{
				int fIdx[] = { center+x , center+y , center+z };

				// The cell stencil
				{
					double vv[3] , dv[3];
					for( int dd=0 ; dd<DIMENSION ; dd++ )
					{
						vv[dd] = evaluator.centerValue( fIdx[dd] , center , false );
						dv[dd] = evaluator.centerValue( fIdx[dd] , center , true  );
					}
					cellStencil( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
					dCellStencil( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
				}

				//// The face stencil
				for( int f=0 ; f<Cube::FACES ; f++ )
				{
					int dir , off;
					Cube::FactorFaceIndex( f , dir , off );
					double vv[3] , dv[3];
					switch( dir )
					{
						case 0:
							vv[0] = evaluator.cornerValue( fIdx[0] , center+off , false );
							vv[1] = evaluator.centerValue( fIdx[1] , center     , false );
							vv[2] = evaluator.centerValue( fIdx[2] , center     , false );
							dv[0] = evaluator.cornerValue( fIdx[0] , center+off , true  );
							dv[1] = evaluator.centerValue( fIdx[1] , center     , true  );
							dv[2] = evaluator.centerValue( fIdx[2] , center     , true  );
							break;
						case 1:
							vv[0] = evaluator.centerValue( fIdx[0] , center     , false );
							vv[1] = evaluator.cornerValue( fIdx[1] , center+off , false );
							vv[2] = evaluator.centerValue( fIdx[2] , center     , false );
							dv[0] = evaluator.centerValue( fIdx[0] , center     , true  );
							dv[1] = evaluator.cornerValue( fIdx[1] , center+off , true  );
							dv[2] = evaluator.centerValue( fIdx[2] , center     , true  );
							break;
						case 2:
							vv[0] = evaluator.centerValue( fIdx[0] , center     , false );
							vv[1] = evaluator.centerValue( fIdx[1] , center     , false );
							vv[2] = evaluator.cornerValue( fIdx[2] , center+off , false );
							dv[0] = evaluator.centerValue( fIdx[0] , center     , true  );
							dv[1] = evaluator.centerValue( fIdx[1] , center     , true  );
							dv[2] = evaluator.cornerValue( fIdx[2] , center+off , true  );
							break;
					}
					faceStencil[f]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
					dFaceStencil[f]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
				}

				//// The edge stencil
				for( int e=0 ; e<Cube::EDGES ; e++ )
				{
					int orientation , i1 , i2;
					Cube::FactorEdgeIndex( e , orientation , i1 , i2 );
					double vv[3] , dv[3];
					switch( orientation )
					{
						case 0:
							vv[0] = evaluator.centerValue( fIdx[0] , center    , false );
							vv[1] = evaluator.cornerValue( fIdx[1] , center+i1 , false );
							vv[2] = evaluator.cornerValue( fIdx[2] , center+i2 , false );
							dv[0] = evaluator.centerValue( fIdx[0] , center    , true  );
							dv[1] = evaluator.cornerValue( fIdx[1] , center+i1 , true  );
							dv[2] = evaluator.cornerValue( fIdx[2] , center+i2 , true  );
							break;
						case 1:
							vv[0] = evaluator.cornerValue( fIdx[0] , center+i1 , false );
							vv[1] = evaluator.centerValue( fIdx[1] , center    , false );
							vv[2] = evaluator.cornerValue( fIdx[2] , center+i2 , false );
							dv[0] = evaluator.cornerValue( fIdx[0] , center+i1 , true  );
							dv[1] = evaluator.centerValue( fIdx[1] , center    , true  );
							dv[2] = evaluator.cornerValue( fIdx[2] , center+i2 , true  );
							break;
						case 2:
							vv[0] = evaluator.cornerValue( fIdx[0] , center+i1 , false );
							vv[1] = evaluator.cornerValue( fIdx[1] , center+i2 , false );
							vv[2] = evaluator.centerValue( fIdx[2] , center    , false );
							dv[0] = evaluator.cornerValue( fIdx[0] , center+i1 , true  );
							dv[1] = evaluator.cornerValue( fIdx[1] , center+i2 , true  );
							dv[2] = evaluator.centerValue( fIdx[2] , center    , true  );
							break;
					}
					edgeStencil[e]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
					dEdgeStencil[e]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
				}

				//// The corner stencil
				for( int c=0 ; c<Cube::CORNERS ; c++ )
				{
					int cx , cy  ,cz;
					Cube::FactorCornerIndex( c , cx , cy , cz );
					double vv[3] , dv[3];
					vv[0] = evaluator.cornerValue( fIdx[0] , center+cx , false );
					vv[1] = evaluator.cornerValue( fIdx[1] , center+cy , false );
					vv[2] = evaluator.cornerValue( fIdx[2] , center+cz , false );
					dv[0] = evaluator.cornerValue( fIdx[0] , center+cx , true  );
					dv[1] = evaluator.cornerValue( fIdx[1] , center+cy , true  );
					dv[2] = evaluator.cornerValue( fIdx[2] , center+cz , true  );
					cornerStencil[c]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
					dCornerStencil[c]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
				}
			}

	// Now set the stencils for the parents
	for( int child=0 ; child<CHILDREN ; child++ )
	{
		int childX , childY , childZ;
		Cube::FactorCornerIndex( child , childX , childY , childZ );
		for( int x=-LeftPointSupportRadius ; x<=RightPointSupportRadius ; x++ ) for( int y=-LeftPointSupportRadius ; y<=RightPointSupportRadius ; y++ ) for( int z=-LeftPointSupportRadius ; z<=RightPointSupportRadius ; z++ )
				{
					int fIdx[] = { center/2+x , center/2+y , center/2+z };

					//// The cell stencil
					{
						double vv[3] , dv[3];
						vv[0] = childEvaluator.centerValue( fIdx[0] , center+childX , false );
						vv[1] = childEvaluator.centerValue( fIdx[1] , center+childY , false );
						vv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ , false );
						dv[0] = childEvaluator.centerValue( fIdx[0] , center+childX , true  );
						dv[1] = childEvaluator.centerValue( fIdx[1] , center+childY , true  );
						dv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ , true  );
						cellStencils[child]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
						dCellStencils[child]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
					}

					//// The face stencil
					for( int f=0 ; f<Cube::FACES ; f++ )
					{
						int dir , off;
						Cube::FactorFaceIndex( f , dir , off );
						double vv[3] , dv[3];
						switch( dir )
						{
							case 0:
								vv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+off , false );
								vv[1] = childEvaluator.centerValue( fIdx[1] , center+childY     , false );
								vv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ     , false );
								dv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+off , true  );
								dv[1] = childEvaluator.centerValue( fIdx[1] , center+childY     , true  );
								dv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ     , true  );
								break;
							case 1:
								vv[0] = childEvaluator.centerValue( fIdx[0] , center+childX     , false );
								vv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+off , false );
								vv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ     , false );
								dv[0] = childEvaluator.centerValue( fIdx[0] , center+childX     , true  );
								dv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+off , true  );
								dv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ     , true  );
								break;
							case 2:
								vv[0] = childEvaluator.centerValue( fIdx[0] , center+childX     , false );
								vv[1] = childEvaluator.centerValue( fIdx[1] , center+childY     , false );
								vv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+off , false );
								dv[0] = childEvaluator.centerValue( fIdx[0] , center+childX     , true  );
								dv[1] = childEvaluator.centerValue( fIdx[1] , center+childY     , true  );
								dv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+off , true  );
								break;
						}
						faceStencils[child][f]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
						dFaceStencils[child][f]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
					}

					//// The edge stencil
					for( int e=0 ; e<Cube::EDGES ; e++ )
					{
						int orientation , i1 , i2;
						Cube::FactorEdgeIndex( e , orientation , i1 , i2 );
						double vv[3] , dv[3];
						switch( orientation )
						{
							case 0:
								vv[0] = childEvaluator.centerValue( fIdx[0] , center+childX    , false );
								vv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+i1 , false );
								vv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+i2 , false );
								dv[0] = childEvaluator.centerValue( fIdx[0] , center+childX    , true  );
								dv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+i1 , true  );
								dv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+i2 , true  );
								break;
							case 1:
								vv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+i1 , false );
								vv[1] = childEvaluator.centerValue( fIdx[1] , center+childY    , false );
								vv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+i2 , false );
								dv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+i1 , true  );
								dv[1] = childEvaluator.centerValue( fIdx[1] , center+childY    , true  );
								dv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+i2 , true  );
								break;
							case 2:
								vv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+i1 , false );
								vv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+i2 , false );
								vv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ    , false );
								dv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+i1 , true  );
								dv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+i2 , true  );
								dv[2] = childEvaluator.centerValue( fIdx[2] , center+childZ    , true  );
								break;
						}
						edgeStencils[child][e]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
						dEdgeStencils[child][e]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
					}

					//// The corner stencil
					for( int c=0 ; c<Cube::CORNERS ; c++ )
					{
						int cx , cy  ,cz;
						Cube::FactorCornerIndex( c , cx , cy , cz );
						double vv[3] , dv[3];
						vv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+cx , false );
						vv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+cy , false );
						vv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+cz , false );
						dv[0] = childEvaluator.cornerValue( fIdx[0] , center+childX+cx , true  );
						dv[1] = childEvaluator.cornerValue( fIdx[1] , center+childY+cy , true  );
						dv[2] = childEvaluator.cornerValue( fIdx[2] , center+childZ+cz , true  );
						cornerStencils[child][c]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = vv[0] * vv[1] * vv[2];
						dCornerStencils[child][c]( x+LeftPointSupportRadius , y+LeftPointSupportRadius , z+LeftPointSupportRadius ) = Point3D< double >( dv[0] * vv[1] * vv[2] , vv[0] * dv[1] * vv[2] , vv[0] * vv[1] * dv[2] );
					}
				}
	}
	if( _bsData ) delete _bsData;
	_bsData = new BSplineData< FEMDegree , BType >( depth );
}
template< class Real >
template< class V , int FEMDegree , BoundaryType BType >
V Octree< Real >::_getValue( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , Point3D< Real > p , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int  LeftPointSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;

	if( IsActiveNode( node->children ) ) fprintf( stderr , "[WARNING] getValue assumes leaf node\n" );
	V value(0);

	while( GetGhostFlag( node ) )
	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );

		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* _n = neighbors.neighbors[i][j][k];

					if( _isValidFEMNode( _n ) )
					{
						int _pIdx[3];
						Point3D< Real > _s ; Real _w;
						_startAndWidth( _n , _s , _w );
						int _fIdx[3];
						functionIndex< FEMDegree , BType >( _n , _fIdx );
						for( int dd=0 ; dd<3 ; dd++ ) _pIdx[dd] = std::max< int >( 0 , std::min< int >( SupportSize-1 , LeftSupportRadius + (int)floor( ( p[dd]-_s[dd] ) / _w ) ) );
						value +=
								solution[ _n->nodeData.nodeIndex ] *
								(Real)
										(
												evaluator._bsData->baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) *
												evaluator._bsData->baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) *
												evaluator._bsData->baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
										);
					}
				}
		node = node->parent;
	}

	LocalDepth d = _localDepth( node );

	for( int dd=0 ; dd<3 ; dd++ )
		if     ( p[dd]==0 ) p[dd] = (Real)(0.+1e-6);
		else if( p[dd]==1 ) p[dd] = (Real)(1.-1e-6);

	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );

		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* _n = neighbors.neighbors[i][j][k];
					if( _isValidFEMNode( _n ) )
					{
						int _pIdx[3];
						Point3D< Real > _s ; Real _w;
						_startAndWidth( _n , _s , _w );
						int _fIdx[3];
						functionIndex< FEMDegree , BType >( _n , _fIdx );
						for( int dd=0 ; dd<3 ; dd++ ) _pIdx[dd] = std::max< int >( 0 , std::min< int >( SupportSize-1 , LeftSupportRadius + (int)floor( ( p[dd]-_s[dd] ) / _w ) ) );
						value +=
								solution[ _n->nodeData.nodeIndex ] *
								(Real)
										(
												evaluator._bsData->baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) *
												evaluator._bsData->baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) *
												evaluator._bsData->baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
										);
					}
				}
		if( d>0 )
		{
			const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
			for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
					{
						const TreeOctNode* _n = neighbors.neighbors[i][j][k];
						if( _isValidFEMNode( _n ) )
						{
							int _pIdx[3];
							Point3D< Real > _s ; Real _w;
							_startAndWidth( _n , _s , _w );
							int _fIdx[3];
							functionIndex< FEMDegree , BType >( _n , _fIdx );
							for( int dd=0 ; dd<3 ; dd++ ) _pIdx[dd] = std::max< int >( 0 , std::min< int >( SupportSize-1 , LeftSupportRadius + (int)floor( ( p[dd]-_s[dd] ) / _w ) ) );
							value +=
									coarseSolution[ _n->nodeData.nodeIndex ] *
									(Real)
											(
													evaluator._bsData->baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) *
													evaluator._bsData->baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) *
													evaluator._bsData->baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
											);
						}
					}
		}
	}
	return value;
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
std::pair< Real , Point3D< Real > > Octree< Real >::_getValueAndGradient( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , Point3D< Real > p , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftSupportRadius = -BSplineSupportSizes< FEMDegree >::SupportStart;
	static const int RightSupportRadius =  BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int  LeftPointSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;

	if( IsActiveNode( node->children ) ) fprintf( stderr , "[WARNING] _getValueAndGradient assumes leaf node\n" );
	Real value(0);
	Point3D< Real > gradient;

	while( GetGhostFlag( node ) )
	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );

		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* _n = neighbors.neighbors[i][j][k];

					if( _isValidFEMNode( _n ) )
					{
						int _pIdx[3];
						Point3D< Real > _s; Real _w;
						_startAndWidth( _n , _s , _w );
						int _fIdx[3];
						functionIndex< FEMDegree , BType >( _n , _fIdx );
						for( int dd=0 ; dd<3 ; dd++ ) _pIdx[dd] = std::max< int >( 0 , std::min< int >( SupportSize-1 , LeftSupportRadius + (int)floor( ( p[dd]-_s[dd] ) / _w ) ) );
						value +=
								solution[ _n->nodeData.nodeIndex ] *
								(Real)
										(
												evaluator._bsData->baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData->baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData->baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
										);
						gradient +=
								Point3D< Real >
										(
												evaluator._bsData->dBaseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData-> baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData-> baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] ) ,
												evaluator._bsData-> baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData->dBaseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData-> baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] ) ,
												evaluator._bsData-> baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData-> baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData->dBaseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
										) * solution[ _n->nodeData.nodeIndex ];
					}
				}
		node = node->parent;
	}


	LocalDepth d = _localDepth( node );

	for( int dd=0 ; dd<3 ; dd++ )
		if     ( p[dd]==0 ) p[dd] = (Real)(0.+1e-6);
		else if( p[dd]==1 ) p[dd] = (Real)(1.-1e-6);

	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );

		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* _n = neighbors.neighbors[i][j][k];

					if( _isValidFEMNode( _n ) )
					{
						int _pIdx[3];
						Point3D< Real > _s ; Real _w;
						_startAndWidth( _n , _s , _w );
						int _fIdx[3];
						functionIndex< FEMDegree , BType >( _n , _fIdx );
						for( int dd=0 ; dd<3 ; dd++ ) _pIdx[dd] = std::max< int >( 0 , std::min< int >( SupportSize-1 , LeftSupportRadius + (int)floor( ( p[dd]-_s[dd] ) / _w ) ) );
						value +=
								solution[ _n->nodeData.nodeIndex ] *
								(Real)
										(
												evaluator._bsData->baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData->baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData->baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
										);
						gradient +=
								Point3D< Real >
										(
												evaluator._bsData->dBaseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData-> baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData-> baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] ) ,
												evaluator._bsData-> baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData->dBaseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData-> baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] ) ,
												evaluator._bsData-> baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData-> baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData->dBaseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
										) * solution[ _n->nodeData.nodeIndex ];
					}
				}
		if( d>0 )
		{
			const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
			for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
					{
						const TreeOctNode* _n = neighbors.neighbors[i][j][k];

						if( _isValidFEMNode( _n ) )
						{
							int _pIdx[3];
							Point3D< Real > _s ; Real _w;
							_startAndWidth( _n , _s , _w );
							int _fIdx[3];
							functionIndex< FEMDegree , BType >( _n , _fIdx );
							for( int dd=0 ; dd<3 ; dd++ ) _pIdx[dd] = std::max< int >( 0 , std::min< int >( SupportSize-1 , LeftSupportRadius + (int)floor( ( p[dd]-_s[dd] ) / _w ) ) );
							value +=
									coarseSolution[ _n->nodeData.nodeIndex ] *
									(Real)
											(
													evaluator._bsData->baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData->baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData->baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
											);
							gradient +=
									Point3D< Real >
											(
													evaluator._bsData->dBaseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData-> baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData-> baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] ) ,
													evaluator._bsData-> baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData->dBaseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData-> baseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] ) ,
													evaluator._bsData-> baseBSplines[ _fIdx[0] ][ _pIdx[0] ]( p[0] ) * evaluator._bsData-> baseBSplines[ _fIdx[1] ][ _pIdx[1] ]( p[1] ) * evaluator._bsData->dBaseBSplines[ _fIdx[2] ][ _pIdx[2] ]( p[2] )
											) * coarseSolution[ _n->nodeData.nodeIndex ];
						}
					}
		}
	}
	return std::pair< Real , Point3D< Real > >( value , gradient );
}
template< class Real >
template< class V , int FEMDegree , BoundaryType BType >
V Octree< Real >::_getCenterValue( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const
{
	static const int SupportSize = BSplineEvaluationData< FEMDegree , BType >::SupportSize;
	static const int  LeftPointSupportRadius =   BSplineEvaluationData< FEMDegree , BType >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineEvaluationData< FEMDegree , BType >::SupportStart;

	if( IsActiveNode( node->children ) ) fprintf( stderr , "[WARNING] getCenterValue assumes leaf node\n" );
	V value(0);
	LocalDepth d = _localDepth( node );

	if( isInterior )
	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );
		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* n = neighbors.neighbors[i][j][k];
					if( IsActiveNode( n ) ) value += solution[ n->nodeData.nodeIndex ] * Real( evaluator.cellStencil( i , j , k ) );
				}
		if( d>0 )
		{
			int _corner = int( node - node->parent->children );
			const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
			for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
					{
						const TreeOctNode* n = neighbors.neighbors[i][j][k];
						if( IsActiveNode( n ) ) value += coarseSolution[n->nodeData.nodeIndex] * Real( evaluator.cellStencils[_corner]( i , j , k ) );
					}
		}
	}
	else
	{
		LocalOffset cIdx;
		_localDepthAndOffset( node , d , cIdx );
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );

		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* n = neighbors.neighbors[i][j][k];

					if( _isValidFEMNode( n ) )
					{
						LocalDepth _d ; LocalOffset fIdx;
						_localDepthAndOffset( n , _d , fIdx );
						value +=
								solution[ n->nodeData.nodeIndex ] *
								Real(
										evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , false ) *
										evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , false ) *
										evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , false )
								);
					}
				}
		if( d>0 )
		{
			const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
			for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
					{
						const TreeOctNode* n = neighbors.neighbors[i][j][k];
						if( _isValidFEMNode( n ) )
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( n , _d , fIdx );
							value +=
									coarseSolution[ n->nodeData.nodeIndex ] *
									Real(
											evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , false ) *
											evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , false ) *
											evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , false )
									);
						}
					}
		}
	}
	return value;
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
std::pair< Real , Point3D< Real > > Octree< Real >::_getCenterValueAndGradient( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const
{
	static const int SupportSize = BSplineEvaluationData< FEMDegree , BType >::SupportSize;
	static const int  LeftPointSupportRadius =   BSplineEvaluationData< FEMDegree , BType >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineEvaluationData< FEMDegree , BType >::SupportStart;

	if( IsActiveNode( node->children ) ) fprintf( stderr , "[WARNING] getCenterValueAndGradient assumes leaf node\n" );
	Real value(0);
	Point3D< Real > gradient;
	LocalDepth d = _localDepth( node );

	if( isInterior )
	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );
		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* n = neighbors.neighbors[i][j][k];
					if( IsActiveNode( n ) )
					{
						value    +=          Real  ( evaluator. cellStencil( i , j , k ) ) * solution[ n->nodeData.nodeIndex ];
						gradient += Point3D< Real >( evaluator.dCellStencil( i , j , k ) ) * solution[ n->nodeData.nodeIndex ];
					}
				}
		if( d>0 )
		{
			int _corner = int( node - node->parent->children );
			const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
			for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
					{
						const TreeOctNode* n = neighbors.neighbors[i][j][k];
						if( IsActiveNode( n ) )
						{
							value    +=          Real  ( evaluator. cellStencils[_corner]( i , j , k ) ) * coarseSolution[n->nodeData.nodeIndex];
							gradient += Point3D< Real >( evaluator.dCellStencils[_corner]( i , j , k ) ) * coarseSolution[n->nodeData.nodeIndex];
						}
					}
		}
	}
	else
	{
		LocalOffset cIdx;
		_localDepthAndOffset( node , d , cIdx );
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );

		for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
				{
					const TreeOctNode* n = neighbors.neighbors[i][j][k];

					if( _isValidFEMNode( n ) )
					{
						LocalDepth _d ; LocalOffset fIdx;
						_localDepthAndOffset( n , _d , fIdx );
						value +=
								Real
										(
												evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , false ) * evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , false ) * evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , false )
										) * solution[ n->nodeData.nodeIndex ];
						gradient +=
								Point3D< Real >
										(
												evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , true  ) * evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , false ) * evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , false ) ,
												evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , false ) * evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , true  ) * evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , false ) ,
												evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , false ) * evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , false ) * evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , true  )
										) * solution[ n->nodeData.nodeIndex ];
					}
				}
		if( d>0 )
		{
			const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
			for( int i=0 ; i<SupportSize ; i++ ) for( int j=0 ; j<SupportSize ; j++ ) for( int k=0 ; k<SupportSize ; k++ )
					{
						const TreeOctNode* n = neighbors.neighbors[i][j][k];
						if( _isValidFEMNode( n ) )
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( n , _d , fIdx );
							value +=
									Real
											(
													evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , false ) * evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , false ) * evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , false )
											) * coarseSolution[ n->nodeData.nodeIndex ];
							gradient +=
									Point3D< Real >
											(
													evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , true  ) * evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , false ) * evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , false ) ,
													evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , false ) * evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , true  ) * evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , false ) ,
													evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , false ) * evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , false ) * evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , true  )
											) * coarseSolution[ n->nodeData.nodeIndex ];
						}
					}
		}
	}
	return std::pair< Real , Point3D< Real > >( value , gradient );
}
template< class Real >
template< class V , int FEMDegree , BoundaryType BType >
V Octree< Real >::_getEdgeValue( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int edge , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const
{
	static const int SupportSize = BSplineEvaluationData< FEMDegree , BType >::SupportSize;
	static const int  LeftPointSupportRadius =  BSplineEvaluationData< FEMDegree , BType >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineEvaluationData< FEMDegree , BType >::SupportStart;
	V value(0);
	LocalDepth d ; LocalOffset cIdx;
	_localDepthAndOffset( node , d , cIdx );
	int startX = 0 , endX = SupportSize , startY = 0 , endY = SupportSize , startZ = 0 , endZ = SupportSize;
	int orientation , i1 , i2;
	Cube::FactorEdgeIndex( edge , orientation , i1 , i2 );
	switch( orientation )
	{
		case 0:
			cIdx[1] += i1 , cIdx[2] += i2;
			if( i1 ) startY++ ; else endY--;
			if( i2 ) startZ++ ; else endZ--;
			break;
		case 1:
			cIdx[0] += i1 , cIdx[2] += i2;
			if( i1 ) startX++ ; else endX--;
			if( i2 ) startZ++ ; else endZ--;
			break;
		case 2:
			cIdx[0] += i1 , cIdx[1] += i2;
			if( i1 ) startX++ ; else endX--;
			if( i2 ) startY++ ; else endY--;
			break;
	}

	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , d );
		for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[x][y][z];
					if( _isValidFEMNode( _node ) )
					{
						if( isInterior ) value += solution[ _node->nodeData.nodeIndex ] * evaluator.edgeStencil[edge]( x , y , z );
						else
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							switch( orientation )
							{
								case 0:
									value +=
											solution[ _node->nodeData.nodeIndex ] *
											Real(
													evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , false ) *
													evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , false ) *
													evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , false )
											);
									break;
								case 1:
									value +=
											solution[ _node->nodeData.nodeIndex ] *
											Real(
													evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , false ) *
													evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , false ) *
													evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , false )
											);
									break;
								case 2:
									value +=
											solution[ _node->nodeData.nodeIndex ] *
											Real(
													evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , false ) *
													evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , false ) *
													evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , false )
											);
									break;
							}
						}
					}
				}
	}
	if( d>0 )
	{
		int _corner = int( node - node->parent->children );
		int _cx , _cy , _cz;
		Cube::FactorCornerIndex( _corner , _cx , _cy , _cz );
		// If the corner/child indices don't match, then the sample position is in the interior of the
		// coarser cell and so the full support resolution should be used.
		switch( orientation )
		{
			case 0:
				if( _cy!=i1 ) startY = 0 , endY = SupportSize;
				if( _cz!=i2 ) startZ = 0 , endZ = SupportSize;
				break;
			case 1:
				if( _cx!=i1 ) startX = 0 , endX = SupportSize;
				if( _cz!=i2 ) startZ = 0 , endZ = SupportSize;
				break;
			case 2:
				if( _cx!=i1 ) startX = 0 , endX = SupportSize;
				if( _cy!=i2 ) startY = 0 , endY = SupportSize;
				break;
		}
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
		for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[x][y][z];
					if( _isValidFEMNode( _node ) )
					{
						if( isInterior ) value += coarseSolution[ _node->nodeData.nodeIndex ] * evaluator.edgeStencils[_corner][edge]( x , y , z );
						else
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							switch( orientation )
							{
								case 0:
									value +=
											coarseSolution[ _node->nodeData.nodeIndex ] *
											Real(
													evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , false ) *
													evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , false ) *
													evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , false )
											);
									break;
								case 1:
									value +=
											coarseSolution[ _node->nodeData.nodeIndex ] *
											Real(
													evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , false ) *
													evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , false ) *
													evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , false )
											);
									break;
								case 2:
									value +=
											coarseSolution[ _node->nodeData.nodeIndex ] *
											Real(
													evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , false ) *
													evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , false ) *
													evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , false )
											);
									break;
							}
						}
					}
				}
	}
	return Real( value );
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
std::pair< Real , Point3D< Real > > Octree< Real >::_getEdgeValueAndGradient( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int edge , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const
{
	static const int SupportSize = BSplineEvaluationData< FEMDegree , BType >::SupportSize;
	static const int  LeftPointSupportRadius =  BSplineEvaluationData< FEMDegree , BType >::SupportEnd;
	static const int RightPointSupportRadius = -BSplineEvaluationData< FEMDegree , BType >::SupportStart;
	double value = 0;
	Point3D< double > gradient;
	LocalDepth d ; LocalOffset cIdx;
	_localDepthAndOffset( node , d , cIdx );

	int startX = 0 , endX = SupportSize , startY = 0 , endY = SupportSize , startZ = 0 , endZ = SupportSize;
	int orientation , i1 , i2;
	Cube::FactorEdgeIndex( edge , orientation , i1 , i2 );
	switch( orientation )
	{
		case 0:
			cIdx[1] += i1 , cIdx[2] += i2;
			if( i1 ) startY++ ; else endY--;
			if( i2 ) startZ++ ; else endZ--;
			break;
		case 1:
			cIdx[0] += i1 , cIdx[2] += i2;
			if( i1 ) startX++ ; else endX--;
			if( i2 ) startZ++ ; else endZ--;
			break;
		case 2:
			cIdx[0] += i1 , cIdx[1] += i2;
			if( i1 ) startX++ ; else endX--;
			if( i2 ) startY++ ; else endY--;
			break;
	}
	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );
		for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[x][y][z];
					if( _isValidFEMNode( _node ) )
					{
						if( isInterior )
						{
							value    += evaluator. edgeStencil[edge]( x , y , z ) * solution[ _node->nodeData.nodeIndex ];
							gradient += evaluator.dEdgeStencil[edge]( x , y , z ) * solution[ _node->nodeData.nodeIndex ];
						}
						else
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );

							double vv[3] , dv[3];
							switch( orientation )
							{
								case 0:
									vv[0] = evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , false );
									vv[1] = evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , false );
									vv[2] = evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , false );
									dv[0] = evaluator.evaluator.centerValue( fIdx[0] , cIdx[0] , true  );
									dv[1] = evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , true  );
									dv[2] = evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , true  );
									break;
								case 1:
									vv[0] = evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , false );
									vv[1] = evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , false );
									vv[2] = evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , false );
									dv[0] = evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , true  );
									dv[1] = evaluator.evaluator.centerValue( fIdx[1] , cIdx[1] , true  );
									dv[2] = evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , true  );
									break;
								case 2:
									vv[0] = evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , false );
									vv[1] = evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , false );
									vv[2] = evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , false );
									dv[0] = evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , true  );
									dv[1] = evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , true  );
									dv[2] = evaluator.evaluator.centerValue( fIdx[2] , cIdx[2] , true  );
									break;
							}
							value += solution[ _node->nodeData.nodeIndex ] * vv[0] * vv[1] * vv[2];
							gradient += Point3D< double >( dv[0]*vv[1]*vv[2] , vv[0]*dv[1]*vv[2] , vv[0]*vv[1]*dv[2] ) * solution[ _node->nodeData.nodeIndex ];
						}
					}
				}
	}
	if( d>0 )
	{
		int _corner = int( node - node->parent->children );
		int _cx , _cy , _cz;
		Cube::FactorCornerIndex( _corner , _cx , _cy , _cz );
		// If the corner/child indices don't match, then the sample position is in the interior of the
		// coarser cell and so the full support resolution should be used.
		switch( orientation )
		{
			case 0:
				if( _cy!=i1 ) startY = 0 , endY = SupportSize;
				if( _cz!=i2 ) startZ = 0 , endZ = SupportSize;
				break;
			case 1:
				if( _cx!=i1 ) startX = 0 , endX = SupportSize;
				if( _cz!=i2 ) startZ = 0 , endZ = SupportSize;
				break;
			case 2:
				if( _cx!=i1 ) startX = 0 , endX = SupportSize;
				if( _cy!=i2 ) startY = 0 , endY = SupportSize;
				break;
		}
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
		for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
				{
					const TreeOctNode* _node = neighbors.neighbors[x][y][z];
					if( _isValidFEMNode( _node ) )
					{
						if( isInterior )
						{
							value    += evaluator. edgeStencils[_corner][edge]( x , y , z ) * coarseSolution[ _node->nodeData.nodeIndex ];
							gradient += evaluator.dEdgeStencils[_corner][edge]( x , y , z ) * coarseSolution[ _node->nodeData.nodeIndex ];
						}
						else
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							double vv[3] , dv[3];
							switch( orientation )
							{
								case 0:
									vv[0] = evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , false );
									vv[1] = evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , false );
									vv[2] = evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , false );
									dv[0] = evaluator.childEvaluator.centerValue( fIdx[0] , cIdx[0] , true  );
									dv[1] = evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , true  );
									dv[2] = evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , true  );
									break;
								case 1:
									vv[0] = evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , false );
									vv[1] = evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , false );
									vv[2] = evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , false );
									dv[0] = evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , true  );
									dv[1] = evaluator.childEvaluator.centerValue( fIdx[1] , cIdx[1] , true  );
									dv[2] = evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , true  );
									break;
								case 2:
									vv[0] = evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , false );
									vv[1] = evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , false );
									vv[2] = evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , false );
									dv[0] = evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , true  );
									dv[1] = evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , true  );
									dv[2] = evaluator.childEvaluator.centerValue( fIdx[2] , cIdx[2] , true  );
									break;
							}
							value += coarseSolution[ _node->nodeData.nodeIndex ] * vv[0] * vv[1] * vv[2];
							gradient += Point3D< double >( dv[0]*vv[1]*vv[2] , vv[0]*dv[1]*vv[2] , vv[0]*vv[1]*dv[2] ) * coarseSolution[ _node->nodeData.nodeIndex ];
						}
					}
				}
	}
	return std::pair< Real , Point3D< Real > >( Real( value ) , Point3D< Real >( gradient ) );
}

template< class Real >
template< class V , int FEMDegree , BoundaryType BType >
V Octree< Real >::_getCornerValue( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int corner , const DenseNodeData< V , FEMDegree >& solution , const DenseNodeData< V , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftPointSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;

	V value(0);
	LocalDepth d ; LocalOffset cIdx;
	_localDepthAndOffset( node , d , cIdx );

	int cx , cy , cz;
	int startX = 0 , endX = SupportSize , startY = 0 , endY = SupportSize , startZ = 0 , endZ = SupportSize;
	Cube::FactorCornerIndex( corner , cx , cy , cz );
	cIdx[0] += cx , cIdx[1] += cy , cIdx[2] += cz;
	{
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );
		if( cx==0 ) endX--;
		else      startX++;
		if( cy==0 ) endY--;
		else      startY++;
		if( cz==0 ) endZ--;
		else      startZ++;
		if( isInterior )
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node=neighbors.neighbors[x][y][z];
						if( IsActiveNode( _node ) ) value += solution[ _node->nodeData.nodeIndex ] * Real( evaluator.cornerStencil[corner]( x , y , z ) );
					}
		else
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node = neighbors.neighbors[x][y][z];
						if( _isValidFEMNode( _node ) )
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							value +=
									solution[ _node->nodeData.nodeIndex ] *
									Real(
											evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , false ) *
											evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , false ) *
											evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , false )
									);
						}
					}
	}
	if( d>0 )
	{
		int _corner = int( node - node->parent->children );
		int _cx , _cy , _cz;
		Cube::FactorCornerIndex( _corner , _cx , _cy , _cz );
		// If the corner/child indices don't match, then the sample position is in the interior of the
		// coarser cell and so the full support resolution should be used.
		if( cx!=_cx ) startX = 0 , endX = SupportSize;
		if( cy!=_cy ) startY = 0 , endY = SupportSize;
		if( cz!=_cz ) startZ = 0 , endZ = SupportSize;
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
		if( isInterior )
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node=neighbors.neighbors[x][y][z];
						if( IsActiveNode( _node ) ) value += coarseSolution[ _node->nodeData.nodeIndex ] * Real( evaluator.cornerStencils[_corner][corner]( x , y , z ) );
					}
		else
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node = neighbors.neighbors[x][y][z];
						if( _isValidFEMNode( _node ) )
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							value +=
									coarseSolution[ _node->nodeData.nodeIndex ] *
									Real(
											evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , false ) *
											evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , false ) *
											evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , false )
									);
						}
					}
	}
	return Real( value );
}
template< class Real >
template< int FEMDegree , BoundaryType BType >
std::pair< Real , Point3D< Real > > Octree< Real >::_getCornerValueAndGradient( const ConstPointSupportKey< FEMDegree >& neighborKey , const TreeOctNode* node , int corner , const DenseNodeData< Real , FEMDegree >& solution , const DenseNodeData< Real , FEMDegree >& coarseSolution , const _Evaluator< FEMDegree , BType >& evaluator , bool isInterior ) const
{
	static const int SupportSize = BSplineSupportSizes< FEMDegree >::SupportSize;
	static const int  LeftPointSupportRadius =   BSplineSupportSizes< FEMDegree >::SupportEnd;
	static const int RightPointSupportRadius = - BSplineSupportSizes< FEMDegree >::SupportStart;

	double value = 0;
	Point3D< double > gradient;
	LocalDepth d ; LocalOffset cIdx;
	_localDepthAndOffset( node , d , cIdx );

	int cx , cy , cz;
	int startX = 0 , endX = SupportSize , startY = 0 , endY = SupportSize , startZ = 0 , endZ = SupportSize;
	Cube::FactorCornerIndex( corner , cx , cy , cz );
	cIdx[0] += cx , cIdx[1] += cy , cIdx[2] += cz;
	{
		if( cx==0 ) endX--;
		else      startX++;
		if( cy==0 ) endY--;
		else      startY++;
		if( cz==0 ) endZ--;
		else      startZ++;
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node );
		if( isInterior )
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node=neighbors.neighbors[x][y][z];
						if( IsActiveNode( _node ) ) value += solution[ _node->nodeData.nodeIndex ] * evaluator.cornerStencil[corner]( x , y , z ) , gradient += evaluator.dCornerStencil[corner]( x , y , z ) * solution[ _node->nodeData.nodeIndex ];
					}
		else
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node = neighbors.neighbors[x][y][z];
						if( _isValidFEMNode( _node ) )
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							double v [] = { evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , false ) , evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , false ) , evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , false ) };
							double dv[] = { evaluator.evaluator.cornerValue( fIdx[0] , cIdx[0] , true  ) , evaluator.evaluator.cornerValue( fIdx[1] , cIdx[1] , true  ) , evaluator.evaluator.cornerValue( fIdx[2] , cIdx[2] , true  ) };
							value += solution[ _node->nodeData.nodeIndex ] * v[0] * v[1] * v[2];
							gradient += Point3D< double >( dv[0]*v[1]*v[2] , v[0]*dv[1]*v[2] , v[0]*v[1]*dv[2] ) * solution[ _node->nodeData.nodeIndex ];
						}
					}
	}
	if( d>0 )
	{
		int _corner = int( node - node->parent->children );
		int _cx , _cy , _cz;
		Cube::FactorCornerIndex( _corner , _cx , _cy , _cz );
		if( cx!=_cx ) startX = 0 , endX = SupportSize;
		if( cy!=_cy ) startY = 0 , endY = SupportSize;
		if( cz!=_cz ) startZ = 0 , endZ = SupportSize;
		const typename TreeOctNode::ConstNeighbors< SupportSize >& neighbors = _neighbors< LeftPointSupportRadius , RightPointSupportRadius >( neighborKey , node->parent );
		if( isInterior )
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node=neighbors.neighbors[x][y][z];
						if( IsActiveNode( _node ) ) value += coarseSolution[ _node->nodeData.nodeIndex ] * evaluator.cornerStencils[_corner][corner]( x , y , z ) , gradient += evaluator.dCornerStencils[_corner][corner]( x , y , z ) * coarseSolution[ _node->nodeData.nodeIndex ];
					}
		else
			for( int x=startX ; x<endX ; x++ ) for( int y=startY ; y<endY ; y++ ) for( int z=startZ ; z<endZ ; z++ )
					{
						const TreeOctNode* _node = neighbors.neighbors[x][y][z];
						if( _isValidFEMNode( _node ) )
						{
							LocalDepth _d ; LocalOffset fIdx;
							_localDepthAndOffset( _node , _d , fIdx );
							double v [] = { evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , false ) , evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , false ) , evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , false ) };
							double dv[] = { evaluator.childEvaluator.cornerValue( fIdx[0] , cIdx[0] , true  ) , evaluator.childEvaluator.cornerValue( fIdx[1] , cIdx[1] , true  ) , evaluator.childEvaluator.cornerValue( fIdx[2] , cIdx[2] , true  ) };
							value += coarseSolution[ _node->nodeData.nodeIndex ] * v[0] * v[1] * v[2];
							gradient += Point3D< double >( dv[0]*v[1]*v[2] , v[0]*dv[1]*v[2] , v[0]*v[1]*dv[2] ) * coarseSolution[ _node->nodeData.nodeIndex ];
						}
					}
	}
	return std::pair< Real , Point3D< Real > >( Real( value ) , Point3D< Real >( gradient ) );
}
template< class Real >
template< int Degree , BoundaryType BType >
Octree< Real >::MultiThreadedEvaluator< Degree , BType >::MultiThreadedEvaluator( const Octree< Real >* tree , const DenseNodeData< Real , Degree >& coefficients , int threads ) : _coefficients( coefficients ) , _tree( tree )
{
	_threads = std::max< int >( 1 , threads );
	_neighborKeys.resize( _threads );
	_coarseCoefficients = _tree->template coarseCoefficients< Real , Degree , BType >( _coefficients );
	_evaluator.set( _tree->_maxDepth );
	for( int t=0 ; t<_threads ; t++ ) _neighborKeys[t].set( tree->_localToGlobal( _tree->_maxDepth ) );
}
template< class Real >
template< int Degree , BoundaryType BType >
Real Octree< Real >::MultiThreadedEvaluator< Degree , BType >::value( Point3D< Real > p , int thread , const TreeOctNode* node )
{
	if( !node ) node = _tree->leaf( p );
	ConstPointSupportKey< Degree >& nKey = _neighborKeys[thread];
	nKey.getNeighbors( node );
	return _tree->template _getValue< Real , Degree >( nKey , node , p , _coefficients , _coarseCoefficients , _evaluator );
}
template< class Real >
template< int Degree , BoundaryType BType >
std::pair< Real , Point3D< Real > > Octree< Real >::MultiThreadedEvaluator< Degree , BType >::valueAndGradient( Point3D< Real > p , int thread , const TreeOctNode* node )
{
	if( !node ) node = _tree->leaf( p );
	ConstPointSupportKey< Degree >& nKey = _neighborKeys[thread];
	nKey.getNeighbors( node );
	return _tree->template _getValueAndGradient< Degree >( nKey , node , p , _coefficients , _coarseCoefficients , _evaluator );
}

#endif // MULTI_GRID_OCTREE_DATA_INCLUDED
