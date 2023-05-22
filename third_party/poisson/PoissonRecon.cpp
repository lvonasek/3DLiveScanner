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

#undef FAST_COMPILE
#undef ARRAY_DEBUG

#include "CmdLineParser.h"
#include "MultiGridOctreeData.h"
#include "Ply.h"

cmdLineString
	In( "in" ) ,
	Out( "out" ) ;

cmdLineReadable
	PolygonMesh( "polygonMesh" ) ,
	Confidence( "confidence" ) ,
	NormalWeights( "nWeights" ) ,
	NonManifold( "nonManifold" ) ,
	LinearFit( "linearFit" ) ,
	PrimalVoxel( "primalVoxel" ) ;

cmdLineInt
	Depth( "depth" , 8 ) ,
	CGDepth( "cgDepth" , 0 ) ,
	KernelDepth( "kernelDepth" ) ,
	AdaptiveExponent( "adaptiveExp" , 1 ) ,
	Iters( "iters" , 8 ) ,
	VoxelDepth( "voxelDepth" , -1 ) ,
	FullDepth( "fullDepth" , 5 ) ,
	MaxSolveDepth( "maxSolveDepth" ) ,
	Threads( "threads" , 4 );

cmdLineFloat
	SamplesPerNode( "samplesPerNode" , 1.5f ) ,
	Scale( "scale" , 1.1f ) ,
	CGSolverAccuracy( "cgAccuracy" , float(1e-3) ) ,
	LowResIterMultiplier( "iterMultiplier" , 1.f ) , 
	PointWeight( "pointWeight" , 4.f );


cmdLineReadable* params[] =
{
	&In , &Depth , &Out ,
	&Scale , &CGSolverAccuracy , &LowResIterMultiplier ,
	&KernelDepth , &SamplesPerNode , &Confidence , &NormalWeights , &NonManifold , &PolygonMesh , &VoxelDepth ,
	&PointWeight , &Threads , &MaxSolveDepth ,
	&AdaptiveExponent ,
	&FullDepth ,
	&CGDepth , &Iters ,
	&LinearFit ,
	&PrimalVoxel
};

template< class Real >
XForm4x4< Real > GetPointXForm( OrientedPointStream< Real >& stream , Real scaleFactor )
{
	Point3D< Real > min , max;
	stream.boundingBox( min , max );
	Point3D< Real > center = ( max + min ) / 2;
	Real scale = std::max< Real >( max[0]-min[0] , std::max< Real >( max[1]-min[1] , max[2]-min[2] ) );
	scale *= scaleFactor;
	for( int i=0 ; i<3 ; i++ ) center[i] -= scale/2;
	XForm4x4< Real > tXForm = XForm4x4< Real >::Identity() , sXForm = XForm4x4< Real >::Identity();
	for( int i=0 ; i<3 ; i++ ) sXForm(i,i) = (Real)(1./scale ) , tXForm(3,i) = -center[i];
	return sXForm * tXForm;
}

template< class Real , int Degree , BoundaryType BType , class Vertex >
int _Execute()
{
	typedef typename Octree< Real >::template DensityEstimator< WEIGHT_DEGREE > DensityEstimator;
	typedef typename Octree< Real >::template InterpolationInfo< false > InterpolationInfo;
	typedef OrientedPointStream< Real > PointStream;
	typedef OrientedPointStreamWithData< Real , Point3D< Real > > PointStreamWithData;
	typedef TransformedOrientedPointStream< Real > XPointStream;
	typedef TransformedOrientedPointStreamWithData< Real , Point3D< Real > > XPointStreamWithData;
	Reset< Real >();

	XForm4x4< Real > xForm , iXForm;
	xForm = XForm4x4< Real >::Identity();

	Real isoValue = 0;

	Octree< Real > tree;
	tree.threads = Threads.value;
	if( !In.set )
	{
		return 0;
	}
	if( !MaxSolveDepth.set ) MaxSolveDepth.value = Depth.value;

	OctNode< TreeNodeData >::SetAllocator( MEMORY_ALLOCATOR_BLOCK_SIZE );

	int kernelDepth = KernelDepth.set ? KernelDepth.value : Depth.value-2;
	if( kernelDepth>Depth.value )
	{
		kernelDepth = Depth.value;
	}

	Real pointWeightSum;
	std::vector< typename Octree< Real >::PointSample >* samples = new std::vector< typename Octree< Real >::PointSample >();
	Real targetValue = (Real)0.5;
	// Read in the samples (and color data)
	{
		PointStream* pointStream = new    PLYOrientedPointStream< Real >( In.value );
		XPointStream _pointStream( xForm , *pointStream );
		xForm = GetPointXForm( _pointStream , (Real)Scale.value ) * xForm;

		XPointStream ps( xForm , *pointStream );
		tree.template init< Point3D< Real > >( ps , Depth.value , Confidence.set , *samples , NULL );

		iXForm = xForm.inverse();
		delete pointStream;
#pragma omp parallel for num_threads( Threads.value )
		for( int i=0 ; i<(int)samples->size() ; i++ ) (*samples)[i].sample.data.n *= (Real)-1;
	}
	DenseNodeData< Real , Degree > solution;

	{
		int solveDepth = MaxSolveDepth.value;
		tree.resetNodeIndices();

		// Get the kernel density estimator [If discarding, compute anew. Otherwise, compute once.]
		// Transform the Hermite samples into a vector field [If discarding, compute anew. Otherwise, compute once.]
		DensityEstimator* density = tree.template setDensityEstimator< WEIGHT_DEGREE >( *samples , kernelDepth , SamplesPerNode.value );
		SparseNodeData< Point3D< Real > , NORMAL_DEGREE >* normalInfo = new SparseNodeData< Point3D< Real > , NORMAL_DEGREE >();
     	*normalInfo = tree.template setNormalField< NORMAL_DEGREE >( *samples , *density , pointWeightSum , BType==BOUNDARY_NEUMANN );
		delete density;

		// Trim the tree and prepare for multigrid
		{
			std::vector< int > indexMap;
			constexpr int MAX_DEGREE = NORMAL_DEGREE > Degree ? NORMAL_DEGREE : Degree;
			tree.template inalizeForBroodedMultigrid< MAX_DEGREE , Degree , BType >( FullDepth.value , typename Octree< Real >::template HasNormalDataFunctor< NORMAL_DEGREE >( *normalInfo ) , &indexMap );
			if( normalInfo ) normalInfo->remapIndices( indexMap );
		}

		// Add the FEM constraints
		DenseNodeData< Real , Degree > constraints = tree.template initDenseNodeData< Degree >( );
		tree.template addFEMConstraints< Degree , BType , NORMAL_DEGREE , BType >( FEMVFConstraintFunctor< NORMAL_DEGREE , BType , Degree , BType >( 1. , 0. ) , *normalInfo , constraints , solveDepth );
		delete normalInfo;

		// Add the interpolation constraints
		InterpolationInfo* iInfo = NULL;
		if( PointWeight.value>0 )
		{
			iInfo = new InterpolationInfo( tree , *samples , targetValue , AdaptiveExponent.value , (Real)PointWeight.value * pointWeightSum , (Real)0 );
			tree.template addInterpolationConstraints< Degree , BType >( *iInfo , constraints , solveDepth );
		}

		// Solve the linear system
		typename Octree< Real >::SolverInfo solverInfo;
		solverInfo.cgDepth = CGDepth.value , solverInfo.iters = Iters.value , solverInfo.cgAccuracy = CGSolverAccuracy.value , solverInfo.lowResIterMultiplier = std::max< double >( 1. , LowResIterMultiplier.value );
		solution = tree.template solveSystem< Degree , BType >( FEMSystemFunctor< Degree , BType >( 0 , 1. , 0 ) , iInfo , constraints , solveDepth , solverInfo );
		if( iInfo ) delete iInfo;
	}

	CoredFileMeshData< Vertex > mesh;
	{
		double valueSum = 0 , weightSum = 0;
		typename Octree< Real >::template MultiThreadedEvaluator< Degree , BType > evaluator( &tree , solution , Threads.value );
#pragma omp parallel for num_threads( Threads.value ) reduction( + : valueSum , weightSum )
		for( int j=0 ; j<samples->size() ; j++ )
		{
			ProjectiveData< OrientedPoint3D< Real > , Real >& sample = (*samples)[j].sample;
			Real w = sample.weight;
			if( w>0 ) weightSum += w , valueSum += evaluator.value( sample.data.p / sample.weight , get_thread_num() , (*samples)[j].node ) * w;
		}
		isoValue = (Real)( valueSum / weightSum );
		if( samples ) delete samples , samples = NULL;
	}

	if( Out.set )
	{
		std::vector< char* > comments;
		tree.template getMCIsoSurface< Degree , BType , WEIGHT_DEGREE , DATA_DEGREE >( solution , isoValue , mesh , !LinearFit.set , !NonManifold.set , PolygonMesh.set );
		PlyWritePolygons( Out.value , &mesh , PLY_ASCII         , &comments[0] , (int)comments.size() , iXForm );
	}

	return 1;
}

int poisson_main( int argc , char* argv[] )
{
	cmdLineParse( argc-1 , &argv[1] , sizeof(params)/sizeof(cmdLineReadable*) , params , 1 );
	if( Out.set )
	{
		BufferedReadWriteFile::setTempPrefix( Out.value );
	}
	return _Execute< float , 2 , BOUNDARY_FREE , PlyVertex< float > >();
}
