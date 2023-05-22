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

#ifndef FUNCTION_DATA_INCLUDED
#define FUNCTION_DATA_INCLUDED

#define BOUNDARY_CONDITIONS 1


#include "PPolynomial.h"

template<int Degree,class Real>
class FunctionData{
	bool useDotRatios;
	int normalize;
#if BOUNDARY_CONDITIONS
	bool reflectBoundary;
#endif // BOUNDARY_CONDITIONS
public:
	const static int     DOT_FLAG = 1;
	const static int   D_DOT_FLAG = 2;
	const static int  D2_DOT_FLAG = 4;
	const static int   VALUE_FLAG = 1;
	const static int D_VALUE_FLAG = 2;

	int depth , res , res2;
	Real *dotTable , *dDotTable , *d2DotTable;
	Real *valueTables , *dValueTables;
#if BOUNDARY_CONDITIONS
	PPolynomial<Degree> baseFunction , leftBaseFunction , rightBaseFunction;
	PPolynomial<Degree-1> dBaseFunction , dLeftBaseFunction , dRightBaseFunction;
#else // !BOUNDARY_CONDITIONS
	PPolynomial<Degree> baseFunction;
	PPolynomial<Degree-1> dBaseFunction;
#endif // BOUNDARY_CONDITIONS
	PPolynomial<Degree+1>* baseFunctions;

	FunctionData(void);
	~FunctionData(void);

	/********************************************************
	 * Sets the translates and scales of the basis function
	 * up to the prescribed depth
	 * <maxDepth> the maximum depth
	 * <F> the basis function
	 * <normalize> how the functions should be scaled
	 *      0] Value at zero equals 1
	 *      1] Integral equals 1
	 *		2] L2-norm equals 1
	 * <useDotRatios> specifies if dot-products of derivatives
	 * should be pre-divided by function integrals
	 * <reflectBoundary> spcifies if function space should be
	 * forced to be reflectively symmetric across the boundary
	 ********************************************************/
#if BOUNDARY_CONDITIONS
	void set( const int& maxDepth , const PPolynomial<Degree>& F , const int& normalize , bool useDotRatios=true , bool reflectBoundary=false );
#else // !BOUNDARY_CONDITIONS
	void set(const int& maxDepth,const PPolynomial<Degree>& F,const int& normalize , bool useDotRatios=true );
#endif // BOUNDARY_CONDITIONS

#if BOUNDARY_CONDITIONS
	Real   dotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 , int boundary1 , int boundary2 ) const;
	Real  dDotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 , int boundary1 , int boundary2 ) const;
	Real d2DotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 , int boundary1 , int boundary2 ) const;
#else // !BOUNDARY_CONDITIONS
	Real   dotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 ) const;
	Real  dDotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 ) const;
	Real d2DotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 ) const;
#endif // BOUNDARY_CONDITIONS
};



//////////////////
// FunctionData //
//////////////////
template<int Degree,class Real>
FunctionData<Degree,Real>::FunctionData(void)
{
	dotTable=dDotTable=d2DotTable=NULL;
	valueTables=dValueTables=NULL;
	res=0;
}

template<int Degree,class Real>
FunctionData<Degree,Real>::~FunctionData(void)
{
	if(res)
	{
		if(  dotTable) delete[]   dotTable;
		if( dDotTable) delete[]  dDotTable;
		if(d2DotTable) delete[] d2DotTable;
		if( valueTables) delete[]  valueTables;
		if(dValueTables) delete[] dValueTables;
	}
	dotTable=dDotTable=d2DotTable=NULL;
	valueTables=dValueTables=NULL;
	res=0;
}

template<int Degree,class Real>
#if BOUNDARY_CONDITIONS
void FunctionData<Degree,Real>::set( const int& maxDepth , const PPolynomial<Degree>& F , const int& normalize , bool useDotRatios , bool reflectBoundary )
#else // !BOUNDARY_CONDITIONS
void FunctionData<Degree,Real>::set(const int& maxDepth,const PPolynomial<Degree>& F,const int& normalize , bool useDotRatios )
#endif // BOUNDARY_CONDITIONS
{
	this->normalize       = normalize;
	this->useDotRatios    = useDotRatios;
#if BOUNDARY_CONDITIONS
	this->reflectBoundary = reflectBoundary;
#endif // BOUNDARY_CONDITIONS

	depth = maxDepth;
	res = BinaryNode<double>::CumulativeCenterCount( depth );
	res2 = (1<<(depth+1))+1;
	baseFunctions = new PPolynomial<Degree+1>[res];
	// Scale the function so that it has:
	// 0] Value 1 at 0
	// 1] Integral equal to 1
	// 2] Square integral equal to 1
	switch( normalize )
	{
		case 2:
			baseFunction=F/sqrt((F*F).integral(F.polys[0].start,F.polys[F.polyCount-1].start));
			break;
		case 1:
			baseFunction=F/F.integral(F.polys[0].start,F.polys[F.polyCount-1].start);
			break;
		default:
			baseFunction=F/F(0);
	}
	dBaseFunction = baseFunction.derivative();
#if BOUNDARY_CONDITIONS
	leftBaseFunction   = baseFunction + baseFunction.shift( -1 );
	rightBaseFunction  = baseFunction + baseFunction.shift(  1 );
	dLeftBaseFunction  =  leftBaseFunction.derivative();
	dRightBaseFunction = rightBaseFunction.derivative();
#endif // BOUNDARY_CONDITIONS
	double c1,w1;
	for( int i=0 ; i<res ; i++ )
	{
		BinaryNode< double >::CenterAndWidth( i , c1 , w1 );
#if BOUNDARY_CONDITIONS
		if( reflectBoundary )
		{
			int d , off;
			BinaryNode< double >::DepthAndOffset( i , d , off );
			if     ( off==0          ) baseFunctions[i] =  leftBaseFunction.scale( w1 ).shift( c1 );
			else if( off==((1<<d)-1) ) baseFunctions[i] = rightBaseFunction.scale( w1 ).shift( c1 );
			else                       baseFunctions[i] =      baseFunction.scale( w1 ).shift( c1 );
		}
		else baseFunctions[i] = baseFunction.scale(w1).shift(c1);
#else // !BOUNDARY_CONDITIONS
		baseFunctions[i] = baseFunction.scale(w1).shift(c1);
#endif // BOUNDARY_CONDITIONS
		// Scale the function so that it has L2-norm equal to one
		switch( normalize )
		{
			case 2:
				baseFunctions[i]/=sqrt(w1);
				break;
			case 1:
				baseFunctions[i]/=w1;
				break;
		}
	}
}


#if BOUNDARY_CONDITIONS
template<int Degree,class Real>
Real FunctionData<Degree,Real>::dotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 , int boundary1 , int boundary2 ) const
{
	const PPolynomial< Degree > *b1 , *b2;
	if     ( boundary1==-1 ) b1 = & leftBaseFunction;
	else if( boundary1== 0 ) b1 = &     baseFunction;
	else if( boundary1== 1 ) b1 = &rightBaseFunction;
	if     ( boundary2==-1 ) b2 = & leftBaseFunction;
	else if( boundary2== 0 ) b2 = &     baseFunction;
	else if( boundary2== 1 ) b2 = &rightBaseFunction;
	double r=fabs( baseFunction.polys[0].start );
	switch( normalize )
	{
		case 2:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)*width1/sqrt(width1*width2));
		case 1:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)*width1/(width1*width2));
		default:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)*width1);
	}
}
template<int Degree,class Real>
Real FunctionData<Degree,Real>::dDotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 , int boundary1 , int boundary2 ) const
{
	const PPolynomial< Degree-1 > *b1;
	const PPolynomial< Degree   > *b2;
	if     ( boundary1==-1 ) b1 = & dLeftBaseFunction;
	else if( boundary1== 0 ) b1 = &     dBaseFunction;
	else if( boundary1== 1 ) b1 = &dRightBaseFunction;
	if     ( boundary2==-1 ) b2 = &  leftBaseFunction;
	else if( boundary2== 0 ) b2 = &      baseFunction;
	else if( boundary2== 1 ) b2 = & rightBaseFunction;
	double r=fabs(baseFunction.polys[0].start);
	switch(normalize){
		case 2:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/sqrt(width1*width2));
		case 1:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/(width1*width2));
		default:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r));
	}
}
template<int Degree,class Real>
Real FunctionData<Degree,Real>::d2DotProduct( const double& center1 , const double& width1 , const double& center2 , const double& width2 , int boundary1 , int boundary2 ) const
{
	const PPolynomial< Degree-1 > *b1 , *b2;
	if     ( boundary1==-1 ) b1 = & dLeftBaseFunction;
	else if( boundary1== 0 ) b1 = &     dBaseFunction;
	else if( boundary1== 1 ) b1 = &dRightBaseFunction;
	if     ( boundary2==-1 ) b2 = & dLeftBaseFunction;
	else if( boundary2== 0 ) b2 = &     dBaseFunction;
	else if( boundary2== 1 ) b2 = &dRightBaseFunction;
	double r=fabs(baseFunction.polys[0].start);
	switch( normalize )
	{
		case 2:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/width2/sqrt(width1*width2));
		case 1:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/width2/(width1*width2));
		default:
			return Real(((*b1)*b2->scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/width2);
	}
}
#else // !BOUNDARY_CONDITIONS
template<int Degree,class Real>
Real FunctionData<Degree,Real>::dotProduct(const double& center1,const double& width1,const double& center2,const double& width2) const{
	double r=fabs(baseFunction.polys[0].start);
	switch( normalize )
	{
		case 2:
			return Real((baseFunction*baseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)*width1/sqrt(width1*width2));
		case 1:
			return Real((baseFunction*baseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)*width1/(width1*width2));
		default:
			return Real((baseFunction*baseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)*width1);
	}
}
template<int Degree,class Real>
Real FunctionData<Degree,Real>::dDotProduct(const double& center1,const double& width1,const double& center2,const double& width2) const{
	double r=fabs(baseFunction.polys[0].start);
	switch(normalize){
		case 2:
			return Real((dBaseFunction*baseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/sqrt(width1*width2));
		case 1:
			return Real((dBaseFunction*baseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/(width1*width2));
		default:
			return Real((dBaseFunction*baseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r));
	}
}
template<int Degree,class Real>
Real FunctionData<Degree,Real>::d2DotProduct(const double& center1,const double& width1,const double& center2,const double& width2) const{
	double r=fabs(baseFunction.polys[0].start);
	switch(normalize){
		case 2:
			return Real((dBaseFunction*dBaseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/width2/sqrt(width1*width2));
		case 1:
			return Real((dBaseFunction*dBaseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/width2/(width1*width2));
		default:
			return Real((dBaseFunction*dBaseFunction.scale(width2/width1).shift((center2-center1)/width1)).integral(-2*r,2*r)/width2);
	}
}
#endif // BOUNDARY_CONDITIONS

#endif // FUNCTION_DATA_INCLUDED