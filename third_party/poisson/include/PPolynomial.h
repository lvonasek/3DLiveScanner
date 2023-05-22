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

#ifndef P_POLYNOMIAL_INCLUDED
#define P_POLYNOMIAL_INCLUDED
#include <vector>
#include "Polynomial.h"
#include "Array.h"
#include "Factor.h"

template< int Degree >
class StartingPolynomial
{
public:
	Polynomial< Degree > p;
	double start;

	template< int Degree2 >
	StartingPolynomial< Degree+Degree2 >  operator * ( const StartingPolynomial< Degree2 >& p ) const;
	StartingPolynomial scale( double s ) const;
	StartingPolynomial shift( double t ) const;
	int operator < ( const StartingPolynomial& sp ) const;
	static int Compare( const void* v1 , const void* v2 );
};

template< int Degree >
class PPolynomial
{
public:
	size_t polyCount;
	Pointer( StartingPolynomial< Degree > ) polys;

	PPolynomial( void );
	PPolynomial( const PPolynomial<Degree>& p );
	~PPolynomial( void );

	PPolynomial& operator = ( const PPolynomial& p );

	int size( void ) const;

	void set( size_t size );
	// Note: this method will sort the elements in sps
	void set( Pointer( StartingPolynomial<Degree> ) sps , int count );
	void reset( size_t newSize );
	PPolynomial& compress( double delta=0. );


	double operator()( double t ) const;
	double integral( double tMin , double tMax ) const;

	template< int Degree2 > PPolynomial< Degree >& operator = ( const PPolynomial< Degree2 >& p );

	PPolynomial  operator + ( const PPolynomial& p ) const;
	PPolynomial  operator - ( const PPolynomial& p ) const;

	template< int Degree2 > PPolynomial< Degree+Degree2 > operator * ( const  Polynomial< Degree2 >& p ) const;
	template< int Degree2 >	PPolynomial< Degree+Degree2 > operator * ( const PPolynomial< Degree2 >& p) const;


	PPolynomial& operator += ( double s );
	PPolynomial& operator -= ( double s );
	PPolynomial& operator *= ( double s );
	PPolynomial& operator /= ( double s );
	PPolynomial  operator +  ( double s ) const;
	PPolynomial  operator -  ( double s ) const;
	PPolynomial  operator *  ( double s ) const;
	PPolynomial  operator /  ( double s ) const;

	PPolynomial scale( double s ) const;
	PPolynomial shift( double t ) const;
	PPolynomial reflect( double r=0. ) const;

	PPolynomial< Degree-1 > derivative(void) const;
	PPolynomial< Degree+1 > integral(void) const;

	PPolynomial< Degree+1 > MovingAverage( double radius ) const;
	static PPolynomial BSpline( double radius=0.5 );

	void write( FILE* fp , int samples , double min , double max ) const;
};


////////////////////////
// StartingPolynomial //
////////////////////////
template<int Degree>
template<int Degree2>
StartingPolynomial<Degree+Degree2> StartingPolynomial<Degree>::operator * (const StartingPolynomial<Degree2>& p) const{
	StartingPolynomial<Degree+Degree2> sp;
	if(start>p.start){sp.start=start;}
	else{sp.start=p.start;}
	sp.p=this->p*p.p;
	return sp;
}
template<int Degree>
StartingPolynomial<Degree> StartingPolynomial<Degree>::scale( double s ) const
{
	StartingPolynomial q;
	q.start = start*s;
	q.p = p.scale(s);
	return q;
}
template<int Degree>
StartingPolynomial<Degree> StartingPolynomial<Degree>::shift(double s) const{
	StartingPolynomial q;
	q.start=start+s;
	q.p=p.shift(s);
	return q;
}


template<int Degree>
int StartingPolynomial<Degree>::operator < (const StartingPolynomial<Degree>& sp) const{
	if(start<sp.start){return 1;}
	else{return 0;}
}
template<int Degree>
int StartingPolynomial<Degree>::Compare(const void* v1,const void* v2){
	double d=((StartingPolynomial*)(v1))->start-((StartingPolynomial*)(v2))->start;
	if     ( d<0 ) return -1;
	else if( d>0 ) return  1;
	else           return  0;
}

/////////////////
// PPolynomial //
/////////////////
template< int Degree >
PPolynomial< Degree >::PPolynomial( void )
{
	polyCount = 0;
	polys = NullPointer( StartingPolynomial< Degree > );
}
template< int Degree >
PPolynomial<Degree>::PPolynomial( const PPolynomial<Degree>& p )
{
	polyCount = 0;
	polys = NullPointer( StartingPolynomial< Degree > );
	set( p.polyCount );
	memcpy( polys , p.polys , sizeof( StartingPolynomial<Degree> ) * p.polyCount );
}

template< int Degree >
PPolynomial< Degree >::~PPolynomial( void )
{
	FreePointer( polys );
	polyCount = 0;
}
template< int Degree >
void PPolynomial< Degree >::set( Pointer( StartingPolynomial< Degree > ) sps , int count )
{
	int c=0;
	set( count );
	qsort( sps , count , sizeof( StartingPolynomial< Degree > ) , StartingPolynomial< Degree >::Compare );
	for( int i=0 ; i<count ; i++ )
	{
		if( !c || sps[i].start!=polys[c-1].start ) polys[c++] = sps[i];
		else{polys[c-1].p+=sps[i].p;}
	}
	reset( c );
}
template< int Degree > int PPolynomial< Degree >::size( void ) const{ return int(sizeof(StartingPolynomial<Degree>)*polyCount); }

template< int Degree >
void PPolynomial<Degree>::set( size_t size )
{
	FreePointer( polys );
	polyCount = size;
	if( size )
	{
		polys = AllocPointer< StartingPolynomial< Degree > >( size );
		memset( polys , 0 , sizeof( StartingPolynomial< Degree > )*size );
	}
}
template< int Degree >
void PPolynomial<Degree>::reset( size_t newSize )
{
	polyCount = newSize;
	polys = ReAllocPointer< StartingPolynomial< Degree > >( polys , newSize );
}
template< int Degree >
PPolynomial< Degree >& PPolynomial< Degree >::compress( double delta )
{
	int _polyCount = (int)polyCount;
	Pointer( StartingPolynomial< Degree > ) _polys = polys;

	polyCount = 1 , polys = NullPointer( StartingPolynomial< Degree > );
	for( int i=1 ; i<_polyCount ; i++ ) if( _polys[i].start-_polys[i-1].start>delta ) polyCount++;
	if( polyCount==_polyCount ) polys = _polys;
	else
	{
		polys = AllocPointer< StartingPolynomial< Degree > >( polyCount );
		polys[0] = _polys[0] , polyCount = 0;
		for( int i=1 ; i<_polyCount ; i++ )
		{
			if( _polys[i].start-_polys[i-1].start>delta ) polys[ ++polyCount ] = _polys[i];
			else polys[ polyCount ].p += _polys[i].p;
		}
		polyCount++;
		FreePointer( _polys );
	}
	return *this;
}

template< int Degree >
PPolynomial<Degree>& PPolynomial<Degree>::operator = (const PPolynomial<Degree>& p){
	set(p.polyCount);
	memcpy(polys,p.polys,sizeof(StartingPolynomial<Degree>)*p.polyCount);
	return *this;
}

template<int Degree>
template<int Degree2>
PPolynomial<Degree>& PPolynomial<Degree>::operator  = (const PPolynomial<Degree2>& p){
	set(p.polyCount);
	for(int i=0;i<int(polyCount);i++){
		polys[i].start=p.polys[i].start;
		polys[i].p=p.polys[i].p;
	}
	return *this;
}

template<int Degree>
double PPolynomial<Degree>::operator ()( double t ) const
{
	double v=0;
	for( int i=0 ; i<int(polyCount) && t>polys[i].start ; i++ ) v += polys[i].p(t);
	return v;
}

template<int Degree>
double PPolynomial<Degree>::integral( double tMin , double tMax ) const
{
	int m=1;
	double start,end,s,v=0;
	start=tMin;
	end=tMax;
	if(tMin>tMax){
		m=-1;
		start=tMax;
		end=tMin;
	}
	for(int i=0;i<int(polyCount) && polys[i].start<end;i++){
		if(start<polys[i].start){s=polys[i].start;}
		else{s=start;}
		v+=polys[i].p.integral(s,end);
	}
	return v*m;
}

template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::operator + (const PPolynomial<Degree>& p) const{
	PPolynomial q;
	int i,j;
	size_t idx=0;
	q.set(polyCount+p.polyCount);
	i=j=-1;

	while(idx<q.polyCount){
		if		(j>=int(p.polyCount)-1)				{q.polys[idx]=  polys[++i];}
		else if	(i>=int(  polyCount)-1)				{q.polys[idx]=p.polys[++j];}
		else if(polys[i+1].start<p.polys[j+1].start){q.polys[idx]=  polys[++i];}
		else										{q.polys[idx]=p.polys[++j];}
		idx++;
	}
	return q;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::operator - (const PPolynomial<Degree>& p) const{
	PPolynomial q;
	int i,j;
	size_t idx=0;
	q.set(polyCount+p.polyCount);
	i=j=-1;

	while(idx<q.polyCount){
		if		(j>=int(p.polyCount)-1)				{q.polys[idx]=  polys[++i];}
		else if	(i>=int(  polyCount)-1)				{q.polys[idx].start=p.polys[++j].start;q.polys[idx].p=p.polys[j].p*(-1.0);}
		else if(polys[i+1].start<p.polys[j+1].start){q.polys[idx]=  polys[++i];}
		else										{q.polys[idx].start=p.polys[++j].start;q.polys[idx].p=p.polys[j].p*(-1.0);}
		idx++;
	}
	return q;
}

template<int Degree>
template<int Degree2>
PPolynomial<Degree+Degree2> PPolynomial<Degree>::operator * (const PPolynomial<Degree2>& p) const{
	PPolynomial<Degree+Degree2> q;
	StartingPolynomial<Degree+Degree2> *sp;
	int i,j,spCount=int(polyCount*p.polyCount);

	sp=(StartingPolynomial<Degree+Degree2>*)malloc(sizeof(StartingPolynomial<Degree+Degree2>)*spCount);
	for(i=0;i<int(polyCount);i++){
		for(j=0;j<int(p.polyCount);j++){
			sp[i*p.polyCount+j]=polys[i]*p.polys[j];
		}
	}
	q.set(sp,spCount);
	free(sp);
	return q;
}
template<int Degree>
template<int Degree2>
PPolynomial<Degree+Degree2> PPolynomial<Degree>::operator * (const Polynomial<Degree2>& p) const{
	PPolynomial<Degree+Degree2> q;
	q.set(polyCount);
	for(int i=0;i<int(polyCount);i++){
		q.polys[i].start=polys[i].start;
		q.polys[i].p=polys[i].p*p;
	}
	return q;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::scale( double s ) const
{
	PPolynomial q;
	q.set( polyCount );
	for( size_t i=0 ; i<polyCount ; i++ ) q.polys[i] = polys[i].scale(s);
	if( s<0 ) qsort( q.polys , polyCount , sizeof( StartingPolynomial< Degree > ) , StartingPolynomial< Degree >::Compare );
	return q;
}
template< int Degree >
PPolynomial< Degree > PPolynomial< Degree >::reflect( double r ) const
{
	PPolynomial q;
	q.set( polyCount );
	for( size_t i=0 ; i<polyCount ; i++ )
	{
		q.polys[i].scale(-1.);
		if( r ) q.polys[i].shift( 2.*r );
	}
	qsort( q.polys , polyCount , sizeof( StartingPolynomial< Degree > ) , StartingPolynomial< Degree >::Compare );
	return q;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::shift( double s ) const
{
	PPolynomial q;
	q.set(polyCount);
	for(size_t i=0;i<polyCount;i++){q.polys[i]=polys[i].shift(s);}
	return q;
}
template<int Degree>
PPolynomial<Degree-1> PPolynomial<Degree>::derivative(void) const{
	PPolynomial<Degree-1> q;
	q.set(polyCount);
	for(size_t i=0;i<polyCount;i++){
		q.polys[i].start=polys[i].start;
		q.polys[i].p=polys[i].p.derivative();
	}
	return q;
}
template<int Degree>
PPolynomial<Degree+1> PPolynomial<Degree>::integral(void) const{
	int i;
	PPolynomial<Degree+1> q;
	q.set(polyCount);
	for(i=0;i<int(polyCount);i++){
		q.polys[i].start=polys[i].start;
		q.polys[i].p=polys[i].p.integral();
		q.polys[i].p-=q.polys[i].p(q.polys[i].start);
	}
	return q;
}
template<int Degree>
PPolynomial<Degree>& PPolynomial<Degree>::operator  += ( double s ) {polys[0].p+=s;}
template<int Degree>
PPolynomial<Degree>& PPolynomial<Degree>::operator  -= ( double s ) {polys[0].p-=s;}
template<int Degree>
PPolynomial<Degree>& PPolynomial<Degree>::operator  *= ( double s )
{
	for(int i=0;i<int(polyCount);i++){polys[i].p*=s;}
	return *this;
}
template<int Degree>
PPolynomial<Degree>& PPolynomial<Degree>::operator  /= ( double s )
{
	for(size_t i=0;i<polyCount;i++){polys[i].p/=s;}
	return *this;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::operator + ( double s ) const
{
	PPolynomial q=*this;
	q+=s;
	return q;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::operator - ( double s ) const
{
	PPolynomial q=*this;
	q-=s;
	return q;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::operator * ( double s ) const
{
	PPolynomial q=*this;
	q*=s;
	return q;
}
template<int Degree>
PPolynomial<Degree> PPolynomial<Degree>::operator / ( double s ) const
{
	PPolynomial q=*this;
	q/=s;
	return q;
}

template< >
PPolynomial< 0 > PPolynomial< 0 >::BSpline( double radius )
{
	PPolynomial q;
	q.set(2);

	q.polys[0].start=-radius;
	q.polys[1].start= radius;

	q.polys[0].p.coefficients[0]= 1.0;
	q.polys[1].p.coefficients[0]=-1.0;
	return q;
}
template< int Degree >
PPolynomial< Degree > PPolynomial<Degree>::BSpline( double radius )
{
	return PPolynomial< Degree-1 >::BSpline().MovingAverage( radius );
}
template<int Degree>
PPolynomial<Degree+1> PPolynomial<Degree>::MovingAverage( double radius ) const
{
	PPolynomial<Degree+1> A;
	Polynomial<Degree+1> p;
	Pointer( StartingPolynomial< Degree+1 > ) sps;
	sps = AllocPointer< StartingPolynomial< Degree+1 > >( polyCount*2 );


	for(int i=0;i<int(polyCount);i++){
		sps[2*i  ].start=polys[i].start-radius;
		sps[2*i+1].start=polys[i].start+radius;
		p=polys[i].p.integral()-polys[i].p.integral()(polys[i].start);
		sps[2*i  ].p=p.shift(-radius);
		sps[2*i+1].p=p.shift( radius)*-1;
	}
	A.set( sps , int(polyCount*2) );
	FreePointer( sps );
	return A*1.0/(2*radius);
}

template<int Degree>
void PPolynomial<Degree>::write(FILE* fp,int samples,double min,double max) const{
	fwrite(&samples,sizeof(int),1,fp);
	for(int i=0;i<samples;i++){
		double x=min+i*(max-min)/(samples-1);
		float v=(*this)(x);
		fwrite(&v,sizeof(float),1,fp);
	}
}
#endif // P_POLYNOMIAL_INCLUDED
