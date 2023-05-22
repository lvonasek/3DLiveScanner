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
prior writften permission. 

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

#ifndef POINT_STREAM_INCLUDED
#define POINT_STREAM_INCLUDED
#include "Ply.h"
#include "Geometry.h"


template< class Real >
class OrientedPointStream
{
public:
	virtual ~OrientedPointStream( void ){}
	virtual void reset( void ) = 0;
	virtual bool nextPoint( OrientedPoint3D< Real >& p ) = 0;
	virtual int nextPoints( OrientedPoint3D< Real >* p , int count )
	{
		int c=0;
		for( int i=0 ; i<count ; i++ , c++ ) if( !nextPoint( p[i] ) ) break;
		return c;
	}
	void boundingBox( Point3D< Real >& min , Point3D< Real >& max )
	{
		bool first = true;
		OrientedPoint3D< Real > p;
		while( nextPoint( p ) )
		{
			for( int i=0 ; i<3 ; i++ )
			{
				if( first || p.p[i]<min[i] ) min[i] = p.p[i];
				if( first || p.p[i]>max[i] ) max[i] = p.p[i];
			}
			first = false;
		}
		reset();
	}
};

template< class Real , class Data >
class OrientedPointStreamWithData : public OrientedPointStream< Real >
{
public:
	virtual ~OrientedPointStreamWithData( void ){}
	virtual void reset( void ) = 0;
	virtual bool nextPoint( OrientedPoint3D< Real >& p , Data& d ) = 0;

	virtual bool nextPoint( OrientedPoint3D< Real >& p ){ Data d ; return nextPoint( p , d ); }
	virtual int nextPoints( OrientedPoint3D< Real >* p , Data* d , int count )
	{
		int c=0;
		for( int i=0 ; i<count ; i++ , c++ ) if( !nextPoint( p[i] , d[i] ) ) break;
		return c;
	}
	virtual int nextPoints( OrientedPoint3D< Real >* p , int count ){ return OrientedPointStream< Real >::nextPoints( p , count ); }
};

template< class Real >
class TransformedOrientedPointStream : public OrientedPointStream< Real >
{
	XForm4x4< Real > _xForm;
	XForm3x3< Real > _normalXForm;
	OrientedPointStream< Real >& _stream;
public:
	TransformedOrientedPointStream( XForm4x4< Real > xForm , OrientedPointStream< Real >& stream ) : _xForm(xForm) , _stream(stream)
	{
		for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) _normalXForm(i,j) = _xForm(i,j);
		_normalXForm = _normalXForm.transpose().inverse();
	};
	virtual void reset( void ){ _stream.reset(); }
	virtual bool nextPoint( OrientedPoint3D< Real >& p )
	{
		bool ret = _stream.nextPoint( p );
		p.p = _xForm * p.p , p.n = _normalXForm * p.n;
		return ret;
	}
};

template< class Real , class Data >
class TransformedOrientedPointStreamWithData : public OrientedPointStreamWithData< Real , Data >
{
	XForm4x4< Real > _xForm;
	XForm3x3< Real > _normalXForm;
	OrientedPointStreamWithData< Real , Data >& _stream;
public:
	TransformedOrientedPointStreamWithData( XForm4x4< Real > xForm , OrientedPointStreamWithData< Real , Data >& stream ) : _xForm(xForm) , _stream(stream)
	{
		for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) _normalXForm(i,j) = _xForm(i,j);
		_normalXForm = _normalXForm.transpose().inverse();
	};
	virtual void reset( void ){ _stream.reset(); }
	virtual bool nextPoint( OrientedPoint3D< Real >& p , Data& d )
	{
		bool ret = _stream.nextPoint( p , d );
		p.p = _xForm * p.p , p.n = _normalXForm * p.n;
		return ret;
	}
};

template< class Real >
class PLYOrientedPointStream : public OrientedPointStream< Real >
{
	char* _fileName;
	PlyFile* _ply;
	int _nr_elems;
	char **_elist;

	int _pCount , _pIdx;
	void _free( void );
public:
	PLYOrientedPointStream( const char* fileName );
	~PLYOrientedPointStream( void );
	void reset( void );
	bool nextPoint( OrientedPoint3D< Real >& p );
};

////////////////////////////
// PLYOrientedPointStream //
////////////////////////////
template< class Real >
PLYOrientedPointStream< Real >::PLYOrientedPointStream( const char* fileName )
{
	_fileName = new char[ strlen( fileName )+1 ];
	strcpy( _fileName , fileName );
	_ply = NULL;
	reset();
}
template< class Real >
void PLYOrientedPointStream< Real >::reset( void )
{
	int fileType;
	float version;
	PlyProperty** plist;
	if( _ply ) _free();
	_ply = ply_open_for_reading( _fileName, &_nr_elems, &_elist, &fileType, &version );
	if( !_ply )
	{
		fprintf( stderr, "[ERROR] Failed to open ply file for reading: %s\n" , _fileName );
		exit( 0 );
	}
	bool foundVertices = false;
	for( int i=0 ; i<_nr_elems ; i++ )
	{
		int num_elems;
		int nr_props;
		char* elem_name = _elist[i];
		plist = ply_get_element_description( _ply , elem_name , &num_elems , &nr_props );
		if( !plist )
		{
			fprintf( stderr , "[ERROR] Failed to get element description: %s\n" , elem_name );
			exit( 0 );
		}

		if( equal_strings( "vertex" , elem_name ) )
		{
			foundVertices = true;
			_pCount = num_elems , _pIdx = 0;
			for( int i=0 ; i<PlyOrientedVertex< Real >::ReadComponents ; i++ )
				if( !ply_get_property( _ply , elem_name , &(PlyOrientedVertex< Real >::ReadProperties[i]) ) )
				{
					fprintf( stderr , "[ERROR] Failed to find property in ply file: %s\n" , PlyOrientedVertex< Real >::ReadProperties[i].name );
					exit( 0 );
				}
		}
		for( int j=0 ; j<nr_props ; j++ )
		{
			free( plist[j]->name );
			free( plist[j] );
		}
		free( plist );
		if( foundVertices ) break;
	}
	if( !foundVertices )
	{
		fprintf( stderr , "[ERROR] Could not find vertices in ply file\n" );
		exit( 0 );
	}
}
template< class Real >
void PLYOrientedPointStream< Real >::_free( void )
{
	if( _ply ) ply_close( _ply ) , _ply = NULL;
	if( _elist )
	{
		for( int i=0 ; i<_nr_elems ; i++ ) free( _elist[i] );
		free( _elist );
	}
}
template< class Real >
PLYOrientedPointStream< Real >::~PLYOrientedPointStream( void )
{
	_free();
	if( _fileName ) delete[] _fileName , _fileName = NULL;
}
template< class Real >
bool PLYOrientedPointStream< Real >::nextPoint( OrientedPoint3D< Real >& p )
{
	if( _pIdx<_pCount )
	{
		PlyOrientedVertex< Real > op;
		ply_get_element( _ply, (void *)&op );
		p.p = op.point;
		p.n = op.normal;
		_pIdx++;
		return true;
	}
	else return false;
}

#endif // POINT_STREAM_INCLUDED
