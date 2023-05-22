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

#include <stdlib.h>
#include "CmdLineParser.h"

cmdLineReadable::cmdLineReadable(const char* name)
{
	set=false;
	this->name=new char[strlen(name)+1];
	strcpy(this->name,name);
}
cmdLineReadable::~cmdLineReadable(void)
{
	if(name) delete[] name;
	name=NULL;
}
int cmdLineReadable::read(char**,int){
	set=true;
	return 0;
}

////////////////
// cmdLineInt //
////////////////
cmdLineInt::cmdLineInt(const char* name) : cmdLineReadable(name) {value=0;}
cmdLineInt::cmdLineInt(const char* name,const int& v) : cmdLineReadable(name) {value=v;}
int cmdLineInt::read(char** argv,int argc){
	if(argc>0){
		value=atoi(argv[0]);
		set=true;
		return 1;
	}
	else{return 0;}
}

cmdLineFloat::cmdLineFloat(const char* name, const float& v) : cmdLineReadable(name) {value=v;}
int cmdLineFloat::read(char** argv,int argc){
	if(argc>0){
		value=(float)atof(argv[0]);
		set=true;
		return 1;
	}
	else{return 0;}
}

///////////////////
// cmdLineString //
///////////////////
cmdLineString::cmdLineString(const char* name) : cmdLineReadable(name) {value=NULL;}
cmdLineString::~cmdLineString(void)
{
	if(value)	delete[] value;
	value=NULL;
}
int cmdLineString::read(char** argv,int argc){
	if(argc>0)
	{
		value=new char[strlen(argv[0])+1];
		strcpy(value,argv[0]);
		set=true;
		return 1;
	}
	else{return 0;}
}

void cmdLineParse(int argc, char **argv,int num,cmdLineReadable** readable,int dumpError)
{
	int i,j;
	while (argc > 0)
	{
		if (argv[0][0] == '-' && argv[0][1]=='-')
		{
			for(i=0;i<num;i++)
			{
				if (!strcmp(&argv[0][2],readable[i]->name))
				{
					argv++, argc--;
					j=readable[i]->read(argv,argc);
					argv+=j,argc-=j;
					break;
				}
			}
			if(i==num){
				if(dumpError)
				{
					fprintf(stderr, "invalid option: %s\n",*argv);
					fprintf(stderr, "possible options are:\n");
					for(i=0;i<num;i++)	fprintf(stderr, "  %s\n",readable[i]->name);
				}
				argv++, argc--;
			}
		}
		else
		{
			if(dumpError)
			{
				fprintf(stderr, "invalid option: %s\n", *argv);
				fprintf(stderr, "  options must start with a \'--\'\n");
			}
			argv++, argc--;
		}
	}
}