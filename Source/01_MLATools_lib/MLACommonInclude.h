
#ifndef __MLA_TOOLS_INTERN_INCLUDE_FILES__ 
#define __MLA_TOOLS_INTERN_INCLUDE_FILES__  


////////////////////////////////////////////////////////////////////////////////////////////////////////
// Platform definition
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Linux
#if defined linux || defined __linux__ || defined __linux
#  define PLATFORM_LINUX
#  define PLATFORM_NAME "Linux"
#endif

// Windows
#if defined _WIN32 || defined WIN32 || defined __NT__ || defined __WIN32__
#  define PLATFORM_WIN32
#  define PLATFORM_NAME "Windows"
#endif

// MacOS X
#if ( defined __MWERKS__ && defined __powerc && !defined macintosh ) || defined __APPLE_CC__ || defined macosx || defined(MACOSX) || defined(__APPLE__) || defined(macintosh)
#  define PLATFORM_MACOSX
#  define PLATFORM_NAME "MacOS X"
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Common libraries
////////////////////////////////////////////////////////////////////////////////////////////////////////

// string
#include <string>


// stream
#include <iostream>
#include <fstream>
#include <sstream>


// stl
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <map>

// glew
#include "glew.h"

// SDL
#include "SDL.h"

// glm
// Ignore warning related to glm
#pragma warning(push, 0)
#include "glm.hpp"
#include "gtx/transform.hpp"
#include "gtc/type_ptr.hpp"
#include "gtc/quaternion.hpp"
#pragma warning(pop)

///////////////////////////////////////////////////////////////////////////////////////////////////////
// macro for math functions
////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef M_PI
// in c++11, M_PI is no more supported (at least here)
// and using std::atan(1.0)*4 can results in some precision
// loss and added compilation time (negligible).
// For all of this reasons, it's better to manually input the values.
#define M_PI 3.141592653589793238462643383279502884
#endif

#define	FLOAT_CMP(float1,float2) (fabs(float1 - float2) < FLT_EPSILON) 
#define	DOUBLE_CMP(double1,double2) (fabs(double1 - double2) < DBL_EPSILON) 
#define SWAP(type,x,y) {type t=x; x=y; y=t; }
#define is_power_of_two(x) ( ((x&(x-1)) == 0) && (x>0) )

////////////////////////////////////////////////////////////////////////////////////////////////////////
// standard outuput for debugging and unexpected behavior
////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined _DEBUG || defined DEBUG || defined DEBUG_RELEASE 

#define	MlaECoutLine(msg)	 std::cout <<__FILE__ <<"," <<std::to_string(__LINE__) << "," << __FUNCTION__ << ";"<< msg <<std::endl; 
#define	MlaECout(msg)		 std::cout <<__FILE__ <<"," <<std::to_string(__LINE__) << "," << __FUNCTION__ << ";"<< msg;


#define	MlaCoutLine(msg)	 std::cout  <<  msg <<std::endl; 
#define	MlaCout(msg)		 std::cout << msg;

#define	MlaLine(msg)		 std::cout<< std::endl; 

#else // _DEBUG

#define	MlaECoutLine(msg)	
#define	MlaECout(msg)		 

#define	MlaCoutLine(msg)	
#define	MlaCout(msg)

#define	MlaLine(msg)		

#endif	// _DEBUG

////////////////////////////////////////////////////////////////////////////////////////////////////////
// specific variable type
////////////////////////////////////////////////////////////////////////////////////////////////////////

//empty string 
static std::string EMPTY_STRING(std::string(""));

#endif //__MLA_TOOLS_INTERN_INCLUDE_FILES__  