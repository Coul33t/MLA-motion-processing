#ifndef __MLA_H__
#define __MLA_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"
#include "MLAParser_lib.h"
#include "MLAGui_lib.h"
#include "MLANeuron_lib.h"


#if defined PLATFORM_WIN32
//adresses pour le lecture des fichiers
#define	MLA_INPUT_PATH "../../../Data/"
#endif

#if defined PLATFORM_LINUX || defined PLATFORM_MACOSX
//adresses pour la lecture des fichiers
#define	MLA_INPUT_PATH "../Data/"
#endif

//adresses pour la lecture des fichiers
#define	MLA_INPUT_BVH_PATH MLA_INPUT_PATH  "Bvh/"
#define	MLA_INPUT_SHADER_PATH MLA_INPUT_PATH  "Shader/"



#endif //__MLA_H__