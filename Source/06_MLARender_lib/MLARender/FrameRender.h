#ifndef __MLA_RENDER_H__
#define __MLA_RENDER_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"

namespace mla {
	namespace framerender{

		void RenderFrame(Frame*, glm::dmat4&, glm::dmat4&, Shader&);
		void DrawFromGlobal(std::map<std::string, glm::dvec3>&, glm::dmat4&, glm::dmat4&, Shader&);

	};
};
#endif