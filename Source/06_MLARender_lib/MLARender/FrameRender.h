#ifndef __MLA_RENDER_H__
#define __MLA_RENDER_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"

namespace Mla {
	namespace FrameRender {

		void RenderFrame(std::shared_ptr<Frame>, glm::dmat4&, glm::dmat4&, Shader&);
		void DrawFromGlobal(std::map<std::string, glm::dvec3>&, glm::dmat4&, glm::dmat4&, Shader&);
		void DrawFromGlobal(std::shared_ptr<Frame>, glm::dmat4&, glm::dmat4&, Shader&);
		void DrawXYZ(glm::dmat4&, glm::dmat4&, Shader&);

	};
};
#endif