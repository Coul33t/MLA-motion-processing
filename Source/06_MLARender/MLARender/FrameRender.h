#ifndef __MLA_RENDER_H__
#define __MLA_RENDER_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"

class FrameRender {
public:
	FrameRender();
	~FrameRender();
	
	/** Draw a frame.

		@param Frame: the frame to be drawn
		@param dmat4: the projection matrix
		@param dmat4: the modelview matrix
		@param Shader: the shader
	*/
	void RenderFrame(Frame*, glm::dmat4&, glm::dmat4&, Shader&);

	/** Draw a skeleton from global coordinates.

			@param map<string, dvec3>: the map containing the coordinates
			@param dmat4: the projection matrix
			@param dmat4: the modelview matrix
			@param Shader: the shader
	*/
	void DrawFromGlobal(std::map<std::string, glm::dvec3>&, glm::dmat4&, glm::dmat4&, Shader&);

	/** Draw a line.
			@param Shader: the shader
			@param dmat4: the projection matrix
			@param dmat4: the modelview matrix
			@param double*: array containing the vertices of the line
			@param double*: array representing the color of the line
	
	*/
	void DisplayLine(Shader&, glm::dmat4&, glm::dmat4&, const double*, const double*);

	/** Draw a point.
		@param Shader: the shader
		@param dmat4: the projection matrix
		@param dmat4: the modelview matrix
		@param double*: array containing the point coordinates
		@param double*: array representing the color of the point

	*/
	void DisplayPoint(Shader&, glm::dmat4&, glm::dmat4&, const double*, const double*);
};

#endif