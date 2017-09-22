#include "FrameRender.h"

FrameRender::FrameRender() {

}

FrameRender::~FrameRender() {

}

/** Draw a frame.

	@param interpolatedFrame: the frame to be drawn
	@param projection the projection matrix
	@param modelview the modelview matrix
	@param shader the shader
*/
void FrameRender::RenderFrame(Frame* frame, glm::dmat4& projection, glm::dmat4& modelview, Shader& shader) {
	double line_vertices[6];
	double point_vertice[3];
	double line_colour[6] = { 1, 1, 0, 1, 1, 0 };
	double point_colour[3] = { 1, 0, 1 };

	glm::dmat4 saved_modelview = modelview;

	std::map<std::string, glm::dmat4> quaternions_to_mat_map;
	
	// For each graph node
	for (unsigned int j = 0; j<frame->getJoints().size(); j++) {

		// If it's the root
		if (!frame->getJoint(j)->getParent()) {
			saved_modelview = glm::translate(saved_modelview, frame->getJoint(j)->getPositions());

			// We slerp them, transform into a matrix and we do the rotation
			saved_modelview = saved_modelview*glm::mat4_cast(frame->getJoint(j)->getOrientations());

			point_vertice[0] = frame->getJoint(j)->getPositions()[0];
			point_vertice[1] = frame->getJoint(j)->getPositions()[1];
			point_vertice[2] = frame->getJoint(j)->getPositions()[2];

			DisplayPoint(shader, projection, modelview, point_vertice, point_colour);

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(frame->getJoint(j)->getJointName(), saved_modelview));
		}

		// Not the root (other joints)
		else {
			// If we find the modelview matrix for the parent
			if (quaternions_to_mat_map.find(frame->getJoint(j)->getParent()->getJointName()) != quaternions_to_mat_map.end()){
				saved_modelview = quaternions_to_mat_map.find(frame->getJoint(j)->getParent()->getJointName())->second;
			}

			// Else, ABANDON SHIP
			else {
				std::cout << "Error : " << frame->getJoint(j)->getParent()->getJointName() << " not found in std::map<std::string, glm::mat4> quaternions_to_mat_map." << std::endl;
				exit(EXIT_FAILURE);
			}

			// Origin (0,0,0) since the modelview is at the parent's origin
			line_vertices[0] = 0;
			line_vertices[1] = 0;
			line_vertices[2] = 0;

			line_vertices[3] = frame->getJoint(j)->getPositions()[0];
			line_vertices[4] = frame->getJoint(j)->getPositions()[1];
			line_vertices[5] = frame->getJoint(j)->getPositions()[2];

			point_vertice[0] = line_vertices[3];
			point_vertice[1] = line_vertices[4];
			point_vertice[2] = line_vertices[5];

			// We draw our line
			DisplayLine(shader, projection, saved_modelview, line_vertices, line_colour);

			// We draw the point
			DisplayPoint(shader, projection, saved_modelview, point_vertice, point_colour);

			//We translate to the next joint (the one we just draw)
			saved_modelview = glm::translate(saved_modelview, glm::dvec3(point_vertice[0], point_vertice[1], point_vertice[2]));

			// We slerp them, transform into a matrix, and we do the rotation
			saved_modelview = saved_modelview*glm::mat4_cast(frame->getJoint(j)->getOrientations());

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(frame->getJoint(j)->getJointName(), saved_modelview));

		}
	}
}

/** Draw a skeleton from global coordinates.

	@param globalCoordinates the map containing the coordinates
	@param projection the projection matrix
	@param modelview the modelview matrix
	@param shader the shader
*/
void FrameRender::DrawFromGlobal(std::map<std::string, glm::dvec3>& globalCoordinates, glm::dmat4& projection, glm::dmat4& modelview, Shader& shader) {
	double point_vertice[3];
	double point_colour[3] = { 0, 1, 1 };

	glm::dmat4 saved_modelview = modelview;

	// j = joints
	for (std::map<std::string, glm::dvec3> ::iterator it = globalCoordinates.begin(); it != globalCoordinates.end(); ++it) {
		modelview = glm::translate(modelview, globalCoordinates.find(it->first)->second);

		point_vertice[0] = globalCoordinates.find(it->first)->second.x;
		point_vertice[1] = globalCoordinates.find(it->first)->second.y;
		point_vertice[2] = globalCoordinates.find(it->first)->second.z;

		DisplayPoint(shader, projection, modelview, point_vertice, point_colour);

		modelview = saved_modelview;
	}
}


// Conversion from dmat4 to mat4 : see
// https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_gpu_shader_fp64.txt
// to avoid this.
// Actually, there's a LOT to do to enable this. We have to switch from GL 3.1
// to 3.2 (or more), but it absolutely breaks the code. TODO, I guess.

/** Draw a line.
	@param shader the shader
	@param projection the projection matrix
	@param modelview the modelview matrix
	@param line_vertices an array containing the vertices of the line
	@param line_colour an array representing the color of the line

*/
void FrameRender::DisplayLine(Shader &shader, glm::dmat4 &projection, glm::dmat4 &modelview, const double *line_vertices, const double *line_colour) {
	glUseProgram(shader.getProgramID());

	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, line_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, line_colour);
	glEnableVertexAttribArray(1);

	glUniformMatrix4fv(glGetUniformLocation(shader.getProgramID(), "modelview"), 1, GL_FALSE, value_ptr((glm::mat4)modelview));
	glUniformMatrix4fv(glGetUniformLocation(shader.getProgramID(), "projection"), 1, GL_FALSE, value_ptr((glm::mat4)projection));

	glDrawArrays(GL_LINES, 0, 2);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glUseProgram(0);
}

/** Draw a point.
	@param shader the shader
	@param projection the projection matrix
	@param modelview the modelview matrix
	@param point array containing the point coordinates
	@param point_color array representing the color of the point

*/
void FrameRender::DisplayPoint(Shader &shader, glm::dmat4 &projection, glm::dmat4 &modelview, const double *point, const double *point_color) {
	glPointSize(10);
	glUseProgram(shader.getProgramID());

	glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, point);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, point_color);
	glEnableVertexAttribArray(1);

	glUniformMatrix4fv(glGetUniformLocation(shader.getProgramID(), "modelview"), 1, GL_FALSE, value_ptr((glm::mat4)modelview));
	glUniformMatrix4fv(glGetUniformLocation(shader.getProgramID(), "projection"), 1, GL_FALSE, value_ptr((glm::mat4)projection));

	glDrawArrays(GL_POINTS, 0, 1);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glUseProgram(0);
}