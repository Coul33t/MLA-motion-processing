#include "FrameRender.h"

FrameRender::FrameRender() {

}

FrameRender::~FrameRender() {

}

void FrameRender::DrawStaticMotion(Motion* motion, glm::dmat4& projection, glm::dmat4& modelview, Shader& shader) {
	double line_vertices[6];
	double point_vertice[3];
	double colour[6] = { 1, 0, 0, 1, 0, 0 };
	double point_colour[3] = { 0, 1, 1 };

	glm::dmat4 saved_modelview = modelview;

	// Dictionnary to keep the offset matrix for each joint
	std::map<std::string, glm::dmat4> quaternions_to_mat_map;

	Frame* frame_to_display = motion->getFrame(1);

	// j = joints
	for (unsigned int j = 0; j<frame_to_display->getJoints().size(); j++) {

		// If it's the root
		if (frame_to_display->getJoint(j)->getParent() == 0) {

			// We translate to the position
			saved_modelview = glm::translate(modelview, frame_to_display->getJoint(j)->getPositions());

			// We transform into a matrix and we do the rotation
			saved_modelview = saved_modelview*glm::mat4_cast(frame_to_display->getJoint(j)->getOrientations());

			point_vertice[0] = frame_to_display->getJoint(j)->getPositions()[0];
			point_vertice[1] = frame_to_display->getJoint(j)->getPositions()[1];
			point_vertice[2] = frame_to_display->getJoint(j)->getPositions()[2];

			DisplayPoint(shader, projection, modelview, point_vertice, point_colour);

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(frame_to_display->getJoint(j)->getJointName(), saved_modelview));
		}

		else {
			// If we find the modelview matrix for the parent
			if (quaternions_to_mat_map.find(frame_to_display->getJoint(j)->getParent()->getJointName()) != quaternions_to_mat_map.end()){
				saved_modelview = quaternions_to_mat_map.find(frame_to_display->getJoint(j)->getParent()->getJointName())->second;
			}

			// Else, ABANDON SHIP
			else {
				std::cout << "Error : " << frame_to_display->getJoint(j)->getParent()->getJointName() << " not found in std::map<std::string, glm::mat4> quaternions_to_mat_map." << std::endl;
				exit(EXIT_FAILURE);
			}
		}

		// Since the parent's modelview will be at its offset, its coordinates are (0,0,0)
		line_vertices[0] = 0;
		line_vertices[1] = 0;
		line_vertices[2] = 0;

		// Since the offsets are relatives to the parent, AND that our current modelview
		// is at the parent's offset, I guess it's ok to do that.
		line_vertices[3] = motion->getFrame(0)->getJoint(j)->getPositions()[0];
		line_vertices[4] = motion->getFrame(0)->getJoint(j)->getPositions()[1];
		line_vertices[5] = motion->getFrame(0)->getJoint(j)->getPositions()[2];

		// Added to draw points
		point_vertice[0] = line_vertices[3];
		point_vertice[1] = line_vertices[4];
		point_vertice[2] = line_vertices[5];

		// We draw the point and the line
		DisplayPoint(shader, projection, saved_modelview, point_vertice, point_colour);
		DisplayLine(shader, projection, saved_modelview, line_vertices, colour);

		// We add the offset to the current matrix ...
		saved_modelview = glm::translate(saved_modelview, glm::dvec3(point_vertice[0], point_vertice[1], point_vertice[2]));

		saved_modelview = saved_modelview*glm::mat4_cast(frame_to_display->getJoint(j)->getOrientations());
		quaternions_to_mat_map.insert(std::make_pair(frame_to_display->getJoint(j)->getJointName(), saved_modelview));
	}
}

void FrameRender::Animate(Motion* motion, glm::dmat4& projection, glm::dmat4& modelview, Shader& shader, const double mixFactor, const double elapsedTime) {
	double line_vertices[6];
	double point_vertice[3];
	double line_colour[6] = { 1, 1, 0, 1, 1, 0 };
	double point_colour[3] = { 1, 0, 1 };

	unsigned int base_frame = (int)(elapsedTime / motion->getFrameTime()) + 1;

	if (base_frame >= motion->getFrames().size() - 1){
		base_frame = 1;
	}

	glm::dmat4 saved_modelview = modelview;

	std::map<std::string, glm::dmat4> quaternions_to_mat_map;

	Frame* interpolatedFrame = motion->interpolateFrame(motion->getFrame(base_frame), motion->getFrame(base_frame + 1), mixFactor);
	
	// For each graph node
	for (unsigned int j = 0; j<interpolatedFrame->getJoints().size(); j++) {

		// If it's the root
		if (!interpolatedFrame->getJoint(j)->getParent()) {
			saved_modelview = glm::translate(saved_modelview, interpolatedFrame->getJoint(j)->getPositions());

			// We slerp them, transform into a matrix and we do the rotation
			saved_modelview = saved_modelview*glm::mat4_cast(interpolatedFrame->getJoint(j)->getOrientations());

			point_vertice[0] = interpolatedFrame->getJoint(j)->getPositions()[0];
			point_vertice[1] = interpolatedFrame->getJoint(j)->getPositions()[1];
			point_vertice[2] = interpolatedFrame->getJoint(j)->getPositions()[2];

			DisplayPoint(shader, projection, modelview, point_vertice, point_colour);

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(interpolatedFrame->getJoint(j)->getJointName(), saved_modelview));
		}

		// Not the root (other joints)
		else {
			// If we find the modelview matrix for the parent
			if (quaternions_to_mat_map.find(interpolatedFrame->getJoint(j)->getParent()->getJointName()) != quaternions_to_mat_map.end()){
				saved_modelview = quaternions_to_mat_map.find(interpolatedFrame->getJoint(j)->getParent()->getJointName())->second;
			}

			// Else, ABANDON SHIP
			else {
				std::cout << "Error : " << interpolatedFrame->getJoint(j)->getParent()->getJointName() << " not found in std::map<std::string, glm::mat4> quaternions_to_mat_map." << std::endl;
				exit(EXIT_FAILURE);
			}

			// Origin (0,0,0) since the modelview is at the parent's origin
			line_vertices[0] = 0;
			line_vertices[1] = 0;
			line_vertices[2] = 0;

			line_vertices[3] = interpolatedFrame->getJoint(j)->getPositions()[0];
			line_vertices[4] = interpolatedFrame->getJoint(j)->getPositions()[1];
			line_vertices[5] = interpolatedFrame->getJoint(j)->getPositions()[2];

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
			saved_modelview = saved_modelview*glm::mat4_cast(interpolatedFrame->getJoint(j)->getOrientations());

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(interpolatedFrame->getJoint(j)->getJointName(), saved_modelview));

		}
	}
}




// Conversion from dmat4 to mat4 : see
// https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_gpu_shader_fp64.txt
// to avoid this.
// Actually, there's a LOT to do to enable this. We have to switch from GL 3.1
// to 3.2 (or more), but it absolutely breaks the code. TODO, I guess.
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