#include "FrameRender.h"

namespace Mla {
	namespace FrameRender {

		namespace {
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
			void DisplayLine(Shader &shader, glm::dmat4 &projection, glm::dmat4 &modelview, const double *line_vertices, const double *line_colour) {
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
			void DisplayPoint(Shader &shader, glm::dmat4 &projection, glm::dmat4 &modelview, const double *point, const double *point_color) {
				glPointSize(5);
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
		}

		/** Draw a frame.

		@param interpolatedFrame: the frame to be drawn
		@param projection the projection matrix
		@param modelview the modelview matrix
		@param shader the shader
		*/
		void RenderFrame(Frame* frame, glm::dmat4& projection, glm::dmat4& modelview, Shader& shader) {
			double line_vertices[6];
			double point_vertice[3];
			double line_colour[6] = { 1, 1, 0, 1, 1, 0 };
			double point_colour[3] = { 1, 0, 1 };

			glm::dmat4 saved_modelview = modelview;

			std::map<std::string, glm::dmat4> quaternions_to_mat_map;

			// For each graph node
			for (unsigned int j = 0; j < frame->getJoints().size(); j++) {

				// If it's the root
				if (frame->getJoint(j) == frame->getRoot()) {
					saved_modelview = glm::translate(saved_modelview, frame->getJoint(j)->getPositions());

					// We slerp them, transform into a matrix and we do the rotation
					saved_modelview = saved_modelview*glm::mat4_cast(frame->getJoint(j)->getOrientations());

					point_vertice[0] = frame->getJoint(j)->getPositions()[0];
					point_vertice[1] = frame->getJoint(j)->getPositions()[1];
					point_vertice[2] = frame->getJoint(j)->getPositions()[2];

					DisplayPoint(shader, projection, modelview, point_vertice, point_colour);

					// We pair it with the joint name and we put it into the map
					quaternions_to_mat_map.insert(std::make_pair(frame->getJoint(j)->getName(), saved_modelview));
				}

				// Not the root (other joints)
				else {

					std::map<std::string, glm::dmat4>::iterator it_quat = quaternions_to_mat_map.find(frame->getJoint(j)->getParent()->getName());

					// If we don't find the modelview matrix for the parent, ABANDON SHIP
					if (it_quat == quaternions_to_mat_map.end()){
						std::cout << "Error : " << frame->getJoint(j)->getParent()->getName() << " not found in std::map<std::string, glm::mat4> quaternions_to_mat_map." << std::endl;
						exit(EXIT_FAILURE);
					}
					
					saved_modelview = it_quat->second;

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
					quaternions_to_mat_map.insert(std::make_pair(frame->getJoint(j)->getName(), saved_modelview));

				}
			}
		}

		/** Draw a skeleton from global coordinates.

		@param frame the frame (global coordinates)
		@param projection the projection matrix
		@param modelview the modelview matrix
		@param shader the shader
		*/
		void DrawFromGlobal(Frame* frame, glm::dmat4& projection, glm::dmat4& modelview, Shader& shader) {
			double line_vertices[6];
			double line_colour[6] = { 0, 1, 0, 1, 0, 1 };
			double point_vertice[3];
			double point_colour[3] = { 0, 1, 1 };

			// j = joints
			for (unsigned int j = 0; j < frame->getJoints().size(); j++) {

				point_vertice[0] = frame->getJoint(j)->getPositions().x;
				point_vertice[1] = frame->getJoint(j)->getPositions().y;
				point_vertice[2] = frame->getJoint(j)->getPositions().z;

				DisplayPoint(shader, projection, modelview, point_vertice, point_colour);

				if(frame->getJoint(j)->getParent()) {
					line_vertices[0] = frame->getJoint(j)->getParent()->getPositions()[0];
					line_vertices[1] = frame->getJoint(j)->getParent()->getPositions()[1];
					line_vertices[2] = frame->getJoint(j)->getParent()->getPositions()[2];
					line_vertices[3] = frame->getJoint(j)->getPositions()[0];
					line_vertices[4] = frame->getJoint(j)->getPositions()[1];
					line_vertices[5] = frame->getJoint(j)->getPositions()[2];

					DisplayLine(shader, projection, modelview, line_vertices, line_colour);
				}
			}
		}

		void DrawXYZ(glm::dmat4& projection, glm::dmat4& modelview, Shader& shader) {
			std::vector<double> l_v;
			std::vector<double> l_c;
			l_v = { 0,0,0,25,0,0 };
			l_c = { 1,0,0,1,0,0 };
			double *line_vertices = &l_v[0];
			double *line_colour = &l_c[0];
			DisplayLine(shader, projection, modelview, line_vertices, line_colour);

			l_v = { 0,0,0,0,25,0 };
			l_c = { 0,1,0,0,1,0 };
			line_vertices = &l_v[0];
			line_colour = &l_c[0];
			DisplayLine(shader, projection, modelview, line_vertices, line_colour);

			l_v = { 0,0,0,0,0,25 };
			l_c = { 0,0,1,0,0,1 };
			line_vertices = &l_v[0];
			line_colour = &l_c[0];
			DisplayLine(shader, projection, modelview, line_vertices, line_colour);
		}
	}
}