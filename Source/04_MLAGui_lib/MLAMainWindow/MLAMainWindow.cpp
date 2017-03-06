#include "MLAMainWindow.h"

MainWindow::MainWindow() :
m_windowTitle("Default name"), m_windowWidth(1600), m_windowHeight(900), m_window(), m_openGLContext(), m_input(), m_modelview(),
m_projection(), m_camera() {
}

MainWindow::MainWindow(std::string window_title, int window_width, int window_height) :
m_windowTitle(window_title), m_windowWidth(window_width), m_windowHeight(window_height), m_window(), m_openGLContext(), m_input(),
m_modelview(), m_projection(), m_camera() {
}

MainWindow::~MainWindow() {
	SDL_GL_DeleteContext(m_openGLContext);
	SDL_DestroyWindow(m_window);
	SDL_Quit();
}

int MainWindow::WindowInit() {

	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		std::cout << "SDL initialization error : " << SDL_GetError() << std::endl;
		SDL_Quit();

		return false;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

	m_window = SDL_CreateWindow(m_windowTitle.c_str(), SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, m_windowWidth, m_windowHeight, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

	if (!m_window) {
		std::cout << "SDL Error : " << SDL_GetError() << std::endl;
		return false;
	}

	m_openGLContext = SDL_GL_CreateContext(m_window);

	if (m_openGLContext == 0) {
		std::cout << SDL_GetError() << std::endl;
		SDL_DestroyWindow(m_window);
		SDL_Quit();

		return -1;
	}

	return true;
}

bool MainWindow::GLinit() {
	GLenum GLEW_initialization(glewInit());

	if (GLEW_initialization != GLEW_OK) {
		std::cout << "GLEW initialization error : " << glewGetErrorString(GLEW_initialization) << std::endl;

		SDL_GL_DeleteContext(m_openGLContext);
		SDL_DestroyWindow(m_window);
		SDL_Quit();

		return false;
	}

	glEnable(GL_DEPTH_TEST);

	return true;
}

bool MainWindow::LoadShader(std::string const vertexShader, std::string const fragmentShader) {
	
	m_shader.setVertexSource(vertexShader);
	m_shader.setFragmentSource(fragmentShader);

	if (!m_shader.Load()){
		std::cout << "Shader couldn't load." << std::endl;
		return false;
	}
	return true;
}

void MainWindow::MainLoop(Motion* motion) {

	float total_animation_time = motion->getFrames().size() * motion->getFrameTime() * 1000.0f;
	float loop_beginning = 0, loop_end = 0, elapsed_time = 0;

	float current_time = 0;
	float mix_factor = 0;

	// identity matrix
	m_modelview = glm::mat4(1.0);
	
	// glm::perspective(FOV, screen ratio, near, far)
	m_projection = glm::perspective(70.0, (double)1600.0/900.0, 0.1, 150.0);

	m_input.DisplayCursor(false);
	m_input.TrapMouse(true);

	while (!m_input.End()) {

		// Useful for frame limiting purpose (see at the end of the function)
		loop_beginning = (float)SDL_GetTicks();

		// Gathering the keyboard events
		m_input.EventUpdate();

		// if (esc), quit the program
		if (m_input.GetKey(SDL_SCANCODE_ESCAPE))
			break;

		// Handle camera's motion
		m_camera.Move(m_input);

		// Clean the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Orient camera
		m_camera.LookAt(m_modelview);

		//DrawStaticMotion(motion);
		mix_factor = (fmod(current_time / 1000.0f, motion->getFrameTime())) / motion->getFrameTime();
		
		Animate(motion, mix_factor, current_time/1000.0f);
		SDL_GL_SwapWindow(m_window);

		loop_end = (float)SDL_GetTicks();
		elapsed_time = loop_end - loop_beginning;

		current_time += elapsed_time;
		if (current_time > total_animation_time)
			current_time = 0;
	}
}

void MainWindow::DrawStaticMotion(Motion* motion) {
	float line_vertices[6];
	float point_vertice[3];
	float colour[6] = { 1, 0, 0, 1, 0, 0 };
	float smaller_color[3] = { 0, 1, 1 };

	glm::mat4 saved_modelview = m_modelview;
	
	// Dictionnary to keep the offset matrix for each joint
	std::map<std::string, glm::mat4> matrix_map;

	// j = joints
	for(unsigned int j=0 ; j<motion->getFrame(0)->getJoints().size(); j++) {

		// If it's the root
		if (motion->getFrame(0)->getJoints().at(j)->getParent() == 0) {
			
			// We translate to the position (mostly useless since the root is almost always at (0,0,0)
			saved_modelview = glm::translate(m_modelview, glm::vec3(motion->getFrame(0)->getJoints().at(j)->getPositions()[0],
				motion->getFrame(0)->getJoints().at(j)->getPositions()[1],
				motion->getFrame(0)->getJoints().at(j)->getPositions()[2]));

			//matrix_map.insert(std::pair<std::string, glm::mat4>(motion->getFrame(0)->getJoints().at(j)->getJointName(), saved_modelview));
		}

		else {
			// If we find the modelview matrix for the parent
			if (matrix_map.find(motion->getFrame(0)->getJoints().at(j)->getParent()->getJointName()) != matrix_map.end()){
				saved_modelview = matrix_map.find(motion->getFrame(0)->getJoints().at(j)->getParent()->getJointName())->second;
			}

			// Else, ABANDON SHIP
			else {
				std::cout << "Error : " << motion->getFrame(0)->getJoints().at(j)->getParent()->getJointName() << " not found in std::map<std::string, glm::mat4> matrix_map." << std::endl;
				break;
			}
		}

		// Since the parent's modelview will be at its offset, its coordinates are (0,0,0)
		line_vertices[0] = 0;
		line_vertices[1] = 0;
		line_vertices[2] = 0;

		// Since the offsets are relatives to the parent, AND that our current modelview
		// is at the parent's offset, I guess it's ok to do that.
		line_vertices[3] = motion->getFrame(0)->getJoints().at(j)->getPositions()[0];
		line_vertices[4] = motion->getFrame(0)->getJoints().at(j)->getPositions()[1];
		line_vertices[5] = motion->getFrame(0)->getJoints().at(j)->getPositions()[2];

		// Added to draw points (maybe adding coordinates later)
		point_vertice[0] = motion->getFrame(0)->getJoints().at(j)->getPositions()[0];
		point_vertice[1] = motion->getFrame(0)->getJoints().at(j)->getPositions()[1];
		point_vertice[2] = motion->getFrame(0)->getJoints().at(j)->getPositions()[2];

		// We draw our line
		DisplayPoint(m_projection, saved_modelview, point_vertice, smaller_color);
		DisplayLine(m_projection, saved_modelview, line_vertices, colour);

		// We add the offset to the current matrix ...
		saved_modelview = glm::translate(saved_modelview, glm::vec3(line_vertices[3],
			line_vertices[4],
			line_vertices[5]));

		// ... and we put it into our map
		matrix_map.insert(std::pair<std::string, glm::mat4>(motion->getFrame(0)->getJoints().at(j)->getJointName(), saved_modelview));
	}
}

void MainWindow::Animate(Motion* motion, float mixFactor, float elapsedTime) {
	float line_vertices[6];
	float point_vertice[3];
	float line_colour[6] = { 1, 1, 0, 1, 1, 0 };
	float point_colour[3] = { 1, 0, 1 };

	unsigned int base_frame = (int)(elapsedTime / motion->getFrameTime()) + 1;

	if (base_frame >= motion->getFrames().size() - 1){
		base_frame = 1;
	}

	// Used to store a vector of quaternion, for the sake of code clarity.
	std::vector<glm::quat> tmp_quat_vector;
	glm::quat current_quat, current_quat_1, current_quat_2;

	glm::mat4 saved_modelview = m_modelview;

	// Hold the 4x4 matrix from a quaternion.
	glm::mat4 quat_to_mat;

	std::map<std::string, glm::quat> quaternions_slerp_map;
	std::map<std::string, glm::mat4> quaternions_to_mat_map;

	// Used to interpolate the position of the root. For the sake of code clarity, we use 3 vector.
	glm::vec3 base_offset;
	glm::vec3 next_offset;
	glm::vec3 current_offset;

	// First, we interpolate each quaternion with the next frame's one.
	// For each joint
	for (unsigned int j = 0; j<motion->getFrame(0)->getJoints().size(); j++) {
		// We find the quaternion of the current frame (base frame) ...
		current_quat_1 = motion->getFrame(base_frame)->getJoints().at(j)->getOrientations();
		// ... and the next one (base frame + 1) ...
		current_quat_2 = motion->getFrame(base_frame + 1)->getJoints().at(j)->getOrientations();;
		// ... then we slerp (spherical interpolation) then.
		quaternions_slerp_map.insert(std::make_pair(motion->getFrame(base_frame)->getJoints().at(j)->getJointName(), glm::slerp(current_quat_1, current_quat_2, mixFactor)));
	}

	// For each graph node
	for (unsigned int j = 0; j<motion->getFrame(0)->getJoints().size(); j++) {
		
		// If it's the root
		if (!motion->getFrame(base_frame)->getJoints().at(j)->getParent()) {

			// We interpolate the position of the offset
			base_offset = glm::vec3(motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[0],
				motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[1],
				motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[2]);

			next_offset = glm::vec3(motion->getFrame(base_frame + 1)->getJoints().at(j)->getPositions()[0],
				motion->getFrame(base_frame + 1)->getJoints().at(j)->getPositions()[1],
				motion->getFrame(base_frame + 1)->getJoints().at(j)->getPositions()[2]);

			current_offset = glm::vec3(next_offset[0] + (base_offset[0] - next_offset[0])*mixFactor,
				next_offset[1] + (base_offset[1] - next_offset[1])*mixFactor,
				next_offset[2] + (base_offset[2] - next_offset[2])*mixFactor);


			saved_modelview = glm::translate(saved_modelview, current_offset);

			// We find the quat and transform it in a 4x4 matrix
			quat_to_mat = glm::mat4_cast(quaternions_slerp_map.find(motion->getFrame(base_frame)->getJoints().at(j)->getJointName())->second);
			// We do the rotation
			saved_modelview = saved_modelview*quat_to_mat;

			point_vertice[0] = current_offset[0];
			point_vertice[1] = current_offset[1];
			point_vertice[2] = current_offset[2];

			/*
			std::cout << "Displaying " << m_joint_graph.GetNodeValues(i).joint_name << " : ("
			<< point_vertice[0] << "," << point_vertice[1] << "," << point_vertice[2] << ")" << std::endl;
			*/

			DisplayPoint(m_projection, m_modelview, point_vertice, point_colour);

			// We pair it with the joint name and we put it into the map
			// Why ? FOR THE GLORY OF SATAN OF COURSE
			// (We need it for the rest of the program, to get the parent's modelview)
			quaternions_to_mat_map.insert(std::make_pair(motion->getFrame(base_frame)->getJoints().at(j)->getJointName(), saved_modelview));
		}

		// Not the root (other joints)
		else {
			// If we find the modelview matrix for the parent
			if (quaternions_to_mat_map.find(motion->getFrame(base_frame)->getJoints().at(j)->getParent()->getJointName()) != quaternions_to_mat_map.end()){
				saved_modelview = quaternions_to_mat_map.find(motion->getFrame(base_frame)->getJoints().at(j)->getParent()->getJointName())->second;
			}

			// Else, ABANDON SHIP
			else {
				std::cout << "Error : " << motion->getFrame(base_frame)->getJoints().at(j)->getParent()->getJointName() << " not found in std::map<std::string, glm::mat4> quaternions_to_mat_map." << std::endl;
				exit(EXIT_FAILURE);
			}

			// Origin (0,0,0) since the modelview is at the parent's origin
			line_vertices[0] = 0;
			line_vertices[1] = 0;
			line_vertices[2] = 0;

			line_vertices[3] = motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[0];
			line_vertices[4] = motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[1];
			line_vertices[5] = motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[2];


			point_vertice[0] = motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[0];
			point_vertice[1] = motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[1];
			point_vertice[2] = motion->getFrame(base_frame)->getJoints().at(j)->getPositions()[2];

			// We draw our line
			DisplayLine(m_projection, saved_modelview, line_vertices, line_colour);

			/*
			std::cout << "Displaying " << m_joint_graph.GetNodeValues(i).joint_name << " : ("
			<< point_vertice[0] << "," << point_vertice[1] << "," << point_vertice[2] << ")" << std::endl;
			std::cout << "Displaying between " << m_joint_graph.GetNodeValues(m_joint_graph.GetNode(i).parent).joint_name << " and " << m_joint_graph.GetNodeValues(i).joint_name << std::endl;
			*/

			// We draw the point
			DisplayPoint(m_projection, saved_modelview, point_vertice, point_colour);

			//We translate to the next joint (the one we just draw)
			saved_modelview = glm::translate(saved_modelview, glm::vec3(point_vertice[0], point_vertice[1], point_vertice[2]));

			// We find the quat and transform it in a 4x4 matrix
			quat_to_mat = glm::mat4_cast(quaternions_slerp_map.find(motion->getFrame(base_frame)->getJoints().at(j)->getJointName())->second);

			// We do the rotation
			saved_modelview = saved_modelview*quat_to_mat;

			// We pair it with the joint name and we put it into the map
			// Why ? FOR THE GLORY OF SATAN OF COURSE
			// (We need it for the rest of the program)
			quaternions_to_mat_map.insert(std::make_pair(motion->getFrame(base_frame)->getJoints().at(j)->getJointName(), saved_modelview));

		}
	}
}

void MainWindow::DisplayLine(glm::mat4 &projection, glm::mat4 &modelview, float *line_vertices, float *line_colour) {
	glUseProgram(m_shader.getProgramID());

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, line_vertices);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, line_colour);
	glEnableVertexAttribArray(1);

	glUniformMatrix4fv(glGetUniformLocation(m_shader.getProgramID(), "modelview"), 1, GL_FALSE, value_ptr(modelview));
	glUniformMatrix4fv(glGetUniformLocation(m_shader.getProgramID(), "projection"), 1, GL_FALSE, value_ptr(projection));

	glDrawArrays(GL_LINES, 0, 2);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glUseProgram(0);
}

void MainWindow::DisplayPoint(glm::mat4 &projection, glm::mat4 &modelview, float *point, float *point_color) {
	glPointSize(10);
	glUseProgram(m_shader.getProgramID());

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, point);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, point_color);
	glEnableVertexAttribArray(1);

	glUniformMatrix4fv(glGetUniformLocation(m_shader.getProgramID(), "modelview"), 1, GL_FALSE, value_ptr(modelview));
	glUniformMatrix4fv(glGetUniformLocation(m_shader.getProgramID(), "projection"), 1, GL_FALSE, value_ptr(projection));

	glDrawArrays(GL_POINTS, 0, 1);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glUseProgram(0);
}