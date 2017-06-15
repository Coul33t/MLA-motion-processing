#include "MLAMainWindow.h"

MainWindow::MainWindow() :
m_windowTitle("Default name"), m_windowWidth(1600), m_windowHeight(900), m_window(), m_openGLContext(), m_input(), m_modelview(),
m_projection(), m_camera(), m_frameRender(){
}

MainWindow::MainWindow(const std::string window_title, const int window_width, const int window_height) :
m_windowTitle(window_title), m_windowWidth(window_width), m_windowHeight(window_height), m_window(), m_openGLContext(), m_input(),
m_modelview(), m_projection(), m_camera(), m_frameRender() {
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

	//TODO: switch to GL 3.2+ (see DisplayLine() comment)
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

bool MainWindow::LoadShader(const std::string vertexShader, const std::string fragmentShader) {
	
	m_shader.setVertexSource(vertexShader);
	m_shader.setFragmentSource(fragmentShader);

	if (!m_shader.Load()){
		std::cout << "Shader couldn't load." << std::endl;
		return false;
	}
	return true;
}

void MainWindow::MainLoop(Motion* motion) {

	double total_animation_time = motion->getFrames().size() * motion->getFrameTime() * 1000.0f;
	double loop_beginning = 0, loop_end = 0, elapsed_time = 0;

	double current_time = 0;

	// slerp factor (value between 0 and 1, indicating " where "
	// we are between the two frames)
	double mix_factor = 0;

	// identity matrix
	m_modelview = glm::dmat4(1.0);
	
	// glm::perspective(FOV, screen ratio, near, far)
	m_projection = glm::perspective(70.0, 1600.0/900.0, 0.1, 250.0);

	m_input.DisplayCursor(false);
	m_input.TrapMouse(true);

	// Displaying offset (0) or animation (1)
	unsigned int display_type = 0;
	
	float current_speed = 1;

	bool neuron_connected = false;

	while (!m_input.End()) {

		// Useful for frame limiting purpose (see at the end of the function)
		loop_beginning = (float)SDL_GetTicks();

		// Gathering the keyboard events
		m_input.EventUpdate();

		// if (esc), quit the program
		if (m_input.GetKey(SDL_SCANCODE_ESCAPE))
			break;

		// Neuron connection
		if (m_input.GetKey(SDL_SCANCODE_I)) {
			if (!neuron_connected) {
				if (m_neuron.Connect())
					neuron_connected = !neuron_connected;
				else
					std::cout << "Can't connect to the Neuron." << std::endl;
			}

			else {
				m_neuron.KillConnection();
				neuron_connected = !neuron_connected;
			}	
		}

		if (m_input.GetKey(SDL_SCANCODE_O)) {
			if (neuron_connected) {
				m_neuron.ToggleExport();
				std::cout << "Export toggle." << std::endl;
			}
		}

		if (m_input.GetKey(SDL_SCANCODE_P)) {
			if (neuron_connected) {
				m_neuron.ToggleDisplay();
				std::cout << "Neuron display toggle." << std::endl;
			}
		}

		// Offset
		if (m_input.GetKey(SDL_SCANCODE_E)) {
			display_type = 0;
		}

		// Animation
		if (m_input.GetKey(SDL_SCANCODE_Q)) {
			display_type = 1;
		}

		if(display_type == 1) {
			if (m_input.GetKey(SDL_SCANCODE_Z) && current_speed > -1) {
				current_speed -= 0.1f;
				std::cout << "Current speed : " << current_speed << std::endl;
			}

			else if (m_input.GetKey(SDL_SCANCODE_X) && current_speed < 1) {
				current_speed += 0.1f; 
				std::cout << "Current speed : " << current_speed << std::endl;
			}

			else if (m_input.GetKey(SDL_SCANCODE_C)) {
				current_speed = -1;
				std::cout << "Current speed : " << current_speed << std::endl;
			}

			else if (m_input.GetKey(SDL_SCANCODE_V)) {
				current_speed = 0;
				std::cout << "Current speed : " << current_speed << std::endl;
			}

			else if (m_input.GetKey(SDL_SCANCODE_B)) {
				current_speed = 1;
				std::cout << "Current speed : " << current_speed << std::endl;
			}
		}

		// Handle camera's motion
		m_camera.Move(m_input);

		// Clean the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Orient camera
		m_camera.LookAt(m_modelview);

		// If the neuron is not connected
		if (!neuron_connected) {
			if (display_type == 0)
				m_frameRender.DrawStaticMotion(motion, m_projection, m_modelview, m_shader);

			else {
				mix_factor = (fmod(current_time / 1000.0, motion->getFrameTime())) / motion->getFrameTime();

				m_frameRender.Animate(motion, m_projection, m_modelview, m_shader, mix_factor, current_time / 1000.0);
			}
		}

		else {
			if(m_neuron.GetDisplay()) {
				NeuronAnimate();
			}
		}
		
		
		SDL_GL_SwapWindow(m_window);

		loop_end = SDL_GetTicks();
		elapsed_time = loop_end - loop_beginning;

		if(display_type == 1) {
			current_time += elapsed_time * current_speed;
			if (current_time < motion->getFrameTime())
				current_time = total_animation_time;
		}	

		else
			current_time = 0;

		if (current_time > total_animation_time)
			current_time = 0;
	}
}

void MainWindow::NeuronAnimate() {
	BvhParser parser;
	Motion* motion = parser.parseBvh("../../../Data/Bvh/display_tmp_bvh.bvh");
	
	double line_vertices[6];
	double point_vertice[3];
	double line_colour[6] = { 1, 1, 0, 1, 1, 0 };
	double point_colour[3] = { 1, 0, 1 };


	glm::dmat4 saved_modelview = m_modelview;

	std::map<std::string, glm::dmat4> quaternions_to_mat_map;

	int currentFrame = 1;
	// For each graph node
	for (unsigned int j = 0; j<motion->getFrame(currentFrame)->getJoints().size(); j++) {

		// If it's the root
		if (!motion->getFrame(1)->getJoints().at(j)->getParent()) {
			
			saved_modelview = glm::translate(saved_modelview, glm::dvec3(motion->getFrame(currentFrame)->getJoint(j)->getPositions()));
			saved_modelview = saved_modelview*glm::mat4_cast(motion->getFrame(currentFrame)->getJoint(j)->getOrientations());

			point_vertice[0] = motion->getFrame(currentFrame)->getJoint(j)->getPositions()[0];
			point_vertice[1] = motion->getFrame(currentFrame)->getJoint(j)->getPositions()[1];
			point_vertice[2] = motion->getFrame(currentFrame)->getJoint(j)->getPositions()[2];

			m_frameRender.DisplayPoint(m_shader, m_projection, m_modelview, point_vertice, point_colour);

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(motion->getFrame(currentFrame)->getJoint(j)->getJointName(), saved_modelview));
		}

		// Not the root (other joints)
		else {
			// If we find the modelview matrix for the parent
			if (quaternions_to_mat_map.find(motion->getFrame(currentFrame)->getJoint(j)->getParent()->getJointName()) != quaternions_to_mat_map.end()){
				saved_modelview = quaternions_to_mat_map.find(motion->getFrame(currentFrame)->getJoint(j)->getParent()->getJointName())->second;
			}

			// Else, ABANDON SHIP
			else {
				std::cout << "Error : " << motion->getFrame(currentFrame)->getJoint(j)->getParent()->getJointName() << " not found in std::map<std::string, glm::mat4> quaternions_to_mat_map." << std::endl;
				exit(EXIT_FAILURE);
			}

			// Origin (0,0,0) since the modelview is at the parent's origin
			line_vertices[0] = 0;
			line_vertices[1] = 0;
			line_vertices[2] = 0;

			line_vertices[3] = motion->getFrame(currentFrame)->getJoint(j)->getPositions()[0];
			line_vertices[4] = motion->getFrame(currentFrame)->getJoint(j)->getPositions()[1];
			line_vertices[5] = motion->getFrame(currentFrame)->getJoint(j)->getPositions()[2];

			point_vertice[0] = line_vertices[3];
			point_vertice[1] = line_vertices[4];
			point_vertice[2] = line_vertices[5];

			// We draw our line
			m_frameRender.DisplayLine(m_shader, m_projection, saved_modelview, line_vertices, line_colour);

			// We draw the point
			m_frameRender.DisplayPoint(m_shader, m_projection, saved_modelview, point_vertice, point_colour);

			//We translate to the next joint (the one we just draw)
			saved_modelview = glm::translate(saved_modelview, glm::dvec3(point_vertice[0], point_vertice[1], point_vertice[2]));
			saved_modelview = saved_modelview*glm::mat4_cast(motion->getFrame(currentFrame)->getJoint(j)->getOrientations());

			// We pair it with the joint name and we put it into the map
			quaternions_to_mat_map.insert(std::make_pair(motion->getFrame(currentFrame)->getJoint(j)->getJointName(), saved_modelview));

		}
	}
}