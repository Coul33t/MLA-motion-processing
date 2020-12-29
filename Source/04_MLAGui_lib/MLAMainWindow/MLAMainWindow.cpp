#include "MLAMainWindow.h"

MainWindow::MainWindow() :
m_window_title("Default name"), m_window_width(1600), m_window_height(900), m_window(), m_openGL_context(), m_input(), m_modelview(),
m_projection(), m_camera() {
}

MainWindow::MainWindow(const std::string window_title, const int window_width, const int window_height) :
m_window_title(window_title), m_window_width(window_width), m_window_height(window_height), m_window(), m_openGL_context(), m_input(),
m_modelview(), m_projection(), m_camera() {
}

MainWindow::~MainWindow() {
	SDL_GL_DeleteContext(m_openGL_context);
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

	m_window = SDL_CreateWindow(m_window_title.c_str(), SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, m_window_width, m_window_height, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

	if (!m_window) {
		std::cout << "SDL Error : " << SDL_GetError() << std::endl;
		return false;
	}

	m_openGL_context = SDL_GL_CreateContext(m_window);

	if (m_openGL_context == 0) {
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

		SDL_GL_DeleteContext(m_openGL_context);
		SDL_DestroyWindow(m_window);
		SDL_Quit();

		return false;
	}

	glEnable(GL_DEPTH_TEST);

	return true;
}

bool MainWindow::LoadShader(const std::string vertex_shader, const std::string fragment_shader) {
	
	m_shader.setSourceVertex(vertex_shader);
	m_shader.setSourceFragment(fragment_shader);

	if (!m_shader.Load()){
		std::cout << "Shader couldn't load." << std::endl;
		return false;
	}
	return true;
}

void MainWindow::MainLoop(std::shared_ptr<Motion> motion) {

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
	
	// Animation speed
	float current_speed = 1;

	bool neuron_connected = false;

	// Frame per frame display (if true, display the next frame)
	bool next_frame = false;
	unsigned int current_frame = 0;

	std::cout << "Controls (azerty keyboard):" << std::endl;
	std::cout << "Escape: quit" << std::endl;
	std::cout << "I: connect to Neuron" << std::endl;
	std::cout << "\tO: Export data" << std::endl;
	std::cout << "E: Display offset" << std::endl;
	std::cout << "A: Display animation" << std::endl;
	std::cout << "F: Frame per frame animation" << std::endl;
	std::cout << "\tN: Display next frame" << std::endl;
	std::cout << "Y: Output current frame and time" << std::endl;
	std::cout << "W: Slows the animation speed" << std::endl;
	std::cout << "X: Increasees the animation speed" << std::endl;
	std::cout << "C: Speed = -1 (reverse animation)" << std::endl;
	std::cout << "V: Speed = 0 (stop animation)" << std::endl;
	std::cout << "B: Speed = 1 (resume animation)" << std::endl;

	while (!m_input.End()) {

		// Useful for frame limiting purpose (see at the end of the function)
		loop_beginning = (float)SDL_GetTicks();

		// Gathering the keyboard events
		m_input.EventUpdate();

		// if (esc), quit the program
		if (m_input.GetKey(SDL_SCANCODE_ESCAPE))
			break;

		if (next_frame)
			next_frame = false;

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

		// Offset
		if (m_input.GetKey(SDL_SCANCODE_E)) {
			display_type = 0;
		}

		// Animation
		if (m_input.GetKey(SDL_SCANCODE_Q)) {
			display_type = 1;
		}

		// Frame per frame
		if (m_input.GetKey(SDL_SCANCODE_F)) {
			display_type = 2;
		}

		if (m_input.GetKey(SDL_SCANCODE_Y)) {
			std::cout << "Current time : " << current_time / 1000 << std::endl;
			std::cout << "Current frame : " << (int)((current_time / 1000.0) / motion->getFrameTime()) + 1 << std::endl;
			m_input.SetKey(SDL_SCANCODE_Y, false);
		}

		if(display_type == 1) {
			if (m_input.GetKey(SDL_SCANCODE_Z) && current_speed > -1) {
				current_speed -= 0.1f;
				std::cout << "Current speed : " << current_speed << std::endl;
				m_input.SetKey(SDL_SCANCODE_Z, false);
			}

			else if (m_input.GetKey(SDL_SCANCODE_X) && current_speed < 1) {
				current_speed += 0.1f; 
				std::cout << "Current speed : " << current_speed << std::endl;
				m_input.SetKey(SDL_SCANCODE_X, false);
			}

			else if (m_input.GetKey(SDL_SCANCODE_C)) {
				current_speed = -1;
				std::cout << "Current speed : " << current_speed << std::endl;
				m_input.SetKey(SDL_SCANCODE_C, false);
			}

			else if (m_input.GetKey(SDL_SCANCODE_V)) {
				current_speed = 0;
				std::cout << "Current speed : " << current_speed << std::endl;
				m_input.SetKey(SDL_SCANCODE_V, false);
			}

			else if (m_input.GetKey(SDL_SCANCODE_B)) {
				current_speed = 1;
				std::cout << "Current speed : " << current_speed << std::endl;
				m_input.SetKey(SDL_SCANCODE_B, false);
			}
		}

		else if (display_type == 2) {
			if (m_input.GetKey(SDL_SCANCODE_N)) {
				next_frame = true;
				std::cout << "Displaying next frame (frame " << current_frame << ")" << std::endl;
				m_input.SetKey(SDL_SCANCODE_N, false);
			}
		}

		// Handle camera's motion
		m_camera.Move(m_input);

		// Clean the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glClearColor(0.5f, 0.5f, 0.5f, 0.0f);

		// Orient camera
		m_camera.LookAt(m_modelview);

		// If the neuron is not connected
		if (!neuron_connected) {
			if (display_type == 0) {
				// GLOBAL CODE -------------------------------------
				/*Frame* global_frame = new Frame();
				global_frame = motion->getFrame(1)->duplicateFrame();
				glm::dmat4 mat = glm::mat4(1.0);
				Mla::MotionOperation::getGlobalCoordinates(motion->getFrame(1), global_frame, motion->getFrame(1)->getJoint("Hips"), mat);			
				Mla::FrameRender::DrawFromGlobal(global_frame, m_projection, m_modelview, m_shader);*/
				// --------------------------------------------------

				Mla::FrameRender::RenderFrame(motion->getFrame(1), m_projection, m_modelview, m_shader);
			}

			else if (display_type == 1) {
				mix_factor = (fmod(current_time / 1000.0, motion->getFrameTime())) / motion->getFrameTime();
				
				unsigned int base_frame = (int)((current_time / 1000.0) / motion->getFrameTime());

				if (base_frame >= motion->getFrames().size() - 1)
					base_frame = 0;

				std::shared_ptr<Frame> interpolated_frame = nullptr;
				interpolated_frame = Mla::MotionOperation::interpolateFrame(motion->getFrame(base_frame), motion->getFrame(base_frame + 1), mix_factor);
				// GLOBAL CODE -------------------------------------
				/*Frame* global_frame = new Frame();
				global_frame = interpolated_frame->duplicateFrame();
				glm::dmat4 mat = glm::mat4(1.0);
				Mla::MotionOperation::getGlobalCoordinates(interpolated_frame, global_frame, interpolated_frame->getJoint("Hips"), mat);
				Mla::FrameRender::DrawFromGlobal(global_frame, m_projection, m_modelview, m_shader);*/
				// --------------------------------------------------
				Mla::FrameRender::RenderFrame(interpolated_frame,
										      m_projection, 
										      m_modelview, 
										      m_shader);

				
				//delete interpolated_frame;
				//delete global_frame;
			}

			else if (display_type == 2) {
				if (next_frame) {
					current_frame += 1;

					if (current_frame >= motion->getFrames().size() - 1)
						current_frame = 0;
				}

				Mla::FrameRender::RenderFrame(motion->getFrame(current_frame),
					m_projection,
					m_modelview,
					m_shader);
			}
		}
		
		Mla::FrameRender::DrawXYZ(m_projection, m_modelview, m_shader);
		
		SDL_GL_SwapWindow(m_window);

		loop_end = SDL_GetTicks();
		elapsed_time = loop_end - loop_beginning;

		if(display_type == 1) {
			current_time += elapsed_time * current_speed;
			// ???
			/*if (current_time < motion->getFrameTime())
				current_time = total_animation_time;*/
		}	

		else
			current_time = 0;

		if (current_time > total_animation_time)
			current_time = 0;

	}

}