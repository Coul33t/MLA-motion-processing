#ifndef __MLA_MAINWINDOW_H__
#define __MLA_MAINWINDOW_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"
#include "MLAParser_lib.h"
#include "MLAGui_lib.h"
#include "MLANeuron_lib.h"
#include "MLARender_lib.h"

class MainWindow {

public:
	MainWindow();
	MainWindow(const std::string, const int, const int);
	~MainWindow();

	int WindowInit();
	bool GLinit();
	bool LoadShader(const std::string, const std::string);
	void MainLoop(Motion*);

	void DrawStaticMotion(Motion*);
	void Animate(Motion*, const double, const double);

	void NeuronAnimate();
	

private:
	std::string m_windowTitle;
	int m_windowWidth;
	int m_windowHeight;

	SDL_Window* m_window;
	SDL_GLContext m_openGLContext;
	SDL_Event m_events;
	Input m_input;

	int m_state;

	Shader m_shader;

	FrameRender m_frameRender;

	glm::dmat4 m_modelview;
	glm::dmat4 m_projection;

	Camera m_camera;

	NeuronConnection m_neuron;
};
#endif //__MLA_MAINWINDOW_H__