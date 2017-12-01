#ifndef __DEF_SHADER__
#define __DEF_SHADER__

#include <glew.h>

#include <iostream>
#include <string>
#include <fstream>

class Shader {

public:
	Shader();
	Shader(Shader const &shader_to_copy);
	Shader(std::string vertexSource, std::string source_fragment);
	~Shader();

	Shader& operator=(Shader const &shader_to_copy);

	void setSourceVertex(const std::string&);
	void setSourceFragment(const std::string&);

	bool Load();
	bool CompileShader(GLuint &shader, GLenum type, std::string const &source_file);
	const GLuint& getProgramID() const;


private:
	GLuint m_vertex_ID;
	GLuint m_fragment_ID;
	GLuint m_program_ID;

	std::string m_source_vertex;
	std::string m_source_fragment;
};

#endif //__DEF_SHADER__