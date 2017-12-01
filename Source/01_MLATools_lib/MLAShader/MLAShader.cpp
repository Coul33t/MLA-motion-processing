#include "MlaShader.h"


// Constructors
Shader::Shader() : m_vertex_ID(0), m_fragment_ID(0), m_program_ID(0), m_source_vertex(), m_source_fragment()
{
}


Shader::Shader(Shader const &shader_to_copy) {

	// Source file copy
	m_source_vertex = shader_to_copy.m_source_vertex;
	m_source_fragment = shader_to_copy.m_source_fragment;


	// Shader loading
	Load();
}


Shader::Shader(std::string source_vertex, std::string source_fragment) : m_vertex_ID(0), m_fragment_ID(0), m_program_ID(0),
m_source_vertex(source_vertex), m_source_fragment(source_fragment) {
}

// Destructor
Shader::~Shader() {
	if(m_vertex_ID)
		glDeleteShader(m_vertex_ID);
	if(m_fragment_ID)
		glDeleteShader(m_fragment_ID);
	if(m_program_ID)
		glDeleteProgram(m_program_ID);
}


// Methods

Shader& Shader::operator=(Shader const &shader_to_copy) {

	// Source file copy
	m_source_vertex = shader_to_copy.m_source_vertex;
	m_source_fragment = shader_to_copy.m_source_fragment;

	// Shader loading
	Load();

	return *this;
}


void Shader::setSourceVertex(const std::string& source_vertex) {
	m_source_vertex = source_vertex;
}
void Shader::setSourceFragment(const std::string& source_fragment) {
	m_source_fragment = source_fragment;
}

bool Shader::Load() {

	// (potential) Old shader destruction
	if (glIsShader(m_vertex_ID) == GL_TRUE)
		glDeleteShader(m_vertex_ID);

	if (glIsShader(m_fragment_ID) == GL_TRUE)
		glDeleteShader(m_fragment_ID);

	if (glIsProgram(m_program_ID) == GL_TRUE)
		glDeleteProgram(m_program_ID);


	// New shaders compilation
	if (!CompileShader(m_vertex_ID, GL_VERTEX_SHADER, m_source_vertex))
		return false;

	if (!CompileShader(m_fragment_ID, GL_FRAGMENT_SHADER, m_source_fragment))
		return false;


	// Program creation
	m_program_ID = glCreateProgram();


	// Shaders binding
	glAttachShader(m_program_ID, m_vertex_ID);
	glAttachShader(m_program_ID, m_fragment_ID);


	// Shaders input locking
	glBindAttribLocation(m_program_ID, 0, "in_Vertex");
	glBindAttribLocation(m_program_ID, 1, "in_Color");
	glBindAttribLocation(m_program_ID, 2, "in_TexCoord0");


	// Program linking
	glLinkProgram(m_program_ID);


	// Linking check
	GLint linkError(0);
	glGetProgramiv(m_program_ID, GL_LINK_STATUS, &linkError);


	if (linkError != GL_TRUE) {
		// Error size
		GLint error_size(0);
		glGetProgramiv(m_program_ID, GL_INFO_LOG_LENGTH, &error_size);

		// Memory allocation
		char *error = new char[error_size + 1];

		// Error recovering
		glGetShaderInfoLog(m_program_ID, error_size, &error_size, error);
		error[error_size] = '\0';

		// Error display
		std::cout << error << std::endl;


		// Freeing memory and returning false
		delete[] error;
		glDeleteProgram(m_program_ID);

		return false;
	}



	// Else, everybody went good

	else
		return true;
}


bool Shader::CompileShader(GLuint &shader, GLenum type, std::string const &source_file) {

	// Shader creation
	shader = glCreateShader(type);

	// Shader checking
	if (shader == 0) {
		std::cout << "Error, shader type (" << type << ") does not exist" << std::endl;
		return false;
	}

	// reading stream
	std::ifstream file(source_file.c_str());

	// Opening test
	if (!file) {
		std::cout << "Error, file " << source_file << " not found" << std::endl;
		glDeleteShader(shader);
		return false;
	}

	// String to read the source code
	std::string line;
	std::string source_code;

	// Reading
	while (getline(file, line))
		source_code += line + '\n';

	// File closing
	file.close();

	// Retrieving of the C string
	const GLchar* source_code_chain = source_code.c_str();

	// Sending the source code to the shader
	glShaderSource(shader, 1, &source_code_chain, 0);

	// Shader compilation
	glCompileShader(shader);

	// Compilation checking
	GLint compilation_error(0);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compilation_error);


	if (compilation_error != GL_TRUE) {
		// Error size retrieving
		GLint error_size(0);
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &error_size);

		// Memory allocation
		char *error = new char[error_size + 1];

		// Error retrieving
		glGetShaderInfoLog(shader, error_size, &error_size, error);
		error[error_size] = '\0';

		// Error display
		std::cout << error << std::endl;

		// Freeing memory and returning false
		delete[] error;
		glDeleteShader(shader);

		return false;
	}

	else
		return true;
}


// Getter
const GLuint& Shader::getProgramID() const {
	return m_program_ID;
}
