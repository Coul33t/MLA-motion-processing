#include "MlaShader.h"


// Constructors
Shader::Shader() : m_vertexID(0), m_fragmentID(0), m_programID(0), m_vertexSource(), m_fragmentSource()
{
}


Shader::Shader(Shader const &shaderToCopy) {

	// Source file copy
	m_vertexSource = shaderToCopy.m_vertexSource;
	m_fragmentSource = shaderToCopy.m_fragmentSource;


	// Shader loading
	Load();
}


Shader::Shader(std::string vertexSource, std::string fragmentSource) : m_vertexID(0), m_fragmentID(0), m_programID(0),
m_vertexSource(vertexSource), m_fragmentSource(fragmentSource) {
}

// Destructor
Shader::~Shader() {
	if(m_vertexID)
		glDeleteShader(m_vertexID);
	if(m_fragmentID)
		glDeleteShader(m_fragmentID);
	if(m_programID)
		glDeleteProgram(m_programID);
}


// Methods

Shader& Shader::operator=(Shader const &shaderToCopy) {

	// Source file copy
	m_vertexSource = shaderToCopy.m_vertexSource;
	m_fragmentSource = shaderToCopy.m_fragmentSource;

	// Shader loading
	Load();

	return *this;
}


void Shader::setVertexSource(const std::string& vertexSource) {
	m_vertexSource = vertexSource;
}
void Shader::setFragmentSource(const std::string& fragmentSource) {
	m_fragmentSource = fragmentSource;
}

bool Shader::Load() {

	// Destruction of a potential old shader
	if (glIsShader(m_vertexID) == GL_TRUE)
		glDeleteShader(m_vertexID);

	if (glIsShader(m_fragmentID) == GL_TRUE)
		glDeleteShader(m_fragmentID);

	if (glIsProgram(m_programID) == GL_TRUE)
		glDeleteProgram(m_programID);


	// Compilation of the new shaders
	if (!CompileShader(m_vertexID, GL_VERTEX_SHADER, m_vertexSource))
		return false;

	if (!CompileShader(m_fragmentID, GL_FRAGMENT_SHADER, m_fragmentSource))
		return false;


	// Program creation
	m_programID = glCreateProgram();


	// Shaders binding
	glAttachShader(m_programID, m_vertexID);
	glAttachShader(m_programID, m_fragmentID);


	// Locking of shaders input
	glBindAttribLocation(m_programID, 0, "in_Vertex");
	glBindAttribLocation(m_programID, 1, "in_Color");
	glBindAttribLocation(m_programID, 2, "in_TexCoord0");


	// Program linking
	glLinkProgram(m_programID);


	// Linking check
	GLint linkError(0);
	glGetProgramiv(m_programID, GL_LINK_STATUS, &linkError);


	if (linkError != GL_TRUE) {
		// Error size
		GLint errorSize(0);
		glGetProgramiv(m_programID, GL_INFO_LOG_LENGTH, &errorSize);

		// Memory allocation
		char *error = new char[errorSize + 1];

		// Error recovering
		glGetShaderInfoLog(m_programID, errorSize, &errorSize, error);
		error[errorSize] = '\0';

		// Error display
		std::cout << error << std::endl;


		// Freeing memory and returning false
		delete[] error;
		glDeleteProgram(m_programID);

		return false;
	}



	// Else, everybody went good

	else
		return true;
}


bool Shader::CompileShader(GLuint &shader, GLenum type, std::string const &sourceFile) {

	// Shader creation
	shader = glCreateShader(type);

	// Shader checking
	if (shader == 0) {
		std::cout << "Error, shader type (" << type << ") does not exist" << std::endl;
		return false;
	}

	// reading stream
	std::ifstream file(sourceFile.c_str());

	// Opening test
	if (!file) {
		std::cout << "Error, file " << sourceFile << " not found" << std::endl;
		glDeleteShader(shader);
		return false;
	}

	// String to read the source code
	std::string line;
	std::string sourceCode;

	// Reading
	while (getline(file, line))
		sourceCode += line + '\n';

	// File closing
	file.close();

	// Retrieving of the C string
	const GLchar* sourceCodeChain = sourceCode.c_str();

	// Sending the source code to the shader
	glShaderSource(shader, 1, &sourceCodeChain, 0);

	// Shader compilation
	glCompileShader(shader);

	// Compilation checking
	GLint compilationError(0);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compilationError);


	if (compilationError != GL_TRUE) {
		// Error size retrieving
		GLint errorSize(0);
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &errorSize);

		// Memory allocation
		char *error = new char[errorSize + 1];

		// Error retrieving
		glGetShaderInfoLog(shader, errorSize, &errorSize, error);
		error[errorSize] = '\0';

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
	return m_programID;
}
