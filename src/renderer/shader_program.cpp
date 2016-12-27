#include <itomp_nlp/renderer/shader_program.h>


namespace itomp_renderer
{

ShaderProgram::ShaderProgram(Renderer* renderer, const std::string& vertex_filename, const std::string& fragment_filename)
    : GLBase(renderer)
{
    vertex_shader_ = loadShader(vertex_filename, GL_VERTEX_SHADER);
    fragment_shader_ = loadShader(fragment_filename, GL_FRAGMENT_SHADER);

    shader_program_ = createShaderProgram();
}

void ShaderProgram::start()
{
    gl_->glUseProgram(shader_program_);
}

void ShaderProgram::stop()
{
    gl_->glUseProgram(0);
}

void ShaderProgram::cleanUp()
{
    stop();

    gl_->glDetachShader(shader_program_, vertex_shader_);
    gl_->glDetachShader(shader_program_, fragment_shader_);
    gl_->glDeleteShader(vertex_shader_);
    gl_->glDeleteShader(fragment_shader_);
    gl_->glDeleteProgram(shader_program_);
}

void ShaderProgram::loadUniform(int location, float value)
{
    gl_->glUniform1f(location, value);
}

void ShaderProgram::loadUniform(int location, const Eigen::Vector3f& v)
{
    gl_->glUniform3fv(location, 1, v.data());
}

void ShaderProgram::loadUniform(int location, bool value)
{
    gl_->glUniform1f(location, value);
}

void ShaderProgram::loadUniform(int location, const Eigen::Matrix4f& m)
{
    gl_->glUniformMatrix4fv(location, 1, GL_FALSE, m.data());
}

void ShaderProgram::bindAttribute(int attribute, const std::string& variable_name)
{
    gl_->glBindAttribLocation(shader_program_, attribute, variable_name.c_str());
}

GLint ShaderProgram::getUniformLocation(const std::string& uniform_name)
{
    return gl_->glGetUniformLocation(shader_program_, uniform_name.c_str());
}

GLuint ShaderProgram::loadShader(const std::string& filename, GLuint shader_type)
{
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL)
        return 0;

    fseek(fp, 0, SEEK_END);
    int len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    GLchar* source = new GLchar[len+1];
    fread(source, 1, len, fp);
    fclose(fp);

    source[len] = 0;

    const GLchar* const_source = const_cast<const GLchar*>(source);


    GLuint shader = gl_->glCreateShader(shader_type);

    gl_->glShaderSource(shader, 1, &const_source, NULL);
    delete source;

    gl_->glCompileShader(shader);

    GLint compiled;
    gl_->glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

    if (!compiled)
    {
        GLsizei len;
        gl_->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);

        GLchar* log = new GLchar[len+1];
        gl_->glGetShaderInfoLog(shader, len, &len, log);
        fprintf(stderr, "Shader compilation failed: %s\n", log);
        delete log;

        return 0;
    }

    return shader;
}

GLuint ShaderProgram::createShaderProgram()
{
    GLuint program = gl_->glCreateProgram();

    gl_->glAttachShader(program, vertex_shader_);
    gl_->glAttachShader(program, fragment_shader_);
    
    gl_->glLinkProgram(program);
    
    GLint linked;
    gl_->glGetProgramiv(program, GL_LINK_STATUS, &linked);

    if (!linked)
    {
        GLsizei len;
        gl_->glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);

        GLchar* log = new GLchar[len+1];
        gl_->glGetProgramInfoLog(program, len, &len, log);
        fprintf(stderr, "Shader linking failed: %s\n", log);
        delete log;

        return 0;
    }
    
    return program;
}

}
