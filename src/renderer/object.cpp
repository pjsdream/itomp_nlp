#include <itomp_nlp/renderer/object.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <itomp_nlp/utils/timing.h>


namespace itomp_renderer
{

Object::Object(Renderer* renderer)
    : GLBase(renderer)
{
}

void Object::draw()
{
    gl_->glActiveTexture(GL_TEXTURE0);
    gl_->glBindTexture(GL_TEXTURE_2D, material_->getDiffuseTexture()->getId());

    gl_->glBindVertexArray(vao_);

    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
    gl_->glEnableVertexAttribArray(2);

    gl_->glDrawElements(GL_TRIANGLES, num_vertices_, GL_UNSIGNED_INT, 0);

    gl_->glDisableVertexAttribArray(0);
    gl_->glDisableVertexAttribArray(1);
    gl_->glDisableVertexAttribArray(2);

    gl_->glBindVertexArray(0);
}

}
