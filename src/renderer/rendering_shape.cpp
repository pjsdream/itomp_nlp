#include <itomp_nlp/renderer/rendering_shape.h>

#include <itomp_nlp/renderer/rendering_capsule.h>
#include <itomp_nlp/renderer/rendering_mesh.h>

#include <itomp_nlp/shape/capsule.h>
#include <itomp_nlp/shape/mesh.h>


namespace itomp
{

RenderingShape::RenderingShape(Renderer* renderer)
    : GLBase(renderer)
{
}

/*
void RenderingShape::draw(GLuint primitive_type)
{
    gl_->glBindVertexArray(vao_);

    if (material_ && material_->hasDiffuseTexture())
    {
        gl_->glActiveTexture(GL_TEXTURE0);
        gl_->glBindTexture(GL_TEXTURE_2D, material_->getDiffuseTexture()->getId());
        gl_->glEnableVertexAttribArray(2);
    }
    else
    {
        gl_->glActiveTexture(GL_TEXTURE0);
        gl_->glBindTexture(GL_TEXTURE_2D, 0);
        gl_->glDisableVertexAttribArray(2);
    }

    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);

    gl_->glDrawElements(primitive_type, num_vertices_, GL_UNSIGNED_INT, 0);

    gl_->glDisableVertexAttribArray(0);
    gl_->glDisableVertexAttribArray(1);
    gl_->glDisableVertexAttribArray(2);

    gl_->glBindVertexArray(0);
}
*/

}
