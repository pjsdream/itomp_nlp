#ifndef ITOMP_RENDERER_OBJECT_H
#define ITOMP_RENDERER_OBJECT_H


#include <itomp_nlp/renderer/gl_base.h>

#include <itomp_nlp/renderer/material.h>


namespace itomp_renderer
{

class Object : public GLBase
{
public:

    Object(Renderer* renderer);

    void draw();

    inline GLuint getVAO() const
    {
        return vao_;
    }

    inline void setVAO(GLuint vao)
    {
        vao_ = vao;
    }

    inline int getNumVertices() const
    {
        return num_vertices_;
    }

    inline void setNumVertices(int num_vertices)
    {
        num_vertices_ = num_vertices;
    }

    inline const Material* getMaterial() const
    {
        return material_;
    }

    inline void setMaterial(Material* material)
    {
        material_ = material;
    }

private:

    GLuint vao_;
    int num_vertices_;

    Material* material_;
};

}


#endif // ITOMP_RENDERER_OBJECT_H