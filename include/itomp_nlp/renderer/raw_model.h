#ifndef ITOMP_RENDERER_RAW_MODEL_H
#define ITOMP_RENDERER_RAW_MODEL_H


namespace itomp_renderer
{

class RawModel
{
public:

    RawModel(int vao, int num_vertices);

    inline int getVAO()
    {
        return vao_;
    }

    inline int getNumVertices()
    {
        return num_vertices_;
    }

private:

    int vao_;
    int num_vertices_;
};

}


#endif // ITOMP_RENDERER_RAW_MODEL_H