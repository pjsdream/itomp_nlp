#ifndef ITOMP_RENDERER_HUMAN_H
#define ITOMP_RENDERER_HUMAN_H


#include <itomp_nlp/renderer/rendering_shape.h>
#include <itomp_nlp/renderer/rendering_capsule.h>

#include <set>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class RenderingHuman : public RenderingShape
{
private:

    template <typename T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

    template <typename T1, typename T2>
    using AlignedMap = std::map<T1, T2, std::less<T1>, 
         Eigen::aligned_allocator<std::pair<const T1, T2> > >;

public:

    RenderingHuman(Renderer* renderer, int num_vertices);

    void setVertex(int i, const Eigen::Vector3d& position, double radius);
    void addEdge(int i, int j);
    void deleteEdge(int i, int j);
    void deleteAllEdges();

    virtual void draw(ShaderProgram* shader);
    
private:
    
    bool need_update_buffer_;
    AlignedVector<Eigen::Vector3d> positions_;
    std::vector<double> radii_;
    std::set<std::pair<int, int> > edges_;

    void updateBuffers();

    std::vector<RenderingCapsule*> capsules_;
};

}


#endif // ITOMP_RENDERER_HUMAN_H