#include <itomp_nlp/renderer/rendering_human.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingHuman::RenderingHuman(Renderer* renderer, int num_vertices)
    : RenderingShape(renderer)
{
    positions_.resize(num_vertices);
    radii_.resize(num_vertices);
}

void RenderingHuman::setVertex(int i, const Eigen::Vector3d& position, double radius)
{
    need_update_buffer_ = true;
    positions_[i] = position;
    radii_[i] = radius;
}

void RenderingHuman::addEdge(int i, int j)
{
    need_update_buffer_ = true;
    edges_.insert(std::make_pair(i, j));
}

void RenderingHuman::deleteEdge(int i, int j)
{
    need_update_buffer_ = true;
    edges_.erase(std::make_pair(i, j));
}

void RenderingHuman::deleteAllEdges()
{
    need_update_buffer_ = true;
    edges_.clear();
}

void RenderingHuman::updateBuffers()
{
    for (int i=capsules_.size(); i<edges_.size(); i++)
        capsules_.push_back(new RenderingCapsule(renderer_));

    for (int i=edges_.size(); i<capsules_.size(); i++)
        delete capsules_[i];
    capsules_.resize(edges_.size());

    int idx = 0;
    for (std::set<std::pair<int, int> >::iterator it = edges_.begin(); it != edges_.end(); it++)
    {
        const int i = it->first;
        const int j = it->second;
        capsules_[idx++]->setCapsule( positions_[i], radii_[i], positions_[j], radii_[j] );
    }

    need_update_buffer_ = false;
}

void RenderingHuman::draw(ShaderProgram* shader)
{
    if (need_update_buffer_)
        updateBuffers();

    shader->loadMaterial(material_);

    for (int i=0; i<capsules_.size(); i++)
        capsules_[i]->draw(shader);
}

}
