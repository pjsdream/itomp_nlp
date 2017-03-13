#include <itomp_nlp/renderer/rendering_kinect_human.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

RenderingKinectHuman::RenderingKinectHuman(Renderer* renderer)
    : RenderingShape(renderer)
{
    kinect_ = new Kinect();

    for (int i=0; i<kinect_->bodyCount(); i++)
        humans_.push_back( new RenderingHuman(renderer, KinectDevice::JointType_Count) );
}

void RenderingKinectHuman::updateBuffers()
{
    for (int i=0; i<kinect_->bodyCount(); i++)
    {
        if (kinect_->isBodyTracked(i))
        {
            humans_[i]->setVertex( 0, kinect_->getBodyJointPosition(i, KinectDevice::JointType_SpineBase),     0.10);
            humans_[i]->setVertex( 1, kinect_->getBodyJointPosition(i, KinectDevice::JointType_SpineMid),      0.10);
            humans_[i]->setVertex( 2, kinect_->getBodyJointPosition(i, KinectDevice::JointType_Neck),          0.05);
            humans_[i]->setVertex( 3, kinect_->getBodyJointPosition(i, KinectDevice::JointType_Head),          0.10);
            humans_[i]->setVertex( 4, kinect_->getBodyJointPosition(i, KinectDevice::JointType_ShoulderLeft),  0.09);
            humans_[i]->setVertex( 5, kinect_->getBodyJointPosition(i, KinectDevice::JointType_ElbowLeft),     0.07);
            humans_[i]->setVertex( 6, kinect_->getBodyJointPosition(i, KinectDevice::JointType_WristLeft),     0.05);
            humans_[i]->setVertex( 7, kinect_->getBodyJointPosition(i, KinectDevice::JointType_HandLeft),      0.05);
            humans_[i]->setVertex( 8, kinect_->getBodyJointPosition(i, KinectDevice::JointType_ShoulderRight), 0.09);
            humans_[i]->setVertex( 9, kinect_->getBodyJointPosition(i, KinectDevice::JointType_ElbowRight),    0.07);
            humans_[i]->setVertex(10, kinect_->getBodyJointPosition(i, KinectDevice::JointType_WristRight),    0.05);
            humans_[i]->setVertex(11, kinect_->getBodyJointPosition(i, KinectDevice::JointType_HandRight),     0.05);
            humans_[i]->setVertex(12, kinect_->getBodyJointPosition(i, KinectDevice::JointType_HipLeft),       0.09);
            humans_[i]->setVertex(13, kinect_->getBodyJointPosition(i, KinectDevice::JointType_KneeLeft),      0.07);
            humans_[i]->setVertex(14, kinect_->getBodyJointPosition(i, KinectDevice::JointType_AnkleLeft),     0.05);
            humans_[i]->setVertex(15, kinect_->getBodyJointPosition(i, KinectDevice::JointType_FootLeft),      0.05);
            humans_[i]->setVertex(16, kinect_->getBodyJointPosition(i, KinectDevice::JointType_HipRight),      0.09);
            humans_[i]->setVertex(17, kinect_->getBodyJointPosition(i, KinectDevice::JointType_KneeRight),     0.07);
            humans_[i]->setVertex(18, kinect_->getBodyJointPosition(i, KinectDevice::JointType_AnkleRight),    0.05);
            humans_[i]->setVertex(19, kinect_->getBodyJointPosition(i, KinectDevice::JointType_FootRight),     0.05);
            humans_[i]->setVertex(20, kinect_->getBodyJointPosition(i, KinectDevice::JointType_SpineShoulder), 0.10);

            humans_[i]->addEdge(0, 1);
            humans_[i]->addEdge(1, 20);
            humans_[i]->addEdge(20, 2);
            humans_[i]->addEdge(2, 3);
            
            humans_[i]->addEdge(20, 4);
            humans_[i]->addEdge(4, 5);
            humans_[i]->addEdge(5, 6);
            humans_[i]->addEdge(6, 7);
            
            humans_[i]->addEdge(20, 8);
            humans_[i]->addEdge(8, 9);
            humans_[i]->addEdge(9, 10);
            humans_[i]->addEdge(10, 11);

            humans_[i]->addEdge(0, 12);
            humans_[i]->addEdge(12, 13);
            humans_[i]->addEdge(13, 14);
            humans_[i]->addEdge(14, 15);
            
            humans_[i]->addEdge(0, 16);
            humans_[i]->addEdge(16, 17);
            humans_[i]->addEdge(17, 18);
            humans_[i]->addEdge(18, 19);
        }

        else
        {
            humans_[i]->deleteAllEdges();
        }
    }
}

void RenderingKinectHuman::draw(LightShader* shader)
{
    kinect_->update();

    updateBuffers();

    shader->loadMaterial(material_);

    for (int i=0; i<humans_.size(); i++)
        humans_[i]->draw(shader);
}

}
