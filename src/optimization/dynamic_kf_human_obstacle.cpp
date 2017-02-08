#include <itomp_nlp/optimization/dynamic_kf_human_obstacle.h>


namespace itomp
{

const std::vector<double> DynamicKFHumanObstacle::radii_ = 
{
    0.10,
    0.10,
    0.05,
    0.10,
    0.09,
    0.07,
    0.05,
    0.05,
    0.09,
    0.07,
    0.05,
    0.05,
    0.09,
    0.07,
    0.05,
    0.05,
    0.09,
    0.07,
    0.05,
    0.05,
    0.10,
};

const std::vector<std::pair<KinectDevice::JointType, KinectDevice::JointType> > DynamicKFHumanObstacle::edges_ = 
{
    {KinectDevice::JointType_SpineBase    , KinectDevice::JointType_SpineMid     },
    {KinectDevice::JointType_SpineMid     , KinectDevice::JointType_SpineShoulder},
    {KinectDevice::JointType_SpineShoulder, KinectDevice::JointType_Neck         },
    {KinectDevice::JointType_Neck         , KinectDevice::JointType_Head         },
    {KinectDevice::JointType_SpineShoulder, KinectDevice::JointType_ShoulderLeft },
    {KinectDevice::JointType_ShoulderLeft , KinectDevice::JointType_ElbowLeft    },
    {KinectDevice::JointType_ElbowLeft    , KinectDevice::JointType_WristLeft    },
    {KinectDevice::JointType_WristLeft    , KinectDevice::JointType_HandLeft     },
    {KinectDevice::JointType_SpineShoulder, KinectDevice::JointType_ShoulderRight},
    {KinectDevice::JointType_ShoulderRight, KinectDevice::JointType_ElbowRight   },
    {KinectDevice::JointType_ElbowRight   , KinectDevice::JointType_WristRight   },
    {KinectDevice::JointType_WristRight   , KinectDevice::JointType_HandRight    },
    {KinectDevice::JointType_SpineBase    , KinectDevice::JointType_HipLeft      },
    {KinectDevice::JointType_HipLeft      , KinectDevice::JointType_KneeLeft     },
    {KinectDevice::JointType_KneeLeft     , KinectDevice::JointType_AnkleLeft    },
    {KinectDevice::JointType_AnkleLeft    , KinectDevice::JointType_FootLeft     },
    {KinectDevice::JointType_SpineBase    , KinectDevice::JointType_HipRight     },
    {KinectDevice::JointType_HipRight     , KinectDevice::JointType_KneeRight    },
    {KinectDevice::JointType_KneeRight    , KinectDevice::JointType_AnkleRight   },
    {KinectDevice::JointType_AnkleRight   , KinectDevice::JointType_FootRight    },
};


DynamicKFHumanObstacle::DynamicKFHumanObstacle(int body_id)
    : body_id_(body_id)
    , camera_transform_(Eigen::Affine3d::Identity())
{
    predictor_ = new KFHumanMotionPrediction(body_id);

    for (int i=0; i<edges_.size(); i++)
        capsules_.push_back(new Capsule2());

    /*
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
            */
}

DynamicKFHumanObstacle::~DynamicKFHumanObstacle()
{
    delete predictor_;

    for (int i=0; i<capsules_.size(); i++)
        delete capsules_[i];
}

std::vector<Shape*> DynamicKFHumanObstacle::getShapes(double t)
{
    predictor_->predict();

    std::vector<Shape*> shapes_;

    if (predictor_->isBodyPredicted())
    {
        for (int i=0; i<edges_.size(); i++)
        {
            KinectDevice::JointType type0 = edges_[i].first;
            KinectDevice::JointType type1 = edges_[i].second;

            Eigen::Vector3d p0 = camera_transform_ * predictor_->getPredictedBodyJointPosition(t, type0);
            Eigen::Vector3d p1 = camera_transform_ * predictor_->getPredictedBodyJointPosition(t, type1);

            const double& r0 = radii_[type0];
            const double& r1 = radii_[type1];

            capsules_[i]->setCapsule(p0, r0, p1, r1);
        }

        shapes_.resize(capsules_.size());
        for (int i=0; i<capsules_.size(); i++)
            shapes_[i] = capsules_[i];
    }

    return shapes_;
}

}
