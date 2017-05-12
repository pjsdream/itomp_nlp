#define _USE_MATH_DEFINES
 
#include <itomp_nlp/interface/main_window.h>

#include <itomp_nlp/shape/capsule2.h>

//#include <itomp_nlp/renderer/rendering_kinect_human.h>

#include <QTimer>


namespace itomp
{

MainWindow::MainWindow()
    : QMainWindow()
{
    setWindowTitle("Viewer");

    resize(800, 600);

    // central visualizer widget setup
    renderer_ = new Renderer(this);
    setCentralWidget(renderer_);
    show();

    // motion planner interface setup
    itomp_interface_ = new ItompInterface();
    itomp_interface_->show();

    // windows position
    itomp_interface_->move(100, 100);
    move( itomp_interface_->pos().x() + itomp_interface_->width(), itomp_interface_->pos().y() );

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(33);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));

    // rendering robots
    const int num_interpolated_variables = itomp_interface_->getNumInterpolatedVariables();
    RobotModel* robot_model = itomp_interface_->getRobotModel();

    Material* white = new Material();
    white->setDiffuse(Eigen::Vector3f(1, 1, 1));
    white->setSpecular(Eigen::Vector3f(1, 1, 1));
    white->setShininess(2);

    /*
    for (int i=0; i<num_interpolated_variables; i++)
    {
        RenderingRobot* rendering_robot = new RenderingRobot(renderer_, robot_model);
        rendering_robots_.push_back(rendering_robot);

        RenderingBox* rendering_box = new RenderingBox(renderer_);
        rendering_box->setSize(Eigen::Vector3d(0.1, 0.1, 0.1));
        rendering_box->setMaterial(white);
        rendering_objects_.push_back(rendering_box);
    }
    */

    // rendering point cloud
    /*
    Eigen::Affine3d camera_transform;
    camera_transform.setIdentity();
    camera_transform.translate(Eigen::Vector3d(1.5, 1.6, 0.9));
    camera_transform.rotate(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
    camera_transform.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));
    rendering_point_cloud_ = new RenderingKinectPoints(renderer_);
    rendering_point_cloud_->setTransform(camera_transform);
    */
    
    // rendering kinect
    /*
    Material* black = new Material();
    black->setDiffuseColor(Eigen::Vector4f(0, 0, 0, 1));

    RenderingBox* rendering_camera = new RenderingBox(renderer_);
    rendering_camera->setMaterial(black);
    rendering_camera->setTransform(camera_transform);
    rendering_camera->setSize(Eigen::Vector3d(0.2, 0.05, 0.05));
    */

    forward_kinematics_ = itomp_interface_->getOptimizerRobot();

    grey_ = new Material();
    grey_->setAmbient(Eigen::Vector3f(0.3, 0.3, 0.3));
    grey_->setDiffuse(Eigen::Vector3f(0.3, 0.3, 0.3));
    grey_->setAlpha(0.25f);
    
    brown_ = new Material();
    brown_->setAmbient(Eigen::Vector3f(139./255, 69./255, 19./255));
    brown_->setDiffuse(Eigen::Vector3f(139./255, 69./255, 19./255));
    brown_->setSpecular(Eigen::Vector3f::Zero());
    
    // rendering table
    const double margin = 0.01;
    rendering_table_ = new RenderingBox(renderer_);
    rendering_table_->setMaterial(brown_);
    rendering_table_->setSize(Eigen::Vector3d(1 - margin, 2 - margin, 0.7 - margin)); // shrinked by margin for rendering bounding box
    rendering_table_->setTransform(Eigen::Affine3d(Eigen::Translation3d(0.8, 0, 0.35)));
    
    // rendering laptop
    /*
    Eigen::Affine3d laptop_transform;
    laptop_transform.setIdentity();
    laptop_transform.translate(Eigen::Vector3d(0.8, 0.6, 0.71));
    laptop_transform.rotate(Eigen::AngleAxisd(2, Eigen::Vector3d(0, 0, 1)));

    rendering_laptop_ = new RenderingBox(renderer_);
    rendering_laptop_->setMaterial(white);
    rendering_laptop_->setTransform(laptop_transform);
    rendering_laptop_->setSize(Eigen::Vector3d(0.4, 0.3, 0.02));
    */
    
    // rendering objects
    Material* red = new Material();
    red->setAmbient(Eigen::Vector3f(1, 0, 0));
    red->setDiffuse(Eigen::Vector3f(1, 0, 0));
    red->setAlpha(1.f);

    Material* blue = new Material();
    blue->setAmbient(Eigen::Vector3f(0, 0, 1));
    blue->setDiffuse(Eigen::Vector3f(0, 0, 1));
    blue->setAlpha(1.f);
    
    // rendering robot trajectory
    rendering_robot_trajectory_ = new RenderingRobotTrajectory(renderer_, robot_model, *itomp_interface_->getRobotState());
    
    // rendering attached object
    RenderingBox* rendering_box = new RenderingBox(renderer_);
    rendering_box->setSize(Eigen::Vector3d(0.1, 0.1, 0.1));
    rendering_box->setMaterial(red);
    //rendering_robot_trajectory_->attachObject(rendering_box);
    rendering_robot_trajectory_->setForwardKinematics(forward_kinematics_);

    RenderingCapsule* capsule1 = new RenderingCapsule(renderer_);
    capsule1->setMaterial(red);
    capsule1->setCapsule(Eigen::Vector3d(0.75, 0.75, 0.7), 0.03, Eigen::Vector3d(0.75, 0.75, 0.8), 0.03);

    RenderingCapsule* capsule2 = new RenderingCapsule(renderer_);
    capsule2->setMaterial(blue);
    capsule2->setCapsule(Eigen::Vector3d(0.75, -0.75, 0.7), 0.03, Eigen::Vector3d(0.75, -0.75, 0.8), 0.03);

    // rendering H shape obstacle
    /*
    Eigen::Affine3d obstacle_center;
    obstacle_center.setIdentity();
    obstacle_center.translate(Eigen::Vector3d(0.9, 0, 0.8));
    RenderingBox* shape1 = new RenderingBox(renderer_);
    shape1->setSize(Eigen::Vector3d(0.5, 1, 0.1));
    shape1->setTransform(obstacle_center);
    shape1->setMaterial(brown_);
    
    obstacle_center.setIdentity();
    obstacle_center.translate(Eigen::Vector3d(0.9, 0, 1.4));
    RenderingBox* shape2 = new RenderingBox(renderer_);
    shape2->setSize(Eigen::Vector3d(0.5, 1, 0.1));
    shape2->setTransform(obstacle_center);
    shape2->setMaterial(brown_);
    
    obstacle_center.setIdentity();
    obstacle_center.translate(Eigen::Vector3d(0.9, 0, 1.1));
    RenderingBox* shape3 = new RenderingBox(renderer_);
    shape3->setSize(Eigen::Vector3d(0.5, 0.1, 0.6));
    shape3->setTransform(obstacle_center);
    shape3->setMaterial(brown_);
    */

    // rendering planes
    std::vector<unsigned char> checkerboard_image = 
    {
        0xFF, 0xFF, 0xFF, 0xFF,  0xCF, 0xCF, 0xCF, 0xFF,
        0xCF, 0xCF, 0xCF, 0xFF,  0xFF, 0xFF, 0xFF, 0xFF,
    };

    Texture* checkerboard_texture = new Texture(renderer_);
    checkerboard_texture->setImage(2, 2, checkerboard_image);

    Material* checkerboard = new Material();
    checkerboard->setAmbient(Eigen::Vector3f(1, 1, 1));
    checkerboard->setDiffuseTexture(checkerboard_texture);
    checkerboard->setSpecular(Eigen::Vector3f::Zero());

    RenderingPlane* plane = new RenderingPlane(renderer_);
    plane->setMaterial(checkerboard);

    RenderingPlane* plane2 = new RenderingPlane(renderer_);
    Eigen::Affine3d yzx;
    yzx.setIdentity();
    yzx.translate(Eigen::Vector3d(-2, 0, 0));
    yzx.rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(0, 1, 0)));
    plane2->setTransform(yzx);
    plane2->setMaterial(checkerboard);

    timer->start();
}

void MainWindow::updateNextFrame()
{
    // KinectDevice::getInstance()->update();
    
    // update current robot trajectory to renderer
    const Trajectory current_robot_trajectory = itomp_interface_->getCurrentTrajectory();
    const double current_time = itomp_interface_->getCurrentTrajectoryTime();

    rendering_robot_trajectory_->setRobotTrajectory(current_robot_trajectory);
    rendering_robot_trajectory_->setTime(current_time);
    
    // update robot trajectory to renderer
    /*
    Eigen::MatrixXd trajectory = itomp_interface_->getBestTrajectory();
    
    int box_idx = 0;
    for (int i=0; i<trajectory.cols() / 2; i++)
    //for (int i=0; i<1; i++)
    {
        Eigen::VectorXd optimizer_robot_trajectory = trajectory.col(i*2);
        
        RobotState robot_state(*itomp_interface_->getRobotState());
        for (int j=0; j<itomp_interface_->getActiveJointNames().size(); j++)
            robot_state.setPosition(itomp_interface_->getActiveJointNames()[j], optimizer_robot_trajectory(j));
        
        // update robot state to renderer
        rendering_robots_[i]->setRobotState(robot_state);

        // update collision boxes
        forward_kinematics_->setPositions(trajectory.col(i*2));
        forward_kinematics_->setVelocities(trajectory.col(i*2+1));
        forward_kinematics_->forwardKinematics();

        const int num_links = forward_kinematics_->getNumLinks();
        for (int j=0; j<num_links; j++)
        {
            const std::vector<Shape*>& shapes = forward_kinematics_->getCollisionShapes(j);

            for (int k=0; k<shapes.size(); k++)
            {
                Shape* shape = shapes[k];

                OBB* obb = dynamic_cast<OBB*>(shape);
                if (obb != 0)
                {
                    if (box_idx >= rendering_boxes_.size())
                        rendering_boxes_.push_back(new RenderingBox(renderer_));

                    rendering_boxes_[box_idx]->setMaterial(grey_);
                    rendering_boxes_[box_idx]->setSize(obb->getSize());
                    rendering_boxes_[box_idx]->setTransform(obb->getTransform());
                    box_idx++;
                }
            }
        }

        // update endeffector transform
        forward_kinematics_->setPositions(trajectory.col(i*2));
        forward_kinematics_->setVelocities(trajectory.col(i*2+1));
        forward_kinematics_->forwardKinematics();

        Eigen::Affine3d transform = forward_kinematics_->getLinkWorldTransform(7);
        transform.translate(Eigen::Vector3d(0.2, 0, 0));
        rendering_objects_[i]->setTransform(transform);
    }


    // remove unused rendering boxes
    while (box_idx < rendering_boxes_.size())
    {
        delete *rendering_boxes_.rbegin();
        rendering_boxes_.pop_back();
    }
        */

    // update scene rendering
    Scene* scene = itomp_interface_->getScene();
    const std::vector<StaticObstacle*> static_obstacles = scene->getStaticObstacles();
    
    /*
    int static_obstacle_idx = 0;
    for (int i=0; i<static_obstacles.size(); i++)
    {
        const StaticObstacle* static_obstacle = static_obstacles[i];
        const std::vector<Shape*> shapes = static_obstacle->getShapes();

        for (int j=0; j<shapes.size(); j++)
        {
            const Shape* shape = shapes[j];

            const OBB* obb = dynamic_cast<const OBB*>(shape);
            if (obb != 0)
            {
                if (static_obstacle_idx >= rendering_static_obstacles_.size())
                    rendering_static_obstacles_.push_back(new RenderingBox(renderer_));

                RenderingBox* rendering_box = dynamic_cast<RenderingBox*>(rendering_static_obstacles_[static_obstacle_idx]);
                rendering_box->setMaterial(grey_);
                rendering_box->setSize(obb->getSize());
                rendering_box->setTransform(obb->getTransform());
                static_obstacle_idx++;
            }
        }
    }

    // remove unused rendering static obstacles
    while (static_obstacle_idx < rendering_static_obstacles_.size())
    {
        delete *rendering_static_obstacles_.rbegin();
        rendering_static_obstacles_.pop_back();
    }
    */

    // dynamic obstacles
    const std::vector<DynamicObstacle*> dynamic_obstacles = scene->getDynamicObstacles();

    int dynamic_obstacle_idx = 0;
    for (int i=0; i<dynamic_obstacles.size(); i++)
    {
        const double t = 0.5;

        DynamicObstacle* dynamic_obstacle = dynamic_obstacles[i];
        const std::vector<Shape*> shapes = dynamic_obstacle->getShapes(t);

        for (int j=0; j<shapes.size(); j++)
        {
            const Shape* shape = shapes[j];

            const Capsule2* capsule = dynamic_cast<const Capsule2*>(shape);
            if (capsule != 0)
            {
                if (dynamic_obstacle_idx >= rendering_dynamic_obstacles_.size())
                    rendering_dynamic_obstacles_.push_back(new RenderingCapsule(renderer_));

                RenderingCapsule* rendering_capsule = dynamic_cast<RenderingCapsule*>(rendering_dynamic_obstacles_[dynamic_obstacle_idx]);
                rendering_capsule->setMaterial(grey_);
                rendering_capsule->setCapsule(capsule->getP(), capsule->getRp(), capsule->getQ(), capsule->getRq());
                dynamic_obstacle_idx++;
            }
        }
    }

    // remove unused rendering dynamic obstacles
    while (dynamic_obstacle_idx < rendering_dynamic_obstacles_.size())
    {
        delete *rendering_dynamic_obstacles_.rbegin();
        rendering_dynamic_obstacles_.pop_back();
    }

    renderer_->update();
}

}
