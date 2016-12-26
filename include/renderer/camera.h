#ifndef ITOMP_RENDERER_VISUALIZER_CAMERA_H
#define ITOMP_RENDERER_VISUALIZER_CAMERA_H


#include <Eigen/Dense>


namespace itomp_renderer
{

class Camera
{
private:

    enum ProjectionType
    {
        ORTHO = 0,
        PERSPECTIVE,
    };

public:

    Camera();

    inline void setAspect(double aspect)
    {
        aspect_ = aspect;
    }

    inline void setPerspective()
    {
        projection_type_ = PERSPECTIVE;
    }

    inline void setOrtho()
    {
        projection_type_ = ORTHO;
    }

    inline void setSensitivityTranslation(double sensitivity)
    {
        sensitivity_translation_ = sensitivity;
    }

    inline void setSensitivityZoom(double sensitivity)
    {
        sensitivity_zoom_ = sensitivity;
    }

    void lookAt(const Eigen::Vector3d& eye, const Eigen::Vector3d center);

    void translatePixel(int dx, int dy);
    void rotatePixel(int dx, int dy);
    void zoomPixel(int dx, int dy);

    Eigen::Matrix4d projectionMatrix();
    Eigen::Matrix4d viewMatrix();

    inline const Eigen::Vector3d& eyePosition() const
    {
        return eye_;
    }

private:

    Eigen::Matrix4d perspective();
    Eigen::Matrix4d ortho();

    ProjectionType projection_type_;

    double fovy_;
    double aspect_;
    double near_;
    double far_;

    Eigen::Vector3d eye_;
    Eigen::Vector3d center_;
    Eigen::Vector3d up_;

    double sensitivity_translation_;
    double sensitivity_rotation_;
    double sensitivity_zoom_;
};

}


#endif // ITOMP_RENDERER_VISUALIZER_CAMERA_H
