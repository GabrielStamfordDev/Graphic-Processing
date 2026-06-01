#pragma once
#include "utils/Scene/sceneSchema.hpp"
#include "src/Ponto.h"
#include "src/Vetor.h"

class Camera {
public:
    int hres, vres;
    Ponto C;
    Camera(const CameraData& cam):
        C(cam.lookfrom), M(cam.lookat), Vup(cam.upVector),
        d(cam.screen_distance), hres(cam.image_width), vres(cam.image_height)
    { build(); }

    Vetor getRayDirection(int i, int j) const {
        double deslocamento_u = (i + 0.5) * pixel_size;
        double deslocamento_v = (j + 0.5) * pixel_size;

        Ponto pixel = upper_left + (U * deslocamento_u) - (V * deslocamento_v);
        return (pixel - C).normalize();
    }

private:
    Ponto M;
    Vetor Vup;
    double d;
    Vetor W, U, V;
    double pixel_size;
    Ponto screen_center;
    Ponto upper_left;

    void build(){
        W = (C - M).normalize();
        U = (Vup.cross(W)).normalize();
        V = (W.cross(U));
        pixel_size = 1.0/(double)hres;

        double screen_width = 1.0;
        double screen_height = vres * pixel_size;
        screen_center = C - (W * d);
        upper_left = screen_center - (U * (screen_width / 2.0)) + (V * (screen_height / 2.0));
    }
};