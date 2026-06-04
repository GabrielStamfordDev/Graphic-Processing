#include <iostream>
#include <string>
#include "utils/scene/sceneParser.cpp"
#include "core/sceneBuilder.h"
#include "core/rayTracing.h"
using namespace std;

int main(int argc, char** argv){
    ios::sync_with_stdio(false);cin.tie(0);
    if(argc < 2){
        cerr<<"Uso: "<<argv[0]<<" cena.josn\n";
        return 1;
    }

    SceneData scene = SceneJsonLoader::loadFile(argv[1]);
    cerr<<"Resolucao: "<<scene.camera.image_width<<" x "<<scene.camera.image_height<<'\n';

    Camera cam(scene.camera);
    bool transformar = true;
    if(argc > 2 && string(argv[2]) == "--no-transform") transformar = false;

    CenaProcessada dados = prepararObjetos(scene.objects, transformar);

    RayTracer(dados, cam, scene);
    return 0;
}