// scene_parser.hpp
#pragma once
#include "scene.hpp"
#include <yaml-cpp/yaml.h>

// Forward declaration
Value parseNode(const YAML::Node& node);

inline bool isVec3(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() != 3)
        return false;

    return node[0].IsScalar() &&
           node[1].IsScalar() &&
           node[2].IsScalar();
}

inline Value parseNode(const YAML::Node& node) {

    if (node.IsNull())
        return nullptr;

    if (node.IsScalar()) {
        try { return node.as<int64_t>(); } catch (...) {}
        try { return node.as<double>(); }  catch (...) {}
        try { return node.as<bool>(); }    catch (...) {}
        return node.as<std::string>();
    }

    if (node.IsSequence()) {

        if (isVec3(node)) {
            return Vec3{
                node[0].as<double>(),
                node[1].as<double>(),
                node[2].as<double>()
            };
        }

        Array arr;
        for (auto n : node)
            arr.push_back(parseNode(n));
        return arr;
    }

    if (node.IsMap()) {
        Object obj;
        for (auto it : node)
            obj[it.first.as<std::string>()] =
                parseNode(it.second);
        return obj;
    }

    throw std::runtime_error("Unsupported YAML node type");
}

inline Scene loadScene(const std::string& file) {
    YAML::Node rootNode = YAML::LoadFile(file);
    Scene scene;
    scene.root = parseNode(rootNode).as<Object>();
    return scene;
}