/**
 * @file DataTypes.h
 * @author Vittorio Francescon (vittorio.francescon@gmail.com)
 * @brief Contains definitions for structs containing key elements of the rigid
 * link model.
 * @version 0.1
 * @date 28/02/2024
 *
 */
#ifndef TENTACLEDATATYPES
#define TENTACLEDATATYPES

#include <math.h>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

using namespace Eigen;

/**
 * @brief Struct containing joint position and orientations, pointers to
 * adjecent joints and transform matrix
 *
 */
struct Joint {
    int index;                         //!< Position in joint chain
    int nextIndex, prevIndex;          //!< Index of next and previous joint
    Vector3d q;                        //!< Joint angle in its own frame
    Matrix3d Orientation = Matrix3d::Zero(); //!< Orientation matrix
    Matrix3d Rotation = Matrix3d::Identity();  //!< Rotation part of local frame
    Vector3d pLocal = Vector3d::Zero();     //!< Positional part of local frame
    Matrix4d Transform = Matrix4d::Zero();  //!< Transform matrix

    Vector3d GlobMag;  //!< Magnetisation in global frame. i.e. observed frame
    Vector3d LocMag;   //!< Magnetisation in local frame. i.e. its own frame
};

/**
 * @brief Struct containing position of each link and mechanical parameters
 *
 */
struct Link {
    int index;  // position in link chain
    int nextIndex, prevIndex;  // index of next and previous link
    double dL;  //!< Link Length
    double d;   //!< Link Diameter
    int E;      //!< Young's modulus
    double v;   //!< Poissant's ratio
};

#endif
