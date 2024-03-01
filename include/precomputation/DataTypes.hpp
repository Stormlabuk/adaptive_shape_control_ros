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
#include <vector>
#include <memory>

using namespace Eigen;

/**
 * @brief Struct containing joint position and orientations, pointers to
 * adjecent joints and transform matrix
 *
 */
struct Joint {
    int index;         //!< Position in joint chain
    std::shared_ptr<Joint> nextJoint;  //!< Pointer to next joint (i+1)
    std::shared_ptr<Joint> prevJoint;  //!< Pointer to previous joint (i-1)
    Vector3d q;  //!< Joint angle in its own frame

    Matrix3d Rotation = Matrix3d::Identity();  //!< Rotation part of local frame
    Vector3d pLocal = Vector3d::Zero();    //!< Positional part of local frame
    Matrix4d Transform = Matrix4d::Zero(); //!< Transform matrix

    Vector3d GlobMag;  //!< Magnetisation in global frame. i.e. observed frame
    Vector3d LocMag;   //!< Magnetisation in local frame. i.e. its own frame 

};

/**
 * @brief Struct containing position of each link and mechanical parameters
 *
 */
struct Link {
    int index;  // position in link chain

    std::shared_ptr<Joint> base;  //!< Joints i and i+1 that make up start and end of Link i
    std::shared_ptr<Joint> head;  //!< Joints i and i+1 that make up start and end of Link i

    double dL;  //!< Link Length
    double d;   //!< Link Diameter
    int E;      //!< Young's modulus
    double v;   //!< Poissant's ratio

};

#endif
