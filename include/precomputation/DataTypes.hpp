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

using namespace Eigen;

/**
 * @brief Struct containing position and orientation of joint i in the chain
 *
 */
struct PosOrientation {
    int index;   //!< position in the chain
    Vector3d p;  //!< position in space GLOBAL
    Matrix3d z;  //!< orientation in space GLOBAL
    
    PosOrientation() {
        index = 0;
        this->p = Vector3d::Zero();
        this->z = Matrix3d::Identity();
    }
    void setPosition(Vector3d Point);
    void setOrientation(Matrix3d Orientation);
};

inline void PosOrientation::setPosition(Vector3d Point) { this->p = Point; }

inline void PosOrientation::setOrientation(Matrix3d Orientation) {
    this->z = Orientation;
}

/**
 * @brief Struct containing joint position and orientations, pointers to
 * adjecent joints and transform matrix
 *
 */
struct Joint {
    int index;         //!< Position in joint chain
    Joint *nextJoint;  //!< Pointer to next joint (i+1)
    Joint *prevJoint;  //!< Pointer to previous joint (i-1)

    PosOrientation *p;  //!< Position and orientation of joint

    Vector3d q;  //!< Global angle of joint

    Matrix3d Rotation;  //!< Rotation part of Transform matrix
    Vector3d pLocal;    //!< Positional part of Transform matrix
    Matrix4d Transform; //!< Transform matrix

    Vector3d GlobMag;  //!< Magnetisation in global frame. i.e. observed frame
    Vector3d LocMag;   //!< Magnetisation in local frame. i.e. its own frame 

    void assignPosOri(PosOrientation &PosOri_);
};

inline void Joint::assignPosOri(PosOrientation &PosOri_) {
    this->p = &PosOri_;
}

/**
 * @brief Struct containing position of each link and mechanical parameters
 *
 */
struct Link {
    int index;  // position in link chain

    PosOrientation *base;  //!< Joints i and i+1 that make up start and end of Link i
    PosOrientation *head;  //!< Joints i and i+1 that make up start and end of Link i

    double dL;  //!< Link Length
    double d;   //!< Link Diameter
    int E;      //!< Young's modulus
    double v;   //!< Poissant's ratio

    void assignPosOri(PosOrientation &PosOri1_, PosOrientation &PosOri2_);
};

inline void Link::assignPosOri(PosOrientation &PosOri1_,
                               PosOrientation &PosOri2_) {
    this->base = &PosOri1_;
    this->head = &PosOri2_;
}

#endif
