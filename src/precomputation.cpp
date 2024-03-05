#include "precomputation/precomputation.hpp"

Precomputation::Precomputation() {
    ros::NodeHandle nh;
    nh.param<int>("precomputation/E", E_, 100e3);
    nh.param<float>("precomputation/len", len_, 10e-3);
    nh.param<float>("precomputation/d", d_, 2e-3);
    nh.param<float>("precomputation/v", v_, 0.43);
    ROS_INFO("Init values: E_: %d_, len_: %f, d_: %f, v_: %f", E_, len_, d_,
             v_);

    nh.param<std::vector<float>>("precomputation/magx", magX_, {});
    nh.param<std::vector<float>>("precomputation/magy", magY_, {});
    nh.param<std::vector<float>>("precomputation/magz", magZ_, {});

    preCalcService_ =
        nh.advertiseService("precomputation/calc_initial_field",
                            &Precomputation::calculateField, this);

    // desiredAnglesSub_ = nh.subscribe(
    //     "/des_angles", 10, &Precomputation::desiredAnglesCallback, this);
}

bool Precomputation::calculateField(
    shapeforming_msgs::CalcInitialField::Request &req,
    shapeforming_msgs::CalcInitialField::Response &res) {
    
    shapeforming_msgs::rl_angles tentacle;
    tentacle= req.tentacle;
    
    desiredAngles_ = tentacle.angles;
    obvJointNo_ = tentacle.count;
    ROS_INFO("Received desired angles. Num: %d", obvJointNo_);
    ROS_INFO("Initialising structs");
    try {
        if (obvJointNo_ < 2) {
            throw std::runtime_error(
                "Number of joints should be greater than 1");
        } else {
            links_.clear();
            joints_.clear();
            populateStructs();
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }

    ROS_INFO("Calculating initial field");
    MatrixXd K, J, S;
    // 1. Evaluate stiffness matrix K
    K = evaluateStiffnessMatrix();
    // 2. Evaluate direct kinematics
    evaluateDirectKinematics();
    // 3. Evaluate geometric jacboian
    J = evaluateGeometricJacobian();
    // 4. evaluate magnetisation to field map
    S = MagnetisationToFieldMap();
    
    // 5. Build stacked deformation Matrix
    MatrixXd Q = MatrixXd::Zero(3 * obvJointNo_, 1);
    for (int i = 0; i < obvJointNo_; i++) {
        Q(seq(0 + i * 3, 2 + i * 3), 0) = joints_[i].q;
    }

    // 6. Ax = b
    MatrixXd Jt = J.transpose();
    MatrixXd RHS = Jt * S;
    MatrixXd LHS = K * Q;
    // 7. Solve for x with lsqminnorm
    Vector3d solution = RHS.completeOrthogonalDecomposition().solve(LHS);

    res.success = true;
    res.field.header.frame_id = "world";
    res.field.header.stamp = ros::Time::now();
    res.field.bx = solution(0) * 1000;
    res.field.by = solution(1) * 1000;
    res.field.bz = solution(2) * 1000;

    releasePointers();

    return res.success;
}

void Precomputation::populateStructs() {
    // Populate the joint and link structs
    // 1. Populate the posOri structs

    // 2. Populate the joint structs
    joints_.resize(obvJointNo_);
    links_.resize(obvJointNo_);
    for (int i = 0; i < obvJointNo_; i++) {
        joints_[i] = Joint();
        joints_[i].index = i;
        joints_[i].LocMag = Vector3d(magX_[i], magY_[i], magZ_[i]);
        joints_[i].q = Vector3d(0, desiredAngles_[i] * M_PI / 180, 0);

        links_[i] = Link();
        links_[i].index = i;
        links_[i].d = d_;
        links_[i].dL = len_;
        links_[i].E = E_;
        links_[i].v = v_;
    }

    for (int i = 0; i < obvJointNo_; i++) {
        if (i == 0) {
            joints_[i].prevJoint = nullptr;
            joints_[i].nextJoint = std::make_shared<Joint>(joints_[i + 1]);
            links_[i].base = std::make_shared<Joint>(joints_[i]);
            links_[i].head = std::make_shared<Joint>(joints_[i + 1]);
        } else if (i == obvJointNo_ - 1) {
            joints_[i].prevJoint = std::make_shared<Joint>(joints_[i - 1]);
            joints_[i].nextJoint = nullptr;
            links_[i].base = std::make_shared<Joint>(joints_[i - 1]);
            links_[i].head = std::make_shared<Joint>(joints_[i]);
        } else {
            joints_[i].prevJoint = std::make_shared<Joint>(joints_[i - 1]);
            joints_[i].nextJoint = std::make_shared<Joint>(joints_[i + 1]);
            links_[i].base = std::make_shared<Joint>(joints_[i - 1]);
            links_[i].head = std::make_shared<Joint>(joints_[i + 1]);
        }
    }

    // for (auto i : joints_) {
    //     if (i.nextJoint != nullptr) {
    //         ROS_INFO("Joint %d has next joint, it is joint %d", i.index,
    //                  i.nextJoint.index);
    //     } else {
    //         ROS_INFO("Joint %d has no next joint", i.index);
    //     }
    //     if (i.prevJoint != nullptr) {
    //         ROS_INFO("Joint %d has prev joint. It is joint %d", i.index,
    //                  i.prevJoint.index);
    //     } else {
    //         ROS_INFO("Joint %d has no prev joint", i.index);
    //     }
    // }

    ROS_INFO("Populated RL structs");
    return;
}

void Precomputation::releasePointers() {
    for (int i = 0; i < obvJointNo_; i++) {
        joints_[i].nextJoint.reset();
        joints_[i].prevJoint.reset();
    }
    joints_.clear();
    links_.clear();
    ROS_INFO("Released all pointers");
}

// void Precomputation::desiredAnglesCallback(
//     const shapeforming_msgs::rl_angles::ConstPtr &msg) {
//     desiredAngles_ = msg.angles;
//     obvJointNo_ = msg.count;
//     ROS_INFO("Received desired angles. Num: %d", obvJointNo_);
//     try {
//         if (obvJointNo_ < 2) {
//             throw std::runtime_error(
//                 "Number of joints should be greater than 1");
//         } else {
//             joints_.clear();
//             links_.clear();
//             populateStructs();
//         }
//     } catch (const std::exception &e) {
//         std::cerr << e.what() << '\n';
//     }
// }

MatrixXd Precomputation::evaluateStiffnessMatrix() {
    MatrixXd K(3 * obvJointNo_, 3 * obvJointNo_);
    for (int i = 0; i < links_.size(); i++) {
        double lRadius = links_[i].d / 2;
        double I = M_PI_4 * pow(lRadius, 4);
        double G = links_[i].E / (2 * (links_[i].v + 1));
        double J = M_PI_2 * pow(lRadius, 4);
        double Kb = links_[i].E * I / links_[i].dL;
        double Kt = G * J / links_[i].dL;
        K(3 * i, 3 * i) = Kb;
        K(3 * i + 1, 3 * i + 1) = Kb;
        K(3 * i + 2, 3 * i + 2) = Kt;
    }
    return K;
}

void Precomputation::evaluateDirectKinematics() {
    if (obvJointNo_ < 2) {
        throw std::runtime_error("Number of joints should be greater than 1");
        return;
    }
    for (int i = 0; i < obvJointNo_; i++) {
        if (i != 0) {
            AngleAxisd rotZ =
                AngleAxisd(joints_[i - 1].q[2], Vector3d::UnitZ());
            AngleAxisd rotY =
                AngleAxisd(joints_[i - 1].q[1], Vector3d::UnitY());
            AngleAxisd rotX =
                AngleAxisd(joints_[i - 1].q[0], Vector3d::UnitX());
            joints_[i].Rotation =
                joints_[i - 1].Rotation * rotZ * rotY * rotX;

            joints_[i].pLocal =
                joints_[i - 1].pLocal +
                joints_[i].Rotation * Vector3d(0, 0, -links_[i - 1].dL);
        }
        joints_[i].Orientation(placeholders::all, 0) =
            joints_[i].Rotation * Vector3d::UnitX();

        joints_[i].Orientation(placeholders::all, 1) =
            AngleAxisd(joints_[i].q[0], Vector3d::UnitX()) *
            joints_[i].Rotation * Vector3d::UnitY();

        joints_[i].Orientation(placeholders::all, 2) =
            AngleAxisd(joints_[i].q[1], Vector3d::UnitY()) *
            AngleAxisd(joints_[i].q[0], Vector3d::UnitX()) *
            joints_[i].Rotation * Vector3d::UnitZ();
    }
}

MatrixXd Precomputation::evaluateGeometricJacobian() {
    /**
     * @note
     *
     * Given J = [ 	J00 J01
     * 				J10 J11 	]
     * Where J_xy = [Jp_xy
     * 				Jo_xy]
     *
     * In the loops below, 	i tracks y
     * 						k tracks x
     *
     * Also the 'stacking' of the full jacobian is actually done by
     * initialising an empty Mat of the correct size and filling in the blocks
     * stacking in the Matrix algebra library we use is possible, but
     * a pain, so filling is good enough, probably.
     */
    Matrix3d Jp, Jo;
    int effJointNo = joints_.size();
    MatrixXd J(6 * obvJointNo_, 3 * obvJointNo_);

    for (int i = 0; i < effJointNo; i++) {
        // i goes vertically
        for (int k = 0; k < effJointNo; k++) {
            // k goes horizontally
            if (k > i) {
                Jp = Matrix3d::Zero();
                Jo = Matrix3d::Zero();
            } else {
                Vector3d pLocal;
                if (joints_[i].nextJoint == nullptr) {
                    pLocal = joints_[i].pLocal;
                } else
                    pLocal = joints_[i + 1].pLocal;
                try {
                    Vector3d pDiff = pLocal - joints_[k].pLocal;
                    Jp.col(0) = joints_[k].Orientation.col(0).cross(pDiff);
                    Jp.col(1) = joints_[k].Orientation.col(1).cross(pDiff);
                    Jp.col(2) = joints_[k].Orientation.col(2).cross(pDiff);

                    Jo.col(0) = joints_[k].Orientation.col(0);
                    Jo.col(1) = joints_[k].Orientation.col(1);
                    Jo.col(2) = joints_[k].Orientation.col(2);
                } catch (const std::exception &e) {
                    std::cerr << e.what() << '\n';
                }
            }
            MatrixXd Jn(Jp.rows() + Jo.rows(), Jp.cols());
            Jn << Jp, Jo;
            J(seq(0 + i * 6, 5 + i * 6), seq(0 + k * 3, 2 + k * 3)) = Jn;
        }
    }
    return J;
}

MatrixXd Precomputation::MagnetisationToFieldMap() {
    MatrixXd S = MatrixXd::Zero(6 * obvJointNo_, 3);
    for (int i = 0; i < obvJointNo_; i++) {
        Matrix3d rot;
        if (joints_[i].nextJoint == nullptr) {
            rot = joints_[i].Rotation;
        } else
            rot = joints_[i].nextJoint->Rotation;
        joints_[i].GlobMag = rot * joints_[i].LocMag;
        // Convert GlobMag to skew-symmetric matrix
        Matrix3d skewSymmetric;
        skewSymmetric << 0, -joints_[i].GlobMag[2], joints_[i].GlobMag[1],
            joints_[i].GlobMag[2], 0, -joints_[i].GlobMag[0],
            -joints_[i].GlobMag[1], joints_[i].GlobMag[0], 0;
        S(seqN(3 + 6 * i, 3), seqN(0, 3)) = -skewSymmetric;
    }
    return S;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "precomputation");
    Precomputation precomputation;
    ros::spin();
    return 0;
}
