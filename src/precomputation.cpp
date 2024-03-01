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

    desiredAnglesSub_ = nh.subscribe(
        "/des_angles", 10, &Precomputation::desiredAnglesCallback, this);
}

bool Precomputation::calculateField(
    shapeforming_msgs::CalcInitialField::Request &req,
    shapeforming_msgs::CalcInitialField::Response &res) {
    ROS_INFO("Calculating initial field");
    // 0. Populate the joint and link structs
    // 1. Evaluate stiffness matrix K
    evaluateStiffnessMatrix();
    // 2. Evaluate direct kinematics
    evaluateDirectKinematics();
    // 3. Evaluate geometric jacboian
    // 4. evaluate magnetisation to field map
    // 5. Ax = b
    // 6. Solve for x with lsqminnorm

    res.success = true;
    res.field.header.frame_id = "world";
    res.field.header.stamp = ros::Time::now();
    res.field.bx = 0;
    res.field.by = 0;
    res.field.bz = 0;
    return res.success;
}

void Precomputation::populateStructs() {
    // Populate the joint and link structs
    // 1. Populate the posOri structs

    // 2. Populate the joint structs
    joints_.clear();
    joints_.resize(obvJointNo_);
    for (int i = 0; i < obvJointNo_; i++) {
        joints_[i] = std::make_shared<Joint>();
        joints_[i]->index = i;
        joints_[i]->LocMag = Vector3d(magX_[i], magY_[i], magZ_[i]);
        joints_[i]->q = Vector3d(0, desiredAngles_[i], 0);
        if (i > 0) {
            joints_[i]->prevJoint = joints_[i - 1];
        } else {
            joints_[i]->prevJoint =
                nullptr;  // Set the first joint's prevJoint to NULL
        }
        if (i < obvJointNo_ - 1) {
            joints_[i]->nextJoint = joints_[i + 1];
        } else {
            joints_[i]->nextJoint =
                nullptr;  // Set the last joint's nextJoint to NULL
        }
    }
    // 3. Populate the link structs
    links_.clear();
    links_.resize(obvJointNo_ - 1);
    for (int i = 0; i < obvJointNo_ - 1; i++) {
        links_[i] = std::make_shared<Link>();
        links_[i]->index = i;
        links_[i]->base = joints_[i];
        links_[i]->head = joints_[i + 1];
        links_[i]->d = d_;
        links_[i]->dL = len_;
        links_[i]->E = E_;
        links_[i]->v = v_;
    }
    ROS_INFO("Populated RL structs");
    return;
}

void Precomputation::desiredAnglesCallback(
    const shapeforming_msgs::rl_angles::ConstPtr &msg) {
    desiredAngles_ = msg->angles;
    obvJointNo_ = msg->count;
    ROS_INFO("Received desired angles. Num: %d", obvJointNo_);
    try {
        if (obvJointNo_ < 2) {
            throw std::runtime_error(
                "Number of joints should be greater than 1");
        } else {
            populateStructs();
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }
}

MatrixXd Precomputation::evaluateStiffnessMatrix() {
    MatrixXd K(3 * obvJointNo_, 3 * obvJointNo_);
    for (int i = 0; i < links_.size(); i++) {
        double lRadius = links_[i]->d / 2;
        double I = M_PI_4 * pow(lRadius, 4);
        double G = links_[i]->E / (2 * (links_[i]->v + 1));
        double J = M_PI_2 * pow(lRadius, 4);
        double Kb = links_[i]->E * I / links_[i]->dL;
        double Kt = G * J / links_[i]->dL;
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
    for (int i = 1; i < obvJointNo_; i++) {
        AngleAxisd rotZ = AngleAxisd(joints_[i]->q[2], Vector3d::UnitZ());
        AngleAxisd rotY = AngleAxisd(joints_[i]->q[1], Vector3d::UnitY());
        AngleAxisd rotX = AngleAxisd(joints_[i]->q[0], Vector3d::UnitX());
        joints_[i]->Rotation = joints_[i - 1]->Rotation * rotZ * rotY * rotX;

        joints_[i]->pLocal =
            joints_[i - 1]->pLocal +
            joints_[i]->Rotation * Vector3d(0, 0, -links_[i - 1]->dL);

        ROS_INFO("joints_[%d]->pLocal: %f, %f, %f", i, joints_[i]->pLocal[0],
                 joints_[i]->pLocal[1], joints_[i]->pLocal[2]);
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "precomputation");
    Precomputation precomputation;
    ros::spin();
    return 0;
}
