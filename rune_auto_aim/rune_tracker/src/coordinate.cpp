#include "rune_tracker/coordinate.h"

namespace rune {

Coordinate::Coordinate() {
    this->Tracker = { -0.00004, -0.049, 0.14 };
    this->Balance800 = { 0, -0.10581, 0.10581 };
    this->Hero = { 0, 0.0561, 0.145 };
    this->LVBU = { 0, -0.045, 0.1045 };
    Rotated_pc2imu =
        (Eigen::AngleAxis(M_PI_2, Eigen::Vector3d::UnitX())
         * Eigen::AngleAxis(M_PI_2, Eigen::Vector3d::UnitZ()));
}

void Coordinate::SetIntrinsics(const double fx, const double cx, const double fy, const double cy) {
    I << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    cv::eigen2cv(I, I_MAT);
}

void Coordinate::SetIntrinsics(const std::vector<double>& intrinsics) {
    switch (intrinsics.size()) {
        case 4: {
            SetIntrinsics(intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
        } break;
        default: {
            throw std::invalid_argument("intrinsics must be of size 4");
        }
    }
}

void Coordinate::SetExtrinsics(const double rx, const double ry, const double rz) {
    cv::Mat tmp = (cv::Mat_<double>(1, 3) << rx, ry, rz);
    cv::Rodrigues(tmp, tmp);
}

void Coordinate::SetExtrinsics(const std::vector<double>& extrinsics) {
    switch (extrinsics.size()) {
        case 3: {
            SetExtrinsics(extrinsics[0], extrinsics[1], extrinsics[2]);
        } break;
        default: {
            throw std::invalid_argument("extrinsics must be of size 3");
        }
    }
}

void Coordinate::SetDistortion(const double k1, const double k2, const double p1, const double p2) {
    D_MAT = (cv::Mat_<double>(1, 4) << k1, k2, p1, p2);
}

void Coordinate::SetDistortion(
    const double k1,
    const double k2,
    const double p1,
    const double p2,
    const double k3
) {
    D_MAT = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
}

void Coordinate::SetDistortion(
    const double k1,
    const double k2,
    const double p1,
    const double p2,
    const double k3,
    const double k4,
    const double k5,
    const double k6
) {
    D_MAT = (cv::Mat_<double>(1, 8) << k1, k2, p1, p2, k3, k4, k5, k6);
}

void Coordinate::SetDistortion(
    const double k1,
    const double k2,
    const double p1,
    const double p2,
    const double k3,
    const double k4,
    const double k5,
    const double k6,
    const double s1,
    const double s2,
    const double s3,
    const double s4
) {
    D_MAT = (cv::Mat_<double>(1, 12) << k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4);
}

void Coordinate::SetDistortion(
    const double k1,
    const double k2,
    const double p1,
    const double p2,
    const double k3,
    const double k4,
    const double k5,
    const double k6,
    const double s1,
    const double s2,
    const double s3,
    const double s4,
    const double tx,
    const double ty
) {
    D_MAT = (cv::Mat_<double>(1, 14) << k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tx, ty);
}

void Coordinate::SetDistortion(const std::vector<double>& distortion) {
    switch (distortion.size()) {
        case 4: {
            SetDistortion(distortion[0], distortion[1], distortion[2], distortion[3]);
        } break;
        case 5: {
            SetDistortion(
                distortion[0],
                distortion[1],
                distortion[2],
                distortion[3],
                distortion[4]
            );
        } break;
        case 8: {
            SetDistortion(
                distortion[0],
                distortion[1],
                distortion[2],
                distortion[3],
                distortion[4],
                distortion[5],
                distortion[6],
                distortion[7]
            );
        } break;
        case 12: {
            SetDistortion(
                distortion[0],
                distortion[1],
                distortion[2],
                distortion[3],
                distortion[4],
                distortion[5],
                distortion[6],
                distortion[7],
                distortion[8],
                distortion[9],
                distortion[10],
                distortion[11]
            );
        } break;
        case 14: {
            SetDistortion(
                distortion[0],
                distortion[1],
                distortion[2],
                distortion[3],
                distortion[4],
                distortion[5],
                distortion[6],
                distortion[7],
                distortion[8],
                distortion[9],
                distortion[10],
                distortion[11],
                distortion[12],
                distortion[13]
            );
        } break;
        default: {
            throw std::invalid_argument("distortion must be of size 4, 5, 8, 12, or 14");
        }
    }
}

void Coordinate::SetRIW() {
    // auto&& roll_2 = 0;
    // auto&& pitch_2 = pitch / 2.;
    // auto&& yaw_2 = yaw / 2.;
    // auto&& q0 = cos(roll_2) * cos(pitch_2) * cos(yaw_2) + sin(roll_2) * sin(pitch_2) * sin(yaw_2);
    // auto&& q1 = sin(roll_2) * cos(pitch_2) * cos(yaw_2) - cos(roll_2) * sin(pitch_2) * sin(yaw_2);
    // auto&& q2 = cos(roll_2) * sin(pitch_2) * cos(yaw_2) + sin(roll_2) * cos(pitch_2) * sin(yaw_2);
    // auto&& q3 = cos(roll_2) * cos(pitch_2) * sin(yaw_2) - sin(roll_2) * sin(pitch_2) * cos(yaw_2);
    // Eigen::Quaternion q(q0, q1, q2, q3);

    //auto&& roll_2 = roll / 2. ;
    auto&& roll_2 = 0;
    auto&& pitch_2 = pitch / 2.;
    auto&& yaw_2 = yaw / 2.;
    auto&& q0 = cos(roll_2) * cos(pitch_2) * cos(yaw_2) + sin(roll_2) * sin(pitch_2) * sin(yaw_2);
    auto&& q1 = sin(roll_2) * cos(pitch_2) * cos(yaw_2) - cos(roll_2) * sin(pitch_2) * sin(yaw_2);
    auto&& q2 = cos(roll_2) * sin(pitch_2) * cos(yaw_2) + sin(roll_2) * cos(pitch_2) * sin(yaw_2);
    auto&& q3 = cos(roll_2) * cos(pitch_2) * sin(yaw_2) - sin(roll_2) * sin(pitch_2) * cos(yaw_2);
    Eigen::Quaternion q(q0, q1, q2, q3);
    r_iw = q.matrix().transpose();
    // Eigen::Vector3d pw_euler = r_iw.eulerAngles(0, 1, 2);

    // auto&& imu2world = Eigen::AngleAxis(-1 * roll, Eigen::Vector3d::UnitX()) *
    //                    Eigen::AngleAxis(-1 * pitch, Eigen::Vector3d::UnitY()) *
    //                    Eigen::AngleAxis(-1 * yaw, Eigen::Vector3d::UnitZ());
    // r_iw = imu2world.matrix();
    // r_iw = q.matrix().transpose();
}

void Coordinate::SetRIW(const Eigen::Quaternion<double>& quaternion) {
    r_iw = quaternion.matrix().transpose();
    // auto&& roll_2 = roll / 2. / 180. * M_PI;
    auto&& roll_2 = 0;
    auto&& pitch_2 = pitch / 2.;
    auto&& yaw_2 = yaw / 2.;
    auto&& q0 = cos(roll_2) * cos(pitch_2) * cos(yaw_2) + sin(roll_2) * sin(pitch_2) * sin(yaw_2);
    auto&& q1 = sin(roll_2) * cos(pitch_2) * cos(yaw_2) - cos(roll_2) * sin(pitch_2) * sin(yaw_2);
    auto&& q2 = cos(roll_2) * sin(pitch_2) * cos(yaw_2) + sin(roll_2) * cos(pitch_2) * sin(yaw_2);
    auto&& q3 = cos(roll_2) * cos(pitch_2) * sin(yaw_2) - sin(roll_2) * sin(pitch_2) * cos(yaw_2);
    Eigen::Quaternion q(q0, q1, q2, q3);
    // std::cout << q.matrix() << std::endl;
    r_iw = q.matrix().transpose();
}

void Coordinate::SetRIW(const double qw, const double qx, const double qy, const double qz) {
    SetRIW(Eigen::Quaternion<double>(qw, qx, qy, qz));
}

void Coordinate::SetRIW(const double (&q)[4]) {
    SetRIW(q[0], q[1], q[2], q[3]);
}

void Coordinate::SetRIW(const std::vector<double>& quaternion) {
    switch (quaternion.size()) {
        case 4: {
            SetRIW(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
        } break;
        default: {
            throw std::invalid_argument("q must be of size 4");
        }
    }
}

void Coordinate::SetRIW(const float (&q)[4]) {
    Eigen::Quaternion tmp((double)q[0], (double)q[1], (double)q[2], (double)q[3]);
    Coordinate::SetRIW(tmp);
}

void Coordinate::SetEuler(const double yaw, const double pitch, const double roll) {
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
}

Eigen::Vector3d pw;
Eigen::Vector3d Coordinate::PnpGetPc(ArmorType type, std::vector<cv::Point2d> pu) {
    // cv::solvePnP(pw_map[type], pu, I_MAT, D_MAT, rvec, tvec);
    cv::solvePnP(pw_map[type], pu, I_MAT, D_MAT, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    Eigen::Vector3d pc;
    cv::cv2eigen(tvec, pc);
    return pc;
}

Eigen::Vector3d Coordinate::RunePcToPw(const Eigen::Vector3d& pc) {
    auto&& rotated_matrix = Eigen::AngleAxis(M_PI / 2, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxis(M_PI / 2, Eigen::Vector3d::UnitZ());
    return rotated_matrix.matrix().transpose() * pc;
    // r_ic << 0., 0., 1., 0.,
    //     1., 0., 0., 0.,
    //     0., 1., 0., 0.,
    //     0., 0., 0., 1.;
    // pc_tmp << pc[0], pc[1], pc[2], 1;
    // pi_tmp = r_ic * pc_tmp;
    // pi << pi_tmp[0], pi_tmp[1], pi_tmp[2];
    // pw = r_iw * pi;
}

Eigen::Vector3d Coordinate::RunePwToPc(const Eigen::Vector3d& pw) {
    auto&& rotated_matrix = Eigen::AngleAxis(-M_PI / 2, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxis(M_PI / 2, Eigen::Vector3d::UnitY());
    // auto&& temp_pc = rotated_matrix.matrix().transpose() * pw;
    // auto temp_pc = (r_iw.transpose() * Rotated_pc2imu.transpose()).transpose() * pw;
    // auto temp_pc = Rotated_pc2imu * (r_iw * pw);
    // auto output_pc = rotated_matrix.matrix().transpose() * Eigen::Vector3d{temp_pc[0] + offset[0], temp_pc[1] + offset[1], temp_pc[2] + offset[2]};
    // return r_iw.transpose() * Eigen::Vector3d{temp_pc[0] + offset[0], temp_pc[1] + offset[1], temp_pc[2] + offset[2]};
    // return pc;
    // return output_pc;
    return rotated_matrix.matrix().transpose() * pw;
}

Eigen::Vector3d Coordinate::RunePcToPu(const Eigen::Vector3d& pc) {
    return I * pc / pc(2, 0);
}

std::vector<cv::Point3d> Coordinate::GeneratePw(double width, double height) {
    double w2 = width / 2, h2 = height / 2;
    return {
        { 0, w2, -h2 },
        { 0, w2, h2 },
        { 0, -w2, h2 },
        { 0, -w2, -h2 },
    };
}
std::vector<cv::Point3d>
Coordinate::GeneratePw(double outerwidth, double insidewidth, double height) {
    return {
        { -1 * outerwidth / 2, height / 2, 0 },
        { -1 * insidewidth / 2, -1 * height / 2, 0 },
        { insidewidth / 2, -1 * height / 2, 0 },
        { outerwidth / 2, height / 2, 0 },
    };
}

Eigen::Vector3d Coordinate::GetEuler() {
    return { this->yaw, this->roll, this->pitch };
}
float Coordinate::CalculateDistanceToCenter(const cv::Point2f& image_point) {
    float cx = I_MAT.at<double>(0, 2);
    float cy = I_MAT.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

Eigen::Vector3d Coordinate::PcToPw(const Eigen::Vector3d& pc) {
    r_ic << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.;
    pc_tmp << pc[0], pc[1], pc[2], 1;
    pi_tmp = r_ic * pc_tmp;
    //pi << pi_tmp[0]  - 0.00004  , pi_tmp[1] - 0.049, pi_tmp[2] + 0.1175; //麦轮
    //pi << pi_tmp[0]    , pi_tmp[1] - 0.10581, pi_tmp[2] + 0.10581; //平衡
    // pi << pi_tmp[0], pi_tmp[1] + 0.0561, pi_tmp[2] + 0.145;  //英雄
    pi << pi_tmp[0], pi_tmp[1] - 0.045, pi_tmp[2] + 0.1045; //全向
    //pi << pi_tmp[0]  , pi_tmp[1], pi_tmp[2] ;
    //Eigen::Vector3d pc_euler = r_iw.eulerAngles(2,1,0).transpose() ;
    //std::cout<<"*********r_iw:"<<"\n"<<pc_euler<<std::endl;
    Eigen::Vector3d eulerAngle(M_PI_2, 0, M_PI_2);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));

    Eigen::Matrix3d rotation_matrix_;
    rotation_matrix_ = rollAngle * pitchAngle * yawAngle;
    pw = (rotation_matrix_ * r_iw).transpose() * pi;
    //std::cout<<"x:"<<pw[0]<<std::endl;
    //std::cout<<"y:"<<pw[1]<<std::endl;
    //std::cout<<"z:"<<pw[2]<<std::endl;
    return pw;
}

Eigen::Vector3d Coordinate::PwToPc(const Eigen::Vector3d& pw) {
    r_ci << 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.;
    Eigen::Vector3d eulerAngle(M_PI_2, 0, M_PI_2);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));

    Eigen::Matrix3d rotation_matrix_;
    rotation_matrix_ = rollAngle * pitchAngle * yawAngle;
    pi = (rotation_matrix_ * r_iw) * pw;
    pi_tmp << pi[0], pi[1], pi[2], 1;
    pc_tmp = r_ci * pi_tmp;
    Eigen::Vector3d pc;
    // pc << pc_tmp[0], pc_tmp[1] - 0.0561, pc_tmp[2] - 0.145;  //英雄
    //pc << pc_tmp[0]+ 0.00004, pc_tmp[1] + 0.049, pc_tmp[2] - 0.1175; //麦轮
    //pc << pi_tmp[0] , pi_tmp[1] + 0.10581, pi_tmp[2] - 0.10581; //平衡
    pc << pc_tmp[0], pc_tmp[1] + 0.045, pc_tmp[2] - 0.1045;
    //pc << pc_tmp[0], pc_tmp[1], pc_tmp[2];
    return pc;
}

Eigen::Vector3d Coordinate::PcToPu(const Eigen::Vector3d& pc) {
    return I * pc / pc(2, 0);
}
} // namespace rune
