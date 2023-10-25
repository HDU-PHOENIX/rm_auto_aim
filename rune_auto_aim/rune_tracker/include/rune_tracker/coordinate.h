#pragma once

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "armor_type.hpp"
#include "sizes.hpp"

namespace rune {

    class Coordinate {
    public:
        ~Coordinate() = default;

        cv::Mat rvec, tvec;
        Eigen::Matrix3d r_iw;
        double yaw;
        double pitch;
        double roll;

        Eigen::Matrix3d Rotated_pc2imu;

        Eigen::Vector3d Tracker;
        Eigen::Vector3d Balance800;
        Eigen::Vector3d LVBU;
        Eigen::Vector3d Hero;
        Eigen::Vector3d offset;

        Coordinate& operator=(const Coordinate& rc) {
            this->SetEuler(rc.yaw, rc.pitch, rc.roll);
            this->D_MAT = rc.D_MAT;
            this->I_MAT = rc.I_MAT;
            this->I = rc.I;
            return *this;
        }

        void SetOffSet(const Eigen::Vector3d& os) {
            offset = os;
        }
        void SetIntrinsics(const double fx, const double fy, const double cx, const double cy);
        void SetIntrinsics(const std::vector<double>& intrinsics);

        void SetExtrinsics(const double rx, const double ry, const double rz);
        void SetExtrinsics(const std::vector<double>& extrinsics);

        void SetDistortion(const double k1, const double k2, const double p1, const double p2);
        void SetDistortion(const double k1, const double k2, const double p1, const double p2, const double k3);
        void SetDistortion(const double k1, const double k2, const double p1, const double p2, const double k3, const double k4, const double k5, const double k6);
        void SetDistortion(const double k1, const double k2, const double p1, const double p2, const double k3, const double k4, const double k5, const double k6, const double s1, const double s2, const double s3, const double s4);
        void SetDistortion(const double k1, const double k2, const double p1, const double p2, const double k3, const double k4, const double k5, const double k6, const double s1, const double s2, const double s3, const double s4, const double tx, const double ty);
        void SetDistortion(const std::vector<double>& distortion);

        void SetRIW(const Eigen::Quaternion<double>& quaternion);
        void SetRIW(const double qw, const double qx, const double qy, const double qz);
        void SetRIW(const double (&q)[4]);
        void SetRIW(const std::vector<double>& quaternion);

        void SetRIW();
        /**
         * @brief get single precision Quaternion
        */
        void SetRIW(const float (&q)[4]);

        void SetEuler(const double yaw, const double pitch, const double roll);

        /**
         * @brief 使用PnP根据像素坐标获取平移向量并转换成Eigen::Vector3d
         *
         * @param type 装甲板类型
         * @param pu 像素坐标
         * @return Eigen::Vector3d 平移向量，相机坐标系下的坐标
         */
        Eigen::Vector3d PnpGetPc(ArmorType type, std::vector<cv::Point2d> pu);

        /**
         * @brief 从相机坐标系转换到世界坐标系
         *
         * @param pc 相机坐标系下的坐标
         * @param r_iw 陀螺仪四元数转换之后的旋转矩阵
         * @return Eigen::Vector3d 世界坐标系下的坐标
         */
        Eigen::Vector3d RunePcToPw(const Eigen::Vector3d& pc);

        /**
         * @brief 从世界坐标系转换到相机坐标系
         *
         * @param pw 世界坐标系下的坐标
         * @param r_iw 陀螺仪四元数转换之后的旋转矩阵
         * @return Eigen::Vector3d 相机坐标系下的坐标
         */
        Eigen::Vector3d RunePwToPc(const Eigen::Vector3d& pw);

        /**
         * @brief 从相机坐标转换到像素坐标
         *
         * @param pc 相机坐标系下的坐标
         * @return Eigen::Vector3d 像素坐标
         */
        Eigen::Vector3d RunePcToPu(const Eigen::Vector3d& pc);

        Eigen::Vector3d GetEuler();

        Eigen::Vector3d PwToPc(const Eigen::Vector3d& pw);
        Eigen::Vector3d PcToPw(const Eigen::Vector3d& pc);
        Eigen::Vector3d PcToPu(const Eigen::Vector3d& pc);

        float CalculateDistanceToCenter(const cv::Point2f& image_point);

        Coordinate();

        Eigen::Matrix3d I;
        // 相机内参矩阵
        cv::Mat I_MAT;
        // 相机畸变矩阵
        cv::Mat D_MAT;
        // 相机坐标到陀螺仪坐标的旋转矩阵
        // Eigen::Matrix3d r_ci;
        // 陀螺仪坐标到世界的旋转矩阵
    private:
        static std::vector<cv::Point3d> GeneratePw(double width, double height);
        static std::vector<cv::Point3d> GeneratePw(double outerwidth, double insidewidth, double height);

        inline static std::map<ArmorType, std::vector<cv::Point3d>> pw_map = {
            {ArmorType::Small, GeneratePw(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)},
            {ArmorType::Big, GeneratePw(BIG_ARMOR_WIDTH, BIG_ARMOR_HEIGHT)},
            {ArmorType::Rune, GeneratePw(RUNE_PNP_OUTER_LIGHTBAR_WIDTH, RUNE_PNP_INSIDE_LIGHTBAR_WIDTH, RUNE_PNP_RADIUS)}
        };

        Eigen::Vector4d pc_tmp;
        Eigen::Vector4d pi_tmp;
        Eigen::Vector3d pi;
        Eigen::Matrix4d r_ic;
        Eigen::Matrix4d r_ci;
    };
};  // namespace phoenix
