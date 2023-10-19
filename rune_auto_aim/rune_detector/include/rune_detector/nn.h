#include <Eigen/Core>
#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>

namespace rune {
    /**
     * @brief 神符的神经网络类，目前使用yolo实现
     * @author Exia (modify from TUP)
     * @todo 将自瞄一起抽象出来整合为一个基类
    */
    class NeuralNetwork {
    public:
        struct RuneObject {
            cv::Point2f vertices[5];//符叶的五个顶点
            cv::Rect_<float> rect;
            int cls;
            int color;
            float prob;
            std::vector<cv::Point2f> pts;
            //cv::Point2f symbol;
        };

        NeuralNetwork();
        ~NeuralNetwork();
        bool Init(std::string path);
        bool detect(cv::Mat& src, std::vector<RuneObject>& objects);

    private:
        InferenceEngine::Core ie;
        InferenceEngine::CNNNetwork network;                    // 网络
        InferenceEngine::ExecutableNetwork executable_network;  // 可执行网络
        InferenceEngine::InferRequest infer_request;            // 推理请求
        InferenceEngine::MemoryBlob::CPtr moutput;
        std::string input_name;
        std::string output_name;

        Eigen::Matrix<float, 3, 3> transfrom_matrix;
    };
}  // namespace phoenix::detector