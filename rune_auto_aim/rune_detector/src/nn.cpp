#include "rune_detector/nn.h"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
namespace rune {

// static constexpr int INPUT_W = 640;    // Width of input
// static constexpr int INPUT_H = 384;    // Height of input
static constexpr int INPUT_W = 416;   // Width of input
static constexpr int INPUT_H = 416;   // Height of input
static constexpr int NUM_CLASSES = 3; // Number of classes
static constexpr int NUM_COLORS = 2;  // Number of color
static constexpr int TOPK = 128;      // TopK
static constexpr float NMS_THRESH = 0.1;
static constexpr float BBOX_CONF_THRESH = 0.9;
static constexpr float MERGE_CONF_ERROR = 0.15;
static constexpr float MERGE_MIN_IOU = 0.1;

struct GridAndStride {
    int grid0;
    int grid1;
    int stride;
};

static inline int argmax(const float* ptr, int len) {
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg])
            max_arg = i;
    }
    return max_arg;
}
/**
     * @brief Resize the image using letterbox
     * @param img Image before resize
     * @param transform_matrix Transform Matrix of Resize
     * @return Image after resize
     */
inline cv::Mat scaledResize(cv::Mat& img, Eigen::Matrix<float, 3, 3>& transform_matrix) {
    float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;

    int dw = INPUT_W - unpad_w;
    int dh = INPUT_H - unpad_h;

    dw /= 2;
    dh /= 2;

    transform_matrix << 1.0 / r, 0, -dw / r,
        0, 1.0 / r, -dh / r,
        0, 0, 1;

    cv::Mat re;
    cv::resize(img, re, cv::Size(unpad_w, unpad_h));
    cv::Mat out;
    cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);
    return out;
}

/**
     * @brief Generate grids and stride.
     * @param target_w Width of input.
     * @param target_h Height of input.
     * @param strides A vector of stride.
     * @param grid_strides Grid stride generated in this function.
     */
static void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides) {
    for (auto stride: strides) {
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;

        for (int g1 = 0; g1 < num_grid_h; g1++) {
            for (int g0 = 0; g0 < num_grid_w; g0++) {
                grid_strides.push_back(GridAndStride { g0, g1, stride });
            }
        }
    }
}

/**
     * @brief Generate Proposal
     * @param grid_strides Grid strides
     * @param feat_ptr Original predition result.
     * @param prob_threshold Confidence Threshold.
     * @param objects Objects proposed.
     */
static void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr, Eigen::Matrix<float, 3, 3>& transform_matrix, float prob_threshold, std::vector<NeuralNetwork::RuneObject>& objects) {
    const int num_anchors = grid_strides.size();
    //Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        const int basic_pos = anchor_idx * (11 + NUM_COLORS + NUM_CLASSES);

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
        float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
        float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
        float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
        float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
        float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
        float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
        float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;
        float x_5 = (feat_ptr[basic_pos + 8] + grid0) * stride;
        float y_5 = (feat_ptr[basic_pos + 9] + grid1) * stride;

        int box_color = argmax(feat_ptr + basic_pos + 11, NUM_COLORS);
        int box_class = argmax(feat_ptr + basic_pos + 11 + NUM_COLORS, NUM_CLASSES);

        float box_objectness = (feat_ptr[basic_pos + 10]);

        // float color_conf = (feat_ptr[basic_pos + 11 + box_color]);
        // float cls_conf = (feat_ptr[basic_pos + 11 + NUM_COLORS + box_class]);

        // cout<<box_objectness<<endl;
        // float box_prob = (box_objectness + cls_conf + color_conf) / 3.0;
        float box_prob = box_objectness;

        if (box_prob >= prob_threshold) {
            NeuralNetwork::RuneObject obj;

            Eigen::Matrix<float, 3, 5> vertices_norm;
            Eigen::Matrix<float, 3, 5> vertices_dst;

            vertices_norm << x_1, x_2, x_3, x_4, x_5,
                y_1, y_2, y_3, y_4, y_5,
                1, 1, 1, 1, 1;

            vertices_dst = transform_matrix * vertices_norm;

            for (int i = 0; i < 5; i++)
                obj.vertices[i] = cv::Point2f(vertices_dst(0, i), vertices_dst(1, i));
            for (int i = 0; i < 5; i++) {
                obj.vertices[i] = cv::Point2f(vertices_dst(0, i), vertices_dst(1, i));
                obj.pts.push_back(obj.vertices[i]);
            }
            std::vector<cv::Point2f> tmp(obj.vertices, obj.vertices + 5);
            obj.rect = cv::boundingRect(tmp);

            obj.cls = box_class;
            obj.color = box_color;
            obj.prob = box_prob;

            objects.push_back(obj);
        }

    } // point anchor loop
}

/**
     * @brief Calculate intersection area between two objects.
     * @param a Object a.
     * @param b Object b.
     * @return Area of intersection.
     */
static inline float intersection_area(const NeuralNetwork::RuneObject& a, const NeuralNetwork::RuneObject& b) {
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<NeuralNetwork::RuneObject>& faceobjects, int left, int right) {
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j) {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j) {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

#pragma omp parallel sections
    {
#pragma omp section
        {
            if (left < j)
                qsort_descent_inplace(faceobjects, left, j);
        }
#pragma omp section
        {
            if (i < right)
                qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<NeuralNetwork::RuneObject>& objects) {
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

static void nms_sorted_bboxes(std::vector<NeuralNetwork::RuneObject>& faceobjects, std::vector<int>& picked, float nms_threshold) {
    picked.clear();
    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) {
        std::vector<cv::Point2f> object_vertices_tmp(faceobjects[i].vertices, faceobjects[i].vertices + 5);
        areas[i] = contourArea(object_vertices_tmp); //计算多边形面积
        // areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++) {
        NeuralNetwork::RuneObject& a = faceobjects[i];
        std::vector<cv::Point2f> vertices_a(a.vertices, a.vertices + 5);
        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++) {
            NeuralNetwork::RuneObject& b = faceobjects[picked[j]];
            std::vector<cv::Point2f> vertices_b(b.vertices, b.vertices + 5);
            std::vector<cv::Point2f> vertices_inter;
            // intersection over union
            // float inter_area = intersection_area(a, b);
            // float union_area = areas[i] + areas[picked[j]] - inter_area;
            //TODO:此处耗时较长，大约1ms，可以尝试使用其他方法计算IOU与多边形面积
            float inter_area = intersectConvexConvex(vertices_a, vertices_b, vertices_inter); //交集面积
            float union_area = areas[i] + areas[picked[j]] - inter_area;                      //并集面积
            float iou = inter_area / union_area;

            if (iou > nms_threshold || isnan(iou)) {
                keep = 0;
                //Stored for Merge
                if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR && a.cls == b.cls && a.color == b.color) {
                    for (int i = 0; i < 5; i++) {
                        b.pts.push_back(a.vertices[i]);
                    }
                }
                // cout<<b.pts_x.size()<<endl;
            }
        }

        if (keep)
            picked.push_back(i);
    }
}
/**
     * @brief Decode outputs.
     * @param prob Original predition output.
     * @param objects Vector of objects predicted.
     * @param img_w Width of Image.
     * @param img_h Height of Image.
     */
static void decodeOutputs(const float* prob, std::vector<NeuralNetwork::RuneObject>& objects, Eigen::Matrix<float, 3, 3>& transform_matrix) {
    std::vector<NeuralNetwork::RuneObject> proposals;
    std::vector<int> strides = { 8, 16, 32 };
    std::vector<GridAndStride> grid_strides;

    generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
    generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals);
    qsort_descent_inplace(proposals); //按照prob从大到小排序

    if (proposals.size() >= TOPK)
        proposals.resize(TOPK);
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, NMS_THRESH);
    int count = picked.size();
    objects.resize(count);

    for (int i = 0; i < count; i++) {
        objects[i] = proposals[picked[i]];
    }
}
NeuralNetwork::NeuralNetwork() {
}

NeuralNetwork::~NeuralNetwork() {
}

//TODO:change to your dir
bool NeuralNetwork::Init(std::string path) {
    // Setting Configuration Values.
    core.set_property("CPU", ov::enable_profiling(true));

    // Step 1.Create openvino runtime core
    model = core.read_model(path);

    // Preprocessing.
    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input().tensor().set_element_type(ov::element::f32);

    // set output precision.
    ppp.output().tensor().set_element_type(ov::element::f32);

    // 将预处理融入原始模型.
    ppp.build();

    // Step 2. Compile the model
    compiled_model = core.compile_model(
        model,
        "CPU",
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
        // "AUTO:GPU,CPU",
        // ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
        // ov::hint::inference_precision(ov::element::u8)
    );

    // compiled_model.set_property(ov::device::priorities("GPU"));

    // Step 3. Create an Inference Request
    infer_request = compiled_model.create_infer_request();

    // Fill Input Tensors with Data
    // get input tensor by index
    // input_tensor = infer_request.get_input_tensor(0);

    // Step 4. Set Inputs
    // Get input port for model with one input
    // auto input_port = compiled_model.input();

    // Create tensor from external memory
    // ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), memory_ptr);
    // input_tensor = ov::Tensor(input_port.get_element_type(), input_port.get_shape(), memory_ptr);

    // // Set input tensor for model with one input
    // infer_request.set_input_tensor(input_tensor);

    // //Step 5. Start Inference
    // infer_request.start_async();
    // infer_request.wait();

    // //Step 6. Process the Inference Results
    // // Get output tensor by tensor name
    // auto output = infer_request.get_tensor("tensor_name");
    // const float output_buffer = output.data<const float>();
    // output_buffer[] - accessing output tensor data

    return true;
}

bool NeuralNetwork::detect(cv::Mat& src, std::vector<NeuralNetwork::RuneObject>& objects) {
    if (src.empty()) {
        return false;
    }
    cv::Mat pr_img = scaledResize(src, transfrom_matrix);
    cv::Mat pre;
    cv::Mat pre_split[3];
    pr_img.convertTo(pre, CV_32F);
    cv::split(pre, pre_split);
    // Get input tensor by index
    input_tensor = infer_request.get_input_tensor(0);
    // 准备输入
    infer_request.set_input_tensor(input_tensor);

    float* tensor_data = input_tensor.data<float_t>();

    auto img_offset = INPUT_H * INPUT_W;
    // Copy img into tensor
    for (int c = 0; c < 3; c++)
    {
        memcpy(tensor_data, pre_split[c].data, INPUT_H * INPUT_W * sizeof(float));
        tensor_data += img_offset;
    }
    // auto st = std::chrono::steady_clock::now();
    // 推理
    infer_request.infer();
    // auto end = std::chrono::steady_clock::now();
    // double infer_dt = std::chrono::duration<double,std::milli>(end - st).count();
    // cout << "infer_time:" << infer_dt << endl;

    // 处理推理结果
    ov::Tensor output_tensor = infer_request.get_output_tensor();
    float* output = output_tensor.data<float_t>();

    decodeOutputs(output, objects, transfrom_matrix);
    for (auto object = objects.begin(); object != objects.end(); ++object)
    {
        if ((*object).pts.size() >= 10) {
            auto N = (*object).pts.size();
            cv::Point2f pts_final[5];

            for (long unsigned int i = 0; i < N; i++) {
                pts_final[i % 5] += (*object).pts[i];
            }

            for (int i = 0; i < 5; i++) {
                pts_final[i].x = pts_final[i].x / (N / 5);
                pts_final[i].y = pts_final[i].y / (N / 5);
            }

            (*object).vertices[0] = pts_final[0];
            (*object).vertices[1] = pts_final[1];
            (*object).vertices[2] = pts_final[2];
            (*object).vertices[3] = pts_final[3];
            (*object).vertices[4] = pts_final[4];
        }
        // (*object).area = (int)(calcTetragonArea((*object).vertices));
    }
    if (objects.size() != 0)
        return true;
    else
        return false;
}

} // namespace rune