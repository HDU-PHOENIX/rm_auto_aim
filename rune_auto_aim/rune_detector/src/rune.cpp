#include "rune.h"

#define SHOW_DEBUG true

#define LOG_DEBUG false

#define LOG_TIME false

#define MODULE_TIME false  //测试模块时间，与LOG_TIME只能true一个

#define NEW_RUNE true

//符的传统部分建议新写

namespace phoenix::detector {

    Rune::Rune()
        : last() {
        auto&& config = Config::GetInstance();
        Neural_Network = config->detector.rune.Neural_Network;
        if (Neural_Network) {
            CONFIDENCE = config->detector.rune.CONFIDENCE;
            // yolo.Init("../assets/model/Rune/model_15/yolox_fp16.onnx");
            yolo.Init("../assets/model/Rune/model_12/yolox.onnx");
            // yolo.Init("../yolox.onnx");//上届的模型改成 320 320输入

            BLUE_GRAY_THRESH = config->detector.rune.BLUE_GRAY_THRESH;
            BLUE_COLOR_THRESH = config->detector.rune.BLUE_COLOR_THRESH;
            RED_GRAY_THRESH = config->detector.rune.RED_GRAY_THRESH;
            RED_COLOR_THRESH = config->detector.rune.RED_COLOR_THRESH;

        } else {
            BLUE_GRAY_THRESH = config->detector.rune.BLUE_GRAY_THRESH;
            BLUE_COLOR_THRESH = config->detector.rune.BLUE_COLOR_THRESH;
            RED_GRAY_THRESH = config->detector.rune.RED_GRAY_THRESH;
            RED_COLOR_THRESH = config->detector.rune.RED_COLOR_THRESH;
        }
    }
    void Rune::detect(sptr<Component::Data> data) {
        this->data = std::static_pointer_cast<Data>(data);
#if LOG_TIME
#define TIMEIT(FUNC)                  \
    Stopwatch::TimeIt(#FUNC, [this] { \
        return FUNC();                \
    })
#elif !(LOG_TIME || MODULE_TIME)
#define TIMEIT(FUNC) FUNC();
#endif

#if (MODULE_TIME & !LOG_TIME)
#define TIMEIT(FUNC)                                                     \
    auto RUN_TIME_##FUNC = Timestamp::Now();                             \
    static Statistic<double> time_##FUNC;                                \
    COUNT++;                                                             \
    if (COUNT == 250) {                                                  \
        COUNT = 0;                                                       \
        time_##FUNC.Clear();                                             \
    }                                                                    \
    FUNC();                                                              \
    time_##FUNC.Push(RUN_TIME_##FUNC.GetTimePassed().GetMilliseconds()); \
    data->sensor->typesetter->Add(Position::TopLeft,                     \
                                  fmt::format("{}:{:6.2f}/{:6.2f}/{:6.2f}/ms", #FUNC, time_##FUNC.Min(), time_##FUNC.Mean(), time_##FUNC.Max()), Colors::White, 20);

#endif
        TIMEIT(Prepare);

        if (Neural_Network == 1) {  //使用传统视觉还是神经网络
            Yolo();
        } else {
            TIMEIT(Binarize);
            TIMEIT(FindContours);
            TIMEIT(Filter);
            // TIMEIT(Select);
            // TIMEIT(Closeout);
        }
    }
#undef TIMEIT

    bool Rune::Yolo() {
        auto input = src.clone();
        // cv::Mat color_mask = src.clone();
        // for(int i=0;i<contours.size();i++)
        // {
        //     if(hierarchy[i][3] == -1){
        //         continue;
        //     }

        // cv::drawContours(mask,contours,-1,255,-1);
        //     // cv::floodFill(mask,Center(contours[i]),255);
        //     // DrawRotateRectangle(color_mask,rects[i],Colors::White,-1);
        // }
        // cv::imshow("color_mask",mask);
        // cv::imshow("mask",mask);
        // for(int i =0;i<src.rows;i++){
        //     for(int j =0;j<src.cols;j++){
        //         input.at<cv::Vec3b>(i,j)[0] = input.at<cv::Vec3b>(i,j)[0] * (mask.at<uchar>(i,j) /255);
        //         input.at<cv::Vec3b>(i,j)[1] = input.at<cv::Vec3b>(i,j)[1] * (mask.at<uchar>(i,j) /255);
        //         input.at<cv::Vec3b>(i,j)[2] = input.at<cv::Vec3b>(i,j)[2] * (mask.at<uchar>(i,j) /255);
        //     }
        // }
        // Mat input = src.clone().mul(mask);

        data->find = false;
        if (!yolo.detect(input, objects)) {  //yolo无法检测到目标
            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0;
            data->find = false;
            Log::Warn("no target");
            return false;
        }
        // if (!data->vertices.empty())
        //     data->vertices.clear();

        RuneClass cls;//符叶枚举类对象
        bool flag1 = false, flag2 = false, flag3 = false;
        ///------------------------生成扇叶对象----------------------------------------------
        for (auto object : objects) {
            auto prob = object.prob;
            if (prob < CONFIDENCE) {
                Log::Info("the confidence is :{}", prob);
                continue;
            }

            RuneTracker runetracker;
            auto&& detect_center = (object.vertices[0] + object.vertices[1] + object.vertices[2] + object.vertices[3] + object.vertices[4]) / 5;

            auto&& get_symbol = [](const cv::Point2f& lightbar_mid_point, const cv::Point2f& armor_center, const double& center_lightbar_ratio, const bool& flag) {  
                //get_symbol是通过符叶的坐标来计算中心R标的位置
                if (flag == 0) {
                    //flag = 0使用装甲板中心和内灯条算出标识符位置
                    return ((lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);

                } else if (flag == 1) {
                    //flag = 1使用装甲板中心和外灯条算出标识符位置
                    return (-(lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);
                }
                return cv::Point2f(0, 0);
            };

            if (object.color == 0 && object.cls == 0) {
                cls = RuneClass::Blue;
                flag1 = true;
                data->symbol = detect_center;

            } else if (object.color == 1 && object.cls == 0) {
                cls = RuneClass::Red;
                flag1 = true;
                data->symbol = detect_center;

            } else if (object.color == 0 && object.cls == 1) {
                cls = RuneClass::BlueUnActivated;
                data->vertices.clear();

                data->vertices.push_back(object.vertices[1]);
                data->vertices.push_back(object.vertices[2]);
                data->vertices.push_back(object.vertices[4]);
                data->vertices.push_back(object.vertices[0]);
                auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
                auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
                auto&& armor = (tmp1 + tmp2) / 2;//装甲板中心
                if (!flag1)//如果yolo没有检测到R标
                {
                    // data->symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 3.5542, 0)) / 2;
                    data->symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 4.0542, 0)) / 2;//相对较准
                }
                data->armor = armor;
                cv::circle(data->dst, data->armor, 4, Colors::Aqua, -1);
                cv::circle(data->dst, data->symbol, 4, Colors::Yellow, -1);
                flag1 = true;
                flag2 = true;

            } else if (object.color == 1 && object.cls == 1) {
                cls = RuneClass::RedUnActivated;
                data->vertices.clear();

                data->vertices.push_back(object.vertices[1]);
                data->vertices.push_back(object.vertices[2]);
                data->vertices.push_back(object.vertices[4]);
                data->vertices.push_back(object.vertices[0]);
                auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
                auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
                auto&& armor = (tmp1 + tmp2) / 2;
                if (!flag1)
                {
                    // data->symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 3.5542, 0)) / 2;
                    data->symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2, armor, 4.0542, 0)) / 2;
                }  //如果yolo没有检测到R标
                    

                data->armor = armor;
                cv::circle(data->dst, data->armor, 6, Colors::Aqua, -1);
                cv::circle(data->dst, data->symbol, 6, Colors::Yellow, -1);
                flag1 = true;
                flag2 = true;

            } else if (object.color == 0 && object.cls == 2) {  //已激活的符叶，可以用来扩展一张图中的得到的信息数量
                cls = RuneClass::BlueActivated;
                flag3 = true;

            } else if (object.color == 1 && object.cls == 2) {  //已激活的符叶，可以用来扩展一张图中的得到的信息数量
                cls = RuneClass::RedActivated;
                flag3 = true;
            }
            for (int i = 0; i < 5; i++) {  //画出五个关键点
                cv::circle(data->dst, object.vertices[i], 5, Colors::White, -1);
                cv::circle(data->dst, (object.vertices[0] + object.vertices[1] + object.vertices[2] + object.vertices[4]) / 4, 5, Colors::White, -1);
            }
        }
        if (flag1 && flag2)  //有R标数据和符叶数据，则认为识别完成
            data->find = true;
        else {
            data->find = false;
        }
        return true;
    }

    bool Rune::Prepare() {
        data->find = false;
        src = data->sensor->camera[Sensor::CameraIdentify::Main];
        dst = data->dst = src.clone();
        return true;
    }

    bool Rune::Binarize() {
        vector<Mat> BGR;
        Mat color_mask, gray_mask, gray;
        split(src, BGR);
        cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        // switch(data->sensor->control[Sensor::ControlIdentify::Main].color){
        switch (TeamColor::Blue) {
            case TeamColor::Red: {
                cv::threshold(gray, gray_mask, RED_GRAY_THRESH, 255, cv::THRESH_BINARY);
                cv::subtract(BGR[2], BGR[0], color_mask);
                cv::threshold(color_mask, color_mask, RED_COLOR_THRESH, 255, cv::THRESH_BINARY);
                break;
            }
            case TeamColor::Blue: {
                cv::threshold(gray, gray_mask, BLUE_GRAY_THRESH, 255, cv::THRESH_BINARY);
                cv::subtract(BGR[0], BGR[2], color_mask);
                cv::threshold(color_mask, color_mask, BLUE_COLOR_THRESH, 255, cv::THRESH_BINARY);
                break;
            }
            default: {
                throw std::runtime_error("Unknow TeamColor!");
            }
        }

        cv::bitwise_and(gray_mask, color_mask, mask);
        // cv::dilate(mask,mask,RECT3);                 //使用大的图形学运算元素会大幅降低速度
        cv::erode(mask, mask, RECT5);
        // cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,RECT3);
        // cv::erode(mask, mask, RECT3);

        // cv::morphologyEx(mask,mask,cv::MORPH_OPEN,RECT3);//因为旧的自制符的边不完整，需要大的图形学运算元素去进行闭运算，使不完整的地方闭合
        cv::GaussianBlur(mask, mask, cv::Size(3, 3), 3, 3);

#if SHOW_DEBUG
        cv::imshow("Binary", mask);
        if (cv::waitKey(1) == 'q')
            exit(0);
#endif
        return true;
    }
    /**
     * @brief 用于调整向量的大小，或对向量元素进行函数运算
     * 
     * @param value 输入向量
     * @param size 需要调整的大小
     * @param func 参数类型为引用类型的向量，返回值为空的函数指针
    */
    template<typename TVector>
    requires IsTemplateOf<std::vector, TVector>
    void resize(TVector& value, std::size_t size, std::function<void(typename TVector::value_type&)> func = nullptr) {
        if (value.size() < size) {
            value.resize(size);
        }
        if (func) {
            for (size_t i = 0, n = value.size(); i < n; ++i) {
                func(value[i]);
            }
        }
    }

    bool Rune::FindContours() {
#if NEW_RUNE

        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        contour_size = contours.size();
        resize(approximations, contour_size);
        resize(children, contour_size, [](auto&& value) {
            value.clear();
        });
        resize(convex_hulls, contour_size);
        resize(areas, contour_size);
        resize(rects, contour_size);
        return true;

        for (size_t i = 0; i < contour_size; i++) {
            rects[i] = cv::minAreaRect(contours[i]);
            areas[i] = cv::contourArea(contours[i]);

#if SHOW_DEBUG
            cv::Point2f points[4];
            rects[i].points(points);
            // for (int i = 0; i < 4; ++i) {
            //     cv::line(dst, points[i], points[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            // }

            // cv::drawContours(dst, contours, i, Colors::Cyan, 2, cv::LINE_AA);
            // DrawRotateRectangle(dst,ellipse[i],Colors::White);
#endif
        }

#else

        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        resize(approximations, contours.size());
        resize(children, contours.size(), [](auto&& value) {
            value.clear();
        });
        resize(convex_hulls, contours.size());
        resize(areas, contours.size());
        resize(rects, contours.size());

        for (size_t i = 0; i < contours.size(); ++i) {
            // cv::approxPolyDP(contours[i], approximations[i], cv::arcLength(contours[i], true) / 100, true);
            rects[i] = cv::minAreaRect(contours[i]);
            areas[i] = cv::contourArea(contours[i]);
            if (hierarchy[i][3] != -1) {
                children[hierarchy[i][3]].push_back(i);  //herarchy[i][3]为父轮廓编号，i为子轮廓编号
            }
#if SHOW_DEBUG
            cv::Point2f points[4];
            rects[i].points(points);
            for (int i = 0; i < 4; ++i) {
                cv::line(dst, points[i], points[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            }
            cv::drawContours(dst, contours, i, Colors::Cyan, 2, cv::LINE_AA);
#endif
        }
#endif
        return true;
    }

    bool Rune::Filter() {
#if NEW_RUNE

        possible_leaves.clear();
        possible_symbols.clear();
        possible_outer_lightbar.clear();
        possible_inside_lightbar.clear();
        hierarchy.clear();
        areas.clear();
        rects.clear();
        ring.size();

        layer_bool = false;

        /**
         * @brief 统计第index个轮廓的统计轮廓个数
        */
        static auto&& count_children = [](const std::vector<cv::Vec4i>& vec, const int& index) {
            int count = 0;
            int in_index = index;
            if (index == -1) {
                return -1;
            }
            while (vec[in_index][0] != -1) {
                count++;
                in_index = vec[in_index][0];
            }
            in_index = index;
            while (vec[in_index][1] != -1) {
                count++;
                in_index = vec[in_index][1];
            }
            return count;
        };

        for (size_t i = 0; i < contour_size; i++) {
            auto width = rects[i].size.width,
                 length = rects[i].size.height;
            if (width > length) {
                std::swap(width, length);
            }
            if (!(width || length)) {
                continue;
            }
            if (hierarchy[i][2] == -1) {
                continue;
            }

            if (count_children(hierarchy, hierarchy[i][2]) == 4 && (InRange(width / length, LENGTH_RATIO_ERROR_RATE_MIN, LENGTH_RATIO_ERROR_RATE_MAX)))  //找到子轮廓总和为4的父轮廓
            {                                                                                                                                            //TODO：增加多个父轮廓的可能情况
                DrawContour(data->dst, contours[i], Colors::White, 6);
                layer1 = i;
                layer_bool = true;
                continue;
            }

            if (width * length <= MIN_AREA * 10)
                continue;

            // if(InRange(length / width,CENTER_RATIO_MIN,CENTER_RATIO_MAX) )               //筛选标识符中心

            // if(InRange(length / width,CENTER_RATIO_MIN,CENTER_RATIO_MAX) && hierarchy[i][3] == -1 && hierarchy[i][2] == -1)               //筛选标识符中心
            if (InRange(length / width, CENTER_RATIO_MIN, CENTER_RATIO_MAX) && hierarchy[i][3] == -1) {
                // continue;
                possible_symbols.emplace_back(i);
                // DrawRotateRectangle(data->dst,rects[i],Colors::Blue,5);//可能是中心的用蓝色画出
            }

            if (InRange(length / width, RUNE_ARMOR_OUTER_LIGHTBAR_RATIO_MIN, RUNE_ARMOR_OUTER_LIGHTBAR_RATIO_MAX) && hierarchy[i][2] == -1 && hierarchy[i][3] == -1)  //筛选外灯条
            {
                possible_outer_lightbar.push_back(i);
                // phoenix::DrawRotateRectangle(data->dst,rects[i],Colors::Green,5);//可能是外灯条的用绿色画出
            }

            if (InRange(length / width, RUNE_ARMOR_INSIDE_LIGHTBAR_RATIO_MIN, RUNE_ARMOR_INSIDE_LIGHTBAR_RATIO_MAX) && hierarchy[i][2] == -1 && hierarchy[i][3] == -1)  //筛选内灯条
            {
                possible_inside_lightbar.push_back(i);
                // phoenix::DrawRotateRectangle(data->dst,rects[i],Colors::Red,5);//可能是内灯条的用红画出
            }

            if (InRange(length / width, LENGTH_RATIO_ERROR_RATE_MIN, LENGTH_RATIO_ERROR_RATE_MAX) && hierarchy[i][3] == -1 && hierarchy[i][2] != 0 && hierarchy[hierarchy[i][2]][2] == -1)  //筛选已激活的圆环
            {
                ring.push_back(i);
                // DrawContour(data->dst,contours[i],Colors::Green,5);
            }
        }

#else

        possible_leaves.clear();
        possible_symbols.clear();
        for (size_t i = 0, size = contours.size(); i < size; ++i) {
            auto width = rects[i].size.width,
                 length = rects[i].size.height;
            if (width > length) {
                std::swap(width, length);
            }
            switch (children[i].size()) {  //筛选中心R标志符
                case 0: {
                    // if (!InRange(length / width, CENTER_RATIO_MIN, CENTER_RATIO_MAX)) {
                    //     continue;
                    // }
                    possible_symbols.emplace_back(i);
                } break;
                case 1: {                                                                          //筛选叶片
                    if (!InRange(length / width, LEAF_ASPECT_RATIO_MIN, LEAF_ASPECT_RATIO_MAX)) {  //用长宽比剪枝
                        continue;
                    }
                    auto width_armor = rects[children[i][0]].size.width,
                         length_armor = rects[children[i][0]].size.height;
                    if (width_armor > length_armor) {
                        std::swap(width_armor, length_armor);
                    }
                    if (!InRange(length_armor / width_armor, ARMOR_ASPECT_RATIO_MIN, ARMOR_ASPECT_RATIO_MAX)) {  //按长宽比筛选出叶片
                        continue;
                    }
                    if (!InRange((length_armor * width_armor) / (length * width), AREA_RATIO_MIN, AREA_RATIO_MAX)) {  //按子轮廓与父轮廓面积比筛选
                        continue;
                    }
                    // Log::Info("length_armor/width_armor = {}",length_armor / width_armor);
                    // Log::Info("length_armor * width_armor)/length*width = {}",(length_armor * width_armor) / (length * width));
                    // Log::Info("area_aspect_ratio({},{})",ARMOR_ASPECT_RATIO_MIN, ARMOR_ASPECT_RATIO_MAX);
                    // Log::Info("area_ratio({},{})",AREA_RATIO_MIN, AREA_RATIO_MAX);
                    possible_leaves.emplace_back(i);
                } break;
            }
        }
#endif
        return true;
    }
#if NEW_RUNE

    bool Rune::Select() {
        auto&& outer_lightbar_size = possible_outer_lightbar.size();
        auto&& inside_lightbar_size = possible_inside_lightbar.size();

        if ((outer_lightbar_size && inside_lightbar_size) == 0) {
            return false;
        }

        std::map<double, std::pair<int, int>> lightbar_match;
        std::vector<double> lightbar_dis;

        for (int i = 0; i < outer_lightbar_size; i++)  //匹配最近可能灯条(可能需要优化)
        {
            // if(possible_outer_lightbar[i] == ring[i])
            // {
            //     continue;
            // }

            if (layer_bool) {
                // cv::circle(data->dst,Center(contours[layer1]),(rects[layer1].size.width + rects[layer1].size.height) / 4,Colors::Green,3);
            }

            // if(layer_bool && (Distance(Center(contours[possible_outer_lightbar[i]]),Center(contours[layer1])) < (rects[layer1].size.width + rects[layer1].size.height) / 4))
            // {
            //     // cv::circle(data->dst,Center(contours[possible_outer_lightbar[i]]),11,Colors::White,-1);
            //     // cv::circle(data->dst,Center(contours[layer1]),11,Colors::White,-1);

            //     continue;
            // }

            auto&& lightbar = Center(contours[possible_outer_lightbar[i]]);

            auto&& width_outer = rects[possible_outer_lightbar[i]].size.width;
            auto&& length_outer = rects[possible_outer_lightbar[i]].size.height;

            for (int j = 0; j < inside_lightbar_size; j++) {
                cv::circle(data->dst, Center(contours[possible_outer_lightbar[i]]), 11, Colors::Blue, -1);
                cv::circle(data->dst, Center(contours[possible_inside_lightbar[j]]), 11, Colors::Green, -1);

                // if(possible_inside_lightbar[j] == ring[j] || possible_inside_lightbar[j] == possible_outer_lightbar[i])     //排除环数轮廓和与外灯条轮廓，筛选出内灯条
                if (possible_inside_lightbar[j] == possible_outer_lightbar[i])  //排除环数轮廓和与外灯条轮廓，筛选出内灯条
                {
                    continue;
                }
                // if(layer_bool && (Distance(Center(contours[possible_inside_lightbar[j]]),Center(contours[layer1])) < (rects[layer1].size.width + rects[layer1].size.height) / 4))   //排除引导标识内的所有轮廓
                // {
                //     Log::Info("continue !");
                //     continue;
                // }
                auto&& width_inside = rects[possible_inside_lightbar[j]].size.width;
                auto&& length_inside = rects[possible_inside_lightbar[j]].size.height;

                if (!InRange((width_outer * length_outer) / (width_inside * length_inside), RUNE_LIGHTBAR_OUTER_INSIDE_RATIO_MIN, RUNE_LIGHTBAR_OUTER_INSIDE_RATIO_MAX))  //两个灯条的面积比来筛选
                {
                    continue;
                }

                auto&& match_one = Center(contours[possible_inside_lightbar[j]]);
                double dis = Distance(lightbar, match_one);
                lightbar_dis.push_back(dis);
                lightbar_match[dis] = std::pair<int, int>(i, j);
            }
        }
        if (lightbar_dis.size() == 0) {
            return false;
        }
        std::sort(lightbar_dis.begin(), lightbar_dis.end());
        std::pair<int, int>& lightbar_pair = lightbar_match[lightbar_dis[0]];

        int lightbar1_index = possible_outer_lightbar[lightbar_pair.first];
        int lightbar2_index = possible_inside_lightbar[lightbar_pair.second];

        if (lightbar1_index == lightbar2_index) {
            return false;
        }

        DrawContour(data->dst, contours[lightbar1_index], Colors::Blue, 4);  //相匹配的两个灯条用黄色画出
        DrawContour(data->dst, contours[lightbar2_index], Colors::White, 4);

        auto&& symbol_size = possible_symbols.size();

        auto&& possible_lightbar_area1 = contours[lightbar1_index];
        auto&& possible_lightbar_area2 = contours[lightbar2_index];

        /**
         * @brief 匹配两个RotatedRect四个最接近的点的函数
         * @param r1 旋转矩形1
         * @param r2 旋转矩形2
        */
        auto&& get_vertices = [](const cv::RotatedRect& r1, const cv::RotatedRect& r2) {
            cv::Point2f p1_array[4];
            cv::Point2f p2_array[4];
            r1.points(p1_array);
            r2.points(p2_array);
            std::vector<float> dis;
            std::map<float, std::pair<int, int>> index;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    auto&& d = Distance(p1_array[i], p2_array[j]);
                    dis.push_back(d);
                    index[d] = std::pair<int, int>(i, j);
                }
            }
            std::sort(dis.begin(), dis.end());
            std::vector<cv::Point2d> vertices;
            vertices = {p1_array[index[dis[0]].first], p1_array[index[dis[1]].first], p2_array[index[dis[0]].second], p2_array[index[dis[1]].second]};
            return vertices;
        };

        auto&& vertices = get_vertices(rects[lightbar1_index], rects[lightbar2_index]);  //得到灯条用pnp测距的角点
        for (auto v : vertices) {
            cv::circle(data->dst, v, 11, Colors::Blue, -1);
        }
        // cv::waitKey(100);
        /**
         * @brief 用于计算两个向量夹角的cos值的函数asd
         * @param vctl 向量l
         * @param vctr 向量r
        */
        auto&& vec_angle = [](const cv::Point2f& vctl, const cv::Point2f& vctr) {
            return fabs(vctl.ddot(vctr)) / (norm(vctl) * norm(vctr));
        };

        cv::circle(data->dst, Center(contours[layer1]), 5, Colors::White, -1);

        for (int i = 0; i < symbol_size; i++)  //匹配标识符
        {
            DrawContour(data->dst, contours[possible_symbols[i]], Colors::Purple, 4);

            // if(possible_symbols[i] == ring[i])
            // {
            //     continue;
            // }

            if (layer_bool && (Distance(Center(contours[possible_symbols[i]]), Center(contours[layer1])) < (rects[layer1].size.width + rects[layer1].size.height) / 4)) {
                continue;
            }

            if (possible_symbols[i] == lightbar1_index || possible_symbols[i] == lightbar2_index) {
                continue;
            }

            auto&& symbol_center = Center(contours[possible_symbols[i]]);
            double dis1 = Distance(symbol_center, Center(contours[lightbar1_index]));
            double dis2 = Distance(symbol_center, Center(contours[lightbar2_index]));

            auto&& armor_center = Center(contours[layer1]);

            auto&& vec1 = (vertices[0] + vertices[1]) / 2 - symbol_center;
            auto&& vec2 = (vertices[2] + vertices[3]) / 2 - symbol_center;
            auto&& vec3 = armor_center - symbol_center;

            auto&& lightbar_vec = (vertices[0] - vertices[1]);

            float cos_threshold = 0.2;

            if (vec_angle(vec1, lightbar_vec) > cos_threshold) {
                return false;
            }

            cv::line(data->dst, symbol_center, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, Colors::White, 5);

            if (vec_angle(vec2, lightbar_vec) > cos_threshold) {
                return false;
            }
            // cv::line(data->dst,symbol_center,(vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4,Colors::Blue,5);

            if (vec_angle(vec3, lightbar_vec) > cos_threshold) {
                return false;
            }

            cv::line(data->dst, symbol_center, (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4, Colors::Blue, 5);

            if (InRange(dis1 / dis2, RUNE_ARMOR_LIGHTBAR2CENTER_RATIO_MIN, RUNE_ARMOR_LIGHTBAR2CENTER_RATIO_MAX)) {
                outer_lightbar = lightbar1_index;
                inside_lightbar = lightbar2_index;
                symbol = i;
            } else if (InRange(dis2 / dis1, RUNE_ARMOR_LIGHTBAR2CENTER_RATIO_MIN, RUNE_ARMOR_LIGHTBAR2CENTER_RATIO_MAX)) {
                inside_lightbar = lightbar1_index;
                outer_lightbar = lightbar2_index;
                symbol = i;
            } else {
                Log::Info("log out");
                continue;
            }

            DrawContour(data->dst, contours[possible_symbols[i]], Colors::LimeGreen, 4);

            // DrawContour(data->dst,contours[possible_symbols[symbol]],Colors::Yellow);
            if (Distance(symbol_center, armor_center) != 0)
                cv::circle(data->dst, symbol_center, Distance(symbol_center, armor_center), Colors::White, 5);
            cv::circle(data->dst, armor_center, 10, Colors::White, -1);
            cv::circle(data->dst, armor_center, 7, Colors::Chocolate, -1);
            cv::circle(data->dst, symbol_center, 5, Colors::CadetBlue, -1);
            cv::line(data->dst, armor_center, symbol_center, Colors::Aqua, 5);  //用白线将标识符与装甲板中心链接
            // data->armor = armor_center;
            // data->symbol = symbol_center;
            // runetracker.vertices = vertices;
            return data->find = true;
        }
        return data->find = false;
    }

#else
    bool Rune::Select() {
        if (possible_leaves.empty()) {
            return false;
        }
        auto leaf_index = std::ranges::max(possible_leaves, {}, [this](auto&& index) {
            return areas[index];
        });
        auto armor_index = children[leaf_index][0];
        auto leaf_center = Center(contours[leaf_index]);
        auto armor_center = Center(contours[armor_index]);
        auto leaf_angle = Angle(armor_center - leaf_center);
        static const float angle_bound = 5_deg;
#if SHOW_DEBUG
        cv::line(data->dst, armor_center, armor_center - Polar(500, leaf_angle + angle_bound), Colors::Red, 2);
        cv::line(data->dst, armor_center, armor_center - Polar(500, leaf_angle - angle_bound), Colors::Red, 2);
#endif
        if (possible_symbols.empty()) {
            return false;
        }

        auto min_dist = std::numeric_limits<double>::max();
        auto symbol_center = cv::Point2f{-1, -1};
        for (std::size_t i = 0, n = possible_symbols.size(); i < n; ++i) {
            auto&& possible_center = Center(contours[possible_symbols[i]]);
#if SHOW_DEBUG
            cv::circle(data->dst, possible_center, 5, Colors::Lime, -1);
#endif
            auto&& possible_dir = leaf_center - possible_center;
            if (std::abs(Revise(Angle(possible_dir) - leaf_angle, -M_PI, M_PI)) > angle_bound) {
                continue;
            }
            if (auto&& distance = cv::norm(possible_dir); distance < min_dist) {
                min_dist = distance;
                symbol_center = possible_center;
            }
        }
        if (min_dist == std::numeric_limits<double>::max()) {
            return false;
        }
        auto&& middle_point = (armor_center + symbol_center) / 2;
        cv::Point2f points[4];
        auto&& leaf_rect = rects[leaf_index];
        auto&& leaf_width = std::min(leaf_rect.size.width, leaf_rect.size.height);
        auto&& armor_rect = rects[armor_index];
        auto&& armor_length = std::max(armor_rect.size.width, armor_rect.size.height);
        auto&& compensate = (leaf_width - armor_length) / 2;
        armor_rect.size.width += compensate;
        armor_rect.size.height += compensate;
        rects[armor_index].points(points);
        std::sort(points, points + 4, [&](auto&& lhs, auto&& rhs) {
            return Revise(Angle(lhs - middle_point) - Angle(rhs - middle_point), -M_PI, M_PI) < 0;
        });

#if SHOW_DEBUG
        for (std::size_t i = 0; i < 4; ++i) {
            cv::line(dst, points[i], symbol_center, Colors::Orange, 2);
            cv::circle(data->dst, points[i], 5, Colors::Red, -1);
            Fonts::CascadiaCode->PutText(data->dst, std::to_string(i), points[i], 5, Colors::White);
            cv::putText(data->dst, std::to_string(i), points[i], cv::FONT_HERSHEY_SIMPLEX, 1, Colors::White, 2);
        }
        cv::circle(data->dst, leaf_center, 5, Colors::Lime, -1);
        cv::circle(data->dst, armor_center, 5, Colors::Lime, -1);
        cv::circle(data->dst, symbol_center, 5, Colors::Lime, -1);
#endif
        data->armor = armor_center;
        data->symbol = symbol_center;
        runetracker.vertices = std::vector<cv::Point2d>(points, points + 4);
        // Log::Info(runetracker.vertices);
        return data->find = false;
        std::cout << "end Select" << std::endl;
    }
#endif
    bool Rune::Closeout() {
        return true;
    }
}  // namespace phoenix::detector

#undef SHOW_DEBUG
#undef LOG_DEBUG
#undef LOG_TIME
#undef NEW_RUNE