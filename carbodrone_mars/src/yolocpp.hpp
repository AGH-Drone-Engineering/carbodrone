#ifndef YOLOCPP_H
#define YOLOCPP_H

#include <vector>
#include <string>
#include <memory>

class YOLOCPP
{
public:
    struct Detection
    {
        int class_id;
        std::string class_name;
        float confidence;
        int x, y, w, h;
    };

    YOLOCPP(std::string model_path, int width, int height, std::vector<std::string> classes);

    ~YOLOCPP();

    std::vector<Detection> detect(const uint8_t *img, int width, int height, int channels);

    float confidence_threshold = 0.25;
    float score_threshold = 0.45;
    float nms_threshold = 0.50;
    bool letterbox = true;

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

#endif // YOLOCPP_H
