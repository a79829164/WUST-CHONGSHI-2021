#include <ros/ros.h>
#include <ros/package.h>
#include <thor/vis.h>
#include <iostream>
#include <NvInfer.h>
#include <opencv2/opencv.hpp>
#include <NvOnnxParser.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <numeric>
#include <string>
#include <librealsense2/rs.hpp>
#include <roborts_msgs/armor_detecte_result.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

float score_threshold = 0.5;

inline uint32_t getElementSize(nvinfer1::DataType t) noexcept {
    switch (t) {
        case nvinfer1::DataType::kINT32:
            return 4;
        case nvinfer1::DataType::kFLOAT:
            return 4;
        case nvinfer1::DataType::kHALF:
            return 2;
        case nvinfer1::DataType::kBOOL:
        case nvinfer1::DataType::kINT8:
            return 1;
    }
    return 0;
}

inline int64_t volume(const nvinfer1::Dims& d)
{
    return std::accumulate(d.d, d.d + d.nbDims, 1, std::multiplies<int64_t>());
}


using namespace nvinfer1;


class Logger : public ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        // suppress info-level messages
        if (severity != Severity::kINFO) {
            ROS_DEBUG("%s", msg);
        }

    }
} gLogger;



int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_inference");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher test_frame_pub = it.advertise("/test_frame", 1);

    auto pipeline = rs2::pipeline();
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    
    rs2::pipeline_profile profile = pipeline.start(config);

    rs2::align align_to_depth(RS2_STREAM_DEPTH);

    const auto intrinDepth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    const auto intrinColor = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();


    std::string onnx_file_name = ros::package::getPath("yolo_inference") + "/model/yolov5s.onnx";
    std::string names_file_name = ros::package::getPath("yolo_inference") + "/model/names.txt";
    std::string trt_file_name = ros::package::getPath("yolo_inference") + "/model/yolov5s_engine.trt";

    // Read class names.
    std::vector<std::string> names;
    std::ifstream names_file(names_file_name);
    if (names_file.good()) {
        std::string line;
        while (std::getline(names_file, line)) {
            std::cout << "Read: " << line << std::endl;
            names.push_back(line);
        }
        std::cout << "Read " << names.size() << " classes." << std::endl;
    } else {
        std::cout << "Could not found names.txt" << std::endl;
        return -1;
    }

    // Parser onnx network.

    // Create the builder and network.
    IBuilder* builder = createInferBuilder(gLogger);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition* network = builder->createNetworkV2(explicitBatch);

    // Create the ONNX parser:
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gLogger);

    // Parse the model:
    parser->parseFromFile(onnx_file_name.c_str(), static_cast<int>(ILogger::Severity::kWARNING));
    for (int i = 0; i < parser->getNbErrors(); ++i)
    {
        std::cout << parser->getError(i)->desc() << std::endl;
    }


    // Create engine.
    ICudaEngine* engine;
    std::ifstream trtFile(trt_file_name);
    if (trtFile.good()) {
        std::cout << "Found engine file." << std::endl;

        // Deserialize engine
        std::stringstream modelData;
        modelData.seekg(0, std::ios::beg);
        modelData << trtFile.rdbuf();
        modelData.seekg(0, std::ios::end);
        const size_t modelSize = modelData.tellg();
        modelData.seekg(0, std::ios::beg);
        void* modelMem = malloc(modelSize);
        modelData.read((char*)modelMem, modelSize);
        IRuntime* runtime = createInferRuntime(gLogger);
        engine = runtime->deserializeCudaEngine(modelMem, modelSize, nullptr);
        free(modelMem);
        runtime->destroy();
        trtFile.close();
    } else {
        std::cout << "Engine file not found. Creating..." << std::endl;

        // Create engine and serialize it.
        trtFile.close();
        std::ofstream trtFileO(trt_file_name);

        // Build the engine using the builder object:
        IBuilderConfig* config = builder->createBuilderConfig();
        config->setMaxWorkspaceSize(1 << 20);
        //config->setFlag(nvinfer1::BuilderFlag::kINT8);
        engine = builder->buildEngineWithConfig(*network, *config);

        // Serialize
        IHostMemory *serializedModel = engine->serialize();
        trtFileO.write(reinterpret_cast<const char*>(serializedModel->data()), serializedModel->size());
        trtFileO.close();
        serializedModel->destroy();
        config->destroy();
    }




    // Create some space to store intermediate activation values. Since the engine holds the network
    // definition and trained parameters, additional space is necessary. These are held in an execution context:
    IExecutionContext *context = engine->createExecutionContext();


    const char* input_blob_name = network->getInput(0)->getName();
    const char* output_blob_name = network->getOutput(0)->getName();
    printf("input_blob_name : %s \n", input_blob_name);
    printf("output_blob_name : %s \n", output_blob_name);

    const int batchSize = network->getInput(0)->getDimensions().d[0];
    const int inputC = network->getInput(0)->getDimensions().d[1];
    const int inputH = network->getInput(0)->getDimensions().d[2];
    const int inputW = network->getInput(0)->getDimensions().d[3];
    const Dims outputShape = network->getOutput(0)->getDimensions();
    printf("batchSize: %d, inputC: %d, inputH : %d, inputW: %d\n", batchSize, inputC, inputH, inputW);

    printf("outputShape: (%d, %d, %d)\tdims: %d\n", outputShape.d[0], outputShape.d[1], outputShape.d[2], outputShape.nbDims);

    //Using these indices, set up a buffer array pointing to the input and output buffers on the GPU:

    std::vector<int64_t> bufferSize;
    int nbBindings = engine->getNbBindings();
    void **buffers = (void **) malloc((nbBindings) * sizeof(void *));
    bufferSize.resize(nbBindings);

    for (int i = 0; i < nbBindings; ++i) {
        Dims dims = engine->getBindingDimensions(i);
        nvinfer1::DataType dtype = engine->getBindingDataType(i);
        int64_t totalSize = volume(dims) * 1 * getElementSize(dtype);
        bufferSize[i] = totalSize;
        std::cout << "Binding layer " << engine->getBindingName(i) << "(" << i << "): " << totalSize << std::endl;
        cudaMalloc(&buffers[i], totalSize);
    }

    // Create cuda stream.
    cudaStream_t stream;
    cudaStreamCreate(&stream);


    rs2::frameset frameset;
    
    cv::Mat frame;
    auto t0 = std::chrono::high_resolution_clock::now();
    while (true) {
        // Read image.
        frameset = pipeline.wait_for_frames();
        rs2::frame color = frameset.get_color_frame();
        frame = cv::Mat(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        // Resize.
        float ratio = float(inputW) / float(frame.cols) < float(inputH) / float(frame.rows) ? float(inputW) / float(frame.cols) : float(inputH) / float(frame.rows);
        cv::Mat flt_img = cv::Mat::zeros(cv::Size(inputW, inputH), CV_8UC3);
        cv::Mat rsz_img;
        cv::resize(frame, rsz_img, cv::Size(), ratio, ratio);
        rsz_img.copyTo(flt_img(cv::Rect(0, 0, rsz_img.cols, rsz_img.rows)));

        cv::Mat blob;
        cv::dnn::blobFromImage(flt_img, blob, 1.0/255, cv::Size(inputW, inputH), cv::Scalar(), true, false, CV_32F);


        // DMA the input to the GPU,  execute the batch asynchronously, and DMA it back:
        cudaMemcpyAsync(buffers[0], blob.data, bufferSize[0], cudaMemcpyHostToDevice, stream);


        // Do inference
        auto st = std::chrono::high_resolution_clock::now();
        context->executeV2(buffers);
        std::cout << "Inference time: " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - st).count() * 1000 << "ms" << std::endl;



        // Get result.
        const int sz[] = {1, 25200, 85};
        cv::Mat inferResult(3, sz, CV_32F);
        cudaMemcpyAsync(inferResult.data, buffers[4], bufferSize[4], cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);



        std::vector<cv::RotatedRect> bboxes;
        std::vector<float> scores;
        std::vector<int> classes;


        for (int i = 0; i < 25200; ++i) {
            float score = inferResult.at<float>(0, i, 4);
            if (score > score_threshold) {
                cv::RotatedRect bbox;
                bbox.center.x = inferResult.at<float>(0, i, 0) / ratio;
                bbox.center.y = inferResult.at<float>(0, i, 1) / ratio;
                bbox.size.width = inferResult.at<float>(0, i, 2) / ratio;
                bbox.size.height = inferResult.at<float>(0, i, 3) / ratio;

                int cls = 0;
                float max_temp = 0;
                for (int j = 5; j < 85; ++j) {
                    float max_val = inferResult.at<float>(0, i, j);
                    if (max_val > max_temp) {
                        max_temp = max_val;
                        cls = j - 5;
                    }
                }

                bboxes.push_back(bbox);
                scores.push_back(score);
                classes.push_back(cls);
            }
        }


        // NMS
        std::vector<int> nmsIndices;
        cv::dnn::NMSBoxes(bboxes, scores, score_threshold, 0.5, nmsIndices);


        // thor visualization
        std::vector<thor::Box> all_detections;
        for (const auto &nmsIndex : nmsIndices) {
            thor::Box one_box{int(bboxes[nmsIndex].center.x - bboxes[nmsIndex].size.width / 2),
                              int(bboxes[nmsIndex].center.y - bboxes[nmsIndex].size.height / 2),
                              int(bboxes[nmsIndex].center.x + bboxes[nmsIndex].size.width / 2),
                              int(bboxes[nmsIndex].center.y + bboxes[nmsIndex].size.height / 2),
                              thor::BoxFormat::XYXY};
            one_box.score = scores[nmsIndex];
            one_box.idx = classes[nmsIndex];
            all_detections.emplace_back(one_box);
        }
        auto res_image = thor::vis::VisualizeDetectionStyleDetectron2(frame, all_detections, names);
        
        
        sensor_msgs::ImagePtr smsg;
        smsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res_image).toImageMsg();
        test_frame_pub.publish(smsg);


        // FPS
        auto t1 = std::chrono::high_resolution_clock::now();
        auto dt = std::chrono::duration<double>(t1 - t0).count();
        std::cout << "FPS: " << 1 / dt << std::endl;
        t0 = t1;

    }



    // release the stream and the buffers
    cudaStreamDestroy(stream);
    cudaFree(buffers[0]);
    cudaFree(buffers[1]);

    // Dispense with the network, builder, and parser if using one.

    context->destroy();
    engine->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
    return 0;

}