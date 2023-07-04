#pragma once
#include "argsParser.h"
#include "buffers.h"
#include "common.h"
#include "logger.h"
#include "parserOnnxConfig.h"

#include "NvInfer.h"
#include <cuda_runtime_api.h>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/timeb.h>

#include "constant.hpp"

using samplesCommon::SampleUniquePtr;


namespace samplesCommon {
	struct TrtSampleParams : public SampleParams
	{
		std::string onnxFileName;
		std::string trtFileName;
		nvinfer1::Dims inputDims;
		std::vector<nvinfer1::Dims> outputDims;
		std::vector<int*> link;
		std::vector<int*> circle;
		cv::Scalar color;
	};
}


//! \brief  The SampleOnnxMNIST class implements the ONNX MNIST sample
//!
//! \details It creates the network using an ONNX model or trt file
//!
class HumanPoseEstimator
{
public:
	HumanPoseEstimator(const samplesCommon::TrtSampleParams& params)
		: mParams(params)
		, mEngine(nullptr)
		, mInputDims(params.inputDims)
		, mOutputDims(params.outputDims)
	{
	}
	~HumanPoseEstimator()
	{
		// delete p_context;
		// delete p_buffers;
	}
	samplesCommon::TrtSampleParams mParams; //!< The parameters for the sample.
	nvinfer1::Dims mInputDims;  //!< The dimensions of the input to the network.
	std::vector<nvinfer1::Dims> mOutputDims; //!< The dimensions of the outputs to the network.
	std::shared_ptr<nvinfer1::ICudaEngine> mEngine; //!< The TensorRT engine used to run the network

	//!
	//! \brief 从trt文件加载模型
	//!
	bool load();

	//!
	//! \brief Function builds the network engine
	//!
	bool build();

	//!
	//! \brief Runs the TensorRT inference engine for this sample
	//!
	bool infer(SampleUniquePtr<nvinfer1::IExecutionContext>& context,
		samplesCommon::BufferManager& buffers,
		cv::Mat img, std::vector<cv::Point2f> &joints, std::vector<double> &maxval);

	//!
	//! \brief 显示命令
	//!
	//int display(cv::Mat img, std::vector<cv::Point2f> &joints, std::vector<double> &maxval, double fps, int intent, int gait);
	int display(cv::Mat img, std::vector<cv::Point2f> &joints, std::vector<double> &maxval, double fps, float * peo);

private:
	// samplesCommon::BufferManager *p_buffers;
	// nvinfer1::IExecutionContext *p_context;

	//!
	//! \brief Parses an ONNX model for MNIST and creates a TensorRT network
	//!
	bool constructNetwork(SampleUniquePtr<nvinfer1::IBuilder>& builder,
		SampleUniquePtr<nvinfer1::INetworkDefinition>& network, SampleUniquePtr<nvinfer1::IBuilderConfig>& config,
		SampleUniquePtr<nvonnxparser::IParser>& parser);

	//!
	//! \brief 预处理函数
	//!
	bool preprocess(cv::Mat img, const samplesCommon::BufferManager& buffers);

	//!
	//! \brief 后处理函数
	//!
	bool postprocess(const samplesCommon::BufferManager& buffers, std::vector<cv::Point2f> &joints, std::vector<double> &maxval);
};

//samplesCommon::TrtSampleParams initializeSampleParams(const samplesCommon::Args& args);
samplesCommon::TrtSampleParams initializeSampleParams();