#include "human_pose_estimator.hpp"

//!
//!  \读取TensorRT模型, 将其反序列化成一个ICudaEngine 实例
//!
bool HumanPoseEstimator::load()
{
	std::ifstream fin(locateFile(mParams.trtFileName, mParams.dataDirs).c_str(), std::ios::in | std::ios::binary);	
	if (!fin.is_open())
	{
		return false;
	}

	fin.seekg(0, std::ios::end);
	size_t length = fin.tellg();
	std::vector<char> data;
	if (length > 0)
	{
		fin.seekg(0, std::ios::beg);
		data.resize(length);
		fin.read((char*)&data[0], length);
	}
	fin.close();

	SampleUniquePtr<IRuntime> runtime{ createInferRuntime(sample::gLogger.getTRTLogger()) };
	mEngine = std::shared_ptr<nvinfer1::ICudaEngine>(
		runtime->deserializeCudaEngine(data.data(), data.size()), samplesCommon::InferDeleter());

	return true;
}


//!
//! \brief Creates the network, configures the builder and creates the network engine
//!
//! \details This function creates the Onnx MNIST network by parsing the Onnx model and builds
//!          the engine that will be used to run MNIST (mEngine)
//!
//! \return true if the engine was created successfully and false otherwise
//!
bool HumanPoseEstimator::build()
{
	auto builder = SampleUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(sample::gLogger.getTRTLogger()));
	if (!builder)
	{
		return false;
	}

	const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
	auto network = SampleUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
	if (!network)
	{
		return false;
	}

	auto config = SampleUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
	if (!config)
	{
		return false;
	}

	auto parser
		= SampleUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, sample::gLogger.getTRTLogger()));
	if (!parser)
	{
		return false;
	}

	auto constructed = constructNetwork(builder, network, config, parser);  // 构造网络
	if (!constructed)
	{
		return false;
	}

	// CUDA stream used for profiling by the builder.
	auto profileStream = samplesCommon::makeCudaStream();
	if (!profileStream)
	{
		return false;
	}
	config->setProfileStream(*profileStream);

	SampleUniquePtr<IHostMemory> plan{ builder->buildSerializedNetwork(*network, *config) };
	if (!plan)
	{
		return false;
	}

	SampleUniquePtr<IRuntime> runtime{ createInferRuntime(sample::gLogger.getTRTLogger()) };
	if (!runtime)
	{
		return false;
	}

	mEngine = std::shared_ptr<nvinfer1::ICudaEngine>(
		runtime->deserializeCudaEngine(plan->data(), plan->size()), samplesCommon::InferDeleter());
	if (!mEngine)
	{
		return false;
	}

	ASSERT(network->getNbInputs() == 1);
	mInputDims = network->getInput(0)->getDimensions();
	ASSERT(mInputDims.nbDims == 4);

	ASSERT(network->getNbOutputs() == 3);
	mOutputDims.push_back(network->getOutput(0)->getDimensions());
	ASSERT(mOutputDims[0].nbDims == 3);
	mOutputDims.push_back(network->getOutput(1)->getDimensions());
	ASSERT(mOutputDims[1].nbDims == 3);
	mOutputDims.push_back(network->getOutput(2)->getDimensions());
	ASSERT(mOutputDims[2].nbDims == 3);

	// p_buffers = new samplesCommon::BufferManager(mEngine);

	// p_context = mEngine->createExecutionContext();
	// if (!p_context)
	// {
		// return false;
	// }

	return true;
}

//!
//! \brief Uses a ONNX parser to create the Onnx MNIST Network and marks the
//!        output layers
//!
//! \param network Pointer to the network that will be populated with the Onnx MNIST network
//!
//! \param builder Pointer to the engine builder
//!
bool HumanPoseEstimator::constructNetwork(SampleUniquePtr<nvinfer1::IBuilder>& builder,
	SampleUniquePtr<nvinfer1::INetworkDefinition>& network, SampleUniquePtr<nvinfer1::IBuilderConfig>& config,
	SampleUniquePtr<nvonnxparser::IParser>& parser)
{
	auto parsed = parser->parseFromFile(locateFile(mParams.trtFileName, mParams.dataDirs).c_str(),
		static_cast<int>(sample::gLogger.getReportableSeverity()));  // 解析后保存到何处？network？
	if (!parsed)
	{
		return false;
	}

	if (mParams.fp16)
	{
		config->setFlag(BuilderFlag::kFP16);
		std::cout << "Using Float16" << std::endl;
	}
	if (mParams.int8)
	{
		config->setFlag(BuilderFlag::kINT8);
		samplesCommon::setAllDynamicRanges(network.get(), 127.0f, 127.0f);
		std::cout << "Using Int8" << std::endl;
	}

	samplesCommon::enableDLA(builder.get(), config.get(), mParams.dlaCore);

	return true;
}

//!
//! \brief Runs the TensorRT inference engine
//!
//! \details This function is the main execution function. It allocates the buffer,
//!          sets inputs and executes the engine.
//!
bool HumanPoseEstimator::infer(SampleUniquePtr<nvinfer1::IExecutionContext>& context,
	samplesCommon::BufferManager& buffers,
 	cv::Mat img, std::vector<cv::Point2f> &joints, std::vector<double> &maxval)
{
	// Create RAII buffer manager object
	// samplesCommon::BufferManager buffers(mEngine);

	//auto context = SampleUniquePtr<nvinfer1::IExecutionContext>(mEngine->createExecutionContext());
	// if (!context)
	// {
	//	   return false;
	// }

	// 预处理
	ASSERT(mParams.inputTensorNames.size() == 1);
	if (!preprocess(img, buffers))
	{
		return false;
	}

	// Memcpy from host input buffers to device input buffers
	buffers.copyInputToDevice();

	bool status = context->executeV2(buffers.getDeviceBindings().data());
	if (!status)
	{
		return false;
	}

	// Memcpy from device output buffers to host output buffers
	buffers.copyOutputToHost();

	// 后处理
	if (!postprocess(buffers, joints, maxval))
	{
		return false;
	}

	return true;
}

//!
//! \brief Reads the input and stores the result in a managed buffer
//!
bool HumanPoseEstimator::preprocess(cv::Mat img, const samplesCommon::BufferManager& buffers)
{
	int inputH = mInputDims.d[2];
	int inputW = mInputDims.d[3];
	cv::Size resize_shape(inputW, inputH);

	cv::Mat imgbgr(inputH, inputW, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::resize(img, imgbgr, resize_shape, 0.0, 0.0, cv::INTER_CUBIC);

	cv::Mat imgf(inputH, inputW, CV_32FC3, cv::Scalar(0, 0, 0));
	cv::Mat mean(3, 1, CV_64F, cv::Scalar(0));
	cv::Mat std(3, 1, CV_64F, cv::Scalar(0));
	cv::meanStdDev(imgbgr, mean, std);
	cv::max(std, cv::Mat(3, 1, CV_64F, cv::Scalar(1e-4)), std);
	cv::Vec3f mean_tar(0.485, 0.456, 0.406);
	cv::Vec3f std_tar(0.229, 0.224, 0.225);

	// std::cout << cv::format(mean, cv::Formatter::FMT_C) << std::endl;
	// std::cout << cv::format(std, cv::Formatter::FMT_C) << std::endl;

	cv::add(imgbgr, cv::Mat(inputH, inputW, CV_32FC3, -mean.at<cv::Vec3d>(0, 0)), imgf, cv::noArray(), CV_32FC3);
	cv::divide(imgf, cv::Mat(inputH, inputW, CV_32FC3, std.at<cv::Vec3d>(0, 0)), imgf, 1, CV_32FC3);
	cv::multiply(imgf, cv::Mat(inputH, inputW, CV_32FC3, std_tar), imgf, 1, CV_32FC3);
	imgf += cv::Mat(inputH, inputW, CV_32FC3, mean_tar);

	float* hostDataBuffer = static_cast<float*>(buffers.getHostBuffer(mParams.inputTensorNames[0]));
	std::vector<cv::Mat> channels;
	cv::split(imgf, channels);
	for (int i = 0; i < 3; i++)
	{
		std::memcpy(hostDataBuffer + i * imgf.total(), channels[i].ptr<float>(0), imgf.total() * sizeof(float));
	}

	return true;
}

//!
//! \brief Classifies digits and verify result
//!
//! \return whether the classification output matches expectations
//!
bool HumanPoseEstimator::postprocess(const samplesCommon::BufferManager& buffers, std::vector<cv::Point2f> &joints, std::vector<double> &maxval)
{
	const int inputH = mInputDims.d[2];
	const int inputW = mInputDims.d[3];
	
	// 读取关节点
	std::vector<std::string>::iterator iter = std::find(mParams.outputTensorNames.begin(),
		mParams.outputTensorNames.end(), "pred");
	int ind = iter - mParams.outputTensorNames.begin();
	const int nJoints = mOutputDims[ind].d[1];
	float* output = static_cast<float*>(buffers.getHostBuffer(mParams.outputTensorNames[ind]));
	for (int i = 0; i < nJoints; i++)
	{
		joints[i].x = output[i * 2] ;
		joints[i].y = output[i * 2 + 1];
	}

	// 读取置信度
	iter = std::find(mParams.outputTensorNames.begin(), mParams.outputTensorNames.end(), "maxval");
	ind = iter - mParams.outputTensorNames.begin();
	output = static_cast<float*>(buffers.getHostBuffer(mParams.outputTensorNames[ind]));
	for (int i = 0; i < nJoints; i++)
	{
		maxval[i] = output[i];
	}

	return true;
}

// int HumanPoseEstimator::display(cv::Mat img, std::vector<cv::Point2f> &joints, std::vector<double> &maxval, double fps, int intent, int gait)
// {
// 	const int inputH = mInputDims.d[2];
// 	const int inputW = mInputDims.d[3];
// 	std::vector<std::string>::iterator iter = std::find(mParams.outputTensorNames.begin(),
// 		mParams.outputTensorNames.end(), "pred");
// 	int ind = iter - mParams.outputTensorNames.begin();
// 	const int nJoints = mOutputDims[ind].d[1];
// 	double thres = CONFIDENCE_THRES;

// 	cv::Point2d *joints_disp = new cv::Point2d[nJoints]();
// 	for (int i = 0; i < nJoints; i++)
// 	{
// 		joints_disp[i].x = double(joints[i].x) * img.cols;
// 		joints_disp[i].y = double(joints[i].y) * img.rows;
// 	}
// 	for (int i = 0; i < nJoints; i++)
// 	{
// 		if (maxval[i] > thres)
// 		{
// 			cv::circle(img, joints_disp[i], 3, mParams.color, 1);
// 			cv::putText(img, std::to_string(i), joints_disp[i], cv::FONT_HERSHEY_PLAIN, 1.3, mParams.color, 1);
// 		}
// 	}
// 	for (int i = 0; i < mParams.link.size(); i++)
// 	{
// 		if (maxval[mParams.link[i][0]] > thres && maxval[mParams.link[i][1]] > thres)
// 			cv::line(img, joints_disp[mParams.link[i][0]], joints_disp[mParams.link[i][1]], mParams.color, 1);
// 	}
// 	for (int i = 0; i < mParams.circle.size(); i++)
// 	{
// 		if (maxval[mParams.circle[i][0]] > thres && maxval[mParams.circle[i][1]] > thres)
// 		{
// 			cv::Point2d p1 = joints_disp[mParams.circle[i][0]];
// 			cv::Point2d p2 = joints_disp[mParams.circle[i][1]];
// 			double r = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2)) / 2;
// 			cv::circle(img, (p1 + p2) / 2, (int)r, mParams.color, 1);
// 		}
// 	}
// 	cv::putText(img, "FPS: " + std::to_string(fps), cv::Point(0, 20), cv::FONT_HERSHEY_PLAIN, 1.3, mParams.color, 1);
// 	cv::putText(img, intention_mode_inv[intent], cv::Point(0, 40), cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(255, 0, 255), 1);
// 	cv::putText(img, gait_phase_inv[gait], cv::Point(0, 60), cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(0, 255, 255), 1);

// 	cv::imshow("display", img);
// 	int key = cv::waitKey(5);
// 	return key;
// }

int HumanPoseEstimator::display(cv::Mat img, std::vector<cv::Point2f> &joints, std::vector<double> &maxval, double fps, float * peo)
{
	const int inputH = mInputDims.d[2];
	const int inputW = mInputDims.d[3];
	std::vector<std::string>::iterator iter = std::find(mParams.outputTensorNames.begin(),
		mParams.outputTensorNames.end(), "pred");
	int ind = iter - mParams.outputTensorNames.begin();
	const int nJoints = mOutputDims[ind].d[1];
	double thres = CONFIDENCE_THRES;

	cv::Point2d *joints_disp = new cv::Point2d[nJoints]();
	for (int i = 0; i < nJoints; i++)
	{
		joints_disp[i].x = double(joints[i].x) * img.cols;
		joints_disp[i].y = double(joints[i].y) * img.rows;
	}
	for (int i = 0; i < nJoints; i++)
	{
		if (maxval[i] > thres)
		{
			cv::circle(img, joints_disp[i], 3, mParams.color, 1);
			cv::putText(img, std::to_string(i), joints_disp[i], cv::FONT_HERSHEY_PLAIN, 1.3, mParams.color, 1);
		}
	}
	for (int i = 0; i < mParams.link.size(); i++)
	{
		if (maxval[mParams.link[i][0]] > thres && maxval[mParams.link[i][1]] > thres)
			cv::line(img, joints_disp[mParams.link[i][0]], joints_disp[mParams.link[i][1]], mParams.color, 1);
	}
	for (int i = 0; i < mParams.circle.size(); i++)
	{
		if (maxval[mParams.circle[i][0]] > thres && maxval[mParams.circle[i][1]] > thres)
		{
			cv::Point2d p1 = joints_disp[mParams.circle[i][0]];
			cv::Point2d p2 = joints_disp[mParams.circle[i][1]];
			double r = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2)) / 2;
			cv::circle(img, (p1 + p2) / 2, (int)r, mParams.color, 1);
		}
	}
	cv::putText(img, "FPS: " + std::to_string(fps), cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 0, 255), 1);
    cv::putText(img, "x=" + std::to_string((int)(peo[0]*1000)) , cv::Point(0, 25), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 0, 255), 1);
    cv::putText(img, "y=" + std::to_string((int)(peo[1]*1000)) , cv::Point(0, 40), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 0, 255), 1);
    cv::putText(img, "z=" + std::to_string((int)(peo[2]/M_PI*180)) , cv::Point(0, 55), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 0, 255), 1);

	cv::imshow("display", img);
	int key = cv::waitKey(1);
	return key;
}

//!
//! \brief Initializes members of the params struct using the command line args
//!
// samplesCommon::TrtSampleParams initializeSampleParams(const samplesCommon::Args& args)
samplesCommon::TrtSampleParams initializeSampleParams()
{
	samplesCommon::TrtSampleParams params;
	params.dataDirs.push_back(MODEL_PATH);
	params.onnxFileName = "model_best192.onnx";//240
	// params.trtFileName = "model_best192.trt";//240
	params.trtFileName = "model_best192_new.trt";//240
	
	params.inputTensorNames.push_back("input");
	params.outputTensorNames.push_back("pred");
	params.outputTensorNames.push_back("sigma");
	params.outputTensorNames.push_back("maxval");
	params.inputDims.nbDims = 4;
	params.inputDims.d[0] = 1; params.inputDims.d[1] = 3; params.inputDims.d[2] = 192; params.inputDims.d[3] = 256;//params.inputDims.d[2] = 240; params.inputDims.d[3] = 320;
	params.outputDims.push_back(nvinfer1::Dims());
	params.outputDims[0].nbDims = 3;
	params.outputDims[0].d[0] = 1; params.outputDims[0].d[1] = 16; params.outputDims[0].d[2] = 2;
	params.outputDims.push_back(nvinfer1::Dims());
	params.outputDims[1].nbDims = 3;
	params.outputDims[1].d[0] = 1; params.outputDims[1].d[1] = 16; params.outputDims[1].d[2] = 2;
	params.outputDims.push_back(nvinfer1::Dims());
	params.outputDims[2].nbDims = 3;
	params.outputDims[2].d[0] = 1; params.outputDims[2].d[1] = 16; params.outputDims[2].d[2] = 1;
	params.dlaCore = 0;//nano上没有DLA加速硬件
	params.int8 = false;
	params.fp16 = true;
	
	params.link.push_back(new int[2]{ 0, 1 });
	params.link.push_back(new int[2]{ 1, 2 });
	params.link.push_back(new int[2]{ 2, 6 });
	params.link.push_back(new int[2]{ 3, 4 });
	params.link.push_back(new int[2]{ 3, 6 });
	params.link.push_back(new int[2]{ 4, 5 });
	params.link.push_back(new int[2]{ 6, 7 });
	params.link.push_back(new int[2]{ 7, 8 });
	params.link.push_back(new int[2]{ 7, 12 });
	params.link.push_back(new int[2]{ 7, 13 });
	params.link.push_back(new int[2]{ 10, 11 });
	params.link.push_back(new int[2]{ 11, 12 });
	params.link.push_back(new int[2]{ 13, 14 });
	params.link.push_back(new int[2]{ 14, 15 });

	params.circle.push_back(new int[2]{ 8, 9 });

	params.color[0] = 255; params.color[1] = 224; params.color[2] = 0;

	return params;
}