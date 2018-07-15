#include "F:/chrome-download/camport2-master/sample/common/common.hpp"


#include <string>

using namespace std;


#pragma comment(lib,"F:\\chrome-download\\camport2-master\\lib\\win\\hostapp\\x86\\tycam.lib")

static char buffer[1024 * 1024];
static bool fakeLock = false; // NOTE: fakeLock may lock failed

struct CallbackData {
	int             index;
	TY_DEV_HANDLE   hDevice;
	DepthRender*    render;
	bool            saveFrame;
	int             saveIdx;
	cv::Mat         depth;
	cv::Mat         leftIR;
	cv::Mat         rightIR;
	cv::Mat         color;
	cv::Mat         point3D;
};

void frameCallback(TY_FRAME_DATA* frame, void* userdata)
{
	CallbackData* pData = (CallbackData*)userdata;
	LOGD("=== Get frame %d", ++pData->index);

	while (fakeLock) {
		MSLEEP(10);
	}
	fakeLock = true;

	pData->depth.release();
	pData->leftIR.release();
	pData->rightIR.release();
	pData->color.release();
	pData->point3D.release();

	parseFrame(*frame, &pData->depth, &pData->leftIR, &pData->rightIR
		, &pData->color, &pData->point3D);

	fakeLock = false;

	if (!pData->color.empty()) {
		LOGI("Color format is %s", colorFormatName(TYImageInFrame(*frame, TY_COMPONENT_RGB_CAM)->pixelFormat));
	}

	LOGD("=== Callback: Re-enqueue buffer(%p, %d)", frame->userBuffer, frame->bufferSize);
	ASSERT_OK(TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize));
}

void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
	if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
		LOGD("=== Event Calllback: Device Offline!");
	}
}

int main(int argc, char* argv[])
{
	string LaunchStr = " ";
	string exe_path;
	string color_or_IR;
	int color_cnt = 0;

	const char* IP = NULL;
	const char* ID = NULL;
	TY_DEV_HANDLE hDevice;
	int _idx = 0;

	/*int argc = 7;
	char * argv[] = { "", "-exe", "F:\\VS-workspace\\BinocularCalibration\\Debug\\BinocularCalibration.exe",
		"-K","F:\\VS-workspace\\BinocularCalibration\\TestData\\IR_left.yaml","IR"
	};*/

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-id") == 0) {
			ID = argv[++i];
		}
		else if (strcmp(argv[i], "-ip") == 0) {
			IP = argv[++i];
		}
		else if (strcmp(argv[i], "-exe") == 0) {
			++i;
			exe_path = argv[i];
		}
		else if (strcmp(argv[i], "-K") == 0) {
			LaunchStr += " -K ";
			LaunchStr = LaunchStr + " " + argv[++i] + " ";
			//LaunchStr = LaunchStr + " " + argv[++i] + " ";
		}
		else if (strcmp(argv[i], "-h") == 0) {
			LOGI("Usage: SimpleView_Callback [-h] [-ip <IP>]");
			LOGI("Usage: SimpleView_Callback -exe exePath -K K.yaml color");
			return 0;
		}
		else {
			color_or_IR = argv[i];
		}
	}

	LOGD("=== Init lib");
	ASSERT_OK(TYInitLib());
	TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer;
	ASSERT_OK(TYLibVersion(pVer));
	LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

	if (IP) {
		LOGD("=== Open device %s", IP);
		ASSERT_OK(TYOpenDeviceWithIP(IP, &hDevice));
	}
	else {
		if (ID == NULL) {
			LOGD("=== Get device info");
			int n;
			ASSERT_OK(TYGetDeviceNumber(&n));
			LOGD("     - device number %d", n);

			TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
			ASSERT_OK(TYGetDeviceList(pBaseInfo, 100, &n));

			if (n == 0) {
				LOGD("=== No device got");
				return -1;
			}
			ID = pBaseInfo[0].id;
		}

		LOGD("=== Open device: %s", ID);
		ASSERT_OK(TYOpenDevice(ID, &hDevice));
	}

#ifdef DEVELOPER_MODE
	LOGD("=== Enter Developer Mode");
	ASSERT_OK(TYEnterDeveloperMode(hDevice));
#endif

	int32_t allComps;
	ASSERT_OK(TYGetComponentIDs(hDevice, &allComps));
	if (allComps & TY_COMPONENT_RGB_CAM) {
		LOGD("=== Has RGB camera, open RGB cam");
		ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM));
	}

	LOGD("=== Configure components, open depth cam");
	int32_t componentIDs = TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_IR_CAM_LEFT | TY_COMPONENT_IR_CAM_RIGHT;
	// int32_t componentIDs = TY_COMPONENT_DEPTH_CAM;
	// int32_t componentIDs = TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_IR_CAM_LEFT;
	ASSERT_OK(TYEnableComponents(hDevice, componentIDs));

	int err = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
	ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

	LOGD("=== Prepare image buffer");
	int32_t frameSize;
	ASSERT_OK(TYGetFrameBufferSize(hDevice, &frameSize));
	LOGD("     - Get size of framebuffer, %d", frameSize);
	ASSERT(frameSize >= 640 * 480 * 2);

	LOGD("     - Allocate & enqueue buffers");
	char* frameBuffer[2];
	frameBuffer[0] = new char[frameSize];
	frameBuffer[1] = new char[frameSize];
	LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
	ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize));
	LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
	ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize));

	LOGD("=== Register frame callback");
	LOGD("Note:  user should call TYEnqueueBuffer to re-enqueue frame buffer.");
	DepthRender render;
	CallbackData cb_data;
	cb_data.index = 0;
	cb_data.hDevice = hDevice;
	cb_data.render = &render;
	cb_data.saveFrame = false;
	cb_data.saveIdx = 0;
	ASSERT_OK(TYRegisterCallback(hDevice, frameCallback, &cb_data));

	LOGD("=== Register event callback");
	LOGD("Note: Callback may block internal data receiving,");
	LOGD("      so that user should not do long time work in callback.");
	ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, NULL));

	LOGD("=== Disable trigger mode");
	ASSERT_OK(TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false));

	LOGD("=== Start capture");
	ASSERT_OK(TYStartCapture(hDevice));

	LOGD("=== Wait for callback");
	bool exit_main = false;
	DepthViewer depthViewer;

	string tmp_launch_str = LaunchStr;

	while (!exit_main) {
		while (fakeLock) {
			MSLEEP(10);
		}
		fakeLock = true;
		double windowRate = 2.;
		if (!cb_data.depth.empty()) {
			//depthViewer.show("depth", cb_data.depth);
		}
		if (!cb_data.leftIR.empty()) {
			cv::namedWindow("LeftIR", 0);
			cv::resizeWindow("LeftIR", cb_data.leftIR.cols / windowRate, cb_data.leftIR.rows / windowRate);
			cv::imshow("LeftIR", cb_data.leftIR);
		}
		if (!cb_data.rightIR.empty()) {
			cv::namedWindow("RightIR", 0);
			cv::resizeWindow("RightIR", cb_data.rightIR.cols / windowRate, cb_data.rightIR.rows / windowRate);
			cv::imshow("RightIR", cb_data.rightIR);
		}
		if (!cb_data.color.empty()) {
			cv::namedWindow("color", 0);
			cv::resizeWindow("color", cb_data.color.cols / windowRate, cb_data.color.rows / windowRate);
			cv::imshow("color", cb_data.color);
		}

		if (cb_data.saveFrame && !cb_data.depth.empty() && !cb_data.leftIR.empty() && !cb_data.rightIR.empty()) {
			LOGI(">>>> save frame %d", cb_data.saveIdx);

			cv::imwrite(std::to_string(cb_data.saveIdx) + "_color.jpg", cb_data.color);
			cv::imwrite(std::to_string(cb_data.saveIdx) + "_leftIR.jpg", cb_data.leftIR);
			cv::imwrite(std::to_string(cb_data.saveIdx) + "_rightIR.jpg", cb_data.rightIR);


			if (color_or_IR == "color") {
				color_cnt++;
				if (1 == color_cnt) {
					tmp_launch_str = LaunchStr;
					tmp_launch_str += " -img " + std::to_string(cb_data.saveIdx) + "_color.jpg ";
				}
				if (color_cnt >= 2) {
					tmp_launch_str += " " + std::to_string(cb_data.saveIdx) + "_color.jpg ";
					tmp_launch_str += " -idx " + std::to_string(cb_data.saveIdx) + " ";
					int ret = (int)ShellExecuteA(NULL, ("open"), exe_path.c_str(), tmp_launch_str.c_str(), NULL, SW_NORMAL);//打开exe
					if (ret < 32)//检测是否指定成功
						MessageBoxA(NULL, "Shell Execute Error!", "Error", 0);
					color_cnt = 0;
				}
			}
			else if (color_or_IR == "IR") {
				tmp_launch_str = LaunchStr;
				tmp_launch_str += " -img " + std::to_string(cb_data.saveIdx) + "_leftIR.jpg "
					+ std::to_string(cb_data.saveIdx) + "_rightIR.jpg ";
				tmp_launch_str += " -idx " + std::to_string(cb_data.saveIdx) + " ";
				int ret = (int)ShellExecuteA(NULL, ("open"), exe_path.c_str(), tmp_launch_str.c_str(), NULL, SW_NORMAL);//打开exe
				if (ret < 32)//检测是否指定成功
					MessageBoxA(NULL, "Shell Execute Error!", "Error", 0);
			}

			cb_data.saveIdx++;
			/*char f[32];
			sprintf(f, "%d.img", cb_data.saveIdx++);
			FILE* fp = fopen(f, "wb");
			fwrite(cb_data.depth.data, 2, cb_data.depth.size().area(), fp);
			fwrite(cb_data.color.data, 3, cb_data.color.size().area(), fp);*/

			// fwrite(cb_data.leftIR.data, 1, cb_data.leftIR.size().area(), fp);
			// fwrite(cb_data.rightIR.data, 1, cb_data.rightIR.size().area(), fp);
			//fclose(fp);

			cb_data.saveFrame = false;
		}

		fakeLock = false;

		int key = cv::waitKey(10);
		switch (key & 0xff) {
		case 0xff:
			break;
		case 'q':
			exit_main = true;
			break;
		case 's':
			cb_data.saveFrame = true;
			break;
		default:
			LOGD("Unmapped key %d", key);
		}

#ifdef DEVELOPER_MODE
		DEVELOPER_MODE_PRINT();
#endif
	}

	ASSERT_OK(TYStopCapture(hDevice));
	ASSERT_OK(TYCloseDevice(hDevice));
	ASSERT_OK(TYDeinitLib());
	delete frameBuffer[0];
	delete frameBuffer[1];

	LOGD("=== Main done!");
	return 0;
}
