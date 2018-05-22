#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;


void help(){
	std::cout << "video2images -pre prefix -suf suffix file_path" << endl;
	std::cout << "\t-pre 'out file prefix'" << endl;
	std::cout << "\t-suf 'out file suffix'" << endl;
}


int main(int argc,char * argv[]){
	try {
		if (argc < 2){
			help(); return 0;
		}

		string file_path, prefix, suffix = ".jpg";
		for (int i = 1; i < argc;) {
			if (!strcmp(argv[i], "-pre")) {
				prefix = argv[++i];
				++i;
			}
			else if (!strcmp(argv[i], "-suf")) {
				suffix = argv[++i];
				++i;
			}
			else{
				file_path = argv[i];
				++i;
			}
		}

		VideoCapture cap;
		cap.open(file_path); 

		if (!cap.isOpened())
			return 1;

		int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);  //帧宽度
		int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //帧高度
		int frameRate = cap.get(CV_CAP_PROP_FPS);  //帧率 x frames/s
		int totalFrames = cap.get(CV_CAP_PROP_FRAME_COUNT); //总帧数

		FileStorage fs;
		fs.open("video_attribute.yaml", FileStorage::WRITE);
		fs	<< "width" << width 
			<< "height" << height
			<< "totalFrames" << totalFrames 
			<< "frameRate" << frameRate;
		fs.release();

		Mat frame;
		int video_index = 0;
		while (1)
		{
			cap >> frame;//等价于cap.read(frame);
			if (frame.empty())
				break;
			imwrite(prefix + to_string(video_index++) + suffix,frame);
		}
		cap.release();
	}
	catch (exception e){
		cout << e.what() << endl;
	}

	return 0;
}