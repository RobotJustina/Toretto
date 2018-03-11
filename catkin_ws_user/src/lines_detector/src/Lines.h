

class Lines
{
private:
	cv::Mat Image; 
public:
	Lines(cv::Mat image);
	cv::Mat segmentationLines();
	void linesToPoints();
	
};