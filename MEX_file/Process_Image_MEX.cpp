// This code is based on the OpenPose C++ API Tutorial from the authors: https://github.com/CMU-Perceptual-Computing-Lab/openpose
// Modified by Gil Serrancolí to create the MEX file and enable the reading of the skeleton from MATLAB.

// ----------------------------- OpenPose C++ API Tutorial - Example 1 - Body from image -----------------------------
// It reads an image, process it, and displays it with the pose keypoints.

// Command-line user intraface
#define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include "mex.h"

// Custom OpenPose flags
// Producer
DEFINE_string(image_path, "examples/media/COCO_val2014_000000000192.jpg",
    "Process an image. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display,                 false,
    "Enable to disable the visual display.");

DEFINE_string(net_resolution, "16x32", "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
	" decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
	" closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
	" any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
	" input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
	" e.g. full HD (1980x1080) and HD (1280x720) resolutions.");

DEFINE_int32(num_gpu_start, 0, "GPU device start number.");

DEFINE_int32(face_render, 0, "Analogous to render_pose but applied to the face. Extra option: -1 to use the same configuration that render_pose is using.");

DEFINE_int32(hand_render, 0, "Analogous to render_pose but applied to the hand. Extra option: -1 to use the same configuration that render_pose is using.");

DEFINE_int32(display, 0, "Display mode: -1 for automatic selection; 0 for no display (useful if there is no X server and/or to slightly speed up the processing if visual output is not required); 2 for 2-D display; 3 for 3-D display (if --3d enabled); and 1 for both 2-D and 3-D display.");

DEFINE_string(model_folder, "models/", "Folder path (absolute or relative) where the models (pose, face, ...) are located.");

DEFINE_string(model_pose, "COCO", "Model to be used. E.g., COCO (18 keypoints), MPI (15 keypoints, ~10% faster), MPI_4_layers (15 keypoints, even faster but less accurate).");

// This worker will just read and return all the jpg files in a directory
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", datumsPtr->at(0)->cvOutputData);
            //cv::waitKey(0);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Alternative 1
            //op::log("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

            // // Alternative 2
            // op::log(datumsPtr->at(0).poseKeypoints, op::Priority::High);

            // // Alternative 3
            // std::cout << datumsPtr->at(0).poseKeypoints << std::endl;

            // // Alternative 4 - Accesing each element of the keypoints
            // op::log("\nKeypoints:", op::Priority::High);
            // const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
            // op::log("Person pose keypoints:", op::Priority::High);
            // for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            // {
            //     op::log("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
            //     for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            //     {
            //         std::string valueToPrint;
            //         for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
            //             valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
            //         op::log(valueToPrint, op::Priority::High);
            //     }
            // }
            // op::log(" ", op::Priority::High);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int tutorialApiCpp(cv::Mat inputImage, cv::Mat outputImage_ptrCV, std::vector<std::vector<std::vector<double>>> &poseKeypoints_out)
{
    try
    {
		
		if(inputImage.empty())
			op::error("Could not open or find the image: " + FLAGS_image_path, __LINE__, __FUNCTION__, __FILE__);
		
        op::log("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configuring OpenPose
        op::log("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();

        // Starting OpenPose
        op::log("Starting thread(s)...", op::Priority::High);
        opWrapper.start();

        // Process and display image
        // const auto imageToProcess = cv::imread(FLAGS_image_path);
		const auto imageToProcess = inputImage;
				
		// Format outputArray
		// Get desired scale sizes
		std::vector<double> scaleInputToNetInputs;
		std::vector<op::Point<int>> netInputSizes;
		double scaleInputToOutput;
		op::Point<int> outputResolution;
		
		//auto outputArray = cvMatToOpOutput.createArray(inputImage, scaleInputToOutput, outputResolution);

		
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
		
        if (datumProcessed != nullptr)
        {
            printKeypoints(datumProcessed);
			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			//op::log("Body keypoints: " + datumProcessed->at(0)->poseKeypoints.toString(), op::Priority::High);
			op::Array<float> poseKeypoints = datumProcessed->at(0)->poseKeypoints;
			
			auto outputImage = datumProcessed->at(0)->poseKeypoints.getConstCvMat();
			
            if (!FLAGS_no_display)
                display(datumProcessed);
			
			for (int i=0; i<outputImage.rows; i++){
				for (int j=0; j<outputImage.cols; j++){
					cv::Vec3b Vec3baux;
					for (int k=0; k<3; k++){
						Vec3baux[k]=(double)outputImage.at<cv::Vec3b>(cv::Point(j, i))[k];
						outputImage_ptrCV.at<cv::Vec3b>(i, j)[k]=Vec3baux[k];
					}
				}
			}
			int npeople = (int)poseKeypoints.getSize(0);
			int nparts = (int)poseKeypoints.getSize(1);
			poseKeypoints_out.resize(npeople);
			// myfile << "people detected =" << (double)poseKeypoints.getSize(0) << std::endl;
			
			for (int i=0; i< npeople; ++i){
				poseKeypoints_out[i].resize(nparts);
				for (int j=0; j<nparts; ++j){
					poseKeypoints_out[i][j].resize(3);
					poseKeypoints_out[i][j][0]=poseKeypoints[{i,j,0}];
					poseKeypoints_out[i][j][1]=poseKeypoints[{i,j,1}];
					poseKeypoints_out[i][j][2]=poseKeypoints[{i,j,2}];
				}

			}
			
        }
        else
            op::log("Image could not be processed.", op::Priority::High);

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return
        return 0;
    }
    catch (const std::exception& e)
    {
        return -1;
    }
}

int main(int argc, char *argv[], cv::Mat inputImage, cv::Mat outputImage, std::vector<std::vector<std::vector<double>>> &poseKeypoints_out)
{
	/*FLAGS_disable_multi_thread = true;*/


    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Running tutorialApiCpp
    return tutorialApiCpp(inputImage, outputImage, poseKeypoints_out);
}


void mexFunction(	int nlhs, mxArray *plhs[], 
					int nrhs, const mxArray *prhs[] )
        /* the MATLAB mex wrapper function */
{

char* argv[1];
argv[0]="";

const mwSize *MatrixSize = mxGetDimensions(prhs[0]);         //dimensions of the datamatrix
int nr_cols = *(MatrixSize + 1);                       //columns
int nr_rows = *(MatrixSize + 0);                       //rows
int nr_z = *(MatrixSize+2);
cv::Mat inputImage(nr_rows,nr_cols,CV_8UC3, cv::Scalar(0,0,255));

double* MatrixData_ptr = mxGetPr(prhs[0]);
std::ofstream myfile;


	for(int i = 0; i < nr_rows; i++){   
		for(int j = 0; j < nr_cols; j++){
		cv::Vec3b color;
		color[2]=MatrixData_ptr[i+j*nr_rows];
		color[1]=MatrixData_ptr[i+j*nr_rows+nr_rows*nr_cols];
		color[0]= MatrixData_ptr[i+j*nr_rows+2*nr_rows*nr_cols];
		
		inputImage.at<cv::Vec3b>(cv::Point(j, i)) = color;
	}
	
}


cv::Mat outputImage(nr_rows,nr_cols,CV_8UC3);
std::vector<std::vector<std::vector<double>>> poseKeypoints_out;

main(1, argv, inputImage, outputImage, poseKeypoints_out);

double* outputImage_ptr0;
double* outputImage_ptr1;
double* outputImage_ptr2;   

plhs[0] = mxCreateDoubleMatrix(nr_rows,nr_cols,mxREAL);		      //output variable ptr 
plhs[1] = mxCreateDoubleMatrix(nr_rows,nr_cols,mxREAL);		      //output variable ptr 
plhs[2] = mxCreateDoubleMatrix(nr_rows,nr_cols,mxREAL);		      //output variable ptr 

outputImage_ptr0 = mxGetPr(plhs[0]); 
outputImage_ptr1 = mxGetPr(plhs[1]);  
outputImage_ptr2 = mxGetPr(plhs[2]);    

for(int i = 0; i < nr_rows; i++){   
		                     
		for(int j = 0; j < nr_cols; j++){
			outputImage_ptr0[i+j*nr_rows]=double(outputImage.at<cv::Vec3b>(cv::Point(j, i))[0]);
			outputImage_ptr1[i+j*nr_rows]=double(outputImage.at<cv::Vec3b>(cv::Point(j, i))[1]);
			outputImage_ptr2[i+j*nr_rows]=double(outputImage.at<cv::Vec3b>(cv::Point(j, i))[2]);

	}	
}

size_t npeople=poseKeypoints_out.size();
size_t nparts=poseKeypoints_out[0].size();


plhs[3]=mxCreateDoubleMatrix(npeople,nparts,mxREAL);
plhs[4]=mxCreateDoubleMatrix(npeople,nparts,mxREAL);
plhs[5]=mxCreateDoubleMatrix(npeople,nparts,mxREAL);	
double* poseKeypointsX_ptr;
double* poseKeypointsY_ptr;
double* poseKeypointsSc_ptr;

poseKeypointsX_ptr=mxGetPr(plhs[3]); 
poseKeypointsY_ptr=mxGetPr(plhs[4]);
poseKeypointsSc_ptr=mxGetPr(plhs[5]);  

for(int i=0; i<npeople; ++i){
	for (int j=0; j<nparts; ++j){
		
		poseKeypointsX_ptr[j*npeople+i]=poseKeypoints_out[i][j][0];
		poseKeypointsY_ptr[j*npeople+i]=poseKeypoints_out[i][j][1];
		poseKeypointsSc_ptr[j*npeople+i]=poseKeypoints_out[i][j][2];
	}
}


}