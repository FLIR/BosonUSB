/*
------------------------------------------------------------------------
-  FLIR Systems - Linux Boson  Capture & Recording                     -
------------------------------------------------------------------------
-  This code is using part of the explanations from this page          -
-  https://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/  -
-                                                                      -
-  and completed to be used with FLIR Boson cameras in 16 and 8 bits.  -
-  Internal AGC for 16bits mode is very basic, with just the intention -
-  of showing how to make that image displayable                       - 
------------------------------------------------------------------------

 BosonUSB [r/y/z/s/f] [0..9]
	r    : raw16 bits video input (default)
	y    : agc-8 bits video input
	z    : zoom mode to 640x480 (only applies to raw16 input)
        f<name> : record TIFFS in Folder <NAME>
	s[b,B]  : camera size : b=boson320, B=boson640
	[0..9]  : linux video port

./BosonUSB   ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB r ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB y ->  opens Boson320 /dev/video0  in AGC-8bits mode
./BosonUSB sB 1    ->  opens Boson640 /dev/video1  in RAW16 mode
./BosonUSB sB y 2  ->  opens Boson640 /dev/video2  in AGC-8bits mode
./BosonUSB fcap -> creates a folder named 'cap' and inside TIFF files (raw16, agc, yuv) will be located.

*/

#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>


#define YUV   0
#define RAW16 1

using namespace cv;

#define v_major 1
#define v_minor 0

// Define COLOR CODES
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"


// Global variables to keep this simple
int width;
int height;

// Types of sensors supported
enum sensor_types {
  Boson320, Boson640
};


/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(Mat input_16, Mat output_8, int height, int width) {
	int i, j;  // aux variables

	// auxiliary variables for AGC calcultion
	unsigned int max1=0;         // 16 bits
	unsigned int min1=0xFFFF;    // 16 bits
	unsigned int value1, value2, value3, value4;

	// RUN a super basic AGC
	for (i=0; i<height; i++) {
		for (j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			if ( value3 <= min1 ) {
				min1 = value3;
			}
			if ( value3 >= max1 ) {
				max1 = value3;
			}
			//printf("%X.%X.%X  ", value1, value2, value3);
		}
	}
	//printf("max1=%04X, min1=%04X\n", max1, min1);

	for (int i=0; i<height; i++) {
		for (int j=0; j<width; j++) {
			value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
			value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
			value3 = ( value1 << 8) + value2;
			value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
			// printf("%04X \n", value4);

			output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
		}
	}

}


/* ---------------------------- Other Aux functions ---------------------------------------*/

// HELP INFORMATION
void print_help() {
	printf(CYN "Boson Capture and Record Video tool v%i.%i" WHT "\n", v_major, v_minor);
	printf(CYN "FLIR Systems" WHT "\n\n");
	printf(WHT "use : " YEL "'BosonUSB r' " WHT "to capture in raw-16 bits mode   (default)\n");
	printf(WHT "Use : " YEL "'BosonUSB y' " WHT "to capture in agc-8  bits mode\n");
  	printf(WHT "Use : " YEL "'BosonUSB z' " WHT "Zoom to 640x512 (only in RAW) mode  (default ZOOM OFF)\n");
	printf(WHT "Use : " YEL "'BosonUSB f<name>' " WHT "record TIFFS in Folder <NAME>\n");
	printf(WHT "Use : " YEL "'BosonUSB [0..9]'   " WHT "to open /dev/Video[0..9]  (default 0)\n");
	printf(WHT "Use : " YEL "'BosonUSB s[b,B]'   " WHT "b=boson320, B=boson640   (default 320)\n");
	printf(WHT "Press " YEL "'q' " WHT " to quit\n");
	printf("\n");
}

/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT
int main(int argc, char** argv )
{
	int ret;
	int fd;
	int i;
	struct v4l2_capability cap;
	long frame=0;     // First frame number enumeration
	char video[20];   // To store Video Port Device
	char label[50];   // To display the information
	char thermal_sensor_name[20];  // To store the sensor name
	char filename[60];  // PATH/File_count
	char folder_name[30];  // To store the folder name
	// Default Program options
	int  video_mode=RAW16;
	int  zoom_enable=0;
	int  record_enable=0;
	sensor_types my_thermal=Boson320;

	// To record images
	std::vector<int> compression_params;
	compression_params.push_back(IMWRITE_PXM_BINARY);

	// Display Help
	print_help();

	// Video device by default
	sprintf(video, "/dev/video0");
	sprintf(thermal_sensor_name, "Boson_320");

	// Read command line arguments
	for (i=0; i<argc; i++) {
		// Check if RAW16 video is desired
		if ( argv[i][0]=='r') {
			video_mode=RAW16;
		}
		// Check if AGC video is desired
		if ( argv[i][0]=='y') {
			video_mode=YUV;
		}
		// Check if ZOOM to 640x512 is enabled
		if ( argv[i][0]=='z') {
        		zoom_enable=1;
      		}
		// Check if recording is enabled
		if ( argv[i][0]=='f') {  // File name has to be more than two chars
            		record_enable=1;
            		if ( strlen(argv[i])>2 ) {
                		strcpy(folder_name, argv[i]+1);
            		}
      		}
		// Look for type/size of sensor
		if ( argv[i][0]=='s') {
          		switch ( argv[i][1] ) {
            			case 'B'/* value */:
                			my_thermal=Boson640;
               				sprintf(thermal_sensor_name, "Boson_640");
		        	        break;
            			default:
				        my_thermal=Boson320;
				        sprintf(thermal_sensor_name, "Boson_320");
          		}
      		}
		// Look for feedback in ASCII
		if (argv[i][0]>='0' && argv[i][0]<='9') {
			sprintf(video, "/dev/video%c",argv[i][0]);
		}

  	}

	// Folder name
	if (record_enable==1) {
		if ( strlen(folder_name)<=1 ) {  // File name has to be more than two chars
		        strcpy(folder_name, thermal_sensor_name);
	        }
	        mkdir(folder_name, 0700);
	        chdir(folder_name);
                printf(WHT ">>> Folder " YEL "%s" WHT " selected to record files\n", folder_name);
  	}

	// Printf Sensor defined
	printf(WHT ">>> " YEL "%s" WHT " selected\n", thermal_sensor_name);

	// We open the Video Device
	printf(WHT ">>> " YEL "%s" WHT " selected\n", video);
	if((fd = open(video, O_RDWR)) < 0){
		perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
		exit(1);
	}

	// Check VideoCapture mode is available
	if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
	    perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
	    exit(1);
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
		exit(1);
	}

	struct v4l2_format format;

	// Two different FORMAT modes, 8 bits vs RAW16
	if (video_mode==RAW16) {
		printf(WHT ">>> " YEL "16 bits " WHT "capture selected\n");

		// I am requiring thermal 16 bits mode
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

		// Select the frame SIZE (will depend on the type of sensor)
		switch (my_thermal) {
			case Boson320:  // Boson320
			          	width=320;
				        height=256;
				        break;
		        case Boson640:  // Boson640
				        width=640;
				        height=512;
				        break;
			default:  // Boson320
				        width=320;
				        height=256;
				        break;
		 }

	} else { // 8- bits is always 640x512 (even for a Boson 320)
		 printf(WHT ">>> " YEL "8 bits " WHT "YUV selected\n");
	         format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works   LUMA, full Cr, full Cb
		 width = 640;
		 height = 512;
	}

	// Common varibles
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;

	// request desired FORMAT
	if(ioctl(fd, VIDIOC_S_FMT, &format) < 0){
		perror(RED "VIDIOC_S_FMT" WHT);
		exit(1);
	}

	// we need to inform the device about buffers to use.
	// and we need to allocate them.
	// we’ll use a single buffer, and map our memory using mmap.
	// All this information is sent using the VIDIOC_REQBUFS call and a
	// v4l2_requestbuffers structure:
	struct v4l2_requestbuffers bufrequest;
	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;   // we are asking for one buffer

	if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
		perror(RED "VIDIOC_REQBUFS" WHT);
		exit(1);
	}

	// Now that the device knows how to provide its data,
	// we need to ask it about the amount of memory it needs,
	// and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
	// and its v4l2_buffer structure.

	struct v4l2_buffer bufferinfo;
	memset(&bufferinfo, 0, sizeof(bufferinfo));

	bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufferinfo.memory = V4L2_MEMORY_MMAP;
	bufferinfo.index = 0;

	if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
		perror(RED "VIDIOC_QUERYBUF" WHT);
		exit(1);
	}


	// map fd+offset into a process location (kernel will decide due to our NULL). lenght and
	// properties are also passed
	printf(WHT ">>> Image width  =" YEL "%i" WHT "\n", width);
	printf(WHT ">>> Image height =" YEL "%i" WHT "\n", height);
	printf(WHT ">>> Buffer lenght=" YEL "%i" WHT "\n", bufferinfo.length);

	void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);

	if(buffer_start == MAP_FAILED){
		perror(RED "mmap" WHT);
		exit(1);
	}

	// Fill this buffer with ceros. Initialization. Optional but nice to do
	memset(buffer_start, 0, bufferinfo.length);

	// Activate streaming
	int type = bufferinfo.type;
	if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
		perror(RED "VIDIOC_STREAMON" WHT);
		exit(1);
	}


	// Declarations for RAW16 representation
        // Will be used in case we are reading RAW16 format
	// Boson320 , Boson 640
	Mat thermal16(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	Mat thermal16_linear(height,width, CV_8U, 1);         // OpenCV output buffer : Data used to display the video

	// Declarations for Zoom representation
    	// Will be used or not depending on program arguments
	Size size(640,512);
	Mat thermal16_linear_zoom;   // (height,width, CV_8U, 1);    // Final representation
	Mat thermal_rgb_zoom;   // (height,width, CV_8U, 1);    // Final representation

	int luma_height ;
	int luma_width ;
	int color_space ;;

	// Declarations for 8bits YCbCr mode
        // Will be used in case we are reading YUV format
	// Boson320, 640 :  4:2:0
	luma_height = height+height/2;
	luma_width = width;
	color_space = CV_8UC1;
 	Mat thermal_luma(luma_height, luma_width,  color_space, buffer_start);  // OpenCV input buffer
	Mat thermal_rgb(height, width, CV_8UC3, 1);    // OpenCV output buffer , BGR -> Three color spaces (640 - 640 - 640 : p11 p21 p31 .... / p12 p22 p32 ..../ p13 p23 p33 ...)

	// Reaad frame, do AGC, paint frame
	for (;;) {

		// Put the buffer in the incoming queue.
		if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}

		// The buffer's waiting in the outgoing queue.
		if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}


		// -----------------------------
		// RAW16 DATA
		if ( video_mode==RAW16 ) {
			AGC_Basic_Linear(thermal16, thermal16_linear, height, width);

			// Display thermal after 16-bits AGC... will display an image
			if (zoom_enable==0) {
                		sprintf(label, "%s : RAW16  Linear", thermal_sensor_name);
                    		imshow(label, thermal16_linear);
          		} else {
			        resize(thermal16_linear, thermal16_linear_zoom, size);
                     		sprintf(label, "%s : RAW16  Linear Zoom", thermal_sensor_name);
                     		imshow(label, thermal16_linear_zoom);
                	}


	        	if (record_enable==1) {
        	        	sprintf(filename, "%s_raw16_%lu.tiff", thermal_sensor_name, frame);
                		imwrite(filename, thermal16 , compression_params );
                        	sprintf(filename, "%s_agc_%lu.tiff", thermal_sensor_name, frame);
                        	imwrite(filename, thermal16_linear , compression_params );
				frame++;
                	}
          	}


		// ---------------------------------
		// DATA in YUV
		else {  // Video is in 8 bits YUV
            		cvtColor(thermal_luma, thermal_rgb, COLOR_YUV2RGB_I420, 0 );   // 4:2:0 family instead of 4:2:2 ...

        		sprintf(label, "%s : 8bits", thermal_sensor_name);
		        imshow(label, thermal_rgb);

			if (record_enable==1) {
                        	sprintf(filename, "%s_yuv_%lu.tiff", thermal_sensor_name, frame);
                        	imwrite(filename, thermal_rgb , compression_params );
				frame++;
                	}

		}

		// Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}
	}
	// Finish Loop . Exiting.

	// Deactivate streaming
	if( ioctl(fd, VIDIOC_STREAMOFF, &type) < 0 ){
		perror(RED "VIDIOC_STREAMOFF" WHT);
		exit(1);
	};

	close(fd);
	return EXIT_SUCCESS;
}
