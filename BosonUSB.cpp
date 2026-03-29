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

 BosonUSB [r/y/z/s/t/f] [0..9]
	r    : raw16 bits video input (default)
	y    : agc-8 bits video input
	z    : zoom mode to 640x480 (only applies to raw16 input)
        f<name> : record TIFFS in Folder <NAME>
        t<number> : number of frames to record
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

// Need to clean video for linux structs to avoid some random initializations problems (not always present)
#define CLEAR(x) memset(&(x), 0, sizeof(x))

// Global variables to keep this simple
int width;
int height;

// Types of sensors supported
enum sensor_types {
  Boson320, 
  Boson640
};


/* ------------- Functions to swap bytes (high - low ) in case platform needs it --------------- */

static inline uint16_t swap_u16(uint16_t v) {
    return static_cast<uint16_t>((v >> 8) | (v << 8));
}

static void print_fourcc(__u32 fmt) {
    printf("%c%c%c%c",
           fmt & 0xFF,
           (fmt >> 8) & 0xFF,
           (fmt >> 16) & 0xFF,
           (fmt >> 24) & 0xFF);
}

/* ---------------------------- 16 bits Mode auxiliary functions ---------------------------------------*/

// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(const cv::Mat& input_16, cv::Mat& output_8, int h, int w) {
    uint16_t min1 = 0xFFFF;
    uint16_t max1 = 0x0000;

    for (int i = 0; i < h; i++) {
        const uint16_t* row = input_16.ptr<uint16_t>(i);
        for (int j = 0; j < w; j++) {
            uint16_t v = row[j];
            if (v < min1) min1 = v;
            if (v > max1) max1 = v;
        }
    }

    if (max1 <= min1) {
        output_8.setTo(0);
        return;
    }

    for (int i = 0; i < h; i++) {
        const uint16_t* in_row = input_16.ptr<uint16_t>(i);
        uint8_t* out_row = output_8.ptr<uint8_t>(i);

        for (int j = 0; j < w; j++) {
            uint16_t v = in_row[j];
            uint32_t scaled = (255u * (v - min1)) / (max1 - min1);
            out_row[j] = static_cast<uint8_t>(scaled);
        }
    }
}

// This is in case image 16 bits is swapped 
void byteswap_image_16(const cv::Mat& src16, cv::Mat& dst16) {
    CV_Assert(src16.type() == CV_16UC1);
    dst16.create(src16.rows, src16.cols, CV_16UC1);

    for (int i = 0; i < src16.rows; ++i) {
        const uint16_t* s = src16.ptr<uint16_t>(i);
        uint16_t* d = dst16.ptr<uint16_t>(i);
        for (int j = 0; j < src16.cols; ++j) {
            d[j] = swap_u16(s[j]);
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
	printf(WHT "Use : " YEL "'BosonUSB f<name> t<frame_count>' " WHT "record TIFFS in Folder <NAME> and stop recording after <FRAME_COUNT> frames\n");
	printf(WHT "Use : " YEL "'BosonUSB [0..9]'   " WHT "to open /dev/Video[0..9]  (default 0)\n");
	printf(WHT "Use : " YEL "'BosonUSB s[b,B]'   " WHT "b=boson320, B=boson640   (default 320)\n");
	printf(WHT "Press " YEL "'q' in video window " WHT " to quit\n");
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
	char filename[128];  // PATH/File_count
	char folder_name[30] = {0};  // To store the folder name
    char video_frames_str[30] = {};

	// Default Program options
	int  video_mode=RAW16;
	int  video_frames=0;
	int  zoom_enable=0;
	int  record_enable=0;
	sensor_types my_thermal=Boson320;

	// To record images
	std::vector<int> compression_params;
	compression_params.push_back(IMWRITE_PXM_BINARY);

	// Display Help
	print_help();

	// Video device by default
	snprintf(video, sizeof(video), "/dev/video0");
    snprintf(thermal_sensor_name, sizeof(thermal_sensor_name), "Boson_320");

	// Read command line arguments
	for (i=1; i<argc; i++) {
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
					break;
        	}
      	}
		// Look for feedback in ASCII
		if (argv[i][0]>='0' && argv[i][0]<='9') {
			sprintf(video, "/dev/video%c",argv[i][0]);
		}
		// Look for frame count
		if ( argv[i][0]=='t') {
			if ( strlen(argv[i])>=2 ) {
			strcpy(video_frames_str, argv[i]+1);
				video_frames = atoi( video_frames_str );
				printf(WHT ">>> Number of frames to record =" YEL "%i" WHT "\n", video_frames);
			}	
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
	CLEAR(cap);
	if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
	    perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
	    exit(1);
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
		exit(1);
	}

	struct v4l2_format format;
	
	CLEAR(format);

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

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
	     format.fmt.pix.pixelformat = V4L2_PIX_FMT_YVU420; // thermal, works LUMA, full Cr, full Cb
		 width = 640;
		 height = 512;
	}

	// Common varibles
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;
   
    printf(WHT ">>> Pixelformat  = " YEL);
    print_fourcc(format.fmt.pix.pixelformat);
    printf(WHT "\n");

	bool is_i420 = (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420);  // YU12
	bool is_yv12 = (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420);  // YV12

	if (video_mode == RAW16 && format.fmt.pix.pixelformat != V4L2_PIX_FMT_Y16) {
    	fprintf(stderr, RED "ERROR: driver did not negotiate Y16 in RAW16 mode\n" WHT);
    	exit(1);
	}

	if (video_mode == YUV && !is_i420 && !is_yv12) {
	    fprintf(stderr, RED "ERROR: driver did not negotiate a supported 8-bit 4:2:0 format\n" WHT);
	    exit(1);
	}

	// we need to inform the device about buffers to use.
	// and we need to allocate them.
	// we’ll use a single buffer, and map our memory using mmap.
	// All this information is sent using the VIDIOC_REQBUFS call and a
	// v4l2_requestbuffers structure:
	struct v4l2_requestbuffers bufrequest;
	CLEAR(bufrequest);
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


	// Common Mats
	Size size(640, 512);

	// RAW16 Mats (only valid in RAW16 mode)
	Mat thermal16;
	Mat thermal16_linear;
	Mat thermal16_linear_zoom;

	// YUV Mats (only valid in YUV mode)
	Mat thermal_yuv;
	Mat thermal_rgb;

	if (video_mode == RAW16) {
	    thermal16 = Mat(height, width, CV_16UC1, buffer_start, format.fmt.pix.bytesperline);
	    thermal16_linear = Mat(height, width, CV_8UC1);
	    thermal16_linear_zoom = Mat();
	} else {
	    thermal_yuv = Mat(height + height / 2, width, CV_8UC1, buffer_start);
	    thermal_rgb = Mat(height, width, CV_8UC3);
	}

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
            
			if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
				// YU12 / I420
				cvtColor(thermal_yuv, thermal_rgb, COLOR_YUV2BGR_I420);
			} else if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420) {
				// YV12
				cvtColor(thermal_yuv, thermal_rgb, COLOR_YUV2BGR_YV12);
			}

			sprintf(label, "%s : 8bits", thermal_sensor_name);
			imshow(label, thermal_rgb);

			if (record_enable==1) {
				sprintf(filename, "%s_yuv_%lu.tiff", thermal_sensor_name, frame);
				imwrite(filename, thermal_rgb);
				frame++;
			}

		}

		// Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}
		// Stop if frame limit reached.
		if (video_frames>0 && frame+1 > video_frames) {
			printf(WHT ">>>" GRN "'Done'" WHT " Frame limit reached, Quitting !\n");
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
