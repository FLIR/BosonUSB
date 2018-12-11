FLIR SYSTEMS, INC <BR>
2018-Nov <BR>
MIT License <BR>

# Description:

This is an example of capturing Boson Video (USB) using V4L2 (Video for Linux 2)
and OpenCV to display the video. Boson USB has two modes, AGC-8 bits and
RAW-16bits.

Video in 16 bits is not paintable so a linear transformation needs to happen
before displaying the image. We call this process AGC, and in this example we
use the most basic one (LINEAR). Then we use stardard call to paint in GREY.
Video is 8 bits is directly paintable, linear transformation happens inside the
 camera, not further actions need to happen in SW.

To Display Boson data we are using OpenCV to convert from YUV to RGB.

# How to use it:
```
BosonUSB [r/y/a/b/z/f/t] [0..9] 
	r    : raw video
	y    : 8 bits
	z    : zoom mode to 640x480
	f<name>    : record TIFFS in Folder <NAME>
	t<video_frames> : record a certain number of frames being <video_frames> default sets to 0 which sets no limit
	s[b,B] : b=boson320, B=boson640   
	[0..9]: video port

./BosonUSB    ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB r  ->  opens Boson320 /dev/video0  in RAW16 mode
./BosonUSB y  ->  opens Boson320 /dev/video0  in AGC-8bits mode
./BosonUSB sB 1   ->  opens Boson640 /dev/video1  in RAW16 mode
./BosonUSB sB y 2 ->  opens Boson640 /dev/video2  in AGC-8bits mode
./BosonUSB fcap -> Captures RAW16 frames and stores them as TIFF files in 'cap' folder.
		   If in RAW16 mode then RAW16 and Linear_AGC are captured per frame
		   If in AGC-8 mode then YUV TIFF only are captured per frame
./BosonUSB fcap t100 -> Captures RAW16 frames and stores them as TIFF files in 'cap' folder and only captures 100 frames
```

# To compile

This SW uses some libraries as v4l2 and OpenCv, they need to be installed first in the PC.
They are not part of this package
```
cmake .
make

(if CMakeCache.txt exists remove it first time)
```

# How to clean the full project
```
make clean
rm -rf CMakeFiles
rm CMakeCache.txt
rm cmake_install.cmake
rm Makefile
```

# References and Credits (other than FLIR)

- http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2
- https://opencv.org
- https://en.wikipedia.org/wiki/Video4Linux
