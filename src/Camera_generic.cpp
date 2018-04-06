#include <sensors/Camera/Camera_generic.h>
using namespace sensors;
using namespace Times;

#ifdef USE_CAMERA

#define CLEAR(x) memset(&(x), 0, sizeof(x))
struct buffer {
	void   *start;
	size_t length;
};
static bool xioctl(int fh, int request, void *arg)
{
	int r;
	do {
		r = v4l2_ioctl(fh, request, arg);
	} while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));
	if (r == -1) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
//		exit(EXIT_FAILURE);
		return false;
	}
	return true;
}
#endif

Camera::Camera() {

}
Camera::~Camera() {

}
bool Camera::Init() {

#ifdef USE_CAMERA
	// Actual code goes here to take Camera image
	struct v4l2_format              fmt;
	struct v4l2_buffer              buf;
	struct v4l2_requestbuffers      req;
	enum v4l2_buf_type              type;
	fd_set                          fds;
	struct timeval                  tv;
	int                             r, fd = -1;
	unsigned int                    i, n_buffers;
	char                            *dev_name = "/dev/video0";
	char                            out_name[256];
	FILE                            *fout;
	struct buffer                   *buffers;
	int                             p;

	fd = v4l2_open(dev_name, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		perror("INIT ERROR: Can't open Camera.");
//		exit(EXIT_FAILURE);
		return false;
	}
	else {
		v4l2_close(fd);
	}
#endif

	return true;
}
bool Camera::Reset() {

	return true;
}
bool Camera::Enable() {

	return true;
}
bool Camera::Disable() {

	return true;
}
bool Camera::TakeImage(int a) {
	//bool Camera::TakeImage(int argc, char **argv, int a){

#ifndef USE_CAMERA

// No camera control for non-camera runs
	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CMA]: (Fake) Camera image taken!\n";
	return true;

#else

// Actual code goes here to take Camera image
	struct v4l2_format              fmt;
	struct v4l2_buffer              buf;
	struct v4l2_requestbuffers      req;
	enum v4l2_buf_type              type;
	fd_set                          fds;
	struct timeval                  tv;
	int                             r, fd = -1;
	unsigned int                    i, n_buffers;
	char                            *dev_name = "/dev/video0";
	char                            out_name[256];
	FILE                            *fout;
	struct buffer                   *buffers;
	int                             p;

	fd = v4l2_open(dev_name, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		perror("Cannot open device");
		exit(EXIT_FAILURE);
	}

	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = 1920;
	fmt.fmt.pix.height = 1080;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	xioctl(fd, VIDIOC_S_FMT, &fmt);
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
		printf("Libv4l didn't accept RGB24 format. Can't proceed.\n");
	//	exit(EXIT_FAILURE);
		return false;
	}
	if ((fmt.fmt.pix.width != 1920) || (fmt.fmt.pix.height != 1080))
		printf("Warning: driver is sending image at %dx%d\n",
			fmt.fmt.pix.width, fmt.fmt.pix.height);

	CLEAR(req);
	req.count = 2;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	bool b1 = xioctl(fd, VIDIOC_REQBUFS, &req);
	if(!b1){ return false; }

	buffers = (buffer*)calloc(req.count, sizeof(*buffers));

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;

		xioctl(fd, VIDIOC_QUERYBUF, &buf);

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, buf.m.offset);

		std::cout << "buffer count: " << n_buffers << "\n";
		if (MAP_FAILED == buffers[n_buffers].start) {
			perror("mmap");
			std::cout << "MAP FAILED ERROR\n";
//			exit(EXIT_FAILURE);
			return false;
		}
	}

	for (i = 0; i < n_buffers; ++i) {
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		bool b2 = xioctl(fd, VIDIOC_QBUF, &buf);
		if(!b2) { return false; }
		std::cout << "buf: " << i << "\n";
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	xioctl(fd, VIDIOC_STREAMON, &type);
	/*for (i = 0; i < 5; i++) {*/
	/*printf("Enter the location point\n");
	scanf("%d",&p);*/
	do {
		FD_ZERO(&fds);
		FD_SET(fd, &fds);

		/* Timeout. */
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		std::cout << "About to select r, timeout(sec): " << tv.tv_sec << ".\n";
		r = select(fd + 1, &fds, NULL, NULL, &tv);
	} while ((r == -1 && (errno = EINTR)));

	if (r == -1) {
		perror("select");
		std::cout << "R IS -1\n";
//		return errno;
		return false;
	}

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	bool b3 = xioctl(fd, VIDIOC_DQBUF, &buf);
	if(!b3){ return false; }

	sprintf(out_name, "image%d.ppm", a);
	fout = fopen(out_name, "w");
	if (!fout) {
		perror("Cannot open image");
//		exit(EXIT_FAILURE);
		return false;
	}
	fprintf(fout, "P6\n%d %d 255\n",
		fmt.fmt.pix.width, fmt.fmt.pix.height);
	fwrite(buffers[buf.index].start, buf.bytesused, 1, fout);
	fclose(fout);

	xioctl(fd, VIDIOC_QBUF, &buf);
	/*}*/

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bool b4 = xioctl(fd, VIDIOC_STREAMOFF, &type);
	if(!b4){ return false; }
	for (i = 0; i < n_buffers; ++i){
		v4l2_munmap(buffers[i].start, buffers[i].length);
		std::cout << "unmap: " << i << "\n";
	}
	v4l2_close(fd);

	TimeModule::Log("CMA", ": Camera image taken!");

#endif

	return true;
}
