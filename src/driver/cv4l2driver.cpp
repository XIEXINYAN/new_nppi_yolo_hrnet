#include "cv4l2driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))
void CV4l2driver::open_device()
{
    struct stat st;

    if (-1 == stat (m_dev_name, &st))
    {
        fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                 m_dev_name, errno, strerror (errno));
        exit (EXIT_FAILURE);
    }
    if (!S_ISCHR (st.st_mode))
    {
        fprintf (stderr, "%s is no device\n", m_dev_name);
        exit (EXIT_FAILURE);
    }
    m_fd = open(m_dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == m_fd)
    {
        fprintf (stderr, "Cannot open '%s': %d, %s\n",
                 m_dev_name, errno, strerror (errno));
        exit (EXIT_FAILURE);
    }
    return;
}
void CV4l2driver::init_device()
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;

    if (-1 == ioctl (m_fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf (stderr, "%s is no V4L2 device\n", m_dev_name);
            exit (EXIT_FAILURE);
        }
        else
        {
            exit(1);
        }
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf (stderr, "%s is no video capture device\n",m_dev_name);
        exit (EXIT_FAILURE);
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf (stderr, "%s does not support streaming i/o\n",m_dev_name);
        exit (EXIT_FAILURE);
    }
    /* Select video input, video standard and tune here. */
    CLEAR (cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == ioctl (m_fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        if (-1 == ioctl (m_fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }
    CLEAR (fmt);
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = m_width;
    fmt.fmt.pix.height      = m_height;
    fmt.fmt.pix.pixelformat = m_pixel_format;
    fmt.fmt.pix.field       = m_filed;

    if (-1 == ioctl (m_fd, VIDIOC_S_FMT, &fmt))
    {
        exit(1);
    }

    /* Note VIDIOC_S_FMT may change width and height. */

    /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;
    init_mmap();
    return;
}
void CV4l2driver::start_capturing()
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i)
    {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (-1 == ioctl (m_fd, VIDIOC_QBUF, &buf))
        {
            exit(1);
        }
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == ioctl (m_fd, VIDIOC_STREAMON, &type))
    {
        exit(1);
    }
    return;
}
void CV4l2driver::stop_capturing()
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl (m_fd, VIDIOC_STREAMOFF, &type))
    {
        exit(1);
    }
}
void CV4l2driver::uninit_device()
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap (m_buffers[i].start, m_buffers[i].length))
            exit(-1);

    free (m_buffers);
}
void CV4l2driver::close_device()
{
    if (-1 == close (m_fd))
        exit(1);
    m_fd = -1;
}


void CV4l2driver::writeImage()
{
    camera_connect_flag = false;
    m_time_stamp = getTimeStamp();
    struct v4l2_buffer buf;
    fd_set fds;
    struct timeval tv;
    static struct timeval t_prv;
    int r;
    FD_ZERO (&fds);
    FD_SET (m_fd, &fds);
    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    r = select (m_fd + 1, &fds, NULL, NULL, &tv);
    if (-1 == r)
    {
        return ;
    }

    if (0 == r)
    {
        fprintf (stderr, "select timeout\n");
        return ;
    }
    CLEAR (buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
   if (-1 == ioctl (m_fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
        case EAGAIN:
            return ;
        case EIO:
            // Could ignore EIO, see spec.
            // fall through
        default:
            return ;
        }
    }
    assert (buf.index < n_buffers);
//    cv::Mat yuv_img;
//    memcpy(m_yuv_img.data, m_buffers[buf.index].start, m_height*m_width*2*sizeof (unsigned char));
    cudaYUV2RGB((unsigned char*)m_buffers[buf.index].start);
//    cv::cvtColor(m_yuv_img,m_bgr_img,CV_YUV2BGR_YUYV);
    camera_connect_flag=true;
    if (-1 == ioctl (m_fd, VIDIOC_QBUF, &buf))
        return ;
    return;
}
void CV4l2driver::init_mmap()
{
//    td::cout<<"##################"<<std::endl;
    struct v4l2_requestbuffers req;

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;

    if (-1 == ioctl (m_fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            fprintf (stderr, "%s does not support "
                             "memory mapping\n", m_dev_name);
            exit (EXIT_FAILURE);
        }
        else
        {
            exit(1);
        }
    }

    m_buffers = (struct buffer *) calloc (req.count, sizeof(*m_buffers));
//    printf ("sizeof(void *)=%ld\n",sizeof(m_buffers));
    if (!m_buffers) {
        fprintf (stderr, "Out of memory\n");
        exit (EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == ioctl (m_fd, VIDIOC_QUERYBUF, &buf))
        {
            exit(1);
        }

        m_buffers[n_buffers].length = buf.length;
        m_buffers[n_buffers].start =
                mmap (nullptr /* start anywhere */,
                      buf.length,
                      PROT_READ | PROT_WRITE /* required */,
                      MAP_SHARED /* recommended */,
                      m_fd, buf.m.offset);

        if (MAP_FAILED == m_buffers[n_buffers].start)
        {
            exit(1);
        }
    }
}
