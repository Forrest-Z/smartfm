/*
*
* Gst-Template: pipeline1 loads and decodes video_ENC.avi passing the stream through AppSink which pushes buffers to pipeline2 to encode and write
* the stream to a file.
*
* OpenCV-Test: using the template to load the videostream and process each frame with openCV and write processed data using GStreamer 
*
* 
To compile, use: g++ -Wall $(pkg-config --cflags --libs gstreamer-0.10 gstreamer-app-0.10 ) FILE_NAME.c -o EXECUTABLE_NAME

Author: Pedro A. Zamuner <pedrozamuner@gmail.com>
November, 2010.
*
*/

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <cv.h>	      /* for OpenCV basic structures for ex.: IPLImage */
#include <highgui.h>  /* for OpenCV functions like cvCvtColor */
#include <iostream>   

using namespace cv;
using namespace std;

static GMainLoop *loop       = NULL;
static GstElement *pipeline1 = NULL;
static GstElement *pipeline2 = NULL;

static int IMG_width  = 400;
static int IMG_height = 300;

static IplImage *img;
static uchar *IMG_data;
CascadeClassifier cascade, nestedCascade;
double scale = 4;

//AppSrc name
static const char app_src_name[]  = "app-src_01";

//AppSink name
static const char app_sink_name[] = "app-sink_01";

//input filename
//static const char in_file_name[] = "video_ENC.avi";
//output filename
//static const char out_file_name[] = "video_PROC.avi";

//temp
//static int x = 0;
//static char imgName[256];


//OpenCV detecting function
static void detectAndDraw(IplImage *img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, double scale);

static GstFlowReturn new_buffer (GstAppSink *app_sink, gpointer user_data)
{	
	GstBuffer *buffer = gst_app_sink_pull_buffer( (GstAppSink*) gst_bin_get_by_name( GST_BIN(pipeline1), app_sink_name));

	g_debug("appsink: buffer timestamp(%llu)\t duration(%llu)\t offset(%llu)\t size(%d)\n", GST_BUFFER_TIMESTAMP(buffer), GST_BUFFER_DURATION(buffer), 	GST_BUFFER_OFFSET(buffer), GST_BUFFER_SIZE(buffer) );

	//debugging
	if (!gst_bin_get_by_name( GST_BIN(pipeline1), app_sink_name))
	{
		g_print("app-sink não está disponível!\n");
	}
	if (!gst_bin_get_by_name( GST_BIN(pipeline2), app_src_name))
	{
		g_print("app-src não está disponível!\n");
	}

	//processing...
	//handle imageData for processing
	IMG_data = (uchar*) img->imageData;

	// copies AppSink buffer data to the uchar vector of IplImage */
	memcpy(IMG_data, GST_BUFFER_DATA(buffer), GST_BUFFER_SIZE(buffer));

	// Image buffer is RGB, but OpenCV handles it as BGR, so channels R and B must be swapped */
	cvConvertImage(img,img,CV_CVTIMG_SWAP_RB);
	//cvCvtColor(img,img,CV_RGB2BGR);  

	//detectar a imagem e desenhar...
	detectAndDraw(img, cascade, nestedCascade, scale);

	// colocar uma tag para salvar imagens?? talvez via linha de comando??
	//x++;
	//sprintf(imgName,"frame-%d.bmp",x);
	//cvSaveImage(imgName,img);

	// Image buffer is RGB, but OpenCV handles it as BGR, so channels R and B must be swapped */
	cvConvertImage(img,img,CV_CVTIMG_SWAP_RB);
	//cvCvtColor(img,img,CV_RGB2BGR);  
	
	//copia a imagem de volta para o buffer, para ser enviado ao pipeline2
	memcpy(GST_BUFFER_DATA(buffer),IMG_data, GST_BUFFER_SIZE(buffer));


	//pushes the buffer to AppSrc, it takes the ownership of the buffer. you do not need to unref
	gst_app_src_push_buffer( GST_APP_SRC( gst_bin_get_by_name(GST_BIN(pipeline2),app_src_name)) , buffer);
	
	return GST_FLOW_OK;
}


static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
	gchar *userdata = (gchar *) data;

	switch(GST_MESSAGE_TYPE(msg))
	{
		case GST_MESSAGE_EOS:
		{
			//sender check - pipeline1 sends a EOS msg to AppSrc in pipeline2
			if ( g_ascii_strcasecmp(userdata, gst_element_get_name(pipeline1)) == 0)
			{
				g_print("EOS detected (%s)\n",userdata);
				gst_app_src_end_of_stream( GST_APP_SRC( gst_bin_get_by_name( GST_BIN(pipeline2), app_src_name ) ) );
			}
 			//sender check - when pipeline2 sends the EOS msg, quite.
			if ( g_ascii_strcasecmp(userdata, gst_element_get_name(pipeline2)) == 0)
			{
				g_print("Finished playback (%s)\n",userdata);
				g_main_loop_quit(loop);
	
			}
			break;
		}

		case GST_MESSAGE_ERROR:
		{
			gchar *debug;
			GError *error;
			
			gst_message_parse_error(msg, &error, &debug);
			g_free(debug);

			g_printerr("Error in pipeline:%s\nError Message: %s\n", userdata, error->message);
			g_error_free(error);

			g_main_loop_quit(loop);
			break;
		}

		case GST_MESSAGE_STATE_CHANGED : 
		{
        	GstState oldstate;
        	GstState newstate;
        	GstState pending;

        	gst_message_parse_state_changed (msg,&oldstate,&newstate,&pending);

        	g_debug("pipeline:%s old:%s new:%s pending:%s", userdata,
												gst_element_state_get_name(oldstate),
        	                   	                gst_element_state_get_name(newstate),
        	                      	            gst_element_state_get_name(pending));
    	 	break;
		}

		case GST_MESSAGE_WARNING: 
		{
			gchar *debug;
			GError *error;

        	gst_message_parse_warning (msg,&error,&debug);
			g_warning("pipeline:%s",userdata);
        	g_warning("debug: %s", debug);
	        g_warning("error: %s", error->message);
    	    g_free (debug);
    	    g_error_free (error);
    		break;
		}

		default:
			break;
	}
	return TRUE;
}

int main(int argc, char **argv)
{
	// GStreamer stuff...
	GError *error = NULL;
	GstBus *bus = NULL;
	GstAppSinkCallbacks callbacks;
    gchar pipeline1_str[256];
    gchar pipeline2_str[256];	
	char input_filename[256];
	char output_filename[256];

	// extract basename from filename.
	// --------------------------------
	if (argc < 2)
	{
		printf("Usage ./opencv-test <video file name>\n");
		return -1;

	}
	strcpy(input_filename,argv[1]);
	char video_basename[256];
	char video_extension[10];
	char *extension;
	char *separator;
	strcpy(video_basename, input_filename);
        extension = (char *) strrchr(video_basename, '.');
        if(extension != NULL)
		{
			strcpy(video_extension,extension);
			 *extension = '\0';
		}
	separator = (char *) rindex(video_basename, '/');
	if(separator != NULL)
	{
		separator++;
		strcpy(video_basename, separator);
	}

	//configuring output file name
	sprintf(output_filename, "%s-PROC%s",video_basename,video_extension);

	//OpenCV stuff...
	int nChannels = 3;
	img = cvCreateImage( cvSize(IMG_width,IMG_height), IPL_DEPTH_8U, nChannels);

	String cascadeName = "haarcascade_frontalface_alt.xml";
	String nestedCascadeName = "haarcascade_eye_tree_eyeglasses.xml";

	if(!nestedCascade.load(nestedCascadeName))
	{
		cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;	
		return -1;
	}

    if(!cascade.load(cascadeName))
    {
        cerr << "ERROR: Could not load classifier cascade" << endl;
        return -1;
    }


	// Initializing GStreamer
	g_print("Inicializando GStreamer.\n");
	gst_init(&argc, &argv);
	
	g_print("Creating Main Loop.\n");
	loop = g_main_loop_new(NULL,FALSE);


	//configuring pipeline parameters string
	// obs.: try g_strdup_printf
	int res = 0;
	res = sprintf(pipeline1_str, "filesrc location=\"%s\" ! avidemux name=demux demux.video_00 ! queue ! xviddec ! videorate ! videoscale ! ffmpegcolorspace ! video/x-raw-rgb, width=%d, height=%d !  appsink name=\"%s\"", input_filename, IMG_width, IMG_height, app_sink_name);
	if (res < 0)
	{
		g_printerr("Error configuring pipeline1's string\n");
		return -1;
	}
	
	res = sprintf(pipeline2_str, "appsrc name=\"%s\" ! queue ! videoparse format=14 width=%d height=%d framerate=25/1 ! videorate ! videoscale ! ffmpegcolorspace ! video/x-raw-rgb, width=%d, height=%d ! xvidenc ! avimux ! filesink location=\"%s\"", app_src_name, IMG_width, IMG_height, IMG_width, IMG_height, output_filename); 
	if (res < 0)
	{
		g_printerr("Error configuring pipeline2's string \n");
		return -1;
	}

	//debugging
	g_print("%s\n",pipeline1_str);
	//creating pipeline1
	pipeline1 = gst_parse_launch(pipeline1_str, &error);	

	if (error) 
	{
		g_printerr("Error [%s]\n",error->message);
        return -1;
	}

	
	//debugging
	g_print("%s\n",pipeline2_str);
	//creating pipeline2
	pipeline2 = gst_parse_launch(pipeline2_str, &error);

	if (error) 
	{
		g_printerr("Error [%s]\n",error->message);
        return -1;
	}

	if (!gst_bin_get_by_name( GST_BIN(pipeline1), app_sink_name))
	{
		g_printerr("Error creating app-sink\n");
		return -1;
	}


	if (!gst_bin_get_by_name( GST_BIN(pipeline2), app_src_name))
	{
		g_printerr("error creating app-src\n");
		return -1;
	}

	// Adding msg handler to Pipeline1
	g_print("Adding msg handler to %s\n", gst_element_get_name(pipeline1));
	bus = gst_pipeline_get_bus( GST_PIPELINE(pipeline1) );
	gst_bus_add_watch(bus, bus_call, gst_element_get_name(pipeline1));
	gst_object_unref(bus);
	
	// Adding msg handler to Pipeline1
	g_print("Adding msg handler to %s\n",gst_element_get_name(pipeline2));
	bus = gst_pipeline_get_bus( GST_PIPELINE(pipeline2) );
	gst_bus_add_watch(bus, bus_call, gst_element_get_name(pipeline2));
	gst_object_unref(bus);

	//configuring AppSink's callback  (Pipeline1)
	callbacks.eos = NULL;
	callbacks.new_preroll = NULL;
	callbacks.new_buffer = new_buffer;
	gst_app_sink_set_callbacks( (GstAppSink*) gst_bin_get_by_name(GST_BIN(pipeline1), app_sink_name), &callbacks, NULL, NULL);


	//Set the pipeline to "playing" state 
	g_print("Setting pipeline1's state to \"playing\".\n");
	gst_element_set_state(pipeline1, GST_STATE_PLAYING);

	//Set the pipeline2 to "playing" state 
	g_print("Setting pipeline2's state to \"playing\".\n");
	gst_element_set_state(pipeline2, GST_STATE_PLAYING);
	
	// Iterate
	g_print("Running...\n");
	g_main_loop_run(loop);

	// Out of the main loop, clean up nicely 
	g_print("Stopping playback - pipeline1\n");
	gst_element_set_state(pipeline1, GST_STATE_NULL);

	g_print("Stopping playback - pipeline2\n");
	gst_element_set_state(pipeline2, GST_STATE_NULL);

	g_print("Deleting pipeline1.\n");
	gst_object_unref( GST_OBJECT(pipeline1) );

	g_print("Deleting pipeline2.\n");
	gst_object_unref( GST_OBJECT(pipeline2) );

	//deleting image
	cvReleaseImage(&img);

	//unref mainloop
	g_main_loop_unref(loop);

    return 0;
}

static void detectAndDraw(IplImage *img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, double scale)
{

	Mat frame = img;
/*
	Mat frameCopy;
	if( frame.empty() )
		printf(" frame vazio\n");
	if( img->origin == IPL_ORIGIN_TL )
		frame.copyTo( frameCopy );
	else
		flip( frame, frameCopy, 0 );
*/


    int i = 0;
    double t = 0;
    vector<Rect> faces;
    const static Scalar colors[] =  { CV_RGB(0,0,255),
        CV_RGB(0,128,255),
        CV_RGB(0,255,255),
        CV_RGB(0,255,0),
        CV_RGB(255,128,0),
        CV_RGB(255,255,0),
        CV_RGB(255,0,0),
        CV_RGB(255,0,255)} ;
    Mat gray, smallImg( cvRound (frame.rows/scale), cvRound(frame.cols/scale), CV_8UC1 );

	/* pré processamento */
    cvtColor( frame, gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
	/* detecção */ 
    cascade.detectMultiScale( smallImg, faces,
        1.1, 2, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        |CV_HAAR_SCALE_IMAGE
        ,
        Size(30, 30) );
    t = (double)cvGetTickCount() - t;

	/* debug */
	if ( faces.empty() )
		printf("**************************************************** VETOR VAZIO ************ \n");	

    printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );


    /* drawing part... */ 
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = colors[i%8];
        int radius;
        center.x = cvRound((r->x + r->width*0.5)*scale);
        center.y = cvRound((r->y + r->height*0.5)*scale);
        radius = cvRound((r->width + r->height)*0.25*scale);
        circle( frame, center, radius, color, 3, 8, 0 );

	if( nestedCascade.empty() )
            continue;
        smallImgROI = smallImg(*r);
        nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
            1.1, 2, 0
            //|CV_HAAR_FIND_BIGGEST_OBJECT
            //|CV_HAAR_DO_ROUGH_SEARCH
            //|CV_HAAR_DO_CANNY_PRUNING
            |CV_HAAR_SCALE_IMAGE
            ,
            Size(30, 30) );
        for( vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
        {
            center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
            center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
            radius = cvRound((nr->width + nr->height)*0.25*scale);
            circle( frame, center, radius, color, 3, 8, 0 );
        }
    }  


}

