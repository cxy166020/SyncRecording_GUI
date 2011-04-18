//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <csignal>

//=============================================================================
// PGR Includes
//=============================================================================
#include <ladybug.h>
#include <ladybuggeom.h>
#include <ladybugrenderer.h>
#include <ladybugstream.h>

//=============================================================================
// Linux/unix headers
//=============================================================================
#include <pthread.h>
#include <sys/time.h>

//=============================================================================
// OpenCV headers
//=============================================================================
#include <cv.h>
#include <highgui.h>

//=============================================================================
// Homebrew barrier synchonization for Cygwin
//=============================================================================
#ifdef __CYGWIN__ 
#define pthread_barrier_t barrier_t
#define pthread_barrier_attr_t barrier_attr_t
#define pthread_barrier_init(b,a,n) barrier_init(b,n)
#define pthread_barrier_destroy(b) barrier_destroy(b)
#define pthread_barrier_wait(b) barrier_wait(b)
#endif

struct barrier_t{
  int needed;
  int called;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
} ;

int barrier_init(barrier_t *barrier,int needed)
{
  barrier->needed = needed;
  barrier->called = 0;
  pthread_mutex_init(&barrier->mutex,NULL);
  pthread_cond_init(&barrier->cond,NULL);
  return 0;
}

int barrier_destroy(barrier_t *barrier)
{
  pthread_mutex_destroy(&barrier->mutex);
  pthread_cond_destroy(&barrier->cond);
  return 0;
}

int barrier_wait(barrier_t *barrier)
{
  pthread_mutex_lock(&barrier->mutex);
  barrier->called++;
  if (barrier->called == barrier->needed) {
    barrier->called = 0;
    pthread_cond_broadcast(&barrier->cond);
  } else {
    pthread_cond_wait(&barrier->cond,&barrier->mutex);
  }
  pthread_mutex_unlock(&barrier->mutex);
  return 0;
}


#define COLOR_PROCESSING_METHOD   LADYBUG_DOWNSAMPLE4     // The fastest color method suitable for real-time usages
//#define COLOR_PROCESSING_METHOD   LADYBUG_HQLINEAR          // High quality method suitable for large stitched images

#define DATA_FORMAT LADYBUG_DATAFORMAT_COLOR_SEP_SEQUENTIAL_JPEG

#define IMAGE_IN_BUFFER 16

// The size of the stitched image
#define PANORAMIC_IMAGE_WIDTH    2048
#define PANORAMIC_IMAGE_HEIGHT   1024 

unsigned int* pImageDataSize;
unsigned int* pImWidth;
unsigned int* pImHeight;


// Number of threads
// #define THREADS 2

int THREADS = 0;

// This variable is set to 0 when user type Ctrl-c 
int execute = 1;

LadybugContext* context = NULL;
LadybugImage* image = NULL;

pthread_t* cam_threads;
pthread_barrier_t barr;

std::ofstream* pOfm;
std::string* pOutputName;

void CleanUp()
{
  for(int i=0; i<THREADS; i++)
    {
      if(context[i])
	{
	  ladybugStop( &(context[i]) );
	  ladybugDestroyContext( &(context[i]) );
	}
    }
	
  if(context) 
    {
      delete[] context;
      context = NULL;
    }
  if(image)
    {
      delete[] image;
      image = NULL;
    }

  pthread_barrier_destroy(&barr);

  for(int i=0; i<THREADS; i++)
    {
      pOfm[i].close();
    }

  if(cam_threads)    delete[] cam_threads;
  if(pOfm)           delete[] pOfm;
  if(pOutputName)    delete[] pOutputName;
  if(pImageDataSize) delete[] pImageDataSize;
  if(pImWidth)       delete[] pImWidth;
  if(pImHeight)      delete[] pImHeight;
}

void trap(int signal)
{
  execute = 0;
}

void HANDLE_ERROR(LadybugError error)
{
  if( error != LADYBUG_OK )
    {
      printf( "Error! Ladybug library reported %s\n", ::ladybugErrorToString( error ) );

      CleanUp();

      exit(1);
    }
}

//
// This function reads an image from camera
//
LadybugError startCamera(int CamIdx)
{
  LadybugError	     error;
  LadybugCameraInfo  caminfo;
	
  int retry = 10;

  //
  // create ladybug context
  //
  printf( "Creating ladybug context...\n" );
  error = ladybugCreateContext( &(context[CamIdx]) );
  if ( error != LADYBUG_OK )
    {
      printf( "Failed creating ladybug context. Exit.\n" );
      return error;
    }

  //
  // Initialize camera
  //
  printf( "Initializing camera...\n" );
  error = ladybugInitializeFromIndex( context[CamIdx], CamIdx );
  if( error != LADYBUG_OK ) return error;

  //
  // Get camera info
  //
  printf( "Getting camera info...\n" );
  error = ladybugGetCameraInfo( context[CamIdx], &caminfo );
  if( error != LADYBUG_OK ) return error;

  //
  // Load config file from the camera head itself
  //
  printf( "Load configuration file...\n" );
  error = ladybugLoadConfig( context[CamIdx], NULL);
  if( error != LADYBUG_OK ) return error;

  //
  // Set the panoramic view angle
  //
  error = ladybugSetPanoramicViewingAngle( context, LADYBUG_FRONT_0_POLE_5);
  if( error != LADYBUG_OK ) return error;
	
  // 
  // Make the rendering engine use the alpha mask
  //
  error = ladybugSetAlphaMasking( context[CamIdx], true );
  if( error != LADYBUG_OK ) return error;

  //
  // Set color processing method.
  //
  error = ladybugSetColorProcessingMethod( context[CamIdx], COLOR_PROCESSING_METHOD );
  if( error != LADYBUG_OK ) return error;     

  //
  // Configure output images in Ladybug liabrary
  //
  printf( "Configure output images in Ladybug library...\n" );
  error = ladybugConfigureOutputImages( context, LADYBUG_PANORAMIC );
  if( error != LADYBUG_OK ) return error;

  error = ladybugSetOffScreenImageSize(
				       context, LADYBUG_PANORAMIC,  
				       PANORAMIC_IMAGE_WIDTH, PANORAMIC_IMAGE_HEIGHT );  
  if( error != LADYBUG_OK ) return error;

  //
  // Auto buffer usage feature
  //
  error = ladybugSetAutoJPEGQualityControlFlag(context[CamIdx], true);
  if( error != LADYBUG_OK ) return error;     

  // 
  // Do not wait if there is no image waiting
  //
  // ladybugSetGrabTimeout(context[CamIdx], 0);

  switch( caminfo.deviceType )
    {
    case LADYBUG_DEVICE_COMPRESSOR:
    case LADYBUG_DEVICE_LADYBUG3:
      printf( "Starting Ladybug camera...\n" );
      error = ladybugStart(
			   context[CamIdx],
			   LADYBUG_RESOLUTION_ANY,
			   DATA_FORMAT);
      break;

    case LADYBUG_DEVICE_LADYBUG:
    default:
      printf( "Unsupported device.\n");
      error = LADYBUG_FAILED;
    }
  if( error != LADYBUG_OK ) return error;
	
  //
  // Grab an image to inspect the image size
  //
  error = LADYBUG_FAILED;
  while ( error != LADYBUG_OK && retry-- > 0)
    {
      error = ladybugGrabImage( context[CamIdx], &(image[CamIdx]) ); 
    }
  if( error != LADYBUG_OK ) return error;

  pImageDataSize[CamIdx] = image[CamIdx].uiDataSizeBytes;
  pImWidth[CamIdx]       = image[CamIdx].uiCols;
  pImHeight[CamIdx]      = image[CamIdx].uiRows;

  return LADYBUG_OK;
}

void* CameraCtrl(void* idx);

//=============================================================================
// Main Routine
//=============================================================================
int main( int argc, char* argv[] )
{
  int ArgCount = 1;
  THREADS = atoi(argv[ArgCount++]);

  pOutputName = new std::string[THREADS];
  for(int i=0; i<THREADS; i++)
    {
      pOutputName[i] = argv[ArgCount++];
    }

  pImageDataSize = new unsigned int[THREADS];
  pImWidth       = new unsigned int[THREADS];
  pImHeight      = new unsigned int[THREADS];

  context = new LadybugContext[THREADS];
  image   = new LadybugImage[THREADS];
	
  memset(context, 0, THREADS*sizeof(LadybugContext));
  memset(context, 0, THREADS*sizeof(LadybugImage));

  cam_threads = new pthread_t[THREADS];
  pOfm = new std::ofstream[THREADS];

  printf("Number of cameras: %d\n", THREADS);
  for(int i=0; i<THREADS; i++)
    {
      printf("Output file name: %s\n", pOutputName[i].c_str());
    }

  for(int i=0; i<THREADS; i++)
    {
      pOfm[i].open(pOutputName[i].c_str(), std::ios::binary | std::ios::trunc);
      if(!pOfm[i].good())
	{
	  printf("Error creating output file!\n");
	  CleanUp();
	  exit(1);
	}
    }
	
  // Barrier Synchronization
  if( pthread_barrier_init(&barr, NULL, THREADS) )
    {
      printf("Could not initialize pthread barrier");
      CleanUp();
      exit(1);
    }

  LadybugError error;
  bool ErrorDetected = false;
	
  //
  // Initialize and start the camera
  //
  for(int i=0; i<THREADS; i++)
    {
      error = startCamera(i);
		
      if( error!= LADYBUG_OK)
	{
	  ErrorDetected = true;
	  break;
	}
    }

  if(ErrorDetected)
    HANDLE_ERROR(error);

  signal(SIGINT, &trap);
	
  for(int i=0; i<THREADS; i++)
    {
      pthread_create(&(cam_threads[i]), NULL, CameraCtrl, &i);
    }

  for(int i=0; i<THREADS; i++)
    {
      pthread_join(cam_threads[i], NULL);
    }
	
  signal(SIGINT, SIG_DFL);

  printf("Cleaning up");
  //
  // clean up
  //
  CleanUp();

  return 0;
}


void* CameraCtrl(void* idx)
{
  LadybugError error;
  int CamIdx = *((int*)idx);
 
  unsigned int    uiRawCols = 0;
  unsigned int    uiRawRows = 0;

  //
  // Set the size of the image to be processed
  //
  if( ( COLOR_PROCESSING_METHOD == LADYBUG_DOWNSAMPLE4) || ( COLOR_PROCESSING_METHOD == LADYBUG_MONO))
    {
      uiRawCols = pImageWidth[CamIdx]/2;
      uiRawRows = pImageHeight[CamIdx]/2;
    }
  else
    {
      uiRawCols = pImageWidth[CamIdx]/2;
      uiRawRows = pImageHeight[CamIdx]/2;
    }

  // Six buffers to hold the color processed (RGB) images
  unsigned char*  arpTextureBuffers[ LADYBUG_NUM_CAMERAS ] = 
    { NULL,NULL,NULL,NULL,NULL,NULL };

  //
  // Initialize alpha mask size - this can take a long time if the
  // masks are not present in the current directory.
  //
  printf( "Initializing alpha masks (this may take some time)...\n" );
  error = ladybugInitializeAlphaMasks( context, uiRawCols, uiRawRows );
  if(error != LADYBUG_OK) printf("Camera %d error initializing alpha masks:", error);


  // error = ladybugSetPanoramicMappingType( context, LADYBUG_MAP_CYLINDRICAL);
  error = ladybugSetPanoramicMappingType( context, LADYBUG_MAP_RADIAL);
  if(error != LADYBUG_OK) printf("Camera %d error setting panoramic mapping type", error);

  //
  // Allocate the image buffers that hold the color-processed 
  // images for all cameras
  //
  for( i = 0; i < LADYBUG_NUM_CAMERAS; i++)
    {
      arpTextureBuffers[ i ] = new unsigned char[ uiRawCols * uiRawRows * 4  ];
    } 

  IplImage* StitchedIm = cvCreateImageHeader(cvSize(PANORAMIC_IMAGE_WIDTH,PANORAMIC_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);

  std::string WinName = sprintf("%d", CamIdx);

  // Create image window
  cvNamedWindow( WinName.c_str(), CV_WINDOW_AUTOSIZE );

  while(execute)
    {
      int rc = pthread_barrier_wait(&barr);
      if(rc != 0)
	{
	  printf("Could not wait on barrier\n");
	}
		
      // Grab an image from the camera
      error = ladybugGrabImage( context[CamIdx], &(image[CamIdx]) ); 
      if(error != LADYBUG_OK) printf("Camera %d error grabbing image:", error);
      
      // Convert the image to 6 RGB buffers
      error = ladybugConvertToMultipleBGRU32( context, &image, arpTextureBuffers, NULL );
      if(error != LADYBUG_OK) printf("Camera %d error converting image:", error);

      // Send the RGB buffers to the graphics card
      error = ladybugUpdateTextures( context, LADYBUG_NUM_CAMERAS, (const unsigned char**)arpTextureBuffers);
      if(error != LADYBUG_OK) printf("Camera %d error updaing textures:", error);
      
      // Stitch the images (inside the graphics card) and retrieve the output to the user's memory
      error = ladybugRenderOffScreenImage(context, LADYBUG_PANORAMIC, &StitchedIm);
      if(error != LADYBUG_OK) printf("Camera %d error stitching image:", error);

      cvShowImage(WinName.c_str(), StitchedIm);
    }

  printf("Cleaning up camera thread...\n");

  delete[] DataBuffer;  
  for( i = 0; i < LADYBUG_NUM_CAMERAS; i++)
    {
      if( arpTextureBuffers[ i ] != NULL )
	{
	  delete [] arpTextureBuffers[ i ];
	} 
    }

  cvReleaseImageHeader(&StitchedIm);

  cvDestroyWindow( WinName.c_str());
  
  pthread_exit(NULL);
}
