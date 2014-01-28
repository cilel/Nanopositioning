/****************************************************************************/

// The main fonction of Nanopositioning

/****************************************************************************/
//#include <math.h>

#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
//#include <visp/vpRobotCamera.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

//#include <visp/vpFeatureLuminance.h>
#include "npFeatureLuminance.h"
#include <visp/vpParseArgv.h>

//#include <visp/vpImageSimulator.h>
#include "npImageSimulator.h"
#include <stdlib.h>

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpPlot.h>

#include <visp/vpNoise.h>
#include <visp/vpImageConvert.h>
#include <visp/vpExponentialMap.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
//#include <netdb.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

//#include <string.h>

#define PORT 1085
#define SRV_IP "127.0.0.1"

#define MU 0.04


// List of allowed command line options
#define GETOPTARGS	"cdi:n:hp:bu:a"


using namespace std;

typedef enum {
     perspective,
     parallel, //donot control translation Z
     parallelZ // control translation Z
 }projectionModel;

projectionModel pjModel;

typedef enum {
     Gauss_statistic,// gauss noise
     Gauss_dynamic // gauss noise in real time
 }noiseModel;

noiseModel nsModel;

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param niter : Number of iterations.

*/
void usage(const char *name, const char *badparam, std::string ipath, int niter)
{
  fprintf(stdout, "\n\
Tracking of Surf key-points.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-c] [-d] [-n <number of iterations>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -a\n\
     Add gauss noise to image.\n\
\n\
  -n %%d                                               %d\n\
     Number of iterations.\n\
\n\
  -h\n\
     Print the help.\n",
      ipath.c_str(), niter);



  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \param niter : Number of iterations.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath,
                bool &click_allowed, bool &display, int &niter, bool &add_noise,bool &blur, double &scale)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg; break; 
    case 'n': niter = atoi(optarg); break;
    case 'a': add_noise = true;  break;
    case 'b': blur = true;  break;
    case 'p':
        if(!strcmp( optarg, "PERS" ))
            pjModel = perspective;
        else if (!strcmp( optarg,"PARA" ))
            pjModel = parallel;
        else if (!strcmp( optarg,"PARZ" ))
            pjModel = parallelZ;
        else
            pjModel = perspective;
        break;
    case 'u':
        if(!strcmp( optarg,"m" ))
            scale = 1;
        else if (!strcmp( optarg, "dm" ))
            scale = 10;
        else if (!strcmp( optarg,"cm" ))
            scale = 100;
        else if (!strcmp( optarg,"mm" ))
            scale = 1000;
        else if (!strcmp( optarg,"um" ))
            scale = 1000000;
        else if (!strcmp( optarg,"nm" ))
            scale = 1000000000;
        else // m
            scale = 1;
        break;
    case 'h': usage(argv[0], NULL, ipath, niter); return false; break;
    default:
      usage(argv[0], optarg, ipath, niter);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, niter);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

void diep(const char *s)
  {
    perror(s);
    exit(1);
  }

int send_wMe(vpHomogeneousMatrix wMe, double scale)
{

    vpTranslationVector T;
    wMe.extract(T);
    vpThetaUVector R;
    wMe.extract(R);

    //cout << "Translation=\n"<<T << "\nRotation=\n" << R << endl;

     /* diep(), #includes and #defines like in the server */

    double Pose_send[6];
    Pose_send[0]=T[0]*1000/scale;//convert meter to milimeter
    Pose_send[1]=T[1]*1000/scale;
    Pose_send[2]=T[2]*1000/scale;
    Pose_send[3]=R[0];
    Pose_send[4]=R[1];
    Pose_send[5]=R[2];

    cout << "Pose_send(wMe)=\n" << Pose_send[0]<< " " << Pose_send[1]<< " " << Pose_send[2]<< " " << Pose_send[3]<< " " << Pose_send[4]<< " " << Pose_send[5] << endl;

       struct sockaddr_in si_other;
       int s, i, slen=sizeof(si_other);

       if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
         diep("socket");

       memset((char *) &si_other, 0, sizeof(si_other));
       si_other.sin_family = AF_INET;
       si_other.sin_port = htons(PORT);
       if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
         fprintf(stderr, "inet_aton() failed\n");
         exit(1);
       }

       if (sendto(s, Pose_send, sizeof(Pose_send), 0, (struct sockaddr *) &si_other, slen)==-1)
           diep("sendto()");
       close(s);

       vpTime::wait(600);

       return 0;

}

void getNoisedImage(vpImage<unsigned char> &Inoised, vpImage<unsigned char> Iin,double noise_mean, double noise_sdv)
{

    Inoised = Iin;
    //  Inoised.init(Itexture.getRows(),Itexture.getCols(),0);

           vpGaussRand noise(noise_sdv, noise_mean);
           for(int i=0; i< Iin.getCols(); i++)
               for(int j=0;j<Iin.getRows();j++)
               {
                   double gauss = noise();
                   double noised =(double) Iin[i][j] + gauss;
                   if (noised < 0)
                       Inoised[i][j] = 0;
                   else if (noised > 255)
                       Inoised[i][j] = 255;
                   else
                       Inoised[i][j] = noised;
               }
}

void getBlurImage(vpImage<unsigned char> &Iblur, vpImage<unsigned char> Iin,double Z)
{
    cv::Mat I,Ib;
    vpImageConvert::convert(Iin,I);
    cv::Size ksize;
    ksize.width = 19;
    ksize.height = 19;
    double a = 1e6;
    double standardZ = 0.007; // should be changed when Z (cMo[2][3]) changes
    double diffZ = abs(Z - standardZ);
    double sigmaX = diffZ * a;

    cv::GaussianBlur(I, Ib, ksize, sigmaX);
    vpImageConvert::convert(Ib,Iblur);

}


int
main(int argc, const char ** argv)
{
  //std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string filename;
  std::string readFileFlag;
  bool opt_click_allowed = true;
  bool opt_display = true;
  int opt_niter = 1000;
  bool add_noise = false;
  bool blur = false;
  double noise_mean = 0;
  double noise_sdv = 4;

  nsModel = Gauss_dynamic;

  double scale = 1;

  ipath = "/dev/shm/out.pgm";
  readFileFlag = "/dev/shm/flag";
  ofstream filecMo,fileVelociy,fileResidu,fileodMo,fileSg;
  filecMo.open ("../Result/Trajectory.txt");
  fileVelociy.open("../Result/Velocity.txt");
  fileResidu.open("../Result/Residual.txt");
  fileodMo.open("../Result/odMo.txt""");
  fileSg.open("../Result/Sg.txt""");

  //test-----------------------------------------------------------
/*
  vpHomogeneousMatrix mdcMo, mdwMo, mdwMc, mdTr;
  mdcMo.resize(4,4);

  mdcMo[0][0]=0.02847823517 ;
  mdcMo[0][1]=0.9582422756;
  mdcMo[0][2]= -0.284535993;
  mdcMo[0][3]=-3.676356648  ;
  mdcMo[1][0]=-0.9988781821;
  mdcMo[1][1]=0.03805426687;
  mdcMo[1][2]=0.02818244337;
  mdcMo[1][3]=5.221864745;
  mdcMo[2][0]=0.03783341727;
  mdcMo[2][1]=0.2834142092;
  mdcMo[2][2]=0.9582510207;
  mdcMo[2][3]=136.599819;
  mdcMo[3][0]=0;
  mdcMo[3][1]=0;
  mdcMo[3][2]=0;
  mdcMo[3][3]=1;

  mdcMo[0][0]=0.9636251428;
  mdcMo[0][1]=-0.02765467202;
  mdcMo[0][2]=-0.2658228795;
  mdcMo[0][3]=-2.569881888  ;
  mdcMo[1][0]=0.03823916077;
  mdcMo[1][1]=0.9986651107;
  mdcMo[1][2]=0.03472410154;
  mdcMo[1][3]=-0.7427281339;
  mdcMo[2][0]=0.2645077517;
  mdcMo[2][1]=0.9633963014 ;
  mdcMo[2][2]=0.9582510207;
  mdcMo[2][3]=116.3720561;
  mdcMo[3][0]=0;
  mdcMo[3][1]=0;
  mdcMo[3][2]=0;
  mdcMo[3][3]=1;

  //mdwMo.buildFrom(0,0,0.01672,0,0,0);

  mdTr.buildFrom(0,0,0,0,0,vpMath::rad(90));

  //mdcMo = mdTr*mdcMo;

  //mdwMc = mdwMo*mdcMo.inverse();


  vpThetaUVector mR;
  vpTranslationVector mT;
  mdcMo.extract(mR);
  mdcMo.extract(mT);

  cout << "mdcMo:\n" << mdcMo << endl;
  cout  << "R:"<< mR << "\n" << "T:" << mT << endl;*/
 //test-----------------------------------------------------------

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_click_allowed,
                 opt_display, opt_niter,add_noise,blur,scale) == false) {
    return (-1);
  }

  double Z = 0.1; // 0.02300; //0.001745*scale;//0.020962*scale

/*
  double ZcMo = 0.0223*scale;
  double Z =0.02231*scale;

  if ((ZcMo-Z)==0)
  {
      cout << "ZcMo and Z should be different." << endl;
      return 0;
  }*/

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;


  // Test if an input path is set
  if (opt_ipath.empty() && ipath.empty()){
    usage(argv[0], NULL, ipath, opt_niter);
    std::cerr << std::endl
              << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
              << std::endl
              << "  environment variable to specify the location of the " << std::endl
              << "  image path where test images are located." << std::endl << std::endl;
    exit(-1);
  }



  vpHomogeneousMatrix cMo,cMod,wMe,eMo,cMw,wMcR,wMc,wMo,Tr,cMe;
/*
  //realsem
  wMcR.buildFrom(0.37205*0.001*scale,-3.4779*0.001*scale,2.0962*0.01*scale,0.201,-0.036,1.962);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-3.55*0.001*scale,1.92167*0.01*scale,0,0,0);
*/
/*
  //perpendicular
  wMcR.buildFrom(0*0.001*scale,-3.55*0.001*scale,2.0962*0.01*scale,0,0,0);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-3.55*0.001*scale,1.92167*0.01*scale,0,0,0);
*/
/*
  //0sem new
  wMcR.buildFrom(0*0.001*scale,-0.13249*0.001*scale,2.62184*0.01*scale,0,0,0);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.92184*0.01*scale,0,0,0);
*/
/*
  //0sem old
  wMcR.buildFrom(0*0.001*scale,0*0.001*scale,2.62184*0.01*scale,0,0,0);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,0*0.001*scale,1.92184*0.01*scale,0,0,0);
*/
/*
  //perspective camera (simulation of optic microscope) with 30deg rotation around x axis
  wMcR.buildFrom(0*0.001*scale,-11.6795*0.001*scale,3.92167*0.01*scale,0.524,0,0);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.92184*0.01*scale,0,0,0);
*/
/*
  // usb camera
  wMcR.buildFrom(-10.1764*0.001*scale,12.6935*0.001*scale,10.672*0.01*scale,0.1,-0.151,-1.525);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.672*0.01*scale,0,0,0);
 */

  //camera usb
  wMcR.buildFrom(0*0.001*scale,-0.13249*0.001*scale,10.672*0.01*scale,0,0,0);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.92184*0.01*scale,0,0,0);

  cout << "scale=" << scale << endl;
  cout << "wMe_desired=\n" << wMe << endl;

  Tr.buildFrom(0,0,0,vpMath::rad(180),0,0);

  wMc = wMcR*Tr;

  cout << "wMc=\n" << wMc << endl;


  cMw = wMc.inverse();
  eMo = wMe.inverse() * wMo;
  //eMo.setIdentity();

  cMod = cMw * wMo;

  cMe = cMw * wMe;

  send_wMe(wMe,scale);//test!! to ensure the init pose of plateform

  vpHomogeneousMatrix cModR;

  cModR = wMcR.inverse()*wMo;

  //test
  vpThetaUVector R_cMod;
  cModR.extract(R_cMod);
  cout<< "Rot_cMod in deg=" << vpMath::deg(R_cMod[0])<< "\t"<< vpMath::deg(R_cMod[1])<< "\t"<< vpMath::deg(R_cMod[2])<<"\n";

  /*
      cout<< "Rot_cMod in deg=" << (R_cMod[0])<< "\t"<< (R_cMod[1])<< "\t"<< (R_cMod[2])<<"\n";

  vpHomogeneousMatrix M ; M =  cModR ;
  vpHomogeneousMatrix Mx180(0,0,0,M_PI,0,0) ;
  vpHomogeneousMatrix My180(0,0,0,0,M_PI,0) ;
  vpHomogeneousMatrix Mz180(0,0,0,0,0,M_PI) ;

  (M*Mx180).print() ; cout << endl ;
  (M*My180).print() ; cout << endl ;
  (M*Mz180).print() ; cout << endl ;

  (M*Mx180.inverse()).print() ; cout << endl ;
  (M*My180.inverse()).print() ; cout << endl ;
  (M*Mz180.inverse()).print() ; cout << endl ;

  R_cMod = M*Mz180 ;
  cout<< "Rot_cMod in deg=" << vpMath::deg(R_cMod[0])<< "\t"<< vpMath::deg(R_cMod[1])<< "\t"<< vpMath::deg(R_cMod[2])<<"\n";

*/

  cout << "cMod=\n"<< cMod << endl;


  vpImage<unsigned char> I,Id;
  filename = ipath;
  const char * filename_c = filename.c_str();

  if( remove( readFileFlag.c_str()) != 0 )
        perror( "Error deleting image file" );

  vpTime::wait(100);

  if(ifstream(filename_c))
  {
      vpImageIo::read(I,filename) ;
   //   if( remove( filename_c) != 0 )
   //      perror( "Error deleting image file" );
  }
  else
      perror( "desired image dose not exist" );

  //std::cout << I << std::endl;

  if(blur)
      getBlurImage(I,I,cMod[2][3]);
    cout << "cMod[2][3]=" << cMod[2][3] << endl;
  if (add_noise)
    getNoisedImage(I,I,noise_mean,noise_sdv);

  Id = I;

  int Iw, Ih;//size of image
  Iw = Id.getWidth();
  Ih = Id.getHeight();

  //nomally one
  vpCameraParameters cam;
 /* if((pjModel == parallel) || ( pjModel == parallelZ))
        cam.initPersProjWithoutDistortion(1817636/scale, 1818494/scale, (int)Iw/2, (int)Ih/2);
  else
        cam.initPersProjWithoutDistortion(44708, 44708, (int)Iw/2, (int)Ih/2);
*/
  cam.initPersProjWithoutDistortion(4800, 4800, (int)Iw/2, (int)Ih/2);// camera usb in simulation
 // cam.initPersProjWithoutDistortion(100000, 100000, (int)Iw/2, (int)Ih/2);// camera usb in simulation

//  vpCameraParameters cam(4011, 3927, (int)Iw/2, (int)Ih/2);//perspective camera with 30deg rotaion around x axis, focal length in Blender = 0.25mm

//   vpCameraParameters cam(1817636/scale, 1818494/scale, (int)Iw/2, (int)Ih/2);//parallel parameters computed by calibration VVS 1817636/scale, 1818494/scale
 // vpCameraParameters cam(8984549/scale, 8955094/scale, (int)Iw/2, (int)Ih/2);//160, 120 parallel
 // vpCameraParameters cam(12237, 12237, (int)Iw/2, (int)Ih/2);//perspective parameters are 12237, 12237 (focal length in Blender = 0.2mm) regularized by computed calibration VVS 101325, 101452,
  // 24813, 24792 for focal length in Blender = 0.6mm, so the regularized perspective parameters is 44708, 44670

  // display the image
#if defined VISP_HAVE_X11
  vpDisplayX d;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI d;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK d;
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
    d.init(I, 1620, 10, "Photometric VS desired feature : s*") ;
    vpDisplay::display(I);
    vpDisplay::flush(I);
  }
  if (opt_display && opt_click_allowed) {
    std::cout << "Click in the image to continue..." << std::endl;
    vpDisplay::getClick(I) ;
  }
#endif


   vpColVector vm;//velocity to be sent for the init pose, in meter or specified unit
/*   vpHomogeneousMatrix vmH;

   vmH.buildFrom(0*scale,0*scale,0*scale,vpMath::rad(0),vpMath::rad(0),vpMath::rad(0));//velocity
   vmH = vmH*Tr;

   cout << "vmH=\n" << vmH << endl;

   vpTranslationVector Tv;
   vmH.extract(Tv);
   vpThetaUVector Rv;
   vmH.extract(Rv);
   vm[1]=Tv[1];
   vm[2]=Tv[2];
   vm[3]=Rv[0];
   vm[4]=Rv[1];
   vm[5]=Rv[2];*/

  vm.resize(6);
  vm[0]= 0.0002*scale;//velocity
 // vm[1]= 0.0002*scale;//velocity
//  vm[2]= 0.001*scale;
//  vm[3]=vpMath::rad(1);
 // vm[4]=vpMath::rad(1);
//  vm[5]=vpMath::rad(-3);

   // vpHomogeneousMatrix wMe_tmp =  wMe * vpExponentialMap::direct(vm,1);

  //  cout << "wMe_tmp=\n" <<  wMe_tmp << endl;

    cMe = cMe * vpExponentialMap::direct(vm,1);
    wMe = wMc * cMe;
    cMo = cMw * wMe * eMo ;

    cout << "vpExponentialMap::direct(vm,1)=\n" << vpExponentialMap::direct(vm,1) << endl;


    cout << "wMe_first=\n" << wMe << endl;
    cout << "cMe_first=\n" << cMe << endl;
    cout << "cMo_first=\n" << cMo << endl;


    send_wMe(wMe,scale);

    //vpTime::wait(1000);

    I.resize(0,0);

    if( remove( readFileFlag.c_str()) != 0 )
          perror( "Error deleting image file" );

    vpTime::wait(100);

    if(ifstream(filename_c))
    {
        vpImageIo::read(I,filename) ;
    //    if( remove( filename_c) != 0 )
    //       perror( "Error deleting image file" );
    }
    else
        perror( "current image dose not exist" );

    if(blur)
        getBlurImage(I,I,cMo[2][3]);
        cout << "cMo[2][3]=" << cMo[2][3] << endl;
    if (add_noise)
      getNoisedImage(I,I,noise_mean,noise_sdv);


//  cMo.buildFrom(0.000000*scale,0.000000*scale,ZcMo,vpMath::rad(-5),vpMath::rad(5),vpMath::rad(5));

/*
  //set the robot at the desired position
  sim.setCameraPosition(cMo);
  I = 0 ;
  sim.getImage(I,cam);  // and aquire the image I
  */

  
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
    d.init(I, 1620, 10, "Photometric VS current feature : s") ;
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;
  }
  if (opt_display && opt_click_allowed) {
    std::cout << "Click in the image to continue..." << std::endl;
    vpDisplay::getClick(I) ;
  }
#endif  

  vpImage<unsigned char> Idiff ;
  Idiff = I ;

  vpImageTools::imageDifference(I,Id,Idiff) ;

  // Affiche de l'image de difference
#if defined VISP_HAVE_X11
  vpDisplayX d1;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI d1;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK d1;
#endif
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
    d1.init(Idiff, 1640+(int)I.getWidth(), 10, "photometric visual servoing : s-s* ") ;
    vpDisplay::display(Idiff) ;
    vpDisplay::flush(Idiff) ;
  }
#endif
  // create the robot (here a simulated free flying camera)
  // ------------------------------------------------------
  // Visual feature, interaction matrix, error
  // s, Ls, Lsd, Lt, Lp, etc
  // ------------------------------------------------------

  // current visual feature built from the image 
  // (actually, this is the image...)

  npFeatureLuminance sI ;
  if(pjModel == parallel)
    sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallel) ;
  else if(pjModel == parallelZ)
    sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallelZ) ;
  else
    sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::perspective) ;
  sI.setCameraParameters(cam) ;
  sI.buildFrom(I) ;

  vpImage<unsigned char> Idip;

#if defined VISP_HAVE_X11
  vpDisplayX ds;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI ds;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK ds;
#endif

    vpImageConvert::convert(sI.imGxy,Idip);
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
  {
    ds.init(Idip, 2680, 10, "Gadient") ;
    vpDisplay::display(Idip);
    vpDisplay::flush(Idip);
  }
#endif


  // desired visual feature built from the image 
  npFeatureLuminance sId ;
  if(pjModel == parallel)
    sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallel) ;
  else if(pjModel == parallelZ)
    sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallelZ) ;
  else
    sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::perspective) ;
  sId.setCameraParameters(cam) ;
  sId.buildFrom(Id) ;

  
  // Matrice d'interaction, Hessien, erreur,...
  vpMatrix Lsd;   // matrice d'interaction a la position desiree
  vpMatrix Hsd;  // hessien a la position desiree
  vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
  vpColVector error ; // Erreur I-I*, photometric information

  vpMatrix Lgsd; // interaction matrix (using image gradient) of the desired position
  vpMatrix Hgsd; // hessien of the desired position (using image gradient)
  vpMatrix Hg;  // Hessien for  levenberg-Marquartd (using image gradient)
  vpColVector sg_error; // error sg-sg*, image gradient

  // Compute the interaction matrix
  // link the variation of image intensity to camera motion

  // here it is computed at the desired position
  sId.interaction(Lsd) ;
  cout << "Size of Lsd:" << Lsd.getRows() << "x" << Lsd.getCols() <<endl;
  //cout << "Lsd=\n" << Lsd <<endl;

  Lgsd = sId.get_Lg();
  cout << "Size of Lgsd:" << Lgsd.getRows() << "x" << Lgsd.getCols() <<endl;
  //cout << "Lsd=\n" << Lsd <<endl;

/*  vpVelocityTwistMatrix cVw;
  cVw.buildFrom(cMw);
  cout << "cVw=\n" << cVw <<endl;*/

  vpVelocityTwistMatrix cVe; //cVo
  cVe.buildFrom(cMe);
  //cVo.buildFrom(cMo);
//  cout << "cVo=\n" << cVo <<endl;

  vpMatrix Js;// visual feature Jacobian
  vpMatrix Jn;// robot Jacobian
  vpMatrix diagHsd;// diag(Hsd)

  /*For parallelZ, image gradient*/
  vpMatrix Jgs;// visual feature Jacobian
  vpMatrix Jgn;// robot Jacobian
  vpMatrix diagHgsd;// diag(Hsd)

  if(pjModel==parallel )
  {
      Jn.resize(6,5);
      Jn[0][0]=1;
      Jn[1][1]=1;
      Jn[3][2]=1;
      Jn[4][3]=1;
      Jn[5][4]=1;

      Js=-Lsd*cVe*Jn;
      //cout << "Lsd*cVw=\n" << Lsd*cVw << endl;

      //cout << "Jn=\n" << Jn << endl;
      cout << "Size of Jn:" << Jn.getRows() << "x" << Jn.getCols() <<endl;

      //cout << "Js=\n" << Js <<endl;
      cout << "Size of Js:" << Js.getRows() << "x" << Js.getCols() <<endl;

      // Compute the Hessian H = L^TL
      Hsd = Js.AtA() ;

      //cout << "Hsd=\n" << Hsd <<endl;

      // Compute the Hessian diagonal for the Levenberg-Marquartd
      // optimization process
      unsigned int n = 5 ;
      diagHsd.resize(n,n) ;
      diagHsd.eye(n);
      for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];
  }
  else if (pjModel==parallelZ)
  {
      Jn.resize(6,5);
      Jn[0][0]=1;
      Jn[1][1]=1;
      Jn[3][2]=1;
      Jn[4][3]=1;
      Jn[5][4]=1;

      Js=-Lsd*cVe*Jn;
      cout << "Size of Jn:" << Jn.getRows() << "x" << Jn.getCols() <<endl;
      cout << "Size of Js:" << Js.getRows() << "x" << Js.getCols() <<endl;

      Jgn.resize(6,1);
      Jgn[2][0]=1;
      Jgs=-Lgsd*cVe*Jgn;

      cout << "Size of Jgn:" << Jgn.getRows() << "x" << Jgn.getCols() <<endl;
      cout << "Size of Jgs:" << Jgs.getRows() << "x" << Jgs.getCols() <<endl;

      // Compute the Hessian H = L^TL
      Hsd = Js.AtA() ;

      Hgsd = Jgs.AtA();

      cout << "Hgsd=\n" << Hgsd <<endl;

      // Compute the Hessian diagonal for the Levenberg-Marquartd
      // optimization process
      unsigned int n = 5 ;
      diagHsd.resize(n,n) ;
      diagHsd.eye(n);
      for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];

      diagHgsd.resize(1,1);
      diagHsd[0][0] = Hsd[0][0];



  }
  else //perspective //or parallelZ
  {
      Jn.resize(6,6);
      Jn.setIdentity();
      Js=-Lsd*cVe*Jn;

      cout << "Size of Js:" << Js.getRows() << "x" << Js.getCols() <<endl;

      //cout << "Js=\n" << Js <<endl;

      // Compute the Hessian H = L^TL
      Hsd = Js.AtA() ;

      cout << "Hsd=\n" << Hsd <<endl;

      // Compute the Hessian diagonal for the Levenberg-Marquartd
      // optimization process
      unsigned int n = 6 ;
      diagHsd.resize(n,n) ;
      diagHsd.eye(n);
      for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];
  }

  // ------------------------------------------------------
  // Control law
  double lambda ; //gain
  vpColVector e ;// velocity to be multiply by lamda
  vpColVector v ; // camera velocity send to the robot
  vpColVector eg; // velocity of z axis
  vpColVector vg ; // camera velocity of z axis

  // ----------------------------------------------------------
  // Minimisation

  double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
  double lambdaGN;
  double lambdaTZ  = 50;// gain for Translation on Z : lambdaTZ*lambda

  mu       =  0.001;
  lambda   = 10 ;
  lambdaGN = 10;

  // set a velocity control mode 
  //robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  // ----------------------------------------------------------
  int iter   = 0;
  int iterGN = 90 ; // swicth to Gauss Newton after iterGN iterations

  vpPlot graphy(4, 600, 800, 1620, 1300, "Nanopositioning");
  vpPlot graphy2(2, 600, 400, 020, 1300, "Nanopositioning");

  graphy.initGraph(0,3);
  graphy.initGraph(1,3);
  graphy.initGraph(2,3);
  graphy.initGraph(3,3);

  graphy.setTitle(0,"odMo: m");
  graphy.setTitle(1,"odMo: deg");
  graphy.setTitle(2,"Velocity: m/s ");
  graphy.setTitle(3,"Velocity: deg/s");


  char unit[40];
  graphy2.initGraph(0,1);
  graphy2.initGraph(1,1);
  graphy2.setTitle(0,"Error");
  graphy2.setTitle(1,"Trajectory of object");
  strncpy( unit, "x", 40 );
  graphy2.setUnitX(1,unit);
  strncpy( unit, "y", 40 );
  graphy2.setUnitY(1,unit);
  strncpy( unit, "z", 40 );
  graphy2.setUnitZ(1,unit);
  graphy2.setColor(0,0,vpColor::red);


  char legend[40];
  strncpy( legend, "Norm Error", 40 );
  graphy2.setLegend(0,0,legend);
  strncpy( legend, "Tx", 40 );
  graphy.setLegend(0,0,legend);
  strncpy( legend, "Ty", 40 );
  graphy.setLegend(0,1,legend);
  strncpy( legend, "Tz", 40 );
  graphy.setLegend(0,2,legend);
  strncpy( legend, "Rx", 40 );
  graphy.setLegend(1,0,legend);
  strncpy( legend, "Ry", 40 );
  graphy.setLegend(1,1,legend);
  strncpy( legend, "Rz", 40 );
  graphy.setLegend(1,2,legend);
  strncpy( legend, "Tx", 40 );

  graphy.setLegend(2,0,legend);
  strncpy( legend, "Ty", 40 );
  graphy.setLegend(2,1,legend);
  strncpy( legend, "Tz", 40 );
  graphy.setLegend(2,2,legend);
  strncpy( legend, "Rx", 40 );
  graphy.setLegend(3,0,legend);
  strncpy( legend, "Ry", 40 );
  graphy.setLegend(3,1,legend);
  strncpy( legend, "Rz", 40 );
  graphy.setLegend(3,2,legend);

  
  double normError = 1000; // norm error = |I-I*|
  double normError_p = 0; // previous norm error
  double threshold=0.2;// condition of convergence
  double convergence_threshold = 0.05;

  // For gaussian noise n~N(m,d^2), E=(I+n)-(I+n')=n-n',E~N(0,2*d^2)
  if(add_noise && (nsModel == Gauss_dynamic))
      threshold += noise_sdv*1.414;

  cout<< "threshold=" << threshold << endl;

 // vpHomogeneousMatrix edMe,e/*dMw ;
  vpHomogeneousMatrix odMo ;

/******************************* Here the loop begins ************************************/
  do
  {

    std::cout << "--------------------------------------------" << iter++ << std::endl ;

    filecMo << iter;

    cMo = cMw * wMe * eMo;

    //cout << "cMoT: " ;

    for(int m=0;m<3;m++)
    {
        filecMo << "\t" << cMo[m][3]*1e6; //convert m to um
   //     cout << "\t" << cMo[m][3];
    }

    //cout << endl;
    filecMo << endl;

    //Acquire the new image
    //sim.setCameraPosition(cMo) ;
    //sim.getImage(I,cam) ;

    // Acquir new image from blender

    if( remove( readFileFlag.c_str()) != 0 )
          perror( "Error deleting image file" );

    vpTime::wait(100);

    if(ifstream(filename_c))
    {
        vpImageIo::read(I,filename) ;

      //  if( remove( filename_c) != 0 )
      //     perror( "Error deleting image file" );
    }
    else
        perror( "current image dose not exist" );

    odMo = cMod.inverse() * cMo;

    vpTranslationVector TodMo;
    odMo.extract(TodMo);
    vpThetaUVector RodMo;
    odMo.extract(RodMo);
    vpColVector TodMoPlot;
    TodMoPlot.resize(3);
    vpColVector RodMoPlot;
    RodMoPlot.resize(3);

    /*--------Here is plot in realtime and save data for gnuplot-------*/
    //convert m, rad to um, deg
    for(int i=0;i<3;i++)
        TodMoPlot[i]= TodMo[i]*1e6;
    for(int i=0;i<3;i++)
        RodMoPlot[i]= vpMath::deg(RodMo[i]);

    fileodMo << iter << "\t" << TodMoPlot.t() << RodMoPlot.t() << endl;

    for(int i=0;i<3;i++)
        graphy.plot(0,i,iter,TodMo[i]/scale);
    for(int i=0;i<3;i++)
        graphy.plot(1,i,iter,vpMath::deg(RodMo[i]));

    //cout<< "ZodMo=" << TodMo[2] << endl;

    if(blur)
        getBlurImage(I,I,cMo[2][3]);

    cout << "cMo[2][3]=" << cMo[2][3] << endl;
    if (add_noise && nsModel == Gauss_dynamic)
        getNoisedImage(I,I,noise_mean, noise_sdv);


#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
    if (opt_display) {
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
#endif
    vpImageTools::imageDifference(I,Id,Idiff) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
    if (opt_display) {
      vpDisplay::display(Idiff) ;
      vpDisplay::flush(Idiff) ;
    }
#endif

    // Compute current visual feature
    sI.buildFrom(I) ;

    int S_g=0; // image gradient
    vpColVector sg = sI.get_sg();
    int nbr_sg = sg.size();
    cout << "nbr_sg:" << nbr_sg << endl;

    for (int m=0; m<nbr_sg;m++)
        S_g+= sg[m];

    fileSg << iter << "\t" << S_g << "\t" << cMod[2][3]-cMo[2][3] << endl;

    // compute current error
    sI.error(sId,error) ;

    if (pjModel==parallelZ)
        sI.sg_error(sId.get_sg(),sg_error) ;

    vpImageConvert::convert(sI.imGxy,Idip);
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK)
  {
    vpDisplay::display(Idip);
    vpDisplay::flush(Idip);
  }
#endif

    if(iter == 1)
        vpImageIo::writePNG(Idiff,"../Result/img_init.png");

    normError_p = normError;

    normError = sqrt(error.sumSquare()/error.getRows());

    graphy2.plot(0,0,iter,normError);

    cout << "|e| "<< normError <<endl ;
    fileResidu << iter << "\t" << normError << endl;

    // double t = vpTime::measureTimeMs() ;

    // ---------- Levenberg Marquardt method --------------
    {
     /* if (normError<2+threshold)//iter > iterGN
      {
          cout << "LM method" << endl;
        mu = 0.0001 ;
        lambda = lambdaGN;
      }*/
      // Compute the levenberg Marquartd term
      {
        H = ((mu * diagHsd) + Hsd).inverseByLU();
      }
      //	compute the control law
      e = H * Js.t() *error ;

   //   vpMatrix Jp ;
   //   int rank ;
    //  rank = Js.pseudoInverse(Jp,1e-20) ;
    //  cout << "rank " << rank << endl ;

      v =  -lambda*e;

      if(pjModel==parallelZ)
      {
          Hg = ((mu * diagHgsd) + Hgsd).inverseByLU();
          eg = Hg * Jgs.t() * sg_error ;
          vg =  -lambda*lambdaTZ*eg;

         // cout << "vg=" << vg << endl;
      }


    }

    if(pjModel==parallel)
    {
        vpColVector vc=v;
        v.resize(6);
        v[5]=vc[4];
        v[4]=vc[3];//vc[3]
        v[3]=vc[2];//vc[2]
        v[2]=0;
        v[1]=vc[1];//vc[1]
        v[0]=vc[0];//vc[0]
    }
    else if (pjModel==parallelZ)
    {

        vpColVector vc=v;
        v.resize(6);
        v[5]=vc[4];
        v[4]=vc[3];//vc[3]
        v[3]=vc[2];//vc[2]
        v[2]=vg[0];
        v[1]=vc[1];//vc[1]
        v[0]=vc[0];//vc[0]
 /*
        vpColVector vc=v;
        v.resize(6);
        v[5]=0;
        v[4]=0;//vc[3]
        v[3]=0;//vc[2]
        v[2]=1e-5;
        v[1]=0;//vc[1]
        v[0]=0;//vc[0]
*/
    }

    if (vpMath::equal(normError,normError_p, convergence_threshold*10) && iter > 200 && lambdaTZ < 1000)
    {
        //lambdaTZ *= 2;
        cout << "----------- Z begin moving --------------" << endl;
    }

    for(int i=0;i<3;i++)
      graphy.plot(2,i,iter,v[i]/scale);
    for(int i=0;i<3;i++)
      graphy.plot(3,i,iter,vpMath::deg(v[i+3]));

    //convert m/s, rad/s to um/s, rad/s
    vpColVector vPlot=v;
      for(int i=0;i<3;i++)
          vPlot[i]=v[i]*1e6;

    fileVelociy << iter << "\t" << vPlot.t() << endl;

    cout << "v=" << v.t() << endl;
    cout << "lambda = " << lambda << "  mu = " << mu ;
    cout << " |Tc| = " << sqrt(v.sumSquare()) << endl;

    // send the robot velocity
    //robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    //cout << "cMo=\n" << cMo << endl;

    vpHomogeneousMatrix eV;
    vpThetaUVector vR;
    vR[0]=v[3]*MU;
    vR[1]=v[4]*MU;
    vR[2]=v[5]*MU;

    eV.insert(vR);
    eV[0][3]=v[0]*MU;
    eV[1][3]=v[1]*MU;
    eV[2][3]=v[2]*MU;

    cMe = cMe * eV;

    //wMe = wMe * eV;

    cout << "wMe1_current=\n" << wMe << endl;

    wMe = wMc * cMe;

    cout << "eV=\n" << eV << endl;

    cout << "wMe2_current=\n" << wMe << endl;

    send_wMe(wMe,scale);

    //cout << "eV=\n "<< eV << endl;

    //cMo =  cMo * vpExponentialMap::direct(v,0.04);

    //cout << "v ExpMap=\n" << vpExponentialMap::direct(v,0.04) << endl;

    cout << "cMo_new=\n" << cMo << endl;

    graphy2.plot(1,0,cMo[0][3]/scale,cMo[1][3]/scale,cMo[2][3]/scale);

   // cout << "cMoTT:" << cMo[0][3]/scale<<cMo[1][3]/scale<<cMo[2][3]/scale << endl;

    /*------------Here begin to save the image for video--------*/

    char img_filename_c[80];
    sprintf(img_filename_c, "../Result/video/img_current_%d.png", iter);
    string img_filename(img_filename_c);
    vpImageIo::writePNG(I,img_filename);
    sprintf(img_filename_c, "../Result/video/img_diff_%d.png", iter);
    string img_filename_d(img_filename_c);
    vpImageIo::writePNG(Idiff,img_filename_d);
    sprintf(img_filename_c, "../Result/video/img_gradient_%d.png", iter);
    string img_filename_g(img_filename_c);
    vpImageIo::writePNG(Idip,img_filename_g);

    /*-----------Here end to save the image for video---------*/

 //   if(iter > 1000)
 //       vpImageIo::writePNG(Idiff,"../Result/img_end.png");

  }
 while(normError > threshold  && iter < opt_niter && !(vpMath::equal(normError,normError_p, convergence_threshold) && normError < 1.5*threshold));
//while(1) ;

 vpImageIo::writePNG(Idiff,"../Result/img_end.png");

 filecMo.close();
 cout << "===========================END==============================" << endl;

  while(1)
       graphy2.plot(1,0,cMo[0][3]/scale,cMo[1][3]/scale,cMo[2][3]/scale);


  //vpDisplay::getClick(graphy.I);
  vpDisplay::getClick(graphy2.I);


}

