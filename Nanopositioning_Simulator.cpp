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
#include <visp/vpExponentialMap.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
//#include <netdb.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>

#define PORT 1085
#define SRV_IP "127.0.0.1"

#define MU 0.04


// List of allowed command line options
#define GETOPTARGS	"cdi:n:hp:bu:"


using namespace std;

typedef enum {
     perspective,
     parallel,
     weakperspective
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
  -b\n\
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
                bool &click_allowed, bool &display, int &niter, bool &add_noise, double &scale)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg; break; 
    case 'n': niter = atoi(optarg); break;
    case 'b': add_noise = true;  break;
    case 'p':
        if(!strcmp( optarg, "PERS" ))
            pjModel = perspective;
        else if (!strcmp( optarg,"PARA" ))
            pjModel = parallel;
        else
            pjModel = perspective;
        break;
    case 'u':
        if(!strcmp( optarg, "mm" ))
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

void diep(char *s)
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

    cout << "Pose_send=\n" << Pose_send[0]<< " " << Pose_send[1]<< " " << Pose_send[2]<< " " << Pose_send[3]<< " " << Pose_send[4]<< " " << Pose_send[5] << endl;

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


       vpTime::wait(500);

       return 0;

}

void getNoisedImage(vpImage<unsigned char> &Inoised, vpImage<unsigned char> in,double noise_mean, double noise_sdv)
{

    Inoised = in;
    //  Inoised.init(Itexture.getRows(),Itexture.getCols(),0);

           vpGaussRand noise(noise_sdv, noise_mean);
           for(int i=0; i< in.getCols(); i++)
               for(int j=0;j<in.getRows();j++)
               {
                   double gauss = noise();
                   double noised =(double) in[i][j] + gauss;
                   if (noised < 0)
                       Inoised[i][j] = 0;
                   else if (noised > 255)
                       Inoised[i][j] = 255;
                   else
                       Inoised[i][j] = noised;
               }
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
  int opt_niter = 40;
  bool add_noise = false;
  double noise_mean =0;
  double noise_sdv = 5;

  nsModel = Gauss_dynamic;

  double scale = 1;

  ipath = "/dev/shm/out.pgm";
  readFileFlag = "/dev/shm/flag";
  ofstream filecMo,fileVelociy,fileResidu;
  filecMo.open ("../Result/Trajectory.txt");
  fileVelociy.open("../Result/Velocity.txt");
  fileResidu.open("../Result/Residual.txt");

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_click_allowed,
                 opt_display, opt_niter,add_noise,scale) == false) {
    return (-1);
  }

  double Z =0.002*scale;//0.020962*scale

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



  vpHomogeneousMatrix cMo,cMod,wMe,eMo,cMw,wMcR,wMc,wMo,Tr;

  wMcR.buildFrom(0.37205*0.001*scale,-3.4779*0.001*scale,2.0962*0.01*scale,0.201,-0.036,1.962);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,-3.55*10e-3*scale,1.92167*0.01*scale,0,0,0);


/*
  wMcR.buildFrom(0*0.001*scale,0*0.001*scale,2.62184*0.01*scale,0,0,0);
  wMe.buildFrom(0*0.001*scale,-0.13249*0.001*scale,1.2293*0.01*scale,0,0,0);
  wMo.buildFrom(0*0.001*scale,0*0.001*scale,1.92184*0.01*scale,0,0,0);
*/
  cout << "scale=" << scale << endl;
  cout << "wMe_desired=\n" << wMe << endl;

  Tr.buildFrom(0,0,0,vpMath::rad(180),0,0);

  wMc = wMcR*Tr;

  cMw = wMc.inverse();
  eMo = wMe.inverse() * wMo;
  //eMo.setIdentity();

  cMod = cMw * wMo;

  //send_wMe(wMe,scale);//test

  //test
 /* vpThetaUVector R_cMod;
   cMod.extract(R_cMod);
   cout<< "R_cMod=" << R_cMod << endl;*/

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


  if (add_noise && (nsModel == Gauss_statistic))
    getNoisedImage(I,I,noise_mean,noise_sdv);

  Id = I;

  int Iw, Ih;//size of image
  Iw = Id.getWidth();
  Ih = Id.getHeight();

  vpCameraParameters cam(8984549/scale, 8955094/scale, (int)Iw/2, (int)Ih/2);//160, 120

  // display the image
#if defined VISP_HAVE_X11
  vpDisplayX d;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI d;p
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
   vm[0]=0.000001*scale;//velocity
 //  vm[1]=0.000001*scale;//velocity
 //  vm[5]=vpMath::rad(-0.1);

    wMe =  wMe * vpExponentialMap::direct(vm,1);
    cMo = cMw * wMe * eMo ;

    cout << "vpExponentialMap::direct(vm,1)=\n" << vpExponentialMap::direct(vm,1) << endl;

    cout << "wMe_first=\n" << wMe << endl;
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
  else
    sI.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::perspective) ;
  sI.setCameraParameters(cam) ;
  sI.buildFrom(I) ;
  

  // desired visual feature built from the image 
  npFeatureLuminance sId ;
  if(pjModel == parallel)
    sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::parallel) ;
  else
    sId.init( I.getHeight(), I.getWidth(), Z, npFeatureLuminance::perspective) ;
  sId.setCameraParameters(cam) ;
  sId.buildFrom(Id) ;
  
  // Matrice d'interaction, Hessien, erreur,...
  vpMatrix Lsd;   // matrice d'interaction a la position desiree
  vpMatrix Hsd;  // hessien a la position desiree
  vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
  vpColVector error ; // Erreur I-I*

  // Compute the interaction matrix
  // link the variation of image intensity to camera motion

  // here it is computed at the desired position
  sId.interaction(Lsd) ;
  cout << "Size of Lsd:" << Lsd.getRows() << "x" << Lsd.getCols() <<endl;
  //cout << "Lsd=\n" << Lsd <<endl;

/*  vpVelocityTwistMatrix cVw;
  cVw.buildFrom(cMw);
  cout << "cVw=\n" << cVw <<endl;*/

  vpVelocityTwistMatrix cVe; //cVo
  cVe.buildFrom(cMw * wMe);
  //cVo.buildFrom(cMo);
//  cout << "cVo=\n" << cVo <<endl;

  vpMatrix Js;// visual feature Jacobian
  vpMatrix Jn;// robot Jacobian
  vpMatrix diagHsd;// diag(Hsd)

 if(pjModel==parallel)
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
  else //perspective
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

  // ----------------------------------------------------------
  // Minimisation

  double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
  double lambdaGN;

  mu       =  0.01;
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
  double threshold=0.5;
  if(add_noise && (nsModel == Gauss_dynamic))
      threshold += noise_sdv;

 // vpHomogeneousMatrix edMe,edMw ;
  vpHomogeneousMatrix odMo ;

  do
  {

    std::cout << "--------------------------------------------" << iter++ << std::endl ;

    filecMo << iter;

    cMo = cMw * wMe * eMo;

    for(int m=0;m<3;m++)
        filecMo << "\t" << cMo[m][3];

    filecMo << endl;

    //  Acquire the new image
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

    for(int i=0;i<3;i++)
        graphy.plot(0,i,iter,TodMo[i]/scale);
    for(int i=0;i<3;i++)
        graphy.plot(1,i,iter,vpMath::deg(RodMo[i]));

    //cout<< "ZodMo=" << TodMo[2] << endl;

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

    // compute current error
    sI.error(sId,error) ;

    normError = sqrt(error.sumSquare()/error.getRows());

    graphy2.plot(0,0,iter,normError);

    cout << "|e| "<< normError <<endl ;
    fileResidu << iter << "\t" << normError << endl;

    // double t = vpTime::measureTimeMs() ;

    // ---------- Levenberg Marquardt method --------------
    {
      if (iter > iterGN)
      {
        mu = 0.0001 ;
        lambda = lambdaGN;
      }

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

    for(int i=0;i<3;i++)
      graphy.plot(2,i,iter,v[i]/scale);
    for(int i=0;i<3;i++)
      graphy.plot(3,i,iter,vpMath::deg(v[i+3]));

    fileVelociy << iter << "\t" << v.t() << endl;

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

    wMe = wMe * eV;

    cout << "eV=\n" << eV << endl;

    cout << "wMe_current=\n" << wMe << endl;

    send_wMe(wMe,scale);

    //cout << "eV=\n "<< eV << endl;

    //cMo =  cMo * vpExponentialMap::direct(v,0.04);

    //cout << "v ExpMap=\n" << vpExponentialMap::direct(v,0.04) << endl;

    cout << "cMo_new=\n" << cMo << endl;

    graphy2.plot(1,0,cMo[0][3]/scale,cMo[1][3]/scale,cMo[2][3]/scale);

  }
 while(normError > threshold  && iter < opt_niter);
//while(1) ;

 filecMo.close();
 cout << "===========================END==============================" << endl;

  while(1)
       graphy2.plot(1,0,cMo[0][3]/scale,cMo[1][3]/scale,cMo[2][3]/scale);


  //vpDisplay::getClick(graphy.I);
  vpDisplay::getClick(graphy2.I);


}

