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
#include "npSimulatorSEM.h"

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
        else
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

int
main(int argc, const char ** argv)
{
  //std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string filename;
  bool opt_click_allowed = true;
  bool opt_display = true;
  int opt_niter = 70;
  bool add_noise = false;
  double noise_mean =0;
  double noise_sdv = 5;
  nsModel = Gauss_dynamic;

  double scale = 1;

  ipath = "../Images/london.jpg";

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_click_allowed,
                 opt_display, opt_niter,add_noise,scale) == false) {
    return (-1);
  }


  double ZcMo = 0.0223*scale;
  double Z =0.02231*scale;

  if ((ZcMo-Z)==0)
  {
      cout << "ZcMo and Z should be different." << endl;
      return 0;
  }

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

  vpImage<unsigned char> Itexture;
  filename = ipath;
  vpImageIo::read(Itexture,filename) ;

  vpImage<unsigned char> Inoised;
  Inoised = Itexture;

//  Inoised.init(Itexture.getRows(),Itexture.getCols(),0);
  if (add_noise && (nsModel == Gauss_statistic))
  {
       vpGaussRand noise(noise_sdv, noise_mean);
       for(int i=0; i< Itexture.getRows(); i++)
           for(int j=0;j<Itexture.getCols();j++)
           {
               double gauss = noise();
               double noised =(double) Itexture[i][j] + gauss;
               if (noised < 0)
                   Inoised[i][j] = 0;
               else if (noised > 255)
                   Inoised[i][j] = 255;
               else
                   Inoised[i][j] = noised;
           }
  }

  vpColVector X[4];
  for (int i = 0; i < 4; i++) X[i].resize(3);

  // Top left corner
  X[0][0] = -0.00008*scale;
  X[0][1] = -0.00006*scale;
  X[0][2] = 0*scale;

  // Top right corner
  X[1][0] = 0.00008*scale;
  X[1][1] = -0.00006*scale;
  X[1][2] = 0*scale;
  
  // Bottom right corner
  X[2][0] = 0.00008*scale;
  X[2][1] = 0.00006*scale;
  X[2][2] = 0*scale;
  
  //Bottom left corner
  X[3][0] = -0.00008*scale;
  X[3][1] = 0.00006*scale;
  X[3][2] = 0*scale;

  npImageSimulator sim;

  sim.setInterpolationType(npImageSimulator::BILINEAR_INTERPOLATION) ;

  if(pjModel==parallel)
    sim.init(Inoised, X, npImageSimulator::parallel);
  else
    sim.init(Inoised, X, npImageSimulator::perspective);
  
  vpCameraParameters cam(8984549/scale, 8955094/scale, 160, 120);//160, 120

  // ----------------------------------------------------------
  // Create the framegraber (here a simulated image)
  vpImage<unsigned char> I(240,320,0) ;
  vpImage<unsigned char> Id ;

  //camera desired position-7.214992365e-06
  vpHomogeneousMatrix cMod ;
  cMod[2][3] = ZcMo;

  //set the robot at the desired position
  sim.setCameraPosition(cMod) ;
  sim.getImage(I,cam);  // and aquire the image I-> Id
  Id = I ;

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
    d.init(I, 20, 10, "Photometric visual servoing : s") ;
    vpDisplay::display(I);
    vpDisplay::flush(I);
  }
  if (opt_display && opt_click_allowed) {
    std::cout << "Click in the image to continue..." << std::endl;
    vpDisplay::getClick(I) ;
  }
#endif

  // ----------------------------------------------------------
  // position the robot at the initial position
  // ----------------------------------------------------------

  //camera desired position

  vpHomogeneousMatrix cMo;

//  cMo.buildFrom(0.000000*scale,0.000000*scale,ZcMo,vpMath::rad(-5),vpMath::rad(5),vpMath::rad(5));
  cMo.buildFrom(0.000000*scale,0.00000*scale,ZcMo,vpMath::rad(0),vpMath::rad(0),vpMath::rad(5));

  //set the robot at the desired position
  sim.setCameraPosition(cMo);
  I = 0 ;
  sim.getImage(I,cam);  // and aquire the image I

  
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
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
    d1.init(Idiff, 40+(int)I.getWidth(), 10, "photometric visual servoing : s-s* ") ;
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

  vpVelocityTwistMatrix cVo;
  cVo.buildFrom(cMo);
  //cout << "cVo=\n" << cVo <<endl;

  vpMatrix Js;
  vpMatrix Jn;
  vpMatrix diagHsd;

 if(pjModel==parallel)
  {
      Jn.resize(6,5);
      Jn[0][0]=1;
      Jn[1][1]=1;
      Jn[3][2]=1;
      Jn[4][3]=1;
      Jn[5][4]=1;

      Js=-Lsd*cVo*Jn;

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
      Js=-Lsd*cVo*Jn;

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
  vpColVector e ;
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

  vpPlot graphy(4, 600, 800, 20, 300, "Nanopositioning");
  vpPlot graphy2(2, 600, 400, 820, 300, "Nanopositioning");

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

  
  double normError = 1000;
  double threshold=0.5;
  if(add_noise && (nsModel == Gauss_dynamic))
      threshold += noise_sdv;

 // vpHomogeneousMatrix edMe,edMw ;
  vpHomogeneousMatrix odMo ;

  do
  {

    std::cout << "--------------------------------------------" << iter++ << std::endl ;

    //  Acquire the new image
    sim.setCameraPosition(cMo) ;
    sim.getImage(I,cam) ;

    odMo = cMod.inverse() * cMo;

    vpTranslationVector TodMo;
    odMo.extract(TodMo);
    vpThetaUVector RodMo;
    odMo.extract(RodMo);

    for(int i=0;i<3;i++)
        graphy.plot(0,i,iter,TodMo[i]/scale);
    for(int i=0;i<3;i++)
        graphy.plot(1,i,iter,vpMath::deg(RodMo[i]));

    cout<< "ZodMo=" << TodMo[2] << endl;

    Inoised = I;
    if (add_noise && nsModel == Gauss_dynamic)
    {
         vpGaussRand noise(noise_sdv, noise_mean);
         for(int i=0; i< I.getRows(); i++)
             for(int j=0;j<I.getCols();j++)
             {
                 double gauss = noise();
                 double noised =(double) I[i][j] + gauss;
                 if (noised < 0)
                     Inoised[i][j] = 0;
                 else if (noised > 255)
                     Inoised[i][j] = 255;
                 else
                     Inoised[i][j] = noised;
             }
    }


#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
    if (opt_display) {
      vpDisplay::display(Inoised) ;
      vpDisplay::flush(Inoised) ;
    }
#endif
    vpImageTools::imageDifference(Inoised,Id,Idiff) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
    if (opt_display) {
      vpDisplay::display(Idiff) ;
      vpDisplay::flush(Idiff) ;
    }
#endif
    // Compute current visual feature
    sI.buildFrom(Inoised) ;

    // compute current error
    sI.error(sId,error) ;

    normError = sqrt(error.sumSquare()/error.getRows());

    graphy2.plot(0,0,iter,normError);

    cout << "|e| "<< normError <<endl ;

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
        v[1]=vc[1];
        v[0]=vc[0];
    }

    for(int i=0;i<3;i++)
      graphy.plot(2,i,iter,v[i]/scale);
    for(int i=0;i<3;i++)
      graphy.plot(3,i,iter,vpMath::deg(v[i+3]));

    cout << "v=" << v.t() << endl;
    cout << "lambda = " << lambda << "  mu = " << mu ;
    cout << " |Tc| = " << sqrt(v.sumSquare()) << endl;

    // send the robot velocity
    //robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    cout << "cMo=\n" << cMo << endl;

    vpHomogeneousMatrix oV;
    vpThetaUVector vR;
    vR[0]=v[3]*0.04;
    vR[1]=v[4]*0.04;
    vR[2]=v[5]*0.04;

    oV.insert(vR);
    oV[0][3]=v[0]*0.04;
    oV[1][3]=v[1]*0.04;
    oV[2][3]=v[2]*0.04;

    cMo = cMo * oV;

    cout << "oVo=\n "<< oV << endl;

    //cMo =  cMo * vpExponentialMap::direct(v,0.04);

    //cout << "v ExpMap=\n" << vpExponentialMap::direct(v,0.04) << endl;

    cout << "cMo_new=\n" << cMo << endl;

    graphy2.plot(1,0,cMo[0][3]/scale,cMo[1][3]/scale,cMo[2][3]/scale);

    vpTime::wait(20);

  }
 while(normError > threshold  && iter < opt_niter);
//while(1) ;

  while(1)
       graphy2.plot(1,0,cMo[0][3]/scale,cMo[1][3]/scale,cMo[2][3]/scale);


  //vpDisplay::getClick(graphy.I);
  vpDisplay::getClick(graphy2.I);

 /* v = 0 ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;*/

}

