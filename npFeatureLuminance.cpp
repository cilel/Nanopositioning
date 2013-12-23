/****************************************************************************
 *
 * Feature Luminance by perspective and parallel projection.
 *****************************************************************************/

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageConvert.h>
#include <visp/vpImageFilter.h>
#include <visp/vpException.h>

#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

#include "npFeatureLuminance.h"

using namespace std;

/*!
  \file vpFeatureLuminance.cpp
  \brief Class that defines the image luminance visual feature

  for more details see
  C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/



/*!
  Initialize the memory space requested for vpFeatureLuminance visual feature.
*/
void
npFeatureLuminance::init()
{
    if (flags == NULL)
      flags = new bool[nbParameters];
    for (unsigned int i = 0; i < nbParameters; i++) flags[i] = false;

    //default value Z (1 meters)
    Z = 1;

    firstTimeIn =0 ;

}


void
npFeatureLuminance::init(unsigned int _nbr, unsigned int _nbc, double _Z, projectionModel projModel)
{
  init() ;

  nbr = _nbr ;
  nbc = _nbc ;

  if((nbr < 2*bord) || (nbc < 2*bord)){
    throw vpException(vpException::dimensionError, "border is too important compared to number of row or column.");
  }

  // number of feature = nb column x nb lines in the images
  dim_s = (nbr-2*bord)*(nbc-2*bord) ;

  imIx.resize(nbr,nbc) ;
  imIy.resize(nbr,nbc) ;
  imG.resize(nbr,nbc) ;
  imGxy.resize(nbr,nbc) ;//for display gradient

  s.resize(dim_s) ;
  sg.resize(dim_s);
  
  if (pixInfo != NULL)
    delete [] pixInfo;

  pixInfo = new vpLuminance[dim_s] ;
  
  Z = _Z ;
  pjModel = projModel;
}

/*! 
  Default constructor that build a visual feature.
*/
npFeatureLuminance::npFeatureLuminance() : vpBasicFeature()
{
    nbParameters = 1;
    dim_s = 0 ;
    bord = 10 ;
    flags = NULL;
    pixInfo = NULL;

    init() ;
}

/*! 
  Default destructor.
*/
npFeatureLuminance::~npFeatureLuminance()
{
  if (pixInfo != NULL) delete [] pixInfo ;
  if (flags != NULL) delete [] flags;
}



/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \param Z : \f$ Z \f$ value to set.
*/
void
npFeatureLuminance::set_Z(const double Z)
{
    this->Z = Z ;
    flags[0] = true;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera frame.

  \return The value of \f$ Z \f$.
*/
double
npFeatureLuminance::get_Z() const
{
    return Z ;
}

vpMatrix
npFeatureLuminance::get_Lg()
{
    return Lg;
}

vpColVector
npFeatureLuminance::get_sg()
{
    return sg;
}


void
npFeatureLuminance::setCameraParameters(vpCameraParameters &_cam)
{
  cam = _cam ;
}


/*!

  Build a luminance feature directly from the image
*/

void
npFeatureLuminance::buildFrom(vpImage<unsigned char> &I)
{
  unsigned int l = 0;
  double Ix,Iy,Ixx, Ixy, Iyx, Iyy ;

  double px = cam.get_px() ;
  double py = cam.get_py() ;



  if (firstTimeIn==0)
    { 
      firstTimeIn=1 ;
      l =0 ;
      for (unsigned int i=bord; i < nbr-bord ; i++)
	{
	  //   cout << i << endl ;
	  for (unsigned int j = bord ; j < nbc-bord; j++)
	    {	double x=0,y=0;
          vpPixelMeterConversion::convertPointWithoutDistortion(cam,
						   i, j,
						   y, x)  ;
	    
	      pixInfo[l].x = x;
	      pixInfo[l].y = y;
          pixInfo[l].Z = Z ;

	      l++;
	    }
	}
    }

  //For image gradient

  for (int i=3; i < nbr-3 ; i++)
{
  //   cout << i << endl ;
  for (int j = 3 ; j < nbc-3; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      imG[i][j] =   vpImageFilter::gaussianFilter(I,i,j) ;
    }
}

  for (int i=3; i < nbr-3 ; i++)
{
  //   cout << i << endl ;
  for (int j = 3 ; j < nbc-3; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      imIx[i][j] =   vpImageFilter::derivativeFilterX(imG,i,j) ;
      imIy[i][j] =   vpImageFilter::derivativeFilterY(imG,i,j) ;
    }
}

  l= 0 ;
  for (int i=bord; i < nbr-bord ; i++)
{
  //   cout << i << endl ;
  for (int j = bord ; j < nbc-bord; j++)
    {
      // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      Ix =  imIx[i][j] ;
      Iy =  imIy[i][j] ;

      sg[l] = ( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ;//sqrt
      imGxy[i][j] = sqrt( vpMath::sqr(Ix) + vpMath::sqr(Iy) ) ;
      Ixx =  vpImageFilter::derivativeFilterX(imIx,i,j) ;
      Ixy =  vpImageFilter::derivativeFilterY(imIx,i,j) ;
      Iyx =  vpImageFilter::derivativeFilterX(imIy,i,j) ;
      Iyy =  vpImageFilter::derivativeFilterY(imIy,i,j) ;

      // Calcul de Z
      pixInfo[l].Ixx  = Ixx;
      pixInfo[l].Ixy  = Ixy;
      pixInfo[l].Iyx  = Iyx;
      pixInfo[l].Iyy  = Iyy;

      l++;
    }
}

  //For luminance
  l= 0 ;
  for (unsigned int i=bord; i < nbr-bord ; i++)
    {
      //   cout << i << endl ;
      for (unsigned int j = bord ; j < nbc-bord; j++)
	{
	  // cout << dim_s <<" " <<l <<"  " <<i << "  " << j <<endl ;
      Ix =  px * vpImageFilter::derivativeFilterX(I,i,j) ;
	  Iy =  py * vpImageFilter::derivativeFilterY(I,i,j) ;
	  
	  // Calcul de Z
	  
	  pixInfo[l].I  =  I[i][j] ;
	  s[l]  =  I[i][j] ;
	  pixInfo[l].Ix  = Ix;
	  pixInfo[l].Iy  = Iy;

	  
	  l++;
	}
    }

}


/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/
void
npFeatureLuminance::interaction(vpMatrix &L)
{
  double x,y,Ix,Iy,z,Zinv;

  double Ixx, Ixy, Iyx, Iyy;

  if(pjModel==perspective)
  {
        L.resize(dim_s,6) ;
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          Zinv =  1 / pixInfo[m].Z;

          {
            L[m][0] = Ix * Zinv;
            L[m][1] = Iy * Zinv;
            L[m][2] = -(x*Ix+y*Iy)*Zinv;
            L[m][3] = -Ix*x*y-(1+y*y)*Iy;
            L[m][4] = (1+x*x)*Ix + Iy*x*y;
            L[m][5]  = Iy*x-Ix*y;
          }
      }
  }
  else if(pjModel==parallel)
  {
        L.resize(dim_s,6) ;
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          z = pixInfo[m].Z;

          {
            L[m][0] = Ix;
            L[m][1] = Iy ;
            /*L[m][2] = -Iy * z;
            L[m][3] = Ix * z;
            L[m][4]  = Iy*x-Ix*y;*/

            L[m][2] =  0 ;
            L[m][3] = -Iy * z;
            L[m][4] = Ix * z;
            L[m][5]  = Iy*x-Ix*y;
          }
      }
    }
  else if(pjModel==parallelZ)
  {
        L.resize(dim_s,6) ;
        Lg.resize(dim_s,6);
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          z = pixInfo[m].Z;

          Ixx = pixInfo[m].Ixx;
          Ixy = pixInfo[m].Ixy;
          Iyx = pixInfo[m].Iyx;
          Iyy = pixInfo[m].Iyy;

          double A = -( Ixx*Ix+Iyx*Iy );
          double B = -( Ixy*Ix+Iyy*Iy );

          {
            L[m][0] = Ix;
            L[m][1] = Iy ;
            L[m][2] = 0;//(A+B);
            L[m][3] = -Iy * z;
            L[m][4] = Ix * z;
            L[m][5]  = Iy*x-Ix*y;
          }
          Lg[m][0] = 0;
          Lg[m][1] = 0;
          Lg[m][2] = (A+B);
          Lg[m][3] = 0;
          Lg[m][4] = 0;
          Lg[m][5] = 0;

      }
    }
  else
  {
        L.resize(dim_s,6) ;
      for(unsigned int m = 0; m< L.getRows(); m++)
      {
          Ix = pixInfo[m].Ix;
          Iy = pixInfo[m].Iy;

          x = pixInfo[m].x ;
          y = pixInfo[m].y ;
          Zinv =  1 / pixInfo[m].Z;

          {
            L[m][0] = Ix * Zinv;
            L[m][1] = Iy * Zinv;
            L[m][2] = -(x*Ix+y*Iy)*Zinv;
            L[m][3] = -Ix*x*y-(1+y*y)*Iy;
            L[m][4] = (1+x*x)*Ix + Iy*x*y;
            L[m][5]  = Iy*x-Ix*y;
          }
      }
  }
        //cout << "L=" << L << endl;
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is made
  thanks to the values of the luminance features \f$ I \f$
*/

vpMatrix  npFeatureLuminance::interaction(const unsigned int /* select */)
{
  /* static */ vpMatrix L  ; // warning C4640: 'L' : construction of local static object is not thread-safe
  interaction(L) ;
  return L ;
}


/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
void
npFeatureLuminance::error(const vpBasicFeature &s_star,
			  vpColVector &e)
{
  e.resize(dim_s) ;

  for (unsigned int i =0 ; i < dim_s ; i++)
    {
      e[i] = s[i] - s_star[i] ;
    }
}

/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired image gradient

  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.
*/
void
npFeatureLuminance::sg_error(const vpColVector &sg_star,
              vpColVector &eg)
{
  eg.resize(dim_s) ;

  for (unsigned int i =0 ; i < dim_s ; i++)
    {
      eg[i] = sg[i] - sg_star[i] ;
    }
}



/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired
 
  \param s_star : Desired visual feature.
  \param select : Not used.

*/
vpColVector
npFeatureLuminance::error(const vpBasicFeature &s_star,
			  const unsigned int /* select */)
{
  /* static */ vpColVector e ; // warning C4640: 'e' : construction of local static object is not thread-safe
  
  error(s_star, e) ;
  
  return e ;

}




/*!

  Not implemented.

 */
void
npFeatureLuminance::print(const unsigned int /* select */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
 }



/*!

  Not implemented.

 */
void
npFeatureLuminance::display(const vpCameraParameters & /* cam */,
                            const vpImage<unsigned char> & /* I */,
                            const vpColor &/* color */,
                            unsigned int /* thickness */) const
{
 static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void
npFeatureLuminance::display(const vpCameraParameters & /* cam */,
                            const vpImage<vpRGBa> & /* I */,
                            const vpColor &/* color */,
                            unsigned int /* thickness */) const
{
  static int firsttime =0 ;

  if (firsttime==0)
  {
    firsttime=1 ;
    vpERROR_TRACE("not implemented") ;
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}


/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a vpFeatureLuminance
  \endcode

*/
npFeatureLuminance *npFeatureLuminance::duplicate() const
{
  npFeatureLuminance *feature = new npFeatureLuminance ;
  return feature ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
