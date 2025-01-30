#include "SPV/SPV.h"
#include <random>


void SPV::computeDeltaFromNumberOfPhosphenes(int &dX, int &dY)
{
    double map_multiplier = map_mode == 2 ? M_PI/4 : 1.0;

    if (grid_mode == 1)
    {
      double computed_delta = ((sqrt(double(size_img_x * size_img_y) * map_multiplier / double(N_fos))));
      dX = std::min(int(floor(computed_delta)),size_img_x);
      dY = std::min(dX,size_img_y);
    }
    else if (grid_mode == 2)
    {
        double computed_delta = ((sqrt(double(size_img_x * size_img_y * 2/sqrt(3))  * map_multiplier / double(N_fos))));
         dX = int(floor(computed_delta));
        dY = int(floor(computed_delta*sqrt(3)/2));
    }

//    std::cout << "dX: " << dX << " dY: " << dY << std::endl;
}


void SPV::genPhosphenesPos()
{

  // We start from the leftmost corner, either limited by the size of the image or the radius
  double yIni = std::min(y0,rExt);
  double xIni = std::min(x0,rExt);

  // Given a known delta (computed depending on N_phos) we compute the number of points in X and Y axis
  int nPointsX = int(floor(std::min(double(size_img_x),rExt*2) / deltaX));
  int nPointsY = int(floor(std::min(double(size_img_y),rExt*2) / deltaY));
  nPointsX_ = nPointsX;
  nPointsY_ = nPointsY;

//  std::cout << "size_img_x " << size_img_x << std::endl;
//  std::cout << "size_img_y " << size_img_y << std::endl;
//  std::cout << "deltaX " << deltaX << std::endl;
//  std::cout << "deltaY " << deltaY << std::endl;
//  std::cout << "nPointsX " << (std::min(double(size_img_x),rExt*2) / deltaX) << std::endl;
//  std::cout << "nPointsY " << (std::min(double(size_img_y),rExt*2) / deltaY) << std::endl;
//  std::cout << "nPointsX " << nPointsX << std::endl;
//  std::cout << "nPointsY " << nPointsY << std::endl;


  cv::Mat phosPoints_ray = cv::Mat::ones(3, 0, CV_64FC1);
  cv::Mat phosPoints_vis = cv::Mat::ones(3, 0, CV_64FC1);


  double rExt2 = pow(rExt, 2);

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,noise_stddev);

  if (grid_mode == 1)
  {
    for (int kY = 0; kY < nPointsY; kY++)
    {
        for (int kX = 0; kX < nPointsX; kX++)
        {
            double noise_x = distribution(generator);
            double x = (deltaX-1)/2 -xIni + deltaX*kX;
            double x_vis = x + noise_x;

            if ((x + x0 - (deltaX-1)/2 < 0) || (x + x0 + (deltaX-1)/2 >= size_img_x) || (x_vis + x0 - (deltaX-1)/2 < 0) || (x_vis + x0 + (deltaX-1)/2 >= size_img_x) )
                continue;

            double noise_y = distribution(generator);
            double y = (deltaY-1)/2 -yIni + deltaY*kY;
            double y_vis = y + noise_y;

            if ((y + y0 - (deltaY-1)/2 < 0) || (y + y0 + (deltaY-1)/2 >= size_img_y) || (y_vis + y0 - (deltaY-1)/2 < 0) || (y_vis + y0 + (deltaY-1)/2 >= size_img_y))
                continue;


            double d2 = x*x + y*y;
            double d2_vis = x_vis*x_vis + y_vis*y_vis;
            if (d2 < rExt2 && d2_vis < rExt2)
            {

                cv::Mat vecPoints(cv::Point3d(x + x0, y + y0, 1.0));
                phosPoints_ray.push_back(vecPoints.t());
                cv::Mat vecPoints_vis(cv::Point3d(x_vis + x0, y_vis + y0, 1.0));
                phosPoints_vis.push_back(vecPoints_vis.t());
            }

        }
    }
  }
  else
  {
    bool displaced = false;
    for (int kY = 0; kY < nPointsY; kY++)
    {

//      if (kY == nPointsY-1 && !(kY % 2 == 0))
//        break;

      double extra_displacement;
      if (!displaced)
      {
        extra_displacement = 0;
        displaced = true;
      }
      else
      {
        extra_displacement = (deltaX-1)/2;
        displaced = false;
      }

      for (int kX = 0; kX < nPointsX; kX++)
      {
//        if (!displaced && (kX == nPointsX -1))
//          break;

        double noise_x = distribution(generator);
        double x = (deltaX-1)/2 -xIni + deltaX*kX + extra_displacement;
        double x_vis = x + noise_x;

        if ((x + x0 - (deltaX-1)/2 < 0) || (x + x0 + (deltaX-1)/2 >= size_img_x) || (x_vis + x0 - (deltaX-1)/2 < 0) || (x_vis + x0 + (deltaX-1)/2 >= size_img_x) )
          continue;

        double noise_y = distribution(generator);
        double y = (deltaY-1)/2 -yIni + deltaY*kY;
        double y_vis = y + noise_y;

        if ((y + y0 - (deltaY-1)/2 < 0) || (y + y0 + (deltaY-1)/2 >= size_img_y) || (y_vis + y0 - (deltaY-1)/2 < 0) || (y_vis + y0 + (deltaY-1)/2 >= size_img_y))
            continue;


        double d2 = x*x + y*y;
        double d2_vis = x_vis*x_vis + y_vis*y_vis;

        if (d2 < rExt2 && d2_vis < rExt2)
        {
          cv::Mat vecPoints(cv::Point3d(x + x0, y + y0, 1.0));
          phosPoints_ray.push_back(vecPoints.t());
          cv::Mat vecPoints_vis(cv::Point3d(x_vis + x0, y_vis + y0, 1.0));
          phosPoints_vis.push_back(vecPoints_vis.t());

//          std::cout << kY << " " << kX << std::endl;

        }

      }
    }
  }

  phosphenesPos = phosPoints_ray.t();
  phosphenesPos_vis = phosPoints_vis.t();

  XiCam = cv::Mat_<double>::zeros(6, phosphenesPos.cols);
  cv::Mat v = K_p.inv() * phosphenesPos;
  v.copyTo(XiCam.rowRange(0, 3));

  rgbPos = K_c* K_p.inv() * phosphenesPos;
  depthPos = K_d * K_p.inv() * phosphenesPos;


  // Sacar los índices y los yaws de los fosfenos de la segunda fila
  //std::vector<int> indexes;
  //std::vector<double> yaws;

  int nPointsX_row2 = nPointsX_;
  if (nPointsX_*nPointsY_ > phosphenesPos.cols)
  {
//      std::cout << "HAY UNO MENOS EN LA SEGUNDA FILA" << std::endl;
    nPointsX_row2--;
  }

  for (int i = nPointsX_; i < nPointsX_+nPointsX_row2; i++)
  {
      indexes.push_back(i);
      double yaw = atan2(XiCam.at<double>(0,i),XiCam.at<double>(2,i));
      yaws.push_back(yaw);

//      std::cout << yaw*180/3.141592 << std::endl;

  }


}




void SPV::updatePhosphenes()
{
  genPhosphenesPos();
  applyDropout();
  lookUpTable = genLookUpTable();
  thDist = (deltaX - 1) / 2;

  std::cout << "Now there are " << phosphenesPos.cols << " phosphenes" << std::endl;
//  N_fos = phosphenesPos.cols;
}


void SPV::applyDropout()
{

  if (dropout > 0)
  {
    int n_phos_before = phosphenesPos.cols;
    int n_phos_after = int((1-dropout)*double(n_phos_before));

    cv::Mat phosPoints = cv::Mat_<double>::ones(3,n_phos_after);
    cv::Mat phosPoints_vis = cv::Mat_<double>::ones(3,n_phos_after);
    cv::Mat rgbPoints = cv::Mat_<double>::ones(3,n_phos_after);
    cv::Mat depthPoints = cv::Mat_<double>::ones(3,n_phos_after);
    cv::Mat XiCam_after = cv::Mat_<double>::zeros(6, n_phos_after);

    std::vector<int> indexes(n_phos_before);
    std::iota(std::begin(indexes), std::end(indexes), 0);
    std::random_shuffle(indexes.begin(), indexes.end());
    indexes.resize(n_phos_after);
    std::sort(indexes.begin(), indexes.end());

    for (int i = 0; i < indexes.size(); i++)
    {
      phosphenesPos.col(indexes[i]).copyTo(phosPoints.col(i));
      phosphenesPos_vis.col(indexes[i]).copyTo(phosPoints_vis.col(i));
      rgbPos.col(indexes[i]).copyTo(rgbPoints.col(i));
      depthPos.col(indexes[i]).copyTo(depthPoints.col(i));
      XiCam.col(indexes[i]).copyTo(XiCam_after.col(i));

    }

    phosPoints.copyTo(phosphenesPos);
    phosPoints_vis.copyTo(phosphenesPos_vis);
    rgbPoints.copyTo(rgbPos);
    depthPoints.copyTo(depthPos);
    XiCam_after.copyTo(XiCam);
  }
}

void SPV::changeDelta(int delta_change)
{
  if (delta_change >= 0)
  {
//    delta += delta_change;
    deltaX += delta_change;
    deltaY += delta_change;
  }
  else
  {
//    if (delta >= 3) delta += delta_change; else std::cout << "El número de fosfenos es demasiado elevado!" << std::endl;
    if (deltaX >= 3) deltaX += delta_change; else std::cout << "El número de fosfenos en X es demasiado elevado!" << std::endl;
    if (deltaY >= 3) deltaY += delta_change; else std::cout << "El número de fosfenos en Y es demasiado elevado!" << std::endl;
  }
}

void SPV::changeDelta(int deltaX_change, int deltaY_change)
{
  if (deltaX_change >= 0)
    deltaX += deltaX_change;
  else
    if (deltaX >= 3) deltaX += deltaX_change; else std::cout << "El número de fosfenos en X es demasiado elevado!" << std::endl;

  if (deltaY_change >= 0)
    deltaY += deltaY_change;
  else
    if (deltaY >= 3) deltaY += deltaY_change; else std::cout << "El número de fosfenos en Y es demasiado elevado!" << std::endl;

}


void SPV::genPhoshenSprite(cv::Mat &sprite, double sigma)
{
  double y0_sprite, x0_sprite;
  y0_sprite = (sprite.rows-1)/2;
  x0_sprite = (sprite.cols-1)/2;

    for (int i = 0; i < sprite.rows; i++)
        for (int j = 0; j < sprite.cols; j++)
        {
            double d2 = pow(j - x0_sprite, 2) + pow(i - y0_sprite, 2);
            double P_x_c = exp(-d2 / 2 / pow(sigma, 2));

//            std::cout << P_x_c << " ";

            unsigned char grayLevel = (unsigned char)round(P_x_c * 255);
            sprite.at<cv::Vec3b>(i, j) = cv::Vec3b(grayLevel, grayLevel, grayLevel);
        }

//    std::cout << sprite.cols << " " << sigma << std::endl;
}

void SPV::updateSprite(double factor)
{

    FOV_sprite = FOV_sprite*factor;
    sprites.clear();
    initSprites();

//  // Phosphene sprite configuration
////  size_sprite *= factor;
////  size_sprite = floor(size_sprite);

//  size_sprite = int(getPixelLengthFromFOV(FOV_sprite, f));

//  if (int(size_sprite) % 2 == 0) size_sprite += 1.0;

//  cv::Mat sprite;
//  cv::resize(reference_sprite, sprite, cv::Size(size_sprite,size_sprite));
////  sprite = sprite_aux.clone();

////  sprite = cv::Mat_<cv::Vec3b>::zeros(size_sprite, size_sprite);
////  sprite.setTo(cv::Scalar(255, 255, 255));
////  genPhoshenSprite(sprite, (size_sprite - 1) / 2); // 3.5
////  sprite_med = sprite.clone();
////  sprite_med = sprite_med * 0.3;

//  sprites.clear();
//  for (size_t i = 0; i <= N_levels; i++)
//  {
//    cv::Mat sprite_aux = sprite.clone();
//    double dim_factor = double(i)/double(N_levels);
//    sprites.push_back(sprite_aux * dim_factor);
//  }
}

/*cv::Mat SPV::genLookUpTable_very_old(double size_img, double rExt, cv::Mat PhosphenesPosition)
{
    //Calculo del fosfeno correspondiente a cada pixel de la imagen original (look up table)

    double minDistance, DistanceX, DistanceY, Distance;

    minDistance = rExt*rExt;

    int k = 0;
    int fosfMasProximo = 0;
    cv::Mat lookupTable = cv::Mat_<int>(size_img, size_img);

//    cout << "Look-up table" << endl;
    for (int i = 0; i < size_img; i++){
        for (int j = 0; j < size_img; j++){

            for (int k = 0; k < PhosphenesPosition.cols; k++){

                DistanceX = PhosphenesPosition.at<double>(0, k) - j;   //MatPhosphene.PositionPhosphenes[k].x;
                DistanceY = PhosphenesPosition.at<double>(1, k) - (size_img -i); //MatPhosphene.PositionPhosphenes[k].y;
                Distance = DistanceX*DistanceX + DistanceY*DistanceY;
                if (Distance < minDistance){
                    fosfMasProximo = k;
                    minDistance = Distance;
                }
                else{
                    minDistance = minDistance;
                }
            }
            lookupTable.at<int>(i, j) = fosfMasProximo;

        }
    }

//    cout << "End look-up table" << endl;

return lookupTable;
}*/


/*cv::Mat SPV::genLookUpTable_old(double size_img_x, double size_img_y, double delta, cv::Mat PhosphenesPosition)
{
//    int k = 0;
    cv::Mat lookupTable = cv::Mat_<int>::zeros(size_img_y, size_img_x);
    for (int k = 0; k < PhosphenesPosition.cols; k++)
    {

        cv::Mat phospheneMat = cv::Mat(delta,delta,CV_32SC1,k+1);
        int iIndex = (int)round(PhosphenesPosition.at<double>(1, k));
        int jIndex = (int)round(PhosphenesPosition.at<double>(0, k));

//        std::cout << iIndex << " " << jIndex << std::endl;

        cv::Mat tmp = lookupTable(cv::Rect(jIndex-(delta-1)/2,iIndex-(delta-1)/2,delta,delta));
        phospheneMat.copyTo(tmp);
    }

return lookupTable;
}*/

cv::Mat SPV::genLookUpTable()
{
    cv::Mat lookUpTable_aux = cv::Mat_<int>::zeros(size_img_y, size_img_x);
    cv::Mat color_img = cv::Mat_<cv::Vec3b>::ones(size_img_y, size_img_x);

    for (int k = 0; k < phosphenesPos.cols; k++)
    {
        int iIndex = (int)round(phosphenesPos.at<double>(1, k));
        int jIndex = (int)round(phosphenesPos.at<double>(0, k));

        lookUpTable_aux.at<int>(iIndex,jIndex) = int(k+1);
    }

    cv::watershed(color_img,lookUpTable_aux);
    lookUpTable = lookUpTable_aux.clone();

    return lookUpTable;
}

cv::Mat SPV::transformCoordinates(cv::Mat &R, cv::Mat &o, cv::Mat &t, double s)
{
        cv::Mat G = cv::Mat_<double>::zeros(6, 6);
        G.at<double>(0, 0) = (R.at<double>(0, 0)*s - t.at<double>(0)*o.at<double>(0));
        G.at<double>(0, 1) = (R.at<double>(0, 1)*s - t.at<double>(0)*o.at<double>(1));
        G.at<double>(0, 2) = (R.at<double>(0, 2)*s - t.at<double>(0)*o.at<double>(2));
        G.at<double>(0, 3) = (R.at<double>(0, 2)*o.at<double>(1) - R.at<double>(0, 1)*o.at<double>(2));
        G.at<double>(0, 4) = -(R.at<double>(0, 2)*o.at<double>(0) - R.at<double>(0, 0)*o.at<double>(2));
        G.at<double>(0, 5) = (R.at<double>(0, 1)*o.at<double>(0) - R.at<double>(0, 0)*o.at<double>(1));
        G.at<double>(1, 0) = (R.at<double>(1, 0)*s - t.at<double>(1)*o.at<double>(0));
        G.at<double>(1, 1) = (R.at<double>(1, 1)*s - t.at<double>(1)*o.at<double>(1));
        G.at<double>(1, 2) = (R.at<double>(1, 2)*s - t.at<double>(1)*o.at<double>(2));
        G.at<double>(1, 3) = (R.at<double>(1, 2)*o.at<double>(1) - R.at<double>(1, 1)*o.at<double>(2));
        G.at<double>(1, 4) = -(R.at<double>(1, 2)*o.at<double>(0) - R.at<double>(1, 0)*o.at<double>(2));
        G.at<double>(1, 5) = (R.at<double>(1, 1)*o.at<double>(0) - R.at<double>(1, 0)*o.at<double>(1));
        G.at<double>(2, 0) = (R.at<double>(2, 0)*s - t.at<double>(2)*o.at<double>(0));
        G.at<double>(2, 1) = (R.at<double>(2, 1)*s - t.at<double>(2)*o.at<double>(1));
        G.at<double>(2, 2) = (R.at<double>(2, 2)*s - t.at<double>(2)*o.at<double>(2));
        G.at<double>(2, 3) = (R.at<double>(2, 2)*o.at<double>(1) - R.at<double>(2, 1)*o.at<double>(2));
        G.at<double>(2, 4) = -(R.at<double>(2, 2)*o.at<double>(0) - R.at<double>(2, 0)*o.at<double>(2));
        G.at<double>(2, 5) = (R.at<double>(2, 1)*o.at<double>(0) - R.at<double>(2, 0)*o.at<double>(1));
        G.at<double>(3, 0) = -(R.at<double>(1, 0)*t.at<double>(2) - R.at<double>(2, 0)*t.at<double>(1));
        G.at<double>(3, 1) = -(R.at<double>(1, 1)*t.at<double>(2) - R.at<double>(2, 1)*t.at<double>(1));
        G.at<double>(3, 2) = -(R.at<double>(1, 2)*t.at<double>(2) - R.at<double>(2, 2)*t.at<double>(1));
        G.at<double>(3, 3) = (R.at<double>(1, 1)*R.at<double>(2, 2) - R.at<double>(1, 2)*R.at<double>(2, 1));
        G.at<double>(3, 4) = -(R.at<double>(1, 0)*R.at<double>(2, 2) - R.at<double>(1, 2)*R.at<double>(2, 0));
        G.at<double>(3, 5) = (R.at<double>(1, 0)*R.at<double>(2, 1) - R.at<double>(1, 1)*R.at<double>(2, 0));
        G.at<double>(4, 0) = (R.at<double>(0, 0)*t.at<double>(2) - R.at<double>(2, 0)*t.at<double>(0));
        G.at<double>(4, 1) = (R.at<double>(0, 1)*t.at<double>(2) - R.at<double>(2, 1)*t.at<double>(0));
        G.at<double>(4, 2) = (R.at<double>(0, 2)*t.at<double>(2) - R.at<double>(2, 2)*t.at<double>(0));
        G.at<double>(4, 3) = -(R.at<double>(0, 1)*R.at<double>(2, 2) - R.at<double>(0, 2)*R.at<double>(2, 1));
        G.at<double>(4, 4) = (R.at<double>(0, 0)*R.at<double>(2, 2) - R.at<double>(0, 2)*R.at<double>(2, 0));
        G.at<double>(4, 5) = -(R.at<double>(0, 0)*R.at<double>(2, 1) - R.at<double>(0, 1)*R.at<double>(2, 0));
        G.at<double>(5, 0) = -(R.at<double>(0, 0)*t.at<double>(1) - R.at<double>(1, 0)*t.at<double>(0));
        G.at<double>(5, 1) = -(R.at<double>(0, 1)*t.at<double>(1) - R.at<double>(1, 1)*t.at<double>(0));
        G.at<double>(5, 2) = -(R.at<double>(0, 2)*t.at<double>(1) - R.at<double>(1, 2)*t.at<double>(0));
        G.at<double>(5, 3) = (R.at<double>(0, 1)*R.at<double>(1, 2) - R.at<double>(0, 2)*R.at<double>(1, 1));
        G.at<double>(5, 4) = -(R.at<double>(0, 0)*R.at<double>(1, 2) - R.at<double>(0, 2)*R.at<double>(1, 0));
        G.at<double>(5, 5) = (R.at<double>(0, 0)*R.at<double>(1, 1) - R.at<double>(0, 1)*R.at<double>(1, 0));
        return(G);
}

cv::Mat SPV::getPluckerRotation(Eigen::Affine3d T)
{
    cv::Mat R;
    cv::eigen2cv(Eigen::Matrix3d(T.rotation()),R);
    cv::Mat t(3,1,CV_64FC1, cv::Scalar(0.0));
    cv::eigen2cv(Eigen::Vector3d(T.translation()),t);
    cv::Mat o(cv::Point3d(0, 0, 0));
    double s = 1;
    cv::Mat G = transformCoordinates(R, o, t, s);

    return G;
}

void SPV::computeVanishingLines(Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag, int intensity)
{

  double line_height = 3;
  double line_length = 10;
  double line_lateral = 1.5;

  cv::Point3d p1_left = cv::Point3d(T.translation()[0] - line_length, T.translation()[1] + line_lateral, line_height);
  cv::Point3d p2_left = cv::Point3d(T.translation()[0] + line_length, T.translation()[1] + line_lateral, line_height);
  cv::Point3d p1_right = cv::Point3d(T.translation()[0] - line_length, T.translation()[1] - line_lateral, line_height);
  cv::Point3d p2_right = cv::Point3d(T.translation()[0] + line_length, T.translation()[1] - line_lateral, line_height);


  cv::Mat LUpLeft = twoPointsSpan(1, cv::Mat(p1_left), 1, cv::Mat(p2_left));
  cv::Mat LUpRight = twoPointsSpan(1, cv::Mat(p1_right), 1, cv::Mat(p2_right));

  cv::Mat LUpLeftCam = G.inv() * LUpLeft;
  cv::Mat LUpRightCam = G.inv() * LUpRight;

  computeLinePlucker(LUpLeftCam, phospheneFlag, intensity);
  computeLinePlucker(LUpRightCam, phospheneFlag, intensity);


}


cv::Mat SPV::twoPointsSpan(double x0, cv::Mat x, double y0, cv::Mat y)
{
        cv::Mat L = cv::Mat_<double>::zeros(6, 1);
        L.rowRange(0, 3) = x0*y - y0*x;
        cv::Mat lBar = x.cross(y);
        lBar.copyTo(L.rowRange(3, 6));

        return(L);
}

void SPV::computeLinePlucker(cv::Mat LCam, std::vector<int> & phospheneFlag, int intensity)
{
  cv::Mat l = (K_p.inv()).t()*LCam.rowRange(3, 6);
  cv::Mat dist = abs(l.t() / norm(l.rowRange(0, 2)) * phosphenesPos);

  cv::Mat binPlot = dist < thDist;

  for (int k = 0; k < phosphenesPos.cols; k++)
          if (binPlot.at<bool>(k) == true)
          {
                  cv::Mat X = twoLinesClosestPoint(LCam, XiCam.col(k));
                  X = X / X.at<double>(0);

                  if (X.at<double>(3) > 0)
                    phospheneFlag[k] = intensity;
          }

}

void SPV::computeSegmentPlucker(cv::Point3d X1Cam, cv::Point3d X2Cam, cv::Mat LCam, std::vector<int> & phospheneFlag, std::vector<double> & phospheneDepth, int intensity)
{
  cv::Mat l = (K_p.inv()).t()*LCam.rowRange(3, 6);
  cv::Mat dist = abs(l.t() / norm(l.rowRange(0, 2)) * phosphenesPos);

  cv::Mat binPlot = dist < thDist*2;

  // std::cout << binPlot << std::endl;

  for (int k = 0; k < phosphenesPos.cols; k++)
          if (binPlot.at<bool>(k) > 0)
          {
                  cv::Mat X = twoLinesClosestPoint(LCam, XiCam.col(k));
                  X = X / X.at<double>(0);

                  Eigen::Vector3d X1_v(X1Cam.x,X1Cam.y,X1Cam.z);
                  Eigen::Vector3d X2_v(X2Cam.x,X2Cam.y,X2Cam.z);

                  Eigen::Vector3d XInter(X.at<double>(1),X.at<double>(2),X.at<double>(3));
                  Eigen::Vector3d Xaux = (XInter-X1_v).array()/(X2_v-X1_v).array();

                  double lambda = Xaux.mean();

                  // std::cout << lambda << std::endl;
                  if (X.at<double>(3) > 0 && lambda>0 && lambda<1)
                  {
                    phospheneFlag[k] = intensity;
                    phospheneDepth[k] = X.at<double>(3);
                  }
          }

}

void SPV::computeSegmentCameraCoordinates(cv::Point3d X1, cv::Point3d X2, std::vector<int> &phospheneFlag, int intensity)
{
  cv::Mat L = twoPointsSpan(1, cv::Mat(X1), 1, cv::Mat(X2));

  std::vector<double> phospheneDepth(size_t(XiCam.cols), 0);
  computeSegmentPlucker(X1,X2, L, phospheneFlag, phospheneDepth, intensity);
}

void SPV::computeSegmentGlobalCoordinates(cv::Point3d X1, cv::Point3d X2, Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag,  std::vector<double> &phospheneDepth, int intensity)
{
  cv::Mat L = twoPointsSpan(1, cv::Mat(X1), 1, cv::Mat(X2));
  cv::Mat LCam = G.inv() * L;

  Eigen::Affine3d Tabs_c = T.inverse();

  Eigen::Vector3d X1_v(X1.x,X1.y,X1.z);
  Eigen::Vector3d X2_v(X2.x,X2.y,X2.z);
  Eigen::Vector3d X1Cam = Tabs_c.rotation()*X1_v+Tabs_c.translation();
  Eigen::Vector3d X2Cam = Tabs_c.rotation()*X2_v+Tabs_c.translation();

  computeSegmentPlucker(cv::Point3d(X1Cam(0),X1Cam(1),X1Cam(2)), cv::Point3d(X2Cam(0),X2Cam(1),X2Cam(2)), LCam, phospheneFlag, phospheneDepth, intensity);

}


void SPV::computeSegmentGlobalCoordinates(std::vector<cv::Point3d> points, Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag, std::vector<double> &phospheneDepth, int intensity)
{
  if (points.size() < 2)
    return;

  Eigen::Affine3d Tabs_c = T.inverse();


  for (size_t i = 0; i< points.size()-1; i++)
  {
    cv::Point3d X1 = points[i];
    cv::Point3d X2 = points[i+1];

    Eigen::Vector3d X1_v(X1.x,X1.y,X1.z);
    Eigen::Vector3d X2_v(X2.x,X2.y,X2.z);
    Eigen::Vector3d X1Cam = Tabs_c.rotation()*X1_v+Tabs_c.translation();
    Eigen::Vector3d X2Cam = Tabs_c.rotation()*X2_v+Tabs_c.translation();

    cv::Mat L = twoPointsSpan(1, cv::Mat(X1), 1, cv::Mat(X2));
    cv::Mat LCam = G.inv() * L;

    computeSegmentPlucker(cv::Point3d(X1Cam(0),X1Cam(1),X1Cam(2)), cv::Point3d(X2Cam(0),X2Cam(1),X2Cam(2)), LCam, phospheneFlag, phospheneDepth, intensity);
  }
}



cv::Mat SPV::twoLinesClosestPoint(cv::Mat L, cv::Mat M)
{
        double l1 = L.at<double>(0);        double l2 = L.at<double>(1);        double l3 = L.at<double>(2);
        double lBar1 = L.at<double>(3);     double lBar2 = L.at<double>(4);     double lBar3 = L.at<double>(5);

        double m1 = M.at<double>(0);        double m2 = M.at<double>(1);        double m3 = M.at<double>(2);
        double mBar1 = M.at<double>(3);     double mBar2 = M.at<double>(4);     double mBar3 = M.at<double>(5);

        cv::Mat X = cv::Mat_<double>(4, 1);
        X.at<double>(0) = -(l1*lBar2*m3 - l1*lBar3*m2 - l2*lBar1*m3 + l2*lBar3*m1 + l3*lBar1*m2 - l3*lBar2*m1)*((l1*l1)*(m2*m2) + (l2*l2)*(m1*m1) + (l1*l1)*(m3*m3) + (l3*l3)*(m1*m1) + (l2*l2)*(m3*m3) + (l3*l3)*(m2*m2) - l1*l2*m1*m2*2.0 - l1*l3*m1*m3*2.0 - l2*l3*m2*m3*2.0);
        X.at<double>(1) = lBar3*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l2*(m1*m1) + l2*(m3*m3) - l1*m1*m2 - l3*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m2 + (l3*l3)*m2 - l1*l2*m1 - l2*l3*m3)) - lBar2*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l3*(m1*m1) + l3*(m2*m2) - l1*m1*m3 - l2*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m3 + (l2*l2)*m3 - l1*l3*m1 - l2*l3*m2));
        X.at<double>(2) = -lBar3*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l1*(m2*m2) + l1*(m3*m3) - l2*m1*m2 - l3*m1*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l2*l2)*m1 + (l3*l3)*m1 - l1*l2*m2 - l1*l3*m3)) + lBar1*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l3*(m1*m1) + l3*(m2*m2) - l1*m1*m3 - l2*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m3 + (l2*l2)*m3 - l1*l3*m1 - l2*l3*m2));
        X.at<double>(3) = lBar2*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l1*(m2*m2) + l1*(m3*m3) - l2*m1*m2 - l3*m1*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l2*l2)*m1 + (l3*l3)*m1 - l1*l2*m2 - l1*l3*m3)) - lBar1*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l2*(m1*m1) + l2*(m3*m3) - l1*m1*m2 - l3*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m2 + (l3*l3)*m2 - l1*l2*m1 - l2*l3*m3));

        return(X);

}



/*cv::Mat SPV::twoLinesClosestPoint(cv::Mat L, cv::Mat M)
{
        #define l1 L.at<double>(0)
        #define l2 L.at<double>(1)
        #define l3 L.at<double>(2)
        #define lBar1 L.at<double>(3)
        #define lBar2 L.at<double>(4)
        #define lBar3 L.at<double>(5)

        #define m1 M.at<double>(0)
        #define m2 M.at<double>(1)
        #define m3 M.at<double>(2)
        #define mBar1 M.at<double>(3)
        #define mBar2 M.at<double>(4)
        #define mBar3 M.at<double>(5)

        cv::Mat X = cv::Mat_<double>(4, 1);
        X.at<double>(0) = -(l1*lBar2*m3 - l1*lBar3*m2 - l2*lBar1*m3 + l2*lBar3*m1 + l3*lBar1*m2 - l3*lBar2*m1)*((l1*l1)*(m2*m2) + (l2*l2)*(m1*m1) + (l1*l1)*(m3*m3) + (l3*l3)*(m1*m1) + (l2*l2)*(m3*m3) + (l3*l3)*(m2*m2) - l1*l2*m1*m2*2.0 - l1*l3*m1*m3*2.0 - l2*l3*m2*m3*2.0);
        X.at<double>(1) = lBar3*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l2*(m1*m1) + l2*(m3*m3) - l1*m1*m2 - l3*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m2 + (l3*l3)*m2 - l1*l2*m1 - l2*l3*m3)) - lBar2*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l3*(m1*m1) + l3*(m2*m2) - l1*m1*m3 - l2*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m3 + (l2*l2)*m3 - l1*l3*m1 - l2*l3*m2));
        X.at<double>(2) = -lBar3*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l1*(m2*m2) + l1*(m3*m3) - l2*m1*m2 - l3*m1*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l2*l2)*m1 + (l3*l3)*m1 - l1*l2*m2 - l1*l3*m3)) + lBar1*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l3*(m1*m1) + l3*(m2*m2) - l1*m1*m3 - l2*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m3 + (l2*l2)*m3 - l1*l3*m1 - l2*l3*m2));
        X.at<double>(3) = lBar2*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l1*(m2*m2) + l1*(m3*m3) - l2*m1*m2 - l3*m1*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l2*l2)*m1 + (l3*l3)*m1 - l1*l2*m2 - l1*l3*m3)) - lBar1*((lBar3*(l1*m2 - l2*m1) - lBar2*(l1*m3 - l3*m1) + lBar1*(l2*m3 - l3*m2))*(l2*(m1*m1) + l2*(m3*m3) - l1*m1*m2 - l3*m2*m3) + (mBar3*(l1*m2 - l2*m1) - mBar2*(l1*m3 - l3*m1) + mBar1*(l2*m3 - l3*m2))*((l1*l1)*m2 + (l3*l3)*m2 - l1*l2*m1 - l2*l3*m3));

        return(X);

        #undef l1
        #undef l2
        #undef l3
        #undef lBar1
        #undef lBar2
        #undef lBar3

        #undef m1
        #undef m2
        #undef m3
        #undef mBar1
        #undef mBar2
        #undef mBar3
}*/








void SPV::computePlanePolygonIntersection(cv::Mat XiAbs, Plane plane, std::vector<int> & chessFlag, std::vector<double> &phospheneDepth, Eigen::Affine3d T, int intensity)
{
    if (plane.contour2f->points.size() < 3)
        return;

    Eigen::Vector3f v1 = plane.coeffs2f.head<3>();
    Eigen::Vector3d v1_d = v1.cast<double>();

    cv::Mat u;
    eigen2cv(v1_d,u);
    double u0 = (double) plane.coeffs2f(3);

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,1>(0,0) = v1;
    // Eigen::Vector3f v2 = v1;
    // v2 = v2 + Eigen::Vector3f(1.0,1.0,0.0); v2.normalize();
    Eigen::Vector3f q1(1.0, 0.0, 0.0);
    Eigen::Vector3f q2(0.0, 1.0, 0.0);
    Eigen::Vector3f q;
    if (fabs(v1.dot(q1)) < fabs(v1.dot(q2)))
      q = q1;
    else
      q = q2;

    // Eigen::Vector3f v2(1.0, 1.0, 0.0); v2.normalize();
    Eigen::Vector3f v2 = v1.cross(q); v2.normalize();
    p2w.block<3,1>(0,1) = v2;
    Eigen::Vector3f v3 = v1.cross(v2); v3.normalize();
    p2w.block<3,1>(0,2) = v3;

    float d1 = fabs(v1.dot(Eigen::Vector3f(1,0,0)));
    float d2 = fabs(v2.dot(Eigen::Vector3f(1,0,0)));
    float d3 = fabs(v3.dot(Eigen::Vector3f(1,0,0)));
    int dir;
    if (d1 > d2 && d1 > d3)
      dir = 1;
    else if (d2 > d1 && d2 > d3)
      dir = 2;
    else
      dir = 3;

    // p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * plane.centroid2f.getVector3fMap().head<3>());
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0)* plane.centroid2f.getVector3fMap().head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*plane.contour2f, cPoints, p2w);

    // Eigen::Vector3f centroide_transformado;
    // Eigen::Affine3f p2w_aff(p2w.cast<float>());
    // pcl::transformPoint(plane.centroid2f.getVector3fMap(), centroide_transformado, p2w_aff);

    // std::cout << "coeffs:" << plane.coeffs2f << std::endl;
    // std::cout << centroide_transformado << std::endl;
    // std::cout << plane.centroid2f << std::endl;
    // std::cout << plane.contour2f->points[0].x << " " << plane.contour2f->points[0].y << " " << plane.contour2f->points[0].z << " " << std::endl;
    // std::cout << plane.contour2f->points[1].x << " " << plane.contour2f->points[1].y << " " << plane.contour2f->points[1].z << " " << std::endl;
    // std::cout << plane.contour2f->points[2].x << " " << plane.contour2f->points[2].y << " " << plane.contour2f->points[2].z << " " << std::endl;
    // std::cout << plane.contour2f->points[3].x << " " << plane.contour2f->points[3].y << " " << plane.contour2f->points[3].z << " " << std::endl;
    // std::cout << p2w.matrix() << std::endl << std::endl;

    std::vector<cv::Point2f> polygon;
    for (int kContour = 0; kContour<cPoints.points.size(); kContour++)
    {
      // std::cout << plane.contour2f->points[kContour].x << " " << plane.contour2f->points[kContour].y << " " << plane.contour2f->points[kContour].z << std::endl;
      // std::cout << cPoints.points[kContour].x << " " << cPoints.points[kContour].y << " " << cPoints.points[kContour].z << std::endl;
        if (dir == 1)
        {
          cv::Point2f contour_point(cPoints.points[kContour].y,cPoints.points[kContour].z);
          polygon.push_back(contour_point);
        }
        else if (dir == 2)
        {
          cv::Point2f contour_point(cPoints.points[kContour].x,cPoints.points[kContour].z);
          polygon.push_back(contour_point);
        }
        else
        {
          cv::Point2f contour_point(cPoints.points[kContour].x,cPoints.points[kContour].y);
          polygon.push_back(contour_point);
        }
    }

    // std::cout << u0 << std::endl << u << std::endl;

    // std::cout << polygon << std::endl;
    for (int kRay = 0; kRay < XiAbs.cols; kRay++)
    {
//        if (chessFlag[kRay] < 3)
//        {
        cv::Mat Xi = XiAbs.col(kRay);


        cv::Mat X = linePlaneMeet(u0, u, Xi.rowRange(0,3), Xi.rowRange(3,6));
        Eigen::Vector4f X_v(float(X.at<double>(0)), float(X.at<double>(1)), float(X.at<double>(2)), 1.0f);

        // X_v.head<3>() = plane.centroid2f.getVector3fMap().head<3>();
        // X_v(0) = X_v(0) + 0.2;

        // std::cout << "X_v" << std::endl << X_v << std::endl << std::endl;
        // std::cout << "T" << std::endl << T.translation() << std::endl << std::endl;

        double distance = sqrt(pow(T.translation()[0] - plane.centroid2f.x,2) + pow(T.translation()[1] - plane.centroid2f.y,2) + pow(T.translation()[2] - plane.centroid2f.z,2));
        if (phospheneDepth[kRay] == 0 || phospheneDepth[kRay] > distance)
          phospheneDepth[kRay] = distance;


        Eigen::Vector4f X_v_rot = p2w*X_v;

        // std::cout << X_v << std::endl <<  X_v_rot << std::endl << std::endl;

        double inPolygon;
        if (dir == 1)
          inPolygon = pointPolygonTest(polygon, cv::Point2f(X_v_rot(1), X_v_rot(2)), false);
        else if (dir == 2)
          inPolygon = pointPolygonTest(polygon, cv::Point2f(X_v_rot(0), X_v_rot(2)), false);
        else
          inPolygon = pointPolygonTest(polygon, cv::Point2f(X_v_rot(0), X_v_rot(1)), false);


        // std::cout << inPolygon << std::endl << std::endl;
        if (inPolygon == 1)
        {
            chessFlag[kRay] = intensity;
        }

        // break;
    }
}

void SPV::computeFlagObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle, cv::Mat lookUpTable, cv::Mat K_p, std::vector<int> & obstacleFlag, int intensity)
{

    for (int i = 0; i<obstacle->points.size(); i++)
    {
        Eigen::Vector3f point_eig = obstacle->points[i].getVector3fMap();

        Eigen::Vector2d pixel (round(point_eig(0)*K_p.at<double>(0,0)/point_eig(2)+K_p.at<double>(0,2)),round(point_eig(1)*K_p.at<double>(1,1)/point_eig(2)+K_p.at<double>(1,2)));
        if ((pixel(0) >= 0) && (pixel(0) < lookUpTable.cols) && (pixel(1) >= 0) && (pixel(1) < lookUpTable.rows))
        {
            double id = lookUpTable.at<int>(pixel(1),pixel(0)) - 1;
            if (id >= 0)
                obstacleFlag[id] = intensity;
        }
    }
}

void SPV::computeFloorPolygonEdges(std::vector<cv::Point2f> points, Eigen::Affine3d T, cv::Mat G, std::vector<int> &phospheneFlag, int intensity)
{


  std::vector<cv::Point3d> points_3d;
  for (size_t i = 0; i < points.size(); i++)
    points_3d.push_back(cv::Point3d(points[i].x,points[i].y,0));

  std::vector<double> phospheneDepth(size_t(XiCam.cols), 0);
  computeSegmentGlobalCoordinates(points_3d, T, G, phospheneFlag, phospheneDepth, intensity);
}


void SPV::computeXYChessPatternPolygonConvHull(cv::Mat & XiAbs, std::vector<cv::Point2f> layout2D, double d, double dChessX, double dChessY, std::vector<int> & chessFlag, int intensity_1, int intensity_2)
{
    cv::Mat XLayout_f(4,layout2D.size(),CV_32FC1, cv::Scalar(1.0));
//    std::vector<cv::Point2f> layout2D_f;
    for (int i= 0; i < layout2D.size(); i++)
    {
        XLayout_f.at<float>(0,i) = layout2D[i].x;
        XLayout_f.at<float>(1,i) = layout2D[i].y;
        XLayout_f.at<float>(2,i) = 0.0f;

//        cv::cv::Point2f layout2D_i_f; layout2D_i_f.x = float(layout2D[i].x); layout2D_i_f.y = float(layout2D[i].y);
//        layout2D_f.push_back(layout2D_i_f);
    }

    cv::Mat XLayout;// = XLayout_f.clone();
    XLayout_f.convertTo(XLayout,CV_64FC1);

    XLayout_f = XLayout_f * 1024;
    cv::Mat XLayout_i;
    XLayout_f.convertTo(XLayout_i, CV_32S);

        cv::Mat hullInput;
        std::vector<int> hullIndex;
        transpose(XLayout_i.rowRange(0, 2), hullInput);
        convexHull(hullInput, hullIndex, true, false); // first true for CLOCKWISE

        cv::Mat sidesL = pointsToPluckerPolygon(hullIndex, XLayout);

        computeChessFlagConvexHull(XiAbs, layout2D, sidesL, d, dChessX, dChessY, chessFlag, intensity_1, intensity_2);
//        return(chessFlag);
}




cv::Mat SPV::pointsToPluckerPolygon(std::vector<cv::Point2f> poly)
{
    int nPoints = poly.size();
    cv::Mat sidesL = cv::Mat_<double>::zeros(6, nPoints);

    for (int k = 0; k < nPoints; k++)
    {
        int indexA = k;
        int indexB;
        if (k == nPoints - 1)
                indexB = 0;
        else
                indexB = k+1;

//        cv::Mat x, y;
        cv::Mat x(3,1,CV_64FC1, cv::Scalar(0.0));
        cv::Mat y(3,1,CV_64FC1, cv::Scalar(0.0));

        x.at<double>(0,0) = (double) poly[indexA].x;
        x.at<double>(1,0) = (double) poly[indexA].y;
        x.at<double>(2,0) = 0.0;

        y.at<double>(0,0) = (double) poly[indexB].x;
        y.at<double>(1,0) = (double) poly[indexB].y;
        y.at<double>(2,0) = 0.0;

//        std::cout << poly << std::endl;
//        std::cout << poly[indexA] << std::endl;
//        std::cout << poly[indexB] << std::endl;

//        cv::Mat XLayout;
//        XLayout_f.convertTo(XLayout,CV_64FC1);


//        std::cout << x << std::endl << y << std::endl;
//        cv::Mat x = poly[indexA].x;
//        cv::Mat y = poly[indexB].y;

//        std::cout << x.rowRange(0,3) << std::endl << y.rowRange(0,3) << std::endl;

        cv::Mat polyL = twoPointsSpan(1, x.rowRange(0, 3), 1, y.rowRange(0, 3));
//        std::cout << polyL << std::endl;
        cv::Mat polyLNorm = normalizeL(polyL);
//        std::cout << polyLNorm << std::endl;
        polyLNorm.copyTo(sidesL.col(k));
    }
//    std::cout << std::endl << sidesL << std::endl;
    return(sidesL);
}

cv::Mat SPV::pointsToPluckerPolygon(std::vector<int> hullIndex, cv::Mat XLayout)
{
        int nPoints = hullIndex.size();
        cv::Mat sidesL = cv::Mat_<double>::zeros(6, nPoints);

        for (int k = 0; k < nPoints; k++)
        {
                int indexA = hullIndex[nPoints - 1 - k];
                int indexB;
                if (k == nPoints - 1)
                        indexB = hullIndex[nPoints - 1];
                else
                        indexB = hullIndex[nPoints - 2 - k];

                cv::Mat x = XLayout.col(indexA);
                cv::Mat y = XLayout.col(indexB);

                cv::Mat polyL = twoPointsSpan(1, x.rowRange(0, 3), 1, y.rowRange(0, 3));
                cv::Mat polyLNorm = normalizeL(polyL);

                polyLNorm.copyTo(sidesL.col(k));

        }
        return(sidesL);
}



cv::Mat SPV::normalizeL(cv::Mat L)
{
        cv::Mat LNorm = cv::Mat_<double>::zeros(6, 1);
        double normL = norm(L.rowRange(0, 3));

        cv::Mat l = L.rowRange(0, 3) / normL;
        cv::Mat lBar = L.rowRange(3, 6) / normL;

        l.copyTo(LNorm.rowRange(0, 3));
        lBar.copyTo(LNorm.rowRange(3, 6));

        return(LNorm);
}

void SPV::computeChessFlag(cv::Mat XiAbs, std::vector<cv::Point2f> layout2D, std::vector<int> & chessFlag, int intensity_1, int intensity_2)
{
        for (int kRay = 0; kRay < XiAbs.cols; kRay++)
        {
            if (chessFlag[kRay] < 3)
            {
                cv::Mat Xi = XiAbs.col(kRay);
                        cv::Mat x = lineXYPlaneMeetMat(0, Xi.rowRange(0, 3), Xi.rowRange(3, 6));

                        // +1 if inside
                        double inPolygon = pointPolygonTest(layout2D, cv::Point2f(float(x.at<double>(0)), float(x.at<double>(1))), false);

                        if (inPolygon == 1)
                        {
                                // The ray intersects the polygon and the 3D coordinates of the point are in x

                                int chessPosX = (int)(floor(x.at<double>(0) / dChessX)) % 2;
                                int chessPosY = (int)(floor(x.at<double>(1) / dChessY)) % 2;

                                /* Implementar un xor */

                                int chess_flag = (int) ((bool)((chessPosX + chessPosY) % 2)) + 1;
                                if (chess_flag == 1)
                                  chessFlag[kRay] = intensity_1;
                                else
                                  chessFlag[kRay] = intensity_2;
                        }
//                        else
//                            chessFlag[kRay] = 0;
                }
        }
}

void SPV::computeFloorFlag(cv::Mat XiAbs, std::vector<cv::Point2f> layout2D, std::vector<int> & chessFlag, int intensity)
{
        for (int kRay = 0; kRay < XiAbs.cols; kRay++)
        {
//            if (chessFlag[kRay] < 3)
//            {
                cv::Mat Xi = XiAbs.col(kRay);
                        cv::Mat x = lineXYPlaneMeetMat(0, Xi.rowRange(0, 3), Xi.rowRange(3, 6));

                        // +1 if inside
                        double inPolygon = pointPolygonTest(layout2D, cv::Point2f(float(x.at<double>(0)), float(x.at<double>(1))), false);

                        if (inPolygon == 1)
                        {
                                // The ray intersects the polygon and the 3D coordinates of the point are in x
                                chessFlag[kRay] = intensity;
                        }
//                }
        }
}


void SPV::computeChessFlagConvexHull(cv::Mat XiAbs, std::vector<cv::Point2f> layout2D, cv::Mat sidesL, double d, double dChessX, double dChessY, std::vector<int> & chessFlag, int intensity_1, int intensity_2)
{
        for (int kRay = 0; kRay < XiAbs.cols; kRay++)
        {
//            if (chessFlag[kRay] < 3)
//            {
                cv::Mat Xi = XiAbs.col(kRay);
//                int intersectFlag = isFaceSingleRayIntersectionMat(sidesL, Xi);
//                if (intersectFlag == 1)
//                {
                        cv::Mat x = lineXYPlaneMeetMat(d, Xi.rowRange(0, 3), Xi.rowRange(3, 6));

                        // +1 if inside
                        double inPolygon = pointPolygonTest(layout2D, cv::Point2f(float(x.at<double>(0)), float(x.at<double>(1))), false);

                        if (inPolygon == 1)
                        {
                                // The ray intersects the polygon and the 3D coordinates of the point are in x

                                int chessPosX = (int)(floor(x.at<double>(0) / dChessX)) % 2;
                                int chessPosY = (int)(floor(x.at<double>(1) / dChessY)) % 2;

                                /* Implementar un xor */
//                                chessFlag[kRay] = (int) ((bool)((chessPosX + chessPosY) % 2)) + 1;
                                int chess_flag = (int) ((bool)((chessPosX + chessPosY) % 2)) + 1;
                                if (chess_flag == 1)
                                  chessFlag[kRay] = intensity_1;
                                else
                                  chessFlag[kRay] = intensity_2;
                        }
//                        else
//                            chessFlag[kRay] = 0;
//                }
//                }
        }
}




int SPV::isFaceSingleRayIntersectionMat(cv::Mat & sidesL, cv::Mat &Xi)
{

        int nSides = sidesL.cols;

        int posAcum = 0;
        int negAcum = 0;

        for (int k = 0; k < nSides; k++)
        {
                double s = twoLinesSide4<double>(sidesL.col(k), Xi);
                if (s>0) posAcum++; else if (s<0) negAcum++;
        }

        int intersectFlag = 0;

        //if (negAcum == nSides) intersectFlag = -1;
        if (posAcum == nSides) intersectFlag = 1;

        return(intersectFlag);



}

template<typename T>
T SPV::twoLinesSide4(cv::Mat G, cv::Mat H)
{
        T s = G.at<T>(0)*H.at<T>(3) + G.at<T>(1)*H.at<T>(4) + G.at<T>(2)*H.at<T>(5) +
                G.at<T>(3)*H.at<T>(0) + G.at<T>(4)*H.at<T>(1) + G.at<T>(5)*H.at<T>(2);
        return(s);
}

cv::Mat SPV::lineXYPlaneMeetMat(double d, cv::Mat l, cv::Mat lBar)
{
        cv::Mat x = cv::Mat_<double>::zeros(3, 1);
        x.at<double>(0) = (d*l.at<double>(0) - lBar.at<double>(1)) / l.at<double>(2);
        x.at<double>(1) = (d*l.at<double>(1) + lBar.at<double>(0)) / l.at<double>(2);
        x.at<double>(2) = d;
        return(x);
}

cv::Mat SPV::linePlaneMeet(double u0, cv::Mat u, cv::Mat l, cv::Mat lBar)
{
    double x0 = u.dot(l);
    cv::Mat x = -u0*l + u.cross(lBar);
    return(x / x0);
}

void SPV::plotSprite(cv::Mat &img, cv::Mat& sprite, int xIni, int yIni)
{
  for (int i = 0; i < sprite.rows; i++)
  {
    if (yIni + i >= 0 && yIni + i < img.rows)
    {
      for (int j = 0; j < sprite.cols; j++)
      {
        if (xIni + j >= 0 && xIni + j < img.cols)
        {
          img.at<cv::Vec3b>(yIni + i, xIni + j) += sprite.at<cv::Vec3b>(i, j);
        }
      }
    }
  }
}

void SPV::visualizeChessFloor(std::vector<cv::Point2f> polygon, cv::Mat XiAbs, cv::Mat G, Eigen::Affine3d T, std::vector<int> & phospheneFlag)
{
  if (polygon.size() > 0)
  {
    computeChessFlag(XiAbs, polygon, phospheneFlag, N_levels, floor(N_levels/2));
    computeFloorPolygonEdges(polygon, T, G, phospheneFlag, N_levels);

  }
  computeVanishingLines(T, G, phospheneFlag, N_levels);
}

void SPV::visualizeWallObstacles(std::vector<Plane> vPlanes, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vObstacles, cv::Mat XiAbs, Eigen::Affine3d T, std::vector<int> & phospheneFlag, bool absolute)
{

  std::vector<double> phospheneDepth(size_t(XiCam.cols), 0);

  for (int Q = 0; Q < vPlanes.size(); Q++)
  {
    if (vPlanes[Q].type == 1)
      computePlanePolygonIntersection(XiAbs, vPlanes[Q], phospheneFlag, phospheneDepth, T,  N_levels);
    else if (vPlanes[Q].type > 1)
      computePlanePolygonIntersection(XiAbs, vPlanes[Q], phospheneFlag, phospheneDepth, T, floor(N_levels/2));
  }

  for (int Q = 0; Q < vObstacles.size(); Q++)
  {
//    if (absolute)
//    {
//      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>());
//      pcl::transformPointCloud(*vObstacles[Q], *obstacle, T.inverse());
//      computeFlagObstacles(obstacle, lookUpTable, K_p, phospheneFlag, N_levels);
//    }
//    else
//    {
      computeFlagObstacles(vObstacles[Q], lookUpTable, K_p, phospheneFlag, N_levels);
//    }
  }
}

void SPV::visualizeWallObstaclesChessFloor(std::vector<cv::Point2f> polygon, std::vector<Plane> vPlanes, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vObstacles, cv::Mat XiAbs,  cv::Mat G, Eigen::Affine3d T, std::vector<int> & phospheneFlag, bool absolute)
{

  if (polygon.size() > 0)
  {
    computeChessFlag(XiAbs, polygon, phospheneFlag, floor(3*N_levels/4), floor(N_levels/4));
    computeFloorPolygonEdges(polygon, T, G, phospheneFlag, N_levels);
  }
//  computeVanishingLines(T, G, phospheneFlag, N_levels);

  std::vector<double> phospheneDepth(size_t(XiCam.cols), 0);

  for (int Q = 0; Q < vPlanes.size(); Q++)
  {
    if (vPlanes[Q].type == 1)
    computePlanePolygonIntersection(XiAbs, vPlanes[Q], phospheneFlag, phospheneDepth, T, N_levels);
    else if (vPlanes[Q].type > 1)
      computePlanePolygonIntersection(XiAbs, vPlanes[Q], phospheneFlag, phospheneDepth, T,floor(N_levels/2));
  }

  for (int Q = 0; Q < vObstacles.size(); Q++)
  {
//    if (absolute)
//    {
//      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>());
//      pcl::transformPointCloud(*vObstacles[Q], *obstacle, T.inverse());
//      computeFlagObstacles(obstacle, lookUpTable, K_p, phospheneFlag, N_levels);
//    }
//    else
//    {
      computeFlagObstacles(vObstacles[Q], lookUpTable, K_p, phospheneFlag, N_levels);
//    }
  }
}

//void SPV::visualizeColor(cv::Mat img, std::vector<int> & phospheneFlag)
//{

//  cv::Mat mat_gray;
//  cv::cvtColor(img, mat_gray, cv::COLOR_RGB2GRAY);

//  for (int k = 0; k < phosphenesPos.cols; k++)
//  {
//    int iIndex = (int)round(phosphenesPos.at<double>(1, k));
//    int jIndex = (int)round(phosphenesPos.at<double>(0, k));

//    double gray_data = double(mat_gray.at<unsigned char>(iIndex, jIndex));

//    phospheneFlag[k] = int(ceil(gray_data/255.0*double(N_levels)));


//  }
//}

void SPV::visualizeColor(cv::Mat img, std::vector<int> & phospheneFlag)
{

  cv::Mat mat_gray;
  cv::cvtColor(img, mat_gray, cv::COLOR_RGB2GRAY);


  for (int k = 0; k < rgbPos.cols; k++)
  {
    int iIndex = int(round(rgbPos.at<double>(1, k)));
    int jIndex = int(round(rgbPos.at<double>(0, k)));

    if (iIndex < 0 || iIndex >= img.rows || jIndex < 0 || jIndex >= img.cols )
        continue;

    double gray_data = double(mat_gray.at<unsigned char>(iIndex, jIndex));

    phospheneFlag[size_t(k)] = int(ceil(gray_data/255.0*double(N_levels)));


  }
}


void SPV::visualizeSegmentation(cv::Mat img, cv::Mat img_segmentation, std::vector<int> & phospheneFlag)
{

  cv::Mat mat_gray;
  cv::cvtColor(img, mat_gray, cv::COLOR_RGB2GRAY);


  for (int k = 0; k < rgbPos.cols; k++)
  {
    int iIndex = int(round(rgbPos.at<double>(1, k)));
    int jIndex = int(round(rgbPos.at<double>(0, k)));
    //std::cout << int(img_segmentation.at<unsigned char>(iIndex, jIndex)) << std::endl;
    if (iIndex < 0 || iIndex >= img.rows || jIndex < 0 || jIndex >= img.cols )
        continue;
    if (img_segmentation.at<unsigned char>(iIndex, jIndex)>0)
       {phospheneFlag[size_t(k)]=N_levels;} else

    {double gray_data = double(mat_gray.at<unsigned char>(iIndex, jIndex));

    phospheneFlag[size_t(k)] = int(ceil(gray_data/255.0*double(N_levels)));}


  }
}

void SPV::visualizeOnlySegmentation(cv::Mat img_segmentation, std::vector<int> & phospheneFlag)
{

  //cv::Mat mat_gray;
  //cv::cvtColor(img, mat_gray, cv::COLOR_RGB2GRAY);


  for (int k = 0; k < rgbPos.cols; k++)
  {
    int iIndex = int(round(rgbPos.at<double>(1, k)));
    int jIndex = int(round(rgbPos.at<double>(0, k)));
    //std::cout << int(img_segmentation.at<unsigned char>(iIndex, jIndex)) << std::endl;
    if (iIndex < 0 || iIndex >= img_segmentation.rows || jIndex < 0 || jIndex >= img_segmentation.cols )
        continue;
    if (img_segmentation.at<unsigned char>(iIndex, jIndex)>1)
       {phospheneFlag[size_t(k)]=N_levels;}


  }
}

void SPV::visualizeDepth(cv::Mat img, std::vector<int> & phospheneFlag)
{

  double min, max;
  cv::minMaxIdx(img, &min, &max);

  for (int k = 0; k < depthPos.cols; k++)
  {
    int iIndex = int(round(depthPos.at<double>(1, k)));
    int jIndex = int(round(depthPos.at<double>(0, k)));

    if (iIndex < 0 || iIndex >= img.rows || jIndex < 0 || jIndex >= img.cols )
        continue;

    double depth_data = double(img.at<float>(iIndex, jIndex));
    if (depth_data < max_depth)
    {

      if (min > 0)
        phospheneFlag[k] = int(ceil((max_depth-((depth_data-min)*max_depth/(max_depth-min)))/max_depth*float(N_levels)));
      else
        phospheneFlag[k] = int(ceil((max_depth-depth_data)/max_depth*float(N_levels)));

    }

//    std::cout << depth_data << std::endl;

//    phospheneFlag[k] = int(ceil(gray_data/255.0*double(N_levels)));


  }
}

void SPV::visualizeDepthAugmented(cv::Mat img, std::vector<int> & phospheneFlag, Eigen::Affine3d T)
{

  double min, max;
  cv::minMaxIdx(img, &min, &max);

  for (int k = 0; k < depthPos.cols; k++)
  {
    int iIndex = int(round(depthPos.at<double>(1, k)));
    int jIndex = int(round(depthPos.at<double>(0, k)));

    if (iIndex < 0 || iIndex >= img.rows || jIndex < 0 || jIndex >= img.cols )
        continue;

    double depth_data = double(img.at<float>(iIndex, jIndex));


    if (depth_data < max_depth)
    {
      Eigen::Vector3d depth_point(XiCam.at<double>(0,k)*depth_data, XiCam.at<double>(1,k)*depth_data, XiCam.at<double>(2,k)*depth_data);
      Eigen::Vector3d depth_point_Abs = T.rotation()*depth_point + T.translation();

      if (depth_point_Abs(2) < 0.10)
      {
        phospheneFlag[k] = 1;
      }
      else
      {
        if (min > 0 & depth_data >= min)
          phospheneFlag[k] = int(ceil((max_depth-((depth_data-min)*max_depth/(max_depth-min)))/max_depth*float(N_levels)));
        else
          phospheneFlag[k] = int(ceil((max_depth-depth_data)/max_depth*float(N_levels)));
      }
    }



  }
}


void SPV::visualizeCanny(cv::Mat img, std::vector<int> & phospheneFlag)
{
  int kernel_size = 3;
  int ratio = 3; //3 //2
  int lowThreshold = 25; //25 //85 //Cuanto mayor es, menos lineas saca

  cv::Mat gray, edge;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  cv::Canny( gray, edge, lowThreshold, lowThreshold*ratio, kernel_size);
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy_line;

  cv::findContours( edge, contours, hierarchy_line, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  int nBounds = contours.size();
  for(int k = 0;k<nBounds;k++)
  {
    for (int p = 0; p<contours[k].size(); p++)
    {

      int row = int(K_p.at<double>(1,1)*((contours[k][p].y - K_c.at<double>(1,2))/K_c.at<double>(1,1)) + K_p.at<double>(1,2));
      int col = int(K_p.at<double>(0,0)*((contours[k][p].x - K_c.at<double>(0,2))/K_c.at<double>(0,0)) + K_p.at<double>(0,2));

      if (row < 0 || row >= size_img_y || col < 0 || col >= size_img_x )
          continue;

      int phosphene_index = lookUpTable.at<int>(row,col) - 1;

      if (phospheneFlag[phosphene_index] == 0)
      {

        double xDiff = phosphenesPos.at<double>(0,phosphene_index) - col;
        double yDiff = phosphenesPos.at<double>(1,phosphene_index) - row;

        double distance = std::sqrt((xDiff * xDiff) + (yDiff * yDiff));

        if (distance < thDist)
        {
          phospheneFlag[phosphene_index] = N_levels;
        }
      }
    }
  }



}

void SPV::visualizeCannyDepth(cv::Mat img, std::vector<int> & phospheneFlag)
{

  double min, max;
  cv::minMaxIdx(img, &min, &max);
  cv::Mat adjMap;
  cv::convertScaleAbs(img, adjMap, 255.0 / max, -min);

  int kernel_size = 3;
  int ratio = 3; //3 //2
  int lowThreshold = 25; //25 //85 //Cuanto mayor es, menos lineas saca

  cv::Mat edge;
//  cv::cvtColor(adjMap, gray, cv::COLOR_BGR2GRAY);
  cv::Canny( adjMap, edge, lowThreshold, lowThreshold*ratio, kernel_size);
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy_line;

  cv::findContours( edge, contours, hierarchy_line, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  int nBounds = contours.size();
  for(int k = 0;k<nBounds;k++)
  {
    for (int p = 0; p<contours[k].size(); p++)
    {
//      int phosphene_index = lookUpTable.at<int>(contours[k][p].y,contours[k][p].x) - 1;
        int row = int(K_p.at<double>(1,1)*((contours[k][p].y - K_d.at<double>(1,2))/K_d.at<double>(1,1)) + K_p.at<double>(1,2));
        int col = int(K_p.at<double>(0,0)*((contours[k][p].x - K_d.at<double>(0,2))/K_d.at<double>(0,0)) + K_p.at<double>(0,2));

        if (row < 0 || row >= size_img_y || col < 0 || col >= size_img_x )
            continue;
        int phosphene_index = lookUpTable.at<int>(row,col) - 1;

      if (phospheneFlag[phosphene_index] == 0)
      {
//        double xDiff = phosphenesPos.at<double>(0,phosphene_index) - contours[k][p].x;
//        double yDiff = phosphenesPos.at<double>(1,phosphene_index) - contours[k][p].y;
          double xDiff = phosphenesPos.at<double>(0,phosphene_index) - col;
          double yDiff = phosphenesPos.at<double>(1,phosphene_index) - row;

        double distance = std::sqrt((xDiff * xDiff) + (yDiff * yDiff));

        if (distance < thDist)
        {
          phospheneFlag[phosphene_index] = N_levels;
        }
      }
    }
  }



}

void SPV::keyAction(char key)
{

    switch (key)
    {
    // VISUAL REPRESENTATIONS
    case '0':
      if (segmentation_mode)
        segmentation_mode = false;
      else
        segmentation_mode = true;
      std::cout << "Segmentation mode enabled!" << std::endl;
      break;
    case '3':
        if (N_levels < 2) { N_levels = 2;   updateSprite(); }
        mode = 1;
        std::cout << "Chess-Floor mode enabled!" << std::endl;
        break;
    case '4':
        if (N_levels < 2) { N_levels = 2;   updateSprite(); }
        mode = 2;
        std::cout << "Wall-Obstacles mode enabled!" << std::endl;
        break;
    case '5':
        if (N_levels < 4) { N_levels = 4;   updateSprite(); }
        mode = 3;
        std::cout << "Wall-Obstacles-Chess-Floor mode enabled!" << std::endl;
        break;
    case '6':
        if (N_levels < 7) { N_levels = 7;   updateSprite(); }
        mode = 4;
        std::cout << "Black and white intensity mode enabled!" << std::endl;
        break;
    case '7':
        if (N_levels < 4) { N_levels = 4;   updateSprite(); }
        mode = 5;
        std::cout << "Depth mode enabled!" << std::endl;
        break;
    case '8':
        if (N_levels < 4) { N_levels = 4;   updateSprite(); }
        mode = 6;
        std::cout << "Augmented depth mode enabled!" << std::endl;
        break;
    case '9':
        mode = 7;
        std::cout << "Canny mode enabled!" << std::endl;
        break;


        // SPV CONFIGURATION
        // Increse/decrease size of phosphene
    case 42: // *
    {
        updateSprite(1.1);
//        std::cout << "Phosphene sprite size is increased to " << size_sprite << " pixels" << std::endl;
        std::cout << "The phosphene size has increased to " << FOV_sprite*180/M_PI << " degrees" << std::endl;
        break;
    }
    case 47: // /
    {
        updateSprite(0.9);
        std::cout << "The phosphene size has decreased to " << FOV_sprite*180/M_PI << " degrees" << std::endl;
//        std::cout << "Phosphene sprite size is decreased to " << size_sprite << " pixels" << std::endl;
        break;
    }
        // Increase/decrease separation between phosphenes
    case 43: // + --> Less separation, more phosphenes
    {
//        changeDelta(-2);
        N_fos = N_fos * 1.1;
        computeDeltaFromNumberOfPhosphenes(deltaX,deltaY);
        updatePhosphenes();
        break;
    }
    case 45: // - --> More separation, less phosphenes
    {
//        changeDelta(2);
        N_fos = N_fos * 0.9;
        computeDeltaFromNumberOfPhosphenes(deltaX,deltaY);
        updatePhosphenes();
        break;
    }
        // Increase/decrease radius of circle with phosphenes inside
    case 'v':
    {
        FOV = FOV*0.9;
        f = getFocalLengthFromFOV(FOV, size_img_x);
        K_p.at<double>(0, 0) = f;
        K_p.at<double>(1, 1) = f;
//        rExt = rExt*1.1;
        updatePhosphenes();
        updateSprite();
        std::cout << "The field of view has decreased to " << FOV*180/M_PI << " degrees" << std::endl;
//        std::cout << "The radius of the circle with phosphenes inside is increased to " << rExt << std::endl;
        break;
    }
    case 'V':
    {
        FOV = FOV*1.1;
        f = getFocalLengthFromFOV(FOV, size_img_x);
        K_p.at<double>(0, 0) = f;
        K_p.at<double>(1, 1) = f;
//        rExt = rExt*1.1;
        updatePhosphenes();
        updateSprite();
        std::cout << "The field of view has increased to " << FOV*180/M_PI << " degrees" << std::endl;
//        rExt = rExt*0.9;
        break;
    }
        // Increase/decrease noise stdev
    case 'n':
    {
        noise_stddev += 1.0;
        updatePhosphenes();
        std::cout << "The stddev of the noise of the phosphenes is increased to " << noise_stddev << std::endl;
        break;
    }
    case 'N':
    {
        if (noise_stddev > 0.0)
        {
            noise_stddev -= 1.0;
            updatePhosphenes();
            std::cout << "The stddev of the noise of the phosphenes is decreased to " << noise_stddev << std::endl;
        }
        break;
    }
        // Increase/decrease dropout ratio
    case 'o':
    {
        dropout += 0.05;
        std::cout << "The dropout ratio is increased to " << dropout << std::endl;
        updatePhosphenes();
        break;
    }
    case 'O':
    {
        if (dropout > 0.0)
        {
            dropout -= 0.05;
            std::cout << "The dropout ratio is decreased to " << dropout << std::endl;
            updatePhosphenes();
        }
        break;
    }
        // Increase/decrease number of levels
    case 'l':
    {
        N_levels++;
        std::cout << "The number of levels is increased to " << N_levels << std::endl;
        updateSprite();
        break;
    }
    case 'L':
    {
        if (N_levels > 1)
        {
            N_levels--;
            std::cout << "The number of levels is decreased to " << N_levels << std::endl;
            updateSprite();
        }
        break;
    }
        // Modifiers from specific representations:
        // - Chess pattern size for modes 1 and 3
        // - Max depth from modes 5 and 7
    case 'j':
    {
        if (mode == 1 || mode == 3)
        {
            dChessX = dChessX + 0.25;
            dChessY = dChessY + 0.25;
            std::cout << "Size of checkerboard tile is increased to " << dChessX << "x" << dChessY << std::endl;
        }
        else if (mode == 5 || mode == 6)
        {
            max_depth += 0.5;
            std::cout << "Max depth is increased to " << max_depth << "m" << std::endl;
        }
        break;
    }
    case 'J':
    {
        if (mode == 1 || mode == 3)
        {
            if (dChessX > 0.25)
            {
                dChessX = dChessX - 0.25;
                dChessY = dChessY - 0.25;
                std::cout << "Size of checkerboard tile is decreased to " << dChessX << "x" << dChessY << std::endl;
            }
            else
                std::cout << "Enough, you cannot make the grid smaller" << std::endl;
        }
        else if (mode == 5 || mode == 6)
        {
            if (max_depth >= 1.0)
            {
                max_depth -= 0.5;
                std::cout << "Max depth is decreased to " << max_depth << "m" << std::endl;
            }
        }
        break;
    }
        // Toggle grid mode: rectangular/hexagonal
    case '.':
    {
        if (grid_mode == 1)
            grid_mode = 2;
        else
            grid_mode = 1;

        computeDeltaFromNumberOfPhosphenes(deltaX,deltaY);
        updatePhosphenes();
        break;
    }
    // Toggle map mode: rectangular/circular
    case ',':
    {
        if (map_mode == 1)
        {
            map_mode = 2;
            rExt = std::min(size_img_x/2, size_img_y/2);
        }
        else
        {
            map_mode = 1;
            rExt = sqrt(x0*x0+y0*y0);
        }

        computeDeltaFromNumberOfPhosphenes(deltaX,deltaY);
        updatePhosphenes();
        break;
    }


    }
}

void SPV::visualize(std::vector<int> phospheneFlag, cv::Mat &img)
{
  for (int k = 0; k < phosphenesPos_vis.cols; k++)
  {
    int iIndex = (int)round(phosphenesPos_vis.at<double>(1, k));
    int jIndex = (int)round(phosphenesPos_vis.at<double>(0, k));
    if (phospheneFlag[k] >= 1)
    {
      plotSprite(img, sprites[phospheneFlag[k]], jIndex - (size_sprite - 1) / 2, iIndex - (size_sprite - 1) / 2);
    }
  }

}

void SPV::dimPhosphenes(std::vector<int> & phospheneFlag, double factor)
{
  for (int i = 0; i < phospheneFlag.size(); i++)
  {
    phospheneFlag[i] = ceil(phospheneFlag[i]*factor);
  }

}

//NAVIGATION
void SPV::plotPoint(std::vector<geometry_msgs::PoseStamped> &path, Eigen::Affine3d T, std::vector<int> &phospheneFlag, int intensity){
  if(path.size()<2){
    return;
  }
  std::vector<cv::Point3d> points;
  points.clear();
  size_t dimv;
  //if(path.size()>=15){
  //  points.resize(15);
    //dimv = 15;
  //}else{
    points.resize(path.size());
    dimv = path.size();
//  }
  //std::cout << N_levels << '\n';
  for(size_t i = 0;i<dimv;i++)
  {
    points[i].x = path[i].pose.position.x;
    points[i].y = path[i].pose.position.y;
    points[i].z = path[i].pose.position.z;
    // ROS_INFO("point %d, X: %f",i,points[i].x);
    // ROS_INFO("point %d, Y: %f",i,points[i].y);
    // ROS_INFO("point %d, Z: %f",i,points[i].z);
  }
  Eigen::Affine3d Tabs_c = T.inverse();
  //Eigen::Affine3d Tabs_c = T;
  for (size_t i = 0; i< points.size()-1; i++)
  {
    cv::Point3d X1 = points[i];
    Eigen::Vector3d X1_v(X1.x,X1.y,X1.z);
    Eigen::Vector3d X1Cam = Tabs_c.rotation()*X1_v+Tabs_c.translation();
    //ROS_INFO("point %d, X: %f",i,X1Cam(0));
    //ROS_INFO("point %d, Y: %f",i,X1Cam(1));
    //ROS_INFO("point %d, Z: %f",i,X1Cam(2));
    Eigen::Vector2d pixel (round(X1Cam(0)*K_p.at<double>(0,0)/X1Cam(2)+K_p.at<double>(0,2)),round(X1Cam(1)*K_p.at<double>(1,1)/X1Cam(2)+K_p.at<double>(1,2)));
    //ROS_INFO("PIXEL X: %f, Y: %f",pixel(0),pixel(1));
    //ROS_INFO("KC X: %d, Y: %d",pixel(0),pixel(1));
    //std::cout << "KC: " << K_c << std::endl;
    if ((pixel(0) >= 0) && (pixel(0) < lookUpTable.cols) && (pixel(1) >= 0) && (pixel(1) < lookUpTable.rows))
    {
        //std::cout << "entrando" << std::endl;
        double id = lookUpTable.at<int>(pixel(1),pixel(0)) - 1;
        if (id >= 0)
            phospheneFlag[id] = intensity;
    }

    //std::cout<<"i: "<< i << '\n';

  }
}

bool SPV::plotPointWithDepth(std::vector<geometry_msgs::PoseStamped> &path, Eigen::Affine3d T, std::vector<int> &phospheneFlag, int intensity, cv::Mat depth_img){

  bool plot_path = false;
  if(path.size()<2){
    return false;
  }
  std::vector<cv::Point3d> points;
  points.clear();
  size_t dimv;
  //if(path.size()>=15){
  //  points.resize(15);
    //dimv = 15;
  //}else{
    points.resize(path.size());
    dimv = path.size();
//  }
  //std::cout << N_levels << '\n';
  for(size_t i = 0;i<dimv;i++)
  {
    points[i].x = path[i].pose.position.x;
    points[i].y = path[i].pose.position.y;
    points[i].z = path[i].pose.position.z;
    // ROS_INFO("point %d, X: %f",i,points[i].x);
    // ROS_INFO("point %d, Y: %f",i,points[i].y);
    // ROS_INFO("point %d, Z: %f",i,points[i].z);
  }
  Eigen::Affine3d Tabs_c = T.inverse();
  //Eigen::Affine3d Tabs_c = T;
  for (size_t i = 0; i< points.size()-1; i++)
  {
    cv::Point3d X1 = points[i];
    Eigen::Vector3d X1_v(X1.x,X1.y,X1.z);
    Eigen::Vector3d X1Cam = Tabs_c.rotation()*X1_v+Tabs_c.translation();
    //ROS_INFO("point %d, X: %f",i,X1Cam(0));
    //ROS_INFO("point %d, Y: %f",i,X1Cam(1));
    //ROS_INFO("point %d, Z: %f",i,X1Cam(2));
    Eigen::Vector2d pixel (round(X1Cam(0)*K_p.at<double>(0,0)/X1Cam(2)+K_p.at<double>(0,2)),round(X1Cam(1)*K_p.at<double>(1,1)/X1Cam(2)+K_p.at<double>(1,2)));
    // ROS_INFO("PIXEL X: %f, Y: %f",pixel(0),pixel(1));
    // std::cout << "LU: " << lookUpTable.cols << " " << lookUpTable.rows << std::endl;
    //ROS_INFO("KC X: %d, Y: %d",pixel(0),pixel(1));
    //std::cout << "KC: " << K_c << std::endl;
    if ((pixel(0) >= 0) && (pixel(0) < lookUpTable.cols) && (pixel(1) >= 0) && (pixel(1) < lookUpTable.rows))
    {
        //std::cout << "entrando" << std::endl;
        double id = lookUpTable.at<int>(pixel(1),pixel(0)) - 1;
        // std::cout << "id: " << id << std::endl;
        if (id >= 0)
        {
          int iIndex = int(round(depthPos.at<double>(1, id)));
          int jIndex = int(round(depthPos.at<double>(0, id)));
          double depth_data = double(depth_img.at<float>(iIndex, jIndex));
          // std::cout << depth_data << std::endl;
          if (~isnan(depth_data) && depth_data > X1Cam(2) - 0.20)
          {
            phospheneFlag[id] = intensity;
            plot_path = true;
          }

        }
    }

    //std::cout<<"i: "<< i << '\n';

  }

  return plot_path;
}


bool SPV::plotPathOnImage(std::vector<geometry_msgs::PoseStamped> &path,cv::Mat XiAbs, Eigen::Affine3d T, std::vector<int> & phospheneFlag, cv::Mat depth_img)
{
  //ROS_INFO_STREAM("Input" << path);compute
  //ROS_INFO_STREAM("Input" << &path);
  // plotPoint(path,T,phospheneFlag,N_levels);
  bool plot_path = plotPointWithDepth(path,T,phospheneFlag,N_levels,depth_img);

  return plot_path;

}

void SPV::updateDirArrow(std::vector<geometry_msgs::PoseStamped> &path, tf::StampedTransform T, std::vector<int> & phospheneFlag)
{
  if(path.size()<1){
    return;
  }
   //Tpr =
   //double yaw_d = atan2(path[3].pose.position.y-T.translation()[1],path[3].pose.position.x-T.translation()[0]);

   int p2 = selectTrajectoryPoint(path,1.5);        //select the point of the path that is X m away from the robot.

   tf::Pose pose2;
   tf::Pose pose1;
   tf::poseMsgToTF(path[p2].pose,pose2);
   tf::poseMsgToTF(path[0].pose,pose1);

   pose2 = T.inverse()*pose2;
   pose1 = T.inverse()*pose1;



   double yaw_d = atan2(pose2.getOrigin().y()-pose1.getOrigin().y(),pose2.getOrigin().x()-pose1.getOrigin().x());
   //double yaw_d = atan2(path[1].pose.position.y,path[1].pose.position.x);
   double err;
   int max_err = (nPointsX_-2)/2;
   //double yaw_d2 = atan2(path[5].pose.position.y-T.translation()[1],path[5].pose.position.x-T.translation()[0]);
   //tf::Transform tmp;
   double roll,pitch,yaw;
   //tf::transformEigenToTF(T,tmp);
   tf::Matrix3x3 m(T.getBasis());
   m.getRPY(roll,pitch,yaw);
   // yaw_robot = atan2(T.rotation()(1,0),T.rotation()(0,0));
   //double F = size_img_y/FOV;
   err = yaw_d-yaw + M_PI;     //to se the values betweem -pi to pi




   //Look nearest value
//   double diff;
//   double diff_old = yaw_d-yaws[0];
//   int yaw_index;
//   for(int i = 1;i < yaws.size();i++){
//     diff = yaw_d - yaws[i];
//     if(diff-diff_old)>0{
//       yaw_index = i-1;
//       break;
//     }else{
//       yaw_index = i;
//     }
//   }
   int best_i = 0;
   double min_diff = 100;

   for (int i=0; i< indexes.size(); i++)
   {
     double diff = fabs(yaws[i]-yaw_d);
     if (diff < min_diff)
     {
       min_diff = diff;
       best_i = i;
     }
   }
    std::cout << "YAW vector: " << yaws[best_i] << std::endl;
    std::cout << "Yaw camera: " << yaw << std::endl;
    std::cout << "Yaw des: " << yaw_d << std::endl;
    std::cout << "Yaw diff: " << err << std::endl;
   col2_up = indexes[best_i];
   col_up = col2_up - nPointsX_;

   col2_down = phosphenesPos.cols - nPointsX_*3 + 1 + col2_up;
   col_down = col2_down + nPointsX_ - 1;

//   if(abs(err)<1){
//     err = 0;
//   }else{
//     //if(err>0) err =  round(2*log10(err));
//     //if(err<0) err = -round(2*log10(abs(err)));
//     if(err>max_err) err = max_err;
//     if(err<-max_err) err = -max_err;
//   }
//            //
//   std::cout << "yaw_d: " << (180/M_PI)*yaw_d << ", yaw_robot: " << (180/M_PI)*yaw << std::endl;
//   std::cout << "error yaw: " << err << "\n";
//   std::cout << "nPointsX_: " << nPointsX_ << std::endl;

//   col_up = col_up_center; //-  err;
//   col2_up = col2_up_center;// - err;
//   col_down = col_down_center;// - err;
//   col2_down = col2_down_center;// - err;
//   std::cout << "col_up: " << col_up << std::endl;
//   std::cout << "col2_up: " << col2_up << std::endl;
//   std::cout << "col_down: " << col_down << std::endl;
//   std::cout << "col2_down: " << col2_down << std::endl;
//   std::cout << "size y: " << size_img_y << std::endl;
//   std::cout << "size x: " << size_img_x << std::endl;

   plotArrow(phospheneFlag,N_levels);

}

void SPV::plotArrow(std::vector<int> & phospheneFlag, int intensity)
{
  //flechaArriba
  phospheneFlag[col_up] = intensity;
  //phospheneFlag[col_up+1] = 2;
  phospheneFlag[col_up+1] = intensity;
  phospheneFlag[col2_up] = intensity;
  //phospheneFlag[col2_up+1] = 2;

  //flechaAbajo
  phospheneFlag[col_down] = intensity;
//  phospheneFlag[col_down+1] = 2;
  phospheneFlag[col_down+1] = intensity;
  phospheneFlag[col2_down] = intensity;
//  phospheneFlag[col2_down+1] = 2;
}

int SPV::selectTrajectoryPoint(std::vector<geometry_msgs::PoseStamped> &path, double thold)
{
  for(int i = 1;i<path.size();i++)
  {
    double d = sqrt(pow(path[i].pose.position.y-path[0].pose.position.y,2)+pow(path[i].pose.position.x-path[0].pose.position.x,2));
    if(d>=thold){return i;}
    if(i == path.size()-1){return i;}
  }
}


bool SPV::plotDoor(cv::Mat depth_img, double door_center_x, double door_center_y, double door_yaw, double door_width, double door_height, Eigen::Affine3d T, cv::Mat G, std::vector<int> & phospheneFlag)
{
  cv::Point3d pt1(door_center_x - door_width/2*cos(door_yaw), door_center_y - door_width/2*sin(door_yaw), 0);
  cv::Point3d pt2(door_center_x - door_width/2*cos(door_yaw), door_center_y - door_width/2*sin(door_yaw), door_height);
  cv::Point3d pt3(door_center_x + door_width/2*cos(door_yaw), door_center_y + door_width/2*sin(door_yaw), door_height);
  cv::Point3d pt4(door_center_x + door_width/2*cos(door_yaw), door_center_y + door_width/2*sin(door_yaw), 0);

  std::vector<int> phospheneFlag_aux(size_t(XiCam.cols), 0);
  std::vector<double> phospheneDepth(size_t(XiCam.cols), 0);
  computeSegmentGlobalCoordinates(pt1, pt2, T, G, phospheneFlag_aux, phospheneDepth, N_levels);
  computeSegmentGlobalCoordinates(pt2, pt3, T, G, phospheneFlag_aux, phospheneDepth, N_levels);
  computeSegmentGlobalCoordinates(pt3, pt4, T, G, phospheneFlag_aux, phospheneDepth, N_levels);
  bool plot_door = false;
  for (int k = 0; k<phospheneFlag_aux.size(); k++)
  {
    if (phospheneFlag_aux[k] > 0)
    {
      // int iIndex = int(round(depthPos.at<double>(1, k)));
      // int jIndex = int(round(depthPos.at<double>(0, k)));
      // double depth_data = double(depth_img.at<float>(iIndex, jIndex));
      // std::cout << phospheneDepth[k] << " " << depth_data << std::endl;
      // if (~isnan(depth_data) && depth_data > phospheneDepth[k] - 0.20)
      // {
        phospheneFlag[k] = N_levels;
        plot_door = true;
      // }
    }
  }


  //
  //
  // computeSegmentGlobalCoordinates(pt2, pt3, T, G, phospheneFlag, N_levels);
  // computeSegmentGlobalCoordinates(pt3, pt4, T, G, phospheneFlag, N_levels);

  return plot_door;

}

bool SPV::plotObject(cv::Mat depth_img, double object_center_x, double object_center_y, double object_yaw, double object_width, double object_height, Eigen::Affine3d T, cv::Mat G, std::vector<int> & phospheneFlag, cv::Mat XiAbs)
{
  // cv::Point3d pt1(object_center_x - object_width/2*cos(object_yaw), object_center_y - object_width/2*sin(object_yaw), 0);
  // cv::Point3d pt2(object_center_x - object_width/2*cos(object_yaw), object_center_y - object_width/2*sin(object_yaw), object_height);
  // cv::Point3d pt3(object_center_x + object_width/2*cos(object_yaw), object_center_y + object_width/2*sin(object_yaw), object_height);
  // cv::Point3d pt4(object_center_x + object_width/2*cos(object_yaw), object_center_y + object_width/2*sin(object_yaw), 0);

  // cv::Point3d pt1(object_center_x + object_width/2 - object_width/2*cos(object_yaw), object_center_y + object_width/2 - object_width/2*sin(object_yaw), 0);
  // cv::Point3d pt2(object_center_x - object_width/2 - object_width/2*cos(object_yaw), object_center_y - object_width/2 - object_width/2*sin(object_yaw), 0);
  // cv::Point3d pt3(object_center_x - object_width/2 + object_width/2*cos(object_yaw), object_center_y - object_width/2 + object_width/2*sin(object_yaw), 0);
  // cv::Point3d pt4(object_center_x + object_width/2 + object_width/2*cos(object_yaw), object_center_y + object_width/2 + object_width/2*sin(object_yaw), 0);

  cv::Point3d pt1(object_center_x - object_width/2, object_center_y + object_width/2, 0);
  cv::Point3d pt2(object_center_x - object_width/2, object_center_y - object_width/2, 0);
  cv::Point3d pt3(object_center_x + object_width/2, object_center_y - object_width/2, 0);
  cv::Point3d pt4(object_center_x + object_width/2, object_center_y + object_width/2, 0);
  cv::Point3d pt5 = pt1; pt5.z = object_height;
  cv::Point3d pt6 = pt2; pt6.z = object_height;
  cv::Point3d pt7 = pt3; pt7.z = object_height;
  cv::Point3d pt8 = pt4; pt8.z = object_height;


  std::vector<int> phospheneFlag_aux(size_t(XiCam.cols), 0);
  std::vector<double> phospheneDepth(size_t(XiCam.cols), 0);


  pcl::PointXYZ pt1p(pt1.x, pt1.y, pt1.z);
  pcl::PointXYZ pt2p(pt2.x, pt2.y, pt2.z);
  pcl::PointXYZ pt3p(pt3.x, pt3.y, pt3.z);
  pcl::PointXYZ pt4p(pt4.x, pt4.y, pt4.z);
  pcl::PointXYZ pt5p(pt5.x, pt5.y, pt5.z);
  pcl::PointXYZ pt6p(pt6.x, pt6.y, pt6.z);
  pcl::PointXYZ pt7p(pt7.x, pt7.y, pt7.z);
  pcl::PointXYZ pt8p(pt8.x, pt8.y, pt8.z);

  std::vector<pcl::PointXYZ> pts;
  pts.push_back(pt1p);pts.push_back(pt2p);pts.push_back(pt3p);pts.push_back(pt4p);
  pts.push_back(pt5p);pts.push_back(pt6p);pts.push_back(pt7p);pts.push_back(pt8p);

  std::vector<std::vector<int>> sides_to_paint;

  std::vector<int> v1 = { 0, 4, 5, 1 };
  std::vector<int> v2 = { 1, 5, 6, 2 };
  std::vector<int> v3 = { 2, 6, 7, 3 };
  std::vector<int> v4 = { 3, 7, 4, 0 };
  std::vector<int> v5 = { 4, 5, 6, 7 };

  // sides_to_paint.push_back(v1);
  // sides_to_paint.push_back(v2);
  // sides_to_paint.push_back(v3);
  // sides_to_paint.push_back(v4);
  sides_to_paint.push_back(v5);

  for (int side = 0; side<sides_to_paint.size(); side++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    for (int points_in_side = 0; points_in_side < sides_to_paint[side].size(); points_in_side++)
      object_cloud->points.push_back(pts[sides_to_paint[side][points_in_side]]);

    Eigen::Vector4f plane_coefficients;

    isPlane (object_cloud, plane_coefficients, 0.5f);

    Plane plane(object_cloud, plane_coefficients);
    plane.getCentroid();
    // plane.getContour();

    // std::cout << plane.contour->points.size() << std::endl;

    plane.coeffs2f = plane.coeffs;
    plane.centroid2f = plane.centroid;
    plane.cloud2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
    *plane.cloud2f = *plane.cloud;
    plane.contour.reset(new pcl::PointCloud<pcl::PointXYZ>());
    *plane.contour = *plane.cloud;
    plane.contour2f.reset(new pcl::PointCloud<pcl::PointXYZ>());
    *plane.contour2f = *plane.cloud;

    // std::cout << plane.contour2f->points[0].x << " " << plane.contour2f->points[0].y << " " << plane.contour2f->points[0].z << " " << std::endl;
    // std::cout << plane.contour2f->points[1].x << " " << plane.contour2f->points[1].y << " " << plane.contour2f->points[1].z << " " << std::endl;
    // std::cout << plane.contour2f->points[2].x << " " << plane.contour2f->points[2].y << " " << plane.contour2f->points[2].z << " " << std::endl;
    // std::cout << plane.contour2f->points[3].x << " " << plane.contour2f->points[3].y << " " << plane.contour2f->points[3].z << " " << std::endl;
    //
    // std::cout << N_levels << std::endl;

    computePlanePolygonIntersection(XiAbs, plane, phospheneFlag_aux, phospheneDepth, T, N_levels);
    // for (int i = 0; i<phospheneFlag_aux.size(); i++)
    //   std::cout << phospheneFlag_aux[i] << " ";

  }










  //computeSegmentGlobalCoordinates(pt1, pt2, T, G, phospheneFlag_aux, phospheneDepth, N_levels);
  //computeSegmentGlobalCoordinates(pt2, pt3, T, G, phospheneFlag_aux, phospheneDepth, N_levels);
  //computeSegmentGlobalCoordinates(pt3, pt4, T, G, phospheneFlag_aux, phospheneDepth, N_levels);

  bool plot_object = false;
  for (int k = 0; k<phospheneFlag_aux.size(); k++)
  {
    if (phospheneFlag_aux[k] > 0)
    {
      // int iIndex = int(round(depthPos.at<double>(1, k)));
      // int jIndex = int(round(depthPos.at<double>(0, k)));
      // double depth_data = double(depth_img.at<float>(iIndex, jIndex));
      // std::cout << phospheneDepth[k] << " " << depth_data << std::endl;
      // if (~isnan(depth_data) && depth_data > phospheneDepth[k] - 0.20)
      // {
        phospheneFlag[k] = N_levels;
        plot_object = true;

      // }
    }
  }

  // phospheneFlag = phospheneFlag_aux;


  //
  //
  // computeSegmentGlobalCoordinates(pt2, pt3, T, G, phospheneFlag, N_levels);
  // computeSegmentGlobalCoordinates(pt3, pt4, T, G, phospheneFlag, N_levels);

  return plot_object;

}
