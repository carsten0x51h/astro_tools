/*****************************************************************************
 *
 *  AstroTools
 *
 *  Copyright(C) 2015 Carsten Schmitt <c.schmitt51h@gmail.com>
 *
 *  This program is free software ; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation ; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY ; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program ; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 ****************************************************************************/

/** 
    The star centroid algorithm is based on the code from: http://aladin.u-strasbg.fr/java/Plugins/Centroid.java
    Further information on image moments can be found on wikipedia: http://de.wikipedia.org/wiki/Moment_%28Bildverarbeitung%29
    Also see http://www.isprs.org/proceedings/XXXV/congress/comm3/papers/341.pdf
*/
#ifndef _CENTROID_HPP_
#define _CENTROID_HPP_ _CENTROID_HPP_

#include <CImg.h>
#include <valarray>

#include "at_exception.hpp"
#include "at_logging.hpp"
#include "util.hpp"

using namespace std;
using namespace cimg_library;

struct CoordTypeT {
  enum TypeE {
    RELATIVE,
    ABSOLUTE,
    _Count
  };
    
  static const char * asStr(const TypeE & inType) {
    switch (inType) {
    case RELATIVE: return "RELATIVE";
    case ABSOLUTE: return "ABSOLUTE";
    default: return "<?>";
    }
  }
};

DEF_Exception(Centroid);


class CentroidCalcT {
public:

  static void getMaxValueAndPosInRegion(const CImg<float> & inImg, int inX, int inY, int inW, int inH, int * outX, int * outY, double * outMaxValue, CoordTypeT::TypeE inCoordType) {
    if (! outX && ! outY && ! outMaxValue)
      return;
    
    // Crop the selection window if at boundary
    int x = (inX < 0 ? 0 : (inX > inImg.width() ? inImg.width() : inX));
    int y = (inY < 0 ? 0 : (inY > inImg.height() ? inImg.height() : inY));
    int w = ((inImg.width() >= inX + inW) ? inW : inImg.width() - inX);
    int h = ((inImg.height() >= inY + inH) ? inH : inImg.height() - inY);
    const CImg <float> & subImage = inImg.get_crop(x, y, x + w, y + h); // TODO: Reference possible?!
    
    // Find max pixel value and position
    double maxValue = 0;
    int maxX, maxY;
    cimg_forXY(subImage, x, y) {
      if (maxValue < subImage(x,y)) {
	maxValue = subImage(x,y);
	maxX = x; maxY = y;
      }
    }

    if (CoordTypeT::ABSOLUTE == inCoordType) {
      maxX += x;
      maxY += y;
    }

    if (outX) *outX = maxX;
    if (outY) *outY = maxY;
    if (outMaxValue) *outMaxValue = maxValue;
  }


  static PositionT starCentroid(const CImg<float> & inImg, PositionT inSelectionCenter, unsigned int inWindowSize, CoordTypeT::TypeE inCoordType) {
    // TODO: Later this one is by default...
    return starCentroid(inImg,
			inSelectionCenter.get<0>() - (inWindowSize / 2),
			inSelectionCenter.get<1>() - (inWindowSize / 2),
			inWindowSize, inWindowSize, inCoordType);
  }

  static PositionT starCentroid(const CImg<float> & inImg, int inX, int inY, int inW, int inH, CoordTypeT::TypeE inCoordType) {
    LOG(debug) << "starCentroid - image dimension (w h)=(" << inImg.width() << " " << inImg.height() << ")" << endl;

    int xPosMax = 0, yPosMax = 0;
    double maxValue = 0;
    getMaxValueAndPosInRegion(inImg, inX, inY, 2.0 * inW, 2.0 * inH, & xPosMax, & yPosMax, & maxValue, CoordTypeT::ABSOLUTE);

    // // 0. Crop image
    // const CImg <float> & subImg = inImg.get_crop(inX, inY, inX + inW, inY + inH);

    // // 1. Find max in delivered subImg
    // int xPosMax = 0, yPosMax = 0;
    // float maxValue = 0;
    // cimg_forXY(subImg, x, y) {
    //   if (maxValue < subImg(x,y)) {
    // 	maxValue = subImg(x,y);
    // 	xPosMax = x;
    // 	yPosMax = y;
    //   }
    // }

    LOG(debug) << "starCentroid - subImg maxValue: " << maxValue << ", xPosMax: " << xPosMax << ", yPosMax: " << yPosMax << endl;
    
    // HACK: We only need one size since it is always squared! inW ...

    // 2. Create information arrays for grid pixels
    // TODO: Parametrize?!
    // const int SIZE = 41; // TODO: How to determine?!
    // const int BACK = 20; // TODO: How to determine?!
    const int SIZE = inW; // TODO: How to determine?!
    //cerr << "SIZE: " << SIZE << endl;

    //const int BACK = 30; // TODO: How to determine?!
    const int BACK = ceil(SIZE / 2.0) - 1; // TODO: How to determine?!
    //cerr << "BACK: " << BACK << endl;

    std::valarray<float> vals(SIZE * SIZE);  // this array will contain all the intensities of the pixels in the SIZE*SIZE grid
    std::valarray<float> dist(SIZE * SIZE);  // contains all distances for each pixel to the central pixel
    std::valarray<float> distX(SIZE * SIZE); // contains the X distance for each pixel to the central pixel
    std::valarray<float> distY(SIZE * SIZE); // contains the Y distance for each pixel to the central pixel

    // X and Y are now central brightest pixel in SIZExSIZE grid
    int startX = xPosMax - BACK, startY = yPosMax - BACK;
    const CImg <float> & subInImg = inImg.get_crop(startX+1, startY+1, startX + SIZE, startY + SIZE);

    LOG(debug) << "starCentroid - subInImg size (w h)=(" << subInImg.width() << " " << subInImg.height() << ")" << endl;


    cimg_forXY(subInImg, x, y) {
      const long off = subInImg.offset(x, y);
      vals[off] = subInImg(x, y); // save pixel intensity
      distX[off] = (double)x - (double)xPosMax; // x distance of pixel from top left
      distY[off] = (double)y - (double)yPosMax; // y distance of pixel from top left
      dist[off] = sqrt(pow((double) distX[off], 2.0) + pow((double) distY[off], 2.0)); // distance of pixel from top-left, uses x and y to find hypotenuse
    }

    // 3. Find valueLow and valueHigh in grid
    float levelLow = vals[0], levelHigh = vals[0];

    for (size_t i = 0; i < vals.size(); ++i) {
      if(vals[i] < levelLow)
	levelLow = vals[i];
      if(vals[i] > levelHigh)
	levelHigh = vals[i];
    }

    LOG(debug) << "starCentroid - subInImg levelLow: " << levelLow << ", levelHigh: " << levelHigh << endl;

    // 4. Create secondMoment array
    // TODO: Simply code - once it works and does what it should!
    typedef vector<double> MomentValuesT;
    MomentValuesT secondMoment; // Do we know size before?!
    double levelCurrHighest = levelHigh;
    double levelNextHighest = 0;
    double currentSum = 0;

    // Find the second moment for levelHigh - for every pixel in the grid, check if intesity = current intensity level, find next highest intensity in grid
    for (size_t i = 0; i < vals.size(); ++i) {
      if(vals[i] == levelCurrHighest) {
	currentSum += sqrt((double) dist[i]); //sum the squares of the distances of the pixel from the center
      }
      if(vals[i] < levelCurrHighest && vals[i] > levelNextHighest){
	levelNextHighest = vals[i]; // next highest intensity in grid, used for iterations
      }
    }

    levelCurrHighest = levelNextHighest; // Set for next iteration
    secondMoment.push_back(currentSum); // Add currentSum to the secondMoment array    

    // Find second moment for all remaining levels
    int count = 1; // Keep track of level

    // FIXME / TODO: Here we had while(levelCurrHighest >= levelLow) {.... which lead to an endless loop (levelCurrHighest == levelLow == 0...)
    while(levelCurrHighest > levelLow) {
      currentSum = 0;
      levelNextHighest = 0;

      // For every pixel in the grid, check if intesity = current intensity level, find next highest intensity in grid
      for (size_t i = 0; i < vals.size(); i++) {
	if(vals[i] == levelCurrHighest)
	  currentSum += sqrt((double) dist[i]); // Sum the squares of the distances of the pixel from the center
	if(vals[i] < levelCurrHighest && vals[i] > levelNextHighest)
	  levelNextHighest = vals[i]; // Next highest intensity in grid, used for iterations
      }
      currentSum += secondMoment.at(count - 1);
      secondMoment.push_back(currentSum);
      count++;

      levelCurrHighest = levelNextHighest;
    } // end while

    LOG(trace) << "starCentroid - List of 2nd moments:" << endl;
    for (MomentValuesT::const_iterator it = secondMoment.begin(); it != secondMoment.end(); ++it) {
      LOG(trace) << "Level: " << std::distance<MomentValuesT::const_iterator>(secondMoment.begin(), it) << " - value: " << *it << endl;
    }
    

    // 5. Create filteredSecondMoment array
    MomentValuesT filteredSecondMoment(secondMoment);

    // For each value in second moment array, compute a normalized moving 5 point center weighted filter
    // (weights of 1,2,3,2,1, Normalization factor = 9) does not compute the first two or last two values of second moment.
    for (size_t i = 2; i < secondMoment.size() - 2; ++i) {
      filteredSecondMoment[i] = ((secondMoment[i - 2] + secondMoment[i - 1] * 2.0 + secondMoment[i] * 3.0 + secondMoment[i + 1] * 2.0 + secondMoment[i + 2]) / 9.0);
    }

    LOG(trace) << "starCentroid - List of filtered 2nd moments:" << endl;
    for (MomentValuesT::const_iterator it = filteredSecondMoment.begin(); it != filteredSecondMoment.end(); ++it) {
      LOG(trace) << "Level: " << std::distance<MomentValuesT::const_iterator>(filteredSecondMoment.begin(), it) << " - Filtered 2nd-Moment: " << *it << endl;
    }


    // 6. Find the Threshold
    int threshInt = 0; // Holds the threshold pixel
    bool check = true;
    int level = filteredSecondMoment.size() - 3; // Set search index to the levelLow and begin search there

    while (check) {
      // If the difference between level and lower level is < 100, threshold has been found
      double checkVal = fabs(filteredSecondMoment[level] - filteredSecondMoment[level - 1]);
      if (checkVal < 100) {
	threshInt = level;
	check = false;
      }
      level--;
      if (level <= 1) {
	throw CentroidExceptionT("Threshold could not be found!");
	check = false;
      }
    }         
    double threshold = vals[threshInt];

    LOG(debug) << "starCentroid - threshInt: " << threshInt << ", threshold: " << threshold << endl;


    // 7. Sum of all intensities, xSum, ySum, Threshold & Calculate Centroid
    double sum = 0; // Sum of all of the intensities in the grid
    double xSum = 0; // Sum of the product of the intensity of a pixel times it's x-distance from the center of the grid
    double ySum = 0; // Sum of the product of the intensity of a pixel times it's y-distance from the center of the grid

    // Threshold data and calculate the sums
    for (size_t i = 0; i < vals.size(); i++) {
      if(vals[i] <= threshold) {
	vals[i] = 0; // Pix intensity <= thres, set to 0
      } else {
	vals[i] = vals[i] - threshold; // Pix intensity > thres, calc subtraction
      }
      sum += vals[i]; // Sum of intensities
      xSum += vals[i] * (double) distX[i]; // Sum of intensities * distance X
      ySum += vals[i] * (double) distY[i]; // Sum of intensities * distance Y
    }

    // Calculate centroid
    if (! sum)
      throw CentroidExceptionT("Unable to determine centroid.");

    double xCentroid = xPosMax + xSum / sum;
    double yCentroid = yPosMax + ySum / sum;


    PositionT centroid;

    // Get closest int values for Xav and Yav
    if (CoordTypeT::RELATIVE == inCoordType) {
      centroid.get<0>() = xCentroid;
      centroid.get<1>() = yCentroid;
    } else if (CoordTypeT::ABSOLUTE == inCoordType) {
      centroid.get<0>() = xCentroid + startX + 1; // TODO: +1 required? ok?!
      centroid.get<1>() = yCentroid + +startY + 1;
    } else {
      AT_ASSERT(Centroid, false, "Not supported.");
    }

    LOG(info) << "starCentroid - (x y)=(" << xCentroid << " " << yCentroid << ")"
	      << ", transformed (x y)=" << centroid << endl;

    return centroid;
  }




  // TODO: Generalize code! No copy and paste!
  // static void starCentroid(const CImg<float> & image, float * centerx, float * centery, CImg<float> * outSelectedImage = 0, bool verbose = false) {
  //   if (verbose) {
  //     cout << "Image data - width: " << image.width() << ", height: " << image.height() << endl;
  //   }

  //   // 1. Find max in delivered image
  //   int xPosMax = 0, yPosMax = 0;
  //   float maxValue = 0;
  //   cimg_forXY(image, x, y) {
  //     if (maxValue < image(x,y)) {
  // 	maxValue = image(x,y);
  // 	xPosMax = x;
  // 	yPosMax = y;
  //     }
  //   }

  //   if (verbose) {
  //     cout << "image maxValue: " << maxValue << ", xPosMax: " << xPosMax << ", yPosMax: " << yPosMax << endl;
  //   }


  //   // 2. Create information arrays for grid pixels
  //   // TODO: Parametrize?!
  //   // const int SIZE = 41; // TODO: How to determine?!
  //   // const int BACK = 20; // TODO: How to determine?!
  //   //const int SIZE = 61; // TODO: How to determine?!
  //   //const int BACK = 30; // TODO: How to determine?!

  //   const int SIZE = image.width(); // TODO: How to determine?!
  //   //cerr << "SIZE: " << SIZE << endl;

  //   const int BACK = ceil(SIZE / 2.0) + 1; // TODO: How to determine?!
  //   //cerr << "BACK: " << BACK << endl;

  //   std::valarray<float> vals(SIZE * SIZE);  // this array will contain all the intensities of the pixels in the SIZE*SIZE grid
  //   std::valarray<float> dist(SIZE * SIZE);  // contains all distances for each pixel to the central pixel
  //   std::valarray<float> distX(SIZE * SIZE); // contains the X distance for each pixel to the central pixel
  //   std::valarray<float> distY(SIZE * SIZE); // contains the Y distance for each pixel to the central pixel

  //   // X and Y are now central brightest pixel in SIZExSIZE grid
  //   int startX = xPosMax - BACK, startY = yPosMax - BACK;
  //   //CImg <float> subImage = image.get_crop(startX+1, startY+1, startX + SIZE, startY + SIZE);
  //   CImg <float> subImage = image; // HACK!!!!!!
  //   if (verbose) {
  //     cout << "subImage size - width: " << subImage.width() << ", height: " << subImage.height() << endl;
  //   }

  //   xPosMax -= startX;
  //   yPosMax -= startY;

  //   cimg_forXY(subImage, x, y) {
  //     const long off = subImage.offset(x, y);
  //     vals[off] = subImage(x, y); // save pixel intensity
  //     distX[off] = (double)x - (double)xPosMax; // x distance of pixel from top left
  //     distY[off] = (double)y - (double)yPosMax; // y distance of pixel from top left
  //     dist[off] = sqrt(pow((double) distX[off], 2.0) + pow((double) distY[off], 2.0)); // distance of pixel from top-left, uses x and y to find hypotenuse
  //   }

  //   // 3. Find valueLow and valueHigh in grid
  //   float levelLow = vals[0], levelHigh = vals[0];

  //   for(size_t i = 0; i < vals.size(); ++i) {
  //     if(vals[i] < levelLow)
  // 	levelLow = vals[i];
  //     if(vals[i] > levelHigh)
  // 	levelHigh = vals[i];
  //   }

  //   if (verbose) {
  //     cout << "subImage levelLow: " << levelLow << ", levelHigh: " << levelHigh << endl;
  //   }


  //   // 4. Create secondMoment array
  //   // TODO: Simply code - once it works and does what it should!
  //   vector<double> secondMoment; // Do we know size before?!
  //   double levelCurrHighest = levelHigh;
  //   double levelNextHighest = 0;
  //   double currentSum = 0;

  //   // Find the second moment for levelHigh - for every pixel in the grid, check if intesity = current intensity level, find next highest intensity in grid
  //   for(size_t i = 0; i < vals.size(); ++i) {
  //     if(vals[i] == levelCurrHighest) {
  // 	currentSum += sqrt((double) dist[i]); //sum the squares of the distances of the pixel from the center
  //     }
  //     if(vals[i] < levelCurrHighest && vals[i] > levelNextHighest){
  // 	levelNextHighest = vals[i]; // next highest intensity in grid, used for iterations
  //     }
  //   }

  //   levelCurrHighest = levelNextHighest; // Set for next iteration
  //   secondMoment.push_back(currentSum); // Add currentSum to the secondMoment array    

  //   // Find second moment for all remaining levels
  //   int count = 1; // Keep track of level

  //   // FIXME / TODO: Here we had while(levelCurrHighest >= levelLow) {.... which lead to an endless loop (levelCurrHighest == levelLow == 0...)
  //   while(levelCurrHighest > levelLow) {
  //     currentSum = 0;
  //     levelNextHighest = 0;

  //     // For every pixel in the grid, check if intesity = current intensity level, find next highest intensity in grid
  //     for(size_t i = 0; i < vals.size(); i++) {
  // 	if(vals[i] == levelCurrHighest)
  // 	  currentSum += sqrt((double) dist[i]); // Sum the squares of the distances of the pixel from the center
  // 	if(vals[i] < levelCurrHighest && vals[i] > levelNextHighest)
  // 	  levelNextHighest = vals[i]; // Next highest intensity in grid, used for iterations
  //     }
  //     currentSum += secondMoment.at(count - 1);
  //     secondMoment.push_back(currentSum);
  //     count++;

  //     levelCurrHighest = levelNextHighest;
  //   } // end while

  //   // DEBUG START
  //   if (verbose) {
  //     int j=0;
  //     cout << "List of 2nd moments:" << endl;
  //     for (vector<double>::const_iterator it = secondMoment.begin(); it != secondMoment.end(); ++it, ++j) {
  // 	cout << "Level: " << j << " - value: " << *it << endl;
  //     }
  //   }
  //   // DEBUG END

  //   // 5. Create filteredSecondMoment array
  //   vector<double> filteredSecondMoment(secondMoment);

  //   // For each value in second moment array, compute a normalized moving 5 point center weighted filter
  //   // (weights of 1,2,3,2,1, Normalization factor = 9) does not compute the first two or last two values of second moment.
  //   for (size_t i = 2; i < secondMoment.size() - 2; ++i) {
  //     filteredSecondMoment[i] = ((secondMoment[i - 2] + secondMoment[i - 1] * 2.0 + secondMoment[i] * 3.0 + secondMoment[i + 1] * 2.0 + secondMoment[i + 2]) / 9.0);
  //   }

  //   // DEBUG START
  //   if (verbose) {
  //     cout << "List of filtered 2nd moments:" << endl;
  //     int k=0;
  //     for (vector<double>::const_iterator it = filteredSecondMoment.begin(); it != filteredSecondMoment.end(); ++it, ++k) {
  // 	cout << "Level: " << k << " - Filtered 2nd-Moment: " << *it << endl;
  //     }
  //   }
  //   // DEBUG END


  //   // 6. Find the Threshold
  //   int threshInt = 0; // Holds the threshold pixel
  //   bool check = true;
  //   int level = filteredSecondMoment.size() - 3; // Set search index to the levelLow and begin search there

  //   while (check) {
  //     // If the difference between level and lower level is < 100, threshold has been found
  //     double checkVal = fabs(filteredSecondMoment[level] - filteredSecondMoment[level - 1]);
  //     if (checkVal < 100) {
  // 	threshInt = level;
  // 	check = false;
  //     }
  //     level--;
  //     if (level <= 1) {
  // 	cerr << "Threshold could not be found!" << endl; // TODO: return code / throw exception?!
  // 	check = false;
  //     }
  //   }         
  //   double threshold = vals[threshInt];

  //   // DEBUG START
  //   if (verbose) {
  //     cout << "threshInt: " << threshInt << ", threshold: " << threshold << endl;
  //   }
  //   // DEBUG END


  //   // 7. Sum of all intensities, xSum, ySum, Threshold & Calculate Centroid
  //   double sum = 0; // Sum of all of the intensities in the grid
  //   double xSum = 0; // Sum of the product of the intensity of a pixel times it's x-distance from the center of the grid
  //   double ySum = 0; // Sum of the product of the intensity of a pixel times it's y-distance from the center of the grid

  //   // Threshold data and calculate the sums
  //   for(size_t i = 0; i < vals.size(); i++) {
  //     if(vals[i] <= threshold) {
  // 	vals[i] = 0; // Pix intensity <= thres, set to 0
  //     } else {
  // 	vals[i] = vals[i] - threshold; // Pix intensity > thres, calc subtraction
  //     }

  //     sum += vals[i]; // Sum of intensities
  //     xSum += vals[i] * (double) distX[i]; // Sum of intensities * distance X
  //     ySum += vals[i] * (double) distY[i]; // Sum of intensities * distance Y
  //   }

  //   // Calculate centroid
  //   double xCentroid = xPosMax + xSum / sum;
  //   double yCentroid = yPosMax + ySum / sum;

  //   // DEBUG START
  //   if (verbose) {
  //     cout << "xCentroid: " << xCentroid << ", yCentroid: " << yCentroid << endl;
  //   }
  //   // DEBUG END

  //   // // Get closest int values for Xav and Yav
  //   *centerx = xCentroid;
  //   *centery = yCentroid;

  //   // TODO: center coordinates currently belong to result image! FIXME!
  //   // *centerx = xCentroid + startX + 1;
  //   // *centery = yCentroid + startY + 1;


  //   if (outSelectedImage) {
  //     *outSelectedImage = subImage;
  //   }
  // }
};

#endif // _CENTROID_HPP_
