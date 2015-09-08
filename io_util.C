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

#include "io_util.hpp"

// TODO: namespace AT

bool readFile(CImg<float> & cimg, const string & inFilename, long * outBitPix, bool inVerboseMode) {
  FITS::setVerboseMode(inVerboseMode);
  
  try {
    std::auto_ptr<FITS> pInfile(new FITS(inFilename, Read, true));
    PHDU& image = pInfile->pHDU(); 

    if (outBitPix) {
      *outBitPix = image.bitpix();
    }

    // read all user-specifed, coordinate, and checksum keys in the image
    image.readAllKeys();

    if (inVerboseMode) {
      cout << image << endl;
    }

    // Set image dimensions
    cimg.resize(image.axis(0) /*x*/, image.axis(1) /*y*/, 1/*z - HACK*/, 1 /*1 color*/);

    // HACK / FIXME: At this point we assume that there is only 1 layer!
    std::valarray<unsigned long> imgData;
    image.read(imgData);

    // For now we create a copy... maybe there is a better way to directly read data into CImg, later...
    // TODO: Which one is correct?
    //cimg_forXY(cimg, x, y) { cimg(x, cimg.height() - 1 - y) = imgData[cimg.offset(x, y)]; }
    cimg_forXY(cimg, x, y) { cimg(x, y) = imgData[cimg.offset(x, y)]; }

  } catch (FitsException&) {
    // will catch all exceptions thrown by CCfits, including errors found by cfitsio (status != 0)
    return false;
  }
};
