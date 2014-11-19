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

#ifndef _IO_UTIL_HPP_
#define _IO_UTIL_HPP_ _IO_UTIL_HPP_

#include <CImg.h>
#include <CCfits/CCfits>
#include <iostream>

#include <ctime>
#include <sys/time.h>
#include <unistd.h>

#include "at_exception.hpp"

using namespace cimg_library;
using namespace CCfits;
using namespace std;

// CCfits helper function
// See http://heasarc.gsfc.nasa.gov/fitsio/ccfits/html/cookbook.html
bool readFile(CImg<float> & cimg, const string & inFilename, bool inVerboseMode = false);

#endif // _IO_UTIL_HPP_
