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

#include <boost/test/unit_test.hpp>

#include "at_logging.hpp"

using namespace std;

BOOST_AUTO_TEST_SUITE(BoostLogging)

/**
 * Boost logging test.
 */
BOOST_AUTO_TEST_CASE(BoostLoggingTest)
{
  // Init logging...
  LoggingT::init();

  cerr << "Logging to debug..." << flush;
  LOG(debug) << "DEBUG LOG..." << endl;
  cerr << "DONE!" << endl;
}

BOOST_AUTO_TEST_SUITE_END()
