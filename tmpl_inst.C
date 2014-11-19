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

#include <boost/program_options.hpp>
#include <string>

using namespace boost::program_options;
using namespace std;

/**
 * NOTE: All typed_values used by plugins - which are not used in the framework - 
 *       need to be instantiated here! Otherwise this results in a segfault when
 *       calling the program_options store() method. This also implies that plugins
 *       are only allowed to use types listed here. Yes, this is ugly... but
 *       currently we have no better solution... :(
 */
typed_value<string> * xStr = value<string>();
typed_value<int> * xInt = value<int>();
