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

#ifndef _INDI_VALIDATOR_HPP_
#define _INDI_VALIDATOR_HPP_ _INDI_VALIDATOR_HPP_

#include <boost/algorithm/string/case_conv.hpp>

/**
 * NOTE: This validate belongs to FrameTypeT which is currently
 *       not part of any namespace - it may go to namespace AT later.
 */
void validate(boost::any & v, const vector<string> & values, FrameTypeT::TypeE * target_type, int) {
  using namespace boost::program_options;
    
  validators::check_first_occurrence(v);
  string s = validators::get_single_string(values);
  boost::to_upper(s);
  FrameTypeT::TypeE type = FrameTypeT::asType(s.c_str());

  if (type != FrameTypeT::_Count) {
    v = any(type);
  } else {
    throw validation_error(validation_error::invalid_option_value);
  }
}

#endif // _INDI_VALIDATOR_HPP_
