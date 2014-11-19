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

#include <boost/algorithm/string/case_conv.hpp>

#ifndef _AT_VALIDATOR_HPP_
#define _AT_VALIDATOR_HPP_ _AT_VALIDATOR_HPP_

/**
 * NOTE: The validate functions have to be part of the boost namespace -
 *       even if typedef BinningT & FrameT are not part of boost (but tuple is).
 */
namespace boost {
  void validate(boost::any & v, const vector<string> & values, BinningT * target_type, int) {
    using namespace boost::program_options;

    static regex r("([0-9]+)(x|,)([0-9]+)");
    validators::check_first_occurrence(v);
    const string & s = validators::get_single_string(values);
    
    smatch match;
    if (regex_match(s, match, r)) {
      v = any(BinningT(lexical_cast<int>(match[1]), lexical_cast<int>(match[3])));
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }

  void validate(boost::any & v, const vector<string> & values, FrameT * target_type, int) {
    using namespace boost::program_options;

    static regex r("([0-9]+)(x|,)([0-9]+)(x|,)([0-9]+)(x|,)([0-9]+)");
    validators::check_first_occurrence(v);
    const string & s = validators::get_single_string(values);

    smatch match;
    if (regex_match(s, match, r)) {
      v = any(FrameT(lexical_cast<int>(match[1]), lexical_cast<int>(match[3]), lexical_cast<int>(match[5]), lexical_cast<int>(match[7])));
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }

  // Note: Also used for DimensionT since same tupple type...
  void validate(boost::any & v, const vector<string> & values, PositionT * target_type, int) {
    using namespace boost::program_options;

    static regex r("([0-9]*\\.?[0-9]+)(x|,)([0-9]*\\.?[0-9]+)");
    validators::check_first_occurrence(v);
    const string & s = validators::get_single_string(values);

    smatch match;
    if (regex_match(s, match, r)) {
      v = any(PositionT(lexical_cast<double>(match[1]), lexical_cast<double>(match[3])));
    } else {
      throw validation_error(validation_error::invalid_option_value);
    }
  }
};

#endif // _AT_VALIDATOR_HPP_
