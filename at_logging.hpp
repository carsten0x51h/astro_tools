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

#ifndef _LOGGING_HPP_
#define _LOGGING_HPP_ _LOGGING_HPP_

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/utility/empty_deleter.hpp>
#include <boost/log/utility/setup/console.hpp>

#include <boost/regex.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>

#include <fstream>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace po = boost::program_options;

using namespace logging::trivial;


template <unsigned SHORT_NAME>
struct OptionLevelT {
public:
  unsigned n;
  explicit OptionLevelT(unsigned n_ = 0) : n(n_) {}
  OptionLevelT & inc(unsigned by = 1) { n += by; return *this; }
  OptionLevelT & set(unsigned val) { n = val; return *this; }
};


template <unsigned SHORT_NAME>
void validate(boost::any & v, const std::vector<std::string> & values, OptionLevelT<SHORT_NAME> * /*target_type*/, int)
{
  using namespace boost::program_options;
  
  // 
  // Get the current value
  // 
  OptionLevelT<SHORT_NAME> i;
  if (!v.empty())
    i = boost::any_cast<OptionLevelT<SHORT_NAME> >(v);
  
  //
  //  Extract any arguments 
  // 
  const std::string& s = validators::get_single_string(values, true);
  if (s.empty()) {
    v = boost::any(i.inc());
    return;
  }
  
  char short_name = SHORT_NAME;
  
  // multiple 'values's
  if (s == std::string(s.length(), short_name)) {
    v = boost::any(i.inc(s.length() + 1));
    return;
  }
  
  // match number
  boost::regex r("^(\\d+)$");
  
  // Do regex match and convert the interesting part to int.
  boost::smatch what;
  if (regex_match(s, what, r)) {
    v = boost::any(i.set(boost::lexical_cast<unsigned>(s)));
    return;
  } else {
    throw validation_error(validation_error::invalid_option_value, "\"" + s + "\" is not a valid argument.");
  }        
}


template<class T, class charT = char>
class t_level_value : public boost::program_options::typed_value<T, charT> {
public:
  t_level_value(T * store_to) : boost::program_options::typed_value<T, charT>(store_to) {} 
  unsigned min_tokens() const { return 0; }
  unsigned max_tokens() const { return 1; }
};

template<class T>
t_level_value<T>* level_value(T* v) {
  return new t_level_value<T>(v);
}


// See http://stackoverflow.com/questions/24170577/simultaneous-logging-to-console-and-file-using-boost
#define LOG(level) BOOST_LOG_SEV(global_logger::get(), level)

typedef src::severity_channel_logger_mt<severity_level, std::string> global_logger_type;

BOOST_LOG_INLINE_GLOBAL_LOGGER_INIT(global_logger, global_logger_type)
{
  return global_logger_type(boost::log::keywords::channel = "global_logger");
}

// TODO: Do we need channels? See http://www.boost.org/doc/libs/1_54_0/libs/log/doc/html/log/detailed/sources.html
class LoggingT {
private:

public:
  static void init(const logging::trivial::severity_level & inLogSev = logging::trivial::debug) {
    logging::add_console_log(
			     std::cout,
    			     keywords::format = "[%TimeStamp%]: %Message%", /*< log record format >*/
    			     keywords::auto_flush = true
			     );

    // TODO / FIXME: where is logfile if just logging one line?!
    logging::add_file_log
      (
       keywords::file_name = "sample_%N.log",                                        /*< file name pattern >*/
       keywords::rotation_size = 10 * 1024 * 1024,                                   /*< rotate files every 10 MiB... >*/
       keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0), /*< ...or at midnight >*/
       keywords::format = "[%TimeStamp%]: %Message%"                                 /*< log record format >*/
       );
    
    logging::core::get()->set_filter(logging::trivial::severity >= inLogSev);

    // TODO: Required? What exactly does this function?!
    logging::add_common_attributes();
  }
  
};

#endif // _LOGGING_HPP_
