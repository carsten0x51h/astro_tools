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

#ifndef _ASTRO_TOOLS_CONSOLE_APP_HPP_
#define _ASTRO_TOOLS_CONSOLE_APP_HPP_ _ASTRO_TOOLS_CONSOLE_APP_HPP_

#include "astro_tools_app.hpp"
#include "at_exception.hpp"

namespace AT {
  DEF_Exception(ATConsoleApp);
  DEF_Exception(UnknownCommand);

  class AstroToolsConsoleAppT : public AstroToolsAppT<AstroToolsConsoleAppT> {

  public:
    static const po::options_description getExtendedGeneralOptions() {
      po::options_description additionalGeneralOptions;
      additionalGeneralOptions.add_options()
      	("command", po::value<string>(), "Execute command.")
      	;
      return additionalGeneralOptions;
    }

    static void printHelpNotice(ostream & os) {
      os << "Type --help for more info." << endl;
    }

    static void printHelp(ostream & os) {
      // Print general help
      po::options_description allDescr;
      allDescr.add(AstroToolsAppT::getGeneralOptions());

      // Add plugin commands
      for (CmdOptionsMapT::const_iterator it = AstroToolsAppT::getConsoleCmdMap().begin(); it != AstroToolsAppT::getConsoleCmdMap().end(); ++it) {
      	allDescr.add(it->second.first);
      }
      os << allDescr << endl;
    }


    static void evaluateCmdLineArguments(int argc, char **argv, po::variables_map * outCmdLineOptionsMap) {
      AT_ASSERT(ATConsoleApp, outCmdLineOptionsMap, "Expect outCmdLineOptionsMap to be valid!");

      // Compose command options from loaded plugins
      po::options_description genericExtDescr;
      genericExtDescr.add(AstroToolsAppT::getGeneralOptions());

      po::positional_options_description pos;
      pos.add("command", 1).add("hidden", -1); // This is a hack to ignore options which are unknown at that time.

      // Possible exceptions: invalid_command_line_syntax, multiple_occurrences, invalid_option_value.
      // For now we handle all the same way.
      try {
	po::store(po::command_line_parser(argc, argv).options(genericExtDescr).allow_unregistered().positional(pos).run(), *outCmdLineOptionsMap);
	po::notify(*outCmdLineOptionsMap);
      } catch(std::exception & exc) {
	cerr << "CASE1..." << endl;
	printHelpNotice(cout);
	throw;
      }

      const string & cmd = (outCmdLineOptionsMap->count("command") ? (*outCmdLineOptionsMap)["command"].as<string>() : "");
      CmdOptionsMapT::const_iterator itCmd = AstroToolsAppT::getConsoleCmdMap().find(cmd);

      po::options_description allowedDescr;
      allowedDescr.add(AstroToolsAppT::getGeneralOptions());
      
      if (! cmd.empty()) {
	if (itCmd == AstroToolsAppT::getConsoleCmdMap().end()) {
	  const string exStr = "Command '" + cmd + "' is unknown.";
	  throw UnknownCommandExceptionT(exStr.c_str());
	} else {
	  // Just allow the options for the selected command...
	  allowedDescr.add(itCmd->second.first);
	}
      }

      sSelectedCmd = cmd;

      // Special case - if user typed help, 
      if (outCmdLineOptionsMap->count("help"))
	return;

      try {
	store(parse_command_line(argc, argv, allowedDescr), *outCmdLineOptionsMap);
	notify(*outCmdLineOptionsMap);
      } catch(std::exception & exc) {
	cerr << "CASE2..." << endl;
	printHelpNotice(cout);
	throw;
      }
    }

    static void init(int argc, char **argv) {
      AstroToolsAppT::init(argc, argv);
    }

    static void execute(int argc, char **argv) {
      CmdOptionsMapT::const_iterator itCmd = AstroToolsAppT::getConsoleCmdMap().find(sSelectedCmd);

      // Handle help...
      if (getCmdLineOptionsMap().count("help") || sSelectedCmd.empty()) {
	if (sSelectedCmd.empty()) {
	  printHelp(cout);
	} else {
	  cout << "Help for '" << sSelectedCmd << "'" << endl;
	  cout << itCmd->second.first << endl;
	}
      } else {
	if (itCmd != AstroToolsAppT::getConsoleCmdMap().end()) {
	  AT_ASSERT(ATConsoleApp, itCmd->second.second, "Function pointer expected to be valid.");
	  LOG(info) << "Executing command " << sSelectedCmd << "... (function-ptr: " << hex << (void*)itCmd->second.second << ")" << flush;
	  itCmd->second.second();
	  LOG(info) << "DONE!" << endl;
	}
      }

    }

  };
}; // end namespace AT


#endif // _ASTRO_TOOLS_CONSOLE_APP_HPP_
