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

#ifndef _ASTRO_TOOLS_APP_HPP_
#define _ASTRO_TOOLS_APP_HPP_ _ASTRO_TOOLS_APP_HPP_

#include <iosfwd>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <map>

#include "at_logging.hpp"
#include "at_plugin_mgr.hpp"


using namespace std;
namespace po = boost::program_options;

#define REGISTER_CONSOLE_CMD_LINE_COMMAND(__inCmd__, __cmdDescr__, __cmdHandler__) \
  CommonAstroToolsAppT::registerConsoleCmdLineCommand(__inCmd__, __cmdDescr__, __cmdHandler__);

#define REGISTER_GUI_CMD_LINE_COMMAND(__inCmd__, __cmdDescr__, __cmdHandler__) \
  CommonAstroToolsAppT::registerGuiCmdLineCommand(__inCmd__, __cmdDescr__, __cmdHandler__);

#define DEFINE_OPTION(__varname__, __optname__, __valtype__, __optdescr__) \
  boost::shared_ptr<po::option_description> __varname__ = boost::make_shared<po::option_description>(__optname__, __valtype__, __optdescr__);

/**
 * Note: Plugins may add additional command line arguments.
 * For the GUI just additional options can be added by the plugin.
 * For the console so called commands can be added and each command
 * can have additional options.
 *
 * The plugin itself does not need to know if it is loaded by the
 * console app or the GUI app. Instead the framework must handle 
 * all the plugin register calls accordingly. For example if a plugin
 * issues a command-register call within the gui application it is
 * simply ignored since the GUI app does not have any commands.
 * Vice versa menu hook registrations are ignored by the console app.
 */

namespace AT {
  DEF_Exception(ATCommonApp);
  DEF_Exception(CommandAlreadyExists);

  typedef void(*CmdHandlerFuncT)(void);
  typedef map<string /* cmd */, pair<po::options_description /* options */, CmdHandlerFuncT> > CmdOptionsMapT;

  class CommonAstroToolsAppT {
  private:
    static void registerCmdLineCommand(CmdOptionsMapT * cmdMap, const string & inCmd, const po::options_description & inCmdDescr, CmdHandlerFuncT inCmdHandler) {
      AT_ASSERT(ATCommonApp, cmdMap, "Expecting valid map pointer!");
      LOG(debug) << "Registering command '" << inCmd << "', func-ptr: '"<< hex << (void*) inCmdHandler
		 << "'... Description: " << inCmdDescr << "..." << endl;

      pair<CmdOptionsMapT::iterator,bool> res = cmdMap->insert(make_pair(inCmd, make_pair(inCmdDescr, inCmdHandler)));
      if (! res.second) {
	const string exStr = "Command '" + inCmd + "' already registered.";
	throw CommandAlreadyExistsExceptionT(exStr.c_str());
      }
    }

  protected:
    static po::options_description sGeneralOptionDescr;
    static po::variables_map sCmdLineOptionsMap;
    static CmdOptionsMapT sConsoleCmdMap;
    static CmdOptionsMapT sGuiCmdMap;
    static string sSelectedCmd;


  public:
    static const po::options_description & getGeneralOptions() { return sGeneralOptionDescr; }
    static const po::variables_map & getCmdLineOptionsMap() { return sCmdLineOptionsMap; }
    static CmdOptionsMapT & getConsoleCmdMap() { return sConsoleCmdMap; }
    static CmdOptionsMapT & getGuiCmdMap() { return sGuiCmdMap; }
    static bool isInQuietMode() { return (sCmdLineOptionsMap.count("quiet") > 0); }

    static void registerConsoleCmdLineCommand(const string & inCmd, const po::options_description & inCmdDescr, CmdHandlerFuncT inCmdHandler) {
      registerCmdLineCommand(& sConsoleCmdMap, inCmd, inCmdDescr, inCmdHandler);
    }
    static void registerGuiCmdLineCommand(const string & inCmd, const po::options_description & inCmdDescr, CmdHandlerFuncT inCmdHandler) {
      registerCmdLineCommand(& sGuiCmdMap, inCmd, inCmdDescr, inCmdHandler);
    }
    
  };

  template<typename T>
  class AstroToolsAppT : public CommonAstroToolsAppT {
  private:

  public:
    AstroToolsAppT();
    ~AstroToolsAppT();
  
    /**
     * Put all init stuff in here which is common for console and GUI app.
     */
    static void init(int argc, char **argv) {
      unsigned verbosity = 0U;

      typedef OptionLevelT<'v'> VOptionLevelT;
      VOptionLevelT optionLevel(verbosity);

      // Declare the supported options.
      // See also http://stackoverflow.com/questions/17093579/specifying-levels-e-g-verbose-using-boost-program-options
      sGeneralOptionDescr.add_options()
	("help", "Show help message.")
	("verbose,v", level_value(& optionLevel), "Print more verbose messages at each additional verbosity level.")
	("quiet,q", "Do not print any messages.")
	("plugin_path", po::value<string>(), "Specify directories where to look for plugins.")
	;

      sGeneralOptionDescr.add(T::getExtendedGeneralOptions());

      // Parse the general options which are required right before loading any plugin
      store(po::command_line_parser(argc, argv).options(sGeneralOptionDescr).allow_unregistered().run(), sCmdLineOptionsMap);

      // Setup logging
      logging::trivial::severity_level sev = logging::trivial::warning;

      if (sCmdLineOptionsMap.count("quiet") > 0) {
	sev = logging::trivial::fatal;
      } else {
	if (sCmdLineOptionsMap.count("verbose") > 0) {
	  const VOptionLevelT & ol = sCmdLineOptionsMap["verbose"].as<VOptionLevelT>();
	  unsigned verboseLevel = (ol.n <= 3 ? ol.n : 3);
	  sev = static_cast<logging::trivial::severity_level> (sev - verboseLevel);
	}
	cout << "Set log-level to: " << sev << endl;
      }
      // TODO: Pass "console" and "log file" as params somehow?
      LoggingT::init(sev, false /*console*/, true /*log file*/);


      // Init plugin search paths
      const string & pluginDirsStr = (sCmdLineOptionsMap.count("plugin_path") > 0 ? sCmdLineOptionsMap["plugin_path"].as<string>() : "./");
      PluginMgrT::init(pluginDirsStr);

      /**
       * NOTE: At this point the plugins have been loaded and have
       *       registered themselves (cmds and/or options) to the framework.
       *       Now we can evaluate the rest of the command line arguments.
       *       This behaviour strongly depends on the console / gui implementation.
       *       On success this call updates the sCmdLineOptionsMap map.
       */
      T::evaluateCmdLineArguments(argc, argv, & sCmdLineOptionsMap);
    }

    static void destroy() {
      LOG(trace) << "AstroTools entering destroy()..." << endl;
      PluginMgrT::destroy();
      LOG(trace) << "destroy() finished..." << endl;
    }

  };

}; // end namespace AT

#endif // _ASTRO_TOOLS_APP_HPP_
