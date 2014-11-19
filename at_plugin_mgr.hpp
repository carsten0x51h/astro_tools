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

#ifndef _AT_PLUGIN_MGR_HPP_
#define _AT_PLUGIN_MGR_HPP_ _AT_PLUGIN_MGR_HPP_

#include <iosfwd>
#include <map>
#include <set>
#include <dlfcn.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#include "at_logging.hpp"
#include "at_exception.hpp"
#include "at_plugin.hpp"

using namespace std;

namespace AT {

  DEF_Exception(PluginMgr);
  DEF_Exception(PluginMgrCannotOpenLib);
  DEF_Exception(PluginMgrCannotLoadSymbol);

  typedef void* (*PluginPtr)();

  class PluginMgrT {
  private:
    static map<string, PluginT*> sPlugins;
    static vector<string> sPluginDirs;
    static set<string> sAvailablePlugins;
    static set<void*> sLibHandles;

  public:
    static inline vector<string> & getPluginDirs() { return sPluginDirs; }

    static void init(const string & inPluginSearchPaths) {
      AT_ASSERT(PluginMgr, sPluginDirs.empty(), "Expect plugin dir vector to be empty!");

      vector<string> tokens;
      boost::split(tokens, inPluginSearchPaths, boost::is_any_of(",:"));
      sPluginDirs.insert(sPluginDirs.end(), tokens.begin(), tokens.end());

      LOG(info) << sPluginDirs.size() << " Plugin dirs..." << endl;
      for (vector<string>::const_iterator it = sPluginDirs.begin(); it != sPluginDirs.end(); ++it) {
	LOG(info) << "Scanning plugin dir '" << *it << "'" << endl;
	scanForPlugins(& sAvailablePlugins, *it);
      }

      LOG(info) << "Found " << sAvailablePlugins.size() << " plugin(s): " << endl;
      for (set<string>::const_iterator it = sAvailablePlugins.begin(); it != sAvailablePlugins.end(); ++it) {
	LOG(info) << *it << endl;
      }
    }

    static void destroy() {
      // Closing library handles...
      for (set<void*>::iterator it = sLibHandles.begin(); it != sLibHandles.end(); ++it) {
	LOG(debug) << "Closing library handle..." << hex << (void*) *it << endl;
	dlclose(*it);
      }
    }

    static void scanForPlugins(set<string> * outPluginPaths, const string & inRootDir = "./") {
      if (! outPluginPaths)
	return;

      for (boost::filesystem::recursive_directory_iterator endIt, dirIt(inRootDir); dirIt != endIt; ++dirIt) {
	const char * filename = dirIt->path().filename().c_str();

	if (! is_directory(dirIt->path()) && boost::starts_with(filename, "libat") && boost::ends_with(filename, ".so")) {
	  // Check for "instance" symbol
	  try {
	    LOG(debug) << "Trying to open " << dirIt->path() << endl;
	    openPluginHandle(dirIt->path().string()); // Try to get the plugin handle
	    outPluginPaths->insert(dirIt->path().string());
	  } catch(std::exception & exc) {
	    LOG(warning) << "Unable to open " << dirIt->path().string() << " - reason: " << exc.what() << endl;
	  }
	}
      }
    }


    static PluginPtr openPluginHandle(const string & inPluginPath) {
      LOG(debug) << "Opening plugin lib '" << inPluginPath.c_str() << "'..." << endl;
      void * handle = dlopen(inPluginPath.c_str(), RTLD_LAZY | RTLD_GLOBAL);
      
      if (! handle) {
	const string exStr = "Cannot open library: " + string(dlerror());
	throw PluginMgrCannotOpenLibExceptionT(exStr.c_str());
      }
      
      // reset errors
      dlerror();

      PluginPtr pluginPtr = (PluginPtr) dlsym(handle, "instance");
      const char * dlsym_error = dlerror();
      
      if (dlsym_error) {
	const string exStr = "Cannot load symbol '" + string(dlsym_error) + "'";
	dlclose(handle);
	throw PluginMgrCannotLoadSymbolExceptionT(exStr.c_str());
      }

      sLibHandles.insert(handle); // Remember handle, close at destruction
      return pluginPtr;
    }




    static void loadPlugin(const string & inPluginPath) {
      PluginPtr pluginPtr = openPluginHandle(inPluginPath);
    }

    static void loadPlugin(PluginPtr inPluginPtr) {
      PluginT * plugin = (PluginT*) inPluginPtr();
      AT_ASSERT(PluginMgr, plugin, "Expect plugin to be a valid pointer!");

      LOG(info) << "Loaded plugin '" << plugin->getName() << "'..." << endl;
      sPlugins.insert(make_pair(plugin->getName(), plugin));
    }
    
  };
}; // end namespace AT

#endif // _AT_PLUGIN_MGR_HPP_
