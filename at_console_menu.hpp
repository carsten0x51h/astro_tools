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

#ifndef _ASTRO_TOOLS_CONSOLE_MENU_HPP_
#define _ASTRO_TOOLS_CONSOLE_MENU_HPP_ _ASTRO_TOOLS_CONSOLE_MENU_HPP_

#include <string>
#include <limits>
#include <vector>
#include <functional>

#include "at_console_display.hpp"

namespace AT {

  class MenuEntryT {
  protected:
    std::string mTitle;
    //std::string mStatusText;
  
  public:
    MenuEntryT(const std::string & inTitle = "unknown") : mTitle(inTitle) {}
    inline const char* getTitle() const { return mTitle.c_str(); };
    virtual void update(int inKey) { }
    //virtual const char * getHelpText() const = 0;
    //inline const std::string & getStatusText() { return mStatusText; };
    virtual std::string getValueStr() const { return ""; };
  };

  class MenuSeparatorT : public MenuEntryT {
  public:
    MenuSeparatorT() : MenuEntryT("") {}
  };




  struct StepModeT {
    enum TypeE {
      LINEAR,
      FACTOR,
      _Count
    };
  };

  template<typename T>
  class MenuFieldT : public MenuEntryT {
  private:
    T mMin;
    T mMax;
    T mSteps;
    typename StepModeT::TypeE mStepMode;
    T * mValuePtr;
    std::function<std::string(T*)> mValueFmtFunc;
    std::function<void(T*)> mUpdateFunc;
    std::function<void(void)> mAbortFunc;

    T limitValue(const T & inValue) const {
      if (inValue < mMin) { return mMin; }
      else if(inValue > mMax) { return mMax; }
      else { return inValue; }
    }
  
  public:  
    MenuFieldT(T * inValuePtr, const std::string & inTitle, const T & inSteps, typename StepModeT::TypeE inStepMode = StepModeT::LINEAR,
	       const T & inMin = std::numeric_limits<T>::min(), const T & inMax = std::numeric_limits<T>::max(),
	       std::function<std::string(T*)> inValueFmtFunc = 0,
	       std::function<void(T*)> inUpdateFunc = 0,
	       std::function<void(void)> inAbortFunc = 0) :
      MenuEntryT(inTitle),
      mValuePtr(inValuePtr),
      mSteps(inSteps),
      mStepMode(inStepMode),
      mMin(inMin),
      mMax(inMax),
      mValueFmtFunc(inValueFmtFunc),
      mUpdateFunc(inUpdateFunc),
      mAbortFunc(inAbortFunc) { }

    inline void setSteps(const T & inSteps) { mSteps = inSteps; }
    
    virtual void update(int inKey) {
      // TODO: AT_ASSERT mValuePtr != 0... 
      T & valRef = *mValuePtr;

      switch(inKey) {
      case 45: /*-*/
      case KEY_LEFT: /*<-*/
	if (mStepMode == StepModeT::LINEAR) {
	  valRef -= mSteps;
	  valRef = limitValue(valRef);
	  if (mUpdateFunc) {
	    mUpdateFunc(& valRef);
	  }
	} else if (mStepMode == StepModeT::FACTOR) {
	  if (mSteps) {
	    valRef /= mSteps;
	    valRef = limitValue(valRef);
	    if (mUpdateFunc) {
	      mUpdateFunc(& valRef);
	    }
	  }
	} else {
	  // Not implemented -> TODO: AT_ASSERT
	}
	break;
      
      case 43: /*+*/
      case KEY_RIGHT: /*+*/
	if (mStepMode == StepModeT::LINEAR) {
	  valRef += mSteps;
	  valRef = limitValue(valRef);
	  if (mUpdateFunc) {
	    mUpdateFunc(& valRef);
	  }
	} else if (mStepMode == StepModeT::FACTOR) {
	  valRef *= mSteps;
	  valRef = limitValue(valRef);
	  if (mUpdateFunc) {
	    mUpdateFunc(& valRef);
	  }
	} else {
	  // Not implemented -> TODO: AT_ASSERT
	}
	break;
      case 27: /*ESC*/ {
	if (mAbortFunc) {
	  mAbortFunc();
	}
      }
      default: /* nothing */
	break;
      }
    }
  
    virtual std::string getValueStr() const {
      if (mValuePtr) {
	return mValueFmtFunc(mValuePtr);
      } else {
	std::stringstream ss;
	ss << *mValuePtr;
	return ss.str();
      }
    }
  };


  template<typename T>
  class MenuSelectT : public MenuEntryT {
  private:
    typename T::TypeE * mValuePtr;
    std::function<std::string(typename T::TypeE*)> mValueFmtFunc;
    std::function<void(typename T::TypeE*)> mUpdateFunc;
    std::function<void(void)> mAbortFunc;
    size_t mPos;
  
  public:  
    MenuSelectT(typename T::TypeE * inValuePtr, const std::string & inTitle,
		std::function<std::string(typename T::TypeE*)> inValueFmtFunc = 0,
		std::function<void(typename T::TypeE*)> inUpdateFunc = 0,
		std::function<void(void)> inAbortFunc = 0) :
      MenuEntryT(inTitle),
      mValuePtr(inValuePtr),
      mValueFmtFunc(inValueFmtFunc),
      mUpdateFunc(inUpdateFunc),
      mAbortFunc(inAbortFunc) { }

    virtual void update(int inKey) {
      // TODO: AT_ASSERT mValuePtr != 0... 
      typename T::TypeE & valRef = *mValuePtr;
    
      switch(inKey) {
      case KEY_LEFT:
	if (valRef > 0) {
	  if (mUpdateFunc) {
	    mUpdateFunc(& valRef);
	  }
	  valRef = static_cast<typename T::TypeE>(valRef - 1);
	}
	break;
      case KEY_RIGHT:
	if (valRef < T::_Count - 1) {
	  if (mUpdateFunc) {
	    mUpdateFunc(& valRef);
	  }
	  valRef = static_cast<typename T::TypeE>(valRef + 1);
	}
	break;
      case 10: /*ENTER*/
	if (mUpdateFunc) {
	  mUpdateFunc(& valRef);
	}
	valRef = static_cast<typename T::TypeE>((valRef + 1) % T::_Count);
	break;
      case 27: /*ESC*/
	if (mAbortFunc) {
	  mAbortFunc();
	}
	break;
      default: /* nothing */
	break;
      }
    }
  
    virtual std::string getValueStr() const {
      if (mValuePtr) {
	return mValueFmtFunc(mValuePtr);
      } else {
	return T::asStr(*mValuePtr);
      }
    }
  };





  class ConsoleMenuT {
  private:
    ConsoleDisplayT * mConsoleDisplay;
    const std::vector<MenuEntryT *> & mMenuEntries;
    int mPosition;
    bool mWantExit;
    
  public:
    ConsoleMenuT(ConsoleDisplayT * inConsoleDisplay, const std::vector<MenuEntryT *> & inMenuEntries) : mConsoleDisplay(inConsoleDisplay), mMenuEntries(inMenuEntries), mPosition(0), mWantExit(false) {}

    inline bool wantExit() const { return mWantExit; }
    
    void update() {
      // Menu handling
      int key = getch();

      switch(key) {
      case KEY_UP:
	while (mPosition > 0) {
	  mPosition--;
	  bool isSeparator =  ! strcmp(mMenuEntries.at(mPosition)->getTitle(), "");
	  if (! isSeparator) { break; }
	}
	break;
      
      case KEY_DOWN:
	while (mPosition < mMenuEntries.size() - 1) {
	  mPosition++;
	  bool isSeparator =  ! strcmp(mMenuEntries.at(mPosition)->getTitle(), "");
	  if (! isSeparator) { break; }
	}
	break;
    
      case 9: /* TAB */
	bool isSeparator;
	do {
	  mPosition++;
	  mPosition = mPosition % mMenuEntries.size();
	
	  isSeparator =  ! strcmp(mMenuEntries.at(mPosition)->getTitle(), "");
	} while (isSeparator);
	break;

      case 'q':
	mWantExit = true;
	break;
	
      default:
	mMenuEntries.at(mPosition)->update(key);

	break;
      } // end switch

      // Update menu display, highlight selected menu entry
      for (std::vector<MenuEntryT *>::const_iterator it = mMenuEntries.begin(); it != mMenuEntries.end(); ++it) {
	const MenuEntryT * entry = *it;
	size_t i = std::distance(mMenuEntries.begin(), it);
	if (i == mPosition) { attron(A_STANDOUT); }
	mConsoleDisplay->print(cLeftMenuBorder, cTopMenuBorder + i, "%s", entry->getTitle());
	if (i == mPosition) { attroff(A_STANDOUT); }
	// Print corresponding value...
	mConsoleDisplay->print(cLeftMenuBorder + cLeftMenuWidth, cTopMenuBorder + i, "%s", entry->getValueStr().c_str());
      }
    }

    static const int cTopMenuBorder = 1;
    static const int cLeftMenuBorder = 10;
    static const int cLeftMenuWidth = 20;
  };

};

#endif /*_ASTRO_TOOLS_CONSOLE_MENU_HPP_*/
