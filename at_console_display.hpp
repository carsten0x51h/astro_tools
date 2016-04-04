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

#ifndef _ASTRO_TOOLS_CONSOLE_DISPLAY_HPP_
#define _ASTRO_TOOLS_CONSOLE_DISPLAY_HPP_ _ASTRO_TOOLS_CONSOLE_DISPLAY_HPP_

#include <ncurses.h>

namespace AT {

  class ConsoleDisplayT {
  public:
    ConsoleDisplayT() {
      // Menu
      initscr();
      crmode();
      keypad(stdscr, TRUE);
      noecho();
      clear();
      refresh();
      timeout(10); // makes getch() non-blocking
    }
    ~ConsoleDisplayT() {
      endwin();
    }
    template <typename... Args> void
    print(int x, int y, const char * c, Args&&... args) {
      move(y, x);
      clrtoeol();
      mvprintw(y, x, c, std::forward<Args>(args)...);
    }
  };

};
#endif /*_ASTRO_TOOLS_CONSOLE_DISPLAY_HPP_*/
