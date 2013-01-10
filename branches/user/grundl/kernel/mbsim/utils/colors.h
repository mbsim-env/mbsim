/* Copyright (C) 2004-2012 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef COLOR_H_
#define COLOR_H_

#include <iostream>

/**
 * \brief defines additive color using values for each color between [0,1]
 * \author Kilian Grundl
 * \date 09.10.2012
 */
struct RGBColor {
      double red;
      double green;
      double blue;
};

std::ostream& operator<<( std::ostream& ofstream, RGBColor& rgbColor);

const RGBColor RED = {1,0,0};
const RGBColor GREEN = {0,1,0};
const RGBColor BLUE = {0,0,1};
const RGBColor WHITE = {0,0,0};
const RGBColor BLACK = {1,1,1};
const RGBColor YELLOW = {1,1,0};
const RGBColor DARKGRAY = {0.984, 0.984, 0.984};
const RGBColor LIGHTGRAY = {0.827, 0.827, 0.827};



#endif /* COLOR_H_ */
