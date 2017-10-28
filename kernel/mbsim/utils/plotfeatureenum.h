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

#ifndef PLOTFEATUREENUM_H_
#define PLOTFEATUREENUM_H_

#include <unordered_map>
#include <functional>
#include <cassert>

namespace MBSim {

class PlotFeatureEnum {
  public:
    bool operator==(const PlotFeatureEnum& a) const { return this==&a; }
    bool operator!=(const PlotFeatureEnum& a) const { return this!=&a; }

    PlotFeatureEnum() {}
    ~PlotFeatureEnum() {}
    // must be defined due to SWIG
    // static void* operator new(std::size_t);
    // static void operator delete(void*, std::size_t);
  private:
    static void operator delete[](void*, std::size_t);
    static void* operator new[](std::size_t);
    PlotFeatureEnum(const PlotFeatureEnum&)=delete;
    PlotFeatureEnum(PlotFeatureEnum&&)=delete;
    PlotFeatureEnum& operator=(const PlotFeatureEnum&)=delete;
    PlotFeatureEnum& operator=(PlotFeatureEnum&&)=delete;
};

#ifndef SWIG

struct PlotFeatureEnumOp {
  size_t operator()(const PlotFeatureEnum &f) const { return reinterpret_cast<size_t>(&f); }
  bool operator()(const PlotFeatureEnum &a, const PlotFeatureEnum &b) const { return &a==&b; }
};

typedef std::unordered_map<std::reference_wrapper<const PlotFeatureEnum>, bool, PlotFeatureEnumOp, PlotFeatureEnumOp> PlotFeatureMap;

#endif

}

#endif
